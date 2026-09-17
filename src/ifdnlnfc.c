/*
 *
 *  libnlnfc - PC/SC IFD Handler for Linux NFC subsystem
 *
 *  Copyright (C) 2024 Juraj Šarinay <juraj@sarinay.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 *  uses code from neard & nfctool by Intel
 *  https://github.com/linux-nfc/neard
 *
 *  ATR derivation code adapted from ifdnfc by Frank Morgner
 *  https://github.com/nfc-tools/ifdnfc
 *
 */

#include "config.h"

#include "ifdnlnfc.h"
#include <debuglog.h>
#include <errno.h>
#include <ifdhandler.h>
#include <linux/nfc.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/genl.h>
#include <netlink/handlers.h>
#include <netlink/netlink.h>
#include <poll.h>
#include <pthread.h>
#include <reader.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <unistd.h>

/* Platform/vendor initialization (e.g. the NXP NPC300 proprietary
 * CORE_SET_CONFIG payloads) is out of scope for this driver. It is applied
 * once, out of band, by the npc300-init tool at boot and on resume; see
 * README. This driver only ever reuses whatever power/RF state it finds the
 * adapter in. */

/* While the PC/SC client has logically powered the card down, re-probe its
 * presence at least this often so removal is still noticed promptly. */
#define PRESENCE_PROBE_INTERVAL_MS 1000

static struct nl_sock *cmd_sock, *event_sock;
static int nfc_family_id;
static int polling_wake_fd = -1;
static struct ifdnlnfc_state ifdnlnfc_state = {
	.socket = -1,
};

/* Protects ifdnlnfc_state and the netlink globals above from races between
 * pcscd's dedicated polling thread (IFDHPolling/IFDHICCPresence) and the
 * per-client worker threads calling the other IFDH* entry points; pcscd
 * does not itself serialize these against each other. Held across each
 * entry point's body, but never across the blocking poll() inside
 * IFDHPolling -- only around the state it reads/writes before and after
 * waiting. */
static pthread_mutex_t state_lock = PTHREAD_MUTEX_INITIALIZER;

static int nl_error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err,
			void *arg)
{
	int *ret = arg;

	(void)nla;
	*ret = err->error;
	return NL_STOP;
}

static int nl_finish_handler(struct nl_msg *msg, void *arg)
{
	int *ret = arg;

	(void)msg;
	*ret = 1;
	return NL_SKIP;
}

static int nl_ack_handler(struct nl_msg *msg, void *arg)
{
	int *ret = arg;

	(void)msg;
	*ret = 1;
	return NL_SKIP;
}

static int nl_send_msg(struct nl_sock *sock, struct nl_msg *msg,
		int (*rx_handler)(struct nl_msg *, void *),
		void *data)
{
	struct nl_cb *cb;
	int err, done;

	cb = nl_cb_alloc(NL_CB_DEFAULT);
	if (!cb)
		return -ENOMEM;

	err = nl_send_auto_complete(sock, msg);
	if (err < 0) {
		nl_cb_put(cb);
		return err;
	}

	err = done = 0;

	nl_cb_err(cb, NL_CB_CUSTOM, nl_error_handler, &err);
	nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, nl_finish_handler, &done);
	nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, nl_ack_handler, &done);

	if (rx_handler)
		nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, rx_handler, data);

	while (err == 0 && done == 0) {
		int recv_err = nl_recvmsgs(sock, cb);

		/*
		 * nl_error_handler() already wrote the real kernel-side errno
		 * from the NLMSG_ERROR ack straight into err (it fires
		 * synchronously inside nl_recvmsgs() above). nl_recvmsgs()'s
		 * own return value in that case is a *separate*,
		 * libnl-internal re-encoding of that same event (its NLE_*
		 * error space, e.g. NLE_INVAL=7 for EINVAL=22) and must not
		 * be allowed to clobber the real value. Only treat a negative
		 * return as a hard failure when neither callback fired at
		 * all, i.e. a genuine transport-level receive failure.
		 */
		if (recv_err < 0 && err == 0 && done == 0)
			err = recv_err;
	}

	nl_cb_put(cb);

	return err;
}

static int nl_set_powered(struct nfc_adapter * adapter, int powered)
{
	struct nl_msg *msg;
	void *hdr;
	int err;
	uint8_t cmd;

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	cmd = powered ? NFC_CMD_DEV_UP : NFC_CMD_DEV_DOWN;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_REQUEST, cmd, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	err = -EMSGSIZE;

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, adapter->idx);

	err = nl_send_msg(cmd_sock, msg, NULL, NULL);

	if (err)
		Log3(PCSC_LOG_ERROR, "Error powering %s NFC adapter. Idx: %d", powered ? "up" : "down", adapter->idx);
	else
		Log3(PCSC_LOG_INFO, "Powering %s NFC adapter. Idx: %d", powered ? "up" : "down", adapter->idx);

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static int set_atr_from_hb(struct nfc_target *target, const unsigned char *hb, int hb_len)
{
	int len = 4 + hb_len;

	if (len + 1 > MAX_ATR_SIZE) {
		/* should never happen */
		Log1(PCSC_LOG_ERROR, "Too many historical bytes.");
		target->atr_len = 0;
		return -1;
	}

	target->atr[0] = 0x3b;
	target->atr[1] = 0x80 + hb_len;
	target->atr[2] = 0x80;
	target->atr[3] = 0x01;

	if (hb_len > 0)
		memcpy(&target->atr[4], hb, hb_len);

	unsigned char tck = target->atr[1];
	for (int i = 2; i < len; i++)
		tck ^= target->atr[i];

	target->atr[len] = tck;
	target->atr_len = len + 1;

	return 0;
}

/*
 * Split the historical bytes out of an ISO 14443-4 ATS as delivered by the
 * kernel's NFC_ATTR_TARGET_ATS attribute.
 *
 * That attribute does NOT include the leading TL (total length) byte of the
 * raw ISO 14443-4 ATS: net/nfc/nci/ntf.c reads and consumes the NCI "RATS
 * Response Length" byte itself and copies only the bytes from T0 onward into
 * target_ats, so ats[0] here is T0, not TL, and ats_len counts T0 onward.
 *
 * On success, *historical_bytes is set to a pointer into ats (or NULL if
 * there are no historical bytes) and *historical_bytes_len to their count.
 * Returns 0 on success, -1 if ats is malformed.
 */
static int ats_historical_bytes(const uint8_t *ats, size_t ats_len,
		const uint8_t **historical_bytes, size_t *historical_bytes_len)
{
	size_t offset = 1;
	uint8_t t0;

	*historical_bytes = NULL;
	*historical_bytes_len = 0;

	if (!ats || ats_len < 1)
		return -1;

	t0 = ats[0];
	if (t0 & 0x10) offset++; /* TA(1) present */
	if (t0 & 0x20) offset++; /* TB(1) present */
	if (t0 & 0x40) offset++; /* TC(1) present */

	if (offset > ats_len)
		return -1;

	*historical_bytes_len = ats_len - offset;
	if (*historical_bytes_len)
		*historical_bytes = ats + offset;

	return 0;
}

static int get_targets_handler(struct nl_msg *msg, void *arg)
{
	struct nlmsghdr *nlh = nlmsg_hdr(msg);
	struct nlattr *attrs[NFC_ATTR_MAX + 1];

	struct list_targets_cb_state *state = arg;

	unsigned char hb[8];

	genlmsg_parse(nlh, 0, attrs, NFC_ATTR_MAX, NULL);

	if (!attrs[NFC_ATTR_TARGET_INDEX] || !attrs[NFC_ATTR_PROTOCOLS]) {
		return NL_SKIP;
	}

	if (state->found == 1) {
		Log1(PCSC_LOG_INFO, "Multiple NFC targets found. All but the first one will be ignored.");
	}

	if (state->found++) {
		return NL_SKIP; // at the moment limit to a single target
	}

	state->target->idx = nla_get_u32(attrs[NFC_ATTR_TARGET_INDEX]);
	state->target->supported_protocols = nla_get_u32(attrs[NFC_ATTR_PROTOCOLS]);

	Log3(PCSC_LOG_INFO, "NFC target found. Index: %d, supported protocols: %0x.", state->target->idx, state->target->supported_protocols);

	state->target->uid_len = 0;
	if (attrs[NFC_ATTR_TARGET_NFCID1]) {
		int len = nla_len(attrs[NFC_ATTR_TARGET_NFCID1]);

		if (len > (int)sizeof(state->target->uid))
			len = (int)sizeof(state->target->uid);
		memcpy(state->target->uid, nla_data(attrs[NFC_ATTR_TARGET_NFCID1]), (size_t)len);
		state->target->uid_len = len;
	}

	if (state->target->supported_protocols & NFC_PROTO_ISO14443_B_MASK) {

		if (attrs[NFC_ATTR_TARGET_SENSB_RES])
			LogXxd(PCSC_LOG_DEBUG, "ATQB: ", nla_data(attrs[NFC_ATTR_TARGET_SENSB_RES]), nla_len(attrs[NFC_ATTR_TARGET_SENSB_RES]));

		if (attrs[NFC_ATTR_TARGET_SENSB_RES] && nla_len(attrs[NFC_ATTR_TARGET_SENSB_RES]) == 11) {
			memcpy(hb, nla_data(attrs[NFC_ATTR_TARGET_SENSB_RES]) + 4, 7);
			hb[7] = 0;
			set_atr_from_hb(state->target, hb, 8);
		}
		else
			set_atr_from_hb(state->target, NULL, 0);
	}

	return NL_OK;
}

static int get_target_ats_handler(struct nl_msg *msg, void *arg)
{
	struct nlmsghdr *nlh = nlmsg_hdr(msg);
	struct nlattr *attrs[NFC_ATTR_MAX + 1];
	const unsigned char *hb = NULL;
	size_t hb_len = 0;

	int ats_len = 0;
	const unsigned char *ats = NULL;

	struct list_targets_cb_state *state = arg;

	genlmsg_parse(nlh, 0, attrs, NFC_ATTR_MAX, NULL);

	if (state->found || !attrs[NFC_ATTR_TARGET_INDEX] || state->target->idx != nla_get_u32(attrs[NFC_ATTR_TARGET_INDEX]))
		return NL_SKIP;

	state->found = 1;

	if (attrs[NFC_ATTR_TARGET_ATS]) {
		ats_len = nla_len(attrs[NFC_ATTR_TARGET_ATS]);
		ats = nla_data(attrs[NFC_ATTR_TARGET_ATS]);
		LogXxd(PCSC_LOG_DEBUG, "Got ATS: ", ats, ats_len);
	}
	else {
		Log1(PCSC_LOG_DEBUG, "ATS not present");
	}

	if (ats && ats_historical_bytes(ats, (size_t)ats_len, &hb, &hb_len))
		Log1(PCSC_LOG_ERROR, "ATS invalid");

	set_atr_from_hb(state->target, hb, (int)hb_len);

	return NL_OK;
}

static int list_targets(struct nfc_adapter * adapter, struct nfc_target *result)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -1;
	struct nfc_target target = {0};
	struct list_targets_cb_state state = {0, &target};

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_DUMP, NFC_CMD_GET_TARGET, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, adapter->idx);

	err = nl_send_msg(cmd_sock, msg, get_targets_handler, &state);
	if (!err && state.found) {
		*result = target;
		nlmsg_free(msg);
		return 0;
	}
	if (!err)
		err = -ENOENT;

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static int get_target_ats(struct nfc_adapter * adapter, struct nfc_target * target)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -ENOENT;

	struct list_targets_cb_state state = {0, target};

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_DUMP, NFC_CMD_GET_TARGET, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, adapter->idx);

	err = nl_send_msg(cmd_sock, msg, get_target_ats_handler, &state);

	if (!err && state.found) {
		nlmsg_free(msg);
		return 0;
	}
	if (!err)
		err = -ENOENT;

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static void wake_polling_thread(void)
{
	uint64_t value = 1;
	ssize_t written;

	if (polling_wake_fd < 0)
		return;

	do {
		written = write(polling_wake_fd, &value, sizeof(value));
	} while (written < 0 && errno == EINTR);
}

static void close_target_socket(void)
{
	if (ifdnlnfc_state.socket >= 0)
		close(ifdnlnfc_state.socket);

	ifdnlnfc_state.socket = -1;
	atomic_store_explicit(&ifdnlnfc_state.card_powered, 0, memory_order_relaxed);
}

static void remove_target(void)
{
	close_target_socket();
	memset(&ifdnlnfc_state.target, 0, sizeof(ifdnlnfc_state.target));
	ifdnlnfc_state.target_valid = 0;
	ifdnlnfc_state.card_present = 0;
	wake_polling_thread();
}

static void reset_driver_state(void)
{
	memset(&ifdnlnfc_state.adapter, 0, sizeof(ifdnlnfc_state.adapter));
	memset(&ifdnlnfc_state.target, 0, sizeof(ifdnlnfc_state.target));
	ifdnlnfc_state.channel_open = 0;
	ifdnlnfc_state.target_valid = 0;
	ifdnlnfc_state.card_present = 0;
	ifdnlnfc_state.adapter_removed = 0;
	atomic_store_explicit(&ifdnlnfc_state.card_powered, 0, memory_order_relaxed);
	ifdnlnfc_state.socket = -1;
}

static int event_handler(struct nl_msg *msg, void *arg)
{
	struct nlattr *attr[NFC_ATTR_MAX + 1];
	struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
	uint32_t cmd = gnlh->cmd;
	uint32_t device_index;

	(void)arg;

	if (!ifdnlnfc_state.channel_open)
		return NL_SKIP;

	if (cmd != NFC_EVENT_TARGETS_FOUND && cmd != NFC_EVENT_TARGET_LOST &&
		cmd != NFC_EVENT_DEVICE_REMOVED)
		return NL_SKIP;

	nla_parse(attr, NFC_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);

	if (!attr[NFC_ATTR_DEVICE_INDEX])
		return NL_SKIP;

	device_index = nla_get_u32(attr[NFC_ATTR_DEVICE_INDEX]);

	if (device_index != ifdnlnfc_state.adapter.idx)
		return NL_SKIP;

	if (cmd == NFC_EVENT_DEVICE_REMOVED) {
		Log2(PCSC_LOG_DEBUG, "NFC adapter removed. Adapter index:%d.", device_index);
		ifdnlnfc_state.adapter_removed = 1;
		remove_target();
		return NL_OK;
	}

	if (cmd == NFC_EVENT_TARGETS_FOUND) {
		/* Don't disturb an established connection with a spurious relist. */
		if (ifdnlnfc_state.socket >= 0)
			return NL_SKIP;
		ifdnlnfc_state.card_present = 1;
		ifdnlnfc_state.target_valid = 0;
		ifdnlnfc_state.adapter.poll_active = 0;
		Log2(PCSC_LOG_DEBUG, "NFC_TARGETS_FOUND. Adapter index:%d.", device_index);
		return NL_OK;
	}

	/* NFC_EVENT_TARGET_LOST: kernel drivers that implement check_presence
	 * may emit this; harmless to act on if it ever arrives, but callers
	 * must not rely on it alone since most drivers never send it. */
	Log2(PCSC_LOG_DEBUG, "NFC target lost. Adapter index:%d.", device_index);
	remove_target();

	return NL_OK;
}

static int family_handler(struct nl_msg *msg, void *arg)
{
	int *group_id = arg;
	struct nlattr *tb[CTRL_ATTR_MAX + 1];
	struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
	struct nlattr *mcgrp;
	int rem_mcgrp;

	nla_parse(tb, CTRL_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
		genlmsg_attrlen(gnlh, 0), NULL);

	if (!tb[CTRL_ATTR_MCAST_GROUPS])
		return NL_SKIP;

	nla_for_each_nested(mcgrp, tb[CTRL_ATTR_MCAST_GROUPS], rem_mcgrp) {
		struct nlattr *tb_mcgrp[CTRL_ATTR_MCAST_GRP_MAX + 1];

		nla_parse(tb_mcgrp, CTRL_ATTR_MCAST_GRP_MAX,
			nla_data(mcgrp), nla_len(mcgrp), NULL);

		if (!tb_mcgrp[CTRL_ATTR_MCAST_GRP_NAME] ||
			!tb_mcgrp[CTRL_ATTR_MCAST_GRP_ID])
			continue;
		if (strncmp(nla_data(tb_mcgrp[CTRL_ATTR_MCAST_GRP_NAME]),
				NFC_GENL_MCAST_EVENT_NAME,
				nla_len(tb_mcgrp[CTRL_ATTR_MCAST_GRP_NAME])))
			continue;
		*group_id = nla_get_u32(tb_mcgrp[CTRL_ATTR_MCAST_GRP_ID]);
		return NL_OK;
	}

	return NL_SKIP;
}

static int get_multicast_id(struct nl_sock *sock, int *group_id)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -EINVAL;
	int ctrlid;

	*group_id = -1;

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	ctrlid = genl_ctrl_resolve(sock, "nlctrl");
	if (ctrlid < 0) {
		err = ctrlid;
		goto nla_put_failure;
	}

	hdr = genlmsg_put(msg, 0, 0, ctrlid, 0,
		0, CTRL_CMD_GETFAMILY, 0);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	NLA_PUT_STRING(msg, CTRL_ATTR_FAMILY_NAME, NFC_GENL_NAME);

	err = nl_send_msg(sock, msg, family_handler, group_id);
	if (!err && *group_id < 0)
		err = -ENOENT;

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static int get_device_handler(struct nl_msg *n, void *arg)
{
	struct nlmsghdr *nlh = nlmsg_hdr(n);
	struct nlattr *attrs[NFC_ATTR_MAX + 1];
	uint32_t protocols = 0;
	uint8_t powered = 0, rf_mode = NFC_RF_NONE;

	struct get_adapter_cb_state *state = arg;

	genlmsg_parse(nlh, 0, attrs, NFC_ATTR_MAX, NULL);

	if (attrs[NFC_ATTR_DEVICE_POWERED]) {
		powered = nla_get_u8(attrs[NFC_ATTR_DEVICE_POWERED]);
	}

	if (attrs[NFC_ATTR_RF_MODE]) {
		rf_mode =  nla_get_u8(attrs[NFC_ATTR_RF_MODE]);
	}

	if (!attrs[NFC_ATTR_PROTOCOLS])
		return NL_SKIP;

	protocols = nla_get_u32(attrs[NFC_ATTR_PROTOCOLS]);
	if (protocols & (NFC_PROTO_ISO14443_MASK | NFC_PROTO_ISO14443_B_MASK)) {
		state->found = 1;
		state->adapter->idx = state->idx;
		state->adapter->initial_mode = rf_mode;
		state->adapter->initial_power = powered;
		state->adapter->protocols = protocols;
		Log4(PCSC_LOG_INFO, "NFC adapter found. Index: %d, powered: %d, supported protocols: %0x.", state->adapter->idx, powered, protocols);
		Log2(PCSC_LOG_DEBUG, "Adapter Mode: %d", rf_mode);
		return NL_OK;
	}
	return NL_SKIP;
}

static int list_devices_handler(struct nl_msg *n, void *arg)
{
	struct nlmsghdr *nlh = nlmsg_hdr(n);
	struct nlattr *attrs[NFC_ATTR_MAX + 1];
	uint32_t protocols = 0;
	uint8_t powered = 0, rf_mode = NFC_RF_NONE;

	struct list_adapters_cb_state * state = arg;

	if (state->found)
		return NL_SKIP;

	genlmsg_parse(nlh, 0, attrs, NFC_ATTR_MAX, NULL);

	if (!attrs[NFC_ATTR_DEVICE_NAME] || nla_strcmp(attrs[NFC_ATTR_DEVICE_NAME], state->name)) return NL_SKIP;

	if (!attrs[NFC_ATTR_DEVICE_INDEX] || !attrs[NFC_ATTR_PROTOCOLS])
		return NL_SKIP;

	if (attrs[NFC_ATTR_DEVICE_POWERED]) {
		powered =  nla_get_u8(attrs[NFC_ATTR_DEVICE_POWERED]);
	}

	if (attrs[NFC_ATTR_RF_MODE]) {
		rf_mode =  nla_get_u8(attrs[NFC_ATTR_RF_MODE]);
	}

	protocols = nla_get_u32(attrs[NFC_ATTR_PROTOCOLS]);
	if (protocols & (NFC_PROTO_ISO14443_MASK | NFC_PROTO_ISO14443_B_MASK)) {
		state->found = 1;
		state->adapter->idx = nla_get_u32(attrs[NFC_ATTR_DEVICE_INDEX]);
		state->adapter->initial_mode = rf_mode;
		state->adapter->initial_power = powered;
		state->adapter->protocols = protocols;
		Log5(PCSC_LOG_INFO, "NFC adapter found. Name: %s, Index: %d, powered: %d, supported protocols: %0x.", state->name, state->adapter->idx, powered, protocols);
		Log2(PCSC_LOG_DEBUG, "Adapter Mode: %d", rf_mode);
	}
	return NL_SKIP;
}

static int poll_for_targets(struct nfc_adapter * adapter)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -EINVAL;
	uint32_t protocols = adapter->protocols &
		(NFC_PROTO_ISO14443_MASK | NFC_PROTO_ISO14443_B_MASK);

	if (adapter->poll_active) {
		Log2(PCSC_LOG_ERROR, "Poll active, not starting. Adapter index: %d.", adapter->idx);
		return 0;
	}
	if (!protocols)
		return -EOPNOTSUPP;

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_REQUEST, NFC_CMD_START_POLL, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, adapter->idx);
	NLA_PUT_U32(msg, NFC_ATTR_IM_PROTOCOLS, protocols);

	err = nl_send_msg(cmd_sock, msg, NULL, NULL);

	if (err)
		Log3(PCSC_LOG_ERROR, "Error %x starting NFC target poll. Adapter index: %d.", err, adapter->idx);
	else {
		Log2(PCSC_LOG_DEBUG, "NFC target poll started. Adapter index:%d.", adapter->idx);
		adapter->poll_active = 1;
	}

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static int stop_poll_for_targets_ex(struct nfc_adapter * adapter, int force)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -EINVAL;

	if (!force && !adapter->poll_active) {
		Log2(PCSC_LOG_INFO, "Poll not active, nothing to stop. Adapter index: %d.", adapter->idx);
		return 0;
	}

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_REQUEST, NFC_CMD_STOP_POLL, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, adapter->idx);

	err = nl_send_msg(cmd_sock, msg, NULL, NULL);

	/* -EINVAL means the kernel considers polling already inactive
	 * (dev->polling was already clear); treat that as success. */
	if (err && err != -EINVAL)
		Log3(PCSC_LOG_ERROR, "Error %x stopping NFC target poll. Adapter index: %d.", err, adapter->idx);
	else {
		err = 0;
		adapter->poll_active = 0;
		Log2(PCSC_LOG_DEBUG, "NFC target poll stopped. Adapter index: %d.", adapter->idx);
	}
nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static int stop_poll_for_targets(struct nfc_adapter * adapter)
{
	return stop_poll_for_targets_ex(adapter, 0);
}

static int get_adapter_by_idx(uint32_t idx, struct nfc_adapter *adapter)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -EINVAL;

	struct get_adapter_cb_state state = {idx, 0, adapter};

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_REQUEST, NFC_CMD_GET_DEVICE, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, idx);

	err = nl_send_msg(cmd_sock, msg, get_device_handler, &state);

	if (state.found) {
		err = 0;
	}
	else if (!err) {
		err = -ENODEV;
		Log2(PCSC_LOG_INFO, "NFC adapter not found. Index: %d", idx);
	}

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static int get_adapter_by_name(const char * adapter_name, struct nfc_adapter * adapter)
{
	struct nl_msg *msg;
	void *hdr;
	int err = 0;

	struct list_adapters_cb_state state = {adapter_name, 0, adapter};

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_DUMP, NFC_CMD_GET_DEVICE, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	err = nl_send_msg(cmd_sock, msg, list_devices_handler, &state);

	if (err || !state.found) {
		err = -ENODEV;
		Log2(PCSC_LOG_INFO, "Adapter %s not found.", adapter_name);
	}

nla_put_failure:
	nlmsg_free(msg);
	return err;
}

static void netlink_cleanup(void)
{
	if (cmd_sock)
		nl_socket_free(cmd_sock);
	if (event_sock)
		nl_socket_free(event_sock);
	if (polling_wake_fd >= 0)
		close(polling_wake_fd);

	cmd_sock = NULL;
	event_sock = NULL;
	polling_wake_fd = -1;
	nfc_family_id = -1;
}

static int netlink_setup(void)
{
	struct nl_cb *cb;
	int err;
	int group_id = -1;

	cmd_sock = nl_socket_alloc();

	if (!cmd_sock) {
		Log1(PCSC_LOG_ERROR, "Out of memory");
		return -ENOMEM;
	}

	event_sock = nl_socket_alloc();

	if (!event_sock) {
		Log1(PCSC_LOG_ERROR, "Out of memory");
		err = -ENOMEM;
		goto failure;
	}

	err = genl_connect(cmd_sock);
	if (err)
		goto failure;

	err = genl_connect(event_sock);
	if (err)
		goto failure;

	err = nl_socket_set_nonblocking(event_sock);
	if (err)
		goto failure;

	nfc_family_id = genl_ctrl_resolve(cmd_sock, "nfc");
	if (nfc_family_id < 0) {
		Log1(PCSC_LOG_DEBUG, "Unable to find NFC netlink family");
		err = nfc_family_id;
		goto failure;
	}

	err = get_multicast_id(cmd_sock, &group_id);

	if (err) {
		Log1(PCSC_LOG_DEBUG, "Unable to find multicast group ID");
		goto failure;
	}

	cb = nl_cb_alloc(NL_CB_VERBOSE);

	if (!cb) {
		Log1(PCSC_LOG_ERROR, "Out of memory");
		err = -ENOMEM;
		goto failure;
	}

	nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, event_handler, NULL);
	nl_socket_set_cb(event_sock, cb);
	nl_cb_put(cb);
	nl_socket_disable_seq_check(event_sock);

	err = nl_socket_add_membership(event_sock, group_id);

	if (err) {
		Log1(PCSC_LOG_DEBUG, "Error adding nl socket to notification group");
		goto failure;
	}

	polling_wake_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
	if (polling_wake_fd < 0) {
		err = -errno;
		goto failure;
	}

	return 0;

failure:
	netlink_cleanup();
	return err;
}

static int connect_target(struct nfc_adapter *adapter, struct nfc_target *target)
{
	int err;
	int fd;
	uint32_t protocol = 0;
	struct sockaddr_nfc sa;

	if (!ifdnlnfc_state.target_valid)
		return -ENOENT;
	if (ifdnlnfc_state.socket >= 0)
		return -EISCONN;

	if (target->supported_protocols & NFC_PROTO_ISO14443_MASK)
		protocol = NFC_PROTO_ISO14443;
	else if (target->supported_protocols & NFC_PROTO_ISO14443_B_MASK)
		protocol = NFC_PROTO_ISO14443_B;
	else {
		Log1(PCSC_LOG_DEBUG, "connect_target(): No suitable NFC protocol found.");
		return -1;
	}

	sa = (struct sockaddr_nfc){PF_NFC, adapter->idx, target->idx, protocol};

	fd = socket(AF_NFC, SOCK_SEQPACKET, NFC_SOCKPROTO_RAW);
	if (fd == -1)
		return -errno;

	if (!connect(fd, (struct sockaddr *) &sa, sizeof(sa))) {
		ifdnlnfc_state.socket = fd;
		target->active_protocol = protocol;
		Log3(PCSC_LOG_DEBUG, "Connected to NFC target. Index: %d, Protocol: %0x.", target->idx, protocol);
		if (protocol == NFC_PROTO_ISO14443) {
			set_atr_from_hb(target, NULL, 0);
			get_target_ats(adapter, target);
		}
		return 0;
	}

	err = -errno;
	close(fd);
	return err;
}

static int initialize_adapter(struct nfc_adapter *adapter)
{
	int err;

	if (adapter->initial_mode == NFC_RF_TARGET) {
		Log1(PCSC_LOG_ERROR, "Adapter busy");
		return -1;
	}

	if (adapter->initial_power) {
		/* The adapter is already up, presumably brought up and
		 * proprietary-initialized by nlnfc-init at boot/resume.
		 * Powering it down and back up here would discard that
		 * (volatile) configuration, so only clear a poll left
		 * running by an interrupted previous session. This is
		 * best-effort cleanup, not a precondition: nlnfc-init itself
		 * never starts polling, so on the common path there is
		 * nothing to stop, and the kernel's response to that varies
		 * (observed -EINVAL, but also other codes). Never fail the
		 * whole channel open over it -- if the adapter is genuinely
		 * in a bad state, poll_for_targets() below will fail loudly
		 * instead. */
		err = stop_poll_for_targets_ex(adapter, 1);
		adapter->poll_active = 0;
		if (err)
			Log2(PCSC_LOG_INFO, "Ignoring stop-poll error %d while reusing an already-powered adapter.", err);
	}
	else {
		err = nl_set_powered(adapter, 1);
		if (err)
			return -1;
	}

	err = poll_for_targets(adapter);
	if (err) {
		if (!adapter->initial_power)
			nl_set_powered(adapter, 0);
		return -1;
	}

	return 0;
}

/* Common body of IFDHCreateChannel(ByName)(): reset driver state, stand up
 * the netlink sockets, look up the adapter via find_adapter()/find_arg, and
 * bring it up. Called with state_lock held. */
static RESPONSECODE open_channel(int (*find_adapter)(void *arg, struct nfc_adapter *adapter),
		void *find_arg)
{
	RESPONSECODE result = IFD_COMMUNICATION_ERROR;

	if (ifdnlnfc_state.channel_open)
		return result;

	reset_driver_state();

	if (netlink_setup())
		return result;

	if (find_adapter(find_arg, &ifdnlnfc_state.adapter)) {
		netlink_cleanup();
		return IFD_NO_SUCH_DEVICE;
	}

	if (!initialize_adapter(&ifdnlnfc_state.adapter)) {
		ifdnlnfc_state.channel_open = 1;
		return IFD_SUCCESS;
	}

	netlink_cleanup();
	return result;
}

static int find_adapter_by_name(void *arg, struct nfc_adapter *adapter)
{
	return get_adapter_by_name((const char *)arg, adapter);
}

static int find_adapter_by_idx(void *arg, struct nfc_adapter *adapter)
{
	return get_adapter_by_idx(*(uint32_t *)arg, adapter);
}

RESPONSECODE
IFDHCreateChannelByName(DWORD Lun, LPSTR DeviceName)
{
	RESPONSECODE result;

	(void)Lun;

	if (!DeviceName)
		return IFD_COMMUNICATION_ERROR;

	pthread_mutex_lock(&state_lock);
	result = open_channel(find_adapter_by_name, DeviceName);
	pthread_mutex_unlock(&state_lock);

	return result;
}

RESPONSECODE
IFDHCreateChannel(DWORD Lun, DWORD Channel)
{
	RESPONSECODE result;
	uint32_t idx = Channel;

	(void)Lun;

	pthread_mutex_lock(&state_lock);
	result = open_channel(find_adapter_by_idx, &idx);
	pthread_mutex_unlock(&state_lock);

	return result;
}

RESPONSECODE
IFDHCloseChannel(DWORD Lun)
{
	RESPONSECODE result = IFD_COMMUNICATION_ERROR;

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (!ifdnlnfc_state.channel_open)
		goto out;

	ifdnlnfc_state.channel_open = 0;

	close_target_socket();

	stop_poll_for_targets(&ifdnlnfc_state.adapter);

	if (!ifdnlnfc_state.adapter.initial_power)
		nl_set_powered(&ifdnlnfc_state.adapter, 0);

	netlink_cleanup();
	reset_driver_state();

	result = IFD_SUCCESS;

out:
	pthread_mutex_unlock(&state_lock);
	return result;
}

static RESPONSECODE IFDHPolling(DWORD Lun, int timeout)
{
	struct pollfd fds[2];
	uint64_t wake_count;
	int effective_timeout = timeout;
	int result;
	int event_fd_dup, wake_fd_dup;

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (!event_sock || polling_wake_fd < 0) {
		pthread_mutex_unlock(&state_lock);
		return IFD_COMMUNICATION_ERROR;
	}

	Log4(PCSC_LOG_DEBUG, "card present: %d, poll active: %d, timeout: %d",
		ifdnlnfc_state.card_present, ifdnlnfc_state.adapter.poll_active, timeout);

	/* Duplicate the fds while still holding the lock, and poll/read the
	 * duplicates instead of the originals below. A concurrent
	 * IFDHCloseChannel() (called from a different pcscd thread while we
	 * are blocked in poll()) tears down event_sock/polling_wake_fd via
	 * netlink_cleanup(), which would otherwise leave us polling (and,
	 * for the wake fd, re-reading) closed -- and potentially reused --
	 * file descriptor numbers. dup() gives us our own reference to the
	 * same underlying open file description, so it keeps working
	 * correctly regardless of what happens to the originals. */
	event_fd_dup = dup(nl_socket_get_fd(event_sock));
	wake_fd_dup = dup(polling_wake_fd);

	/* Release the lock before the (potentially multi-second) blocking
	 * wait below, so other IFDH* entry points are not stalled by it. */
	pthread_mutex_unlock(&state_lock);

	if (event_fd_dup < 0 || wake_fd_dup < 0) {
		if (event_fd_dup >= 0)
			close(event_fd_dup);
		if (wake_fd_dup >= 0)
			close(wake_fd_dup);
		return IFD_COMMUNICATION_ERROR;
	}

	fds[0] = (struct pollfd){event_fd_dup, POLLIN, 0};
	fds[1] = (struct pollfd){wake_fd_dup, POLLIN, 0};

	/* While the ICC is not currently powered by the PC/SC client, there
	 * is no reliable kernel event for the tag leaving the field (most
	 * NCI drivers, including nxp-nci, never implement check_presence /
	 * emit NFC_EVENT_TARGET_LOST). Wake up periodically instead so
	 * IFDHICCPresence can actively re-probe with an empty I-block. */
	if (!atomic_load_explicit(&ifdnlnfc_state.card_powered, memory_order_relaxed) &&
		(timeout < 0 || timeout > PRESENCE_PROBE_INTERVAL_MS))
		effective_timeout = PRESENCE_PROBE_INTERVAL_MS;

	do {
		result = poll(fds, 2, effective_timeout);
	} while (result < 0 && errno == EINTR);

	if (result >= 0 && (fds[1].revents & POLLIN)) {
		int read_result;

		do {
			read_result = read(wake_fd_dup, &wake_count, sizeof(wake_count));
		} while (read_result < 0 && errno == EINTR);
	}

	close(event_fd_dup);
	close(wake_fd_dup);

	if (result < 0 || (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) ||
		(fds[1].revents & (POLLERR | POLLHUP | POLLNVAL)))
		return IFD_COMMUNICATION_ERROR;

	return IFD_SUCCESS;
}

RESPONSECODE
IFDHGetCapabilities(DWORD Lun, DWORD Tag, PDWORD Length, PUCHAR Value)
{
	(void)Lun;

	if (!Length || !Value)
		return IFD_COMMUNICATION_ERROR;

	switch (Tag) {
	case TAG_IFD_ATR:
#ifdef SCARD_ATTR_ATR_STRING
	case SCARD_ATTR_ATR_STRING:
#endif
		pthread_mutex_lock(&state_lock);
		if (!ifdnlnfc_state.target_valid || ifdnlnfc_state.socket < 0) {
			pthread_mutex_unlock(&state_lock);
			return IFD_COMMUNICATION_ERROR;
		}
		if (*Length < (DWORD)ifdnlnfc_state.target.atr_len) {
			pthread_mutex_unlock(&state_lock);
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		}
		*Length = ifdnlnfc_state.target.atr_len;
		memcpy(Value, &ifdnlnfc_state.target.atr, *Length);
		pthread_mutex_unlock(&state_lock);
		break;
	case TAG_IFD_SIMULTANEOUS_ACCESS:
		if (*Length < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*Value = 0;
		*Length = 1;
		break;
	case TAG_IFD_THREAD_SAFE:
		if (*Length < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*Value	= 0;
		*Length = 1;
		break;
	case TAG_IFD_SLOTS_NUMBER:
		if (*Length < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*Value	= 1;
		*Length = 1;
		break;
	case TAG_IFD_POLLING_THREAD_WITH_TIMEOUT:
		if (*Length < sizeof(void *))
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*Length = sizeof(void *);
		*(void **)Value = IFDHPolling;
		break;
	case TAG_IFD_POLLING_THREAD_KILLABLE:
		if (*Length < 1)
			return IFD_ERROR_INSUFFICIENT_BUFFER;
		*Length = 1;
		*Value = 1;
		break;
	default:
		Log2(PCSC_LOG_DEBUG, "Tag %08lx not supported", Tag);
		return IFD_ERROR_TAG;
	}
	return IFD_SUCCESS;
}

RESPONSECODE
IFDHSetCapabilities(DWORD Lun, DWORD Tag, DWORD Length, PUCHAR Value)
{
	(void)Lun;
	(void)Tag;
	(void)Length;
	(void)Value;

	return IFD_ERROR_VALUE_READ_ONLY;
}

RESPONSECODE
IFDHSetProtocolParameters(DWORD Lun, DWORD Protocol, UCHAR Flags, UCHAR PTS1,
			UCHAR PTS2, UCHAR PTS3)
{
	(void)Lun;
	(void)Flags;
	(void)PTS1;
	(void)PTS2;
	(void)PTS3;

	if (Protocol != SCARD_PROTOCOL_T1)
		return IFD_PROTOCOL_NOT_SUPPORTED;

	return IFD_SUCCESS;
}

static RESPONSECODE copy_atr(PUCHAR atr, PDWORD atr_length)
{
	DWORD required = ifdnlnfc_state.target.atr_len;

	if (!atr_length || (required && !atr))
		return IFD_COMMUNICATION_ERROR;

	if (*atr_length < required) {
		*atr_length = required;
		return IFD_ERROR_INSUFFICIENT_BUFFER;
	}

	*atr_length = required;
	if (required)
		memcpy(atr, ifdnlnfc_state.target.atr, required);

	return IFD_SUCCESS;
}

RESPONSECODE
IFDHPowerICC(DWORD Lun, DWORD Action, PUCHAR Atr, PDWORD AtrLength)
{
	int err;
	RESPONSECODE result;

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (!ifdnlnfc_state.channel_open) {
		result = IFD_COMMUNICATION_ERROR;
		goto out;
	}

	switch (Action) {

	case IFD_RESET:
	case IFD_POWER_UP:
		if (Action == IFD_RESET)
			Log1(PCSC_LOG_DEBUG, "IFD_RESET");
		else
			Log1(PCSC_LOG_DEBUG, "IFD_POWER_UP");

		if (!ifdnlnfc_state.card_present || !ifdnlnfc_state.target_valid) {
			result = IFD_ERROR_POWER_ACTION;
			goto out;
		}

		if (ifdnlnfc_state.socket >= 0)
			/* Already connected (kept alive across a prior
			 * IFD_POWER_DOWN, which never touches the kernel-side
			 * target -- see below): the target was never actually
			 * deactivated, so there is nothing to redo here. */
			err = 0;
		else
			err = connect_target(&ifdnlnfc_state.adapter, &ifdnlnfc_state.target);

		if (err) {
			Log2(PCSC_LOG_DEBUG, "Unable to activate NFC target: %d", err);
			remove_target();
			if (AtrLength)
				*AtrLength = 0;
			result = IFD_ERROR_POWER_ACTION;
			goto out;
		}

		result = copy_atr(Atr, AtrLength);
		atomic_store_explicit(&ifdnlnfc_state.card_powered,
			result == IFD_SUCCESS, memory_order_relaxed);
		if (result != IFD_SUCCESS)
			wake_polling_thread();
		goto out;

	case IFD_POWER_DOWN:
		Log1(PCSC_LOG_DEBUG, "IFD_POWER_DOWN");
		if (AtrLength)
			*AtrLength = 0;

		/* Keep the raw socket connected: the NFC card is still
		 * physically in the field, and closing it would deactivate
		 * the kernel target and force PC/SC's next power-up through
		 * a fresh discovery cycle, churning the target index and
		 * dropping any external chip configuration. IFDHICCPresence
		 * probes it with an empty I-block on demand instead. */
		atomic_store_explicit(&ifdnlnfc_state.card_powered, 0, memory_order_relaxed);
		wake_polling_thread();
		result = IFD_SUCCESS;
		goto out;
	default:
		result = IFD_NOT_SUPPORTED;
		goto out;
	}

out:
	pthread_mutex_unlock(&state_lock);
	return result;
}

/* Send tx_len bytes from tx and read back into rx (capacity rx_cap) over
 * the connected raw NFC target socket, storing the actual response length
 * in *rx_len. Caller must hold state_lock and have already verified the
 * socket is connected. On any I/O failure the target is dropped (the
 * kernel side gives no cleaner signal than a short write or an empty/
 * failed read that the target is gone) and -1 is returned. */
static int raw_transceive(const unsigned char *tx, size_t tx_len,
		unsigned char *rx, size_t rx_cap, size_t *rx_len)
{
	ssize_t bytes_written;
	ssize_t bytes_read;
	unsigned char kernel_header;
	struct iovec iov[] = {{&kernel_header, 1}, {rx, rx_cap}};

	do {
		bytes_written = send(ifdnlnfc_state.socket, tx, tx_len, MSG_NOSIGNAL);
	} while (bytes_written < 0 && errno == EINTR);

	if (bytes_written < 0 || (size_t)bytes_written != tx_len) {
		Log3(PCSC_LOG_DEBUG, "Wrote %ld bytes instead of %ld", (long)bytes_written, (long)tx_len);
		remove_target();
		return -1;
	}

	/* rawsock_data_exchange_complete()/rawsock_add_header() in the
	 * kernel (net/nfc/rawsock.c) prepends one extra byte (always 0) to
	 * every response before it is queued for us to read; strip it here
	 * rather than leak it into the ICC response PC/SC clients see. */
	do {
		bytes_read = readv(ifdnlnfc_state.socket, iov, 2);
	} while (bytes_read < 0 && errno == EINTR);

	if (bytes_read == -1) {
		Log2(PCSC_LOG_DEBUG, "readv() error, errno: %d", errno);
	}

	if (bytes_read < 1) {
		remove_target();
		return -1;
	}

	*rx_len = (size_t)(bytes_read - 1);
	return 0;
}

/* PC/SC Part 10 pseudo-APDU extensions used by German eID software (e.g.
 * AusweisApp2, per BSI TR-03119) to query reader identity and raw target
 * data a normal ICC-facing APDU can't reach. Every command is a 5-byte
 * case-2 APDU (FF INS P1 P2 Le, no command data); Le=0 means "return
 * everything". Mirrors ifdnfc-nci's src/ifdnfc-nci.c IFDHTransmitToICC()
 * pseudo-APDU handling. Caller must hold state_lock (needs a connected
 * target for UID/historical bytes) and pass TxBuffer[0] == 0xFF. */
static RESPONSECODE handle_pseudo_apdu(const unsigned char *tx, size_t tx_len,
		unsigned char *rx, size_t rx_cap, size_t *rx_len)
{
	struct nfc_target *target = &ifdnlnfc_state.target;
	const unsigned char *data = NULL;
	size_t data_len = 0;
	size_t le;
	size_t off;

	if (tx_len != 5) {
		if (rx_cap < 2)
			return IFD_COMMUNICATION_ERROR;
		rx[0] = 0x67;
		rx[1] = 0x00;
		*rx_len = 2;
		return IFD_SUCCESS;
	}

	le = tx[4];

	switch (tx[1]) {
	case 0x9A: /* Reader information */
		if (tx[2] != 0x01)
			goto not_supported;
		switch (tx[3]) {
		case 0x01: /* Vendor name */
			data = (const unsigned char *)"Linux NFC";
			break;
		case 0x03: /* Product name */
			data = (const unsigned char *)"Netlink NFC Reader";
			break;
		case 0x06: /* Firmware version -- not queryable via the
			    * generic kernel NFC netlink API, unlike the
			    * vendor HAL ifdnfc-nci links against. */
			data = (const unsigned char *)"n/a";
			break;
		case 0x07: /* Driver version */
			data = (const unsigned char *)PACKAGE_VERSION;
			break;
		default:
			goto not_supported;
		}
		data_len = strlen((const char *)data);
		break;
	case 0xCA: /* Get Data */
		switch (tx[2]) {
		case 0x00: /* Get UID */
			data = target->uid;
			data_len = (size_t)target->uid_len;
			break;
		case 0x01: /* Get ATS historical bytes */
			if (target->atr_len >= 5) {
				data = &target->atr[4];
				data_len = (size_t)(target->atr[1] - 0x80);
			}
			break;
		default:
			goto not_supported;
		}
		break;
	default:
		goto not_supported;
	}

	if (le == 0)
		le = data_len;

	if (le < data_len) {
		if (rx_cap < 2)
			return IFD_COMMUNICATION_ERROR;
		rx[0] = 0x6C;
		rx[1] = (unsigned char)data_len;
		*rx_len = 2;
		return IFD_SUCCESS;
	}

	if (rx_cap < le + 2)
		return IFD_COMMUNICATION_ERROR;

	off = data_len;
	if (data_len)
		memcpy(rx, data, data_len);
	if (le > data_len) {
		/* End of data reached before Le bytes: pad with zeros. */
		memset(rx + off, 0, le - data_len);
		off = le;
		rx[off++] = 0x62;
		rx[off++] = 0x82;
	} else {
		rx[off++] = 0x90;
		rx[off++] = 0x00;
	}
	*rx_len = off;
	return IFD_SUCCESS;

not_supported:
	if (rx_cap < 2)
		return IFD_COMMUNICATION_ERROR;
	rx[0] = 0x6A;
	rx[1] = 0x81;
	*rx_len = 2;
	return IFD_SUCCESS;
}

RESPONSECODE
IFDHTransmitToICC(DWORD Lun, SCARD_IO_HEADER SendPci, PUCHAR TxBuffer, DWORD
		TxLength, PUCHAR RxBuffer, PDWORD RxLength, PSCARD_IO_HEADER RecvPci)
{
	RESPONSECODE result;
	size_t rx_len = 0;

	(void)Lun;

	/* Held across raw_transceive() below, not just the state checks: the
	 * socket fd must not be closed by a concurrent remove_target() (from
	 * IFDHPolling/IFDHICCPresence on the polling thread) while a
	 * transceive using it is in flight. */
	pthread_mutex_lock(&state_lock);

	if (ifdnlnfc_state.socket < 0) {
		result = IFD_COMMUNICATION_ERROR;
		goto out;
	}

	if (TxLength && TxBuffer[0] == 0xFF) {
		result = handle_pseudo_apdu(TxBuffer, TxLength, RxBuffer, *RxLength, &rx_len);
		if (result == IFD_SUCCESS) {
			*RxLength = (DWORD)rx_len;
			RecvPci->Protocol = SCARD_PROTOCOL_T1;
		} else {
			*RxLength = 0;
		}
		goto out;
	}

	/* SendPci.Protocol is NOT the SCARD_PROTOCOL_* bitmask (T0=1, T1=2):
	 * winscard.c's SCardTransmit() remaps it to a plain 0 (T=0) / 1 (T=1)
	 * before calling the IFD handler ("PC/SC starts at 1 for bit masking
	 * but the IFD_Handler just wants 0 or 1", per its own comment) --
	 * unlike IFDHSetProtocolParameters()'s Protocol argument, which is
	 * the real, unremapped SCARD_PROTOCOL_* value. Comparing against
	 * SCARD_PROTOCOL_T1 (2) here instead of 1 meant this rejected every
	 * transmit unconditionally, on any protocol. */
	if (SendPci.Protocol != 1) {
		result = IFD_NOT_SUPPORTED;
		goto out;
	}

	if (raw_transceive(TxBuffer, TxLength, RxBuffer, *RxLength, &rx_len)) {
		*RxLength = 0;
		result = IFD_ICC_NOT_PRESENT;
		goto out;
	}

	*RxLength = (DWORD)rx_len;
	RecvPci->Protocol = SCARD_PROTOCOL_T1;

	result = IFD_SUCCESS;

out:
	pthread_mutex_unlock(&state_lock);
	return result;
}

static int load_current_target(void)
{
	if (list_targets(&ifdnlnfc_state.adapter, &ifdnlnfc_state.target)) {
		remove_target();
		return -1;
	}

	ifdnlnfc_state.target_valid = 1;
	return 0;
}

RESPONSECODE
IFDHICCPresence(DWORD Lun)
{
	RESPONSECODE result;
	int err;
	unsigned char probe_rx[32];
	size_t probe_rx_len;

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (!ifdnlnfc_state.channel_open) {
		result = IFD_COMMUNICATION_ERROR;
		goto out;
	}

	if (ifdnlnfc_state.adapter_removed) {
		result = IFD_NO_SUCH_DEVICE;
		goto out;
	}

	if (ifdnlnfc_state.card_present) {
		if (ifdnlnfc_state.socket >= 0 &&
			!atomic_load_explicit(&ifdnlnfc_state.card_powered, memory_order_relaxed)) {
			/* Empty I-block: zero-length data exchange, exactly
			 * what linux_libnfc-nci's own RW_T4tPresenceCheck()
			 * uses by default (RW_T4T_CHK_EMPTY_I_BLOCK, see
			 * rw_t4t.c). This is a pure ISO 14443-4/T=1
			 * protocol-layer exchange -- no APDU header, nothing
			 * for the card to interpret as a command -- so unlike
			 * a SELECT it cannot disturb whatever application/
			 * applet a paused client transaction has selected.
			 * Any response at all is proof of life; only a
			 * raw_transceive() I/O failure means the target is
			 * actually gone. */
			if (!raw_transceive(NULL, 0, probe_rx, sizeof(probe_rx), &probe_rx_len)) {
				(void)probe_rx_len;
				result = IFD_SUCCESS;
				goto out;
			}
			/* raw_transceive() already called remove_target() on failure. */
		}
		else {
			result = IFD_SUCCESS;
			goto out;
		}
	}

	if (!ifdnlnfc_state.adapter.poll_active)
		poll_for_targets(&ifdnlnfc_state.adapter);

	err = nl_recvmsgs_default(event_sock);

	if (ifdnlnfc_state.adapter_removed) {
		result = IFD_NO_SUCH_DEVICE;
		goto out;
	}

	if (!err && ifdnlnfc_state.card_present)
	{
		result = load_current_target() ? IFD_COMMUNICATION_ERROR : IFD_SUCCESS;
		goto out;
	}
	result = IFD_ICC_NOT_PRESENT;

out:
	pthread_mutex_unlock(&state_lock);
	return result;
}

RESPONSECODE
IFDHControl(DWORD Lun, DWORD dwControlCode, PUCHAR TxBuffer, DWORD TxLength,
	PUCHAR RxBuffer, DWORD RxLength, LPDWORD pdwBytesReturned)
{
	(void)Lun;
	(void)TxBuffer;
	(void)TxLength;

	if (!pdwBytesReturned)
		return IFD_COMMUNICATION_ERROR;

	*pdwBytesReturned = 0;

	if (dwControlCode == CM_IOCTL_GET_FEATURE_REQUEST)
	{
		PCSC_TLV_STRUCTURE *pcsc_tlv = (PCSC_TLV_STRUCTURE *)RxBuffer;

		if (!RxBuffer || RxLength < sizeof(*pcsc_tlv))
			return IFD_ERROR_INSUFFICIENT_BUFFER;

		pcsc_tlv->tag = FEATURE_GET_TLV_PROPERTIES;
		pcsc_tlv->length = 4;
		pcsc_tlv->value = htonl(IOCTL_FEATURE_GET_TLV_PROPERTIES);
		*pdwBytesReturned = sizeof(PCSC_TLV_STRUCTURE);
		return IFD_SUCCESS;
	}

	if (dwControlCode == IOCTL_FEATURE_GET_TLV_PROPERTIES)
	{
		int p = 0;

		if (!RxBuffer || RxLength < 6)
			return IFD_ERROR_INSUFFICIENT_BUFFER;

		RxBuffer[p++] = PCSCv2_PART10_PROPERTY_dwMaxAPDUDataSize;
		RxBuffer[p++] = 4;	/* length */
		RxBuffer[p++] = 0xff;
		RxBuffer[p++] = 0xff;
		RxBuffer[p++] = 0;
		RxBuffer[p++] = 0;
		*pdwBytesReturned = p;
		return IFD_SUCCESS;
	}

	return IFD_ERROR_NOT_SUPPORTED;
}
