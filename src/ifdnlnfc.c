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
#define PRESENCE_PROBE_INTERVAL_MS 3000

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

static int nl_reactivate_target(uint32_t adapter_idx, uint32_t target_idx, uint32_t protocol)
{
	struct nl_msg *msg;
	void *hdr;
	int err;
	uint8_t cmd;

	msg = nlmsg_alloc();
	if (!msg)
		return -ENOMEM;

	cmd = NFC_CMD_ACTIVATE_TARGET;

	hdr = genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, nfc_family_id, 0,
			NLM_F_REQUEST, cmd, NFC_GENL_VERSION);
	if (!hdr) {
		err = -EINVAL;
		goto nla_put_failure;
	}

	err = -EMSGSIZE;

	NLA_PUT_U32(msg, NFC_ATTR_DEVICE_INDEX, adapter_idx);
	NLA_PUT_U32(msg, NFC_ATTR_TARGET_INDEX, target_idx);
	NLA_PUT_U32(msg, NFC_ATTR_PROTOCOLS, protocol);

	err = nl_send_msg(cmd_sock, msg, NULL, NULL);

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

static int stop_poll_for_targets(struct nfc_adapter * adapter)
{
	struct nl_msg *msg;
	void *hdr;
	int err = -EINVAL;

	if (!adapter->poll_active) {
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

static int reactivate_current_target(void)
{
	int err;

	if (!ifdnlnfc_state.target_valid || ifdnlnfc_state.socket < 0)
		return -ENOTCONN;

	err = nl_reactivate_target(ifdnlnfc_state.adapter.idx,
		ifdnlnfc_state.target.idx, ifdnlnfc_state.target.active_protocol);
	if (err)
		return err;

	if (ifdnlnfc_state.target.active_protocol == NFC_PROTO_ISO14443) {
		set_atr_from_hb(&ifdnlnfc_state.target, NULL, 0);
		get_target_ats(&ifdnlnfc_state.adapter, &ifdnlnfc_state.target);
	}

	return 0;
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
		 * proprietary-initialized by npc300-init at boot/resume.
		 * Powering it down and back up here would discard that
		 * (volatile) configuration, so only clear a poll left
		 * running by an interrupted previous session. */
		adapter->poll_active = 1;
		err = stop_poll_for_targets(adapter);
		adapter->poll_active = 0;
		if (err)
			return -1;
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

RESPONSECODE
IFDHCreateChannelByName(DWORD Lun, LPSTR DeviceName)
{
	RESPONSECODE result = IFD_COMMUNICATION_ERROR;

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (ifdnlnfc_state.channel_open || !DeviceName)
		goto out;

	reset_driver_state();

	if (netlink_setup())
		goto out;

	if (get_adapter_by_name(DeviceName, &ifdnlnfc_state.adapter)) {
		netlink_cleanup();
		result = IFD_NO_SUCH_DEVICE;
		goto out;
	}

	if (!initialize_adapter(&ifdnlnfc_state.adapter)) {
		ifdnlnfc_state.channel_open = 1;
		result = IFD_SUCCESS;
		goto out;
	}

	netlink_cleanup();

out:
	pthread_mutex_unlock(&state_lock);
	return result;
}

RESPONSECODE
IFDHCreateChannel(DWORD Lun, DWORD Channel)
{
	RESPONSECODE result = IFD_COMMUNICATION_ERROR;

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (ifdnlnfc_state.channel_open)
		goto out;

	reset_driver_state();

	if (netlink_setup())
		goto out;

	if (get_adapter_by_idx(Channel, &ifdnlnfc_state.adapter)) {
		netlink_cleanup();
		result = IFD_NO_SUCH_DEVICE;
		goto out;
	}

	if (!initialize_adapter(&ifdnlnfc_state.adapter)) {
		ifdnlnfc_state.channel_open = 1;
		result = IFD_SUCCESS;
		goto out;
	}

	netlink_cleanup();

out:
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

	(void)Lun;

	pthread_mutex_lock(&state_lock);

	if (!event_sock || polling_wake_fd < 0) {
		pthread_mutex_unlock(&state_lock);
		return IFD_COMMUNICATION_ERROR;
	}

	Log4(PCSC_LOG_DEBUG, "card present: %d, poll active: %d, timeout: %d",
		ifdnlnfc_state.card_present, ifdnlnfc_state.adapter.poll_active, timeout);

	fds[0] = (struct pollfd){nl_socket_get_fd(event_sock), POLLIN, 0};
	fds[1] = (struct pollfd){polling_wake_fd, POLLIN, 0};

	/* Release the lock before the (potentially multi-second) blocking
	 * wait below, so other IFDH* entry points are not stalled by it. */
	pthread_mutex_unlock(&state_lock);

	/* While the ICC is not currently powered by the PC/SC client, there
	 * is no reliable kernel event for the tag leaving the field (most
	 * NCI drivers, including nxp-nci, never implement check_presence /
	 * emit NFC_EVENT_TARGET_LOST). Wake up periodically instead so
	 * IFDHICCPresence can actively re-probe via target reactivation. */
	if (!atomic_load_explicit(&ifdnlnfc_state.card_powered, memory_order_relaxed) &&
		(timeout < 0 || timeout > PRESENCE_PROBE_INTERVAL_MS))
		effective_timeout = PRESENCE_PROBE_INTERVAL_MS;

	do {
		result = poll(fds, 2, effective_timeout);
	} while (result < 0 && errno == EINTR);

	if (result < 0 || (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) ||
		(fds[1].revents & (POLLERR | POLLHUP | POLLNVAL)))
		return IFD_COMMUNICATION_ERROR;

	if (fds[1].revents & POLLIN) {
		do {
			result = read(polling_wake_fd, &wake_count, sizeof(wake_count));
		} while (result < 0 && errno == EINTR);
	}

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
		Log1(PCSC_LOG_DEBUG, Action == IFD_RESET ? "IFD_RESET" : "IFD_POWER_UP");

		if (!ifdnlnfc_state.card_present || !ifdnlnfc_state.target_valid) {
			result = IFD_ERROR_POWER_ACTION;
			goto out;
		}

		if (ifdnlnfc_state.socket >= 0)
			/* Already connected (kept alive across a prior
			 * IFD_POWER_DOWN): bring the target back to ACTIVE
			 * state rather than reconnecting from scratch. */
			err = reactivate_current_target();
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
		 * reactivates the same target on demand instead. */
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

RESPONSECODE
IFDHTransmitToICC(DWORD Lun, SCARD_IO_HEADER SendPci, PUCHAR TxBuffer, DWORD
		TxLength, PUCHAR RxBuffer, PDWORD RxLength, PSCARD_IO_HEADER RecvPci)
{
	RESPONSECODE result;
	ssize_t bytes_read;
	ssize_t bytes_written;

	unsigned char null_header;
	struct iovec iov[] = {{&null_header, 1}, {RxBuffer, *RxLength}};

	(void)Lun;

	/* Held across send()/readv() below, not just the state checks: the
	 * socket fd must not be closed by a concurrent remove_target() (from
	 * IFDHPolling/IFDHICCPresence on the polling thread) while a
	 * transceive using it is in flight. */
	pthread_mutex_lock(&state_lock);

	if (ifdnlnfc_state.socket < 0) {
		result = IFD_COMMUNICATION_ERROR;
		goto out;
	}

	if (SendPci.Protocol != SCARD_PROTOCOL_T1) {
		result = IFD_NOT_SUPPORTED;
		goto out;
	}

	do {
		bytes_written = send(ifdnlnfc_state.socket, TxBuffer, TxLength, MSG_NOSIGNAL);
	} while (bytes_written < 0 && errno == EINTR);

	if (bytes_written < 0 || (DWORD)bytes_written != TxLength) {
		Log3(PCSC_LOG_DEBUG, "Wrote %ld bytes instead of %ld", (long)bytes_written, (long)TxLength);
		*RxLength = 0;
		remove_target();
		result = IFD_ICC_NOT_PRESENT;
		goto out;
	}

	do {
		bytes_read = readv(ifdnlnfc_state.socket, iov, 2);
	} while (bytes_read < 0 && errno == EINTR);

	if (bytes_read == -1) {
		Log2(PCSC_LOG_DEBUG, "readv() error, errno: %d", errno);
	}

	if (bytes_read < 1)
	{
		*RxLength = 0;
		remove_target();
		result = IFD_ICC_NOT_PRESENT;
		goto out;
	}

	bytes_read--;

	*RxLength = (DWORD)bytes_read;
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
			/* Logically powered down but kept connected: actively
			 * re-probe presence via target reactivation, since no
			 * kernel event reliably tells us if the tag left the
			 * field while we were idle. */
			err = reactivate_current_target();
			if (err) {
				Log2(PCSC_LOG_DEBUG, "NFC presence probe failed: %d", err);
				remove_target();
			}
			else {
				result = IFD_SUCCESS;
				goto out;
			}
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
