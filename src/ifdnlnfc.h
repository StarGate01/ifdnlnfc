/*
 *
 *  libnlnfc - PCSC driver for Linux Kernel NFC devices
 *
 *  Copyright (C) 2024 Juraj Šarinay <juraj@sarinay.com>
 *
 *  uses code from neard & nfctool by Intel
 *  https://github.com/linux-nfc/neard
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
 */

#include "config.h"
#include <ifdhandler.h>
#include <inttypes.h>
#include <linux/nfc.h>
#include <stdatomic.h>

// constants from the USB CCID IFD Handler by Ludovic Rousseau
// for consistency
#define CLASS2_IOCTL_MAGIC 0x330000
#define IOCTL_FEATURE_GET_TLV_PROPERTIES SCARD_CTL_CODE(FEATURE_GET_TLV_PROPERTIES + CLASS2_IOCTL_MAGIC)

static struct nl_sock *cmd_sock, *event_sock;
static int nfc_family_id;

struct nfc_adapter {
	uint32_t idx;
	int poll_active;
	uint8_t initial_power;
	uint8_t initial_mode;
	uint32_t protocols;
};

struct nfc_target {
	uint32_t idx;
	uint32_t supported_protocols;
	uint32_t active_protocol;
	uint8_t  atr[MAX_ATR_SIZE];
	int atr_len;
	/* NFC-A unique identifier (NFC_ATTR_TARGET_NFCID1), for the
	 * PC/SC Part 10 pseudo-APDU GET DATA(00) extension. */
	uint8_t  uid[NFC_NFCID1_MAXSIZE];
	int uid_len;
};

struct ifdnlnfc_state {
	struct nfc_adapter adapter;
	struct nfc_target target;
	int channel_open;
	/* A target was reported by a poll and has a valid kernel target index. */
	int target_valid;
	/* NFC_EVENT_TARGETS_FOUND for the current target arrived, i.e. the
	 * kernel still tracks the target represented by "target" above,
	 * whether or not the raw socket is currently connected to it. */
	int card_present;
	/* The NFC adapter itself disappeared (e.g. device removal). */
	int adapter_removed;
	/* The PC/SC client has powered the ICC up (IFD_POWER_UP/IFD_RESET
	 * succeeded, no IFD_POWER_DOWN since). Distinct from card_present:
	 * we keep the raw socket connected across IFD_POWER_DOWN so the
	 * kernel target index survives PC/SC's idle power-down cycling. */
	atomic_int card_powered;
	int socket;
};

struct list_adapters_cb_state {
	const char * name;
	int found;
	struct nfc_adapter * adapter;
};

struct get_adapter_cb_state {
	int found;
	uint32_t idx;
	struct nfc_adapter * adapter;
};

struct list_targets_cb_state {
	int found;
	struct nfc_target *target;
};
