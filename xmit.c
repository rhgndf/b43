// SPDX-License-Identifier: GPL-2.0-or-later
/*

  Broadcom B43 wireless driver

  Transmission (TX/RX) related functions.

  Copyright (C) 2005 Martin Langer <martin-langer@gmx.de>
  Copyright (C) 2005 Stefano Brivio <stefano.brivio@polimi.it>
  Copyright (C) 2005, 2006 Michael Buesch <m@bues.ch>
  Copyright (C) 2005 Danny van Dyk <kugelfang@gentoo.org>
  Copyright (C) 2005 Andreas Jaggi <andreas.jaggi@waterwave.ch>


*/

/*
 * HT packet-format and timing algorithms are adapted from brcmsmac's
 * main.c, rate.c and d11.h, distributed under the following ISC notice:
 *
 * Copyright (c) 2010 Broadcom Corporation
 * Copyright (c) 2013 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "xmit.h"
#include "phy_common.h"
#include "dma.h"
#include "pio.h"
#include <linux/crc8.h>
#include <linux/ktime.h>
#include <linux/unaligned.h>

static const struct b43_tx_legacy_rate_phy_ctl_entry b43_tx_legacy_rate_phy_ctl[] = {
	{ B43_CCK_RATE_1MB,	0x0,			0x0 },
	{ B43_CCK_RATE_2MB,	0x0,			0x1 },
	{ B43_CCK_RATE_5MB,	0x0,			0x2 },
	{ B43_CCK_RATE_11MB,	0x0,			0x3 },
	{ B43_OFDM_RATE_6MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_BPSK },
	{ B43_OFDM_RATE_9MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_BPSK },
	{ B43_OFDM_RATE_12MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_QPSK },
	{ B43_OFDM_RATE_18MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QPSK },
	{ B43_OFDM_RATE_24MB,	B43_TXH_PHY1_CRATE_1_2,	B43_TXH_PHY1_MODUL_QAM16 },
	{ B43_OFDM_RATE_36MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QAM16 },
	{ B43_OFDM_RATE_48MB,	B43_TXH_PHY1_CRATE_2_3,	B43_TXH_PHY1_MODUL_QAM64 },
	{ B43_OFDM_RATE_54MB,	B43_TXH_PHY1_CRATE_3_4,	B43_TXH_PHY1_MODUL_QAM64 },
};

static const struct b43_tx_legacy_rate_phy_ctl_entry *
b43_tx_legacy_rate_phy_ctl_ent(u8 bitrate)
{
	const struct b43_tx_legacy_rate_phy_ctl_entry *e;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(b43_tx_legacy_rate_phy_ctl); i++) {
		e = &(b43_tx_legacy_rate_phy_ctl[i]);
		if (e->bitrate == bitrate)
			return e;
	}

	B43_WARN_ON(1);
	return NULL;
}

/* Extract the bitrate index out of a CCK PLCP header. */
static int b43_plcp_get_bitrate_idx_cck(struct b43_plcp_hdr6 *plcp)
{
	switch (plcp->raw[0]) {
	case 0x0A:
		return 0;
	case 0x14:
		return 1;
	case 0x37:
		return 2;
	case 0x6E:
		return 3;
	}
	return -1;
}

/* Extract the bitrate index out of an OFDM PLCP header. */
static int b43_plcp_get_bitrate_idx_ofdm(struct b43_plcp_hdr6 *plcp, bool ghz5)
{
	/* For 2 GHz band first OFDM rate is at index 4, see main.c */
	int base = ghz5 ? 0 : 4;

	switch (plcp->raw[0] & 0xF) {
	case 0xB:
		return base + 0;
	case 0xF:
		return base + 1;
	case 0xA:
		return base + 2;
	case 0xE:
		return base + 3;
	case 0x9:
		return base + 4;
	case 0xD:
		return base + 5;
	case 0x8:
		return base + 6;
	case 0xC:
		return base + 7;
	}
	return -1;
}

u8 b43_plcp_get_ratecode_cck(const u8 bitrate)
{
	switch (bitrate) {
	case B43_CCK_RATE_1MB:
		return 0x0A;
	case B43_CCK_RATE_2MB:
		return 0x14;
	case B43_CCK_RATE_5MB:
		return 0x37;
	case B43_CCK_RATE_11MB:
		return 0x6E;
	}
	B43_WARN_ON(1);
	return 0;
}

u8 b43_plcp_get_ratecode_ofdm(const u8 bitrate)
{
	switch (bitrate) {
	case B43_OFDM_RATE_6MB:
		return 0xB;
	case B43_OFDM_RATE_9MB:
		return 0xF;
	case B43_OFDM_RATE_12MB:
		return 0xA;
	case B43_OFDM_RATE_18MB:
		return 0xE;
	case B43_OFDM_RATE_24MB:
		return 0x9;
	case B43_OFDM_RATE_36MB:
		return 0xD;
	case B43_OFDM_RATE_48MB:
		return 0x8;
	case B43_OFDM_RATE_54MB:
		return 0xC;
	}
	B43_WARN_ON(1);
	return 0;
}

void b43_generate_plcp_hdr(struct b43_plcp_hdr4 *plcp,
			   const u16 octets, const u8 bitrate, bool ht,
			   bool aggregate)
{
	__u8 *raw = plcp->raw;

	if (ht) {
		raw[0] = bitrate;
		put_unaligned_le16(octets, raw + 1);
		raw[3] = 0x07; /* Smoothing, not sounding, reserved. */
		if (aggregate)
			raw[3] |= 0x08; /* Aggregation */
	} else if (b43_is_ofdm_rate(bitrate)) {
		u32 d;

		d = b43_plcp_get_ratecode_ofdm(bitrate);
		B43_WARN_ON(octets & 0xF000);
		d |= (octets << 5);
		plcp->data = cpu_to_le32(d);
	} else {
		u32 plen;

		plen = octets * 16 / bitrate;
		if ((octets * 16 % bitrate) > 0) {
			plen++;
			if ((bitrate == B43_CCK_RATE_11MB)
			    && ((octets * 8 % 11) < 4)) {
				raw[1] = 0x84;
			} else
				raw[1] = 0x04;
		} else
			raw[1] = 0x04;
		plcp->data |= cpu_to_le32(plen << 16);
		raw[0] = b43_plcp_get_ratecode_cck(bitrate);
	}
}

/* MCS 0-7 with BCC at 20 MHz, from brcmsmac's MCS table */
static const u16 b43_ht20_ndbps[8] = { 26, 52, 78, 104, 156, 208, 234, 260 };
static const u16 b43_ht20_phyctl[8] = {
	0x0000, 0x0800, 0x0a00, 0x1000, 0x1200, 0x1900, 0x1a00, 0x1c00,
};

static unsigned int b43_ht20_symbols(u8 mcs, unsigned int octets)
{
	return DIV_ROUND_UP(16 + 8 * octets + 6,
			    b43_ht20_ndbps[mcs & 7] * (1 + (mcs >> 3)));
}

/* Airtime includes the FCS/trailer in octets, but not SIFS. */
static unsigned int b43_tx_airtime(u8 rate, bool ht, bool short_preamble,
				    unsigned int octets, bool gmode)
{
	if (ht)
		return 36 + 4 * (rate >> 3) +
		       4 * b43_ht20_symbols(rate, octets) +
		       (gmode ? 6 : 0);
	if (b43_is_ofdm_rate(rate))
		return 20 + 4 * DIV_ROUND_UP(16 + 8 * octets + 6,
					     2 * rate) + (gmode ? 6 : 0);
	return (short_preamble ? 96 : 192) +
	       DIV_ROUND_UP(16 * octets, rate);
}

static u8 b43_tx_response_rate(struct b43_wldev *dev,
			       struct ieee80211_tx_info *info,
			       u8 rate, bool ht, bool protection)
{
	static const u8 mcs_legacy[8] = { 12, 24, 36, 48, 72, 96, 108, 108 };
	struct ieee80211_supported_band *sband =
		dev->wl->hw->wiphy->bands[info->band];
	struct ieee80211_vif *vif = info->control.vif;
	u8 response = 0;
	int i;
	u8 target = ht ? mcs_legacy[rate & 7] : rate;

	if (protection && vif && vif->bss_conf.use_cts_prot)
		target = B43_CCK_RATE_11MB;
	/* Keep the modulation class of the target, after ERP protection. */
	for (i = 0; vif && i < sband->n_bitrates; i++) {
		u8 candidate = sband->bitrates[i].hw_value;

		if ((vif->bss_conf.basic_rates & BIT(i)) &&
		    b43_is_ofdm_rate(candidate) == b43_is_ofdm_rate(target) &&
		    candidate <= target && candidate > response)
			response = candidate;
	}
	if (!response) {
		/* No basic rate fits: use a mandatory rate of the same class. */
		if (b43_is_cck_rate(target))
			response = target;
		else if (target >= B43_OFDM_RATE_24MB)
			response = B43_OFDM_RATE_24MB;
		else if (target >= B43_OFDM_RATE_12MB)
			response = B43_OFDM_RATE_12MB;
		else
			response = B43_OFDM_RATE_6MB;
	}
	return response;
}

/* Responses and protection frames follow the BSS preamble setting. */
static bool b43_tx_response_short_preamble(struct ieee80211_vif *vif, u8 rate)
{
	return vif && vif->bss_conf.use_short_preamble &&
	       b43_is_cck_rate(rate) && rate != B43_CCK_RATE_1MB;
}

/* The response is a 14-byte ACK, or a 32-byte compressed Block Ack. */
static unsigned int b43_tx_nav(struct b43_wldev *dev,
			       struct ieee80211_tx_info *info,
			       u8 rate, bool ht, bool short_preamble,
			       unsigned int next_len, bool block_ack)
{
	u8 response = b43_tx_response_rate(dev, info, rate, ht, false);
	bool response_short_preamble =
		b43_tx_response_short_preamble(info->control.vif, response);
	unsigned int sifs = dev->phy.gmode ? 10 : 16;
	unsigned int nav = sifs + b43_tx_airtime(response, false,
					response_short_preamble,
					block_ack ? 32 : 14, dev->phy.gmode);

	if (next_len)
		nav = 2 * nav + sifs + b43_tx_airtime(rate, ht, short_preamble,
						     next_len, dev->phy.gmode);
	return nav;
}

/* TODO: verify if needed for SSLPN or LCN  */
static u16 b43_generate_tx_phy_ctl1(struct b43_wldev *dev, u8 bitrate, bool ht)
{
	const struct b43_phy *phy = &dev->phy;
	const struct b43_tx_legacy_rate_phy_ctl_entry *e;
	u16 control = 0;
	u16 bw;

	if (phy->type == B43_PHYTYPE_LP)
		bw = B43_TXH_PHY1_BW_20;
	else /* FIXME */
		bw = B43_TXH_PHY1_BW_20;

	if (ht) {
		control = bw | b43_ht20_phyctl[bitrate & 7];
		if (bitrate >= 8)
			control |= B43_TXH_PHY1_MODE_SDM |
				   (1 << B43_TXH_PHY1_NSS_SHIFT);
		else
			control |= B43_TXH_PHY1_MODE_CDD;
	} else if (b43_is_cck_rate(bitrate) && phy->type != B43_PHYTYPE_LP) {
		control = bw;
	} else {
		control = bw;
		e = b43_tx_legacy_rate_phy_ctl_ent(bitrate);
		if (e) {
			control |= e->coding_rate;
			control |= e->modulation;
		}
		control |= B43_TXH_PHY1_MODE_SISO;
	}

	return control;
}

static u8 b43_calc_fallback_rate(u8 bitrate, int gmode)
{
	switch (bitrate) {
	case B43_CCK_RATE_1MB:
		return B43_CCK_RATE_1MB;
	case B43_CCK_RATE_2MB:
		return B43_CCK_RATE_1MB;
	case B43_CCK_RATE_5MB:
		return B43_CCK_RATE_2MB;
	case B43_CCK_RATE_11MB:
		return B43_CCK_RATE_5MB;
	/*
	 * Don't just fallback to CCK; it may be in 5GHz operation
	 * and falling back to CCK won't work out very well.
	 */
	case B43_OFDM_RATE_6MB:
		if (gmode)
			return B43_CCK_RATE_5MB;
		else
			return B43_OFDM_RATE_6MB;
	case B43_OFDM_RATE_9MB:
		return B43_OFDM_RATE_6MB;
	case B43_OFDM_RATE_12MB:
		return B43_OFDM_RATE_9MB;
	case B43_OFDM_RATE_18MB:
		return B43_OFDM_RATE_12MB;
	case B43_OFDM_RATE_24MB:
		return B43_OFDM_RATE_18MB;
	case B43_OFDM_RATE_36MB:
		return B43_OFDM_RATE_24MB;
	case B43_OFDM_RATE_48MB:
		return B43_OFDM_RATE_36MB;
	case B43_OFDM_RATE_54MB:
		return B43_OFDM_RATE_48MB;
	}
	B43_WARN_ON(1);
	return 0;
}

/* Generate a TX data header. */
int b43_generate_txhdr(struct b43_wldev *dev,
		       u8 *_txhdr,
		       struct sk_buff *skb_frag,
		       struct ieee80211_tx_info *info,
		       u16 cookie,
		       const struct b43_txhdr_ampdu *ampdu)
{
	const unsigned char *fragment_data = skb_frag->data;
	unsigned int fragment_len = skb_frag->len;
	struct b43_txhdr *txhdr = (struct b43_txhdr *)_txhdr;
	const struct b43_phy *phy = &dev->phy;
	struct ieee80211_hdr *wlhdr =
	    (struct ieee80211_hdr *)fragment_data;
	int use_encryption = !!info->control.hw_key;
	__le16 fctl = wlhdr->frame_control;
	struct ieee80211_rate *fbrate;
	u8 rate, rate_fb;	/* Legacy 500 kbps units, or MCS index. */
	int rate_ofdm, rate_fb_ofdm;
	bool rate_ht = false, rate_fb_ht = false;
	bool ht, ack, short_preamble, short_preamble_fb;
	unsigned int fes = 0, fes_fb = 0, next_len = 0;
	unsigned int plcp_fragment_len, ppdu_len;
	bool protect = !ampdu || ampdu->first;
	u32 mac_ctl = 0;
	u16 phy_ctl = 0;
	bool fill_phy_ctl1 = (phy->type == B43_PHYTYPE_LP ||
			      phy->type == B43_PHYTYPE_N ||
			      phy->type == B43_PHYTYPE_HT);
	u8 extra_ft = 0;
	struct ieee80211_rate *txrate;
	struct ieee80211_vif *vif = info->control.vif;
	struct ieee80211_tx_rate *rates;

	memset(txhdr, 0, sizeof(*txhdr));

	if (info->control.rates[0].flags & IEEE80211_TX_RC_MCS) {
		rate = info->control.rates[0].idx;
		txrate = NULL;
		rate_ht = true;
	} else {
		txrate = ieee80211_get_tx_rate(dev->wl->hw, info);
		rate = txrate ? txrate->hw_value : B43_CCK_RATE_1MB;
	}
	rate_ofdm = !rate_ht && b43_is_ofdm_rate(rate);
	if (info->control.rates[1].idx >= 0 &&
	    (info->control.rates[1].flags & IEEE80211_TX_RC_MCS)) {
		rate_fb = info->control.rates[1].idx;
		rate_fb_ht = true;
	} else if (rate_ht && info->control.rates[1].idx < 0) {
		/* Without a fallback stage, retry at the primary MCS. */
		rate_fb_ht = true;
		rate_fb = rate;
	} else {
		fbrate = ieee80211_get_alt_retry_rate(dev->wl->hw, info, 0) ? : txrate;
		rate_fb = fbrate ? fbrate->hw_value : rate;
	}
	rate_fb_ofdm = !rate_fb_ht && b43_is_ofdm_rate(rate_fb);
	ht = rate_ht || rate_fb_ht;
	/* A-MPDU members use the primary HT rate; the caller checks limits. */
	if (ampdu && (!rate_ht || (info->flags & IEEE80211_TX_CTL_NO_ACK)))
		return -EINVAL;
	short_preamble = !rate_ht &&
		(info->control.rates[0].flags & IEEE80211_TX_RC_USE_SHORT_PREAMBLE);
	short_preamble_fb = !rate_fb_ht && !rate_fb_ofdm &&
		rate_fb != B43_CCK_RATE_1MB &&
		(info->control.rates[info->control.rates[1].idx < 0 ? 0 : 1].flags &
		 IEEE80211_TX_RC_USE_SHORT_PREAMBLE);
	ack = !(info->flags & IEEE80211_TX_CTL_NO_ACK);

	if (rate_ht)
		txhdr->phy_rate = rate;
	else if (rate_ofdm)
		txhdr->phy_rate = b43_plcp_get_ratecode_ofdm(rate);
	else
		txhdr->phy_rate = b43_plcp_get_ratecode_cck(rate);
	txhdr->mac_frame_ctl = wlhdr->frame_control;
	memcpy(txhdr->tx_receiver, wlhdr->addr1, ETH_ALEN);

	/* Calculate duration for fallback rate */
	if (ht || (rate_fb == rate) ||
	    (wlhdr->duration_id & cpu_to_le16(0x8000)) ||
	    (wlhdr->duration_id == cpu_to_le16(0))) {
		/* If the fallback rate equals the normal rate or the
		 * dur_id field contains an AID, CFP magic or 0,
		 * use the original dur_id field. */
		txhdr->dur_fb = wlhdr->duration_id;
	} else {
		txhdr->dur_fb = ieee80211_generic_frame_duration(
			dev->wl->hw, info->control.vif, info->band,
			fragment_len, fbrate);
	}

	plcp_fragment_len = fragment_len + FCS_LEN;
	if (use_encryption) {
		u8 key_idx = info->control.hw_key->hw_key_idx;
		struct b43_key *key;
		int wlhdr_len;
		size_t iv_len;

		if (B43_WARN_ON(key_idx >= ARRAY_SIZE(dev->key)))
			return -ENOKEY;
		key = &(dev->key[key_idx]);

		if (unlikely(!key->keyconf)) {
			/* This key is invalid. This might only happen
			 * in a short timeframe after machine resume before
			 * we were able to reconfigure keys.
			 * Drop this packet completely. Do not transmit it
			 * unencrypted to avoid leaking information. */
			return -ENOKEY;
		}

		/* Hardware appends ICV. */
		plcp_fragment_len += info->control.hw_key->icv_len;

		key_idx = b43_kidx_to_fw(dev, key_idx);
		mac_ctl |= (key_idx << B43_TXH_MAC_KEYIDX_SHIFT) &
			   B43_TXH_MAC_KEYIDX;
		mac_ctl |= (key->algorithm << B43_TXH_MAC_KEYALG_SHIFT) &
			   B43_TXH_MAC_KEYALG;
		wlhdr_len = ieee80211_hdrlen(fctl);
		if (key->algorithm == B43_SEC_ALGO_TKIP) {
			u16 phase1key[5];
			int i;
			/* we give the phase1key and iv16 here, the key is stored in
			 * shm. With that the hardware can do phase 2 and encryption.
			 */
			ieee80211_get_tkip_p1k(info->control.hw_key, skb_frag, phase1key);
			/* phase1key is in host endian. Copy to little-endian txhdr->iv. */
			for (i = 0; i < 5; i++) {
				txhdr->iv[i * 2 + 0] = phase1key[i];
				txhdr->iv[i * 2 + 1] = phase1key[i] >> 8;
			}
			/* iv16 */
			memcpy(txhdr->iv + 10, ((u8 *) wlhdr) + wlhdr_len, 3);
		} else {
			iv_len = min_t(size_t, info->control.hw_key->iv_len,
				     ARRAY_SIZE(txhdr->iv));
			memcpy(txhdr->iv, ((u8 *) wlhdr) + wlhdr_len, iv_len);
		}
	}
	if (ht && !ieee80211_is_ctl(fctl)) {
		/* mac80211 does not export the next fragment's length. Reserve
		 * one full-sized fragment, an upper bound for the final one. */
		if (ieee80211_has_morefrags(fctl))
			next_len = plcp_fragment_len;
		wlhdr->duration_id = cpu_to_le16(ack ?
			b43_tx_nav(dev, info, rate, rate_ht, short_preamble,
				   next_len, ampdu) : 0);
		txhdr->dur_fb = cpu_to_le16(ack ?
			b43_tx_nav(dev, info, rate_fb, rate_fb_ht,
				   short_preamble_fb, next_len, ampdu) : 0);
	}
	/* The first header describes the whole A-MPDU PPDU. Continuation
	 * PLCP and mixed-mode lengths stay individual. */
	ppdu_len = ampdu && ampdu->first ? ampdu->len : plcp_fragment_len;
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_598.plcp),
				      ppdu_len, rate, rate_ht,
				      ampdu && ampdu->first);
		break;
	case B43_FW_HDR_351:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_351.plcp),
				      plcp_fragment_len, rate, rate_ht,
				      ampdu && ampdu->first);
		break;
	case B43_FW_HDR_410:
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->format_410.plcp),
				      plcp_fragment_len, rate, rate_ht,
				      ampdu && ampdu->first);
		break;
	}
	b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)(&txhdr->plcp_fb),
			      plcp_fragment_len, rate_fb, rate_fb_ht, false);
	if (ampdu && ampdu->first)
		txhdr->format_598.preload_size = cpu_to_le16(ampdu->preload);
	if (rate_ht)
		txhdr->mimo_modelen = cpu_to_le16(3 *
			(b43_ht20_symbols(rate, ppdu_len) + (rate >> 3) + 4) - 3);
	if (rate_fb_ht)
		txhdr->mimo_ratelen_fb = cpu_to_le16(3 *
			(b43_ht20_symbols(rate_fb, ppdu_len) + (rate_fb >> 3) + 4) - 3);
	if (ht) {
		fes = b43_tx_airtime(rate, rate_ht, short_preamble,
				     ppdu_len, phy->gmode) +
		      (ack ? b43_tx_nav(dev, info, rate, rate_ht, short_preamble,
					0, ampdu) : 0);
		fes_fb = b43_tx_airtime(rate_fb, rate_fb_ht, short_preamble_fb,
					ppdu_len, phy->gmode) +
			 (ack ? b43_tx_nav(dev, info, rate_fb, rate_fb_ht,
					  short_preamble_fb, 0, ampdu) : 0);
	}

	/* Extra Frame Types */
	if (rate_fb_ht)
		extra_ft |= B43_TXH_EFT_FB_HT;
	else if (rate_fb_ofdm)
		extra_ft |= B43_TXH_EFT_FB_OFDM;
	else
		extra_ft |= B43_TXH_EFT_FB_CCK;

	/* Set channel radio code. Note that the micrcode ORs 0x100 to
	 * this value before comparing it to the value in SHM, if this
	 * is a 5Ghz packet.
	 */
	txhdr->chan_radio_code = phy->channel;

	/* PHY TX Control word */
	if (rate_ht)
		phy_ctl |= B43_TXH_PHY_ENC_HT;
	else if (rate_ofdm)
		phy_ctl |= B43_TXH_PHY_ENC_OFDM;
	else
		phy_ctl |= B43_TXH_PHY_ENC_CCK;
	if (short_preamble)
		phy_ctl |= B43_TXH_PHY_SHORTPRMBL;

	switch (b43_ieee80211_antenna_sanitize(dev, 0)) {
	case 0: /* Default */
		phy_ctl |= B43_TXH_PHY_ANT01AUTO;
		break;
	case 1: /* Antenna 0 */
		phy_ctl |= B43_TXH_PHY_ANT0;
		break;
	case 2: /* Antenna 1 */
		phy_ctl |= B43_TXH_PHY_ANT1;
		break;
	case 3: /* Antenna 2 */
		phy_ctl |= B43_TXH_PHY_ANT2;
		break;
	case 4: /* Antenna 3 */
		phy_ctl |= B43_TXH_PHY_ANT3;
		break;
	default:
		B43_WARN_ON(1);
	}

	rates = info->control.rates;
	if (ht && protect && vif && ack) {
		u16 ht_prot = vif->bss_conf.ht_operation_mode &
			      IEEE80211_HT_OP_MODE_PROTECTION;

		/* 20 MHz protection only covers 40 MHz PPDUs, which 20 MHz-only
		 * HT stations cannot receive. */
		if (vif->bss_conf.use_cts_prot ||
		    ht_prot == IEEE80211_HT_OP_MODE_PROTECTION_NONMEMBER ||
		    ht_prot == IEEE80211_HT_OP_MODE_PROTECTION_NONHT_MIXED)
			rates[0].flags |= IEEE80211_TX_RC_USE_RTS_CTS;
	}
	/* As brcmsmac does, protect an HT PPDU when any retry stage asks for
	 * RTS/CTS. minstrel_ht asks on its fallback stages, so every A-MPDU
	 * starts with an RTS; unprotected A-MPDUs raise the PSM watchdog more
	 * often.
	 */
	if (ht && protect && ack) {
		int i;

		for (i = 1; i < IEEE80211_TX_MAX_RATES && rates[i].idx >= 0; i++)
			if (rates[i].flags & IEEE80211_TX_RC_USE_RTS_CTS)
				rates[0].flags |= IEEE80211_TX_RC_USE_RTS_CTS;
	}
	/* MAC control */
	if (!(info->flags & IEEE80211_TX_CTL_NO_ACK))
		mac_ctl |= B43_TXH_MAC_ACK;
	/* use hardware sequence counter as the non-TID counter */
	if (info->flags & IEEE80211_TX_CTL_ASSIGN_SEQ)
		mac_ctl |= B43_TXH_MAC_HWSEQ;
	if ((info->flags & IEEE80211_TX_CTL_FIRST_FRAGMENT) &&
	    (!ampdu || ampdu->first))
		mac_ctl |= B43_TXH_MAC_STMSDU;
	if (ampdu && ampdu->last)
		mac_ctl |= B43_TXH_MAC_AMPDU_LAST;
	else if (ampdu && ampdu->first)
		mac_ctl |= B43_TXH_MAC_AMPDU_FIRST;
	else if (ampdu)
		mac_ctl |= B43_TXH_MAC_AMPDU_INTER;
	if (!phy->gmode)
		mac_ctl |= B43_TXH_MAC_5GHZ;
	if (short_preamble_fb)
		mac_ctl |= B43_TXH_MAC_FB_SHORTPRMBL;

	/* Overwrite rates[0].count to make the retry calculation
	 * in the tx status easier. need the actual retry limit to
	 * detect whether the fallback rate was used.
	 */
	if ((rates[0].flags & IEEE80211_TX_RC_USE_RTS_CTS) ||
	    (rates[0].count <= dev->wl->hw->conf.long_frame_max_tx_count)) {
		rates[0].count = B43_LONG_FALLBACK_LIMIT;
		mac_ctl |= B43_TXH_MAC_LONGFRAME;
	} else {
		rates[0].count = B43_SHORT_FALLBACK_LIMIT;
	}

	/* Generate the RTS or CTS-to-self frame. Only the first A-MPDU
	 * member's protection precedes the PPDU. */
	if (protect &&
	    ((rates[0].flags & IEEE80211_TX_RC_USE_RTS_CTS) ||
	     (rates[0].flags & IEEE80211_TX_RC_USE_CTS_PROTECT))) {
		unsigned int len;
		struct ieee80211_hdr *hdr;
		int rts_rate, rts_rate_fb;
		int rts_rate_ofdm, rts_rate_fb_ofdm;
		struct b43_plcp_hdr6 *plcp;
		struct ieee80211_rate *rts_cts_rate;

		rts_cts_rate = ieee80211_get_rts_cts_rate(dev->wl->hw, info);

		/* mac80211 selects no protection rate for MCS frames. */
		if (ht)
			rts_rate = b43_tx_response_rate(dev, info, rate, rate_ht, true);
		else
			rts_rate = rts_cts_rate ? rts_cts_rate->hw_value : B43_CCK_RATE_1MB;
		rts_rate_ofdm = b43_is_ofdm_rate(rts_rate);
		if (ht)
			rts_rate_fb = b43_tx_response_rate(dev, info, rate_fb,
							   rate_fb_ht, true);
		else
			rts_rate_fb = b43_calc_fallback_rate(rts_rate, phy->gmode);
		rts_rate_fb_ofdm = b43_is_ofdm_rate(rts_rate_fb);

		if (rates[0].flags & IEEE80211_TX_RC_USE_CTS_PROTECT) {
			struct ieee80211_cts *cts;

			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				cts = (struct ieee80211_cts *)
					(txhdr->format_598.rts_frame);
				break;
			case B43_FW_HDR_351:
				cts = (struct ieee80211_cts *)
					(txhdr->format_351.rts_frame);
				break;
			case B43_FW_HDR_410:
				cts = (struct ieee80211_cts *)
					(txhdr->format_410.rts_frame);
				break;
			}
			if (ht) {
				cts->frame_control = cpu_to_le16(IEEE80211_FTYPE_CTL |
							       IEEE80211_STYPE_CTS);
				memcpy(cts->ra, wlhdr->addr2, ETH_ALEN);
			} else {
				ieee80211_ctstoself_get(dev->wl->hw, info->control.vif,
							fragment_data, fragment_len,
							info, cts);
			}
			mac_ctl |= B43_TXH_MAC_SENDCTS;
			len = sizeof(struct ieee80211_cts);
		} else {
			struct ieee80211_rts *rts;

			switch (dev->fw.hdr_format) {
			case B43_FW_HDR_598:
				rts = (struct ieee80211_rts *)
					(txhdr->format_598.rts_frame);
				break;
			case B43_FW_HDR_351:
				rts = (struct ieee80211_rts *)
					(txhdr->format_351.rts_frame);
				break;
			case B43_FW_HDR_410:
				rts = (struct ieee80211_rts *)
					(txhdr->format_410.rts_frame);
				break;
			}
			if (ht) {
				rts->frame_control = cpu_to_le16(IEEE80211_FTYPE_CTL |
							       IEEE80211_STYPE_RTS);
				memcpy(rts->ra, wlhdr->addr1, ETH_ALEN);
				memcpy(rts->ta, wlhdr->addr2, ETH_ALEN);
			} else {
				ieee80211_rts_get(dev->wl->hw, info->control.vif,
						  fragment_data, fragment_len,
						  info, rts);
			}
			mac_ctl |= B43_TXH_MAC_SENDRTS;
			len = sizeof(struct ieee80211_rts);
		}
		len += FCS_LEN;

		/* Generate the PLCP headers for the RTS/CTS frame */
		switch (dev->fw.hdr_format) {
		case B43_FW_HDR_598:
			plcp = &txhdr->format_598.rts_plcp;
			break;
		case B43_FW_HDR_351:
			plcp = &txhdr->format_351.rts_plcp;
			break;
		case B43_FW_HDR_410:
			plcp = &txhdr->format_410.rts_plcp;
			break;
		}
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)plcp,
				      len, rts_rate, false, false);
		plcp = &txhdr->rts_plcp_fb;
		b43_generate_plcp_hdr((struct b43_plcp_hdr4 *)plcp,
				      len, rts_rate_fb, false, false);

		switch (dev->fw.hdr_format) {
		case B43_FW_HDR_598:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_598.rts_frame);
			break;
		case B43_FW_HDR_351:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_351.rts_frame);
			break;
		case B43_FW_HDR_410:
			hdr = (struct ieee80211_hdr *)
				(&txhdr->format_410.rts_frame);
			break;
		}
		txhdr->rts_dur_fb = hdr->duration_id;
		if (ht) {
			/* The HT RTS/CTS frames above carry no duration yet. */
			bool rts_short_preamble =
				b43_tx_response_short_preamble(vif, rts_rate);
			bool rts_short_preamble_fb =
				b43_tx_response_short_preamble(vif, rts_rate_fb);
			unsigned int sifs = phy->gmode ? 10 : 16;
			unsigned int nav = sifs + fes;
			unsigned int nav_fb = sifs + fes_fb;

			if (rts_short_preamble)
				mac_ctl |= B43_TXH_MAC_RTS_SHORTPRMBL;
			if (rts_short_preamble_fb)
				mac_ctl |= B43_TXH_MAC_RTS_FB_SHORTPRMBL;
			if (!(rates[0].flags & IEEE80211_TX_RC_USE_CTS_PROTECT)) {
				nav += sifs + b43_tx_airtime(rts_rate, false,
							rts_short_preamble, 14, phy->gmode);
				nav_fb += sifs + b43_tx_airtime(rts_rate_fb, false,
							rts_short_preamble_fb, 14, phy->gmode);
			}
			hdr->duration_id = cpu_to_le16(nav);
			txhdr->rts_dur_fb = cpu_to_le16(nav_fb);
			fes = nav + b43_tx_airtime(rts_rate, false,
						  rts_short_preamble, len, phy->gmode);
			fes_fb = nav_fb + b43_tx_airtime(rts_rate_fb, false,
							rts_short_preamble_fb, len, phy->gmode);
		}

		if (rts_rate_ofdm) {
			extra_ft |= B43_TXH_EFT_RTS_OFDM;
			txhdr->phy_rate_rts =
			    b43_plcp_get_ratecode_ofdm(rts_rate);
		} else {
			extra_ft |= B43_TXH_EFT_RTS_CCK;
			txhdr->phy_rate_rts =
			    b43_plcp_get_ratecode_cck(rts_rate);
		}
		if (rts_rate_fb_ofdm)
			extra_ft |= B43_TXH_EFT_RTSFB_OFDM;
		else
			extra_ft |= B43_TXH_EFT_RTSFB_CCK;

		if (fill_phy_ctl1) {
			txhdr->phy_ctl1_rts = cpu_to_le16(
				b43_generate_tx_phy_ctl1(dev, rts_rate, false));
			txhdr->phy_ctl1_rts_fb = cpu_to_le16(
				b43_generate_tx_phy_ctl1(dev, rts_rate_fb, false));
		}
	}
	/* brcmsmac's AMPDU_FBR_NULL_DELIM; a legacy RTS fallback PLCP only
	 * uses bytes 0-3. */
	if (ampdu)
		txhdr->rts_plcp_fb.raw[5] = ampdu->ndelim;
	if (ht && ieee80211_is_data_qos(fctl) &&
	    skb_get_queue_mapping(skb_frag) < B43_QOS_QUEUE_NUM &&
	    dev->wl->qos_params[skb_get_queue_mapping(skb_frag)].p.txop) {
		txhdr->tx_fes_time_norm = cpu_to_le16(fes);
		txhdr->tx_fes_time_fb = cpu_to_le16(fes_fb);
	}

	/* Magic cookie */
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		txhdr->format_598.cookie = cpu_to_le16(cookie);
		break;
	case B43_FW_HDR_351:
		txhdr->format_351.cookie = cpu_to_le16(cookie);
		break;
	case B43_FW_HDR_410:
		txhdr->format_410.cookie = cpu_to_le16(cookie);
		break;
	}

	if (fill_phy_ctl1) {
		txhdr->phy_ctl1 =
			cpu_to_le16(b43_generate_tx_phy_ctl1(dev, rate, rate_ht));
		txhdr->phy_ctl1_fb =
			cpu_to_le16(b43_generate_tx_phy_ctl1(dev, rate_fb, rate_fb_ht));
	}

	/* Apply the bitfields */
	txhdr->mac_ctl = cpu_to_le32(mac_ctl);
	txhdr->phy_ctl = cpu_to_le16(phy_ctl);
	txhdr->extra_ft = extra_ft;

	return 0;
}

/* On-air PSDU length of a TX frame: hardware appends ICV and FCS. */
unsigned int b43_tx_psdu_len(struct sk_buff *skb)
{
	struct ieee80211_tx_info *info = IEEE80211_SKB_CB(skb);
	unsigned int len = skb->len + FCS_LEN;

	if (info->control.hw_key)
		len += info->control.hw_key->icv_len;
	return len;
}

/* The mixed-mode L-SIG length field limits the HT PPDU duration. */
bool b43_ampdu_tx_fits(struct b43_wldev *dev, struct ieee80211_tx_info *info,
		       unsigned int ampdu_len)
{
	u8 mcs = info->control.rates[0].idx;

	return 3 * (b43_ht20_symbols(mcs, ampdu_len) + (mcs >> 3) + 4) - 3 <= 4095;
}

/* Minimum bytes from one MPDU start to the next for the peer's density
 * code: spacing 2^(density - 3) microseconds, none for zero. */
unsigned int b43_ampdu_tx_spacing(struct b43_wldev *dev,
				  struct ieee80211_tx_info *info, u8 density)
{
	u8 mcs = info->control.rates[0].idx;

	if (!density)
		return 0;
	return DIV_ROUND_UP((1 << density) * b43_ht20_ndbps[mcs & 7] *
			    (1 + (mcs >> 3)), 8 * 8 * 4);
}

static s8 b43_rssi_postprocess(struct b43_wldev *dev,
			       u8 in_rssi, int ofdm,
			       int adjust_2053, int adjust_2050)
{
	struct b43_phy *phy = &dev->phy;
	struct b43_phy_g *gphy = phy->g;
	s32 tmp;

	switch (phy->radio_ver) {
	case 0x2050:
		if (ofdm) {
			tmp = in_rssi;
			if (tmp > 127)
				tmp -= 256;
			tmp *= 73;
			tmp /= 64;
			if (adjust_2050)
				tmp += 25;
			else
				tmp -= 3;
		} else {
			if (dev->dev->bus_sprom->
			    boardflags_lo & B43_BFL_RSSI) {
				if (in_rssi > 63)
					in_rssi = 63;
				B43_WARN_ON(phy->type != B43_PHYTYPE_G);
				tmp = gphy->nrssi_lt[in_rssi];
				tmp = 31 - tmp;
				tmp *= -131;
				tmp /= 128;
				tmp -= 57;
			} else {
				tmp = in_rssi;
				tmp = 31 - tmp;
				tmp *= -149;
				tmp /= 128;
				tmp -= 68;
			}
			if (phy->type == B43_PHYTYPE_G && adjust_2050)
				tmp += 25;
		}
		break;
	case 0x2060:
		if (in_rssi > 127)
			tmp = in_rssi - 256;
		else
			tmp = in_rssi;
		break;
	default:
		tmp = in_rssi;
		tmp -= 11;
		tmp *= 103;
		tmp /= 64;
		if (adjust_2053)
			tmp -= 109;
		else
			tmp -= 83;
	}

	return (s8) tmp;
}

/* x^8 + x^2 + x + 1, bit-reversed */
#define B43_HT_SIG_CRC_POLY	0xe0

DECLARE_CRC8_TABLE(b43_ht_sig_crc_table);

void b43_ht_sig_crc_init(void)
{
	crc8_populate_lsb(b43_ht_sig_crc_table, B43_HT_SIG_CRC_POLY);
}

/* HT-SIG: 34 data bits, complemented CRC-8, then six zero tail bits */
static bool b43_ht_sig_crc_valid(const u8 *plcp)
{
	u8 crc = crc8(b43_ht_sig_crc_table, plcp, 4, CRC8_INIT_VALUE);
	int i;

	crc ^= plcp[4] & 3;
	for (i = 0; i < 2; i++)
		crc = (crc >> 1) ^ ((crc & 1) ? B43_HT_SIG_CRC_POLY : 0);
	return (u8)~crc == ((plcp[4] >> 2) | (plcp[5] << 6));
}

static const u8 *b43_rx_ht_plcp(struct b43_wldev *dev, const u8 *plcp,
			       u32 macstat, u16 mactime, u16 channel)
{
	u64 now;
	u32 tsf;

	if (get_unaligned_le32(plcp) || get_unaligned_le16(plcp + 4)) {
		dev->rx_ht_valid = false;
		if ((plcp[0] & 0x7f) > 15)
			return NULL;
		if (!(macstat & B43_RX_MAC_RXST_VALID) &&
		    !b43_ht_sig_crc_valid(plcp))
			return NULL;
		if ((plcp[3] & 0x08) && !(macstat & B43_RX_MAC_RXST_VALID)) {
			now = ktime_get_ns();
			tsf = b43_read32(dev, B43_MMIO_REV3PLUS_TSF_LOW);
			memcpy(dev->rx_ht_plcp, plcp, sizeof(dev->rx_ht_plcp));
			dev->rx_ht_tsf = tsf - (u16)(tsf - mactime);
			dev->rx_ht_channel = channel;
			dev->rx_ht_deadline = now + 64 * NSEC_PER_MSEC;
			dev->rx_ht_valid = true;
		}
		return plcp;
	}

	if (!dev->rx_ht_valid || (u16)dev->rx_ht_tsf != mactime ||
	    dev->rx_ht_channel != channel ||
	    ktime_get_ns() >= dev->rx_ht_deadline)
		goto invalid;
	/* Resolve the TSF epoch too: equal low 16 bits can belong to a later
	 * PPDU even when an older first member was processed only recently. */
	tsf = b43_read32(dev, B43_MMIO_REV3PLUS_TSF_LOW);
	if (tsf - (u16)(tsf - mactime) != dev->rx_ht_tsf)
		goto invalid;
	if (macstat & B43_RX_MAC_RXST_VALID)
		dev->rx_ht_valid = false;
	return dev->rx_ht_plcp;

invalid:
	dev->rx_ht_valid = false;
	return NULL;
}

void b43_rx(struct b43_wldev *dev, struct sk_buff *skb, const void *_rxhdr)
{
	struct ieee80211_rx_status status;
	struct b43_plcp_hdr6 *plcp;
	const u8 *ht_plcp = NULL;
	struct ieee80211_hdr *wlhdr;
	const struct b43_rxhdr_fw4 *rxhdr = _rxhdr;
	__le16 fctl;
	u16 phystat0, phystat3;
	u16 chanstat, mactime;
	u32 macstat;
	u16 chanid;
	int padding, rate_idx;

	memset(&status, 0, sizeof(status));

	/* Get metadata about the frame from the header. */
	phystat0 = le16_to_cpu(rxhdr->phy_status0);
	phystat3 = le16_to_cpu(rxhdr->phy_status3);
	switch (dev->fw.hdr_format) {
	case B43_FW_HDR_598:
		macstat = le32_to_cpu(rxhdr->format_598.mac_status);
		mactime = le16_to_cpu(rxhdr->format_598.mac_time);
		chanstat = le16_to_cpu(rxhdr->format_598.channel);
		break;
	case B43_FW_HDR_410:
	case B43_FW_HDR_351:
		macstat = le32_to_cpu(rxhdr->format_351.mac_status);
		mactime = le16_to_cpu(rxhdr->format_351.mac_time);
		chanstat = le16_to_cpu(rxhdr->format_351.channel);
		break;
	}
	/* brcmsmac treats frame type 3 as an error, and the BCM43227 was
	 * not seen to report it.
	 */
	if (dev->ht &&
	    (phystat0 & B43_RX_PHYST0_FTYPE) == B43_RX_PHYST0_STD_N)
		goto drop;
	if (dev->ht && !(phystat0 & B43_RX_PHYST0_PRE_N))
		dev->rx_ht_valid = false;

	if (unlikely(macstat & B43_RX_MAC_FCSERR)) {
		dev->wl->ieee_stats.dot11FCSErrorCount++;
		status.flag |= RX_FLAG_FAILED_FCS_CRC;
	}
	if (unlikely(phystat0 & (B43_RX_PHYST0_PLCPHCF | B43_RX_PHYST0_PLCPFV)))
		status.flag |= RX_FLAG_FAILED_PLCP_CRC;
	if (phystat0 & B43_RX_PHYST0_SHORTPRMBL)
		status.enc_flags |= RX_ENC_FLAG_SHORTPRE;
	if (dev->ht && (phystat0 & B43_RX_PHYST0_PRE_N)) {
		if (!(macstat & B43_RX_MAC_RXST_VALID))
			status.flag &= ~RX_FLAG_FAILED_PLCP_CRC;
		padding = (macstat & B43_RX_MAC_PADDING) ? 2 : 0;
		if (dev->fw.hdr_format == B43_FW_HDR_598 &&
		    !(status.flag & RX_FLAG_FAILED_PLCP_CRC) &&
		    skb->len >= sizeof(struct b43_plcp_hdr6) + padding)
			ht_plcp = b43_rx_ht_plcp(dev, skb->data + padding,
						macstat, mactime, chanstat);
	}
	if (macstat & B43_RX_MAC_DECERR) {
		/* Decryption with the given key failed.
		 * Drop the packet. We also won't be able to decrypt it with
		 * the key in software. */
		goto drop;
	}

	/* Skip PLCP and padding */
	padding = (macstat & B43_RX_MAC_PADDING) ? 2 : 0;
	if (unlikely(skb->len < (sizeof(struct b43_plcp_hdr6) + padding))) {
		b43dbg(dev->wl, "RX: Packet size underrun (1)\n");
		goto drop;
	}
	plcp = (struct b43_plcp_hdr6 *)(skb->data + padding);
	skb_pull(skb, sizeof(struct b43_plcp_hdr6) + padding);
	/* The skb contains the Wireless Header + payload data now */
	if (unlikely(skb->len < (2 + 2 + 6 /*minimum hdr */  + FCS_LEN))) {
		b43dbg(dev->wl, "RX: Packet size underrun (2)\n");
		goto drop;
	}
	wlhdr = (struct ieee80211_hdr *)(skb->data);
	fctl = wlhdr->frame_control;

	if (macstat & B43_RX_MAC_DEC) {
		unsigned int keyidx;
		int wlhdr_len;

		keyidx = ((macstat & B43_RX_MAC_KEYIDX)
			  >> B43_RX_MAC_KEYIDX_SHIFT);
		/* We must adjust the key index here. We want the "physical"
		 * key index, but the ucode passed it slightly different.
		 */
		keyidx = b43_kidx_to_raw(dev, keyidx);
		if (B43_WARN_ON(keyidx >= ARRAY_SIZE(dev->key)))
			goto drop;

		if (dev->key[keyidx].algorithm != B43_SEC_ALGO_NONE) {
			wlhdr_len = ieee80211_hdrlen(fctl);
			if (unlikely(skb->len < (wlhdr_len + 3))) {
				b43dbg(dev->wl,
				       "RX: Packet size underrun (3)\n");
				goto drop;
			}
			status.flag |= RX_FLAG_DECRYPTED;
		}
	}

	/* Link quality statistics */
	switch (chanstat & B43_RX_CHAN_PHYTYPE) {
	case B43_PHYTYPE_HT:
		/* TODO: is max the right choice? */
		status.signal = max_t(__s8,
			max(rxhdr->phy_ht_power0, rxhdr->phy_ht_power1),
			rxhdr->phy_ht_power2);
		break;
	case B43_PHYTYPE_N:
		/* Broadcom has code for min and avg, but always uses max */
		if (rxhdr->power0 == 16 || rxhdr->power0 == 32)
			status.signal = max(rxhdr->power1, rxhdr->power2);
		else
			status.signal = max(rxhdr->power0, rxhdr->power1);
		break;
	case B43_PHYTYPE_B:
	case B43_PHYTYPE_G:
	case B43_PHYTYPE_LP:
		status.signal = b43_rssi_postprocess(dev, rxhdr->jssi,
						  (phystat0 & B43_RX_PHYST0_OFDM),
						  (phystat0 & B43_RX_PHYST0_GAINCTL),
						  (phystat3 & B43_RX_PHYST3_TRSTATE));
		break;
	}

	if (dev->ht && (phystat0 & B43_RX_PHYST0_PRE_N)) {
		/* Type 2 is HT on N-PHY, not a legacy CCK rate code. A
		 * failed HT-SIG cannot provide trustworthy MCS metadata. */
		if (!ht_plcp)
			goto drop;
		rate_idx = ht_plcp[0] & 0x7f;
		if (!(macstat & B43_RX_MAC_RXST_VALID))
			status.flag |= RX_FLAG_NO_SIGNAL_VAL;
		status.encoding = RX_ENC_HT;
		status.enc_flags &= ~RX_ENC_FLAG_SHORTPRE;
		status.bw = (ht_plcp[0] & 0x80) ?
			RATE_INFO_BW_40 : RATE_INFO_BW_20;
		if (ht_plcp[3] & 0x80)
			status.enc_flags |= RX_ENC_FLAG_SHORT_GI;
		if (ht_plcp[3] & 0x40)
			status.enc_flags |= RX_ENC_FLAG_LDPC;
		status.enc_flags |= ((ht_plcp[3] >> 4) & 3) <<
				    RX_ENC_FLAG_STBC_SHIFT;
	} else if (phystat0 & B43_RX_PHYST0_OFDM)
		rate_idx = b43_plcp_get_bitrate_idx_ofdm(plcp,
					!!(chanstat & B43_RX_CHAN_5GHZ));
	else
		rate_idx = b43_plcp_get_bitrate_idx_cck(plcp);
	if (unlikely(rate_idx == -1)) {
		/* PLCP seems to be corrupted.
		 * Drop the frame, if we are not interested in corrupted frames. */
		if (!(dev->wl->filter_flags & FIF_PLCPFAIL) ||
		    !(status.flag & RX_FLAG_FAILED_PLCP_CRC))
			goto drop;
	}
	status.rate_idx = rate_idx;
	status.antenna = !!(phystat0 & B43_RX_PHYST0_ANT);

	/*
	 * All frames on monitor interfaces and beacons always need a full
	 * 64-bit timestamp. Monitor interfaces need it for diagnostic
	 * purposes and beacons for IBSS merging.
	 * This code assumes we get to process the packet within 16 bits
	 * of timestamp, i.e. about 65 milliseconds after the PHY received
	 * the first symbol.
	 */
	if (ieee80211_is_beacon(fctl) || dev->wl->radiotap_enabled) {
		u16 low_mactime_now;

		b43_tsf_read(dev, &status.mactime);
		low_mactime_now = status.mactime;
		status.mactime = status.mactime & ~0xFFFFULL;
		status.mactime += mactime;
		if (low_mactime_now <= mactime)
			status.mactime -= 0x10000;
		status.flag |= RX_FLAG_MACTIME_START;
	}

	chanid = (chanstat & B43_RX_CHAN_ID) >> B43_RX_CHAN_ID_SHIFT;
	switch (chanstat & B43_RX_CHAN_PHYTYPE) {
	case B43_PHYTYPE_G:
		status.band = NL80211_BAND_2GHZ;
		/* Somewhere between 478.104 and 508.1084 firmware for G-PHY
		 * has been modified to be compatible with N-PHY and others.
		 */
		if (dev->fw.rev >= 508)
			status.freq = ieee80211_channel_to_frequency(chanid, status.band);
		else
			status.freq = chanid + 2400;
		break;
	case B43_PHYTYPE_N:
	case B43_PHYTYPE_LP:
	case B43_PHYTYPE_HT:
		/* chanid is the SHM channel cookie. Which is the plain
		 * channel number in b43. */
		if (chanstat & B43_RX_CHAN_5GHZ)
			status.band = NL80211_BAND_5GHZ;
		else
			status.band = NL80211_BAND_2GHZ;
		status.freq =
			ieee80211_channel_to_frequency(chanid, status.band);
		break;
	default:
		B43_WARN_ON(1);
		goto drop;
	}

	memcpy(IEEE80211_SKB_RXCB(skb), &status, sizeof(status));
	ieee80211_rx_ni(dev->wl->hw, skb);

#if B43_DEBUG
	dev->rx_count++;
#endif
	return;
drop:
	if (!ht_plcp)
		dev->rx_ht_valid = false;
	dev_kfree_skb_any(skb);
}

void b43_handle_txstatus(struct b43_wldev *dev,
			 const struct b43_txstatus *status)
{
	b43_debugfs_log_txstat(dev, status);

	if (status->for_ampdu) {
		b43_dma_handle_ampdu_txstatus(dev, status);
		return;
	}
	if (status->intermediate)
		return;
	if (!status->acked)
		dev->wl->ieee_stats.dot11ACKFailureCount++;
	if (status->rts_count) {
		if (status->rts_count == 0xF)	//FIXME
			dev->wl->ieee_stats.dot11RTSFailureCount++;
		else
			dev->wl->ieee_stats.dot11RTSSuccessCount++;
	}

	if (b43_using_pio_transfers(dev))
		b43_pio_handle_txstatus(dev, status);
	else
		b43_dma_handle_txstatus(dev, status);

	b43_phy_txpower_check(dev, 0);
}

/* Fill out the mac80211 TXstatus report based on the b43-specific
 * txstatus report data. This returns a boolean whether the frame was
 * successfully transmitted. */
bool b43_fill_txstatus_report(struct b43_wldev *dev,
			      struct ieee80211_tx_info *report,
			      const struct b43_txstatus *status)
{
	bool frame_success = true;
	int retry_limit;

	/* preserve the confiured retry limit before clearing the status
	 * The xmit function has overwritten the rc's value with the actual
	 * retry limit done by the hardware */
	retry_limit = report->status.rates[0].count;
	ieee80211_tx_info_clear_status(report);

	if (status->acked) {
		/* The frame was ACKed. */
		report->flags |= IEEE80211_TX_STAT_ACK;
	} else {
		/* The frame was not ACKed... */
		if (!(report->flags & IEEE80211_TX_CTL_NO_ACK)) {
			/* ...but we expected an ACK. */
			frame_success = false;
		}
	}
	if (status->frame_count == 0) {
		/* The frame was not transmitted at all. */
		report->status.rates[0].count = 0;
	} else if (status->rts_count > dev->wl->hw->conf.short_frame_max_tx_count) {
		/*
		 * If the short retries (RTS, not data frame) have exceeded
		 * the limit, the hw will not have tried the selected rate,
		 * but will have used the fallback rate instead.
		 * Don't let the rate control count attempts for the selected
		 * rate in this case, otherwise the statistics will be off.
		 */
		report->status.rates[0].count = 0;
		report->status.rates[1].count = status->frame_count;
	} else {
		if (status->frame_count > retry_limit &&
		    report->status.rates[1].idx >= 0) {
			report->status.rates[0].count = retry_limit;
			report->status.rates[1].count = status->frame_count -
					retry_limit;

		} else {
			report->status.rates[0].count = status->frame_count;
			report->status.rates[1].idx = -1;
		}
	}

	return frame_success;
}

/* Stop any TX operation on the device (suspend the hardware queues) */
void b43_tx_suspend(struct b43_wldev *dev)
{
	if (b43_using_pio_transfers(dev))
		b43_pio_tx_suspend(dev);
	else
		b43_dma_tx_suspend(dev);
}

/* Resume any TX operation on the device (resume the hardware queues) */
void b43_tx_resume(struct b43_wldev *dev)
{
	if (b43_using_pio_transfers(dev))
		b43_pio_tx_resume(dev);
	else
		b43_dma_tx_resume(dev);
}
