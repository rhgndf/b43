// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Broadcom B43 wireless driver
 * PPR (Power Per Rate) management
 *
 * Copyright (c) 2014 Rafał Miłecki <zajec5@gmail.com>
 */

#include "ppr.h"
#include "b43.h"

#define ppr_for_each_entry(ppr, i, entry)				\
	for (i = 0, entry = &(ppr)->__all_rates[i];			\
	     i < B43_PPR_RATES_NUM;					\
	     i++, entry++)

void b43_ppr_clear(struct b43_wldev *dev, struct b43_ppr *ppr)
{
	memset(ppr, 0, sizeof(*ppr));

	/* Compile-time PPR check */
	BUILD_BUG_ON(sizeof(struct b43_ppr) != B43_PPR_RATES_NUM * sizeof(u8));
}

void b43_ppr_add(struct b43_wldev *dev, struct b43_ppr *ppr, int diff)
{
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		*rate = clamp_val(*rate + diff, 0, 127);
	}
}

void b43_ppr_apply_max(struct b43_wldev *dev, struct b43_ppr *ppr, u8 max)
{
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		*rate = min(*rate, max);
	}
}

void b43_ppr_apply_min(struct b43_wldev *dev, struct b43_ppr *ppr, u8 min)
{
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		*rate = max(*rate, min);
	}
}

u8 b43_ppr_get_max(struct b43_wldev *dev, struct b43_ppr *ppr)
{
	u8 res = 0;
	int i;
	u8 *rate;

	ppr_for_each_entry(ppr, i, rate) {
		res = max(*rate, res);
	}

	return res;
}

/* OFDM 6 and 9 Mbit/s take MCS 0's power, 12-54 Mbit/s MCS 1-6's */
static void b43_ppr_mcs_to_ofdm(u8 *ofdm, const u8 *mcs)
{
	int i;

	ofdm[0] = mcs[0];
	for (i = 1; i < B43_PPR_OFDM_RATES_NUM; i++)
		ofdm[i] = mcs[i - 1];
}

bool b43_ppr_load_max_from_sprom(struct b43_wldev *dev, struct b43_ppr *ppr,
				 enum b43_band band)
{
	struct b43_ppr_rates *rates = &ppr->rates;
	struct ssb_sprom *sprom = dev->dev->bus_sprom;
	struct b43_phy *phy = &dev->phy;
	u8 maxpwr, off;
	u32 sprom_ofdm_po;
	u16 *sprom_mcs_po;
	u8 extra_cdd_po, extra_stbc_po, extra_bw40_po;
	int i;

	switch (band) {
	case B43_BAND_2G:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_2g,
			     sprom->core_pwr_info[1].maxpwr_2g);
		sprom_ofdm_po = sprom->ofdm2gpo;
		sprom_mcs_po = sprom->mcs2gpo;
		extra_cdd_po = (sprom->cddpo >> 0) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 0) & 0xf;
		extra_bw40_po = (sprom->bw40po >> 0) & 0xf;
		break;
	case B43_BAND_5G_LO:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_5gl,
			     sprom->core_pwr_info[1].maxpwr_5gl);
		sprom_ofdm_po = sprom->ofdm5glpo;
		sprom_mcs_po = sprom->mcs5glpo;
		extra_cdd_po = (sprom->cddpo >> 8) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 8) & 0xf;
		extra_bw40_po = (sprom->bw40po >> 8) & 0xf;
		break;
	case B43_BAND_5G_MI:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_5g,
			     sprom->core_pwr_info[1].maxpwr_5g);
		sprom_ofdm_po = sprom->ofdm5gpo;
		sprom_mcs_po = sprom->mcs5gpo;
		extra_cdd_po = (sprom->cddpo >> 4) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 4) & 0xf;
		extra_bw40_po = (sprom->bw40po >> 4) & 0xf;
		break;
	case B43_BAND_5G_HI:
		maxpwr = min(sprom->core_pwr_info[0].maxpwr_5gh,
			     sprom->core_pwr_info[1].maxpwr_5gh);
		sprom_ofdm_po = sprom->ofdm5ghpo;
		sprom_mcs_po = sprom->mcs5ghpo;
		extra_cdd_po = (sprom->cddpo >> 12) & 0xf;
		extra_stbc_po = (sprom->stbcpo >> 12) & 0xf;
		extra_bw40_po = (sprom->bw40po >> 12) & 0xf;
		break;
	default:
		WARN_ON_ONCE(1);
		return false;
	}

	if (band == B43_BAND_2G) {
		for (i = 0; i < 4; i++) {
			off = ((sprom->cck2gpo >> (i * 4)) & 0xf) * 2;
			rates->cck[i] = maxpwr - off;
		}
	}

	/* OFDM */
	for (i = 0; i < 8; i++) {
		off = ((sprom_ofdm_po >> (i * 4)) & 0xf) * 2;
		rates->ofdm[i] = maxpwr - off;
	}

	/* MCS 20 SISO */
	rates->mcs_20[0] = rates->ofdm[0];
	rates->mcs_20[1] = rates->ofdm[2];
	rates->mcs_20[2] = rates->ofdm[3];
	rates->mcs_20[3] = rates->ofdm[4];
	rates->mcs_20[4] = rates->ofdm[5];
	rates->mcs_20[5] = rates->ofdm[6];
	rates->mcs_20[6] = rates->ofdm[7];
	rates->mcs_20[7] = rates->ofdm[7];

	/* MCS 20 CDD */
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[0] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_cdd[i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_cdd[i] -= extra_cdd_po;
	}
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[1] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_cdd[4 + i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_cdd[4 + i] -= extra_cdd_po;
	}

	/* OFDM 20 CDD */
	rates->ofdm_20_cdd[0] = rates->mcs_20_cdd[0];
	rates->ofdm_20_cdd[1] = rates->mcs_20_cdd[0];
	rates->ofdm_20_cdd[2] = rates->mcs_20_cdd[1];
	rates->ofdm_20_cdd[3] = rates->mcs_20_cdd[2];
	rates->ofdm_20_cdd[4] = rates->mcs_20_cdd[3];
	rates->ofdm_20_cdd[5] = rates->mcs_20_cdd[4];
	rates->ofdm_20_cdd[6] = rates->mcs_20_cdd[5];
	rates->ofdm_20_cdd[7] = rates->mcs_20_cdd[6];

	/* MCS 20 STBC */
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[0] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_stbc[i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_stbc[i] -= extra_stbc_po;
	}
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[1] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_stbc[4 + i] = maxpwr - off;
		if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
			rates->mcs_20_stbc[4 + i] -= extra_stbc_po;
	}

	/* MCS 20 SDM */
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[2] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_sdm[i] = maxpwr - off;
	}
	for (i = 0; i < 4; i++) {
		off = ((sprom_mcs_po[3] >> (i * 4)) & 0xf) * 2;
		rates->mcs_20_sdm[4 + i] = maxpwr - off;
	}

	/* 40 MHz, as brcmsmac wlc_phy_txpwr_apply_nphy() does it: N-PHY
	 * rev 5+ has separate MCS offsets for 40 MHz (SISO, CDD and STBC
	 * from words 4 and 5, SDM from words 6 and 7), older PHYs take the
	 * 20 MHz powers.
	 */
	if (phy->type == B43_PHYTYPE_N && phy->rev >= 5) {
		for (i = 0; i < 8; i++) {
			off = ((sprom_mcs_po[4 + i / 4] >> (i % 4 * 4)) & 0xf) * 2;
			rates->mcs_40[i] = maxpwr - off;
			rates->mcs_40_cdd[i] = maxpwr - off - extra_cdd_po;
			rates->mcs_40_stbc[i] = maxpwr - off - extra_stbc_po;

			off = ((sprom_mcs_po[6 + i / 4] >> (i % 4 * 4)) & 0xf) * 2;
			rates->mcs_40_sdm[i] = maxpwr - off;
		}
		b43_ppr_mcs_to_ofdm(rates->ofdm_40, rates->mcs_40);
		b43_ppr_mcs_to_ofdm(rates->ofdm_40_cdd, rates->mcs_40_cdd);
	} else {
		/* The 40 MHz rates are laid out as the 20 MHz ones */
		memcpy(&ppr->__all_rates[B43_PPR_40_FIRST],
		       &ppr->__all_rates[B43_PPR_20_FIRST],
		       B43_PPR_40_END - B43_PPR_40_FIRST);
	}
	if (phy->type == B43_PHYTYPE_N && phy->rev >= 3)
		for (i = B43_PPR_40_FIRST; i < B43_PPR_40_END; i++)
			ppr->__all_rates[i] -= extra_bw40_po * 2;
	rates->mcs_32 = rates->mcs_40_cdd[0];

	return true;
}
