/*
 * Copyright (c) 2015-2017, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <lib/xlat_tables/xlat_tables_v2.h>

#include <mce.h>
#include <memctrl.h>
#include <memctrl_v2.h>
#include <smmu.h>
#include <tegra_def.h>
#include <tegra_platform.h>

/* Video Memory base and size (live values) */
static uint64_t video_mem_base;
static uint64_t video_mem_size_mb;

static void tegra_memctrl_reconfig_mss_clients(void)
{
#if ENABLE_ROC_FOR_ORDERING_CLIENT_REQUESTS
	uint32_t val, wdata_0, wdata_1;

	/*
	 * Assert Memory Controller's HOTRESET_FLUSH_ENABLE signal for
	 * boot and strongly ordered MSS clients to flush existing memory
	 * traffic and stall future requests.
	 */
	val = tegra_mc_read_32(MC_CLIENT_HOTRESET_CTRL0);
	assert(val == MC_CLIENT_HOTRESET_CTRL0_RESET_VAL);

	wdata_0 = MC_CLIENT_HOTRESET_CTRL0_HDA_FLUSH_ENB |
#if ENABLE_AFI_DEVICE
		  MC_CLIENT_HOTRESET_CTRL0_AFI_FLUSH_ENB |
#endif
		  MC_CLIENT_HOTRESET_CTRL0_SATA_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL0_XUSB_HOST_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL0_XUSB_DEV_FLUSH_ENB;
	tegra_mc_write_32(MC_CLIENT_HOTRESET_CTRL0, wdata_0);

	/* Wait for HOTRESET STATUS to indicate FLUSH_DONE */
	do {
		val = tegra_mc_read_32(MC_CLIENT_HOTRESET_STATUS0);
	} while ((val & wdata_0) != wdata_0);

	/* Wait one more time due to SW WAR for known legacy issue */
	do {
		val = tegra_mc_read_32(MC_CLIENT_HOTRESET_STATUS0);
	} while ((val & wdata_0) != wdata_0);

	val = tegra_mc_read_32(MC_CLIENT_HOTRESET_CTRL1);
	assert(val == MC_CLIENT_HOTRESET_CTRL1_RESET_VAL);

	wdata_1 = MC_CLIENT_HOTRESET_CTRL1_SDMMC4A_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_APE_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_SE_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_ETR_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_AXIS_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_EQOS_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_UFSHC_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_BPMP_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_AON_FLUSH_ENB |
		  MC_CLIENT_HOTRESET_CTRL1_SCE_FLUSH_ENB;
	tegra_mc_write_32(MC_CLIENT_HOTRESET_CTRL1, wdata_1);

	/* Wait for HOTRESET STATUS to indicate FLUSH_DONE */
	do {
		val = tegra_mc_read_32(MC_CLIENT_HOTRESET_STATUS1);
	} while ((val & wdata_1) != wdata_1);

	/* Wait one more time due to SW WAR for known legacy issue */
	do {
		val = tegra_mc_read_32(MC_CLIENT_HOTRESET_STATUS1);
	} while ((val & wdata_1) != wdata_1);

	/*
	 * Change MEMTYPE_OVERRIDE from SO_DEV -> PASSTHRU for boot and
	 * strongly ordered MSS clients. ROC needs to be single point
	 * of control on overriding the memory type. So, remove TSA's
	 * memtype override.
	 *
	 * MC clients with default SO_DEV override still enabled at TSA:
	 * AONW, BPMPW, SCEW, APEW
	 */
#if ENABLE_AFI_DEVICE
	mc_set_tsa_passthrough(AFIW);
#endif
	mc_set_tsa_passthrough(HDAW);
	mc_set_tsa_passthrough(SATAW);
	mc_set_tsa_passthrough(XUSB_HOSTW);
	mc_set_tsa_passthrough(XUSB_DEVW);
	mc_set_tsa_passthrough(SDMMCWAB);
	mc_set_tsa_passthrough(APEDMAW);
	mc_set_tsa_passthrough(SESWR);
	mc_set_tsa_passthrough(ETRW);
	mc_set_tsa_passthrough(AXISW);
	mc_set_tsa_passthrough(EQOSW);
	mc_set_tsa_passthrough(UFSHCW);
	mc_set_tsa_passthrough(BPMPDMAW);
	mc_set_tsa_passthrough(AONDMAW);
	mc_set_tsa_passthrough(SCEDMAW);

	/* Parker has no IO Coherency support and need the following:
	 * Ordered MC Clients on Parker are AFI, EQOS, SATA, XUSB.
	 * ISO clients(DISP, VI, EQOS) should never snoop caches and
	 *     don't need ROC/PCFIFO ordering.
	 * ISO clients(EQOS) that need ordering should use PCFIFO ordering
	 *     and bypass ROC ordering by using FORCE_NON_COHERENT path.
	 * FORCE_NON_COHERENT/FORCE_COHERENT config take precedence
	 *     over SMMU attributes.
	 * Force all Normal memory transactions from ISO and non-ISO to be
	 *     non-coherent(bypass ROC, avoid cache snoop to avoid perf hit).
	 * Force the SO_DEV transactions from ordered ISO clients(EQOS) to
	 *     non-coherent path and enable MC PCFIFO interlock for ordering.
	 * Force the SO_DEV transactions from ordered non-ISO clients (PCIe,
	 *     XUSB, SATA) to coherent so that the transactions are
	 *     ordered by ROC.
	 * PCFIFO ensure write ordering.
	 * Read after Write ordering is maintained/enforced by MC clients.
	 * Clients that need PCIe type write ordering must
	 *     go through ROC ordering.
	 * Ordering enable for Read clients is not necessary.
	 * R5's and A9 would get necessary ordering from AXI and
	 *     don't need ROC ordering enable:
	 *     - MMIO ordering is through dev mapping and MMIO
	 *       accesses bypass SMMU.
	 *     - Normal memory is accessed through SMMU and ordering is
	 *       ensured by client and AXI.
	 *     - Ack point for Normal memory is WCAM in MC.
	 *     - MMIO's can be early acked and AXI ensures dev memory ordering,
	 *       Client ensures read/write direction change ordering.
	 *     - See Bug 200312466 for more details.
	 *
	 * CGID_TAG_ADR is only present from T186 A02. As this code is common
	 *    between A01 and A02, tegra_memctrl_set_overrides() programs
	 *    CGID_TAG_ADR for the necessary clients on A02.
	 */
	mc_set_txn_override(HDAR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(BPMPW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(PTCR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVDISPLAYR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(EQOSW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVJPGSWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(ISPRA, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCWAA, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(VICSRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(MPCOREW, CGID_TAG_DEFAULT, SO_DEV_ZERO, NO_OVERRIDE, NO_OVERRIDE);
	mc_set_txn_override(GPUSRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(AXISR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SCEDMAW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(EQOSR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	/* See bug 200131110 comment #35*/
	mc_set_txn_override(APEDMAR, CGID_TAG_CLIENT_AXI_ID, SO_DEV_CLIENT_AXI_ID, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVENCSRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCRAB, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(VICSRD1, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(BPMPDMAR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(VIW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCRAA, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(AXISW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(XUSB_DEVR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(UFSHCR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(TSECSWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(GPUSWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SATAR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(XUSB_HOSTW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_COHERENT);
	mc_set_txn_override(TSECSWRB, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(GPUSRD2, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SCEDMAR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(GPUSWR2, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(AONDMAW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	/* See bug 200131110 comment #35*/
	mc_set_txn_override(APEDMAW, CGID_TAG_CLIENT_AXI_ID, SO_DEV_CLIENT_AXI_ID, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(AONW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(HOST1XDMAR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(ETRR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SESWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVJPGSRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVDECSRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(TSECSRDB, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(BPMPDMAW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(APER, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVDECSRD1, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(XUSB_HOSTR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(ISPWA, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SESRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SCER, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(AONR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(MPCORER, CGID_TAG_DEFAULT, SO_DEV_ZERO, NO_OVERRIDE, NO_OVERRIDE);
	mc_set_txn_override(SDMMCWA, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(HDAW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVDECSWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(UFSHCW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(AONDMAR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SATAW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_COHERENT);
	mc_set_txn_override(ETRW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(VICSWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVENCSWR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	/* See bug 200131110 comment #35 */
	mc_set_txn_override(AFIR, CGID_TAG_DEFAULT, SO_DEV_CLIENT_AXI_ID, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCWAB, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCRA, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(NVDISPLAYR1, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(ISPWB, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(BPMPR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(APEW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(SDMMCR, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	mc_set_txn_override(XUSB_DEVW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_COHERENT);
	mc_set_txn_override(TSECSRD, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);
	/*
	 * See bug 200131110 comment #35 - there are no normal requests
	 * and AWID for SO/DEV requests is hardcoded in RTL for a
	 * particular PCIE controller
	 */
	mc_set_txn_override(AFIW, CGID_TAG_DEFAULT, SO_DEV_CLIENT_AXI_ID, FORCE_NON_COHERENT, FORCE_COHERENT);
	mc_set_txn_override(SCEW, CGID_TAG_DEFAULT, SO_DEV_ZERO, FORCE_NON_COHERENT, FORCE_NON_COHERENT);

	/*
	 * At this point, ordering can occur at ROC. So, remove PCFIFO's
	 * control over ordering requests.
	 *
	 * Change PCFIFO_*_ORDERED_CLIENT from ORDERED -> UNORDERED for
	 * boot and strongly ordered MSS clients
	 */
	val = MC_PCFIFO_CLIENT_CONFIG1_RESET_VAL &
#if ENABLE_AFI_DEVICE
		mc_set_pcfifo_unordered_boot_so_mss(1, AFIW) &
#endif
		mc_set_pcfifo_unordered_boot_so_mss(1, HDAW) &
		mc_set_pcfifo_unordered_boot_so_mss(1, SATAW);
	tegra_mc_write_32(MC_PCFIFO_CLIENT_CONFIG1, val);

	val = MC_PCFIFO_CLIENT_CONFIG2_RESET_VAL &
		mc_set_pcfifo_unordered_boot_so_mss(2, XUSB_HOSTW) &
		mc_set_pcfifo_unordered_boot_so_mss(2, XUSB_DEVW);
	tegra_mc_write_32(MC_PCFIFO_CLIENT_CONFIG2, val);

	val = MC_PCFIFO_CLIENT_CONFIG3_RESET_VAL &
		mc_set_pcfifo_unordered_boot_so_mss(3, SDMMCWAB);
	tegra_mc_write_32(MC_PCFIFO_CLIENT_CONFIG3, val);

	val = MC_PCFIFO_CLIENT_CONFIG4_RESET_VAL &
		mc_set_pcfifo_unordered_boot_so_mss(4, SESWR) &
		mc_set_pcfifo_unordered_boot_so_mss(4, ETRW) &
		mc_set_pcfifo_unordered_boot_so_mss(4, AXISW) &
		mc_set_pcfifo_unordered_boot_so_mss(4, UFSHCW) &
		mc_set_pcfifo_unordered_boot_so_mss(4, BPMPDMAW) &
		mc_set_pcfifo_unordered_boot_so_mss(4, AONDMAW) &
		mc_set_pcfifo_unordered_boot_so_mss(4, SCEDMAW);
	/* EQOSW is the only client that has PCFIFO order enabled. */
	val |= mc_set_pcfifo_ordered_boot_so_mss(4, EQOSW);
	tegra_mc_write_32(MC_PCFIFO_CLIENT_CONFIG4, val);

	val = MC_PCFIFO_CLIENT_CONFIG5_RESET_VAL &
		mc_set_pcfifo_unordered_boot_so_mss(5, APEDMAW);
	tegra_mc_write_32(MC_PCFIFO_CLIENT_CONFIG5, val);

	/*
	 * Deassert HOTRESET FLUSH_ENABLE for boot and strongly ordered MSS
	 * clients to allow memory traffic from all clients to start passing
	 * through ROC
	 */
	val = tegra_mc_read_32(MC_CLIENT_HOTRESET_CTRL0);
	assert(val == wdata_0);

	wdata_0 = MC_CLIENT_HOTRESET_CTRL0_RESET_VAL;
	tegra_mc_write_32(MC_CLIENT_HOTRESET_CTRL0, wdata_0);

	val = tegra_mc_read_32(MC_CLIENT_HOTRESET_CTRL1);
	assert(val == wdata_1);

	wdata_1 = MC_CLIENT_HOTRESET_CTRL1_RESET_VAL;
	tegra_mc_write_32(MC_CLIENT_HOTRESET_CTRL1, wdata_1);

#endif
}

static void tegra_memctrl_set_overrides(void)
{
	tegra_mc_settings_t *plat_mc_settings = tegra_get_mc_settings();
	const mc_txn_override_cfg_t *mc_txn_override_cfgs;
	uint32_t num_txn_override_cfgs;
	uint32_t i, val;

	/* Get the settings from the platform */
	assert(plat_mc_settings);
	mc_txn_override_cfgs = plat_mc_settings->txn_override_cfg;
	num_txn_override_cfgs = plat_mc_settings->num_txn_override_cfgs;

	/*
	 * Set the MC_TXN_OVERRIDE registers for write clients.
	 */
	if ((tegra_chipid_is_t186()) &&
	    (!tegra_platform_is_silicon() ||
	    (tegra_platform_is_silicon() && (tegra_get_chipid_minor() == 1)))) {

		/*
		 * GPU and NVENC settings for Tegra186 simulation and
		 * Silicon rev. A01
		 */
		val = tegra_mc_read_32(MC_TXN_OVERRIDE_CONFIG_GPUSWR);
		val &= ~MC_TXN_OVERRIDE_CGID_TAG_MASK;
		tegra_mc_write_32(MC_TXN_OVERRIDE_CONFIG_GPUSWR,
			val | MC_TXN_OVERRIDE_CGID_TAG_ZERO);

		val = tegra_mc_read_32(MC_TXN_OVERRIDE_CONFIG_GPUSWR2);
		val &= ~MC_TXN_OVERRIDE_CGID_TAG_MASK;
		tegra_mc_write_32(MC_TXN_OVERRIDE_CONFIG_GPUSWR2,
			val | MC_TXN_OVERRIDE_CGID_TAG_ZERO);

		val = tegra_mc_read_32(MC_TXN_OVERRIDE_CONFIG_NVENCSWR);
		val &= ~MC_TXN_OVERRIDE_CGID_TAG_MASK;
		tegra_mc_write_32(MC_TXN_OVERRIDE_CONFIG_NVENCSWR,
			val | MC_TXN_OVERRIDE_CGID_TAG_CLIENT_AXI_ID);

	} else {

		/*
		 * Settings for Tegra186 silicon rev. A02 and onwards.
		 */
		for (i = 0; i < num_txn_override_cfgs; i++) {
			val = tegra_mc_read_32(mc_txn_override_cfgs[i].offset);
			val &= ~MC_TXN_OVERRIDE_CGID_TAG_MASK;
			tegra_mc_write_32(mc_txn_override_cfgs[i].offset,
				val | mc_txn_override_cfgs[i].cgid_tag);
		}
	}
}

/*
 * Init Memory controller during boot.
 */
void tegra_memctrl_setup(void)
{
	uint32_t val;
	const uint32_t *mc_streamid_override_regs;
	uint32_t num_streamid_override_regs;
	const mc_streamid_security_cfg_t *mc_streamid_sec_cfgs;
	uint32_t num_streamid_sec_cfgs;
	tegra_mc_settings_t *plat_mc_settings = tegra_get_mc_settings();
	uint32_t i;

	INFO("Tegra Memory Controller (v2)\n");

#if ENABLE_SMMU_DEVICE
	/* Program the SMMU pagesize */
	tegra_smmu_init();
#endif
	/* Get the settings from the platform */
	assert(plat_mc_settings);
	mc_streamid_override_regs = plat_mc_settings->streamid_override_cfg;
	num_streamid_override_regs = plat_mc_settings->num_streamid_override_cfgs;
	mc_streamid_sec_cfgs = plat_mc_settings->streamid_security_cfg;
	num_streamid_sec_cfgs = plat_mc_settings->num_streamid_security_cfgs;

	/* Program all the Stream ID overrides */
	for (i = 0; i < num_streamid_override_regs; i++)
		tegra_mc_streamid_write_32(mc_streamid_override_regs[i],
			MC_STREAM_ID_MAX);

	/* Program the security config settings for all Stream IDs */
	for (i = 0; i < num_streamid_sec_cfgs; i++) {
		val = mc_streamid_sec_cfgs[i].override_enable << 16 |
		      mc_streamid_sec_cfgs[i].override_client_inputs << 8 |
		      mc_streamid_sec_cfgs[i].override_client_ns_flag << 0;
		tegra_mc_streamid_write_32(mc_streamid_sec_cfgs[i].offset, val);
	}

	/*
	 * All requests at boot time, and certain requests during
	 * normal run time, are physically addressed and must bypass
	 * the SMMU. The client hub logic implements a hardware bypass
	 * path around the Translation Buffer Units (TBU). During
	 * boot-time, the SMMU_BYPASS_CTRL register (which defaults to
	 * TBU_BYPASS mode) will be used to steer all requests around
	 * the uninitialized TBUs. During normal operation, this register
	 * is locked into TBU_BYPASS_SID config, which routes requests
	 * with special StreamID 0x7f on the bypass path and all others
	 * through the selected TBU. This is done to disable SMMU Bypass
	 * mode, as it could be used to circumvent SMMU security checks.
	 */
	tegra_mc_write_32(MC_SMMU_BYPASS_CONFIG,
			  MC_SMMU_BYPASS_CONFIG_SETTINGS);

	/*
	 * Re-configure MSS to allow ROC to deal with ordering of the
	 * Memory Controller traffic. This is needed as the Memory Controller
	 * boots with MSS having all control, but ROC provides a performance
	 * boost as compared to MSS.
	 */
	tegra_memctrl_reconfig_mss_clients();

	/* Program overrides for MC transactions */
	tegra_memctrl_set_overrides();
}

/*
 * Restore Memory Controller settings after "System Suspend"
 */
void tegra_memctrl_restore_settings(void)
{
	/*
	 * Re-configure MSS to allow ROC to deal with ordering of the
	 * Memory Controller traffic. This is needed as the Memory Controller
	 * resets during System Suspend with MSS having all control, but ROC
	 * provides a performance boost as compared to MSS.
	 */
	tegra_memctrl_reconfig_mss_clients();

	/* Program overrides for MC transactions */
	tegra_memctrl_set_overrides();

	/* video memory carveout region */
	if (video_mem_base) {
		tegra_mc_write_32(MC_VIDEO_PROTECT_BASE_LO,
				  (uint32_t)video_mem_base);
		tegra_mc_write_32(MC_VIDEO_PROTECT_BASE_HI,
				  (uint32_t)(video_mem_base >> 32));
		tegra_mc_write_32(MC_VIDEO_PROTECT_SIZE_MB, video_mem_size_mb);

		/*
		 * MCE propagates the VideoMem configuration values across the
		 * CCPLEX.
		 */
		mce_update_gsc_videomem();
	}
}

/*
 * Secure the BL31 DRAM aperture.
 *
 * phys_base = physical base of TZDRAM aperture
 * size_in_bytes = size of aperture in bytes
 */
void tegra_memctrl_tzdram_setup(uint64_t phys_base, uint32_t size_in_bytes)
{
	/*
	 * Setup the Memory controller to allow only secure accesses to
	 * the TZDRAM carveout
	 */
	INFO("Configuring TrustZone DRAM Memory Carveout\n");

	tegra_mc_write_32(MC_SECURITY_CFG0_0, (uint32_t)phys_base);
	tegra_mc_write_32(MC_SECURITY_CFG3_0, (uint32_t)(phys_base >> 32));
	tegra_mc_write_32(MC_SECURITY_CFG1_0, size_in_bytes >> 20);

	/*
	 * When TZ encryption enabled,
	 * We need setup TZDRAM before CPU to access TZ Carveout,
	 * otherwise CPU will fetch non-decrypted data.
	 * So save TZDRAM setting for retore by SC7 resume FW.
	 */

	mmio_write_32(TEGRA_SCRATCH_BASE + SECURE_SCRATCH_RSV55_LO,
					tegra_mc_read_32(MC_SECURITY_CFG0_0));
	mmio_write_32(TEGRA_SCRATCH_BASE + SECURE_SCRATCH_RSV55_HI,
					tegra_mc_read_32(MC_SECURITY_CFG3_0));
	mmio_write_32(TEGRA_SCRATCH_BASE + SECURE_SCRATCH_RSV54_HI,
					tegra_mc_read_32(MC_SECURITY_CFG1_0));

	/*
	 * MCE propagates the security configuration values across the
	 * CCPLEX.
	 */
	mce_update_gsc_tzdram();
}

/*
 * Secure the BL31 TZRAM aperture.
 *
 * phys_base = physical base of TZRAM aperture
 * size_in_bytes = size of aperture in bytes
 */
void tegra_memctrl_tzram_setup(uint64_t phys_base, uint32_t size_in_bytes)
{
	uint32_t index;
	uint32_t total_128kb_blocks = size_in_bytes >> 17;
	uint32_t residual_4kb_blocks = (size_in_bytes & (uint32_t)0x1FFFF) >> 12;
	uint32_t val;

	INFO("Configuring TrustZone SRAM Memory Carveout\n");

	/*
	 * Reset the access configuration registers to restrict access
	 * to the TZRAM aperture
	 */
	for (index = MC_TZRAM_CLIENT_ACCESS_CFG0;
	     index < ((uint32_t)MC_TZRAM_CARVEOUT_CFG + (uint32_t)MC_GSC_CONFIG_REGS_SIZE);
	     index += 4U) {
		tegra_mc_write_32(index, 0);
	}

	/*
	 * Set the TZRAM base. TZRAM base must be 4k aligned, at least.
	 */
	assert((phys_base & (uint64_t)0xFFF) == 0U);
	tegra_mc_write_32(MC_TZRAM_BASE_LO, (uint32_t)phys_base);
	tegra_mc_write_32(MC_TZRAM_BASE_HI,
		(uint32_t)(phys_base >> 32) & MC_GSC_BASE_HI_MASK);

	/*
	 * Set the TZRAM size
	 *
	 * total size = (number of 128KB blocks) + (number of remaining 4KB
	 * blocks)
	 *
	 */
	val = (residual_4kb_blocks << MC_GSC_SIZE_RANGE_4KB_SHIFT) |
	      total_128kb_blocks;
	tegra_mc_write_32(MC_TZRAM_SIZE, val);

	/*
	 * Lock the configuration settings by disabling TZ-only lock
	 * and locking the configuration against any future changes
	 * at all.
	 */
	val = tegra_mc_read_32(MC_TZRAM_CARVEOUT_CFG);
	val &= ~MC_GSC_ENABLE_TZ_LOCK_BIT;
	val |= MC_GSC_LOCK_CFG_SETTINGS_BIT;
	tegra_mc_write_32(MC_TZRAM_CARVEOUT_CFG, val);

	/*
	 * MCE propagates the security configuration values across the
	 * CCPLEX.
	 */
	mce_update_gsc_tzram();
}

static void tegra_lock_videomem_nonoverlap(uint64_t phys_base,
					   uint64_t size_in_bytes)
{
	uint32_t index;
	uint64_t total_128kb_blocks = size_in_bytes >> 17;
	uint64_t residual_4kb_blocks = (size_in_bytes & (uint32_t)0x1FFFF) >> 12;
	uint64_t val;

	/*
	 * Reset the access configuration registers to restrict access to
	 * old Videomem aperture
	 */
	for (index = MC_VIDEO_PROTECT_CLEAR_ACCESS_CFG0;
	     index < ((uint32_t)MC_VIDEO_PROTECT_CLEAR_ACCESS_CFG0 + (uint32_t)MC_GSC_CONFIG_REGS_SIZE);
	     index += 4U) {
		tegra_mc_write_32(index, 0);
	}

	/*
	 * Set the base. It must be 4k aligned, at least.
	 */
	assert((phys_base & (uint64_t)0xFFF) == 0U);
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_BASE_LO, (uint32_t)phys_base);
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_BASE_HI,
		(uint32_t)(phys_base >> 32) & (uint32_t)MC_GSC_BASE_HI_MASK);

	/*
	 * Set the aperture size
	 *
	 * total size = (number of 128KB blocks) + (number of remaining 4KB
	 * blocks)
	 *
	 */
	val = (uint32_t)((residual_4kb_blocks << MC_GSC_SIZE_RANGE_4KB_SHIFT) |
			 total_128kb_blocks);
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_SIZE, (uint32_t)val);

	/*
	 * Lock the configuration settings by enabling TZ-only lock and
	 * locking the configuration against any future changes from NS
	 * world.
	 */
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_CFG,
			  (uint32_t)MC_GSC_ENABLE_TZ_LOCK_BIT);

	/*
	 * MCE propagates the GSC configuration values across the
	 * CCPLEX.
	 */
}

static void tegra_unlock_videomem_nonoverlap(void)
{
	/* Clear the base */
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_BASE_LO, 0);
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_BASE_HI, 0);

	/* Clear the size */
	tegra_mc_write_32(MC_VIDEO_PROTECT_CLEAR_SIZE, 0);
}

static void tegra_clear_videomem(uintptr_t non_overlap_area_start,
				 unsigned long long non_overlap_area_size)
{
	/*
	 * Map the NS memory first, clean it and then unmap it.
	 */
	mmap_add_dynamic_region(non_overlap_area_start, /* PA */
				non_overlap_area_start, /* VA */
				non_overlap_area_size, /* size */
				MT_NS | MT_RW | MT_EXECUTE_NEVER); /* attrs */

	zero_normalmem((void *)non_overlap_area_start, non_overlap_area_size);
	flush_dcache_range(non_overlap_area_start, non_overlap_area_size);

	mmap_remove_dynamic_region(non_overlap_area_start,
		non_overlap_area_size);
}

/*
 * Program the Video Memory carveout region
 *
 * phys_base = physical base of aperture
 * size_in_bytes = size of aperture in bytes
 */
void tegra_memctrl_videomem_setup(uint64_t phys_base, uint32_t size_in_bytes)
{
	uintptr_t vmem_end_old = video_mem_base + (video_mem_size_mb << 20);
	uintptr_t vmem_end_new = phys_base + size_in_bytes;
	unsigned long long non_overlap_area_size;

	/*
	 * Setup the Memory controller to restrict CPU accesses to the Video
	 * Memory region
	 */
	INFO("Configuring Video Memory Carveout\n");

	/*
	 * Configure Memory Controller directly for the first time.
	 */
	if (video_mem_base == 0U)
		goto done;

	/*
	 * Lock the non overlapping memory being cleared so that other masters
	 * do not accidently write to it. The memory would be unlocked once
	 * the non overlapping region is cleared and the new memory
	 * settings take effect.
	 */
	tegra_lock_videomem_nonoverlap(video_mem_base,
				       video_mem_size_mb << 20);

	/*
	 * Clear the old regions now being exposed. The following cases
	 * can occur -
	 *
	 * 1. clear whole old region (no overlap with new region)
	 * 2. clear old sub-region below new base
	 * 3. clear old sub-region above new end
	 */
	INFO("Cleaning previous Video Memory Carveout\n");

	if (phys_base > vmem_end_old || video_mem_base > vmem_end_new) {
		tegra_clear_videomem(video_mem_base,
				     (uint64_t)video_mem_size_mb << 20);
	} else {
		if (video_mem_base < phys_base) {
			non_overlap_area_size = phys_base - video_mem_base;
			tegra_clear_videomem(video_mem_base, non_overlap_area_size);
		}
		if (vmem_end_old > vmem_end_new) {
			non_overlap_area_size = vmem_end_old - vmem_end_new;
			tegra_clear_videomem(vmem_end_new, non_overlap_area_size);
		}
	}

done:
	/* program the Videomem aperture */
	tegra_mc_write_32(MC_VIDEO_PROTECT_BASE_LO, (uint32_t)phys_base);
	tegra_mc_write_32(MC_VIDEO_PROTECT_BASE_HI,
			  (uint32_t)(phys_base >> 32));
	tegra_mc_write_32(MC_VIDEO_PROTECT_SIZE_MB, size_in_bytes >> 20);

	/* unlock the previous locked nonoverlapping aperture */
	tegra_unlock_videomem_nonoverlap();

	/* store new values */
	video_mem_base = phys_base;
	video_mem_size_mb = size_in_bytes >> 20;

	/*
	 * MCE propagates the VideoMem configuration values across the
	 * CCPLEX.
	 */
	mce_update_gsc_videomem();
}

/*
 * This feature exists only for v1 of the Tegra Memory Controller.
 */
void tegra_memctrl_disable_ahb_redirection(void)
{
	; /* do nothing */
}
