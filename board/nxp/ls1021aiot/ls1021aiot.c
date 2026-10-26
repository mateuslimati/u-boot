// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2021 NXP
 */

#include <config.h>
#include <command.h>
#include <clock_legacy.h>
#include <fdt_support.h>
#include <fdtdec.h>
#include <init.h>
#include <net.h>
#include <asm/arch/immap_ls102xa.h>
#include <asm/arch/clock.h>
#include <asm/arch/fsl_serdes.h>
#include <asm/arch/ls102xa_stream_id.h>
#include <asm/global_data.h>
#include <linux/delay.h>

#include <asm/arch/ls102xa_devdis.h>
#include <asm/arch/ls102xa_soc.h>
#include <asm/sections.h>
#include <fsl_csu.h>
#include <fsl_immap.h>
#include <fsl_ifc.h>
#include <netdev.h>
#include <fsl_mdio.h>
#include <tsec.h>
#include <spl.h>

#include <fsl_validate.h>
#include "../common/sleep.h"

DECLARE_GLOBAL_DATA_PTR;

#define DDR_SIZE		0x40000000

int checkboard(void)
{
	puts("Board: LS1021AIOT\n");

#ifndef CONFIG_QSPI_BOOT
	struct ccsr_gur *dcfg = (struct ccsr_gur *)CFG_SYS_FSL_GUTS_ADDR;
	u32 cpldrev;

	cpldrev = in_be32(&dcfg->gpporcr1);

	printf("CPLD:  V%d.%d\n", ((cpldrev >> 28) & 0xf), ((cpldrev >> 24) &
		0xf));
#endif
	{
		struct ccsr_ddr *ddr = (struct ccsr_ddr *)CFG_SYS_FSL_DDR_ADDR;

		printf("ECC:   %s%s, errors 0x%08x\n",
		       in_be32(&ddr->sdram_cfg) & SDRAM_CFG_ECC_EN ? "on" : "off",
		       in_be32(&ddr->sdram_cfg_3) & 0x40000000 ? ", fixing and scrubbing" : "",
		       in_be32(&ddr->err_detect));
	}
	return 0;
}

void ddrmc_init(void)
{
	struct ccsr_ddr *ddr = (struct ccsr_ddr *)CFG_SYS_FSL_DDR_ADDR;
	u32 temp_sdram_cfg, tmp;

	out_be32(&ddr->sdram_cfg, DDR_SDRAM_CFG);

	out_be32(&ddr->cs0_bnds, DDR_CS0_BNDS);
	out_be32(&ddr->cs0_config, DDR_CS0_CONFIG);

	out_be32(&ddr->timing_cfg_0, DDR_TIMING_CFG_0);
	out_be32(&ddr->timing_cfg_1, DDR_TIMING_CFG_1);
	out_be32(&ddr->timing_cfg_2, DDR_TIMING_CFG_2);
	out_be32(&ddr->timing_cfg_3, DDR_TIMING_CFG_3);
	out_be32(&ddr->timing_cfg_4, DDR_TIMING_CFG_4);
	out_be32(&ddr->timing_cfg_5, DDR_TIMING_CFG_5);

	out_be32(&ddr->sdram_cfg_2, DDR_SDRAM_CFG_2);
	out_be32(&ddr->ddr_cdr2, DDR_DDR_CDR2);

	out_be32(&ddr->sdram_mode, DDR_SDRAM_MODE);
	out_be32(&ddr->sdram_mode_2, DDR_SDRAM_MODE_2);

	out_be32(&ddr->sdram_interval, DDR_SDRAM_INTERVAL);

	out_be32(&ddr->ddr_wrlvl_cntl, DDR_DDR_WRLVL_CNTL);

	out_be32(&ddr->ddr_wrlvl_cntl_2, DDR_DDR_WRLVL_CNTL_2);
	out_be32(&ddr->ddr_wrlvl_cntl_3, DDR_DDR_WRLVL_CNTL_3);

	out_be32(&ddr->ddr_cdr1, DDR_DDR_CDR1);

	out_be32(&ddr->sdram_clk_cntl, DDR_SDRAM_CLK_CNTL);
	out_be32(&ddr->ddr_zq_cntl, DDR_DDR_ZQ_CNTL);

	out_be32(&ddr->cs0_config_2, DDR_CS0_CONFIG_2);

	/* DDR erratum A-009942 */
	tmp = in_be32(&ddr->debug[28]);
	out_be32(&ddr->debug[28], tmp | 0x0070006f);

	udelay(500);

	temp_sdram_cfg = (DDR_SDRAM_CFG_MEM_EN & ~SDRAM_CFG_BI);

	out_be32(&ddr->sdram_cfg, DDR_SDRAM_CFG | temp_sdram_cfg);

	/*
	 * With D_INIT the controller writes all of the memory, and so a valid
	 * ECC code, before it may be used: wait for it (about 0.3 s for 1 GiB).
	 * The SPL timer does not run yet here, so count register reads
	 * instead of using udelay(); the bound is several seconds.
	 */
	for (tmp = 0; tmp < 100000000; tmp++)
		if (!(in_be32(&ddr->sdram_cfg_2) & SDRAM_CFG2_D_INIT))
			break;
	if (tmp == 100000000)
		puts("DDR: memory initialization did not finish\n");

	/*
	 * ECC corrects a single-bit error on the way to the CPU but leaves it
	 * in memory, so every later read of that word raises it again. With
	 * ECC_FIX_EN the controller writes the corrected word back; the scrub
	 * reads walk through all of the memory to fix the words nobody reads.
	 * Only allowed once D_INIT is done.
	 */
	out_be32(&ddr->sdram_cfg_3, DDR_SDRAM_CFG_3_SCRUB);
}

int dram_init(void)
{
#if (!defined(CONFIG_SPL) || defined(CONFIG_XPL_BUILD))
	ddrmc_init();
#endif

	erratum_a008850_post();

	gd->ram_size = DDR_SIZE;
	return 0;
}

int board_early_init_f(void)
{
	struct ccsr_scfg *scfg = (struct ccsr_scfg *)CFG_SYS_FSL_SCFG_ADDR;

#ifdef CONFIG_TSEC_ENET
	/* clear BD & FR bits for BE BD's and frame data */
	clrbits_be32(&scfg->etsecdmamcr, SCFG_ETSECDMAMCR_LE_BD_FR);
	out_be32(&scfg->etsecmcr, SCFG_ETSECCMCR_GE2_CLK125);

#endif

#ifdef CONFIG_FSL_IFC
	init_early_memctl_regs();
#endif

	arch_soc_init();

	return 0;
}

#ifdef CONFIG_XPL_BUILD
void board_init_f(ulong dummy)
{
	/* Clear the BSS */
	memset(__bss_start, 0, __bss_end - __bss_start);

	get_clocks();

	preloader_console_init();

	dram_init();

	/* Allow OCRAM access permission as R/W */

#ifdef CONFIG_LAYERSCAPE_NS_ACCESS
	enable_layerscape_ns_access();
#endif

	board_init_r(NULL, 0);
}
#endif

int board_init(void)
{
#ifndef CONFIG_SYS_FSL_NO_SERDES
	fsl_serdes_init();
#endif

	ls102xa_smmu_stream_id_init();

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	return 0;
}
#endif

#if defined(CONFIG_MISC_INIT_R)
int misc_init_r(void)
{
#ifdef CONFIG_FSL_DEVICE_DISABLE
	device_disable(devdis_tbl, ARRAY_SIZE(devdis_tbl));

#endif
	return 0;
}
#endif

int ft_board_setup(void *blob, struct bd_info *bd)
{
	int off = fdt_node_offset_by_compat_reg(blob, FSL_IFC_COMPAT,
						CFG_SYS_IFC_ADDR);
	bool ifc_enabled = off >= 0 && fdtdec_get_is_enabled(blob, off);

	ft_cpu_setup(blob, bd);

	/*
	 * ft_cpu_setup() disables the IFC when booting with QSPI, but on this
	 * board the system CPLD on IFC CS0 works alongside QSPI. Keep the IFC
	 * enabled when the device tree enables it.
	 */
	if (ifc_enabled) {
		off = fdt_node_offset_by_compat_reg(blob, FSL_IFC_COMPAT,
						    CFG_SYS_IFC_ADDR);
		fdt_set_node_status(blob, off, FDT_STATUS_OKAY);
	}

#ifdef CONFIG_PCI
	ft_pci_setup(blob, bd);
#endif

	return 0;
}

void flash_write16(u16 val, void *addr)
{
	u16 shftval = (((val >> 8) & 0xff) | ((val << 8) & 0xff00));

	__raw_writew(shftval, addr);
}

u16 flash_read16(void *addr)
{
	u16 val = __raw_readw(addr);

	return (((val) >> 8) & 0x00ff) | (((val) << 8) & 0xff00);
}

/*
 * System CPLD on IFC CS0. Images before 3.0 have one register at every
 * address: reads return the SW2 switches, writes go to a control register
 * (0xF6 after reset; bit 0 is LED0 from 2.9). Image 3.0 has addressed
 * registers while CS0 runs in NOR ADM "AVD" mode, once 0x3C is written at
 * offset 0xC3; it goes back to the single register at every PORESET. The
 * cpld command enters that mode only while it runs and puts CS0 back as
 * it was, so the OS always finds the IFC set up by U-Boot.
 */
#define CPLD_VER		0x00
#define CPLD_ID			0x01
#define CPLD_DIP		0x02
#define CPLD_RST		0x08
#define CPLD_CTRL		0x09
#define CPLD_ACTION		0x0b
#define CPLD_WDT_CTRL		0x0c
#define CPLD_WDT_TIMEOUT	0x0d
#define CPLD_CAUSE		0x0f
#define CPLD_LAST		0x1c
#define CPLD_UNLOCK		0xc3
#define CPLD_ID_VALUE		0x1a
#define CPLD_UNLOCK_VALUE	0x3c
#define CPLD_CTRL_DEFAULT	0xf6
#define CPLD_CTRL_LED0		BIT(0)

/* CS0 for the 3.0 registers: 8 bits, NOR, ADM_SHFT_MODE = 1, AVD NOR */
#define CPLD_NOR_CSOR		0x80000020
static const u32 cpld_nor_ftim[3] = {
	FTIM0_NOR_TACSE(1) | FTIM0_NOR_TEADC(32) | FTIM0_NOR_TAVDS(4) | FTIM0_NOR_TEAHC(16),
	FTIM1_NOR_TACO(0xff) | FTIM1_NOR_TRAD_NOR(0x3f) | FTIM1_NOR_TSEQRAD_NOR(0x3f),
	FTIM2_NOR_TCS(0xf) | FTIM2_NOR_TCH(0xf) | FTIM2_NOR_TWPH(0x10) | FTIM2_NOR_TWP(0xff),
};

static const char *const cpld_resets[8] = {
	"switch", "sgmii1_phy", "sgmii2_phy", "usb_hub",
	"arduino_shield", "hdmi", "pcie1", "pcie2",
};

struct cpld_cs0 {
	u32 cspr;
	u32 csor;
	u32 ftim[3];
};

static u8 cpld_ctrl = CPLD_CTRL_DEFAULT;	/* copy of the 2.x register */

static u8 cpld_rd(unsigned int reg)
{
	return readb(CFG_SYS_CPLD_BASE + reg);
}

static void cpld_wr(unsigned int reg, u8 val)
{
	writeb(val, CFG_SYS_CPLD_BASE + reg);
}

/* the version the CPLD drove on IFC_AD0-7 at PORESET */
static u8 cpld_version(void)
{
	struct ccsr_gur *dcfg = (struct ccsr_gur *)CFG_SYS_FSL_GUTS_ADDR;

	return in_be32(&dcfg->gpporcr1) >> 24;
}

static void cpld_cs0_set(u32 cspr, u32 csor, const u32 *ftim)
{
	int i;

	set_ifc_cspr(0, cspr & ~CSPR_V);
	set_ifc_csor(0, csor);
	for (i = 0; i < 3; i++)
		set_ifc_ftim(0, IFC_FTIM0 + i, ftim[i]);
	set_ifc_cspr(0, cspr);
	get_ifc_cspr(0);
}

/* unlock the 3.0 registers; 0 when they answer */
static int cpld_open(struct cpld_cs0 *saved)
{
	int i;

	if (cpld_version() < 0x30)
		return -ENODEV;

	saved->cspr = get_ifc_cspr(0);
	saved->csor = get_ifc_csor(0);
	for (i = 0; i < 3; i++)
		saved->ftim[i] = get_ifc_ftim(0, IFC_FTIM0 + i);

	/* MSEL 0 is NOR */
	cpld_cs0_set(saved->cspr & ~CSPR_MSEL, CPLD_NOR_CSOR, cpld_nor_ftim);
	cpld_wr(CPLD_UNLOCK, CPLD_UNLOCK_VALUE);
	if (cpld_rd(CPLD_ID) == CPLD_ID_VALUE)
		return 0;

	cpld_cs0_set(saved->cspr, saved->csor, saved->ftim);
	return -EIO;
}

static void cpld_close(struct cpld_cs0 *saved)
{
	cpld_wr(CPLD_UNLOCK, 0);
	cpld_cs0_set(saved->cspr, saved->csor, saved->ftim);
}

static int cpld_reset_id(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cpld_resets); i++)
		if (!strcmp(name, cpld_resets[i]))
			return i;
	return -1;
}

static void cpld_info(void)
{
	static const char *const causes[] = {
		[0x01] = "power-up or full reset", [0x02] = "cold reboot",
		[0x04] = "warm reset", [0x0a] = "watchdog, cold reboot",
		[0x0c] = "watchdog, warm reset", [0x10] = "RESET_REQ_B (OS reboot)",
	};
	u8 c = cpld_rd(CPLD_CAUSE), rst = cpld_rd(CPLD_RST);
	u8 wdt = cpld_rd(CPLD_WDT_CTRL);
	int i;

	printf("CPLD %d.%d, SW2 0x%02x, last reset: %s (0x%02x)\n",
	       cpld_rd(CPLD_VER) >> 4, cpld_rd(CPLD_VER) & 0xf, cpld_rd(CPLD_DIP),
	       c < ARRAY_SIZE(causes) && causes[c] ? causes[c] : "?", c);
	printf("held in reset:");
	for (i = 0; i < ARRAY_SIZE(cpld_resets); i++)
		if (!(rst & BIT(i)))
			printf(" %s", cpld_resets[i]);
	printf("%s\n", rst == 0xff ? " none" : "");
	printf("watchdog: %s, %d s\n", wdt & 1 ? (wdt & 2 ? "on, cold reboot" :
	       "on, warm reset") : "off", cpld_rd(CPLD_WDT_TIMEOUT));
}

static int do_cpld_v3(int argc, char *const argv[])
{
	ulong reg, val;
	int i, id;

	if (!strcmp(argv[1], "info") && argc == 2) {
		cpld_info();
	} else if (!strcmp(argv[1], "dump") && argc == 2) {
		for (i = 0; i <= CPLD_LAST; i++)
			printf("%02x%c", cpld_rd(i), (i & 7) == 7 || i == CPLD_LAST ? '\n' : ' ');
	} else if (!strcmp(argv[1], "read") && argc == 3) {
		reg = hextoul(argv[2], NULL);
		printf("0x%02lx = 0x%02x\n", reg & 0xff, cpld_rd(reg & 0xff));
	} else if (!strcmp(argv[1], "write") && argc == 4) {
		cpld_wr(hextoul(argv[2], NULL) & 0xff, hextoul(argv[3], NULL));
	} else if (!strcmp(argv[1], "led0") && argc == 3) {
		val = cpld_rd(CPLD_CTRL) & ~CPLD_CTRL_LED0;
		if (!strcmp(argv[2], "on"))
			val |= CPLD_CTRL_LED0;
		else if (strcmp(argv[2], "off"))
			return CMD_RET_USAGE;
		cpld_wr(CPLD_CTRL, val);
	} else if (!strcmp(argv[1], "reset") && (argc == 3 || argc == 4)) {
		id = cpld_reset_id(argv[2]);
		if (id < 0)
			return CMD_RET_USAGE;
		val = cpld_rd(CPLD_RST);
		if (argc == 3 || !strcmp(argv[3], "pulse")) {
			cpld_wr(CPLD_RST, val & ~BIT(id));
			mdelay(10);
			cpld_wr(CPLD_RST, val | BIT(id));
		} else if (!strcmp(argv[3], "hold")) {
			cpld_wr(CPLD_RST, val & ~BIT(id));
		} else if (!strcmp(argv[3], "run")) {
			cpld_wr(CPLD_RST, val | BIT(id));
		} else {
			return CMD_RET_USAGE;
		}
	} else if (!strcmp(argv[1], "reboot") && argc == 3) {
		if (!strcmp(argv[2], "cold"))
			val = 0xc0;
		else if (!strcmp(argv[2], "warm"))
			val = 0xa1;
		else if (!strcmp(argv[2], "full"))
			val = 0xb0;
		else
			return CMD_RET_USAGE;
		cpld_wr(CPLD_ACTION, val);
		mdelay(3000);
		puts("CPLD did not reset the board\n");
		return CMD_RET_FAILURE;
	} else if (!strcmp(argv[1], "wdt") && (argc == 3 || argc == 4)) {
		if (!strcmp(argv[2], "off")) {
			cpld_wr(CPLD_WDT_CTRL, 0);
		} else {
			val = dectoul(argv[2], NULL);
			if (!val || val > 255)
				return CMD_RET_USAGE;
			cpld_wr(CPLD_WDT_TIMEOUT, val);
			cpld_wr(CPLD_WDT_CTRL, 1 | (argc == 4 && !strcmp(argv[3], "cold") ? 2 : 0));
		}
	} else {
		return CMD_RET_USAGE;
	}
	return CMD_RET_SUCCESS;
}

static int do_cpld(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct cpld_cs0 saved;
	int ret;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (!cpld_open(&saved)) {
		ret = do_cpld_v3(argc, argv);
		cpld_close(&saved);
		return ret;
	}

	/* 2.x: only the switches and LED0 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "bcsr")) {
		printf("CPLD %d.%d (single register), SW2 0x%02x\n",
		       cpld_version() >> 4, cpld_version() & 0xf, cpld_rd(0));
	} else if (!strcmp(argv[1], "led0") && argc == 3) {
		if (!strcmp(argv[2], "on"))
			cpld_ctrl |= CPLD_CTRL_LED0;
		else if (!strcmp(argv[2], "off"))
			cpld_ctrl &= ~CPLD_CTRL_LED0;
		else
			return CMD_RET_USAGE;
		cpld_wr(0, cpld_ctrl);
	} else {
		puts("needs CPLD image 3.0\n");
		return CMD_RET_FAILURE;
	}
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	cpld, 4, 0, do_cpld,
	"system CPLD",
	"info               - version, SW2, last reset, resets held, watchdog\n"
	"cpld led0 on|off\n"
	"cpld dump               - registers 0x00-0x1c (CPLD 3.0 from here on)\n"
	"cpld read <reg>\n"
	"cpld write <reg> <val>\n"
	"cpld reset <chip> [pulse|hold|run]\n"
	"    chip: switch sgmii1_phy sgmii2_phy usb_hub arduino_shield hdmi pcie1 pcie2\n"
	"cpld reboot cold|warm|full\n"
	"cpld wdt <seconds> [cold] | off - the OS watchdog driver takes it over"
);
