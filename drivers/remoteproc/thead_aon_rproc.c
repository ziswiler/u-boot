// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote processor driver for the Thead TH1520 E902 core running AON firmware
 *
 * Copyright (C) 2026 Codethink Ltd.
 */

#define DEBUG

#include <asm/io.h>
#include <cpu_func.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/device-internal.h>
#include <dm/lists.h>
#include <errno.h>
#include <hang.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>
#include <malloc.h>
#include <remoteproc.h>

//TH1520 System User Manual

//Start Address	End Address	Addressing Space	Description
//0x0000_0000	0xAFFF_FFFF	16GB			DDR
//E902_ADDR_remap

//4 Clock
//SUBSYS	REE BASE	TEE BASE	REE BASE	TEE BASE
//		ADDRESS(C910)	ADDRESS(C910)	ADDRESS(E902)	ADDRESS(E902)
//AON_SUBSYS	NA		0xFF_FFF4_6000	NA		0xFFF4_6000
//4.4.2.1.9 AONSYS_PLL_STS: System and audio PLL lock flag
//Offset: 0x90
//4.4.2.1.10 AONSYS_CLK_CFG: AON clock control register
//Offset: 0x100

//turn on the clock of AONSYS
//aon clk 72 MHz
//aon i2c clk 24 MHz
//AONSYS_PLL_STS
//AONSYS_CLK_CFG

//C910 configures the reset start addresses of E902 and C906, and releases reset of E902 and C906

//5 Reset
//SUBSYS	REE BASE	TEE BASE	REE BASE	TEE BASE
//		ADDRESS		ADDRESS		ADDRESS		ADDRESS
//		(C910)		(C910)		(E902)		(E902)
//AON_SUBSYS	NA		0xFF_FFF4_4000	NA		0xFFF4_4000
//SW_E902_CRST_N vs. SW_E902_HAD_RST_N

//AON_AHB_ADEXT: AON AHB address extension register
//AONSYS_AHB_SPACE_SEL
//				C910 Access Space		E902 Access Space
//				REE Start	TEE Start	REE Start	TEE Start
//				Address		Address		Address		Address
//System control registers	0xFFEF018000	0xFFFF018000	0xBF018000	0xFF018000
//AON_SUBSYS registers		Not accessible	0xFFFFF48000	Not accessible	0xFFF48000
//E902_RST_ADDR	0x44	E902 reset start address register		16.3.3.2.16/1075
//AON_AHB_ADEXT	0x60	AON_SUBSYS AHB bus address extension register	16.3.3.2.23/1077

#define C910_E902_START_ADDRESS 0xFFFFEF8000

#define E902_AON_CONFIG_SIZE	0xC00

//Start Address	End Address	Addressing Space	Description
//FF_FFC2_1000	0xFF_FFC2_1FFF	4KB			IOPMP_AO
//I/O Physical Memory Protection (IOPMP)
//Always-On (AO)
#define E902_IOPMP_BASE         0xFFFFC21000

//Figure & Table 1-3 E902 memory map

//Start Address		End Address	Addressing Space	Description
//0xFFEF_8000		0xFFEF_FFFF	32KB			AO_SRAM
//0xFFF0_0000		0xFFF0_7FFF	32KB			STR_MEM

//Figure & Table 1-4 Map relationship between E902 address and system address

#define E902_START_ADDRESS      (0xFFEF8000 + E902_AON_CONFIG_SIZE)

//5 Reset
//SUBSYS	REE BASE	TEE BASE	REE BASE	TEE BASE
//		ADDRESS		ADDRESS		ADDRESS		ADDRESS
//		(C910)		(C910)		(E902)		(E902)
//AON_SUBSYS	NA		0xFF_FFF4_4000	NA		0xFFF4_4000
//5.4.2.1.7 E902_RST_CFG: E902 soft reset configuration register
//Offset: 0x24
#define E902_SYSREG_RESET       0xfffff44024

//6.5.2.2.16 E902_RST_ADDR: E902 reset start address register

//16 System Configuration
//				C910 Access Space		E902 Access Space
//				REE Start	TEE Start	REE Start	TEE Start
//				Address		Address		Address		Address
//AON_SUBSYS registers		Not accessible	0xFFFFF48000	Not accessible	0xFFF48000
//16.3.3.2.16 E902_RST_ADDR: E902 reset start address register
//Offset: 0x44
#define E902_SYSREG_START       0xfffff48044

#define THEAD_AON_RPROC_DEV_TO_PHY_OFFSET 0xff00000000

/**
 * thead_aon_rproc_device_to_virt() - Convert device address to virtual address
 * @dev:	device to operate upon
 * @da:		device address
 * @size:	Size of the memory region @da is pointing to
 * @is_iomem:	optional pointer filled in to indicate if @da is iomapped memory
 * Return: converted virtual address
 */
static void *thead_aon_rproc_device_to_virt(struct udevice *dev, ulong da,
					     ulong size, bool *is_iomem)
{
	u64 paddr;

	/* Use a simple offset conversion */
	paddr = da + THEAD_AON_RPROC_DEV_TO_PHY_OFFSET;

	return phys_to_virt(paddr);
}

/**
 * thead_aon_rproc_load() - Load the remote processor
 * @dev:	corresponding remote processor device
 * @addr:	Address in memory where image is stored
 * @size:	Size in bytes of the image
 *
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int thead_aon_rproc_load(struct udevice *dev, ulong addr, ulong size)
{
	int ret = rproc_elf_load_image(dev, addr, size);
	if (ret < 0) {
		dev_err(dev, "Loading elf failed %d\n", ret);
		goto error;
	}

	flush_cache((uintptr_t)C910_E902_START_ADDRESS, 0x10000);

	u32 boot_vector = rproc_elf_get_boot_addr(dev, addr);

	dev_dbg(dev, "%s: Boot vector = 0x%x\n", __func__, boot_vector);

error:
	return ret;
}

/**
 * thead_aon_rproc_start() - Start the remote processor
 * @dev:	corresponding remote processor device
 *
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int thead_aon_rproc_start(struct udevice *dev)
{
	writel(0xffffffff, (void *)(E902_IOPMP_BASE + 0xc0));
	writel(0x0, (void *)E902_SYSREG_RESET);
	writel(E902_START_ADDRESS, (void *)E902_SYSREG_START);
	flush_cache((uintptr_t)C910_E902_START_ADDRESS, 0x10000);
	writel(0x3, (void *)E902_SYSREG_RESET);

	return 0;
}

/**
 * thead_aon_rproc_stop() - Stop the remote processor
 * @dev:	corresponding remote processor device
 *
 * Return: 0 if all went ok, else corresponding -ve error
 */
static int thead_aon_rproc_stop(struct udevice *dev)
{
	writel(0x3, (void *)E902_SYSREG_RESET);

	return 0;
}

static const struct dm_rproc_ops thead_aon_rproc_ops = {
	.device_to_virt = thead_aon_rproc_device_to_virt,
	.load = thead_aon_rproc_load,
	.start = thead_aon_rproc_start,
	.stop = thead_aon_rproc_stop,
};

#define LIGHT_APCLK_ADDRBASE    0xffff011000
#define LIGHT_AONCLK_ADDRBASE   0xfffff46000
#define LIGHT_DDRCLK_ADDRBASE   0xffff005000
#define LIGHT_MISC_SUBSYS_ADDRBASE      0xfffc02d000
#define LIGHT_VI_SUBSYS_ADDRBASE        0xfff4041000
#define LIGHT_VO_SUBSYS_ADDRBASE        0xffff401000
#define LIGHT_VO_SUBSYS_R_ADDRBASE      0xffef528000
#define LIGHT_VP_SUBSYS_ADDRBASE        0xfffcc01000
#define LIGHT_DSP_SUBSYS_ADDRBASE       0xffff041000
#define LIGHT_AUDIO_SUBSYS_ADDRBASE     0xffcb000000
#define LIGHT_APSYS_RSTGEN_ADDRBASE     0xffff015000
#define LIGHT_DPU_CLOCK_GATING_CTRL0    0xffef601A28
#define LIGHT_DPU_CLOCK_GATING_CTRL1    0xffef601A2C

/**
 * thead_aon_rproc_probe() - Basic probe
 * @dev:        corresponding k3 remote processor device
 *
 * Return: 0 if all goes good, else appropriate error message.
 */
static int thead_aon_rproc_probe(struct udevice *dev)
{
	int ret = 0;
	unsigned int tmp;
        uint32_t val;

        /* 1. double check all pll lock */
        udelay(60);
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x80);
        if (!((tmp & 0x3fe) == 0x3fe)) {
                dev_dbg(dev, "Not all PLLs locked\n");
		return -1;
	}

        /* 2. update sys_pll to frac mode, 2438.5536MHz */
        /* switch share_sram_clk to audio_pll_foutvco */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x104);
        tmp |= 0x2000;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x104);

        /* switch audio_c906_cclk to audio_pll_foutvco */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x110);
        tmp |= 0x2000;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x110);

        /* switch audio_subsys_aclk to audio_pll_foutvco */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x108);
        tmp |= 0x2000;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x108);
#if 0
        /* disable audio_i2s_src_clk */
        tmp = readl((void *)LIGHT_AUDIO_SUBSYS_ADDRBASE + 0x4);
        tmp &= ~0x20200;
        writel(tmp, (void *)LIGHT_AUDIO_SUBSYS_ADDRBASE + 0x4);

        /* disable peri_i2s_src_clk */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x1f0);
        tmp &= ~0x2;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1f0);
#endif
        udelay(1);

        /* set sys_pll_foutvco to 2438.5536MHz */
        writel(0x20000000, (void *)LIGHT_AONCLK_ADDRBASE + 0x14);
        writel(0x03606501, (void *)LIGHT_AONCLK_ADDRBASE + 0x10);
        writel(0x209b3d08, (void *)LIGHT_AONCLK_ADDRBASE + 0x14);
        udelay(3);
        writel(0x009b3d08, (void *)LIGHT_AONCLK_ADDRBASE + 0x14);
        readl((void *)LIGHT_AONCLK_ADDRBASE + 0x90);
        readl((void *)LIGHT_AONCLK_ADDRBASE + 0x90);
        while(!(readl((void *)LIGHT_AONCLK_ADDRBASE + 0x90) & 0x2));
        udelay(11);

        /* switch share_sram_clk to sys_pll_foutvco */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x104);
        tmp &= ~0x2000;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x104);

        /* switch audio_c906_cclk to sys_pll_foutvco */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x110);
        tmp &= ~0x2000;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x110);

        /* switch audio_subsys_aclk to sys_pll_foutvco */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x108);
        tmp &= ~0x2000;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x108);

        /* 3. update audio_pll, to frac mode, 884.736MHz */
        /* switch aonsys_clk to pad_osc_clk */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x100);
        tmp |= 0x10;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x100);

        /* set audio_pll_foutvco to frac mode, 884.736MHz */
        writel(0x20000000, (void *)LIGHT_AONCLK_ADDRBASE + 0x04);
        writel(0x01302401, (void *)LIGHT_AONCLK_ADDRBASE + 0x00);
        writel(0x20dd2f70, (void *)LIGHT_AONCLK_ADDRBASE + 0x04);
        udelay(3);
        writel(0x00dd2f70, (void *)LIGHT_AONCLK_ADDRBASE + 0x04);
        readl((void *)LIGHT_AONCLK_ADDRBASE + 0x90);
        readl((void *)LIGHT_AONCLK_ADDRBASE + 0x90);
        while(!(readl((void *)LIGHT_AONCLK_ADDRBASE + 0x90) & 0x1));
        udelay(11);

        /* switch aonsys_clk to audio_pll_foutpostdiv */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x100);
        tmp &= ~0x10;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x100);

        /* switch aoi2c_ic_clk to audio_pll_fout3 */
        tmp = readl((void *)LIGHT_AONCLK_ADDRBASE + 0x11c);
        tmp &= ~0x1;
        writel(tmp, (void *)LIGHT_AONCLK_ADDRBASE + 0x11c);
#if 0
        /* enable audio_i2s_src_clk */
        tmp = readl((void *)LIGHT_AUDIO_SUBSYS_ADDRBASE + 0x4);
        tmp |= 0x20200;
        writel(tmp, (void *)LIGHT_AUDIO_SUBSYS_ADDRBASE + 0x4);

        /* enable peri_i2s_src_clk */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x1f0);
        tmp |= 0x2;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1f0);
#endif
        /* set apb3_cpusys_pclk to ahb2_cpusys_hclk/2 */
        /* CPU AHB 125MHz  CPU pclk 125MHz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x130);
        tmp &= ~0x8;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x130);
        udelay(1);
        tmp &= ~0x7;
        tmp |= 0x1;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x130);
        udelay(1);
        tmp |= 0x8;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x130);
        udelay(1);
        /* CPU AHB 125MHz  CPU pclk 62.5MHz */

        /* set ahb2_cpusys_hclk to 250Mhz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x120);
        tmp |= 0x20;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x120);
        udelay(1);
        tmp &= ~0x10;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x120);
        udelay(1);
        tmp &= ~0x7;
        tmp |= 0x2;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x120);
        udelay(1);
        tmp |= 0x10;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x120);
        udelay(1);
        tmp &= ~0x20;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x120);
        udelay(1);
        /* CPU AHB 250MHz  CPU pclk 125MHz */

        /* perisys_apb_pclk to perisys_ahb_hclk/4 */
        /* perisys_ahb_hclk 62.5MHz  perisys_apb_pclk 62.5MHz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x150);
        tmp &= ~0x8;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x150);
        udelay(1);
        tmp &= ~0x7;
        tmp |= 0x3;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x150);
        udelay(1);
        tmp |= 0x8;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x150);
        udelay(1);
        /* perisys_ahb_hclk 62.5MHz  perisys_apb_pclk 15.625MHz */

        /* perisys_ahb_hclk to 250MHz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x140);
        tmp |= 0x20;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x140);
        udelay(1);
        /* perisys_ahb_hclk 24MHz  perisys_apb_pclk 6MHz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x140);
        tmp &= ~0x10;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x140);
        udelay(1);
        tmp &= ~0xf;
        tmp |= 0x2;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x140);
        /* perisys_ahb_hclk 24MHz  perisys_apb_pclk 6MHz */
        udelay(1);
        tmp |= 0x10;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x140);
        udelay(1);
        tmp &= ~0x20;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x140);
        /* perisys_ahb_hclk 250MHz  perisys_apb_pclk 62.5MHz */

        /* set dpu0_pll_div_clk to dpu0_pll_foutpostdiv/16 as 74.25MHz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x1e8);
        tmp &= ~0x100;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1e8);
        udelay(1);
        tmp &= ~0xff;
        tmp |= 0x10;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1e8);
        udelay(1);
        tmp |= 0x100;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1e8);
        udelay(1);

        /* set dpu1_pll_div_clk to dpu1_pll_foutpostdiv/16 as 74.25MHz */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x1ec);
        tmp &= ~0x100;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1ec);
        udelay(1);
        tmp &= ~0xff;
        tmp |= 0x10;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1ec);
        udelay(1);
        tmp |= 0x100;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1ec);
        udelay(1);

        /*5. enable necessary gates */
        /* enable dsp_subsys, vi_subsys, vo_subsys all clocls */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x220);
        tmp |= 0x7;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x220);

        /* AP rst_gen: VP/VO/VI/DSP */
        writel(0xf, (void *)LIGHT_APSYS_RSTGEN_ADDRBASE + 0x220);

        /* enable dsp0/1_cclk, dsp0/1_pclk */
        tmp = readl((void *)LIGHT_DSP_SUBSYS_ADDRBASE + 0x24);
        tmp |= 0xf;
        writel(tmp, (void *)LIGHT_DSP_SUBSYS_ADDRBASE + 0x24);

        /* enable gpu_core_clk, gpu_cfg_aclk */
        tmp = readl((void *)LIGHT_VO_SUBSYS_ADDRBASE + 0x50);
        tmp |= 0x18;
        writel(tmp, (void *)LIGHT_VO_SUBSYS_ADDRBASE + 0x50);

        tmp = readl((void *)LIGHT_VO_SUBSYS_R_ADDRBASE + 0x50);
        tmp |= 0x3ff;
        writel(tmp, (void *)LIGHT_VO_SUBSYS_R_ADDRBASE + 0x50);

        /* enable dpu_pixelclk0/1, dpu_hclk, dpu_aclk, dpu_cclk */
        tmp = readl((void *)LIGHT_VO_SUBSYS_ADDRBASE + 0x50);
        tmp |= 0x3e0;
        writel(tmp, (void *)LIGHT_VO_SUBSYS_ADDRBASE + 0x50);

        /* enable npu_axi_aclk, npu_core_clk */
        tmp = readl((void *)LIGHT_APCLK_ADDRBASE + 0x1c8);
        tmp |= 0x30;
        writel(tmp, (void *)LIGHT_APCLK_ADDRBASE + 0x1c8);

#define AON_PADMUX_BASE (0xfffff4a000)
       /* init aon-iic pad */
        val = readl((void *)(AON_PADMUX_BASE + 0x10));
        val &=~0xffff0000;
        val |=0x208<<16;
        writel( val, (void *)(AON_PADMUX_BASE + 0x10));
        val = readl((void *)(AON_PADMUX_BASE + 0x14));
        val &=~0xffff;
        val |=0x208;
        writel( val, (void *)(AON_PADMUX_BASE + 0x14));

	dev_dbg(dev, "Remoteproc successfully probed\n");

        return ret;
}

static const struct udevice_id thead_aon_rproc_ids[] = {
	{ .compatible = "thead,th1520-aon-rproc" },
	{ }
};

U_BOOT_DRIVER(thead_aon_rproc) = {
	.name		= "thead-aon-rproc",
	.of_match	= thead_aon_rproc_ids,
	.id		= UCLASS_REMOTEPROC,
	.ops		= &thead_aon_rproc_ops,
	.probe		= thead_aon_rproc_probe,
};
