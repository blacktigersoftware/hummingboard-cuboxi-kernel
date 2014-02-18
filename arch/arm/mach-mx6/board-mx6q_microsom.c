/*
 * SolidRun micromSOM board initialization.
 * Copyright (C) 2013 SolidRun ltd.
 * Based on sabresd board from Freescale Semiconductor, Inc. All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#ifdef CONFIG_ANDROID
#include <linux/ion.h>
#endif
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>
#include <mach/system.h>
#include <mach/imx_rfkill.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_microsom.h"
#include "board-mx6dl_microsom.h"

#define USOM_WL_RST		IMX_GPIO_NR(5, 26)
#define USOM_BT_RST		IMX_GPIO_NR(6, 0)
#define USOM_BT_SHUTDOWN	IMX_GPIO_NR(6,15)
#define USOM_REG_ON		IMX_GPIO_NR(3, 19)
#define USOM_XTAL_ON		IMX_GPIO_NR(5, 5)
#define USOM_ENET_RST		IMX_GPIO_NR(4, 15)

static struct clk *sata_clk;
static int caam_enabled;
static int disable_wifi_detect = 0;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

extern u32 enable_ldo_mode;

static const struct esdhc_platform_data mx6q_usom_sd1_data __initconst = {
	.cd_gpio = -EINVAL,
	.wp_gpio = -EINVAL,
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_usom_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

/* I2C2 / HDMI DDC */
static struct imxi2c_platform_data mx6q_usom_i2c_data = {
	.bitrate = 100000,
};
static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

static int mx6q_usom_fec_ar8035_phy_init(struct phy_device *phydev)
{
	unsigned short val;
	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	gpio_set_value(USOM_ENET_RST, 1);
	mdelay(10);
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);
	/* To enable AR8035 ouput a 125MHz clk from CLK_25M */
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x8016);
	phy_write(phydev, 0xd, 0x4007);
	val = phy_read(phydev, 0xe);

	val &= 0xffe3;
	val |= 0x18;
	phy_write(phydev, 0xe, val);
	/* Introduce tx clock delay */
	phy_write(phydev, 0x1d, 0x5);
	val = phy_read(phydev, 0x1e);
	val |= 0x0100;
	phy_write(phydev, 0x1e, val);
	/*check phy power*/
	val = phy_read(phydev, 0x0);

	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}
static int mx6q_usom_fec_ar8030_phy_init(struct phy_device *phydev)
{
	unsigned short val;
	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	gpio_set_value(USOM_ENET_RST, 1);
	mdelay(10);
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x805d);
	phy_write(phydev, 0xd, 0x4003);
	val = phy_read(phydev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(phydev, 0xe, val);
	if (val & BMCR_PDOWN)
		phy_write(phydev, 0x0, (val & ~BMCR_PDOWN));

	return 0;
}

static struct fec_platform_data fec_data_rgmii __initdata = {
	.init = mx6q_usom_fec_ar8035_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};
static struct fec_platform_data fec_data_rmii __initdata = {
	.init = mx6q_usom_fec_ar8030_phy_init,
	.phy = PHY_INTERFACE_MODE_RMII,
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	.gpio_irq = MX6_ENET_IRQ,
#endif
};

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_usom_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_usom_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_usom_sata_data = {
	.init = mx6q_usom_sata_init,
	.exit = mx6q_usom_sata_exit,
};
#endif

struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};

/* Needs refinment to configure only the internal supply */
static struct regulator_consumer_supply usom_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data usom_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(usom_vmmc_consumers),
	.consumer_supplies = usom_vmmc_consumers,
};

static struct fixed_voltage_config usom_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &usom_vmmc_init,
};

static struct platform_device usom_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &usom_vmmc_reg_config,
	},
};

struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct ipuv3_fb_platform_data usom_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "hdmi",
	.interface_pix_fmt = IPU_PIX_FMT_RGB24,
	.mode_str = "1920x1080M@60",
	.default_bpp = 32,
	.int_clk = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	}, {
	.disp_dev = "ldb",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	.late_init = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	if (hdmi_SDMA_check())
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On microsom board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_usom_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_usom_hdmi_ddc_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_usom_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_usom_hdmi_ddc_pads));
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_usom_i2c2_pads,
			ARRAY_SIZE(mx6dl_usom_i2c2_pads));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_usom_i2c2_pads,
			ARRAY_SIZE(mx6q_usom_i2c2_pads));
}

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 0,
	.disp_id = 0,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 1,
	.ext_ref = 1,
	.mode = LDB_SEP1,
	.sec_ipu_id = 1,
	.sec_disp_id = 0,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	}, {
	.rev = 4,
	.csi_clk[0] = "clko_clk",
	.bypass_reset = false,
	},
};

#ifdef CONFIG_ANDROID
static struct ion_platform_data imx_ion_data = {
	.nr = 1,
	.heaps = {
		{
		.id = 0,
		.type = ION_HEAP_TYPE_CARVEOUT,
		.name = "vpu_ion",
		.size = SZ_16M,
		.cacheable = 1,
		},
	},
};
#endif

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
	.phy_reg_vlev = 0x0294,
	.phy_reg_cksymtx = 0x800d,
};
static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};

void __init fixup_usom_board(struct machine_desc *desc, struct tag *tags,
			   char **cmdline, struct meminfo *mi)
{
	char *str;
	struct tag *t;
	int i = 0;
	struct ipuv3_fb_platform_data *pdata_fb = usom_fb_data;

	for_each_tag(t, tags) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fbmem=");
			if (str != NULL) {
				str += 6;
				pdata_fb[i++].res_size[0] = memparse(str, &str);
				while (*str == ',' &&
					i < ARRAY_SIZE(usom_fb_data)) {
					str++;
					pdata_fb[i++].res_size[0] = memparse(str, &str);
				}
			}
#ifdef CONFIG_ANDROID
			/* ION reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "ionmem=");
			if (str != NULL) {
				str += 7;
				imx_ion_data.heaps[0].size = memparse(str, &str);
			}
#endif
			/* Primary framebuffer base address */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "fb0base=");
			if (str != NULL) {
				str += 8;
				pdata_fb[0].res_base[0] =
						simple_strtol(str, &str, 16);
			}
			/* GPU reserved memory */
			str = t->u.cmdline.cmdline;
			str = strstr(str, "gpumem=");
			if (str != NULL) {
				str += 7;
				imx6q_gpu_pdata.reserved_mem_size = memparse(str, &str);
			}
			break;
		}
	}
}

static struct mxc_dvfs_platform_data usom_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

static int usom_bt_power_change(int status)
{
	if(status){
		printk(KERN_INFO "%s : Bluetooth power on\n",__FUNCTION__);
		gpio_set_value(USOM_BT_RST, 1);
		msleep(100);
	}
	else{
		printk(KERN_INFO "%s : Bluetooth power off\n",__FUNCTION__);
		gpio_set_value(USOM_BT_RST, 0);
		msleep(11);
	}
	return 0;
}


static struct platform_device usom_bt_rfkill = {
	.name = "mxc_bt_rfkill",
};

static struct imx_bt_rfkill_platform_data usom_bt_rfkill_data = {
	.power_change = usom_bt_power_change,
};

static const struct imxuart_platform_data usom_bt_uart_data = {
        .flags = IMXUART_HAVE_RTSCTS | IMXUART_SDMA,
        .dma_req_tx = MX6Q_DMA_REQ_UART4_TX,
        .dma_req_rx = MX6Q_DMA_REQ_UART4_RX,
};


static void __init imx6q_usom_init_wifi_bt(void) {
	/* init and put in reset */
	gpio_request(USOM_WL_RST, "wl-rst");
	gpio_direction_output(USOM_WL_RST, 0);
	gpio_request(USOM_BT_RST, "bt-rst");
	gpio_direction_output(USOM_BT_RST, 0);
	gpio_request(USOM_BT_SHUTDOWN, "bt-shutdown");
	gpio_direction_output(USOM_BT_SHUTDOWN, 0);
	gpio_request(USOM_REG_ON, "wl-bt-reg-on");
	gpio_direction_output(USOM_REG_ON, 0);
	gpio_request(USOM_XTAL_ON, "wl-bt-xtal-on");
	gpio_direction_output(USOM_XTAL_ON, 0);
	msleep(1);
	gpio_set_value(USOM_REG_ON, 1);
	gpio_set_value(USOM_XTAL_ON, 1);

	
	/* Now release from reset */
	msleep (2); /* 20 mSec sounds too big */
	gpio_set_value(USOM_WL_RST, 1);
	gpio_set_value(USOM_BT_RST, 1);
	gpio_set_value(USOM_BT_SHUTDOWN, 1);
	msleep (2); /* 20 mSec sounds too big */

	/* Register SDIO as brfmac */
        imx6q_add_imx_uart(3, &usom_bt_uart_data);
	mxc_register_device(&usom_bt_rfkill, &usom_bt_rfkill_data);
	imx6q_add_sdhci_usdhc_imx(0, &mx6q_usom_sd1_data);
}

void __init imx6q_usom_init_fec(void) {
	struct clk *enet;
	/* Set GPR1, bit 21 to 1 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
	/* Set enet clock to 25MHz for RGMII phy */
	enet = clk_get_sys("fec_clk.0", NULL);
	if (IS_ERR(enet))
		pr_err("Unable to get enet.0 clock\n");
	else {
		clk_prepare(enet);
		clk_set_rate(enet, 25000000);
		clk_enable(enet);
	}
	if (cpu_is_mx6dl())
		mxc_iomux_v3_setup_multiple_pads(mx6dl_usom_ar8035_phy,
			ARRAY_SIZE(mx6dl_usom_ar8035_phy));
	else
		mxc_iomux_v3_setup_multiple_pads(mx6q_usom_ar8035_phy,
			ARRAY_SIZE(mx6q_usom_ar8035_phy));
	gpio_request(USOM_ENET_RST, "eth-phy-rst");
	udelay(1000); // Maybe not needed since 0 value is already asserted (pull down)
	gpio_direction_output(USOM_ENET_RST, 0);
	udelay(2000); // Maybe not needed since 0 value is already asserted (pull down)
	gpio_set_value(USOM_ENET_RST, 1);
	mdelay(10);
	imx6_init_fec(fec_data_rgmii);
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
	/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
	mxc_iomux_set_specialbits_register(IOMUX_OBSRV_MUX1_OFFSET,
		OBSRV_MUX1_ENET_IRQ, OBSRV_MUX1_MASK);
#endif
}
/*
 * Board specific initialization.
 */
void __init mx6_usom_board_init(void)
{
	int i;
	struct platform_device *voutdev;
	struct clk *clko, *clko2;
	struct clk *new_parent;
	int rate;

	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_usom_pads,
			ARRAY_SIZE(mx6q_usom_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_usom_pads,
			ARRAY_SIZE(mx6dl_usom_pads));
	}

	gp_reg_id = usom_dvfscore_data.reg_id;
	soc_reg_id = usom_dvfscore_data.soc_id;

	/* Now this is reall dangerous !!! */
	/* TODO - Enable LDO bypass mode */
	/*
	 * MX6DL/Solo only supports single IPU
	 * The following codes are used to change ipu id
	 * and display id information for MX6DL/Solo. Then
	 * register 1 IPU device and up to 2 displays for
	 * MX6DL/Solo
	 */
	if (cpu_is_mx6dl()) {
		ldb_data.ipu_id = 0;
		ldb_data.sec_ipu_id = 0;
	}
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(usom_fb_data); i++)
			imx6q_add_ipuv3fb(i, &usom_fb_data[i]);
	} else
		for (i = 0; i < 2 && i < ARRAY_SIZE(usom_fb_data); i++)
			imx6q_add_ipuv3fb(i, &usom_fb_data[i]);

	imx6q_add_vdoa();
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}
	imx6q_add_v4l2_capture(0, &capture_data[0]);
	imx6q_add_v4l2_capture(1, &capture_data[1]);
	imx6q_add_imx_snvs_rtc();

	if (1 == caam_enabled)
		imx6q_add_imx_caam();


	/* I2C2 / HDMI DDC */
	imx6q_add_imx_i2c(1, &mx6q_usom_i2c_data);
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_usom_anatop_thermal_data);
	/* Set IPU AXI transactions with QoS = 0xf as a real time access */
	mxc_iomux_set_gpr_register(6, 0, 32, 0xffffffff);

	imx6q_usom_init_fec();
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	/* SATA is not supported by MX6DL/Solo */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_usom_sata_data);
#else
		mx6q_usom_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	imx6q_add_vpu();
	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&usom_dvfscore_data);
	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	if (cpu_is_mx6dl()) {
		imx6dl_add_imx_pxp();
		imx6dl_add_imx_pxp_client();
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);

	/* Camera and audio use osc clock */
	clko = clk_get(NULL, "clko_clk");
	if (!IS_ERR(clko))
		clk_set_parent(clko, clko2);

//	pm_power_off = mx6_snvs_poweroff;

	imx6q_add_busfreq();

	imx6_add_armpmu();
	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);
	/* WLan and BT stuff */
	if (disable_wifi_detect != 1)
		imx6q_usom_init_wifi_bt();
}

extern void __iomem *twd_base;
static void __init mx6_usom_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

struct sys_timer mx6_usom_timer = {
	.init   = mx6_usom_timer_init,
};
void __init mx6q_usom_reserve(void)
{
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)
	phys_addr_t phys;
	if (imx6q_gpu_pdata.reserved_mem_size) {
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
	if (vout_mem.res_msize) {
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, SZ_1G);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}

}

static int __init disable_wifi_detect_func(char *p)
{
	disable_wifi_detect = 1;
	return 0;
}

early_param("disable_wifi_detect", disable_wifi_detect_func);

