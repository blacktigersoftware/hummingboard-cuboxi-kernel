/*
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
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#ifdef CONFIG_IR_GPIO_CIR
#include <media/gpio-ir-recv.h>
#endif

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_asrc.h>
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
#include "board-mx6q_hb.h"
#include "board-mx6dl_hb.h"

#define CAM_GPIO		IMX_GPIO_NR(2, 10)
#define GPIO_IR_IN		IMX_GPIO_NR(3, 5)
#define HB_USB_OTG_PWR		IMX_GPIO_NR(3, 22)
#define HB_USB_H1_PWR		IMX_GPIO_NR(1, 0)
#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(4, 18) /* TODO */
#endif

#ifdef CONFIG_IMX_PCIE
#define HB_PCIE_RST		IMX_GPIO_NR(3, 4)
#define HB_PCIE_DIS		IMX_GPIO_NR(4, 9)
#endif

#define HEADER26_PIN07		IMX_GPIO_NR(1, 1)
#define HEADER26_PIN11		IMX_GPIO_NR(3, 9)
#define HEADER26_PIN12		IMX_GPIO_NR(3, 8)
#define HEADER26_PIN13		IMX_GPIO_NR(3, 7)
#define HEADER26_PIN15		IMX_GPIO_NR(3, 6)
#define HEADER26_PIN16		IMX_GPIO_NR(7, 2)
#define HEADER26_PIN18		IMX_GPIO_NR(7, 3)
#define HEADER26_PIN22		IMX_GPIO_NR(3, 3)

static int caam_enabled;
extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

static const struct esdhc_platform_data mx6q_hb_sd2_data __initconst = {
	.keep_power_at_suspend = 1,
	.support_8bit = 0,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};
static struct mxc_audio_platform_data mx6_hb_audio_data;

static int mx6_hb_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_hb_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_hb_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_hb_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 5,
	.init = mx6_hb_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_hb_audio_device = {
	.name = "imx-sgtl5000",
};

static struct imxi2c_platform_data mx6q_hb_i2c_data = {
	.bitrate = 100000,
};

/* I2C1 */
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
	{
		I2C_BOARD_INFO("pcf8523", 0x68),
	},
};

/* I2C3 */
static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
};

static const struct imx_pcie_platform_data mx6_hb_pcie_data __initconst = {
	.pcie_pwr_en	= -EINVAL,
	.pcie_rst	= HB_PCIE_RST,
	.pcie_wake_up	= -EINVAL,
	.pcie_dis	= HB_PCIE_DIS,
};

static void imx6q_hb_usbotg_vbus(bool on)
{
	if (on)
		gpio_set_value(HB_USB_OTG_PWR, 1);
	else
		gpio_set_value(HB_USB_OTG_PWR, 0);
}

static void imx6q_hb_host1_vbus(bool on)
{
	if (on)
		gpio_set_value(HB_USB_H1_PWR, 1);
	else
		gpio_set_value(HB_USB_H1_PWR, 0);
}

static void __init imx6q_hb_init_usb(void)
{
	int ret = 0;
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(HB_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO HB_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(HB_USB_OTG_PWR, 0);
	/* keep USB host1 VBUS always on */
	ret = gpio_request(HB_USB_H1_PWR, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO HB_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(HB_USB_H1_PWR, 0);
	/*
	 * ID pin is fake sampled from RX_ER pin. Notice that this pad is configured
	 * to be pulled-down 100kOhm by default.
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	mx6_set_otghost_vbus_func(imx6q_hb_usbotg_vbus);
	mx6_set_host1_vbus_func(imx6q_hb_host1_vbus);
}

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


static void mx6q_mipi_sensor_io_init(void)
{
	struct clk *clko2;
	struct clk *new_parent;
	int rate, ret;
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
	ret = gpio_request(CAM_GPIO, "cam-gpio");
	if (ret) {
		pr_err("failed to get GPIO CAM_GPIO: %d\n",
			ret);
		return;
	}
	gpio_direction_output(CAM_GPIO, 0);
	msleep(5);
	gpio_set_value(CAM_GPIO, 1);
	msleep(5);
	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 0);
	else if (cpu_is_mx6dl())
		mxc_iomux_set_gpr_register(13, 0, 3, 0);
}

static void hb_mipi_sensor_reset(int powerdown)
{
        if (powerdown)
                gpio_set_value(CAM_GPIO, 0);
        else
                gpio_set_value(CAM_GPIO, 1);

        msleep(2);
}

static struct fsl_mxc_camera_platform_data mipi_csi2_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_mipi_sensor_io_init,
/*	.pwdn = hb_mipi_sensor_reset, */
};

static struct regulator_consumer_supply hb_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
};

static struct regulator_init_data hb_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(hb_vmmc_consumers),
	.consumer_supplies = hb_vmmc_consumers,
};

static struct fixed_voltage_config hb_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &hb_vmmc_init,
};

static struct platform_device hb_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &hb_vmmc_reg_config,
	},
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

static int spdif_clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_actual;
	rate_actual = clk_round_rate(clk, rate);
	clk_set_rate(clk, rate_actual);
	return 0;
}

static struct mxc_spdif_platform_data mxc_spdif_data = {
	.spdif_tx		= 1,		/* enable tx */
	.spdif_rx		= 0,		/* enable rx */
	/*
	 * spdif0_clk will be 454.7MHz divided by ccm dividers.
	 *
	 * 44.1KHz: 454.7MHz / 7 (ccm) / 23 (spdif) = 44,128 Hz ~ 0.06% error
	 * 48KHz:   454.7MHz / 4 (ccm) / 37 (spdif) = 48,004 Hz ~ 0.01% error
	 * 32KHz:   454.7MHz / 6 (ccm) / 37 (spdif) = 32,003 Hz ~ 0.01% error
	 */
	.spdif_clk_44100	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_clk_48000	= 1,    /* tx clk from spdif0_clk_root */
	.spdif_div_44100	= 23,
	.spdif_div_48000	= 37,
	.spdif_div_32000	= 37,
	.spdif_clk_set_rate	= spdif_clk_set_rate,
	.spdif_clk		= NULL, /* spdif bus clk */
};

static struct imx_esai_platform_data sab_esai_pdata = {
	.flags	= IMX_ESAI_NET,
};
#ifdef CONFIG_SND_SOC_SGTL5000 /* To be remved */

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "0-000a",
};

static struct regulator_init_data sgtl5000_sabrelite_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabrelite_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabrelite_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 2500000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabrelite_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */
static int __init imx6q_init_audio(void)
{
	struct clk *pll3_pfd, *esai_clk;
	imx6q_add_imx_esai(0, &sab_esai_pdata);

	esai_clk = clk_get(NULL, "esai_clk");
	if (IS_ERR(esai_clk))
		return PTR_ERR(esai_clk);

	pll3_pfd = clk_get(NULL, "pll3_pfd_508M");
	if (IS_ERR(pll3_pfd))
		return PTR_ERR(pll3_pfd);

	clk_set_parent(esai_clk, pll3_pfd);
	clk_set_rate(esai_clk, 101647058);
	mxc_register_device(&mx6_hb_audio_device,
			    &mx6_hb_audio_data);
	imx6q_add_imx_ssi(1, &mx6_hb_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_sabrelite_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddd_reg_devices);
#endif
	return 0;
}

#ifdef CONFIG_IR_GPIO_CIR
static struct gpio_ir_recv_platform_data hb_ir_data = {
	.gpio_nr = GPIO_IR_IN,
	.active_low = 1,
};

static struct platform_device hb_ir = {
        .name   = "gpio-rc-recv",
	.id     = -1,
	.dev    = {
		.platform_data  = &hb_ir_data,
	}
};
#endif

static struct platform_pwm_backlight_data mx6_hb_pwm_lvds_backlight_data = {
	.pwm_id = 2,
	.max_brightness = 248,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

/*
 * Board specific initialization.
 */
/* Following to be exported to an .h file for the microsom */
extern void __init mx6_usom_board_init(void); 
extern void __init fixup_usom_board(struct machine_desc *desc, struct tag *tags,
					char **cmdline, struct meminfo *mi);
extern void __init mx6q_usom_reserve(void);
extern struct sys_timer mx6_usom_timer;

static void __init mx6_hb_board_init(void)
{
	int i;
	if (cpu_is_mx6q())
		mxc_iomux_v3_setup_multiple_pads(mx6q_hb_pads,
			ARRAY_SIZE(mx6q_hb_pads));
	else if (cpu_is_mx6dl()) {
		mxc_iomux_v3_setup_multiple_pads(mx6dl_hb_pads,
			ARRAY_SIZE(mx6dl_hb_pads));
	}
	mx6_usom_board_init();
	imx6q_add_imx_uart(0, NULL);

	for (i = 0; i < ARRAY_SIZE(capture_data); i++) {
		if (!cpu_is_mx6q())
			capture_data[i].ipu = 0;
		imx6q_add_v4l2_capture(i, &capture_data[i]);
	}
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_i2c(0, &mx6q_hb_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_hb_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info, /* I2C1 */
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info, /* I2C3 */
			ARRAY_SIZE(mxc_i2c2_board_info));

	imx6q_add_sdhci_usdhc_imx(1, &mx6q_hb_sd2_data);
	imx6q_hb_init_usb();

	imx6q_init_audio();
	platform_device_register(&hb_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);

	imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	imx6q_add_mxc_pwm(2);
	imx6q_add_mxc_pwm(3);

	/* LVDS backlight */
	imx6q_add_mxc_pwm_backlight(0, &mx6_hb_pwm_lvds_backlight_data);

	mxc_spdif_data.spdif_core_clk = clk_get_sys("mxc_spdif.0", NULL);
	clk_put(mxc_spdif_data.spdif_core_clk);
	imx6q_add_spdif(&mxc_spdif_data);
	imx6q_add_spdif_dai();
	imx6q_add_spdif_audio_device();

	/* Add PCIe RC interface support */
	imx6q_add_pcie(&mx6_hb_pcie_data);
#ifdef CONFIG_IR_GPIO_CIR
	/* Register the infra red receiver as a GPIO device */
	platform_device_register(&hb_ir);
#endif
	/* 26 pin header gpio config */
	gpio_request (HEADER26_PIN07, "pin07");
	gpio_request (HEADER26_PIN11, "pin11");
	gpio_request (HEADER26_PIN12, "pin12");
	gpio_request (HEADER26_PIN13, "pin13");
	gpio_request (HEADER26_PIN15, "pin15");
	gpio_request (HEADER26_PIN16, "pin16");
	gpio_request (HEADER26_PIN18, "pin18");
	gpio_request (HEADER26_PIN22, "pin22");
	gpio_export (HEADER26_PIN07, 1);
	gpio_export (HEADER26_PIN11, 1);
	gpio_export (HEADER26_PIN12, 1);
	gpio_export (HEADER26_PIN13, 1);
	gpio_export (HEADER26_PIN15, 1);
	gpio_export (HEADER26_PIN16, 1);
	gpio_export (HEADER26_PIN18, 1);
	gpio_export (HEADER26_PIN22, 1);
}

/*
 * initialize __mach_desc_MX6Q_HB data structure.
 */
MACHINE_START(HB, "SolidRun i.MX6 Quad/Dual/DualLite/Solo HummingBoard")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_usom_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_hb_board_init,
	.timer = &mx6_usom_timer,
	.reserve = mx6q_usom_reserve,
MACHINE_END
