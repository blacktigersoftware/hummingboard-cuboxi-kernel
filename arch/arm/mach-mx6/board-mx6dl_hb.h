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

#ifndef _BOARD_MX6DL_HB_H
#define _BOARD_MX6DL_HB_H

#include <mach/iomux-mx6dl.h>

static iomux_v3_cfg_t mx6dl_hb_pads[] = {
	/* SPDIF out */
	MX6DL_PAD_GPIO_17__SPDIF_OUT1,

	/* CAN1  */
	MX6DL_PAD_SD3_CLK__CAN1_RXCAN,
	MX6DL_PAD_SD3_CMD__CAN1_TXCAN,

	/* CCM  */
	MX6DL_PAD_GPIO_5__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6DL_PAD_NANDF_CS2__CCM_CLKO2,		/* MIPI CSI clock */

	/* ECSPI2 */
	MX6DL_PAD_EIM_CS0__ECSPI2_SCLK,
	MX6DL_PAD_EIM_CS1__ECSPI2_MOSI,
	MX6DL_PAD_EIM_OE__ECSPI2_MISO,
	MX6DL_PAD_EIM_RW__ECSPI2_SS0,
	MX6DL_PAD_EIM_LBA__ECSPI2_SS1,

	/* I2C1 */
	MX6DL_PAD_EIM_D21__I2C1_SCL,
	MX6DL_PAD_EIM_D28__I2C1_SDA,

	/* I2C3 */
	MX6DL_PAD_EIM_D17__I2C3_SCL,
	MX6DL_PAD_EIM_D18__I2C3_SDA,

	/* PWM */
	MX6DL_PAD_DISP0_DAT8__PWM1_PWMO,
	MX6DL_PAD_DISP0_DAT9__PWM2_PWMO,
	MX6DL_PAD_SD4_DAT1__PWM3_PWMO,
	MX6DL_PAD_SD4_DAT2__GPIO_2_10,

	/* UART1 for debug */
	MX6DL_PAD_CSI0_DAT10__UART1_TXD,
	MX6DL_PAD_CSI0_DAT11__UART1_RXD,

	/* UART2 RX, TX, CTS and RTS */
	MX6DL_PAD_SD4_DAT4__UART2_RXD,
	MX6DL_PAD_SD4_DAT5__UART2_RTS,
	MX6DL_PAD_SD4_DAT6__UART2_CTS,
	MX6DL_PAD_SD4_DAT7__UART2_TXD,

	/* USBOTG ID pin
	 * This controls RX_ER pad to be USB OTG ID and internally pull it down
	 * thus forcing it to be USB host.
	 * A better way to do this is by chaning the driver itself to accept
	 * a flag indicating that this USB OTG will work only in host mode.
	 */
	IOMUX_PAD(0x05C4, 0x01F4, 0, 0x0790, 1, PAD_CTL_PKE | PAD_CTL_PUE | \
		PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW |	\
		PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS),

	/* USB power enable pins */
	MX6DL_PAD_EIM_D22__GPIO_3_22,
	MX6DL_PAD_GPIO_0__GPIO_1_0,

	/* USB OC pin */
	MX6DL_PAD_KEY_COL4__USBOH3_USBOTG_OC,
	MX6DL_PAD_GPIO_3__USBOH3_USBH1_OC,

	/* USDHC2 */
	MX6DL_PAD_SD2_CLK__USDHC2_CLK,
	MX6DL_PAD_SD2_CMD__USDHC2_CMD,
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6DL_PAD_GPIO_4__USDHC2_CD,		/* SD2_CD */

	/* IR in */
	MX6DL_PAD_EIM_DA5__GPIO_3_5,

	/* AUD5 to SGTL5000 audio codec */
	MX6DL_PAD_KEY_COL0__AUDMUX_AUD5_TXC,
	MX6DL_PAD_KEY_ROW0__AUDMUX_AUD5_TXD,
	MX6DL_PAD_KEY_COL1__AUDMUX_AUD5_TXFS,
	MX6DL_PAD_DISP0_DAT19__AUDMUX_AUD5_RXD,
	MX6DL_PAD_GPIO_5__CCM_CLKO,

	/* Camera interface */
	MX6Q_PAD_NANDF_CS2__CCM_CLKO2,

	/* PCI-E signals */
	MX6DL_PAD_EIM_DA4__GPIO_3_4,	/* Reset */
	MX6DL_PAD_KEY_ROW1__GPIO_4_9,	/* Disable */

	/* 26 pin header GPIOs */
	MX6DL_PAD_GPIO_1__GPIO_1_1,
	MX6DL_PAD_EIM_DA8__GPIO_3_8,
	MX6DL_PAD_EIM_DA9__GPIO_3_9,
	MX6DL_PAD_EIM_DA7__GPIO_3_7,
	MX6DL_PAD_EIM_DA6__GPIO_3_6,
	MX6DL_PAD_SD3_CMD__GPIO_7_2,
	MX6DL_PAD_SD3_CLK__GPIO_7_3,
	MX6DL_PAD_EIM_DA3__GPIO_3_3,
};

#endif
