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

#ifndef _BOARD_MX6Q_HB_H
#define _BOARD_MX6Q_HB_H

#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_hb_pads[] = {
	/* SPDIF out */
	MX6Q_PAD_GPIO_17__SPDIF_OUT1,

	/* CAN1  */
	MX6Q_PAD_SD3_CLK__CAN1_RXCAN,
	MX6Q_PAD_SD3_CMD__CAN1_TXCAN,

	/* CCM  */
	MX6Q_PAD_GPIO_5__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6Q_PAD_NANDF_CS2__CCM_CLKO2,		/* MIPI CSI clock */

	/* ECSPI2 */
	MX6Q_PAD_EIM_CS0__ECSPI2_SCLK,
	MX6Q_PAD_EIM_CS1__ECSPI2_MOSI,
	MX6Q_PAD_EIM_OE__ECSPI2_MISO,
	MX6Q_PAD_EIM_RW__ECSPI2_SS0,
	MX6Q_PAD_EIM_LBA__ECSPI2_SS1,

	/* I2C1 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,
	MX6Q_PAD_EIM_D28__I2C1_SDA,

	/* I2C3 */
	MX6Q_PAD_EIM_D17__I2C3_SCL,
	MX6Q_PAD_EIM_D18__I2C3_SDA,

	/* PWM */
	MX6Q_PAD_DISP0_DAT8__PWM1_PWMO,
	MX6Q_PAD_DISP0_DAT9__PWM2_PWMO,
	MX6Q_PAD_SD4_DAT1__PWM3_PWMO,
	MX6Q_PAD_SD4_DAT2__PWM4_PWMO,

	/* UART1 for debug */
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,

	/* UART2 RX, TX, CTS and RTS */
	MX6Q_PAD_SD4_DAT4__UART2_RXD,
	MX6Q_PAD_SD4_DAT5__UART2_RTS,
	MX6Q_PAD_SD4_DAT6__UART2_CTS,
	MX6Q_PAD_SD4_DAT7__UART2_TXD,

	/* USBOTG ID pin */
	IOMUX_PAD(0x05F4, 0x0224, 3, 0x0000, 0, (PAD_CTL_PKE | PAD_CTL_PUE | \
		PAD_CTL_PUS_100K_DOWN | PAD_CTL_SPEED_LOW |	\
		PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST | PAD_CTL_HYS)),
	/*
	 * Originally MX6Q_PAD_GPIO_1__USBOTG_ID, but we want it to be
	 * pulled down ID pin for fixed host connection.
	 */

	/* USB power enable pins */
	MX6Q_PAD_EIM_D22__GPIO_3_22,
	MX6Q_PAD_GPIO_0__GPIO_1_0,

	/* USB OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,
	MX6Q_PAD_GPIO_3__USBOH3_USBH1_OC,

	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6Q_PAD_GPIO_4__USDHC2_CD,		/* SD2_CD */

	/* IR in */
	MX6Q_PAD_GPIO_2__GPIO_1_2,
};

#endif
