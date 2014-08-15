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

#ifndef _BOARD_MX6Q_CUBOX_I_H
#define _BOARD_MX6Q_CUBOX_I_H

#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_cubox_i_pads[] = {
	/* SPDIF out */
	MX6Q_PAD_GPIO_17__SPDIF_OUT1,

	/* I2C3 */
	MX6Q_PAD_EIM_D17__I2C3_SCL,
	MX6Q_PAD_EIM_D18__I2C3_SDA,

	/* PWM */
	MX6Q_PAD_DISP0_DAT8__PWM1_PWMO,
	MX6Q_PAD_DISP0_DAT9__PWM2_PWMO,

	/* UART1 for debug */
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,

	/* USB power enable pins */
	MX6Q_PAD_EIM_D22__GPIO_3_22,
	MX6Q_PAD_GPIO_0__GPIO_1_0,

	/* USB OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,
	MX6Q_PAD_GPIO_3__USBOH3_USBH1_OC,

	/* USB OTG ID */
	MX6Q_PAD_GPIO_1__USBOTG_ID,

	/* USDHC2 */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6Q_PAD_GPIO_4__USDHC2_CD,		/* SD2_CD */
	MX6Q_PAD_GPIO_2__USDHC2_WP,

	/* IR in */
	MX6Q_PAD_EIM_DA9__GPIO_3_9 
};

#endif
