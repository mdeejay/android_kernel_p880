/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define PN544_MAGIC	0xE9

#ifdef CONFIG_LGE_NFC_PN544
/*
#define PN544_DRV_NAME		CONFIG_LGE_NFC_DEV_NAME
#define NFC_GPIO_VEN		CONFIG_LGE_NFC_GPIO_VEN
#define NFC_GPIO_IRQ		CONFIG_LGE_NFC_GPIO_IRQ
#define NFC_GPIO_FIRM		CONFIG_LGE_NFC_GPIO_MODE
#define NFC_I2C_SLAVEADDR	CONFIG_LGE_NFC_GPIO_I2C
*/
#define PN544_DRV_NAME		"pn544"
#define NFC_GPIO_VEN		189 //TEGRA_GPIO_PX5
#define NFC_GPIO_IRQ		178 //TEGRA_GPIO_PW2		178
#if defined(MACH_X3_REV_A) || defined(MACH_X3_REV_B) || defined(MACH_X3_REV_C)
#define NFC_GPIO_FIRM 			25 //TEGRA_GPIO_PD1		25
#else
#define NFC_GPIO_FIRM 			136 //TEGRA_GPIO_PR0  136 : NFC MODE
#endif
#define NFC_I2C_SLAVE_ADDR	0x28
#endif

/*
 * PN544 power control via ioctl
 * PN544_SET_PWR(0): power off
 * PN544_SET_PWR(1): power on
 * PN544_SET_PWR(2): reset and power on with firmware download enabled
 */
#define PN544_SET_PWR	_IOW(PN544_MAGIC, 0x01, unsigned int)

struct pn544_i2c_platform_data {
	unsigned int irq_gpio;
	unsigned int ven_gpio;
	unsigned int firm_gpio;
};
