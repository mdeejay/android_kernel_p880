/*
 * arch/arm/mach-tegra/board-x3.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_X3_H
#define _MACH_TEGRA_BOARD_X3_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps80031.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/syscalls.h>

#define MISC_PARTITION				"/dev/block/platform/sdhci-tegra.3/by-name/MSC"
#define MISC_MSG_LENGTH 32
#define MISC_MSG_BASE_OFFSET 0x1000
typedef enum{
    msg_type_misc = 0,
    msg_type_webdn,
    msg_type_muic_path,
    msg_type_smpl,
    msg_type_gang,
    msg_type_chg,
    msg_type_bootcause,	
    msg_type_max_count
}misc_msg_type;
#define MISC_MSG_COUNT msg_type_max_count
#define MISC_MSG_SIZE MISC_MSG_COUNT * MISC_MSG_LENGTH

typedef enum{
    hw_rev_pcb_type_A = 0,
    hw_rev_pcb_type_B,
    hw_rev_pcb_type_C,
    hw_rev_pcb_type_D,
    hw_rev_pcb_type_E,
    hw_rev_pcb_type_1_0,
    hw_rev_pcb_type_1_1,
    hw_rev_pcb_type_1_2,
    hw_rev_pcb_type_1_3,
    hw_rev_pcb_type_MAX
}hw_rev_pcb_type;


int x3_charge_init(void);
int x3_sdhci_init(void);
int x3_pinmux_init(void);
int x3_panel_init(void);
int x3_sensors_init(void);
int touch_init(void);
int x3_kbc_init(void);
int x3_emc_init(void);
int x3_regulator_init(void);
int x3_modem_init(void);
int x3_suspend_init(void);
int x3_edp_init(void);
void __init x3_tsensor_init(void);
#ifdef CONFIG_TEGRA_BPC_MGMT
void x3_bpc_mgmt_init(void);
#endif
int x3_sensor_input_init(void);
int get_misc_msg(misc_msg_type msg, char* misc_msg, int size);
int set_misc_msg(misc_msg_type msg, char* misc_msg, int size);
hw_rev_pcb_type x3_get_hw_rev_pcb_version(void);


#define ENT_TPS80031_GPIO_BASE	   TEGRA_NR_GPIOS
#define ENT_TPS80031_GPIO_REGEN1 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN1)
#define ENT_TPS80031_GPIO_REGEN2 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_REGEN2)
#define ENT_TPS80031_GPIO_SYSEN	 (ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_SYSEN)
#define ENT_TPS80031_GPIO_END	(ENT_TPS80031_GPIO_BASE + TPS80031_GPIO_NR)

/* External peripheral irq base */
#define ENT_TPS80031_IRQ_BASE	TEGRA_NR_IRQS
#define ENT_TPS80031_IRQ_END  (ENT_TPS80031_IRQ_BASE + TPS80031_INT_NR)

/* Audio-related GPIOs */
#define TEGRA_GPIO_HP_DET	TEGRA_GPIO_PBB6

#define TEGRA_GPIO_HP_HOOK	TEGRA_GPIO_PO5
#define TEGRA_GPIO_EAR_MIC	TEGRA_GPIO_PX0
#define TEGRA_GPIO_SUB_MIC	TEGRA_GPIO_PI6

#define BOARD_1205		(0x0C05)
#define BOARD_E1197		(0x0B61)
#define X3_FAB_A01	(0x01)
#define SKU_BATTERY_SUPPORT	(1 << 8)

#define MODEM_PWR_ON		TEGRA_GPIO_PO0
#define MODEM_RESET		TEGRA_GPIO_PV1

#define CP2AP_ACK1_HOST_ACTIVE			TEGRA_GPIO_PU5
#define CP2AP_ACK2_HOST_WAKEUP			TEGRA_GPIO_PV0
#define AP2CP_ACK2_SUSPEND_REQ			TEGRA_GPIO_PS6
#define AP2CP_ACK1_SLAVE_WAKEUP			TEGRA_GPIO_PS7

#if defined(CONFIG_MFD_MAX77663)
#define MAX77663_GPIO_BASE		TEGRA_NR_GPIOS
#define MAX77663_IRQ_BASE		TEGRA_NR_IRQS
#endif /* defined(CONFIG_MFD_MAX77663) */

#define TDIODE_OFFSET	(9000)	/* in millicelsius */

#ifdef CONFIG_TEGRA_BPC_MGMT
/* Battery Peak Current Management */
#define TEGRA_BPC_TRIGGER		TEGRA_GPIO_PJ6
#define TEGRA_BPC_TIMEOUT		200 /* ms */
#define TEGRA_BPC_CPU_PWR_LIMIT	0 /* in mW, (0 disables) */
#endif

#define TEGRA_CUR_MON_THRESHOLD		2500
#define TEGRA_CUR_MON_RESISTOR		10
#define TEGRA_CUR_MON_MIN_CORES		2


#endif /*_MACH_TEGRA_BOARD_X3_H */
