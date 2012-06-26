/*
 * Copyright (C) 2011 LG Electronics Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LGE_BATTERY_H__
#define __LGE_BATTERY_H__

struct lge_battery_platform_data {
	char	*gauge_name;
	char	*charger_name;
#if defined(CONFIG_ADC_TSC2007)  //LGE_CHANGE pyocool.cho@lge.com
	char	*adc_name;//LGE_CHANGE pyocool.cho@lge.com "for ADC"
#endif	
};
#endif	// __LGE_BATTERY_H__

typedef struct __battery_graph_prop
{
	s32 x;
	s32 y;
} battery_graph_prop;

typedef enum {
        DISCHARGING_OFF = 0, //Temperature is reached proper degrees.
        DISCHARGING_ON, //dischaging cause of high or low temperature.
}recharging_state_t;
