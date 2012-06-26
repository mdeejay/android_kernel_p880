/*
 *  lge_battery.c
 *
 *  LGE Battery Charger Interface Driver
 *
 *  Copyright (C) 2011 LG Electronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/android_alarm.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>

#include <linux/power/lge_battery.h>
#include "../../arch/arm/mach-tegra/board.h"`

#define POLLING_INTERVAL	(10 * HZ)
#define POLLING_INTERVAL_DISCHG (20 * HZ)
#define CAMERA_STATE_ROOT	"/sys/devices/platform/tegra_camera/power_save_rec"

// For Battery Scenario Start
#define FOR_LG_TEMPERATURE_SCENARIO 1 
#define TEMP_TIMES 10000
#define TEMP_LOW_DISCHARGING -100
#define TEMP_HIGH_DISCHARGING 550
#define TEMP_LOW_RECHARGING -50
#define TEMP_HIGH_RECHARGING 500
#define TEMP_LOW_NO_BATT -300
#define BATT_VCELL_LOW_VOLT	3500 
#define BATT_SOC_LOW_TEMP_SET	0 
#define OTP_OPERATE_SWITCH 1
#define RECORDING_OPERATE_SWITCH 1
#include "../misc/muic/muic.h"
extern TYPE_CHARGING_MODE charging_mode;
extern bool tegra_camera_get_power_save_rec(void);

#define LGE_MAX8971_WORK_AROUND


#define UNLIMITED_TEMP_VAL	0xA4 //decimal 164
#define UNLIMITED_TEMP_HIGH	390
#define UNLIMITED_TEMP_LOW	-50


#define __DEBUG_TEMP

#ifdef __DEBUG_TEMP
#define DTEMP(fmt, args...) printk("[TEMP] " fmt "\n", ##args) 
#else
#define DTEMP(fmt, args...) 
#endif


static ssize_t lge_battery_show(struct device *dev,
			    struct device_attribute *attr, 
			    char *buf);

static ssize_t lge_battery_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, 
			     size_t count);


struct lge_battery_info {
	struct device 		*dev;

	char 			*gauge_name;
	char 			*charger_name;
	char 			*adc_name;

	struct power_supply	psy_bat;
	struct power_supply	psy_usb;
	struct power_supply 	psy_ac;
	struct power_supply 	psy_factory;

	/* battery */
	unsigned int 		bat_health;
	unsigned int 		bat_vcell;
	unsigned int 		bat_soc;
	int 			bat_temp;
	unsigned int		bat_id;
	unsigned int 		polling_interval;
	unsigned int		bat_temp_adc;
	unsigned int		temp_control; 

	/* charger */
	int			online;
	int			charging_status;

	/* work */
	struct workqueue_struct	*monitor_wqueue;
	struct work_struct	monitor_work;
	struct work_struct	cable_work;
	struct delayed_work	polling_work;

	struct workqueue_struct *battery_power_update_workqueue;

	int present;
	
	int camera_state;
	int pre_camera_state;
	int camera_charging_switch;
	
	int cable_work_state;
	int charging_state_temp;
};

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property lge_battery_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ADC,
	POWER_SUPPLY_PROP_TEMP_CONTROL,
	POWER_SUPPLY_PROP_VALID_BATT_ID,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

static enum power_supply_property lge_battery_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
//******************************************************************//
// Battery Temperature Calculation Algorithm                        //
// int reference_graph()                                            //
//******************************************************************//

battery_graph_prop battery_temp_graph[] =
{
	
	{1800, -50 * TEMP_TIMES},
	{1750, -40 * TEMP_TIMES},
	{1680, -30 * TEMP_TIMES},
	{1585, -20 * TEMP_TIMES},
	{1445, -10 * TEMP_TIMES},
	{1273, 0 * TEMP_TIMES},
	{1073, 10 * TEMP_TIMES},
	{855, 20 * TEMP_TIMES},
	{633, 30 * TEMP_TIMES},
	{498, 40 * TEMP_TIMES},
	{366, 50 * TEMP_TIMES},
	{290, 60 * TEMP_TIMES},
	{200, 70 * TEMP_TIMES},
	{150, 80 * TEMP_TIMES},
	{100, 90 * TEMP_TIMES},
	{80, 100 * TEMP_TIMES},
};

int lge_battery_average_temp(int temp)
{
#define MAX_ABNORMAL_COUNT 2
	static int abnormal_temp_count = 0;
	static int old_temp = 200;
	int av_temp;

	if(temp > 600)
	{
		if( abnormal_temp_count < MAX_ABNORMAL_COUNT )
		{
			abnormal_temp_count++;
			av_temp = old_temp;
		}
		else
		{
			av_temp = temp;
		}
	}
	else
	{
		av_temp = temp;
		old_temp = temp;
		abnormal_temp_count = 0;
	}

	

	return 	av_temp;
}

int lge_battery_reference_graph(int __x, battery_graph_prop* ref_battery_graph, int arraysize)
{
	int i = 1;
	int __y = 0;
	int slope, const_term;
	int delta_y, delta_x;

	

	while( __x < ref_battery_graph[i].x \
		&& i < (arraysize - 1) )
	{
		i++;
	}

	delta_x = ref_battery_graph[i-1].x - ref_battery_graph[i].x;
	delta_y = (ref_battery_graph[i-1].y - ref_battery_graph[i].y);

	slope = delta_y  / delta_x;

	const_term = (ref_battery_graph[i].y) - (ref_battery_graph[i].x * slope);

	__y = (__x* slope + const_term);

	

	if(ref_battery_graph[i-1].y > ref_battery_graph[i].y)
	{
		if(__y > ref_battery_graph[i-1].y)
		{
			__y = ref_battery_graph[i-1].y;
			
		}
		else if(__y < ref_battery_graph[i].y)
		{
			__y = ref_battery_graph[i].y;
			
		}
	}
	else
	{
		if(__y < ref_battery_graph[i-1].y)
		{
			__y = ref_battery_graph[i-1].y;
			
		}
		else if(__y > ref_battery_graph[i].y)
		{
			__y = ref_battery_graph[i].y;
			
		}
	}

	return __y;
}

static void lge_battery_get_camera_info(struct lge_battery_info *info, char *filename)
{
#if 1
	int camera_state;

	if(tegra_camera_get_power_save_rec())
	{
		camera_state = 49;
	}
	else
	{
		camera_state = 48;
	}

	dev_info(info->dev,"read recording data is [%d]\n",camera_state);

	if(camera_state != info->camera_state){
		info->camera_state = camera_state;
	}

#else

#endif
}



static recharging_state_t recharging_wait_temperature_state = DISCHARGING_OFF;

int lge_battery_is_recharging_temperature(struct lge_battery_info *info, int temp)
{
	// Check recharging flag
	if((temp >= TEMP_LOW_RECHARGING && temp <= TEMP_HIGH_RECHARGING) || info->temp_control == UNLIMITED_TEMP_VAL){
		recharging_wait_temperature_state = DISCHARGING_OFF; // clear flag
		
		DTEMP("Clear recharging wait temperature flag\n");
		return true;
	}
	else return false;
}	

int lge_battery_state_temperature(struct lge_battery_info *info, int temp)
{
	if( (temp > (TEMP_LOW_DISCHARGING) && temp < (TEMP_HIGH_DISCHARGING)) \
		|| ( temp <= (TEMP_LOW_NO_BATT))|| info->temp_control == UNLIMITED_TEMP_VAL  )
	{
		if(recharging_wait_temperature_state && !lge_battery_is_recharging_temperature(info, temp ) ) {
			DTEMP("Wait for appropriate recharging temperature\n");
			return false;
		}
		info->bat_health = POWER_SUPPLY_HEALTH_GOOD;
		DTEMP("battery state temperature is True(Can Charging)!\n");
		return true;
	}
	
	if( !( temp <= TEMP_LOW_NO_BATT) && !(info->temp_control == UNLIMITED_TEMP_VAL)) {
		recharging_wait_temperature_state = DISCHARGING_ON; // set flag
		if(temp < TEMP_LOW_DISCHARGING){
			info->bat_health = POWER_SUPPLY_HEALTH_COLD;
		}
		else if(temp > TEMP_HIGH_DISCHARGING){
			info->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
		}
		DTEMP("Set recharging wait temperature flag : temp_C[%d]\n", temp);
	}
	
	DTEMP("battery state temperature is False(Can not Charging) !!\n");
	return false;
}


static void lge_battery_get_adc_info(struct lge_battery_info *info)
{
	struct power_supply *psy = power_supply_get_by_name(info->adc_name);
	union power_supply_propval value;
	int batt_ADC_value = 0;
	int batt_Temp_C = 0;

	if (!psy) {
		dev_err(info->dev, "%s: can not get adc info\n", __func__);
		return;
	}
	if(info->online == POWER_SUPPLY_TYPE_FACTORY){
		info->bat_temp = 0;
	}		
	else{
		psy->get_property(psy, POWER_SUPPLY_PROP_TEMP_ADC, &value);
		batt_ADC_value = value.intval;
		info->bat_temp_adc = batt_ADC_value;
		DTEMP("batt_ADC_value is %d\n",batt_ADC_value);
		batt_Temp_C = lge_battery_average_temp(lge_battery_reference_graph((s64)batt_ADC_value, battery_temp_graph, ARRAY_SIZE(battery_temp_graph)) / (TEMP_TIMES / 10));
			
		info->bat_temp = batt_Temp_C;
		dev_dbg(info->dev, "info->bat_temp = %d\n", info->bat_temp);

		lge_battery_state_temperature(info, info->bat_temp);
	}

	return;
}
static void lge_battery_get_gauge_info(struct lge_battery_info *info)
{
	struct power_supply *psy = power_supply_get_by_name(info->gauge_name);
	union power_supply_propval value;
	
	static int old_soc = 100;
	static int new_soc = 100;

	if (!psy) {
		dev_err(info->dev, "%s: can not get gauge info\n", __func__);
		return;
	}

	psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &value);
	info->bat_vcell = value.intval;
	dev_dbg(info->dev, "info->bat_vcell = %d\n", info->bat_vcell);

	psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &value);
	
	if(info->charging_status == POWER_SUPPLY_STATUS_CHARGING){ 		// Charging	
		info->bat_soc = value.intval;
		new_soc = value.intval;
		old_soc = value.intval;
		
	}else{									// Discharging
		
		if((info->bat_temp > TEMP_LOW_NO_BATT && info->bat_temp < TEMP_LOW_DISCHARGING) && (info->charging_status != POWER_SUPPLY_STATUS_CHARGING) && (info->bat_vcell < BATT_VCELL_LOW_VOLT)&& info->bat_soc <= 4){
			info->bat_soc = 0;
			dev_dbg(info->dev, "Low Temp Low Vcell ! Power down !info->bat_soc = %d\n", info->bat_soc);
		}
		
		else{
			if(old_soc >= new_soc){
		
				old_soc = new_soc;
			}else{}

			new_soc = value.intval;

			if(old_soc < new_soc){
				info->bat_soc = old_soc;

			}else{
				info->bat_soc = new_soc;

			}
		}
		
	}
	dev_dbg(info->dev, "info->bat_soc = %d\n", info->bat_soc);

	return;
}

static int lge_battery_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct lge_battery_info *info = 
		container_of(psy, struct lge_battery_info, psy_bat);

	/* should be modified.. */
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			val->intval = info->charging_status;
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = info->bat_health;
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_TEMP:
			if(info->temp_control == UNLIMITED_TEMP_VAL)
			{
				if(info->bat_temp > UNLIMITED_TEMP_HIGH)
					val->intval = UNLIMITED_TEMP_HIGH;
				else if(info->bat_temp < UNLIMITED_TEMP_LOW)
					val->intval = UNLIMITED_TEMP_LOW;
				else
					val->intval = info->bat_temp;
			}
			else
			{
				val->intval = info->bat_temp;
			}
			break;
		case POWER_SUPPLY_PROP_TEMP_ADC:
			val->intval = info->bat_temp_adc;
			break;
		case POWER_SUPPLY_PROP_TEMP_CONTROL:
			val->intval = info->temp_control;
			break;
		case POWER_SUPPLY_PROP_VALID_BATT_ID:
			val->intval = info->bat_id;
			break;
		case POWER_SUPPLY_PROP_ONLINE:
			val->intval = info->online;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = info->bat_vcell;
			if (val->intval == -1)
				return -EINVAL;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = info->bat_soc;
			if (val->intval == -1)
				return -EINVAL;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

static int lge_battery_cable_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct lge_battery_info *info;
	
	if(val->intval == POWER_SUPPLY_TYPE_MAINS)				
		info = container_of(psy, struct lge_battery_info, psy_ac);
	else if(val->intval == POWER_SUPPLY_TYPE_USB)					
		info = container_of(psy, struct lge_battery_info, psy_usb);
	else if(val->intval == POWER_SUPPLY_TYPE_FACTORY)
		info = container_of(psy, struct lge_battery_info, psy_factory);

        info->charger_name = "charger";
        info->online = val->intval;
	dev_info(info->dev, "Called lge_battery_cable_set_property\n");
	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			dev_info(info->dev, "%s : Cable was changed : %d\n",__func__,
					val->intval);
			switch (val->intval) {
				case POWER_SUPPLY_TYPE_MAINS:	// TA
				case POWER_SUPPLY_TYPE_USB:	// USB
				case POWER_SUPPLY_TYPE_FACTORY:
					info->online = val->intval;                    
                                info->charging_status = POWER_SUPPLY_STATUS_CHARGING;                
					break;
				case POWER_SUPPLY_TYPE_UPS:
				default:
					return -EINVAL;
					break;
			}
			if(val->intval == POWER_SUPPLY_TYPE_MAINS)	
			{
				power_supply_changed(&info->psy_ac);
			}
			else if(val->intval == POWER_SUPPLY_TYPE_USB)				
			{
				power_supply_changed(&info->psy_usb);
			}
			else if(val->intval == POWER_SUPPLY_TYPE_FACTORY)
			{
				power_supply_changed(&info->psy_factory);
			}

			queue_work(info->battery_power_update_workqueue,&info->cable_work);
			
			break;
		default:
			return -EINVAL;
	}

	return 0;
}


static int lge_battery_battery_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct lge_battery_info *info = 
		container_of(psy, struct lge_battery_info, psy_bat);

	dev_info(info->dev, "%s : supply property : %d\n",__func__, psp);
	
	

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			info->charger_name = info->psy_bat.name;
			info->online = POWER_SUPPLY_TYPE_BATTERY;
			info->charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;				
			power_supply_changed(&info->psy_bat);
			schedule_work(&info->monitor_work);			
			break;
		case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
			info->charger_name = info->psy_bat.name;
			info->online = POWER_SUPPLY_TYPE_BATTERY;
			if (val->intval != POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL)
				return -EINVAL;			
			power_supply_changed(&info->psy_bat);
			schedule_work(&info->monitor_work);			
			break;
		case POWER_SUPPLY_PROP_TEMP:
			info->bat_temp = val->intval;
			break;
		case POWER_SUPPLY_PROP_TEMP_CONTROL:
			printk("temp_control is %d, %x\n", val->intval, val->intval);
			if(val->intval == UNLIMITED_TEMP_VAL)
			{
				info->temp_control = val->intval;
			}
			else
			{
				info->temp_control = 0;
			}
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static int lge_battery_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct lge_battery_info *info = container_of(psy, struct lge_battery_info,
						 psy_usb);
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	val->intval = (info->online == POWER_SUPPLY_TYPE_USB);

	return 0;
}

static int lge_battery_ac_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct lge_battery_info *info = container_of(psy, struct lge_battery_info,
						 psy_ac);
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	val->intval = (info->online == POWER_SUPPLY_TYPE_MAINS);

	return 0;
}

static int lge_battery_factory_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct lge_battery_info *info = container_of(psy, struct lge_battery_info,
						 psy_factory);
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	val->intval = (info->online == POWER_SUPPLY_TYPE_FACTORY);

	return 0;
}


static void lge_battery_update_info(struct lge_battery_info *info)
{
	lge_battery_get_gauge_info(info);
	lge_battery_get_adc_info(info);
	return;
}


static int lge_battery_enable_charger(struct lge_battery_info *info, bool enable)
{
	struct power_supply *psy;
	union power_supply_propval val_type;
	union power_supply_propval val_chg_current;
	int ret;

	if (enable && info->bat_health != POWER_SUPPLY_HEALTH_GOOD) {
		info->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
		dev_info(info->dev, "Battery Health is BAD\n");

	}

	printk("%s : name is %s, enable = (%d)\n", __func__, info->charger_name, enable);
	psy = power_supply_get_by_name(info->charger_name);
	if (!psy) {
		dev_err(info->dev, "Fail to get charger psy\n");
		return -ENODEV;
	}

	if (enable) {	/* Enable charging */

		switch (info->online) {
			case POWER_SUPPLY_TYPE_USB:
#if RECORDING_OPERATE_SWITCH	
				
				if((charging_mode != CHARGING_USB) && (charging_mode != CHARGING_FACTORY)){
					if(info->camera_state == 49){
						if(info->bat_soc <= 11){
                                                info->camera_charging_switch = 1;
                                                info->online = POWER_SUPPLY_TYPE_MAINS;
                                                dev_info(info->dev, "%s :We detected Camera Recording, Charging Current will be 900mA\n",__func__);
                                        	}
                                        else if((info->bat_soc > 11) && (info->bat_soc < 30) && info->camera_charging_switch == 1){
                                                info->online = POWER_SUPPLY_TYPE_MAINS;
                                                dev_info(info->dev, "%s :We detected Camera Recording, Charging Current will be 900mA\n",__func__);
                                        	}
					}
					else if(info->camera_state != info->pre_camera_state){
                                        	info->online = POWER_SUPPLY_TYPE_MAINS;
                                        	info->camera_charging_switch = 0;
                                        	dev_info(info->dev, "%s :Recording END, Charging Current will be 900mA\n",__func__);
                                	}
                                	info->pre_camera_state = info->camera_state;
				}
				
#endif
				val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
                                val_chg_current.intval = info->online;

				break;
			case POWER_SUPPLY_TYPE_MAINS:				
			case POWER_SUPPLY_TYPE_FACTORY:
#if RECORDING_OPERATE_SWITCH
				
				if((charging_mode != CHARGING_USB) && (charging_mode != CHARGING_FACTORY)){
					if(info->camera_state == 49){
						info->online = POWER_SUPPLY_TYPE_USB;
						info->camera_charging_switch = 0;
						dev_info(info->dev, "%s :We detected Camera Recording, Charging Current will be 500mA\n",__func__);
					}
					info->pre_camera_state = info->camera_state;
	                                dev_info(info->dev,"%s : camera_state is %d , pre_camera_state is %d\n", info->camera_state, info->pre_camera_state);
				}
				
#endif
				val_type.intval = POWER_SUPPLY_STATUS_CHARGING;
				val_chg_current.intval = info->online;

				break;
			case POWER_SUPPLY_TYPE_BATTERY:
			case POWER_SUPPLY_TYPE_UPS:
			default:
				dev_err(info->dev, "Will not charging : %d\n",
						info->online);
				return -EINVAL;
		}
		/* Set charging current */
		ret = psy->set_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW,
				&val_chg_current);
		if (ret) {
			dev_err(info->dev, "Fail to set charging cur : %d\n", 
					ret);
			return ret;
		}

	} 
	else { 	/* Disable charging */
		val_type.intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}		                

	ret = psy->set_property(psy, POWER_SUPPLY_PROP_STATUS, &val_type);

	if (ret) {
		dev_err(info->dev, "Fail to set charging status : %d\n", ret);
		return ret;
	}

	return 0;
}

static void lge_battery_cable_work(struct work_struct *work)
{
	struct lge_battery_info *info = container_of(work, struct lge_battery_info,
						 cable_work);

	printk("%s: online : (%d)\n", __func__, info->online);

	switch (info->online) {
		case POWER_SUPPLY_TYPE_MAINS:
		case POWER_SUPPLY_TYPE_USB:
#if OTP_OPERATE_SWITCH
			if((recharging_wait_temperature_state == DISCHARGING_ON) && (info->charging_status == POWER_SUPPLY_STATUS_CHARGING)){
                                        info->charging_state_temp = lge_battery_enable_charger(info, false);
                                        info->charging_status  = POWER_SUPPLY_STATUS_DISCHARGING;
                                        printk("%s: Discharging cause of temperature (%s)\n", __func__,info->bat_health);
                                printk("%s : CALLED in Temperature scean\n",__func__);
			        break;
			}
                        else if((recharging_wait_temperature_state == DISCHARGING_ON) && (info->charging_status != POWER_SUPPLY_STATUS_CHARGING)){
                                printk("%s : charging state = %d\n",__func__, info->charging_status);
                                break;
                        }
                        else if (!lge_battery_enable_charger(info, true)) {
                                info->charging_status =
                                        POWER_SUPPLY_STATUS_CHARGING;
                                printk("%s : CALLED in Normal Charger scean\n",__func__);
                        }	
			break;
#endif
		case POWER_SUPPLY_TYPE_FACTORY:
			if (!lge_battery_enable_charger(info, true)) {
				info->charging_status = 
					POWER_SUPPLY_STATUS_CHARGING;
			}
			break;
		default:
			dev_err(info->dev, "%s: Invalid cable type\n", __func__);
			break;;
	}
	power_supply_changed(&info->psy_bat);
	return;
}

#define FOR_MONITORING_TEMP_N_CURR 1 

#if FOR_MONITORING_TEMP_N_CURR
extern int get_temp_for_log(long *pTemp);
extern int get_current_for_log(int *pCurrent_mA);
#endif

extern unsigned char chg_flag_muic;
static void lge_battery_monitor_work(struct work_struct *work)
{
	struct lge_battery_info *info = container_of(work, struct lge_battery_info,
						 monitor_work);
	static unsigned int old_bat_soc = 0;
	static unsigned int chg_cnt_old= 0;
	static unsigned int chg_cnt_new= 0;
#if defined(LGE_MAX8971_WORK_AROUND)
	static unsigned int old_soc = 0;
#endif
#if FOR_MONITORING_TEMP_N_CURR
	int ret;
	long temp_now;
	int current_now;
#endif
	
	lge_battery_update_info(info);

#if FOR_MONITORING_TEMP_N_CURR
	ret = get_temp_for_log(&temp_now);
	if (ret < 0) dev_err(info->dev, "get_temp_for_log fail[%d]\n", ret);
	ret = get_current_for_log(&current_now);
	if ((ret < 0) && (current_now != -9998)) dev_err(info->dev, "get_current_for_log fail[%d]\n", ret);
	if (current_now == -9998)
		dev_info(info->dev, "temp_now[%dmC], current_sensor_off[single_core]\n", temp_now);
	else
		dev_info(info->dev, "temp_now[%dmC], current_now[%dmA]\n", temp_now, current_now);
#endif

	chg_cnt_old = chg_cnt_new;
	chg_cnt_new = chg_flag_muic;
	if(chg_cnt_old == chg_cnt_new){
		dev_info(info->dev, "chg_flag_muic is initialized\n");
		chg_flag_muic = 0;
	}
#if defined(LGE_MAX8971_WORK_AROUND)
#define MAX_CHARGER_INT_COUNT 10 
	else if((info->online == POWER_SUPPLY_TYPE_MAINS) && (chg_flag_muic > MAX_CHARGER_INT_COUNT)){
		dev_info(info->dev, "chg_flag_muic is overcounted %d, Reset %s\n", chg_flag_muic , info->charger_name);
		lge_battery_enable_charger(info, false);
		dev_info(info->dev, "%s off",info->charger_name);
		mdelay(1);
		lge_battery_enable_charger(info, true);
		dev_info(info->dev, "%s on",info->charger_name);
 	
	}
	if((info->charging_status == POWER_SUPPLY_STATUS_CHARGING) && (info->online != POWER_SUPPLY_TYPE_FACTORY)){
		if(info->bat_soc > old_soc)
			old_soc = info->bat_soc;
		if(old_soc > (info->bat_soc + 5)){
			old_soc = info->bat_soc;

			dev_info(info->dev, "charger do not work correctly : soc %d, Reset %s\n", info->bat_soc , info->charger_name);
	                lge_battery_enable_charger(info, false);
        	        dev_info(info->dev, "%s off",info->charger_name);
			mdelay(1);
			lge_battery_enable_charger(info, true);
			dev_info(info->dev, "%s on",info->charger_name);
		}
	}
#endif
	if(info->bat_soc == 0){
		power_supply_changed(&info->psy_bat);
		dev_info(info->dev,
		"soc(%d),  vcell(%d), temp(%d), charging(%d), health(%d)\n",
		info->bat_soc, 
		info->bat_vcell / 1000, info->bat_temp ,
		info->charging_status, info->bat_health);
	}else{
		if(old_bat_soc  == info->bat_soc){

		}else{
		power_supply_changed(&info->psy_bat);
		old_bat_soc = info->bat_soc;
		}
		dev_info(info->dev,
		"soc(%d),  vcell(%d), temp(%d), charging(%d), health(%d), id(%d), online(%d), system_rev(%d)\n",
		info->bat_soc, 
		info->bat_vcell / 1000, info->bat_temp ,
		info->charging_status, info->bat_health, info->bat_id, info->online, system_rev);
	}
	lge_battery_get_camera_info(info,CAMERA_STATE_ROOT);
	return;
}

static void lge_battery_polling_work(struct work_struct *work)
{
	struct lge_battery_info *info;
	int forced_cable_work_needed = 0;
	info = container_of(work, struct lge_battery_info, polling_work.work);
	
	
	schedule_work(&info->monitor_work);
#if OTP_OPERATE_SWITCH	
	if(info->online != POWER_SUPPLY_TYPE_BATTERY && info->online != POWER_SUPPLY_TYPE_FACTORY ){
		if(recharging_wait_temperature_state == DISCHARGING_ON && info->charging_status == POWER_SUPPLY_STATUS_CHARGING){
		info->cable_work_state = 1;
		forced_cable_work_needed = 1;
	}
	else if(info->cable_work_state == 1 && recharging_wait_temperature_state == DISCHARGING_OFF && info->charging_status != POWER_SUPPLY_STATUS_CHARGING ){
		info->cable_work_state = 0;
		forced_cable_work_needed = 1;
		}
	}
#endif

#if RECORDING_OPERATE_SWITCH
	// It Start Recording Start && Status charging
	if((charging_mode != CHARGING_USB) && (charging_mode != CHARGING_FACTORY)){
		if(info->camera_state == 49){
			if((info->charging_status == POWER_SUPPLY_STATUS_CHARGING) && info->camera_state != info->pre_camera_state )
 			{
				forced_cable_work_needed = 1;
 			}
		}
	/* After Stop Recording. Switch to Normal Charge mode.
	   also SOC check and Change Charging Current*/
		if((info->camera_state == 49 && info->charging_status == POWER_SUPPLY_STATUS_CHARGING && ((info->bat_soc < 11 && info->camera_charging_switch == 0) || (info->bat_soc > 30 &&info->camera_charging_switch == 1))))
 		{
			forced_cable_work_needed = 1;
	 	}
		if((info->camera_state != info->pre_camera_state) && (info->camera_state == 48) && (info->charging_status == POWER_SUPPLY_STATUS_CHARGING)){
			forced_cable_work_needed = 1;
		}
	}

#endif

	if(forced_cable_work_needed == 1){
		queue_delayed_work(info->battery_power_update_workqueue, &info->cable_work, 0);
	}

	if(info->charging_status == POWER_SUPPLY_STATUS_NOT_CHARGING){ 
		queue_delayed_work(info->battery_power_update_workqueue, &info->polling_work, POLLING_INTERVAL_DISCHG);
	}else{							
		queue_delayed_work(info->battery_power_update_workqueue, &info->polling_work, info->polling_interval);
	}
	return;
}

#define LGE_BCI_ATTR(_name)			\
{						\
	.attr	= {				\
		.name = #_name,			\
		.mode = 0664,			\
	},					\
	.show	= lge_battery_show,		\
	.store	= lge_battery_store,		\
}

static struct device_attribute lge_battery_attrs[] = {
	LGE_BCI_ATTR(bat_soc),
	LGE_BCI_ATTR(bat_vcell),
	LGE_BCI_ATTR(bat_temp),
};

static ssize_t lge_battery_show(struct device *dev,
			    struct device_attribute *attr, 
			    char *buf)
{

	int i = 0; //, val;
	const ptrdiff_t off = attr - lge_battery_attrs;

	switch (off) {
		default:
			i = -EINVAL;
	}

	return i;
}

static ssize_t lge_battery_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int ret = 0;
	const ptrdiff_t off = attr - lge_battery_attrs;
	

	switch (off) {
		default:
			ret = -EINVAL;
	}

	return ret;
}

static int lge_battery_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(lge_battery_attrs); i++) {
		rc = device_create_file(dev, &lge_battery_attrs[i]);
		if (rc)
			goto failed;
	}
	goto succeed;

failed:
	while (i--)
		device_remove_file(dev, &lge_battery_attrs[i]);
succeed:
	return rc;
}

static int lge_battery_is_charging(struct lge_battery_info *info)
{
	struct power_supply *psy = power_supply_get_by_name(info->charger_name);
	union power_supply_propval value;
	int ret;

	if (!psy) {
		dev_err(info->dev, "%s: can not get charger psy\n", __func__);
		return -ENODEV;
	}

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &value);
	if (ret < 0) {
		dev_err(info->dev, "%s: can not get status: %d\n", __func__,
			ret);
		return ret;
	}

	return value.intval;
}

static __devinit int lge_battery_probe(struct platform_device *pdev)
{
	struct lge_battery_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct lge_battery_info *info;
	struct power_supply *psy;
	int ret = 0;

	dev_info(&pdev->dev, "%s: LGE Battery Charger Interface "
				"Driver Loading\n", __func__);

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "memory error\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, info);

	info->dev = &pdev->dev;

	if (!pdata->gauge_name || !pdata->charger_name) {
		dev_err(info->dev, "%s: can not get name of gauge or charger\n",
			__func__);
		goto err_kfree;
	}

	info->gauge_name		= pdata->gauge_name;
	info->charger_name		= pdata->charger_name;

	
	info->adc_name			= pdata->adc_name;
	info->psy_bat.name		= "battery";
	info->psy_bat.type		= POWER_SUPPLY_TYPE_BATTERY;
	info->psy_bat.properties	= lge_battery_battery_props;
	info->psy_bat.num_properties	= ARRAY_SIZE(lge_battery_battery_props);
	info->psy_bat.get_property	= lge_battery_battery_get_property;
	info->psy_bat.set_property	= lge_battery_battery_set_property;
	info->online			= POWER_SUPPLY_TYPE_BATTERY;

	info->psy_usb.name		= "usb";
	info->psy_usb.type		= POWER_SUPPLY_TYPE_USB;
	info->psy_usb.supplied_to	= supply_list;
	info->psy_usb.num_supplicants	= ARRAY_SIZE(supply_list);
	info->psy_usb.properties	= lge_battery_power_props;
	info->psy_usb.num_properties	= ARRAY_SIZE(lge_battery_power_props);
	info->psy_usb.get_property	= lge_battery_usb_get_property;
	info->psy_usb.set_property	= lge_battery_cable_set_property;	
	info->online			= POWER_SUPPLY_TYPE_USB;

	info->psy_ac.name		= "ac";
	info->psy_ac.type		= POWER_SUPPLY_TYPE_MAINS;
	info->psy_ac.supplied_to	= supply_list;
	info->psy_ac.num_supplicants	= ARRAY_SIZE(supply_list);
	info->psy_ac.properties		= lge_battery_power_props;
	info->psy_ac.num_properties	= ARRAY_SIZE(lge_battery_power_props);
	info->psy_ac.get_property	= lge_battery_ac_get_property;	
	info->psy_ac.set_property	= lge_battery_cable_set_property;	
	info->online			= POWER_SUPPLY_TYPE_MAINS;

	
	info->psy_factory.name		= "factory";
	info->psy_factory.type		= POWER_SUPPLY_TYPE_FACTORY;
	info->psy_factory.supplied_to	= supply_list;
	info->psy_factory.num_supplicants	= ARRAY_SIZE(supply_list);
	info->psy_factory.properties		= lge_battery_power_props;
	info->psy_factory.num_properties	= ARRAY_SIZE(lge_battery_power_props);
	info->psy_factory.get_property	= lge_battery_factory_get_property;
	info->psy_factory.set_property	= lge_battery_cable_set_property;	
	info->online			= POWER_SUPPLY_TYPE_FACTORY;

	psy = power_supply_get_by_name(info->charger_name);
	if (!psy) {
		dev_err(info->dev, "%s: fail to get charger\n", __func__);
		return -ENODEV;
	}


	info->bat_health = POWER_SUPPLY_HEALTH_GOOD;

	info->charging_status = lge_battery_is_charging(info);
	if (info->charging_status < 0) {
		info->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	info->polling_interval	= POLLING_INTERVAL;
	info->bat_temp_adc = 0;
	info->bat_temp = 0;
	info->camera_charging_switch = 0;
	info->temp_control = 0;
	info->cable_work_state = 0;
	info->charging_state_temp = 1;
	info->camera_state = 48;
	info->pre_camera_state = 48;
	info->bat_id = is_tegra_batteryVerified();
	info->temp_control = 0xFF;

	/* init power supplier framework */
	ret = power_supply_register(&pdev->dev, &info->psy_bat);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_bat\n",
			__func__);
		goto err_wake_lock;
	}

	ret = power_supply_register(&pdev->dev, &info->psy_usb);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_usb\n",
			__func__);
		goto err_supply_unreg_bat;
	}

	ret = power_supply_register(&pdev->dev, &info->psy_ac);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_ac\n", __func__);
		goto err_supply_unreg_usb;
	}

	ret = power_supply_register(&pdev->dev, &info->psy_factory);
	if (ret) {
		dev_err(info->dev, "%s: failed to register psy_factory\n", __func__);
		goto err_supply_unreg_factory;
	}

	info->monitor_wqueue = 
		create_freezable_workqueue(dev_name(&pdev->dev));
	if (!info->monitor_wqueue) {
		dev_err(info->dev, "%s: fail to create workqueue\n", __func__);
		goto err_supply_unreg_ac;
	}

	INIT_WORK(&info->monitor_work, lge_battery_monitor_work);
	INIT_WORK(&info->cable_work, lge_battery_cable_work);

	info->battery_power_update_workqueue = create_workqueue("x3_battery_workqueue");

	INIT_DELAYED_WORK_DEFERRABLE(&info->polling_work, lge_battery_polling_work);

	queue_delayed_work(info->battery_power_update_workqueue, &info->polling_work, 10*HZ);
	

	return 0;

err_supply_unreg_ac:
	power_supply_unregister(&info->psy_ac);
err_supply_unreg_usb:
	power_supply_unregister(&info->psy_usb);
err_supply_unreg_factory:
	power_supply_unregister(&info->psy_factory);	
err_supply_unreg_bat:
	power_supply_unregister(&info->psy_bat);
err_wake_lock:
err_kfree:
	kfree(info);

	return ret;
}

static int __devexit lge_battery_remove(struct platform_device *pdev)
{
	struct lge_battery_info *info = platform_get_drvdata(pdev);

	flush_workqueue(info->monitor_wqueue);
	destroy_workqueue(info->monitor_wqueue);

	cancel_delayed_work(&info->polling_work);

	power_supply_unregister(&info->psy_bat);
	power_supply_unregister(&info->psy_usb);
	power_supply_unregister(&info->psy_ac);
	power_supply_unregister(&info->psy_factory);

	kfree(info);

	return 0;
}

#if defined(CONFIG_PM)
static int lge_battery_suspend(struct device *dev)
{
	struct lge_battery_info *info = dev_get_drvdata(dev);

	cancel_work_sync(&info->monitor_work);
	cancel_delayed_work(&info->polling_work);

	return 0;
}

static int lge_battery_resume(struct device *dev)
{
	struct lge_battery_info *info = dev_get_drvdata(dev);

	

	queue_delayed_work(info->battery_power_update_workqueue, &info->polling_work, HZ);
	

	return 0;
}

static const struct dev_pm_ops lge_battery_pm_ops = {
	.suspend	= lge_battery_suspend,
	.resume		= lge_battery_resume,
};
#endif

static struct platform_driver lge_battery_driver = {
	.driver	= {
		.name	= "lge-battery",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm	= &lge_battery_pm_ops,
#endif
	},
	.probe	= lge_battery_probe,
	.remove	= __devexit_p(lge_battery_remove),
};

static int __init lge_battery_init(void)
{
	return platform_driver_register(&lge_battery_driver);
}

static void __exit lge_battery_exit(void)
{
	platform_driver_unregister(&lge_battery_driver);
}

fs_initcall(lge_battery_init);
module_exit(lge_battery_exit);

MODULE_DESCRIPTION("LGE Battery Driver");
MODULE_AUTHOR("Yool-Je Cho <yoolje.cho@lge.com>");
MODULE_LICENSE("GPL");

