/*
 * arch/arm/mach-tegra/lge/board-x3-gpio-spi-toshiba.c
 *
 * Copyright (C) 2011 LG Electronics, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <mach-tegra/gpio-names.h>
#include <lge/board-x3-gpio-spi-toshiba.h>



static int spi_sclk		= TEGRA_GPIO_PZ4;
static int spi_cs		= TEGRA_GPIO_PN4;
static int lcd_reset_n		= TEGRA_GPIO_PW0;
static int spi_mosi		= TEGRA_GPIO_PZ2;
static int spi_miso		= TEGRA_GPIO_PN5;
static int dsi_bridge_en	= TEGRA_GPIO_PV6;

void gpio_init_set(void)
{
	int ret;

	printk("gpio_init_set start!!\n");
	
	

	ret = gpio_request(dsi_bridge_en, "pv6");
	if (ret < 0){
		gpio_free(dsi_bridge_en);
		return ret;
	}
	ret=gpio_direction_output(dsi_bridge_en, 0);
	if (ret < 0){
		gpio_free(dsi_bridge_en);
		return ret;
	}
	else
		tegra_gpio_enable(dsi_bridge_en);

	ret = gpio_request(lcd_reset_n, "pw0");
	if (ret < 0){
		gpio_free(lcd_reset_n);
		return ret;
	}
	ret=gpio_direction_output(lcd_reset_n, 0);
	if (ret < 0){
		gpio_free(lcd_reset_n);
		return ret;
	}
	else
		tegra_gpio_enable(lcd_reset_n);

	ret = gpio_request(spi_cs, "pn4");
	if (ret < 0){
		gpio_free(spi_cs);
		return ret;
	}
	ret=gpio_direction_output(spi_cs, 1);
	if (ret < 0){
		gpio_free(spi_cs);
		return ret;
	}
	else
		tegra_gpio_enable(spi_cs);

	ret = gpio_request(spi_sclk, "pz4");
	if (ret < 0){
		gpio_free(spi_sclk);
		return ret;
	}
	ret=gpio_direction_output(spi_sclk, 1);
	if (ret < 0){
		gpio_free(spi_sclk);
		return ret;
	}
	else
		tegra_gpio_enable(spi_sclk);

	ret = gpio_request(spi_mosi, "pz2");
	if (ret < 0){
		gpio_free(spi_mosi);
		return ret;
	}
#if defined(GPIO_SETTING_CHANGE)
	ret=gpio_direction_input(spi_mosi);
#else
	ret=gpio_direction_output(spi_mosi, 1);
#endif
	if (ret < 0){
		gpio_free(spi_mosi);
		return ret;
	}
	else
		tegra_gpio_enable(spi_mosi);

	ret = gpio_request(spi_miso, "pn5");
	if (ret < 0){
		gpio_free(spi_miso);
		return ret;
	}
#if defined(GPIO_SETTING_CHANGE)
	ret=gpio_direction_output(spi_miso, 1);
#else
	ret=gpio_direction_input(spi_miso);
#endif

	if (ret < 0){
		gpio_free(spi_miso);
		return ret;
	}
	else
		tegra_gpio_enable(spi_miso);


	printk("gpio_init_set complete!!\n");
	
}


static void lgit_lcd_reset(void)
{
	mdelay(15);

    gpio_set_value(lcd_reset_n,1);
    mdelay(5);
	gpio_set_value(lcd_reset_n,0);
    mdelay(5);
    gpio_set_value(lcd_reset_n,1);
    mdelay(15);

}

#define T_cycle_value_us		90 

static void spi_write_byte(unsigned int val)
{
	int i;
	unsigned int shift = 1;

	for (i = 0; i < 32; i++) {


#if defined(GPIO_SETTING_CHANGE)

		if (val & shift_bit32[i])
			gpio_set_value(spi_miso, 1);
		else
			gpio_set_value(spi_miso, 0);
#else
		if (val & shift_bit32[i])
			gpio_set_value(spi_mosi, 1);
		else
			gpio_set_value(spi_mosi, 0);
#endif	
		gpio_set_value(spi_sclk, 0);
		udelay(T_cycle_value_us*4);
		
		gpio_set_value(spi_sclk, 1);
		udelay(T_cycle_value_us*4);
	}

}



static int send_gpio_to_spi_emulation(unsigned int reg)
{
	spi_write_byte(reg);

	return 0;

}

static int set_gpio_to_spi_emulation(unsigned int data)
{
	unsigned int cmd_read = 0x00010000;
	spi_write_byte(data);

	return 0;

}

static void gpio_emulation_spi(struct spi_cmd_data *cmds, int cnt)
{
	struct spi_cmd_data *cm;
	int i,j;
	unsigned int cmd, data, sending_packit, loop;

	cm = cmds;


	if ( !spi_cs )
		printk( " >>> pin error - spi_cs \n" );
	if ( !spi_sclk )
		printk( " >>> pin error - spi_sclk \n" );
	if ( !spi_mosi )
		printk( " >>> pin error - spi_mosi \n" );
	if ( !spi_miso )
		printk( " >>> pin error - spi_miso \n" );

	for (i = 0; i < cnt; i++) {

		if (cm->payload || cm->dtype) 
		{
			/* Enable the Chip Select - low */
			cmd = 0; data=0; sending_packit=0;
		
			cmd = cm->dtype;
			data = cm->payload[0];
			sending_packit = cmd | data;	
			loop = (int)cm->dlen;

			
			gpio_set_value(spi_cs, 0);
			udelay(33);
			
			for( j = 0 ; j < loop ; j+=2 ){

				
				if( j == 0 )
					set_gpio_to_spi_emulation(sending_packit);	
				else{
					sending_packit = 0; cmd=0; data=0;
					cmd = cm->payload[j-1];
					data = cm->payload[j];
					sending_packit = cmd | data;
					
					send_gpio_to_spi_emulation(sending_packit);
				}
			}

			udelay(33);
			gpio_set_value(spi_cs, 1);	
			udelay(T_cycle_value_us*8);
		}
		
		if (cm->wait)
			msleep(cm->wait);
		cm++;
	}
	

}

static void gpio_spi_init(void)
{

	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_mosi, 0);
	gpio_set_value(spi_miso, 0);
	gpio_set_value(spi_cs, 1);
}



void lgit_disp_off(void)
{
}



void lgit_disp_on(void)
{

	mdelay(100);
	

	gpio_spi_init();

	gpio_set_value(dsi_bridge_en, 1);

	 mdelay(5);
	gpio_set_value(lcd_reset_n, 1);
	 mdelay(15);

	 mdelay(100);
	 
	gpio_emulation_spi(init_toshiba_bridge_sequence, ARRAY_SIZE(init_toshiba_bridge_sequence));
	
#if defined(CONFIG_MACH_X3_HD_HITACHI)
	gpio_emulation_spi(toshiba_hitachi_power_on_set, ARRAY_SIZE(toshiba_hitachi_power_on_set));

#elif defined(CONFIG_MACH_X3_HD_LGD)
	gpio_emulation_spi(toshiba_lgd_power_on_set, ARRAY_SIZE(toshiba_lgd_power_on_set));
	gpio_emulation_spi(toshiba_lgd_power_supply_set, ARRAY_SIZE(toshiba_lgd_power_supply_set));
#endif

	gpio_emulation_spi(toshiba_lgd_sleep_out_set, ARRAY_SIZE(toshiba_lgd_sleep_out_set));

	gpio_emulation_spi(toshiba_end_of_init_sequence, ARRAY_SIZE(toshiba_end_of_init_sequence));

}

