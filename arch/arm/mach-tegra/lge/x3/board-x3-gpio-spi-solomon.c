#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <mach-tegra/gpio-names.h>
#include <lge/board-x3-gpio-spi-solomon.h>


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
	
	ret = gpio_request(lcd_reset_n, "pw0");
	if (ret < 0){
		gpio_free(lcd_reset_n);
		return ret;
	}
	ret=gpio_direction_output(lcd_reset_n, 1);
	if (ret < 0){
		gpio_free(lcd_reset_n);
		return ret;
	}
	else
		tegra_gpio_enable(lcd_reset_n);

	ret = gpio_request(dsi_bridge_en, "pv6");
	if (ret < 0){
		gpio_free(dsi_bridge_en);
		return ret;
	}
	ret=gpio_direction_output(dsi_bridge_en, 1);
	if (ret < 0){
		gpio_free(dsi_bridge_en);
		return ret;
	}
	else
		tegra_gpio_enable(dsi_bridge_en);


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
	ret=gpio_direction_output(spi_mosi, 1);
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
	ret=gpio_direction_input(spi_miso);
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

    gpio_set_value(lcd_reset_n,1);
    mdelay(20);
	gpio_set_value(lcd_reset_n,0);
    mdelay(20);
    gpio_set_value(lcd_reset_n,1);
    mdelay(20);

}

#define T_cycle_value_us		90

static void spi_write_byte(u16 val)
{
	int i;


	for (i = 0; i < 9; i++) {

		
		if (val & shift_bit[i])
			gpio_set_value(spi_mosi, 1);
		else
			gpio_set_value(spi_mosi, 0);

		udelay(T_cycle_value_us*4);
		gpio_set_value(spi_sclk, 1);

		udelay(T_cycle_value_us*4);
		gpio_set_value(spi_sclk, 0);
	}

}
static int send_gpio_to_spi_emulation(u16 reg)
{
	spi_write_byte(reg);

	return 0;

}

static int set_gpio_to_spi_emulation(u16 data)
{

	spi_write_byte(0x100+data);

	return 0;

}

static void gpio_emulation_spi(struct spi_cmd_data *cmds, int cnt)
{
	struct spi_cmd_data *cm;
	int i,j;

	cm = cmds;


	if ( !spi_cs )
		printk( " >>> pin error - spi_cs \n" );
	if ( !spi_sclk )
		printk( " >>> pin error - spi_sclk \n" );
	if ( !spi_mosi )
		printk( " >>> pin error - spi_mosi \n" );
	if ( !spi_miso )
		printk( " >>> pin error - spi_miso \n" );

	printk(" >>> array cnt : %d \n", cnt);
	
	for (i = 0; i < cnt; i++) {

		if (cm->payload || cm->dtype) 
		{
			/* Enable the Chip Select - low */
			gpio_set_value(spi_cs, 0);
			udelay(33);

			send_gpio_to_spi_emulation(cm->dtype);
			
			for (j = 0; j < cm->dlen; j++)
			{
				
				set_gpio_to_spi_emulation(cm->payload[j]);
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
	gpio_emulation_spi(lgit_power_off_set, ARRAY_SIZE(lgit_power_off_set));
}



void lgit_disp_on(void)
{
	lgit_lcd_reset();
	gpio_spi_init();
	
	gpio_emulation_spi(init_ssd2825_sequence, ARRAY_SIZE(init_ssd2825_sequence));
	
#if defined(CONFIG_MACH_X3_HD_HITACHI)
	gpio_emulation_spi(hitachi_power_on_set, ARRAY_SIZE(hitachi_power_on_set));
#elif defined(CONFIG_MACH_X3_HD_LGD)
	gpio_emulation_spi(lgit_power_on_set, ARRAY_SIZE(lgit_power_on_set));
#endif

	gpio_emulation_spi(LGD_leave_sleep_sequence, ARRAY_SIZE(LGD_leave_sleep_sequence));

	gpio_emulation_spi(ssd2825_video_sequence, ARRAY_SIZE(ssd2825_video_sequence));
}

