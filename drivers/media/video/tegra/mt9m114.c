/*
 * mt9m114.c - mt9m114 sensor driver
 *
 * Copyright (c) 2011, NVIDIA, All Rights Reserved.
 *
 * Contributors:
 *      hyeongjin.kim<hyeongjin.kim@lge.com>
 *
 * Leverage mt9m114.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

/**
 * SetMode Sequence for 640x480. Phase 0. Sensor Dependent.
 * This sequence should put sensor in streaming mode for 640x480
 * This is usually given by the FAE or the sensor vendor.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/mt9m114.h>
#include <linux/gpio.h>
#if 1
#include <linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_output_enable(void);
#endif
struct mt9m114_reg {
	u16 addr;
	u16 val;
};

struct mt9m114_info {
	int mode;
	struct i2c_client *i2c_client;
	struct mt9m114_platform_data *pdata;
};

#ifdef MT9M114_DEBUG
	#define mt_info pr_info
#else
	#define mt_info(arg...) do {} while (0)
#endif

#define  mt_err pr_err

#define MT9M114_TABLE_WAIT_MS 0
#define MT9M114_TABLE_END 1
#define MT9M114_MAX_RETRIES 3

static struct mt9m114_reg mode_640x480[] = {
{MT9M114_TABLE_WAIT_MS, 100},
{0x301A, 0x0230},
{MT9M114_TABLE_WAIT_MS, 100},
{0x301A, 0x0230},
{0x098E, 0x1000},
{0x098E, 0xC97E},
{0x0990, 0x0100},
{0xC980, 0x0660},
{0xC982, 0x0700},
{0xC984, 0x8041},
{0xC988, 0x0F00},
{0xC98A, 0x0B07},
{0xC98C, 0x0D01},
{0xC98E, 0x071D},
{0xC990, 0x0006},
{0xC992, 0x0A0C},
{0xC800, 0x0004},
{0xC802, 0x0004},
{0xC804, 0x03CB},
{0xC806, 0x050B},
{0x098E, 0x4808},
{0x0990, 0x02DC},
{0x0992, 0x6C00},
{0xC80C, 0x0001},
{0xC80E, 0x00DB},
{0xC810, 0x05B3},
{0xC812, 0x03EE},
{0xC814, 0x0636},
{0xC816, 0x0060},
{0xC818, 0x03C3},
{0xC826, 0x0020},
{0xC834, 0x0000},
{0xC854, 0x0000},
{0xC856, 0x0000},
{0xC858, 0x0500},
{0xC85A, 0x03C0},
{0x098E, 0xC85C},
{0x0990, 0x0300},
{0xC868, 0x0500},
{0xC86A, 0x03C0},
{0x098E, 0xC878},
{0x0990, 0x0000},
{0xC88C, 0x1E02},
{0xC88E, 0x0A00},
{0xC914, 0x0000},
{0xC916, 0x0000},
{0xC918, 0x04FF},
{0xC91A, 0x03BF},
{0xC91C, 0x0000},
{0xC91E, 0x0000},
{0xC920, 0x00FF},
{0xC922, 0x00BF},
{0x098E, 0xE801},
{0x0990, 0x0000},
{0x098E, 0xCC03},
{0x0990, 0x0200},
{0x098E, 0xC88B},
{0x0990, 0x3400},
{0x316A, 0x8270},
{0x316C, 0x8270},
{0x3ED0, 0x2305},
{0x3ED2, 0x77CF},
{0x316E, 0x8202},
{0x3180, 0x87FF},
{0x30D4, 0x6080},
{0xA802, 0x0008},
{0x3E14, 0xFF39},	
{0x301A, 0x0234},

{0x098E, 0xDC00},
{0x0990, 0x2800},
{0x0080, 0x8002},

{MT9M114_TABLE_WAIT_MS, 100},

 {MT9M114_TABLE_END, 0x0000}
};

enum {
	MT9M114_MODE_680x480,
};

static struct mt9m114_reg *mode_table[] = {
	[MT9M114_MODE_680x480] = mode_640x480,
};

static int mt9m114_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	*val = data[2] << 8 | data[3];

	return 0;
}

static int mt9m114_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val >> 8);
	data[3] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 4;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		mt_info("mt9m114: i2c transfer under transferring %x %x\n", addr, val);
		if (err == 1)
			return 0;
		retry++;
		mt_err("mt9m114: i2c transfer failed, retrying %x %x\n",
		       addr, val);
		msleep(3);
	} while (retry <= MT9M114_MAX_RETRIES);

	return err;
}

static int mt9m114_write_table(struct i2c_client *client,
			      const struct mt9m114_reg table[],
			      const struct mt9m114_reg override_list[],
			      int num_override_regs)
{
	int err;
	const struct mt9m114_reg *next;
	int i;
	u16 val;

	mt_info("mt9m114: mt9m114_write_table entered");
	for (next = table; next->addr != MT9M114_TABLE_END; next++) {
		mt_info("mt9m114: mt9m114_write_table 1");
		if (next->addr == MT9M114_TABLE_WAIT_MS) {
			mt_info("mt9m114: mt9m114_write_table : MT9M114_TABLE_WAIT_MS ");
			msleep(next->val);
			continue;
		}

		val = next->val;

		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		mt_info("mt9m114: mt9m114_write_table 2");
		err = mt9m114_write_reg(client, next->addr, val);
		if (err) {
			mt_err("mt9m114: mt9m114_write_table : err");
			return err;
		}
	}
	return 0;
}

static int mt9m114_set_mode(struct mt9m114_info *info, struct mt9m114_mode *mode)
{
	int sensor_mode;
	int err;

	mt_info("%s: xres %u yres %u\n", __func__, mode->xres, mode->yres);
	if (mode->xres == 640 && mode->yres == 480)
		sensor_mode = MT9M114_MODE_680x480;
	else if (mode->xres == 1280 && mode->yres == 960)
		sensor_mode = MT9M114_MODE_680x480;
	else {
		mt_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	mt_info("mt9m114: mt9m114_set_mode before write table");
	err = mt9m114_write_table(info->i2c_client, mode_table[sensor_mode],
		NULL, 0);
	if (err)
		return err;

	info->mode = sensor_mode;
	return 0;
}

static int mt9m114_get_status(struct mt9m114_info *info,
		struct mt9m114_status *dev_status)
{
	int err;

	err = mt9m114_write_reg(info->i2c_client, 0x98C, dev_status->data);
	if (err)
		return err;

	err = mt9m114_read_reg(info->i2c_client, 0x0990,
		(u16 *) &dev_status->status);
	if (err)
		return err;

	return err;
}

static long mt9m114_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	int err;
	struct mt9m114_info *info = file->private_data;
	
	mt_info("mt9m114: mt9m114_ioctl ");

	switch (cmd) {
	case MT9M114_IOCTL_SET_MODE:
	{
		struct mt9m114_mode mode;
		if (copy_from_user(&mode,
				   (const void __user *)arg,
				   sizeof(struct mt9m114_mode))) {
			return -EFAULT;
		}

		mt_info("mt9m114: mt9m114_ioctl : MT9M114_IOCTL_SET_MODE");
		return mt9m114_set_mode(info, &mode);
	}

	default:
		mt_info("mt9m114: mt9m114_ioctl : default");
		return -EINVAL;
	}
	return 0;
}

static struct mt9m114_info *info;

#if 1
#define SUB_CAM_RESET_N     221
#define VT_1V8V_EN          194
#endif

static int mt9m114_power_on(void)
{
#if 1
	subpm_set_output(LDO1,1);
	subpm_output_enable();
	subpm_set_output(LDO2,1);
	subpm_output_enable();
	udelay(100);
#endif

#if 1
	gpio_direction_output(VT_1V8V_EN, 1);
	gpio_set_value(VT_1V8V_EN, 1);
	udelay(100);
	gpio_direction_output(SUB_CAM_RESET_N, 1);
	gpio_set_value(SUB_CAM_RESET_N, 1);
	udelay(100);
#endif

	return 0;
}

static int mt9m114_power_off(void)
{
#if 1
        gpio_direction_output(VT_1V8V_EN, 0);
        gpio_set_value(VT_1V8V_EN, 0);
        udelay(100);
#endif

#if 1
	subpm_set_output(LDO1,0);
	subpm_output_enable();
	subpm_set_output(LDO2,0);
	subpm_output_enable();
	udelay(100);
#endif
	return 0;
}

static int mt9m114_open(struct inode *inode, struct file *file)
{
	struct mt9m114_status dev_status;
	int err;

	mt_info("mt9m114: mt9m114_ioctl : mt9m114_open");

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();
	else 
		mt9m114_power_on();

	dev_status.data = 0;
	dev_status.status = 0;
	err = mt9m114_get_status(info, &dev_status);
	return err;
}

int mt9m114_release(struct inode *inode, struct file *file)
{

	mt_info("mt9m114: mt9m114_ioctl : mt9m114_release");
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();
	else 
		mt9m114_power_off();

	file->private_data = NULL;
	return 0;
}

static const struct file_operations mt9m114_fileops = {
	.owner = THIS_MODULE,
	.open = mt9m114_open,
	.unlocked_ioctl = mt9m114_ioctl,
	.release = mt9m114_release,
};

static struct miscdevice mt9m114_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mt9m114",
	.fops = &mt9m114_fileops,
};

static int mt9m114_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	mt_info("mt9m114: probing sensor.\n");

	info = kzalloc(sizeof(struct mt9m114_info), GFP_KERNEL);
	if (!info) {
		mt_err("mt9m114: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&mt9m114_device);
	if (err) {
		mt_err("mt9m114: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;

	i2c_set_clientdata(client, info);

#if 1
	tegra_gpio_enable(VT_1V8V_EN);
	gpio_request(VT_1V8V_EN, "vt_1.8v_en");
	tegra_gpio_enable(SUB_CAM_RESET_N);
	gpio_request(SUB_CAM_RESET_N, "sub_cam_reset_n");
#endif

	return 0;
}

static int mt9m114_remove(struct i2c_client *client)
{
	struct mt9m114_info *info;
	info = i2c_get_clientdata(client);
	misc_deregister(&mt9m114_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id mt9m114_id[] = {
	{ "mt9m114", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static struct i2c_driver mt9m114_i2c_driver = {
	.driver = {
		.name = "mt9m114",
		.owner = THIS_MODULE,
	},
	.probe = mt9m114_probe,
	.remove = mt9m114_remove,
	.id_table = mt9m114_id,
};

static int __init mt9m114_init(void)
{
	mt_info("mt9m114 sensor driver loading\n");
	return i2c_add_driver(&mt9m114_i2c_driver);
}

static void __exit mt9m114_exit(void)
{
	i2c_del_driver(&mt9m114_i2c_driver);
}

module_init(mt9m114_init);
module_exit(mt9m114_exit);
