/*
* dw9714.c - focuser driver
*
* Copyright (c) 2011, NVIDIA, All Rights Reserved.
*
 * Contributors:
 *      Sachin Nikam <snikam@nvidia.com>
 *
 * Based on ov5650.c.
 *
* This file is licensed under the terms of the GNU General Public License
* version 2. This program is licensed "as is" without any warranty of any
* kind, whether express or implied.
*/

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/dw9714.h>
#include <linux/gpio.h>

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
#include <linux/regulator/lp8720.h>
extern void subpm_set_output(subpm_output_enum outnum, int onoff);
extern void subpm_set_gpio(int onoff);
#endif

#define POS_LOW       0
#define POS_HIGH      1023
#define SETTLETIME_MS 30
#define FOCAL_LENGTH  (3.2f)
#define FNUMBER       (2.4f)

DEFINE_MUTEX(dw9714_lock);
#define DW9714_MAX_RETRIES (3)

#define DW9714_VCM_PWD_GPIO     137 

#define DW9714_USE_LINEAR_SLOPE_CONTROL (1)

static struct dw9714_info *info;

static int dw9714_write(struct i2c_client *client, u16 value)
{
	int count;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

#if DW9714_USE_LINEAR_SLOPE_CONTROL
 	data[0] = value >> 8;
 	data[1] = value & 0xff;
#else
	data[0] = (u8) ((value >> 4) & 0x3F);
	data[1] = value & 0xff;
#endif

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = ARRAY_SIZE(data);
	msg[0].buf = data;

	do {
		count = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (count == ARRAY_SIZE(msg))
			return 0;
		retry++;
		pr_err("dw9714: i2c transfer failed, retrying %x\n",
				value);
		msleep(1);
	} while (retry <= DW9714_MAX_RETRIES);
	return -EIO;  
}

#ifndef JKR_TEST
static u32 register_code;
static ssize_t dw9714_ctrl_store(struct device* dev,                                                            
							struct device_attribute* attr, const char* buf, size_t count) 
{                                                                                                                    
	unsigned long	position	=	simple_strtoul(buf, NULL, 10);                                                
	int err;
	
	printk(KERN_INFO "enter %s\n", __func__);																													 

	if (position < 0 || position > 1023){
    	pr_err("[CAM] DW9714: INVALID position = %l (low:0, high:1023)", position);
		return count;
	}
	register_code = position;
	err = dw9714_write(info->i2c_client, 0xECA3);
  	if (err)
      goto dw9714_fail;
	err = dw9714_write(info->i2c_client, 0xF200|(0x0F<<3));
	if (err)
	  goto dw9714_fail;
	err = dw9714_write(info->i2c_client, 0xDC51);
	if (err)
	  goto dw9714_fail;
	err = dw9714_write(info->i2c_client, ((position<<4 |
	                                        (0x3 << 2 ) |
	                                        (0x0 << 0))));
	if (err)
	  goto dw9714_fail;

	return count;
	  
	dw9714_fail:
	  pr_err("[CAM] DW9714: %s: set position failed(%d)\n", __func__, err);
	return count;
}                                                                                                                    
                                                                                                                     
static ssize_t dw9714_ctrl_show(struct device* dev,                                                             
							struct device_attribute* attr, const char* buf, size_t count) 
{    
	printk(KERN_INFO "enter %s\n", __func__);

	return	snprintf(buf, PAGE_SIZE, "%d\n", register_code);                                                   
}                                                                                                                    
static DEVICE_ATTR(dw9714_ctrl, 0664, dw9714_ctrl_show, dw9714_ctrl_store);
#endif

static int dw9714_set_position(struct dw9714_info *info, u32 position)
{
  
	int err;

	if (position < info->config.pos_low || position > info->config.pos_high){
		pr_err(" [CAM] DW9714 !! WARNNG TRIED TO SET POSITION !! position = %d, info->config.pos_low = %d, info->config.pos_high = %d\n",
				position, info->config.pos_low, info->config.pos_high);
	}
	
	if(register_code != 0) position = register_code;
	
#if DW9714_USE_LINEAR_SLOPE_CONTROL
  err = dw9714_write(info->i2c_client, ((position<<4 | (0x3 << 2 ) | (0x2 << 0))));
  if (err)
      goto dw9714_set_position_fail;

  return 0;
  
  dw9714_set_position_fail:
      pr_err("[CAM] DW9714: %s: set position failed\n", __func__);
      return err;
#else
	return dw9714_write(info->i2c_client, position);
#endif
}

static long dw9714_ioctl(struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct dw9714_info *info = file->private_data;

	int ret;
	switch (cmd) {
	case DW9714_IOCTL_GET_CONFIG:
	{
		if (copy_to_user((void __user *) arg, &info->config, sizeof(info->config))) 
    {
			pr_err("%s: 0x%x\n", __func__, __LINE__);
			return -EFAULT;
		}
		break;
	}
	case DW9714_IOCTL_SET_POSITION:
		mutex_lock(&dw9714_lock);
		ret = dw9714_set_position(info, (u32) arg);
		mutex_unlock(&dw9714_lock);
		return ret;
    
	default:
		return -EINVAL;
	}

	return 0;
}

static int dw9714_power_on(void)
{

#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
  subpm_set_gpio(1);

  subpm_set_output(LDO5,1);
  mdelay(12);
#endif
  
  gpio_direction_output(DW9714_VCM_PWD_GPIO, 1);
  gpio_set_value(DW9714_VCM_PWD_GPIO, 1);  
  udelay(10);

	return 0;
}

static int dw9714_power_off(void)
{
    
    gpio_set_value(DW9714_VCM_PWD_GPIO, 0);
    gpio_direction_output(DW9714_VCM_PWD_GPIO, 0);    
    udelay(10);
    
#if defined(CONFIG_REGULATOR_CAM_SUBPMIC_LP8720)
    subpm_set_output(LDO5,0);
    udelay(100);
    
    subpm_set_gpio(0);
#endif
	return 0;
}

static int dw9714_open(struct inode *inode, struct file *file)
{
	int err;
	pr_info("%s\n", __func__);

  dw9714_power_on();

	
	file->private_data = info;
#if DW9714_USE_LINEAR_SLOPE_CONTROL
  err = dw9714_write(info->i2c_client, 0xECA3);
  if (err)
      goto dw9714_set_position_fail;

  err = dw9714_write(info->i2c_client, 0xF200|(0x0F<<3));
  if (err)
      goto dw9714_set_position_fail;

  err = dw9714_write(info->i2c_client, 0xDC51);
  if (err)
      goto dw9714_set_position_fail;
#endif
  
	return 0;

  dw9714_set_position_fail:
      pr_err("[CAM] DW9714: %s: set position failed\n", __func__);
      return err;

}

int dw9714_release(struct inode *inode, struct file *file)
{
  struct dw9714_info *info = file->private_data;
  int ret;
	pr_info("%s ver 1.2\n", __func__);
  
  mutex_lock(&dw9714_lock);
  ret = dw9714_set_position(info, (u32)0);
  mutex_unlock(&dw9714_lock);
  msleep(5);

	file->private_data = NULL;

  dw9714_power_off();
  
	return 0;
}


static const struct file_operations dw9714_fileops = {
	.owner = THIS_MODULE,
	.open = dw9714_open,
	.unlocked_ioctl = dw9714_ioctl,
	.release = dw9714_release,
};

static struct miscdevice dw9714_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dw9714",
	.fops = &dw9714_fileops,
};

static int dw9714_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;

	pr_info("%s\n", __func__);

	info = kzalloc(sizeof(struct dw9714_info), GFP_KERNEL);
	if (!info) {
		pr_err("dw9714: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&dw9714_device);
	if (err) {
		pr_err("dw9714: Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	tegra_gpio_enable(DW9714_VCM_PWD_GPIO);
	err = gpio_request(DW9714_VCM_PWD_GPIO, "8m_cam_vcm_pwd");
  if (err < 0)
    pr_err("%s: gpio_request failed for gpio %s\n",
      __func__, "8m_cam_vcm_pwd");

  
	info->i2c_client = client;
	info->config.settle_time = SETTLETIME_MS;
	info->config.focal_length = FOCAL_LENGTH;
	info->config.fnumber = FNUMBER;
	info->config.pos_low = POS_LOW;
	info->config.pos_high = POS_HIGH;
	i2c_set_clientdata(client, info);

#ifndef JKR_TEST
	if(device_create_file(&client->dev, &dev_attr_dw9714_ctrl)){                                                                                                      
		printk("[CAM] DW9714: device create file dw9714_ctrl fail!\n");                                      
	}
#endif
	
	pr_info("%s: Position low %d high %d defined in kernel drivre is\n", __func__, POS_LOW, POS_HIGH);
	return 0;
}

static int dw9714_remove(struct i2c_client *client)
{
	struct dw9714_info *info;
  
	pr_info("%s\n", __func__);
#ifndef JKR_TEST
	if (client != 0) {                                                                                            
		device_remove_file(&client->dev, &dev_attr_dw9714_ctrl);                                         
	}
#endif
	info = i2c_get_clientdata(client);
	misc_deregister(&dw9714_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id dw9714_id[] = {
	{ "dw9714", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, dw9714_id);

static struct i2c_driver dw9714_i2c_driver = {
	.driver = {
		.name = "dw9714",
		.owner = THIS_MODULE,
	},
	.probe = dw9714_probe,
	.remove = dw9714_remove,
	.id_table = dw9714_id,
};

static int __init dw9714_init(void)
{
	pr_info("dw9714 sensor driver loading\n");
	i2c_add_driver(&dw9714_i2c_driver);

	return 0;
}

static void __exit dw9714_exit(void)
{
	pr_info("%s\n", __func__);
	i2c_del_driver(&dw9714_i2c_driver);
}

module_init(dw9714_init);
module_exit(dw9714_exit);
