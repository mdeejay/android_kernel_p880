/*
 *
 * Copyright (C) 2009 LGE, Inc
 * Author: Jun-Yeong Han <j.y.han@lge.com>
 * Port: Yool-Je Cho <yoolje.cho@lge.com>
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

#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/limits.h>

#define ERS_DRIVER_NAME "lge-ers-kernel"

static atomic_t enable = ATOMIC_INIT(1);
static atomic_t report_num = ATOMIC_INIT(1);


static int panic_report(struct notifier_block *this,
		unsigned long event, void *ptr)
{
#if 0
	
#endif

	return NOTIFY_DONE;
}

static ssize_t ers_show(struct device *dev, 
			struct device_attribute *attr, 
			char *buf)
{
	int value = atomic_read(&enable);

	if (value == 0) {
		printk("The ers of kernel was disabled.\n");
	} 
	else {
		printk("The ers of kernel was enabled.\n");
	}

	return value;
}

static ssize_t ers_store(struct device *dev, 
			 struct device_attribute *attr, 
			 const char *buf, 
			 size_t size)
{
	int value;

	sscanf(buf, "%d", &value);
	atomic_set(&enable, value);

	return size;
}

static DEVICE_ATTR(ers, S_IRUGO | S_IWUSR, ers_show, ers_store);

static ssize_t ers_panic_store(struct device *dev, 
			       struct device_attribute *attr, 
			       const char *buf, 
			       size_t size)
{
	BUG();

	return size;
}

static DEVICE_ATTR(ers_panic, S_IRUGO | S_IWUSR, 0, ers_panic_store);

static struct notifier_block ers_block = {
	    .notifier_call  = panic_report,
};

static int __devinit ers_probe(struct platform_device *pdev)
{
	int ret;

	atomic_notifier_chain_register(&panic_notifier_list, &ers_block);

	ret = device_create_file(&pdev->dev, &dev_attr_ers);
	if (ret < 0) {
		printk("device_create_file error!\n");
		return ret;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_ers_panic);
	if (ret < 0) {
		printk("device_create_file error!\n");
		return ret;
	}

	return ret;
}

static int __devexit ers_remove(struct platform_device *pdev)
{	
	device_remove_file(&pdev->dev, &dev_attr_ers);
	device_remove_file(&pdev->dev, &dev_attr_ers_panic);

	return 0;
}

static struct platform_driver ers_driver = {
	.probe	= ers_probe,
	.remove = __devexit_p(ers_remove),
	.driver = {
		.name = ERS_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ers_init(void)
{
	return platform_driver_register(&ers_driver);
}
module_init(ers_init);

static void __exit ers_exit(void)
{
	platform_driver_unregister(&ers_driver);
}
module_exit(ers_exit);

MODULE_DESCRIPTION("Exception Reporting System Driver");
MODULE_AUTHOR("Jun-Yeong Han <junyeong.han@lge.com>");
MODULE_AUTHOR("Yool-Je Cho <yoolje.cho@lge.com>");
MODULE_LICENSE("GPL");
