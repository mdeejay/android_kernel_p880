/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * Copyright 2010-2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */



#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/hook_test_header.h>

#define FACTORY_AT_COMMAND_GKPD


#define LGE_GPIO_KEY_NOT_USED_RESUME

#if defined (FACTORY_AT_COMMAND_GKPD)
#include <linux/wakelock.h>
#endif 

enum gpio_key_status{
	GPIO_KEY_RELEASED = 0,
	GPIO_KEY_PRESSED,
	GPIO_KEY_NONE,
	GPIO_KEY_MAX,
};


struct gpio_button_data {
	struct gpio_keys_button *button;
	struct input_dev *input;
#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
	struct workqueue_struct	 *button_wq;
	struct work_struct 	button_work;
	struct delayed_work work_delayed_rising;
	struct delayed_work work_delayed_falling;
	struct timespec debouncetime;	
	enum   gpio_key_status keystatus;
#else	
	struct timer_list timer;
	struct work_struct work;
#endif	
	int timer_debounce;	/* in msecs */
	bool disabled;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
	struct workqueue_struct	 *button_wq;
#endif
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct gpio_button_data data[0];
};


#ifdef FACTORY_AT_COMMAND_GKPD
static unsigned int test_mode = 0;
static int test_code = 0, gkpd_last_index = 0;
static unsigned char gkpd_value[21];
static struct wake_lock key_wake_lock;
#endif

static int gpiokeyinitdebounce = -1;

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

static inline struct device *_to_parent(struct input_dev *input)
{
	return input->dev.parent;
}


/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(gpio_to_irq(bdata->button->gpio));
#ifndef	LGE_GPIO_KEY_NOT_USED_RESUME		
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);
#endif
		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(gpio_to_irq(bdata->button->gpio));
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordinly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);




#ifdef FACTORY_AT_COMMAND_GKPD
int get_test_mode(void)
{
	return test_mode;
}
EXPORT_SYMBOL(get_test_mode);


typedef struct
{
	char         out;
	unsigned char in;

} Conv;

 Conv GKPD_table[]=
 {
	 {'D',  KEY_VOLUMEDOWN}
	,{'U',  KEY_VOLUMEUP}

#if defined(CONFIG_MACH_LGE_COSMOPOLITAN)
	,{'F',  KEY_3D}
#elif defined(CONFIG_MACH_LGE_P940)
#if defined(CONFIG_MACH_LGE_P940_EVB)
	,{'F',  KEY_GESTURE}
#else
	,{'F',  KEY_CAPTURE}
#endif
#endif
	,{'H',  KEY_HOOK}
	,{'B',  KEY_POWER} 
	,{0, 0}
};

int gkpd_KeyConvert(int key)
{
	u16 indexCount = 4;
	int i = 0;
	
	while ((i < indexCount) && (key != 0xFF))
	{
		if (GKPD_table[i].in == key)
			return GKPD_table[i].out;
		i++;
	}
	return key;
}

EXPORT_SYMBOL(gkpd_KeyConvert);

void write_gkpd_value(int value)
{
	int i;
	int con ;

	con=gkpd_KeyConvert(value);
	value = con;

	if (gkpd_last_index == 20) {
		gkpd_value[gkpd_last_index] = value;
		for ( i = 0; i < 20 ; i++) {
			gkpd_value[i] = gkpd_value[i + 1];
		}			
		gkpd_value[gkpd_last_index] = '\n';
	}
	else {
		gkpd_value[gkpd_last_index] = value;
		gkpd_value[gkpd_last_index + 1] = '\n';
		gkpd_last_index++;
		
		printk("[GPIO_KEYS] %s() gkpd_value[%d]:%d %c\n", __func__, gkpd_last_index, value, value);

	}		
}

EXPORT_SYMBOL(write_gkpd_value);


static ssize_t keypad_test_mode_show(struct device *dev,  struct device_attribute *attr,  char *buf)
{	
	int i;
	int ret = 0;
	int written_cnt = 0;
	int total_written_cnt = 0;

	dev_dbg(dev, "[GPIO_KEYS] %s() gkpd_last_index : %d\n", __func__, gkpd_last_index);

	for(i = 0; i < gkpd_last_index; i++)
	{
		dev_dbg(dev, "[!] %s() code value : %c\n", __func__, gkpd_value[i]);
		written_cnt = sprintf(buf+total_written_cnt, "%c", gkpd_value[i]);
        total_written_cnt += written_cnt; 
	}

	ret = total_written_cnt ; 

	gkpd_last_index = 0;
	
	memset(gkpd_value, 0x00, sizeof(unsigned char)*21);

	return ret;

}

static ssize_t keypad_test_mode_store(struct device *dev,  struct device_attribute *attr,  const char *buf, size_t count)
{
    int ret;
	int i;

    ret = sscanf(buf, "%d", &test_mode);

	gkpd_last_index = ret;

	dev_dbg(dev, "[GPIO_KEYS] %s [%d]\n",__func__, gkpd_last_index);


	if(test_mode == 1)
	{
	
		wake_lock(&key_wake_lock);

		for(i = 0; i < gkpd_last_index; i++)
		{
			dev_dbg(dev, "test_mode[%d], keypad_test_mode_store[%d]", test_mode, gkpd_last_index);
			gkpd_value[i] = 0;
		}

		gkpd_last_index = 0;
	}
	else if(test_mode == 0)
	{
		wake_unlock(&key_wake_lock);
	}

	return count;
}
static DEVICE_ATTR(key_test_mode, 0664, keypad_test_mode_show, keypad_test_mode_store);
#endif //FACTORY_AT_COMMAND_GKPD

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	&dev_attr_key_test_mode.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};

static void gpio_keys_report_event(struct gpio_button_data *bdata)
{
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;

	dev_dbg(_to_parent(input), "[GPIO_KEYS] gpio_keys : %s\n",__func__);

	input_event(input, type, button->code, !!state);
	input_sync(input);	

	dev_dbg(_to_parent(input), "[GPIO_KEYS] test_mode :%d, code : %d, state :%d\n",test_mode, button->code, state);

#ifdef FACTORY_AT_COMMAND_GKPD
	if(test_mode == 1 && state )
	{
		dev_dbg(_to_parent(input), "[GPIO_KEYS] gpio_keys : %s GKPD 2 \n",__func__);

		test_code = button->code/* & ~GROUP_MASK*/;
		write_gkpd_value(test_code);
	}			
#endif

}

static void gpio_keys_work_func(struct work_struct *work)
{
#ifndef	LGE_GPIO_KEY_NOT_USED_RESUME
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);

	dev_dbg(_to_parent(bdata->input), "[GPIO_KEYS] gpio_keys : %s\n",__func__);

	gpio_keys_report_event(bdata);
#endif	
}

#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
static void gpio_keys_work_delayed_falling_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work_delayed_falling);
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;

	int status = !!gpio_get_value(button->gpio);
	//status 0  ==> Press
	dev_dbg("%s : [%d][%s][%d]\n",__func__, button->code, status?"Release":"Press", button->debounce_interval);

	if(status == 1 && button->debounce_interval) return;

	printk("%s : Press   [%d][%d]\n",__func__, button->code, button->isrPressCount);		

	input_report_key(input, button->code, 1);
	input_sync(input);
	bdata->keystatus = GPIO_KEY_PRESSED;

#ifdef FACTORY_AT_COMMAND_GKPD
	if(test_mode == 1)
	{
		dev_dbg(_to_parent(input), "[GPIO_KEYS] gpio_keys : %s GKPD 2 \n",__func__);

		test_code = button->code/* & ~GROUP_MASK*/;
		write_gkpd_value(test_code);
	}			
#endif

}

static void gpio_keys_work_delayed_rising_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work_delayed_rising);
	struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;


	int status = !!gpio_get_value(button->gpio);
	dev_dbg("%s : [%d][%s][%d]\n",__func__, button->code, status?"Release":"Press", button->debounce_interval);

//status 1 ==> Release
	if(status == 0 && button->debounce_interval) return; 

	printk("%s : Release [%d][%d]\n",__func__, button->code, button->isrReleaseCount);
	bdata->keystatus = GPIO_KEY_RELEASED;

	input_report_key(input, button->code, 0);
	input_sync(input);
}
#endif


static void gpio_keys_timer(unsigned long _data)
{
#ifndef	LGE_GPIO_KEY_NOT_USED_RESUME
	struct gpio_button_data *data = (struct gpio_button_data *)_data;

	schedule_work(&data->work);
#endif	
}

static void gpio_keys_work(struct work_struct *work)
{
	struct gpio_button_data *bdata = container_of(work, struct gpio_button_data, button_work);
	struct gpio_keys_button *button = bdata->button;


	int status = !!gpio_get_value(button->gpio);

	dev_dbg("%s : [%d][%s][%d]\n",__func__, button->code, status?"Release":"Press", button->debounce_interval);

	if(status) // Release
	{
		button->isrReleaseCount++;
		if(bdata->keystatus == GPIO_KEY_PRESSED)
		{
		#if 0
			cancel_delayed_work(&bdata->work_delayed_rising);
			schedule_delayed_work(&bdata->work_delayed_rising, msecs_to_jiffies(1));	
		#else
			cancel_delayed_work_sync(&bdata->work_delayed_rising);
			queue_delayed_work(bdata->button_wq, &bdata->work_delayed_rising, msecs_to_jiffies(button->debounce_interval));		
		#endif
		}
		else if(button->debounce_interval == 0)
		{
			queue_delayed_work(bdata->button_wq, &bdata->work_delayed_falling, 0);
			queue_delayed_work(bdata->button_wq, &bdata->work_delayed_rising, 0);
		}

	}
	else  //Press
	{
		button->isrPressCount++;
		if(bdata->keystatus == GPIO_KEY_NONE || bdata->keystatus == GPIO_KEY_RELEASED)
		{
			#if 0
			cancel_delayed_work(&bdata->work_delayed_falling);
			schedule_delayed_work(&bdata->work_delayed_falling, msecs_to_jiffies(1));	
			#else
			cancel_delayed_work_sync(&bdata->work_delayed_falling);
			queue_delayed_work(bdata->button_wq, &bdata->work_delayed_falling, msecs_to_jiffies(button->debounce_interval));
			#endif
		}		
	}

}

static irqreturn_t gpio_keys_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

#if 0  
	struct gpio_keys_button *button = bdata->button;
	int status = !!gpio_get_value(button->gpio);
	printk("%s : [%d][%s]\n",__func__, button->code, status?"Release":"Press");
#endif

	queue_work(bdata->button_wq, &bdata->button_work);

	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct gpio_button_data *bdata,
					 struct gpio_keys_button *button)
{
	char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	unsigned long irqflags;
	int irq, error;

	dev_dbg(dev, "[GPIO_KEYS] gpio_keys : %s\n",__func__);

#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
	INIT_WORK(&bdata->button_work, gpio_keys_work);	
	INIT_DELAYED_WORK_DEFERRABLE(&bdata->work_delayed_rising, gpio_keys_work_delayed_rising_func);
	INIT_DELAYED_WORK_DEFERRABLE(&bdata->work_delayed_falling, gpio_keys_work_delayed_falling_func);
#else
	setup_timer(&bdata->timer, gpio_keys_timer, (unsigned long)bdata);
	INIT_WORK(&bdata->work, gpio_keys_work_func);
#endif
	error = gpio_request(button->gpio, desc);
	if (error < 0) {
		dev_err(dev, "failed to request GPIO %d, error %d\n",
			button->gpio, error);
		goto fail2;
	}

	error = gpio_direction_input(button->gpio);
	if (error < 0) {
		dev_err(dev, "failed to configure"
			" direction for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	dev_dbg(dev, "[GPIO_KEYS] debounce_interval : %s [%d]\n",__func__, button->debounce_interval);

	if(gpiokeyinitdebounce != -1)
	{
		button->debounce_interval = gpiokeyinitdebounce;
	}

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			button->gpio, error);
		goto fail3;
	}

	button->isrPressCount = button->isrReleaseCount = 0;

#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(irq, gpio_keys_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail3;
	}
#else
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;


	error = request_any_context_irq(irq, gpio_keys_isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			irq, error);
		goto fail3;
	}
#endif

	return 0;

fail3:
	gpio_free(button->gpio);
fail2:
	return error;
}

static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	dev_dbg(_to_parent(input), "[GPIO_KEYS] gpio_keys : %s\n",__func__);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	dev_dbg(_to_parent(input), "[GPIO_KEYS] gpio_keys : %s\n",__func__);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;

	dev_dbg(dev, "[GPIO_KEYS] gpio_keys : %s\n",__func__);

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
	ddata->button_wq = create_singlethread_workqueue("gpio_key_wq");

	if (!ddata->button_wq)
	{
		error = -ENOMEM;
		goto fail1;
	}

#endif

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];
		unsigned int type = button->type ?: EV_KEY;

		bdata->input = input;
		bdata->button = button;
#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
		bdata->button_wq = ddata->button_wq;

		bdata->keystatus = GPIO_KEY_NONE;
#endif
		error = gpio_keys_setup_key(pdev, bdata, button);

		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;

		input_set_capability(input, type, button->code);
	}


	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		if (button->wakeup) {
			int irq = gpio_to_irq(button->gpio);
			enable_irq_wake(irq);
		}
	}

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

#ifdef FACTORY_AT_COMMAND_GKPD
	wake_lock_init(&key_wake_lock, WAKE_LOCK_SUSPEND, "sora-gpio-keys");
#endif

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}

#ifndef	LGE_GPIO_KEY_NOT_USED_RESUME
	/* get current state of buttons */
	for (i = 0; i < pdata->nbuttons; i++)
		gpio_keys_report_event(&ddata->data[i]);
	input_sync(input);
#endif
	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 fail2:
	while (--i >= 0) {
		free_irq(gpio_to_irq(pdata->buttons[i].gpio), &ddata->data[i]);
#ifndef	LGE_GPIO_KEY_NOT_USED_RESUME				
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);
		cancel_work_sync(&ddata->data[i].work);
#endif				
		gpio_free(pdata->buttons[i].gpio);
	}

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

#ifdef	LGE_GPIO_KEY_NOT_USED_RESUME
	if (ddata->button_wq)
		destroy_workqueue(ddata->button_wq);
#endif
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

#ifdef FACTORY_AT_COMMAND_GKPD
	wake_lock_destroy(&key_wake_lock);
#endif

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < pdata->nbuttons; i++) {
		int irq = gpio_to_irq(pdata->buttons[i].gpio);
		free_irq(irq, &ddata->data[i]);
#ifndef	LGE_GPIO_KEY_NOT_USED_RESUME				
		if (ddata->data[i].timer_debounce)
			del_timer_sync(&ddata->data[i].timer);		
		cancel_work_sync(&ddata->data[i].work);
#endif		
		gpio_free(pdata->buttons[i].gpio);
	}

	input_unregister_device(input);

	return 0;
}


#if 1
static int gpio_keys_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		button->debounce_interval = 0;
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data;
	int wakeup_key = KEY_RESERVED;
	int i;

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i];
		button->debounce_interval = 1;
	}

	return 0;
}

static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
};
#endif

static int __init gpio_keys_device_debounce(char *str)
{
	sscanf(str,"%d ",&gpiokeyinitdebounce);

	printk("%s [%d]\n", __func__, gpiokeyinitdebounce);
	return 1;
}
__setup("gpiokeydebounce=", gpio_keys_device_debounce);


static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
#if 1
		.pm	= &gpio_keys_pm_ops,
#endif
	}
};

static int __init gpio_keys_init(void)
{
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

module_init(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for CPU GPIOs");
MODULE_ALIAS("platform:gpio-keys");
