/*
 * leds-lm3533.c -- LM3533 LED driver
 *
 * Copyright (C) 2011 Texas Instruments
 *
 * Author: Johan Hovold <jhovold@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/file.h>
#include "lm3533.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define LM3533_LVCTRLBANK_MIN		2
#define LM3533_LVCTRLBANK_MAX		5
#define LM3533_LVCTRLBANK_COUNT		4
#define LM3533_RISEFALLTIME_MAX		7
#define LM3533_ALS_LV_MIN		2
#define LM3533_ALS_LV_MAX		3

#define LM3533_REG_CTRLBANK_BCONF_BASE		0x1b
#define LM3533_REG_PATTERN_ENABLE		0x28
#define LM3533_REG_PATTERN_LOW_TIME_BASE	0x71
#define LM3533_REG_PATTERN_HIGH_TIME_BASE	0x72
#define LM3533_REG_PATTERN_RISETIME_BASE	0x74
#define LM3533_REG_PATTERN_FALLTIME_BASE	0x75

#define LM3533_REG_PATTERN_STEP			0x10

#define LM3533_REG_CTRLBANK_BCONF_MAPPING_MASK	0x04
#define LM3533_REG_CTRLBANK_BCONF_ALS_MASK	0x03

#define LM3533_LED_FLAG_PATTERN_ENABLE		1


struct lm3533_led {
	struct lm3533 *lm3533;
	struct lm3533_ctrlbank cb;
	struct led_classdev cdev;
	int id;

	struct mutex mutex;
	unsigned long flags;

	struct work_struct work;
	u8 new_brightness;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif

};

#define to_lm3533_led(_cdev) \
	container_of(_cdev, struct lm3533_led, cdev)


static inline int lm3533_led_get_ctrlbank_id(struct lm3533_led *led)
{
	return led->id + 2;
}

static inline u8 lm3533_led_get_lv_reg(struct lm3533_led *led, u8 base)
{
	return base + led->id;
}

static inline u8 lm3533_led_get_pattern(struct lm3533_led *led)
{
	return led->id;
}

static inline u8 lm3533_led_get_pattern_reg(struct lm3533_led *led,
								u8 base)
{
	return base + lm3533_led_get_pattern(led) * LM3533_REG_PATTERN_STEP;
}

static int lm3533_led_pattern_enable(struct lm3533_led *led, int enable)
{
	u8 mask;
	u8 val;
	int pattern;
	int state;
	int ret = 0;

	dev_dbg(led->cdev.dev, "%s - %d\n", __func__, enable);

	mutex_lock(&led->mutex);

	state = test_bit(LM3533_LED_FLAG_PATTERN_ENABLE, &led->flags);
	if ((enable && state) || (!enable && !state))
		goto out;

	pattern = lm3533_led_get_pattern(led);
	mask = 1 << (2 * pattern);

	if (enable)
		val = mask;
	else
		val = 0;

	ret = lm3533_update(led->lm3533, LM3533_REG_PATTERN_ENABLE, val, mask);
	if (ret) {
		dev_err(led->cdev.dev, "failed to enable pattern %d (%d)\n",
							pattern, enable);
		goto out;
	}

	__change_bit(LM3533_LED_FLAG_PATTERN_ENABLE, &led->flags);
out:
	mutex_unlock(&led->mutex);

	return ret;
}

static void lm3533_led_work(struct work_struct *work)
{
	struct lm3533_led *led = container_of(work, struct lm3533_led, work);

	dev_dbg(led->cdev.dev, "%s - %u\n", __func__, led->new_brightness);  
    
	if (led->new_brightness == 0)
		lm3533_led_pattern_enable(led, 0);	/* disable blink */

	lm3533_ctrlbank_set_brightness(&led->cb, led->new_brightness);
}

static void lm3533_led_set(struct led_classdev *cdev,
						enum led_brightness value)
{
	struct lm3533_led *led = to_lm3533_led(cdev);

	dev_dbg(led->cdev.dev, "%s - %d\n", __func__, value);
    
	led->new_brightness = value;
	schedule_work(&led->work);
}

static enum led_brightness lm3533_led_get(struct led_classdev *cdev)
{
	struct lm3533_led *led = to_lm3533_led(cdev);
	u8 val;
	int ret;

	ret = lm3533_ctrlbank_get_brightness(&led->cb, &val);
	if (ret)
		return ret;

	dev_dbg(led->cdev.dev, "%s - %u\n", __func__, val);

	return val;
}

/* Pattern generator defines -- delays in us */
#define LM3533_LED_DELAY_GROUP1_BASE	0x00
#define LM3533_LED_DELAY_GROUP2_BASE	0x3d
#define LM3533_LED_DELAY_GROUP3_BASE	0x80
#define LM3533_LED_DELAY_MAX		0xff

#define LM3533_LED_DELAY_GROUP1_STEP	16384
#define LM3533_LED_DELAY_GROUP2_STEP	131072
#define LM3533_LED_DELAY_GROUP3_STEP	524288
#define LM3533_LED_DELAY_GROUP1_MIN	16384
#define LM3533_LED_DELAY_GROUP2_MIN	1130496
#define LM3533_LED_DELAY_GROUP3_MIN	10305536
#define LM3533_LED_DELAY_GROUP1_MAX	999424
#define LM3533_LED_DELAY_GROUP2_MAX	9781248
#define LM3533_LED_DELAY_GROUP3_MAX	76890112

/* Delay limits in ms */
#define LM3533_LED_DELAY_ON_MAX		9845
#define LM3533_LED_DELAY_OFF_MAX	77140

static int time_to_val(long *t, long t_min, long t_max, long t_step,
							int v_min, int v_max)
{
	int val;

	*t += t_step / 2;
	val = (*t - t_min) / t_step + v_min;
	val = clamp(val, v_min, v_max);
	*t = t_step * (val - v_min) + t_min;

	return val;
}

static int lm3533_led_get_delay(long *delay)
{
	int val;

	*delay *= 1000;

	if (*delay >= LM3533_LED_DELAY_GROUP3_MIN -
					LM3533_LED_DELAY_GROUP3_STEP / 2) {
		val = time_to_val(delay, LM3533_LED_DELAY_GROUP3_MIN,
					LM3533_LED_DELAY_GROUP3_MAX,
					LM3533_LED_DELAY_GROUP3_STEP,
					LM3533_LED_DELAY_GROUP3_BASE,
					0xff);
	} else if (*delay >= LM3533_LED_DELAY_GROUP2_MIN -
					LM3533_LED_DELAY_GROUP2_STEP / 2) {
		val = time_to_val(delay, LM3533_LED_DELAY_GROUP2_MIN,
					LM3533_LED_DELAY_GROUP2_MAX,
					LM3533_LED_DELAY_GROUP2_STEP,
					LM3533_LED_DELAY_GROUP2_BASE,
					LM3533_LED_DELAY_GROUP3_BASE - 1);
	} else {
		val = time_to_val(delay, LM3533_LED_DELAY_GROUP1_MIN,
					LM3533_LED_DELAY_GROUP1_MAX,
					LM3533_LED_DELAY_GROUP1_STEP,
					LM3533_LED_DELAY_GROUP1_BASE,
					LM3533_LED_DELAY_GROUP2_BASE - 1);
	}

	*delay /= 1000;

	return val;
}

static int lm3533_led_delay_set(struct lm3533_led *led, u8 base,
							unsigned long *delay)
{
	u8 val;
	u8 reg;
	long t;
	int ret;

	t = *delay;
	val = lm3533_led_get_delay(&t);

	dev_dbg(led->cdev.dev, "%s - %lu: %ld (0x%02x)\n", __func__,
							*delay, t, val);
	reg = lm3533_led_get_pattern_reg(led, base);
	ret = lm3533_write(led->lm3533, reg, val);
	if (ret)
		dev_err(led->cdev.dev, "failed to set delay (%02x)\n", reg);

	*delay = t;

	return ret;
}

static int lm3533_led_delay_on_set(struct lm3533_led *led, unsigned long *t)
{
	*t = min_t(long, *t, LM3533_LED_DELAY_GROUP2_MAX / 1000);

	return lm3533_led_delay_set(led, LM3533_REG_PATTERN_HIGH_TIME_BASE, t);
}

static int lm3533_led_delay_off_set(struct lm3533_led *led, unsigned long *t)
{
	*t = min_t(long, *t, LM3533_LED_DELAY_GROUP3_MAX / 1000);

	return lm3533_led_delay_set(led, LM3533_REG_PATTERN_LOW_TIME_BASE, t);
}

static int lm3533_led_blink_set(struct led_classdev *cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	struct lm3533_led *led = to_lm3533_led(cdev);
	int ret;

	dev_dbg(led->cdev.dev, "%s - on = %lu, off = %lu\n", __func__,
							*delay_on, *delay_off);

	if (*delay_on > LM3533_LED_DELAY_ON_MAX ||
					*delay_off > LM3533_LED_DELAY_OFF_MAX)
		return -EINVAL;

	if (*delay_on == 0 && *delay_off == 0) {
		*delay_on = 500;
		*delay_off = 500;
	}

	ret = lm3533_led_delay_on_set(led, delay_on);
	if (ret)
		return ret;

	ret = lm3533_led_delay_off_set(led, delay_off);
	if (ret)
		return ret;

	return lm3533_led_pattern_enable(led, 1);
}

static ssize_t show_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);

	return sprintf(buf, "%d\n", led->id);
}

/*
 * Pattern generator rise/fall times:
 *
 *   0 - 2048 us (default)
 *   1 - 262 ms
 *   2 - 524 ms
 *   3 - 1.049 s
 *   4 - 2.097 s
 *   5 - 4.194 s
 *   6 - 8.389 s
 *   7 - 16.78 s
 */
static ssize_t show_risefalltime(struct device *dev,
					struct device_attribute *attr,
					char *buf, u8 base)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	ssize_t ret;
	u8 reg;
	u8 val;

	reg = lm3533_led_get_pattern_reg(led, base);
	ret = lm3533_read(led->lm3533, reg, &val);
	if (ret)
		return ret;

	return sprintf(buf, "%x\n", val);
}

static ssize_t show_risetime(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_risefalltime(dev, attr, buf,
					LM3533_REG_PATTERN_RISETIME_BASE);
}

static ssize_t show_falltime(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_risefalltime(dev, attr, buf,
					LM3533_REG_PATTERN_FALLTIME_BASE);
}

static ssize_t store_risefalltime(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len, u8 base)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	unsigned long val;
	u8 reg;
	int ret;

	if (strict_strtoul(buf, 0, &val) || val > LM3533_RISEFALLTIME_MAX)
		return -EINVAL;

	reg = lm3533_led_get_pattern_reg(led, base);
	ret = lm3533_write(led->lm3533, reg, (u8)val);
	if (ret)
		return ret;

	return len;
}

static ssize_t store_risetime(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	return store_risefalltime(dev, attr, buf, len,
					LM3533_REG_PATTERN_RISETIME_BASE);
}

static ssize_t store_falltime(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	return store_risefalltime(dev, attr, buf, len,
					LM3533_REG_PATTERN_FALLTIME_BASE);
}

/*
 * ALS settings:
 *
 *   0 - ALS disabled
 *   2 - ALS mapper 2
 *   3 - ALS mapper 3
 */
static ssize_t show_als(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	u8 reg;
	u8 val;
	int als;
	int ret;

	reg = lm3533_led_get_lv_reg(led, LM3533_REG_CTRLBANK_BCONF_BASE);
	ret = lm3533_read(led->lm3533, reg, &val);
	if (ret)
		return ret;

	als = val & LM3533_REG_CTRLBANK_BCONF_ALS_MASK;

	return sprintf(buf, "%d\n", als);
}

static ssize_t store_als(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	unsigned long als;
	u8 reg;
	u8 mask;
	int ret;

	if (strict_strtoul(buf, 0, &als))
		return -EINVAL;

	if (als != 0 && (als < LM3533_ALS_LV_MIN || als > LM3533_ALS_LV_MAX))
		return -EINVAL;

	reg = lm3533_led_get_lv_reg(led, LM3533_REG_CTRLBANK_BCONF_BASE);
	mask = LM3533_REG_CTRLBANK_BCONF_ALS_MASK;

	ret = lm3533_update(led->lm3533, reg, (u8)als, mask);
	if (ret)
		return ret;

	return len;
}

static ssize_t show_linear(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	u8 reg;
	u8 val;
	int linear;
	int ret;

	reg = lm3533_led_get_lv_reg(led, LM3533_REG_CTRLBANK_BCONF_BASE);
	ret = lm3533_read(led->lm3533, reg, &val);
	if (ret)
		return ret;

	if (val & LM3533_REG_CTRLBANK_BCONF_MAPPING_MASK)
		linear = 1;
	else
		linear = 0;

	return sprintf(buf, "%x\n", linear);
}

static ssize_t store_linear(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	unsigned long linear;
	u8 reg;
	u8 mask;
	u8 val;
	int ret;

	if (strict_strtoul(buf, 0, &linear))
		return -EINVAL;

	reg = lm3533_led_get_lv_reg(led, LM3533_REG_CTRLBANK_BCONF_BASE);
	mask = LM3533_REG_CTRLBANK_BCONF_MAPPING_MASK;

	if (linear)
		val = mask;
	else
		val = 0;

	ret = lm3533_update(led->lm3533, reg, val, mask);
	if (ret)
		return ret;

	return len;
}

#define show_ctrlbank_attr(_name)					\
static ssize_t show_##_name(struct device *dev,				\
				struct device_attribute *attr,		\
				char *buf)				\
{									\
	struct led_classdev *led_cdev = dev_get_drvdata(dev);		\
	struct lm3533_led *led = to_lm3533_led(led_cdev);		\
	u8 val;								\
	int ret;							\
									\
	ret = lm3533_ctrlbank_get_##_name(&led->cb, &val);		\
	if (ret)							\
		return ret;						\
									\
	return sprintf(buf, "%d\n", val);				\
}

#define store_ctrlbank_attr(_name)					\
static ssize_t store_##_name(struct device *dev,			\
				struct device_attribute *attr,		\
				const char *buf, size_t len)		\
{									\
	struct led_classdev *led_cdev = dev_get_drvdata(dev);		\
	struct lm3533_led *led = to_lm3533_led(led_cdev);		\
	unsigned long val;						\
	int ret;							\
									\
	if (strict_strtoul(buf, 0, &val) || val > LM3533_ATTR_VAL_MAX)	\
		return -EINVAL;						\
									\
	ret = lm3533_ctrlbank_set_##_name(&led->cb, (u8)val);		\
	if (ret)							\
		return ret;						\
									\
	return len;							\
}

show_ctrlbank_attr(max_current);
store_ctrlbank_attr(max_current);
show_ctrlbank_attr(pwm);
store_ctrlbank_attr(pwm);

lm3533_attr_rw(als);
lm3533_attr_rw(falltime);
lm3533_attr_ro(id);
lm3533_attr_rw(linear);
lm3533_attr_rw(max_current);
lm3533_attr_rw(pwm);
lm3533_attr_rw(risetime);

static struct attribute *lm3533_led_attributes[] = {
	&dev_attr_als.attr,
	&dev_attr_falltime.attr,
	&dev_attr_id.attr,
	&dev_attr_linear.attr,
	&dev_attr_max_current.attr,
	&dev_attr_pwm.attr,
	&dev_attr_risetime.attr,
	NULL,
};

static mode_t lm3533_led_attr_is_visible(struct kobject *kobj,
					     struct attribute *attr, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_led *led = to_lm3533_led(led_cdev);
	mode_t mode = attr->mode;

	if (attr == &dev_attr_als.attr) {
		if (!led->lm3533->have_als)
			mode = 0;
	}

	return mode;
};

static struct attribute_group lm3533_led_attribute_group = {
	.is_visible	= lm3533_led_attr_is_visible,
	.attrs		= lm3533_led_attributes
};

static int lm3533_led_setup(struct lm3533_led *led,
					struct lm3533_led_platform_data *pdata)
{
	int ret;


	ret = lm3533_ctrlbank_set_max_current(&led->cb, pdata->max_current);
	if (ret)
		return ret;

	return lm3533_ctrlbank_set_pwm(&led->cb, pdata->pwm);
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3533_led_early_suspend(struct early_suspend *handler)
{
	struct lm3533_led *led;

	led = container_of(handler, struct lm3533_led, early_suspend);

	lm3533_ctrlbank_disable(&led->cb);
}

static void lm3533_led_late_resume(struct early_suspend *handler)
{
	struct lm3533_led *led;

	led = container_of(handler, struct lm3533_led, early_suspend);

	lm3533_ctrlbank_enable(&led->cb);
}

static void lm3533_led_register_early_suspend(struct lm3533_led *led)
{
	led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	led->early_suspend.suspend = lm3533_led_early_suspend;
	led->early_suspend.resume = lm3533_led_late_resume;

	register_early_suspend(&led->early_suspend);
}

static void lm3533_led_unregister_early_suspend(struct lm3533_led *led)
{
	unregister_early_suspend(&led->early_suspend);
}
#endif

static int lm3533_led_probe(struct platform_device *pdev)
{
	struct lm3533 *lm3533;
	struct lm3533_led_platform_data *pdata;
	struct lm3533_led *led;
	int ret;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	lm3533 = dev_get_drvdata(pdev->dev.parent);
	if (!lm3533)
		return -EINVAL;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "no platform data\n");
		return -EINVAL;
	}

	if (pdev->id < 0 || pdev->id >= LM3533_LVCTRLBANK_COUNT) {
		dev_err(&pdev->dev, "illegal LED id %d\n", pdev->id);
		return -EINVAL;
	}

	led = kzalloc(sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->lm3533 = lm3533;
	led->cdev.name = pdata->name;
	led->cdev.default_trigger = pdata->default_trigger;
	led->cdev.brightness_set = lm3533_led_set;
	led->cdev.brightness_get = lm3533_led_get;
	led->cdev.blink_set = lm3533_led_blink_set;
	led->cdev.brightness = LED_OFF;
	led->id = pdev->id;

	mutex_init(&led->mutex);
	INIT_WORK(&led->work, lm3533_led_work);

	/* The class framework makes a callback to get brightness during
	 * registration so use parent device (for error reporting) until
	 * registered.
	 */
	led->cb.lm3533 = lm3533;
	led->cb.id = lm3533_led_get_ctrlbank_id(led);
	led->cb.dev = lm3533->dev; 

	platform_set_drvdata(pdev, led);

	ret = led_classdev_register(pdev->dev.parent, &led->cdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register LED %d\n", pdev->id);
		goto err_free;
	}

	led->cb.dev = led->cdev.dev; 
	//lm3533_led_register_early_suspend(led);

	ret = sysfs_create_group(&led->cdev.dev->kobj,
						&lm3533_led_attribute_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs attributes\n");
		goto err_unregister;
	}

	ret = lm3533_led_setup(led, pdata);
	if (ret)
		goto err_sysfs_remove;

	ret = lm3533_ctrlbank_enable(&led->cb);
	if (ret)
		goto err_sysfs_remove;

	return 0;

err_sysfs_remove:
	sysfs_remove_group(&led->cdev.dev->kobj, &lm3533_led_attribute_group);
err_unregister:
	led_classdev_unregister(&led->cdev);
	flush_work(&led->work);
	cancel_work_sync(&led->work);
err_free:
	kfree(led);

	return ret;
}

static int lm3533_led_remove(struct platform_device *pdev)
{
	struct lm3533_led *led = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	lm3533_ctrlbank_disable(&led->cb);
	sysfs_remove_group(&led->cdev.dev->kobj, &lm3533_led_attribute_group);
	led_classdev_unregister(&led->cdev);
	flush_work(&led->work);
	cancel_work_sync(&led->work);
	kfree(led);

	return 0;
}

static void lm3533_led_shutdown(struct platform_device *pdev)
{

	struct lm3533_led *led = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	lm3533_ctrlbank_disable(&led->cb);
	lm3533_led_set(&led->cdev, LED_OFF);		/* disable blink */
	flush_work(&led->work);
	cancel_work_sync(&led->work);
}

int power_off_led_count =0;// justin temp solution for led 0.6mA.
int is_led_suspend_enter = 0;

static int lm3533_led_resume(struct platform_device *pdev)
{
    int ret;
	struct lm3533_led *led = platform_get_drvdata(pdev);
    
    if(is_led_suspend_enter==1)
    {
    	lm3533_enable(led->lm3533);
        
    	ret = lm3533_set_hvled_config(led->lm3533, 2, 0) ;
    	if (ret)
    	    printk( "lm3533_set_hvled_config error%s\n", __func__);

    	ret = lm3533_set_boost_ovp(led->lm3533, 3) ;
    	if (ret)
    	    printk( "lm3533_set_boost_ovp error %s\n", __func__);

        is_led_suspend_enter = 0;
    }

	ret = lm3533_led_setup(led, pdev->dev.platform_data);
	if (ret)
	    printk( "lm3533_led_setup error %s\n", __func__);

	ret = lm3533_ctrlbank_enable(&led->cb);
	if (ret)
	    printk( "lm3533_ctrlbank_enable error %s\n", __func__);

	return 0;
}

static int lm3533_led_suspend(struct platform_device *pdev,
	pm_message_t state)
{
    int ret;
	struct lm3533_led *led = platform_get_drvdata(pdev);

    if(pdev->id==3)
        return 0;//this is button backlight.

    if(led->new_brightness==0)
        power_off_led_count++;
    
    if((pdev->id==0)&&(power_off_led_count==3))
    {
        lm3533_disable(led->lm3533);
        is_led_suspend_enter = 1;
        power_off_led_count = 0;
    }
    else if((pdev->id==0)&&(power_off_led_count!=3))
    {
            power_off_led_count = 0;//to be fix;
    }
    
	return 0;
}

static struct platform_driver lm3533_led_driver = {
	.driver = {
		.name = "lm3533-leds",
		.owner = THIS_MODULE,
	},
	.suspend    = lm3533_led_suspend,
	.resume     = lm3533_led_resume,
	.probe		= lm3533_led_probe,
	.remove		= lm3533_led_remove,
	.shutdown	= lm3533_led_shutdown,
};

static int __init lm3533_led_init(void)
{
	return platform_driver_register(&lm3533_led_driver);
}
module_init(lm3533_led_init);

static void __exit lm3533_led_exit(void)
{
	platform_driver_unregister(&lm3533_led_driver);
}
module_exit(lm3533_led_exit);

MODULE_AUTHOR("Johan Hovold <jhovold@gmail.com>");
MODULE_DESCRIPTION("LM3533 LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lm3533-leds");
