/*
 * lm3533-bl.c -- LM3533 Backlight driver
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
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <mach/mt_gpio.h>

#include "lm3533.h"
#include <linux/leds.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define LM3533_HVCTRLBANK_COUNT		2
#define LM3533_BL_MAX_BRIGHTNESS	255

#define LM3533_REG_CTRLBANK_AB_BCONF	0x1a


struct lm3533_bl {
	struct lm3533 *lm3533;
	struct lm3533_ctrlbank cb;
	//struct backlight_device *bd;
	struct led_classdev cdev;
	int id;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#define to_lm3533_bl(_cdev) \
	container_of(_cdev, struct lm3533_bl, cdev)


#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3533_bl_register_early_suspend(struct lm3533_bl *bl);
static void lm3533_bl_unregister_early_suspend(struct lm3533_bl *bl);
#else
static void lm3533_bl_register_early_suspend(struct lm3533_bl *bl) { }
static void lm3533_bl_unregister_early_suspend(struct lm3533_bl *bl) { }
#endif

static struct lm3533_bl *g_bl = NULL ;

void lm3533_backlight_disable()
{
	if(g_bl != NULL)
	lm3533_ctrlbank_disable(&g_bl->cb);
}
EXPORT_SYMBOL_GPL(lm3533_backlight_disable);

void lm3533_backlight_enable()
{
	if(g_bl != NULL)
	lm3533_ctrlbank_enable(&g_bl->cb);
}
EXPORT_SYMBOL_GPL(lm3533_backlight_enable);

void lm3533_backlight_linear_mapping(u8 val)
{
    if (val) {
		if(g_bl != NULL)
        lm3533_update(g_bl->cb.lm3533, LM3533_REG_CTRLBANK_AB_BCONF, 0xa, 0xa);
    }
}
EXPORT_SYMBOL_GPL(lm3533_backlight_linear_mapping);

void lm3533_backlight_pwm(u8 val)
{
	if(g_bl != NULL)
    lm3533_ctrlbank_set_pwm(&g_bl->cb, val);
}
EXPORT_SYMBOL_GPL(lm3533_backlight_pwm);

void lm3533_backlight_current(u8 val)
{
	if(g_bl != NULL)
	lm3533_ctrlbank_set_max_current(&g_bl->cb, val) ;
}
EXPORT_SYMBOL_GPL(lm3533_backlight_current);

void lm3533_backlight_brightness(u8 val)
{
	if(g_bl != NULL){
	    printk("lm3533: %p:%p\n",g_bl,&g_bl->cb);
		lm3533_ctrlbank_set_brightness(&g_bl->cb, val);	
	}
}
EXPORT_SYMBOL_GPL(lm3533_backlight_brightness);

static inline int lm3533_bl_get_ctrlbank_id(struct lm3533_bl *bl)
{
	return bl->id;
}

/*
static int lm3533_bl_update_status(struct backlight_device *bd)
{
	struct lm3533_bl *bl = bl_get_data(bd);
	int brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	return lm3533_ctrlbank_set_brightness(&bl->cb, (u8)brightness);
}

static int lm3533_bl_get_brightness(struct backlight_device *bd)
{
	struct lm3533_bl *bl = bl_get_data(bd);
	u8 val;
	int ret;

	ret = lm3533_ctrlbank_get_brightness(&bl->cb, &val);
	if (ret)
		return ret;

	return val;
}

static const struct backlight_ops lm3533_bl_ops = {
	.get_brightness	= lm3533_bl_get_brightness,
	.update_status	= lm3533_bl_update_status,
};
*/

static ssize_t show_id(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);

	return sprintf(buf, "%d\n", bl->id);
}

/*
 * ALS settings:
 *
 *   0 - ALS disabled
 *   1 - ALS mapper 1 (backlight 0)
 *   2 - ALS mapper 2 (backlight 1)
 */
static ssize_t show_als(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);
	int ctrlbank = lm3533_bl_get_ctrlbank_id(bl);
	u8 val;
	u8 mask;
	int als;
	int ret;

	ret = lm3533_read(bl->lm3533, LM3533_REG_CTRLBANK_AB_BCONF, &val);
	if (ret)
		return ret;

	mask = 2 * ctrlbank;
	als = val & mask;
	if (als)
		als = ctrlbank + 1;

	return sprintf(buf, "%d\n", als);
}

static ssize_t store_als(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);
	int ctrlbank = lm3533_bl_get_ctrlbank_id(bl);
	unsigned long als;
	u8 val;
	u8 mask;
	int ret;

	if (strict_strtoul(buf, 0, &als))
		return -EINVAL;

	if (als != 0 && (als != ctrlbank + 1))
		return -EINVAL;

	mask = 1 << (2 * ctrlbank);

	if (als)
		val = mask;
	else
		val = 0;

	ret = lm3533_update(bl->lm3533, LM3533_REG_CTRLBANK_AB_BCONF, val,
									mask);
	if (ret)
		return ret;

	return len;
}

static ssize_t show_linear(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);
	u8 val;
	u8 mask;
	int linear;
	int ret;

	if (bl == NULL) {
		printk("lm3533 lcd-backligth show linear bl NULL\n") ;
		return 0 ;
	}

	ret = lm3533_read(bl->lm3533, LM3533_REG_CTRLBANK_AB_BCONF, &val);
	if (ret)
		return ret;

	mask = 1 << (2 * lm3533_bl_get_ctrlbank_id(bl) + 1);

	if (val & mask)
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
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);
	unsigned long linear;
	u8 mask;
	u8 val;
	int ret;

	if (strict_strtoul(buf, 0, &linear))
		return -EINVAL;

	mask = 1 << (2 * lm3533_bl_get_ctrlbank_id(bl) + 1);

	if (linear)
		val = mask;
	else
		val = 0;

	ret = lm3533_update(bl->lm3533, LM3533_REG_CTRLBANK_AB_BCONF, val,
									mask);
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
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);			\
	u8 val;								\
	int ret;							\
									\
	ret = lm3533_ctrlbank_get_##_name(&bl->cb, &val);		\
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
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);			\
	unsigned long val;						\
	int ret;							\
									\
	if (strict_strtoul(buf, 0, &val) || val > LM3533_ATTR_VAL_MAX)	\
		return -EINVAL;						\
									\
	ret = lm3533_ctrlbank_set_##_name(&bl->cb, (u8)val);		\
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
lm3533_attr_ro(id);
lm3533_attr_rw(linear);
lm3533_attr_rw(max_current);
lm3533_attr_rw(pwm);

static struct attribute *lm3533_bl_attributes[] = {
	&dev_attr_als.attr,
	&dev_attr_id.attr,
	&dev_attr_linear.attr,
	&dev_attr_max_current.attr,
	&dev_attr_pwm.attr,
	NULL,
};

static mode_t lm3533_bl_attr_is_visible(struct kobject *kobj,
					     struct attribute *attr, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct lm3533_bl *bl = to_lm3533_bl(led_cdev);

	mode_t mode = attr->mode;

	if (attr == &dev_attr_als.attr) {
		if (!bl->lm3533->have_als)
			mode = 0;
	}

	return mode;
};

static struct attribute_group lm3533_bl_attribute_group = {
	.is_visible	= lm3533_bl_attr_is_visible,
	.attrs		= lm3533_bl_attributes
};

static int  lm3533_bl_setup(struct lm3533_bl *bl,
					struct lm3533_bl_platform_data *pdata)
{
	int ret;

	ret = lm3533_ctrlbank_set_max_current(&bl->cb, pdata->max_current);
	if (ret)
		return ret;

	ret = lm3533_ctrlbank_set_pwm(&bl->cb, pdata->pwm);
	if (ret)
		return ret ;

	return lm3533_ctrlbank_set_brightness(&bl->cb, 0xFF) ;
}


/*
extern int disp_bls_set_backlight(unsigned int level);
#define	LEVEL_BIT_CNT 10
static void lm3533_bl_set(struct led_classdev *cdev,
						enum led_brightness value)
{
	int level ;

	level = (((1 << LEVEL_BIT_CNT) - 1) * value + 127) / 255 ;
	printk("lm3533 bl set:%d, level:%d\n", value, level) ;
	disp_bls_set_backlight(level) ;		
}*/

static void lm3533_bl_set(struct led_classdev *cdev,
						enum led_brightness value)
{
	struct lm3533_bl *bl = to_lm3533_bl(cdev);

	lm3533_ctrlbank_set_brightness(&bl->cb, value);
}


static enum led_brightness lm3533_bl_get(struct led_classdev *cdev)
{
	struct lm3533_bl *bl = to_lm3533_bl(cdev);
	u8 val;
	int ret;

	ret = lm3533_ctrlbank_get_brightness(&bl->cb, &val);
	if (ret)
		return ret;

	return val;
}

static int lm3533_bl_probe(struct platform_device *pdev)
{
	struct lm3533 *lm3533;
	struct lm3533_bl_platform_data *pdata;
	struct lm3533_bl *bl;
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

	if (pdev->id < 0 || pdev->id >= LM3533_HVCTRLBANK_COUNT) {
		dev_err(&pdev->dev, "illegal backlight id %d\n", pdev->id);
		return -EINVAL;
	}

	bl = kzalloc(sizeof(*bl), GFP_KERNEL);
	if (!bl) {
		dev_err(&pdev->dev,
				"failed to allocate memory for backlight\n");
		return -ENOMEM;
	}

	bl->lm3533 = lm3533;
	bl->cdev.name = pdata->name;
	bl->cdev.brightness_set = lm3533_bl_set;
	bl->cdev.brightness_get = lm3533_bl_get;
	bl->cdev.brightness = pdata->default_brightness;
	bl->id = pdev->id;

	bl->cb.lm3533 = lm3533;
	bl->cb.id = lm3533_bl_get_ctrlbank_id(bl);
	bl->cb.dev = lm3533->dev; 
	
	g_bl = bl ;
	platform_set_drvdata(pdev, bl);

	ret = led_classdev_register(pdev->dev.parent, &bl->cdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register LED %d\n", pdev->id);
		goto err_free;
	}

	bl->cb.dev = bl->cdev.dev;

	//lm3533_bl_register_early_suspend(bl);

	ret = sysfs_create_group(&bl->cdev.dev->kobj, &lm3533_bl_attribute_group);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to create sysfs attributes\n");
		goto err_unregister;
	}

	ret = lm3533_bl_setup(bl, pdata);
	if (ret)
		goto err_sysfs_remove;

	ret = lm3533_ctrlbank_enable(&bl->cb);
	if (ret)
		goto err_sysfs_remove;

	return 0;

err_sysfs_remove:
	sysfs_remove_group(&bl->cdev.dev->kobj, &lm3533_bl_attribute_group);
err_unregister:
	lm3533_bl_unregister_early_suspend(bl);
	led_classdev_unregister(&bl->cdev);
err_free:
	kfree(bl);
	g_bl=NULL;
	printk("lm3533: bl proble fail\n");
	
	return ret;
}


static int lm3533_bl_remove(struct platform_device *pdev)
{
	struct lm3533_bl *bl = platform_get_drvdata(pdev);

	lm3533_ctrlbank_disable(&bl->cb);
	sysfs_remove_group(&bl->cdev.dev->kobj, &lm3533_bl_attribute_group);
	lm3533_bl_unregister_early_suspend(bl);
	led_classdev_unregister(&bl->cdev);
	kfree(bl);
	g_bl=NULL;
	printk("lm3533: bl remove\n");
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3533_bl_early_suspend(struct early_suspend *handler)
{
	struct lm3533_bl *bl;

	bl = container_of(handler, struct lm3533_bl, early_suspend);

	lm3533_ctrlbank_disable(&bl->cb);
}

static void lm3533_bl_early_resume(struct early_suspend *handler)
{
	struct lm3533_bl *bl;    
	bl = container_of(handler, struct lm3533_bl, early_suspend);
    
	lm3533_ctrlbank_enable(&bl->cb);
}

static void lm3533_bl_register_early_suspend(struct lm3533_bl *bl)
{
	bl->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	bl->early_suspend.suspend = lm3533_bl_early_suspend;
	bl->early_suspend.resume = lm3533_bl_early_resume;

	register_early_suspend(&bl->early_suspend);
}

static void lm3533_bl_unregister_early_suspend(struct lm3533_bl *bl)
{
	unregister_early_suspend(&bl->early_suspend);
}
#endif

#if 0
static int lm3533_bl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct lm3533_bl *bl = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	return lm3533_ctrlbank_disable(&bl->cb);
}

static int lm3533_bl_resume(struct platform_device *pdev)
{
	struct lm3533_bl *bl = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	return lm3533_ctrlbank_enable(&bl->cb);
}
#else
#define lm3533_bl_suspend	NULL
#define lm3533_bl_resume	NULL
#endif

static void lm3533_bl_shutdown(struct platform_device *pdev)
{
	struct lm3533_bl *bl = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	lm3533_ctrlbank_disable(&bl->cb);
}

static struct platform_driver lm3533_bl_driver = {
	.driver = {
		.name	= "lm3533-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= lm3533_bl_probe,
	.remove		= lm3533_bl_remove,
	.shutdown	= lm3533_bl_shutdown,
	.suspend	= lm3533_bl_suspend,
	.resume		= lm3533_bl_resume,
};

static int __init lm3533_bl_init(void)
{
	return platform_driver_register(&lm3533_bl_driver);
}
module_init(lm3533_bl_init);

static void __exit lm3533_bl_exit(void)
{
	platform_driver_unregister(&lm3533_bl_driver);
}
module_exit(lm3533_bl_exit);

MODULE_AUTHOR("Johan Hovold <jhovold@gmail.com>");
MODULE_DESCRIPTION("LM3533 Backlight driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lm3533-backlight");
