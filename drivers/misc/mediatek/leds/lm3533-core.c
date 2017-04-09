/*
 * lm3533-core.c -- LM3533 Core
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
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <mach/mt_gpio.h>
#include "lm3533.h"


#define LM3533_BOOST_OVP_MAX		0x03
#define LM3533_BOOST_OVP_MASK		0x06
#define LM3533_BOOST_OVP_SHIFT		1

#define LM3533_BOOST_FREQ_MAX		0x01
#define LM3533_BOOST_FREQ_MASK		0x01
#define LM3533_BOOST_FREQ_SHIFT		0

#define LM3533_BL_ID_MASK		1
#define LM3533_LED_ID_MASK		3
#define LM3533_BL_ID_MAX		1
#define LM3533_LED_ID_MAX		3

#define LM3533_HVLED_ID_MAX		2
#define LM3533_LVLED_ID_MAX		5

#define LM3533_REG_OUTPUT_CONF1		0x10
#define LM3533_REG_OUTPUT_CONF2		0x11
#define LM3533_REG_BOOST_PWM		0x2c

static struct mfd_cell lm3533_als_devs[] = {
	{
		.name	= "lm3533-als",
		.id	= -1,
	},
};

static struct mfd_cell lm3533_bl_devs[] = {
	{
		.name	= "lm3533-backlight",
		.id	= 0,
	},
	{
		.name	= "lm3533-backlight",
		.id	= 1,
	},
};

static struct mfd_cell lm3533_led_devs[] = {
	{
		.name	= "lm3533-leds",
		.id	= 0,
	},
	{
		.name	= "lm3533-leds",
		.id	= 1,
	},
	{
		.name	= "lm3533-leds",
		.id	= 2,
	},
	{
		.name	= "lm3533-leds",
		.id	= 3,
	},
	{
		.name	= "lm3533-leds",
		.id	= 4,
	},
};

static int __lm3533_read(struct lm3533 *lm3533, u8 reg, u8 *val)
{
	int ret;

	ret = lm3533->read(lm3533, reg, val);
	if (ret < 0) {
		dev_err(lm3533->dev, "failed to read register %02x: %d\n",
								reg, ret);
		return ret;
	}

	dev_dbg(lm3533->dev, "read [%02x]: %02x\n", reg, *val);

	return 0;
}

static int __lm3533_write(struct lm3533 *lm3533, u8 reg, u8 val)
{
	int ret;

	dev_dbg(lm3533->dev, "write [%02x]: %02x\n", reg, val);

	ret = lm3533->write(lm3533, reg, val);
	if (ret < 0) {
		dev_err(lm3533->dev, "failed to write register %02x: %d\n",
								reg, ret);
	}

	return ret;
}

int lm3533_read(struct lm3533 *lm3533, u8 reg, u8 *val)
{
	int ret;

	mutex_lock(&lm3533->io_mutex);
	ret = __lm3533_read(lm3533, reg, val);
	mutex_unlock(&lm3533->io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(lm3533_read);

int lm3533_write(struct lm3533 *lm3533, u8 reg, u8 val)
{
	int ret;

	mutex_lock(&lm3533->io_mutex);
	ret = __lm3533_write(lm3533, reg, val);
	mutex_unlock(&lm3533->io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(lm3533_write);

int lm3533_update(struct lm3533 *lm3533, u8 reg, u8 val, u8 mask)
{
	u8 old_val;
	u8 new_val;
	int ret;

	mutex_lock(&lm3533->io_mutex);
	ret = __lm3533_read(lm3533, reg, &old_val);
	if (ret)
		goto out;
	new_val = (old_val & ~mask) | (val & mask);
	if (new_val != old_val)
		ret = __lm3533_write(lm3533, reg, new_val);
out:
	mutex_unlock(&lm3533->io_mutex);

	return ret;
}
EXPORT_SYMBOL_GPL(lm3533_update);

/*
 * HVLED output config -- output hvled controlled by backlight bl
 */
int lm3533_set_hvled_config(struct lm3533 *lm3533, u8 hvled, u8 bl)
{
	u8 val;
	u8 mask;
	int shift;
	int ret;

	if (hvled == 0 || hvled > LM3533_HVLED_ID_MAX)
		return -EINVAL;

	if (bl > LM3533_BL_ID_MAX)
		return -EINVAL;

	shift = hvled - 1;
	mask = LM3533_BL_ID_MASK << shift;
	val = bl << shift;

	ret = lm3533_update(lm3533, LM3533_REG_OUTPUT_CONF1, val, mask);
	if (ret)
		dev_err(lm3533->dev, "failed to set hvled config\n");

	return ret;
}

/*
 * LVLED output config -- output lvled controlled by LED led
 */
static int lm3533_set_lvled_config(struct lm3533 *lm3533, u8 lvled, u8 led)
{
	u8 reg;
	u8 val;
	u8 mask;
	int shift;
	int ret;

	if (lvled == 0 || lvled > LM3533_LVLED_ID_MAX)
		return -EINVAL;

	if (led > LM3533_LED_ID_MAX)
		return -EINVAL;

	if (lvled < 4) {
		reg = LM3533_REG_OUTPUT_CONF1;
		shift = 2 * lvled;
	} else {
		reg = LM3533_REG_OUTPUT_CONF2;
		shift = 2 * (lvled - 4);
	}

	mask = LM3533_LED_ID_MASK << shift;
	val = led << shift;

	ret = lm3533_update(lm3533, reg, val, mask);
	if (ret)
		dev_err(lm3533->dev, "failed to set lvled config\n");

	return ret;
}

static void lm3533_gpio_value(int gpio, int value)
{
	mt_set_gpio_mode(gpio, GPIO_MODE_00);
	mt_set_gpio_dir(gpio, GPIO_DIR_OUT);
	mt_set_gpio_out(gpio, value);
}

void lm3533_enable(struct lm3533 *lm3533)
{
	if (lm3533->gpio_hwen >= 0)
		lm3533_gpio_value(lm3533->gpio_hwen, 1);
}

void lm3533_disable(struct lm3533 *lm3533)
{
	if (lm3533->gpio_hwen >= 0)
		lm3533_gpio_value(lm3533->gpio_hwen, 0);
}

#ifdef CONFIG_DEBUG_FS

const struct {
	u8 min;
	u8 max;
} lm3533_regs[] = {
	{ 0x10, 0x2c },
	{ 0x30, 0x38 },
	{ 0x40, 0x45 },
	{ 0x50, 0x57 },
	{ 0x60, 0x6e },
	{ 0x70, 0x75 },
	{ 0x80, 0x85 },
	{ 0x90, 0x95 },
	{ 0xa0, 0xa5 },
	{ 0xb0, 0xb2 },
};

static int lm3533_regs_show(struct seq_file *s, void * __unused)
{
	struct lm3533 *lm3533 = s->private;
	u8 reg;
	u8 val;
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(lm3533_regs); ++i) {
		for (reg = lm3533_regs[i].min;
					reg <= lm3533_regs[i].max; ++reg) {
			ret = lm3533_read(lm3533, reg, &val);
			if (ret)
				return ret;

			seq_printf(s, "[%02x]: %02x\n", reg, val);
		}
	}

	return 0;
}

static int lm3533_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lm3533_regs_show, inode->i_private);
}

static int lm3533_set_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static ssize_t lm3533_set_reg_write(struct file *file,
					const char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct lm3533 *lm3533 = file->private_data;
	char buf[32];
	int len;
	unsigned reg;
	unsigned val;
	int ret;

	len = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;
	buf[len] = 0;

	if (sscanf(buf, "%x %x", &reg, &val) != 2)
		return -EINVAL;

	if (reg > 0xff || val > 0xff)
		return -EINVAL;

	ret = lm3533_write(lm3533, (u8)reg, (u8)val);
	if (ret)
		return ret;

	return len;
}

static const struct file_operations lm3533_regs_fops = {
	.open		= lm3533_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations lm3533_set_reg_fops = {
	.open		= lm3533_set_reg_open,
	.write		= lm3533_set_reg_write,
};

static void lm3533_debugfs_init(struct lm3533 *lm3533)
{
	struct dentry *d;

	lm3533->debugfs_root = debugfs_create_dir("lm3533", NULL);
	if (!lm3533->debugfs_root) {
		dev_err(lm3533->dev, "failed to create debugfs root\n");
		return;
	}

	d = debugfs_create_file("regs", 0444, lm3533->debugfs_root, lm3533,
							&lm3533_regs_fops);
	if (!d)
		dev_err(lm3533->dev, "failed to create debugfs regs file\n");

	d = debugfs_create_file("set_reg", 0644, lm3533->debugfs_root, lm3533,
							&lm3533_set_reg_fops);
	if (!d) {
		dev_err(lm3533->dev,
				"failed to create debugfs set_reg file\n");
	}
}

static void lm3533_debugfs_cleanup(struct lm3533 *lm3533)
{
	debugfs_remove_recursive(lm3533->debugfs_root);
}

#else	/* CONFIG_DEBUG_FS */
static void lm3533_debugfs_init(struct lm3533 *lm3533) { }
static void lm3533_debugfs_cleanup(struct lm3533 *lm3533) { }
#endif	/* CONFIG_DEBUG_FS */

enum lm3533_attribute_type {
	LM3533_ATTR_TYPE_BACKLIGHT,
	LM3533_ATTR_TYPE_LED,
};

struct lm3533_device_attribute {
	struct device_attribute dev_attr;
	enum lm3533_attribute_type type;
	union {
		struct {
			u8 id;
		} output;
		struct {
			u8 reg;
			u8 shift;
			u8 mask;
			u8 max;
		} generic;
	} u;
};

#define to_lm3533_dev_attr(_attr) \
	container_of(_attr, struct lm3533_device_attribute, dev_attr)

static ssize_t show_lm3533_reg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lm3533 *lm3533 = dev_get_drvdata(dev);
	struct lm3533_device_attribute *lattr = to_lm3533_dev_attr(attr);
	u8 val;
	int ret;

	ret = lm3533_read(lm3533, lattr->u.generic.reg, &val);
	if (ret)
		return ret;

	val = (val & lattr->u.generic.mask) >> lattr->u.generic.shift;

	return sprintf(buf, "%u\n", val);
}

static ssize_t store_lm3533_reg(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t len)
{
	struct lm3533 *lm3533 = dev_get_drvdata(dev);
	struct lm3533_device_attribute *lattr = to_lm3533_dev_attr(attr);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 0, &val) || val > lattr->u.generic.max)
		return -EINVAL;

	val = val << lattr->u.generic.shift;
	ret = lm3533_update(lm3533, lattr->u.generic.reg, (u8)val,
							lattr->u.generic.mask);
	if (ret)
		return ret;

	return len;
}

#define GENERIC_ATTR(_reg, _max, _mask, _shift) \
	{ .reg		= _reg, \
	  .max		= _max, \
	  .mask		= _mask, \
	  .shift	= _shift }

#define LM3533_GENERIC_ATTR(_name, _mode, _show, _store, _type,	\
						_reg, _max, _mask, _shift) \
	struct lm3533_device_attribute lm3533_dev_attr_##_name = { \
		.dev_attr	= __ATTR(_name, _mode, _show, _store), \
		.type		= _type, \
		.u.generic	= GENERIC_ATTR(_reg, _max, _mask, _shift) }

#define lm3533_attr_gen_rw(_name, _type, _reg, _max, _mask, _shift) \
	static LM3533_GENERIC_ATTR(_name, S_IRUGO | S_IWUSR, \
					show_lm3533_reg, store_lm3533_reg, \
					_type, _reg, _max, _mask, _shift)

#define boost_attr(_name, _NAME) \
	lm3533_attr_gen_rw(_name, LM3533_ATTR_TYPE_BACKLIGHT, \
				LM3533_REG_BOOST_PWM, LM3533_##_NAME##_MAX, \
				LM3533_##_NAME##_MASK, LM3533_##_NAME##_SHIFT)
/*
 * Boost Over Voltage Protection Select
 *
 *   0 -- 16 V (default)
 *   1 -- 24 V
 *   2 -- 32 V
 *   3 -- 40 V
 */
boost_attr(boost_ovp, BOOST_OVP);

int lm3533_set_boost_ovp(struct lm3533 *lm3533, u8 boost_ovp)
{
	u8 val;
	int ret;

	if (LM3533_BOOST_OVP_MAX < boost_ovp)
		return -EINVAL;

	val = boost_ovp << LM3533_BOOST_OVP_SHIFT ;

	ret = lm3533_update(lm3533, LM3533_REG_BOOST_PWM, val, 
				LM3533_BOOST_OVP_MASK) ;
	if (ret)
		dev_err(lm3533->dev, "failed to set boost ovp config\n");

	return ret;
}

/*
 * Boost Frequency Select
 *
 *   0 -- 500 kHz (default)
 *   1 -- 1 MHz
 */
boost_attr(boost_freq, BOOST_FREQ);

static ssize_t show_output(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lm3533 *lm3533 = dev_get_drvdata(dev);
	struct lm3533_device_attribute *lattr = to_lm3533_dev_attr(attr);
	int id = lattr->u.output.id;
	u8 reg;
	u8 val;
	u8 mask;
	int shift;
	int ret;

	if (lattr->type == LM3533_ATTR_TYPE_BACKLIGHT) {
		reg = LM3533_REG_OUTPUT_CONF1;
		shift = id - 1;
		mask = LM3533_BL_ID_MASK << shift;
	} else {
		if (id < 4) {
			reg = LM3533_REG_OUTPUT_CONF1;
			shift = 2 * id;
		} else {
			reg = LM3533_REG_OUTPUT_CONF2;
			shift = 2 * (id - 4);
		}
		mask = LM3533_LED_ID_MASK << shift;
	}

	ret = lm3533_read(lm3533, reg, &val);
	if (ret)
		return ret;

	val = (val & mask) >> shift;

	return sprintf(buf, "%u\n", val);
}

static ssize_t store_output(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct lm3533 *lm3533 = dev_get_drvdata(dev);
	struct lm3533_device_attribute *lattr = to_lm3533_dev_attr(attr);
	int id = lattr->u.output.id;
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 0, &val) || val > LM3533_ATTR_VAL_MAX)
		return -EINVAL;

	if (lattr->type == LM3533_ATTR_TYPE_BACKLIGHT)
		ret = lm3533_set_hvled_config(lm3533, id, (u8)val);
	else
		ret = lm3533_set_lvled_config(lm3533, id, (u8)val);

	if (ret)
		return ret;

	return len;
}

#define LM3533_OUTPUT_ATTR(_name, _mode, _show, _store, _type, _id) \
	struct lm3533_device_attribute lm3533_dev_attr_##_name = \
		{ .dev_attr	= __ATTR(_name, _mode, _show, _store), \
		  .type		= _type, \
		  .u.output	= { .id = _id }, }

#define lm3533_output_attr_rw(_name, _type, _id) \
	static LM3533_OUTPUT_ATTR(output_##_name, S_IRUGO | S_IWUSR, \
					show_output, store_output, _type, _id)

#define output_hvled(_nr) \
	lm3533_output_attr_rw(hvled##_nr, LM3533_ATTR_TYPE_BACKLIGHT, _nr)
#define output_lvled(_nr) \
	lm3533_output_attr_rw(lvled##_nr, LM3533_ATTR_TYPE_LED, _nr)
/*
 * Output config:
 *
 * output_hvled<nr> -- 0-1
 * output_lvled<nr> -- 0-4
 */
output_hvled(1);
output_hvled(2);
output_lvled(1);
output_lvled(2);
output_lvled(3);
output_lvled(4);
output_lvled(5);

static struct attribute *lm3533_attributes[] = {
	&lm3533_dev_attr_boost_freq.dev_attr.attr,
	&lm3533_dev_attr_boost_ovp.dev_attr.attr,
	&lm3533_dev_attr_output_hvled1.dev_attr.attr,
	&lm3533_dev_attr_output_hvled2.dev_attr.attr,
	&lm3533_dev_attr_output_lvled1.dev_attr.attr,
	&lm3533_dev_attr_output_lvled2.dev_attr.attr,
	&lm3533_dev_attr_output_lvled3.dev_attr.attr,
	&lm3533_dev_attr_output_lvled4.dev_attr.attr,
	&lm3533_dev_attr_output_lvled5.dev_attr.attr,
	NULL,
};

#define to_dev_attr(_attr) \
	container_of(_attr, struct device_attribute, attr)

static mode_t lm3533_attr_is_visible(struct kobject *kobj,
					     struct attribute *attr, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct lm3533 *lm3533 = dev_get_drvdata(dev);
	struct device_attribute *dattr = to_dev_attr(attr);
	struct lm3533_device_attribute *lattr = to_lm3533_dev_attr(dattr);
	enum lm3533_attribute_type type = lattr->type;
	mode_t mode = attr->mode;

	if (!lm3533->have_backlights && type == LM3533_ATTR_TYPE_BACKLIGHT)
		mode = 0;
	else if (!lm3533->have_leds && type == LM3533_ATTR_TYPE_LED)
		mode = 0;

	return mode;
};

static struct attribute_group lm3533_attribute_group = {
	.is_visible	= lm3533_attr_is_visible,
	.attrs		= lm3533_attributes
};

static int lm3533_device_als_init(struct lm3533 *lm3533)
{
	struct lm3533_platform_data *pdata = lm3533->dev->platform_data;
	int ret;

	if (!pdata->als)
		return 0;

	lm3533_als_devs[0].platform_data = pdata->als;
	lm3533_als_devs[0].pdata_size = sizeof(*pdata->als);

	ret = mfd_add_devices(lm3533->dev, 0, lm3533_als_devs, 1, NULL, 0, NULL);
	if (ret) {
		dev_err(lm3533->dev, "failed to add ALS device\n");
		return ret;
	}

	lm3533->have_als = 1;

	return 0;
}

static int lm3533_device_bl_init(struct lm3533 *lm3533)
{
	struct lm3533_platform_data *pdata = lm3533->dev->platform_data;
	int i;
	int ret;

	if (!pdata->backlights || pdata->num_backlights == 0)
		return 0;

	if (pdata->num_backlights > ARRAY_SIZE(lm3533_bl_devs))
		pdata->num_backlights = ARRAY_SIZE(lm3533_bl_devs);

	for (i = 0; i < pdata->num_backlights; ++i) {
		lm3533_bl_devs[i].platform_data = &pdata->backlights[i];
		lm3533_bl_devs[i].pdata_size = sizeof(pdata->backlights[i]);
	}

	ret = mfd_add_devices(lm3533->dev, 0, lm3533_bl_devs,
					pdata->num_backlights, NULL, 0, NULL);
	if (ret) {
		dev_err(lm3533->dev, "failed to add backlight devices\n");
		return ret;
	}

	lm3533->have_backlights = 1;

	return 0;
}

static int lm3533_device_led_init(struct lm3533 *lm3533)
{
	struct lm3533_platform_data *pdata = lm3533->dev->platform_data;
	int i;
	int ret;

	if (!pdata->leds || pdata->num_leds == 0)
		return 0;

	if (pdata->num_leds > ARRAY_SIZE(lm3533_led_devs))
		pdata->num_leds = ARRAY_SIZE(lm3533_led_devs);

	for (i = 0; i < pdata->num_leds; ++i) {
		lm3533_led_devs[i].platform_data = &pdata->leds[i];
		lm3533_led_devs[i].pdata_size = sizeof(pdata->leds[i]);
	}

	ret = mfd_add_devices(lm3533->dev, 0, lm3533_led_devs,
						pdata->num_leds, NULL, 0, NULL);
	if (ret) {
		dev_err(lm3533->dev, "failed to add LED devices\n");
		return ret;
	}

	lm3533->have_leds = 1;

	return 0;
}


int  lm3533_device_init(struct lm3533 *lm3533)
{
	struct lm3533_platform_data *pdata = lm3533->dev->platform_data;
	int ret;

	dev_dbg(lm3533->dev, "%s\n", __func__);

	if (!pdata) {
		dev_err(lm3533->dev, "no platform data\n");
		return -EINVAL;
	}

	mutex_init(&lm3533->io_mutex);
	lm3533->gpio_hwen = pdata->gpio_hwen;

	dev_set_drvdata(lm3533->dev, lm3533);

	/*
	if (lm3533->gpio_hwen >= 0) {
		ret = gpio_request_one(lm3533->gpio_hwen, GPIOF_OUT_INIT_LOW,
								"lm3533-hwen");
		if (ret < 0) {
			dev_err(lm3533->dev,
				"failed to request HWEN GPIO %d\n",
				lm3533->gpio_hwen);
			return ret;
		}
	}*/

	lm3533_enable(lm3533);

	/* HVLED2 (index 2) is controlled by Control Bank A (index 0)*/
	ret = lm3533_set_hvled_config(lm3533, 2, 0) ;
	if (ret)
		goto err_disable ;

	/*
 	*   Default boost_ovp is 16V, Promote to 40 V
	*
 	*   0 -- 16 V (default)
 	*   1 -- 24 V
 	*   2 -- 32 V
 	*   3 -- 40 V
 	*/
	ret = lm3533_set_boost_ovp(lm3533, 3) ;
	if (ret)
		goto err_disable ;

	lm3533_device_als_init(lm3533);
	lm3533_device_bl_init(lm3533);
	lm3533_device_led_init(lm3533);

	ret = sysfs_create_group(&lm3533->dev->kobj, &lm3533_attribute_group);
	if (ret < 0) {
		dev_err(lm3533->dev, "failed to create sysfs attributes\n");
		goto err_unregister;
	}

	lm3533_debugfs_init(lm3533);

	return 0;

err_unregister:
	mfd_remove_devices(lm3533->dev);
err_disable:
	//lm3533_disable(lm3533);

	/*
	if (lm3533->gpio_hwen >= 0)
		gpio_free(lm3533->gpio_hwen);
	*/

	return ret;
}

void lm3533_device_exit(struct lm3533 *lm3533)
{
	dev_dbg(lm3533->dev, "%s\n", __func__);

	lm3533_debugfs_cleanup(lm3533);

	sysfs_remove_group(&lm3533->dev->kobj, &lm3533_attribute_group);

	mfd_remove_devices(lm3533->dev);
	lm3533_disable(lm3533);

	/*
	if (lm3533->gpio_hwen >= 0)
		gpio_free(lm3533->gpio_hwen);
	*/
}

MODULE_AUTHOR("Johan Hovold <jhovold@gmail.com>");
MODULE_DESCRIPTION("LM3533 Core");
MODULE_LICENSE("GPL");
