/*
 * lm3533-i2c.c -- LM3533 I2C interface
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
#include <linux/slab.h>
#include <linux/i2c.h>

#include <mach/mt_gpio.h>
#include "lm3533.h"

static unsigned char first_success=0;

static int lm3533_i2c_read(struct lm3533 *lm3533, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(lm3533->i2c, reg);
	if (ret < 0)
		return ret;

	*val = (u8)ret;

	return 0;
}

static int lm3533_i2c_write(struct lm3533 *lm3533, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(lm3533->i2c, reg, val);
}

static int  lm3533_i2c_probe(struct i2c_client *i2c,
					const struct i2c_device_id *id)
{
	struct lm3533 *lm3533;
	int ret;

	dev_dbg(&i2c->dev, "%s\n", __func__);

	if(first_success==1)
		return -1;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	lm3533 = kzalloc(sizeof(*lm3533), GFP_KERNEL);
	if (!lm3533)
		return -ENOMEM;

	i2c_set_clientdata(i2c, lm3533);

	lm3533->dev = &i2c->dev;
	lm3533->i2c = i2c;
	lm3533->irq = i2c->irq;
	lm3533->read = lm3533_i2c_read;
	lm3533->write = lm3533_i2c_write;

	ret = lm3533_device_init(lm3533);
	if (ret) {
		kfree(lm3533);
		return ret;
	}

	first_success=1;

	return 0;
}

static int lm3533_i2c_remove(struct i2c_client *i2c)
{
	struct lm3533 *lm3533 = i2c_get_clientdata(i2c);

	dev_dbg(&i2c->dev, "%s\n", __func__);

	lm3533_device_exit(lm3533);

	kfree(lm3533);

	return 0;
}

static const struct i2c_device_id lm3533_i2c_ids[] = {
	{ "lm3533", 0 },
	{ "lm3533_a", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, lm3533_i2c_ids);

static struct i2c_driver lm3533_i2c_driver = {
	.driver = {
		   .name = "lm3533_all",
		   .owner = THIS_MODULE,
	},
	.id_table	= lm3533_i2c_ids,
	.probe		= lm3533_i2c_probe,
	.remove		= lm3533_i2c_remove,
};

static struct lm3533_bl_platform_data lm3533_backlights[] = {
	{
		.name			= "lm3533-lcd",
		.default_brightness	= 255,
		.max_current		= 0x16,  /*22.6mA */
		.pwm			= 0x01, /* enable pwm */
	},
};

static struct lm3533_led_platform_data lm3533_leds[] = {
	{
		.name			= "red",
		.default_trigger	= "none",
		.max_current		= 0x00,
	},
	{
		.name			= "green",
		.default_trigger	= "none",
		.max_current		= 0x00,
	},
	{
		.name			= "blue", 
		.default_trigger	= "none",
		.max_current		= 0x00,
	},
	{
		.name			= "button-backlight",
		.default_trigger	= "none",
		.max_current		= 0xa,
	},
};

static struct lm3533_platform_data ll95_lm3533_data = {
	.gpio_hwen	= GPIO119,	/* KPROW0 */
	.als		= NULL,
	.backlights	= lm3533_backlights,
	.num_backlights	= ARRAY_SIZE(lm3533_backlights),
	.leds		= lm3533_leds,
	.num_leds	= ARRAY_SIZE(lm3533_leds),
};

static struct i2c_board_info __initdata ll95_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("lm3533", (0x36)),
		.platform_data  = &ll95_lm3533_data,
	},
#if 1
	{
		I2C_BOARD_INFO("lm3533_a", (0x38)),
		.platform_data  = &ll95_lm3533_data,
	},
#endif
};

static int __init lm3533_i2c_init(void)
{
	i2c_register_board_info(2, ll95_i2c_boardinfo, 
				ARRAY_SIZE(ll95_i2c_boardinfo)); 
	return i2c_add_driver(&lm3533_i2c_driver);
}
module_init(lm3533_i2c_init);

static void __exit lm3533_i2c_exit(void)
{
	i2c_del_driver(&lm3533_i2c_driver);
}
module_exit(lm3533_i2c_exit);

MODULE_AUTHOR("Johan Hovold <jhovold@gmail.com>");
MODULE_DESCRIPTION("LM3533 I2C interface");
MODULE_LICENSE("GPL");
