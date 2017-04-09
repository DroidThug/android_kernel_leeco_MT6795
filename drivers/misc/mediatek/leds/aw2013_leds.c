/*
 * leds-aw2013.c -- aw2013 LED driver
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
//#include <linux/mfd/core.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/file.h>
#include <linux/i2c.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define aw2013_ATTR_VAL_MAX	255

#define aw2013_attr_ro(_name) \
	static DEVICE_ATTR(_name, S_IRUGO, show_##_name, NULL)
#define aw2013_attr_rw(_name) \
	static DEVICE_ATTR(_name, 0644 , show_##_name, \
			store_##_name)

#if CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define RED      0
#define GREEN    1
#define BLUE     2
#define Imax          0x02
#define Rise_time   0x02
#define Hold_time   0x02
#define Hold_time_R   0x03
#define Fall_time     0x02
#define Off_time      0x02
#define Off_time_R    0x04
#define Delay_time   0x02
#define Period_Num  0x00
/* Delay limits in ms */
#define AW2013_LED_DELAY_ON_MAX		9845
#define AW2013_LED_DELAY_OFF_MAX	77140

struct device;
struct dentry;
struct i2c_client;

static int breath_led_flag = 0;
static int rise_time = 2;
static int fall_time = 2;
static int sleep_flag = 0;

struct aw2013_led_platform_data {
	char *name;
	const char *default_trigger;
	u8 max_current;			/* 0 - 31 */
	u8 pwm;				/* 0 - 0x3f */
};

struct aw2013_led {
	struct led_classdev cdev;
	int id;

	struct mutex mutex;
	unsigned long flags;

	struct work_struct work;
	u8 new_brightness;

#if  CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

};

struct led_state {
	int led_on;
	int led_brightness;
	int led_blink_on;
	unsigned char led_risetime;
	unsigned char led_holdtime;
	unsigned char led_falltime;
	unsigned char led_offtime;
};


struct aw2013 {
	struct device *dev;
	struct i2c_client *i2c;

	struct mutex io_mutex;

	int irq;
	int num_leds;
	
#if 1
	struct aw2013_led_platform_data *aw2013_data;
	struct aw2013_led  led_aw2013[3];
	struct led_state g_leds_state[3];
#endif

};



static struct i2c_client *g_i2c_aw2013=NULL;
struct aw2013 *g_aw2013 = NULL;

static struct aw2013_led_platform_data aw2013_leds[];

#define to_aw2013_led(_cdev) \
	container_of(_cdev, struct aw2013_led, cdev)


static int aw2013_i2c_read_reg(u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(g_i2c_aw2013, reg);
	if (ret < 0)
		return ret;

	*val = (u8)ret;

	return 0;
}

static int AW2013_i2c_write_reg(unsigned char reg,unsigned char data)
{
	return i2c_smbus_write_byte_data(g_i2c_aw2013, reg, data);
}

void AW2013_delay_1ms(unsigned int time)
{
	mdelay(time);
}

//*******************************AW2013 led on/off***********************************///
static void led_on_aw2013(int led0,int led0_bright, int led1,int led1_bright, int led2, int led2_bright)  //led on=0x01   ledoff=0x00
{  
	int led0_value=0;
	int led1_value=0;
	int led2_value=0;
	if(led0_bright>0xff)
		led0_value=0xff;
	else if(led0_bright<=0x0)
		led0_value=0;
	else
		led0_value=led0_bright;

	if(led1_bright>0xff)
		led1_value=0xff;
	else if(led1_bright<=0x0)
		led1_value=0;
	else
		led1_value=led1_bright;

	if(led2_bright>0xff)
		led2_value=0xff;
	else if(led2_bright<=0x0)
		led2_value=0;
	else
		led2_value=led2_bright;
#if 0
	if(!(led2|led1|led0))  {
		AW2013_i2c_write_reg(0x30, 0);				//all led off	
		AW2013_i2c_write_reg(0x01,0);
		return;
	}	
#endif
	if(breath_led_flag == 1)
	{
		AW2013_i2c_write_reg(0x31, Imax  | 0x70);	//config mode, IMAX = 5mA	//breath mode
		AW2013_i2c_write_reg(0x32, Imax  | 0x70);	//config mode, IMAX = 5mA	
		AW2013_i2c_write_reg(0x33, Imax  | 0x70);	//config mode, IMAX = 5mA	
	}
	else
	{
		AW2013_i2c_write_reg(0x01, 0x01);		// enable LED	
		AW2013_i2c_write_reg(0x31, Imax);	//config mode, IMAX = 5mA	//pwm mode
		AW2013_i2c_write_reg(0x32, Imax);	//config mode, IMAX = 5mA	
		AW2013_i2c_write_reg(0x33, Imax);	//config mode, IMAX = 5mA	
	}
	AW2013_i2c_write_reg(0x34, led0_value);	// LED0 level, //pwm max brightness
	AW2013_i2c_write_reg(0x35, led1_value);	// LED1 level,
	AW2013_i2c_write_reg(0x36, led2_value);	// LED2 level,

	AW2013_i2c_write_reg(0x30, led2<<2|led1<<1|led0);	       //led on=0x01 ledoff=0x00	
	AW2013_delay_1ms(1);
	printk("alvin_wu:led_on_aw2013\n");
}


static void aw2013_breath_all(int led0,int led1,int led2)  //led on=0x01   ledoff=0x00
{  
	AW2013_i2c_write_reg(0x01, 0x01);		// enable LED	
	if(led0)
	{
		AW2013_i2c_write_reg(0x37,g_aw2013->g_leds_state[RED].led_risetime<<4 | g_aw2013->g_leds_state[RED].led_holdtime);
		AW2013_i2c_write_reg(0x38, g_aw2013->g_leds_state[RED].led_falltime<<4  | g_aw2013->g_leds_state[RED].led_offtime);	       
		AW2013_i2c_write_reg(0x39, Delay_time<<4| Period_Num); 
	}
	if(led1)
	{
		AW2013_i2c_write_reg(0x3a, g_aw2013->g_leds_state[GREEN].led_risetime<<4 | g_aw2013->g_leds_state[GREEN].led_holdtime);
		AW2013_i2c_write_reg(0x3b,g_aw2013->g_leds_state[GREEN].led_falltime<<4 | g_aw2013->g_leds_state[GREEN].led_offtime);	     
		AW2013_i2c_write_reg(0x3c, Delay_time<<4| Period_Num);
	}
	if(led2)
	{
		AW2013_i2c_write_reg(0x3d, g_aw2013->g_leds_state[BLUE].led_risetime<<4 | g_aw2013->g_leds_state[BLUE].led_holdtime);			
		AW2013_i2c_write_reg(0x3e, g_aw2013->g_leds_state[BLUE].led_falltime<<4 | g_aw2013->g_leds_state[BLUE].led_offtime);	      
		AW2013_i2c_write_reg(0x3f, Delay_time<<4| Period_Num);
	}

	AW2013_delay_1ms(1);
	printk("alvin_wu:aw2013_breath_all\n");

}

int aw2013_device_init(struct aw2013 *aw2013)
{
	printk("%s\n", __func__);
	mutex_init(&aw2013->io_mutex);
	return 0;
}


static void aw2013_led_work(struct work_struct *work)
{
	struct aw2013_led *led = container_of(work, struct aw2013_led, work);

	printk("alvin_wu:%s - id(%d)->brightness(%u)\n", __func__, led->id,led->new_brightness); 
	if(led->new_brightness == 0)
		breath_led_flag =0;
	switch(led->id)
	{
	case 0:
		if(led->new_brightness == 0)
		{
			g_aw2013->g_leds_state[RED].led_on = 0;
			g_aw2013->g_leds_state[RED].led_brightness = 0;
			g_aw2013->g_leds_state[RED].led_blink_on = 0;
		}
		else
		{
			g_aw2013->g_leds_state[RED].led_on = 1;
			g_aw2013->g_leds_state[RED].led_brightness = led->new_brightness ;
		}
		break;

	case 1:
		if(led->new_brightness == 0)
		{
			g_aw2013->g_leds_state[GREEN].led_on = 0;
			g_aw2013->g_leds_state[GREEN].led_brightness = 0;
			g_aw2013->g_leds_state[GREEN].led_blink_on = 0;
		}
		else
		{
			g_aw2013->g_leds_state[GREEN].led_on = 1;
			g_aw2013->g_leds_state[GREEN].led_brightness = led->new_brightness ;
		}		
		break;

	case 2:
		if(led->new_brightness == 0)
		{
			g_aw2013->g_leds_state[BLUE].led_on = 0;
			g_aw2013->g_leds_state[BLUE].led_brightness = 0;
			g_aw2013->g_leds_state[BLUE].led_blink_on = 0;
		}
		else
		{
			g_aw2013->g_leds_state[BLUE].led_on = 1;
			g_aw2013->g_leds_state[BLUE].led_brightness = led->new_brightness ;
		}
		break;

	default:
		printk("aw2013_led_work: led id error!\n");
		break;
	}
	led_on_aw2013(g_aw2013->g_leds_state[RED].led_on, g_aw2013->g_leds_state[RED].led_brightness, g_aw2013->g_leds_state[GREEN].led_on, g_aw2013->g_leds_state[GREEN].led_brightness, g_aw2013->g_leds_state[BLUE].led_on, g_aw2013->g_leds_state[BLUE].led_brightness);
}

static void aw2013_led_set(struct led_classdev *cdev,enum led_brightness value)
{
	struct aw2013_led *led = to_aw2013_led(cdev);
	mutex_lock(&g_aw2013->io_mutex);
	printk("alvin_wu:%s - %d\n", __func__, value);
	led->new_brightness = value;
	printk("alvin_wu:led->id = %d,led->new_brightness = %d\n",led->id, led->new_brightness);
	schedule_work(&led->work);
	mutex_unlock(&g_aw2013->io_mutex);
}

static int aw2013_get_brightness(int id)
{
	char val = 0;
	aw2013_i2c_read_reg( (0x34+id), &val);
	printk("alvin_wu:aw2013_get_brightness,val = %d\n",val);
	return val;
}

static enum led_brightness aw2013_led_get(struct led_classdev *cdev)
{
	struct aw2013_led *led = to_aw2013_led(cdev);
	u8 ret;
	ret = aw2013_get_brightness(led->id);
	printk( "alvin_wu:%s - ret = %u\n", __func__, ret);

	return ret;
}

static void breath_set_effect(int id,unsigned long delay_on,unsigned long delay_off)
{
	if(delay_on > 0 && delay_on <= 43)
		g_aw2013->g_leds_state[id].led_holdtime=0x0;	//0.13 s
	else if(delay_on > 43 && delay_on <= 86)
		g_aw2013->g_leds_state[id].led_holdtime=0x1;	//0.26 s
	else if(delay_on > 86 && delay_on <= 129)
		g_aw2013->g_leds_state[id].led_holdtime=0x2;	 //0.52 s
	else if(delay_on > 129 && delay_on <= 172)
		g_aw2013->g_leds_state[id].led_holdtime=0x3;		// 1.04 s
	else if(delay_on > 172 && delay_on <= 215)
		g_aw2013->g_leds_state[id].led_holdtime=0x4;		// 2.08 s
	else
		g_aw2013->g_leds_state[id].led_holdtime=0x5;		// 4.16s

	if(delay_off > 0 && delay_off <= 43)
		g_aw2013->g_leds_state[id].led_offtime=0x0;
	else if(delay_off > 43 && delay_off <= 86)
		g_aw2013->g_leds_state[id].led_offtime=0x1;
	else if(delay_off > 86 && delay_off <= 129)
		g_aw2013->g_leds_state[id].led_offtime=0x2;
	else if(delay_off > 129&& delay_off <= 172)
		g_aw2013->g_leds_state[id].led_offtime=0x3;
	else if(delay_off > 172 && delay_off <= 215)
		g_aw2013->g_leds_state[id].led_offtime=0x4;
	else
		g_aw2013->g_leds_state[id].led_offtime=0x5;
	printk("g_aw2013->g_leds_state[%d].led_risetime=%d,g_aw2013->g_leds_state[%d].led_falltime = %d\n",id,g_aw2013->g_leds_state[id].led_risetime,id,g_aw2013->g_leds_state[id].led_falltime);
	
}

static int aw2013_led_blink_set(struct led_classdev *cdev,unsigned long *delay_on,unsigned long *delay_off)
{
	struct aw2013_led *led = to_aw2013_led(cdev);
	mutex_lock(&g_aw2013->io_mutex);
	printk("%s - on = %lu, off = %lu\n", __func__,*delay_on, *delay_off);
	if (*delay_on > AW2013_LED_DELAY_ON_MAX ||*delay_off > AW2013_LED_DELAY_OFF_MAX)
		return -EINVAL;
#if 0
	if (*delay_on == 0 && *delay_off == 0) {
		*delay_on = 500;
		*delay_off = 500;
		if(led->id == 0)
		{
			g_aw2013->g_leds_state[led->id].led_holdtime=Hold_time_R;
			g_aw2013->g_leds_state[led->id].led_offtime=Off_time_R;
		}else{
			g_aw2013->g_leds_state[led->id].led_holdtime=Hold_time;
			g_aw2013->g_leds_state[led->id].led_offtime=Off_time;
		}
	}
	else
	{
		 breath_set_effect(led->id,*delay_on,*delay_off);
	}
#endif
	AW2013_i2c_write_reg(0x00, 0x55); //reset aw2013
	if(rise_time == 0)
		g_aw2013->g_leds_state[led->id].led_holdtime=0x0;	//0.13 s
	else if(rise_time == 1)
		g_aw2013->g_leds_state[led->id].led_holdtime=0x1;	//0.26 s
	else if(rise_time == 2)
		g_aw2013->g_leds_state[led->id].led_holdtime=0x2;	 //0.52 s
	else if(rise_time == 3)
		g_aw2013->g_leds_state[led->id].led_holdtime=0x3;		// 1.04 s
	else if(rise_time == 4)
		g_aw2013->g_leds_state[led->id].led_holdtime=0x4;		// 2.08 s
	else
		g_aw2013->g_leds_state[led->id].led_holdtime=0x5;		// 4.16s

	if(fall_time == 0)
		g_aw2013->g_leds_state[led->id].led_offtime=0x0;
	else if(fall_time == 1)
		g_aw2013->g_leds_state[led->id].led_offtime=0x1;
	else if(fall_time == 2)
		g_aw2013->g_leds_state[led->id].led_offtime=0x2;
	else if(fall_time == 3)
		g_aw2013->g_leds_state[led->id].led_offtime=0x3;
	else if(fall_time == 4)
		g_aw2013->g_leds_state[led->id].led_offtime=0x4;
	else
		g_aw2013->g_leds_state[led->id].led_offtime=0x5;
	printk("g_aw2013->g_leds_state[%d].led_risetime=%d,g_aw2013->g_leds_state[%d].led_falltime = %d\n",led->id,g_aw2013->g_leds_state[led->id].led_risetime,led->id,g_aw2013->g_leds_state[led->id].led_falltime);
	switch(led->id)
	{
	case 0:
		g_aw2013->g_leds_state[RED].led_blink_on = 1;
		g_aw2013->g_leds_state[GREEN].led_blink_on = 0;
		g_aw2013->g_leds_state[BLUE].led_blink_on = 0;
		break;

	case 1:
		g_aw2013->g_leds_state[RED].led_blink_on = 0;
		g_aw2013->g_leds_state[GREEN].led_blink_on = 1;
		g_aw2013->g_leds_state[BLUE].led_blink_on = 0;
		break;

	case 2:
		g_aw2013->g_leds_state[RED].led_blink_on = 0;
		g_aw2013->g_leds_state[GREEN].led_blink_on = 0;
		g_aw2013->g_leds_state[BLUE].led_blink_on = 1;
		break;

	default:
		printk("aw2013_led_blink_set: led id error!\n");
		break;
	}
	breath_led_flag = 1;
	printk("alvin_wu:aw2013_led_blink_set:led->id=%d\n",led->id);
	aw2013_breath_all(g_aw2013->g_leds_state[RED].led_blink_on, g_aw2013->g_leds_state[GREEN].led_blink_on, g_aw2013->g_leds_state[BLUE].led_blink_on);
	mutex_unlock(&g_aw2013->io_mutex);
	return 0;
}

static ssize_t show_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	ret = aw2013_get_brightness(led->id);
	return sprintf(buf, "id = %d,brightness = %d\n", led->id,ret);
}

static ssize_t show_risetime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	u8 value = 0;
//	aw2013_i2c_read_reg((0x37+led->id*3),&value);
//	value = value >> 4;
	value = rise_time;
	return sprintf(buf, "%d\n", value);
}

static ssize_t show_falltime(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	u8 value = 0;
//	aw2013_i2c_read_reg((0x38+led->id*3),&value);
//	value = value >> 4;
	value = fall_time;
	return sprintf(buf, "%d\n", value);
}


static ssize_t store_risetime(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	int  rise_time_store;
	if (sscanf(buf, "%d", &rise_time_store) == 1)
	{
		rise_time = rise_time_store;
	}
//		AW2013_i2c_write_reg((0x37+led->id*3),rise_time_store<<4 | g_aw2013->g_leds_state[led->id].led_holdtime);
	return len;
}

static ssize_t store_falltime(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{	
	int fall_time_store;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	if (sscanf(buf, "%d", &fall_time_store) == 1)
	{
		fall_time = fall_time_store;
	}
//		AW2013_i2c_write_reg((0x38+led->id * 3), fall_time_store<<4  | g_aw2013->g_leds_state[led->id].led_offtime);
	return len;
}


static ssize_t store_sleep_flag(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{	
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	sscanf(buf, "%d", &sleep_flag);
	if(sleep_flag <  0)
	{
		printk("custom input  error\n");
	}
	return len;
}

static ssize_t show_sleep_flag(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	return sprintf(buf, "%d\n", sleep_flag);
}


aw2013_attr_rw(falltime);
aw2013_attr_ro(id);
aw2013_attr_rw(risetime);
aw2013_attr_rw(sleep_flag);

static struct attribute *aw2013_led_attributes[] = {
	&dev_attr_falltime.attr,
	&dev_attr_id.attr,
	&dev_attr_risetime.attr,
	&dev_attr_sleep_flag.attr,
	NULL,
};

static mode_t aw2013_led_attr_is_visible(struct kobject *kobj,
		struct attribute *attr, int n)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct aw2013_led *led = to_aw2013_led(led_cdev);
	mode_t mode = attr->mode;

	return mode;
};

static struct attribute_group aw2013_led_attribute_group = {
	.is_visible	= aw2013_led_attr_is_visible,
	.attrs		= aw2013_led_attributes
};


#if CONFIG_HAS_EARLYSUSPEND
static void aw2013_led_early_suspend(struct early_suspend *handler)
{
	struct aw2013_led *led;
	led = container_of(handler, struct aw2013_led, early_suspend);
	if(sleep_flag){
		g_aw2013->g_leds_state[BLUE].led_on = 1;
		g_aw2013->g_leds_state[BLUE].led_brightness = 255;
		aw2013_breath_all(g_aw2013->g_leds_state[RED].led_blink_on, g_aw2013->g_leds_state[GREEN].led_blink_on, g_aw2013->g_leds_state[BLUE].led_blink_on);
		led_on_aw2013(g_aw2013->g_leds_state[RED].led_on, g_aw2013->g_leds_state[RED].led_brightness, g_aw2013->g_leds_state[GREEN].led_on, g_aw2013->g_leds_state[GREEN].led_brightness, g_aw2013->g_leds_state[BLUE].led_on, g_aw2013->g_leds_state[BLUE].led_brightness);
	}

	printk("alvin_wu:aw2013_led_early_suspend!\n");
}

static void aw2013_led_late_resume(struct early_suspend *handler)
{
	struct aw2013_led *led;
	led = container_of(handler, struct aw2013_led, early_suspend);
	if(sleep_flag){
		g_aw2013->g_leds_state[BLUE].led_on = 1;
		g_aw2013->g_leds_state[BLUE].led_brightness = 0;
		led_on_aw2013(g_aw2013->g_leds_state[RED].led_on, g_aw2013->g_leds_state[RED].led_brightness, g_aw2013->g_leds_state[GREEN].led_on, g_aw2013->g_leds_state[GREEN].led_brightness, g_aw2013->g_leds_state[BLUE].led_on, g_aw2013->g_leds_state[BLUE].led_brightness);
	}
	if(breath_led_flag == 1)
	{
		AW2013_i2c_write_reg(0x31, Imax  | 0x70);	//config mode, IMAX = 5mA//breath mode
		AW2013_i2c_write_reg(0x32, Imax  | 0x70);	//config mode, IMAX = 5mA
		AW2013_i2c_write_reg(0x33, Imax  | 0x70);	//config mode, IMAX = 5mA
	}
	printk("alvin_wu:aw2013_led_late_resume\n");
}

static void aw2013_led_register_early_suspend(struct aw2013_led *led)
{
	led->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	led->early_suspend.suspend = aw2013_led_early_suspend;
	led->early_suspend.resume = aw2013_led_late_resume;

	register_early_suspend(&led->early_suspend);
}

static void aw2013_led_unregister_early_suspend(struct aw2013_led *led)
{
	unregister_early_suspend(&led->early_suspend);
}
#endif


static int  aw2013_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	struct aw2013 *aw2013=NULL;
	struct aw2013_led *led = NULL;
	int ret;
	int i=0;

	printk("%s\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	aw2013 = (struct aw2013 *)kzalloc(sizeof(struct aw2013), GFP_KERNEL);
	if (aw2013 == NULL)
		return -ENOMEM;
	else
		printk("struct aw2013 kzalloc success!\n");

	aw2013->aw2013_data = (struct aw2013_led_platform_data *)kzalloc(sizeof(struct aw2013_led_platform_data), GFP_KERNEL);
	if (aw2013->aw2013_data == NULL)
		return -ENOMEM;
	else
		printk("struct aw2013_led _platform_data  kzalloc success!\n");

	aw2013->dev  = (struct device *)kzalloc(sizeof(struct device), GFP_KERNEL);
	if (aw2013->dev  == NULL)
		return -ENOMEM;
	else
		printk("struct device  kzalloc success!\n");

	aw2013->i2c = (struct i2c_client *)kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (aw2013->i2c == NULL)
		return -ENOMEM;
	else
		printk("struct i2c_client  kzalloc success!\n");

	led = (struct aw2013_led *)kzalloc(sizeof(struct aw2013_led), GFP_KERNEL);
	if (led  == NULL)
		return -ENOMEM;
	else
		printk("struct aw2013_led  kzalloc success!\n");

	i2c_set_clientdata(i2c, aw2013);

	aw2013->dev = &i2c->dev;
	aw2013->i2c = i2c;
	aw2013->irq = i2c->irq;
	aw2013->num_leds=3; //if has more leds added ,need modify this
	aw2013->aw2013_data=&aw2013_leds[0];
	g_i2c_aw2013=i2c;

	printk("aw2013_g_i2c_aw2013!\n");
	for(i = 0; i <= 2; i++)
	{
		aw2013->g_leds_state[i].led_risetime = Rise_time;
		aw2013->g_leds_state[i].led_holdtime = Hold_time;
		aw2013->g_leds_state[i].led_falltime = Fall_time;
		aw2013->g_leds_state[i].led_offtime = Off_time;
		aw2013->g_leds_state[i].led_on = 0;
		aw2013->g_leds_state[i].led_blink_on = 0;
		aw2013->g_leds_state[i].led_brightness = 0;
	}
	ret = aw2013_device_init(aw2013);
	if (ret) {
		printk("aw2013_device_init  fail\n");
		kfree(aw2013);
		return ret;

	}
	else
		printk("aw2013_device_init success\n");
#if 0 //need add 
	//read sensor id , return -1 if fail

	return -1;
#endif 

	for(i=0;i<aw2013->num_leds;i++)
	{
		printk("aw2013->aw2013_data[i].name = %s\n",aw2013->aw2013_data[i].name);
		aw2013->led_aw2013[i].cdev.name = aw2013->aw2013_data[i].name;
		aw2013->led_aw2013[i].cdev.brightness_set = aw2013_led_set;
		aw2013->led_aw2013[i].cdev.brightness_get = aw2013_led_get;
		aw2013->led_aw2013[i].cdev.blink_set =  aw2013_led_blink_set;
		aw2013->led_aw2013[i].cdev.brightness = LED_OFF;
		aw2013->led_aw2013[i].id = i;
		INIT_WORK(&aw2013->led_aw2013[i].work, aw2013_led_work);
		ret = led_classdev_register(&i2c->dev, &aw2013->led_aw2013[i].cdev);
		if (ret) {
			printk("aw2013 failed to register LED %d\n", aw2013->led_aw2013[i].id);
		}
		else
			printk("aw2013 success to register LED %d\n", aw2013->led_aw2013[i].id);

		ret = sysfs_create_group(&aw2013->led_aw2013[i].cdev.dev->kobj , &aw2013_led_attribute_group);
		if (ret < 0) {
			printk("aw2013 failed to create sysfs attributes\n");
		}
		else
			printk("aw2013 success  to create sysfs attributes\n");
	}
	aw2013_led_register_early_suspend(led);
	g_aw2013 = aw2013;
	return 0;
}

static int aw2013_i2c_remove(struct i2c_client *i2c)
{
	int i;
	for(i=0;i<g_aw2013->num_leds;i++)
	{
		sysfs_remove_group(&g_aw2013->led_aw2013[i].cdev.dev->kobj, &aw2013_led_attribute_group);
		led_classdev_unregister(&g_aw2013->led_aw2013[i].cdev);
		flush_work(&g_aw2013->led_aw2013[i].work);
		cancel_work_sync(&g_aw2013->led_aw2013[i].work);	
	}
	return 0;
}


static const struct i2c_device_id aw2013_i2c_ids[] = {
	{ "aw2013", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, aw2013_i2c_ids);

static struct i2c_driver aw2013_i2c_driver = {
	.driver = {
		.name = "aw2013",
		.owner = THIS_MODULE,
	},
	.id_table	= aw2013_i2c_ids,
	.probe		= aw2013_i2c_probe,
	.remove		= aw2013_i2c_remove,
};

static struct aw2013_led_platform_data aw2013_leds[] = {
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
};

static struct i2c_board_info __initdata ll95_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("aw2013", (0x45)),
	},
};


static int aw2013_led_probe(struct platform_device *pdev)
{
	i2c_register_board_info(3, ll95_i2c_boardinfo,ARRAY_SIZE(ll95_i2c_boardinfo)); 
	if(i2c_add_driver(&aw2013_i2c_driver))
	{
		printk("fail to add device into i2c");
	}
	return 0;
}

static int aw2013_led_remove(struct platform_device *pdev)
{
	printk("%s\n", __func__);
	AW2013_i2c_write_reg(0x00, 0x55);
	kfree(g_i2c_aw2013);
	kfree(g_aw2013);
	return 0;
}

static int aw2013_led_resume(struct platform_device *pdev)
{

	if(sleep_flag){
		g_aw2013->g_leds_state[BLUE].led_on = 1;
		g_aw2013->g_leds_state[BLUE].led_brightness = 255;
	aw2013_breath_all(g_aw2013->g_leds_state[RED].led_blink_on, g_aw2013->g_leds_state[GREEN].led_blink_on, g_aw2013->g_leds_state[BLUE].led_blink_on);
	led_on_aw2013(g_aw2013->g_leds_state[RED].led_on, g_aw2013->g_leds_state[RED].led_brightness, g_aw2013->g_leds_state[GREEN].led_on, g_aw2013->g_leds_state[GREEN].led_brightness, g_aw2013->g_leds_state[BLUE].led_on, g_aw2013->g_leds_state[BLUE].led_brightness);
	return 0;
	}
	AW2013_i2c_write_reg(0x30, g_aw2013->g_leds_state[BLUE].led_blink_on<<2|g_aw2013->g_leds_state[GREEN].led_blink_on<<1|g_aw2013->g_leds_state[RED].led_blink_on);//led on=0x01 ledoff=0x00
	AW2013_i2c_write_reg(0x01,0x01);
	AW2013_delay_1ms(1);
	printk("alvin_wu:aw2013 led  resume\n");
	return 0;
}

static int aw2013_led_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	if(sleep_flag){
		g_aw2013->g_leds_state[BLUE].led_on = 1;
		g_aw2013->g_leds_state[BLUE].led_brightness = 0;
		aw2013_breath_all(g_aw2013->g_leds_state[RED].led_blink_on, g_aw2013->g_leds_state[GREEN].led_blink_on, g_aw2013->g_leds_state[BLUE].led_blink_on);
		led_on_aw2013(g_aw2013->g_leds_state[RED].led_on, g_aw2013->g_leds_state[RED].led_brightness, g_aw2013->g_leds_state[GREEN].led_on, g_aw2013->g_leds_state[GREEN].led_brightness, g_aw2013->g_leds_state[BLUE].led_on, g_aw2013->g_leds_state[BLUE].led_brightness);
	}
	if(!(g_aw2013->g_leds_state[RED].led_blink_on|g_aw2013->g_leds_state[GREEN].led_blink_on|g_aw2013->g_leds_state[BLUE].led_blink_on))  {
		AW2013_i2c_write_reg(0x30, 0);				//all led off
		AW2013_i2c_write_reg(0x01,0);
	}
	if(breath_led_flag == 0 )
	{
		AW2013_i2c_write_reg(0x00, 0x55); //reset aw2013
		printk("alvin_wu:aw2013 led suspend\n");
	}
	else
	{
		printk("alvin_wu:aw2013 led not suspend\n");
	}
		
	return 0;
}



static struct platform_driver aw2013_led_driver = {
	.driver = {
		.name = "aw2013-leds",
		.owner = THIS_MODULE,
	},
	.suspend    = aw2013_led_suspend,
	.resume     = aw2013_led_resume,
	.probe		= aw2013_led_probe,
	.remove		= aw2013_led_remove,
};

#ifdef CONFIG_OF
static struct platform_device aw2013_leds_device = {
	.name = "aw2013-leds",
	.id = -1
};
#endif



static int __init aw2013_led_init(void)
{
	int ret;

#ifdef CONFIG_OF
	ret = platform_device_register(&aw2013_leds_device);
	if (ret)
		printk("[LED]mt65xx_leds_init:dev:E%d\n", ret);
#endif

	return platform_driver_register(&aw2013_led_driver);
}


static void __exit aw2013_led_exit(void)
{
	platform_driver_unregister(&aw2013_led_driver);
}


module_init(aw2013_led_init);
module_exit(aw2013_led_exit);
MODULE_AUTHOR("Johan Hovold <jhovold@gmail.com>");
MODULE_DESCRIPTION("aw2013 LED driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:aw2013-leds");

