
/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <cust_eint.h>

#define SENSOR_NAME "hall_gpio"

struct sensor_data {
    struct switch_dev sdev;
    struct work_struct work;
    //struct delayed_work work;
    struct device *dev;
    struct platform_device *pdev;	
    atomic_t state;
    unsigned gpio;
    int irq;
};

static struct sensor_data *this_data = NULL;
static int old_INT_stat = 1;
unsigned int hall_state_value = 0;

static irqreturn_t gpio_irq_handler(void)
{
#if DEBUG
	printk("[hall_sensor]>>>gpio_irq_handler\n");
#endif

	//schedule_delayed_work(&this_data->work, 0);
	schedule_work(&this_data->work);
	//return IRQ_HANDLED;
	//mt65xx_eint_unmask(CUST_EINT_MHALL_NUM);
        mt_eint_mask(CUST_EINT_MHALL_NUM);
    
}

//static void switch_work_func(struct delayed_work *work)
static void switch_work_func(struct work_struct *work)
{
	int state = 0;
	struct sensor_data	*data =
		container_of(work, struct sensor_data, work);

	state = mt_get_gpio_in(data->gpio);

#if DEBUG
	printk("[hall_sensor][hall_gpio]>>>switch_work_func state=%d, old_INT_stat=%d\n", state, old_INT_stat);
#endif

       if(old_INT_stat != state)
       {
            if(state == 0)
                //mt65xx_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINT_POLARITY_HIGH);
                  //mt_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINTF_TRIGGER_HIGH);
                   mt_eint_set_polarity(CUST_EINT_MHALL_NUM, 1);
            else
                //mt65xx_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINT_POLARITY_LOW);
                  //mt_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINTF_TRIGGER_LOW);
                  mt_eint_set_polarity(CUST_EINT_MHALL_NUM, 0);


            old_INT_stat = state;
       }
       
    mt_eint_unmask(CUST_EINT_MHALL_NUM);

	switch_set_state(&this_data->sdev, state);
}



static ssize_t show_hall_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	int state;

	state = switch_get_state(&this_data->sdev);
	return sprintf(buf, "%d\n", state);
}

static ssize_t store_hall_value(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	int state = 0;

	hall_state_value = simple_strtoul(buf,&pvalue,16);
	
	if(buf != NULL && size != 0)
	{
	    if(hall_state_value == 1) /*  wake up */
	    {
	        state = 1;
	        switch_set_state(&this_data->sdev, state);
	    }
            
           if(hall_state_value == 0) /*  sleep  */
           {
                state = 0;
                switch_set_state(&this_data->sdev, state);
           }		
	}

	return size;
}

static DEVICE_ATTR(hall_value, 0664, show_hall_value, store_hall_value);

static int sensor_probe(struct platform_device *pdev)
{
	struct sensor_data *hall_data;
	int ret = 0;
	int ret_device_file = 0;
	
	printk(KERN_DEBUG "hall_sensor %s\n", __func__);
	
       //switch device registing
	hall_data = kzalloc(sizeof(struct sensor_data), GFP_KERNEL);
	if (!hall_data)
	{
	    printk("[hall_sensor]sensor_probe>>>kzalloc fail\n");
	    return -ENOMEM;
	}

	this_data = hall_data;
	hall_data->sdev.name = SENSOR_NAME;
	hall_data->gpio = GPIO_MHALL_EINT_PIN;

       ret = switch_dev_register(&hall_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
		
	//INIT_DELAYED_WORK(&hall_data->work, switch_work_func);
	INIT_WORK(&hall_data->work, switch_work_func);

       ret_device_file = device_create_file(&(pdev->dev), &dev_attr_hall_value); 
	if (ret_device_file) {
		pr_err("sensor_probe: driver_create_file fail!!! \n");
	}

	mt_set_gpio_mode(GPIO_MHALL_EINT_PIN, GPIO_MHALL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_MHALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_MHALL_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_MHALL_EINT_PIN, GPIO_PULL_UP);

	//mt65xx_eint_set_sens(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_SENSITIVE);
        //mt_eint_set_sens( CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_SENSITIVE);
	//mt65xx_eint_set_polarity(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_POLARITY);
        //mt_eint_set_polarity( CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_POLARITY);
	//mt65xx_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);
	//mt65xx_eint_registration(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_EN, CUST_EINT_MHALL_POLARITY, gpio_irq_handler, 0);
        mt_eint_registration(CUST_EINT_MHALL_NUM,CUST_EINT_MHALL_TYPE,  gpio_irq_handler, 0);
        mt_eint_set_hw_debounce(CUST_EINT_MHALL_NUM, CUST_EINT_MHALL_DEBOUNCE_CN);

	//mt65xx_eint_unmask(CUST_EINT_MHALL_NUM);  
        mt_eint_unmask(CUST_EINT_MHALL_NUM);

	switch_work_func(&hall_data->work);
	
	return 0;

err_switch_dev_register:
	kfree(hall_data);

	return ret;
}

static int  sensor_remove(struct platform_device *pdev)
{
	//cancel_delayed_work_sync(&this_data->work);
	cancel_work_sync(&this_data->work);
       switch_dev_unregister(&this_data->sdev);
	kfree(this_data);

	return 0;
}

static int sensor_suspend(struct platform_device *pdev, pm_message_t state)
{
#if DEBUG
	printk("[em6781][suspend]!!!!!!\n");
#endif
	//cancel_delayed_work_sync(&this_data->work);
	cancel_work_sync(&this_data->work);
	
	return 0;
}

static int sensor_resume(struct platform_device *pdev)
{
#if DEBUG	
	printk("[hall_gpio][resume]!!!!!!\n");
#endif

	schedule_work(&this_data->work);
	//schedule_delayed_work(&this_data->work, 0);
	return 0;
}

#ifdef CONFIG_OF
	static const struct of_device_id hallsensor_of_match[] = {
		{ .compatible = "mediatek,hall_gpio", },
		{},
	};
#endif

static struct platform_driver sensor_driver = {
    .probe      = sensor_probe,
    .remove     = sensor_remove,
    .suspend    = sensor_suspend,
    .resume     = sensor_resume,
    .driver = {
        .name   = SENSOR_NAME,
        //.owner  = THIS_MODULE,
         #ifdef CONFIG_OF
			.of_match_table = hallsensor_of_match,
		#endif
    },
};

static int __init sensor_init(void)
{
	return platform_driver_register(&sensor_driver);
}

static void __exit sensor_exit(void)
{
	platform_driver_unregister(&sensor_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");

