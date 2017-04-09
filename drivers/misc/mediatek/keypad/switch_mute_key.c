
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

#define MUTE_KEY_NAME "mute_key"
#define DEBUG 1

struct mute_key_data {
    struct switch_dev sdev;
    struct work_struct work;
    struct device *dev;
    struct platform_device *pdev;	
    atomic_t state;
    unsigned gpio;
    int irq;
};

static struct mute_key_data *this_data = NULL;
static int old_INT_stat = 1;
unsigned int mute_state_value = 0;

static irqreturn_t gpio_irq_handler(void)
{
#if DEBUG
	printk("[muet_key]>>>gpio_irq_handler\n");
#endif

	schedule_work(&this_data->work);
	mt_eint_unmask(CUST_EINT_SWITCH_MUTE_NUM);
    
}

static void switch_work_func(struct work_struct *work)
{
	int state = 0;
	struct mute_key_data	*data =
		container_of(work, struct mute_key_data, work);

	state = mt_get_gpio_in(data->gpio);

#if DEBUG
	printk("[muet_key]>>>switch_work_func state=%d, old_INT_stat=%d\n", state, old_INT_stat);
#endif

       if(old_INT_stat != state)
       {
            if(state == 0)
                  mt_eint_set_polarity(CUST_EINT_SWITCH_MUTE_NUM, 1);
            else
                  mt_eint_set_polarity(CUST_EINT_SWITCH_MUTE_NUM, 0);


            old_INT_stat = state;
       }
       
	switch_set_state(&this_data->sdev, !state);
}

static ssize_t show_mute_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	int state;

	state = switch_get_state(&this_data->sdev);
	return sprintf(buf, "%d\n", state);
}

static ssize_t store_mute_value(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	int state = 0;

	mute_state_value = simple_strtoul(buf,&pvalue,16);
	
	if(buf != NULL && size != 0)
	{
	    if(mute_state_value == 1) /*  wake up */
	    {
	        state = 1;
	        switch_set_state(&this_data->sdev, state);
	    }
            
           if(mute_state_value == 0) /*  sleep  */
           {
                state = 0;
                switch_set_state(&this_data->sdev, state);
           }		
	}

	return size;
}

static DEVICE_ATTR(mute_value, 0664, show_mute_value, store_mute_value);

static int mute_key_probe(struct platform_device *pdev)
{
	struct mute_key_data *mute_data;
	int ret = 0;
	int ret_device_file = 0;
	
	mt_set_gpio_mode(GPIO_SWITCH_MUTE_EINT, GPIO_SWITCH_MUTE_EINT_M_EINT);
	mt_set_gpio_dir(GPIO_SWITCH_MUTE_EINT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_SWITCH_MUTE_EINT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_SWITCH_MUTE_EINT, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_SWITCH_MUTE_NUM, CUST_EINT_SWITCH_MUTE_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_SWITCH_MUTE_NUM,CUST_EINT_SWITCH_MUTE_TYPE,  gpio_irq_handler, 0);
	mt_eint_unmask(CUST_EINT_SWITCH_MUTE_NUM);
	
    //switch device registing
	mute_data = kzalloc(sizeof(struct mute_key_data), GFP_KERNEL);
	if (!mute_data)
	{
	    printk("[muet_key]mute_key_probe>>>kzalloc fail\n");
	    return -ENOMEM;
	}

	this_data = mute_data;
	mute_data->sdev.name = MUTE_KEY_NAME;
	mute_data->gpio = GPIO_SWITCH_MUTE_EINT;

       ret = switch_dev_register(&mute_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
		
	INIT_WORK(&mute_data->work, switch_work_func);

       ret_device_file = device_create_file(&(pdev->dev), &dev_attr_mute_value); 
	if (ret_device_file) {
		pr_err("mute_key_probe: driver_create_file fail!!! \n");
	}

	switch_work_func(&mute_data->work);
	printk("switch mute key probe ok!!!\n");
	return 0;

err_switch_dev_register:
	kfree(mute_data);

	return ret;
}

static int  mute_key_remove(struct platform_device *pdev)
{
	cancel_work_sync(&this_data->work);
	switch_dev_unregister(&this_data->sdev);
	kfree(this_data);

	return 0;
}

static int mute_key_suspend(struct platform_device *pdev, pm_message_t state)
{
#if DEBUG
	printk("[em6781][suspend]!!!!!!\n");
#endif
	cancel_work_sync(&this_data->work);
	
	return 0;
}

static int mute_key_resume(struct platform_device *pdev)
{
#if DEBUG	
	printk("[mute_gpio][resume]!!!!!!\n");
#endif

	schedule_work(&this_data->work);
	return 0;
}

#ifdef CONFIG_OF
	static const struct of_device_id mute_of_match[] = {
		{ .compatible = "mediatek,mute_gpio", },
		{},
	};
#endif

static struct platform_driver mute_key_driver = {
    .probe      = mute_key_probe,
    .remove     = mute_key_remove,
    .suspend    = mute_key_suspend,
    .resume     = mute_key_resume,
    .driver = {
        .name   = MUTE_KEY_NAME,
        //.owner  = THIS_MODULE,
         #ifdef CONFIG_OF
			.of_match_table = mute_of_match,
		#endif
    },
};

static int __init mute_key_init(void)
{
	return platform_driver_register(&mute_key_driver);
}

static void __exit mute_key_exit(void)
{
	platform_driver_unregister(&mute_key_driver);
}

module_init(mute_key_init);
module_exit(mute_key_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");

