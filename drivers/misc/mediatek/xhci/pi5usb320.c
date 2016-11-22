#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <mach/mt_pm_ldo.h>
#include <linux/interrupt.h>
#include <linux/time.h>
#include <cust_eint.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <linux/kthread.h>
#include <mach/mt_gpio.h>

#define PI5USB320_I2C_NAME "PI5USB320"
static int trysink_flag =0;
int high_current_cable =0;
struct pi5usb_i2c_data {
	struct i2c_client	*client;
    struct work_struct  eint_work;
    struct delayed_work  drp_work;
    struct task_struct  *thread;

    #if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend    early_drv;
    #endif   
};

static DEFINE_MUTEX(pi5usb_i2c_lock);

struct pi5usb_i2c_data *pusb_i2c_data;

 
static int pusb320_i2c_read(struct i2c_client *client,char *rxData, int length)
{
	return i2c_master_recv(client, (const char*)rxData, length);
	 
}

static int pusb320_i2c_write(struct i2c_client *client,char *txData, int length)
{
	return i2c_master_send(client, (const char*)txData, length);
}

#ifndef USE_EARLY_SUSPEND
static int pi5usb_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//wait to do something
	return 0;
}
static int pi5usb_resume(struct i2c_client *client)
{
	//wait to do something
	return 0;
}
#else 
static void pi5usb_early_suspend(struct early_suspend *h) 
{
	//wait to do something
}
static void pi5usb_late_resume(struct early_suspend *h)
{
	//wait to do something
    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
}
#endif

static int pi5usb_power_onoff(int value)
{
    if(value)
        hwPowerOn(MT6331_POWER_LDO_VMC, VOL_3300, "tusb");
    else
        hwPowerDown(MT6331_POWER_LDO_VMC,"tusb");

    printk(KERN_ERR "justin test printk\n");
}

static void pusb_eint_work(struct work_struct *work)
{
    char rx_buffer[4];

    char tx_buffer[4]={0x00};
    
    u8 attached_value = 0;
    //static int uart_flag =0;
    static int  current_mode = 0x04;

    pusb320_i2c_read(pusb_i2c_data->client, rx_buffer, 4);
    attached_value = (rx_buffer[3] &0x1c)>>2;
    printk("pi5usb attached value:%x %x %x %x %x %x\n",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],attached_value,trysink_flag);
    if(((rx_buffer[2] &0x02)>>1)&&(rx_buffer[3]==0x00))//unattached.
    {
        trysink_flag=0;
        current_mode=0x04;
        high_current_cable=0;
        mt_set_gpio_out(175, 1);

    }

    if(attached_value==0x04)
    {
        mt_set_gpio_dir(175, 1);
        mt_set_gpio_out(175, 0);
       // uart_flag = 1;
    }
    
   /* if((attached_value==0x00)&&(uart_flag == 1))
    {
        uart_flag = 0;
        mt_set_gpio_out(175, 1);
    }*/


    if(rx_buffer[3]==0x00||rx_buffer[3]==0x80)//abnormal int.
    {
        current_mode=0x04;
    }
    
    if(rx_buffer[3]==0x04)//audio accessory cable(bad) .
    {
        tx_buffer[1]=0x01;
        pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
        mdelay(30);
    }
    
    if(attached_value==0x03)
    {
        current_mode=0x04;
        //it is a audio accessory to do...
    }
    
    if(attached_value==0x02)
    {
        if((rx_buffer[3]&0x60)>>5==0x03)
        {
            high_current_cable = 1;
        }
        current_mode=0x04;
        //host device do nothing return.
    }

    
    if(attached_value==0x01)//device
    {
        if(trysink_flag==0)
        {
            tx_buffer[1]=0x01;
            pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);//enter UFP.
            mdelay(500);
            pusb320_i2c_read(pusb_i2c_data->client, rx_buffer, 4);

            printk("pi5usb attached value11:%x %x %x %x %x %x\n",rx_buffer[0],rx_buffer[1],rx_buffer[2],rx_buffer[3],attached_value,trysink_flag);

            
            if(((rx_buffer[3] &0x1c)>>2)==0x02)
            {
                current_mode=0x00;
                printk("trySNK host attached\n");
            }
            else
            {
            //reintial 
                tx_buffer[1]=0x05;
                pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
                mdelay(50);
                tx_buffer[1]=0x01;
                pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
                mdelay(30);
                tx_buffer[1]=0x05;
                pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
                mdelay(30);
                pusb320_i2c_read(pusb_i2c_data->client, rx_buffer, 4);
                current_mode=0x04;
            }
            trysink_flag=1;
            
        }
    }
    
        mdelay(20);
        tx_buffer[1]=current_mode;//enter DRP.
        pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);

    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);

}

static void cc_eint_interrupt_handler(void)
{
    schedule_work(&pusb_i2c_data->eint_work);
}

static void pi5usb320_set_device(void)
{
    char rx_buffer[4];
    char tx_buffer[2]={0x00,0x01};
    pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
    mdelay(1200);// to be fix
    pusb320_i2c_read(pusb_i2c_data->client, rx_buffer, 4);
    mdelay(50);
    tx_buffer[1] = 0x00;
    pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
}

static void pi5usb320_set_host(void)
{
    char rx_buffer[4];
    char tx_buffer[2]={0x00,0x03};
    pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
    mdelay(1200);// to be fix
    pusb320_i2c_read(pusb_i2c_data->client, rx_buffer, 4);
    mdelay(50);
    tx_buffer[1] = 0x02;
    pusb320_i2c_write(pusb_i2c_data->client,tx_buffer,2);
}


static int pi5usb_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct pi5usb_i2c_data *fusb;
    char tx_buffer[2]={0x00,0x01};
    char rx_buffer[4];
    int ret;

    pi5usb_power_onoff(1);
        
	fusb = kzalloc(sizeof(struct pi5usb_i2c_data), GFP_KERNEL);
	if (!fusb) {
		dev_err(&i2c->dev, "private data alloc fail\n");
        return -1;
	}
    
    pusb_i2c_data = fusb;
	i2c_set_clientdata(i2c, fusb);
	fusb->client = i2c;

    ret = pusb320_i2c_read(fusb->client, rx_buffer, 4);
    if((ret <0) || (rx_buffer[0]!= 0))
    {
    	dev_err(&i2c->dev, "attached pusb fail\n");
        goto exit;
    }
    
   //intial code.
    pusb320_i2c_write(fusb->client,tx_buffer,2);
    mdelay(30);
    
    tx_buffer[1]=0x04;
    pusb320_i2c_write(fusb->client,tx_buffer,2);

    #ifdef USE_EARLY_SUSPEND
	fusb->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	fusb->early_drv.suspend  = pi5usb_early_suspend,
	fusb->early_drv.resume   = pi5usb_late_resume,    
	register_early_suspend(&fusb->early_drv);
    #endif 
    
    INIT_WORK(&pusb_i2c_data->eint_work, pusb_eint_work);
    mt_eint_registration(CUST_EINT_CC_DECODER_NUM, CUST_EINTF_TRIGGER_FALLING, cc_eint_interrupt_handler, 0);// disable auto-unmask
    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
    
    mt_set_gpio_out(175, 1);//temp solution for typeC-A detect!

    schedule_work(&pusb_i2c_data->eint_work);//insert cable power on.

    return 0;
    
exit:
    kfree(fusb);
    pusb_i2c_data = NULL;
    return -1;
}

static int pi5usb_remove(struct i2c_client *i2c)
{
	i2c_unregister_device(i2c);
	kfree(i2c_get_clientdata(i2c));
    pi5usb_power_onoff(0);
    return 0;
}

static const struct i2c_device_id pi5usb_id[] = {
	{ PI5USB320_I2C_NAME, 0 },
	{ }
};

static struct i2c_board_info __initdata pi5usb_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(PI5USB320_I2C_NAME, (0x1d)),
	},
};

static struct i2c_driver pi5usb_i2c_driver = {
	.driver = {
		.name = PI5USB320_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= pi5usb_probe,
	.remove		= pi5usb_remove,
	#if !defined(USE_EARLY_SUSPEND)    
    .suspend            = pi5usb_suspend,
    .resume             = pi5usb_resume,
    #endif
	.id_table	= pi5usb_id,
};
static int __init pi5usb_i2c_init(void)
{
	i2c_register_board_info(3, pi5usb_i2c_boardinfo, 
				ARRAY_SIZE(pi5usb_i2c_boardinfo)); 
	return i2c_add_driver(&pi5usb_i2c_driver);
}

static void __exit pi5usb_i2c_exit(void)
{
	i2c_del_driver(&pi5usb_i2c_driver);
}

module_init(pi5usb_i2c_init);
module_exit(pi5usb_i2c_exit);

