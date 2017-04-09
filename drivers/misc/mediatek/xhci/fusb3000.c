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

#define FUSB3000_I2C_NAME "FUSB3000"

int CC_DECODER_TYPE;

struct fusb3000_i2c_data {
	struct i2c_client	*client;
    struct work_struct  eint_work;
    struct delayed_work  prob_work;
    struct task_struct  *thread;

    #if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend    early_drv;
    #endif   
};

extern void cc_decoder_mode_off(void);
extern void cc_decoder_mode_on(void);
struct fusb3000_i2c_data *fusb_i2c_data;
/*----------------------------------------------------------------------------*/
static int fusb3000_i2c_read(struct i2c_client *client, char reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

    return ret;
}

static int fusb3000_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int fusb3000_power_onoff(int value)
{
    if(value)
        hwPowerOn(MT6331_POWER_LDO_VMC, VOL_3300, "fusb");
    else
        hwPowerDown(MT6331_POWER_LDO_VMC,"fusb");
}

#ifndef USE_EARLY_SUSPEND
static int fusb3000_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//wait to do something
	return 0;
}
static int fusb3000_resume(struct i2c_client *client)
{
	//wait to do something
	return 0;
}
#else 
static void fusb3000_early_suspend(struct early_suspend *h) 
{
	//wait to do something
}
static void fusb3000_late_resume(struct early_suspend *h)
{
	//wait to do something
    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
}
#endif

#ifdef DEBUG_TEST
int tolower(int c)
{
	if ((c >= 'A') && (c <= 'Z'))
		return c + ('a' - 'A');
	return c;
}

int htoi(char *s)  
{  
    int i;  
    int n = 0;  
    if (s[0] == '0' && (s[1]=='x' || s[1]=='X'))  
    {  
        i = 2;  
    }  
    else  
    {  
        i = 0;  
    }  
    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z');++i)  
    {  
        if (tolower(s[i]) > '9')  
        {  
            n = 16 * n + (10 + tolower(s[i]) - 'a');  
        }  
        else  
        {  
            n = 16 * n + (tolower(s[i]) - '0');  
        }  
    }      
    return n;  
}  

static ssize_t usb_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t usb_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	 char dat[64] ;
     int cmd,len;
     char * data_value;
     int dat1[64];
     int i;
	sscanf(buf, "%s", dat);
    char *s = dat;    
    for(i=0;(data_value = strsep(&s, "_"))!=NULL; i++)
    {
        if(i==0)
            cmd = htoi(data_value);
        else if (i==1)
            len = htoi(data_value);
        else
            dat1[i-2] = htoi(data_value);
    }
    
    printk("cmd(%x) len(%x) dat1(%x) dat2(%x) dat3(%x)\n",cmd,len,dat1[0],dat1[1],dat1[2]);
    fusb3000_i2c_write(fusb_i2c_data->client,cmd,dat1);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t usb_show_recv(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t usb_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	char dat[64] ;
	int cmd,len;
	char * data_value;
	int dat1[64];
	int i;
	unsigned int data_array[16];
	unsigned char buffer[5];

	sscanf(buf, "%s", dat);
	char *s = dat;
	for(i=0;(data_value = strsep(&s, "_"))!=NULL; i++)
	{
		if(i==0)
		    data_array[0] = htoi(data_value);
		else if (i==1)
		    len = htoi(data_value);
		/*else
		    dat1[0] = htoi(data_value);*/
	}
    fusb3000_i2c_read(fusb_i2c_data->client,data_array[0]);
	return 0;
}
/*----------------------------------------------------------------------------*/


static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, usb_show_send,  usb_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, usb_show_recv,  usb_store_recv);

static struct driver_attribute *usb_attr_list[] = {
    &driver_attr_send,
    &driver_attr_recv,
};

 int usb_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(usb_attr_list)/sizeof(usb_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, usb_attr_list[idx])))
		{            
			printk("driver_create_file (%s) = %d\n", usb_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

/*----------------------------------------------------------------------------*/
 int usb_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(usb_attr_list)/sizeof(usb_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, usb_attr_list[idx]);
	}
	
	return err;
}
#endif

void fusb300_power_init(void)
{
   // fusb3000_power_onoff(1);
    msleep(100);
    if(fusb3000_i2c_read(fusb_i2c_data->client,0x02)!=0x0C)
    {
        fusb3000_i2c_write(fusb_i2c_data->client,0x0E,0x00);
        fusb3000_i2c_write(fusb_i2c_data->client,0x26,0xFF);
        fusb3000_i2c_write(fusb_i2c_data->client,0x27,0xF8);
        fusb3000_i2c_write(fusb_i2c_data->client,0x28,0x00);
        fusb3000_i2c_write(fusb_i2c_data->client,0x29,0x80);
        fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x0C);
        fusb3000_i2c_write(fusb_i2c_data->client,0x0b,0x01);
        fusb3000_i2c_write(fusb_i2c_data->client,0x06,0x08);
        fusb3000_i2c_write(fusb_i2c_data->client,0x0a,0x7b);
        fusb3000_i2c_read(fusb_i2c_data->client,0x42);
        msleep(100);
        mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
        printk("fusb300_power_init successful\n");
    }
}

static int usb_check_dfp(void *unused)
{
    int cc1=0;
    int cc2=0;
    int status0_value=0;
    int interrupt_value=0;

    static int tag_flag = 0;
    fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x44);
    status0_value=fusb3000_i2c_read(fusb_i2c_data->client,0x40);
    if((status0_value&0x20)==0x20)
    {
        cc1=0;
    }
    else
    {
        if((status0_value&0x03)==0x00)
        {
            cc1=1;
        }
        else
        {
            cc1=2;
        }
    }
    
    fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x88);
    status0_value=fusb3000_i2c_read(fusb_i2c_data->client,0x40);
    if((status0_value&0x20)==0x20)
    {
        cc2=0;
    }
    else
    {
        if((status0_value&0x03)==0x00)
        {
            cc2=1;
        }
        else
        {
            cc2=2;
        }
    }
    
    printk("usb_check_dfp for cc value cc1(%d) cc2(%d)\n",cc1,cc2);    
    if((cc1!=0)||(cc2!=0))
    {
        if(cc1==2)
        {
            fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x64);
        }
        else if(cc2==2)
        {
            fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x98);
        }
        else  
        {
            printk("this table is not letv otg table return\n");
            /*fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x0C);
            fusb3000_i2c_write(fusb_i2c_data->client,0x0b,0x01);
            fusb3000_i2c_write(fusb_i2c_data->client,0x06,0x00);
            fusb3000_i2c_read(fusb_i2c_data->client,0x42);*/
            return 0;
        }
        tag_flag = 1;
        cc_decoder_mode_on();
    }
    else if((cc1==0)&&(cc2==0))
    {
        printk("this is usb/AC, do nothing just return\n");
        fusb300_power_init();
        return 0;
    }

    
    if(tag_flag==1)
    {

        while(1)
        {
            interrupt_value=fusb3000_i2c_read(fusb_i2c_data->client,0x42);
            status0_value=fusb3000_i2c_read(fusb_i2c_data->client,0x40);        
            if(((interrupt_value&0x20)==0x20)&&((status0_value&0x20)==0x20))
            {
                cc_decoder_mode_off();
                //msleep(200);
                fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x0C);
                fusb3000_i2c_write(fusb_i2c_data->client,0x0b,0x01);
                fusb3000_i2c_write(fusb_i2c_data->client,0x06,0x08);
                fusb3000_i2c_read(fusb_i2c_data->client,0x42);
                msleep(10);
                mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);

                tag_flag = 0;
                return 0;
            }
            else
            {
                //otg tbale insert wait to do something.
                //printk("otg table insert\n");
                msleep(200);

            }
        }
    }
    return 0;
}

static void cc_eint_interrupt_handler(void)
{
    schedule_work(&fusb_i2c_data->eint_work);
}
static void fusb_prob_work(struct work_struct *work)
{
    mt_eint_mask(CUST_EINT_CC_DECODER_NUM);
    fusb3000_i2c_write(fusb_i2c_data->client,0x06,0x08);
    fusb3000_i2c_write(fusb_i2c_data->client,0x04,0x26);
    fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x40);
    fusb3000_i2c_write(fusb_i2c_data->client,0x0B,0x07);   
    usb_check_dfp(NULL);
    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
}

static void fusb_eint_work(struct work_struct *work)
{
    int interruput_value = 0;

    interruput_value = fusb3000_i2c_read(fusb_i2c_data->client,0x42);
        
    if(((interruput_value&0x80)==0x80)||((interruput_value&0x20)==0x20)||(interruput_value==0x00))//we mask vbus& Icomp interrupt
    {
        mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
        return;
    }
    
    fusb3000_i2c_write(fusb_i2c_data->client,0x06,0x28);
    fusb3000_i2c_write(fusb_i2c_data->client,0x04,0x26);
    fusb3000_i2c_write(fusb_i2c_data->client,0x02,0x40);
    fusb3000_i2c_write(fusb_i2c_data->client,0x0B,0x07);   
    fusb_i2c_data->thread= kthread_run(usb_check_dfp, 0, "usb300_check_dfp");   
}

static int fusb3000_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct fusb3000_i2c_data *fusb;
    int err;
    int device_id;
    int irq_id;
    
    fusb3000_power_onoff(1);
        
	fusb = kzalloc(sizeof(struct fusb3000_i2c_data), GFP_KERNEL);
	if (!fusb) {
		dev_err(&i2c->dev, "private data alloc fail\n");
        goto exit;
	}
    
    fusb_i2c_data = fusb;
	i2c_set_clientdata(i2c, fusb);
	fusb->client = i2c;
    device_id =fusb3000_i2c_read(fusb->client,0x01);
    
    if((device_id!=0x50)&&(device_id!=0x60))
    {
        printk("fusb3000_probe for find id(%x) error\n",device_id);
        goto exit;
    }
    
    fusb3000_i2c_write(fusb->client,0x0E,0x00);
    fusb3000_i2c_write(fusb->client,0x26,0xFF);
    fusb3000_i2c_write(fusb->client,0x27,0xF8);
    fusb3000_i2c_write(fusb->client,0x28,0x00);
    fusb3000_i2c_write(fusb->client,0x29,0x80);
    fusb3000_i2c_write(fusb->client,0x02,0x0C);
    fusb3000_i2c_write(fusb->client,0x0b,0x01);
    fusb3000_i2c_write(fusb->client,0x06,0x08);
    fusb3000_i2c_write(fusb->client,0x0a,0x7b);
    fusb3000_i2c_read(fusb->client,0x42);
    msleep(100);
    
    #ifdef USE_EARLY_SUSPEND
	fusb->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	fusb->early_drv.suspend  = fusb3000_early_suspend,
	fusb->early_drv.resume   = fusb3000_late_resume,    
	register_early_suspend(&fusb->early_drv);
    #endif 
    
    INIT_WORK(&fusb_i2c_data->eint_work, fusb_eint_work);
    INIT_DELAYED_WORK(&fusb_i2c_data->prob_work, fusb_prob_work);
    
    mt_eint_registration(CUST_EINT_CC_DECODER_NUM, CUST_EINTF_TRIGGER_FALLING, cc_eint_interrupt_handler, 0);// disable auto-unmask
    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
    
    schedule_delayed_work(&fusb_i2c_data->prob_work,msecs_to_jiffies(2000));
    
    return 0;
    
exit:
    return -1;
}

static int fusb3000_remove(struct i2c_client *i2c)
{
	i2c_unregister_device(i2c);
	kfree(i2c_get_clientdata(i2c));

    fusb3000_power_onoff(0);

    return 0;
}

static const struct i2c_device_id fusb3000_id[] = {
	{ FUSB3000_I2C_NAME, 0 },
	{ }
};

static struct i2c_board_info __initdata fusb3000_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(FUSB3000_I2C_NAME, (0x22)),
	},
};

static struct i2c_driver fusb3000_i2c_driver = {
	.driver = {
		.name = FUSB3000_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= fusb3000_probe,
	.remove		= fusb3000_remove,
	#if !defined(USE_EARLY_SUSPEND)    
    .suspend            = fusb3000_suspend,
    .resume             = fusb3000_resume,
    #endif
	.id_table	= fusb3000_id,
};
static int __init fusb3000_i2c_init(void)
{
	i2c_register_board_info(3, fusb3000_i2c_boardinfo, 
				ARRAY_SIZE(fusb3000_i2c_boardinfo)); 
	return i2c_add_driver(&fusb3000_i2c_driver);
}

static void __exit fusb3000_i2c_exit(void)
{
	i2c_del_driver(&fusb3000_i2c_driver);
}

module_init(fusb3000_i2c_init);
module_exit(fusb3000_i2c_exit);
