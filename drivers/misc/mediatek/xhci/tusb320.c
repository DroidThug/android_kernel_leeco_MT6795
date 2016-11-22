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

#define TUSB320_I2C_NAME "TUSB320"

struct tusb320_i2c_data {
	struct i2c_client	*client;
    struct work_struct  eint_work;
    struct delayed_work  trysnk_work;
    struct task_struct  *thread;

    #if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend    early_drv;
    #endif   
};

static DEFINE_MUTEX(tusb320_i2c_lock);

struct tusb320_i2c_data *tusb_i2c_data;
/*----------------------------------------------------------------------------*/
static int tusb320_i2c_read(struct i2c_client *client, u8 *data, u8 reg)
{
//	ret = i2c_smbus_read_byte_data(client, reg);
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

	mutex_lock(&tusb320_i2c_lock);
  
    client->ext_flag=((client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = reg;
    ret = i2c_master_send(client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        client->ext_flag=0;

        mutex_unlock(&tusb320_i2c_lock);
        return 0;
    }
    
    readData = cmd_buf[0];
    *data = readData;

    client->ext_flag=0;
	mutex_unlock(&tusb320_i2c_lock);

    return ret;
}

static int tusb320_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static void tusb320_power_onoff(int value)
{
    if(value)
        hwPowerOn(MT6331_POWER_LDO_VMC, VOL_3300, "tusb");
    else
        hwPowerDown(MT6331_POWER_LDO_VMC,"tusb");
}

#ifndef USE_EARLY_SUSPEND
static int tusb320_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//wait to do something
	return 0;
}
static int tusb320_resume(struct i2c_client *client)
{
	//wait to do something
	return 0;
}
#else 
static void tusb320_early_suspend(struct early_suspend *h) 
{
    tusb320_power_onoff(0);
	//wait to do something
}
static void tusb320_late_resume(struct early_suspend *h)
{
    tusb320_power_onoff(1);
	//wait to do something
    mt_eint_unmask(CUST_EINT_CC_DECODER_NUM);
}
#endif

int tusb_check_type_cable(void)
{
    u8 cable_type = 0;
    int ret;

    if(tusb_i2c_data == NULL)
	    return 0;

    ret=tusb320_i2c_read(tusb_i2c_data->client,&cable_type,0x08);

    printk("tusb test cable type %x\n",cable_type);

    return ((cable_type&0x30)>>4);
}

void tusb320_dump_register(void)
{
    int i=0;
    kal_uint8 tusb320_reg[12] = {0};

    printk("[tusb320]");

#if 1
    for (i=0;i<11;i++)
    {
        tusb320_i2c_read(tusb_i2c_data->client, &tusb320_reg[i], i+0x00);
        printk("[0x%x]=0x%x \n", i, tusb320_reg[i]);        
    }

    tusb320_i2c_read(tusb_i2c_data->client, &tusb320_reg[11], 0x45);
    printk("[0x45]=0x%x \n", tusb320_reg[11]); 
    printk("\n");
#endif

}


#define CURRENT_MODE_ADVERTISE_MASK	0x3
#define CURRENT_MODE_ADVERTISE_SHIFT	6
#define CURRENT_MODE_ADVERTISE_900MA	0
#define CURRENT_MODE_ADVERTISE_1500MA	1
#define CURRENT_MODE_ADVERTISE_3000MA	2

static void tusb320_set_current_mode_adv(int mode)
{
	int ret;
	u8 current_mode;

	if(mode > CURRENT_MODE_ADVERTISE_3000MA)
		mode = CURRENT_MODE_ADVERTISE_3000MA;

	tusb320_i2c_read(tusb_i2c_data->client, &current_mode, 0x08);

	current_mode &= ~(CURRENT_MODE_ADVERTISE_MASK << CURRENT_MODE_ADVERTISE_SHIFT);
	current_mode |= mode << CURRENT_MODE_ADVERTISE_SHIFT;

	ret = tusb320_i2c_write(tusb_i2c_data->client,0x08, current_mode);
	if(ret < 0)
	{
		printk("%s, set 0x08 fail, ret=%d\n", __func__, ret);
	}
	else
		printk("%s,  reg(0x08)=0x%x\n", __func__, current_mode);
}

u8 try_snk_attemp =0;
static void tusb_trysnk_work(struct work_struct *work)
{
    try_snk_attemp = 0x00;
}

static void tusb_eint_work(struct work_struct *work)
{
	u8 interruput_value = 0;
	u8 attached_value = 0;
	u8 csr_reg = 0;
	int cable_connect = 0;

	tusb320_i2c_write(tusb_i2c_data->client, 0x09, 0x10);
	tusb320_i2c_read(tusb_i2c_data->client,&interruput_value,0x09);

	if(interruput_value & 0xc0)
		cable_connect = 1;
	else 
		cable_connect = 0;

	attached_value = (interruput_value >> 6);

	tusb320_i2c_read(tusb_i2c_data->client,&csr_reg,0x08);

	printk("tusb320 attached value:%x try_snk_attemp%x cable_connect=%d,csr_reg=0x%x\n",attached_value ,try_snk_attemp, cable_connect, csr_reg);
	if(cable_connect)
	{
		if (attached_value == 0x01) //DFP. Perform Try.SNK.
		{
			if(try_snk_attemp==0)
			{
				printk("tusb320 DFP mode set try snk\n");
				try_snk_attemp = 0x01; //Set Try_Snk flag.
				tusb320_i2c_write(tusb_i2c_data->client,0x0A, 0x08); //Set Soft Reset
				mdelay(25); // Wait 25ms.
				tusb320_i2c_write(tusb_i2c_data->client,0x0A, 0x08); // Set Soft Reset
				schedule_delayed_work(&tusb_i2c_data->trysnk_work,500);
				tusb320_set_current_mode_adv(CURRENT_MODE_ADVERTISE_900MA);
				tusb320_i2c_write(tusb_i2c_data->client,0x09, 0x06); // Change DRP duty Cycle
			}
		}
		else if (attached_value == 0x02) //DFP. Perform Try.SNK.
		{
			tusb320_set_current_mode_adv(CURRENT_MODE_ADVERTISE_1500MA);
			printk("tusb320 UFP mode\n");
		}
		else if((attached_value == 0x03) && (((csr_reg & 0x0E) >> 1) == 0x06))
		{
			printk("tusb320 accessory mode\n");
			mt_set_gpio_out(175, 0);
			return;
		}
		mt_set_gpio_out(175, 1);
	}
	else
	{
		try_snk_attemp = 0;
		tusb320_set_current_mode_adv(CURRENT_MODE_ADVERTISE_1500MA);
		mt_set_gpio_out(175, 1);
	}
}

static irqreturn_t cc_eint_interrupt_handler(int irq,void *data)
{
	schedule_work(&tusb_i2c_data->eint_work);
	return IRQ_HANDLED;
}

static int tusb320_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct tusb320_i2c_data *fusb;
    u8 device_id_0,device_id_1;
    int ret;
    
    tusb320_power_onoff(1);
        
	fusb = kzalloc(sizeof(struct tusb320_i2c_data), GFP_KERNEL);
	if (!fusb) {
		dev_err(&i2c->dev, "private data alloc fail\n");
        return -1;
	}
    
    tusb_i2c_data = fusb;
	i2c_set_clientdata(i2c, fusb);
	fusb->client = i2c;
    
    ret=tusb320_i2c_read(fusb->client,&device_id_0,0x00);
    ret=tusb320_i2c_read(fusb->client,&device_id_1,0x01);
    if((device_id_0!=0x30)|| (device_id_1!=0x32))
    {
        dev_err(&i2c->dev, "tubs320 do not attached\n");
        goto exit;
    }

    #ifdef USE_EARLY_SUSPEND
	fusb->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	fusb->early_drv.suspend  = tusb320_early_suspend,
	fusb->early_drv.resume   = tusb320_late_resume,    
	register_early_suspend(&fusb->early_drv);
    #endif 

    tusb320_set_current_mode_adv(CURRENT_MODE_ADVERTISE_1500MA);

    INIT_WORK(&tusb_i2c_data->eint_work, tusb_eint_work);
    INIT_DELAYED_WORK(&tusb_i2c_data->trysnk_work, tusb_trysnk_work);

    ret = request_irq(mt_gpio_to_irq(CUST_EINT_CC_DECODER_NUM), cc_eint_interrupt_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "tusb320-eint", fusb);

    schedule_work(&tusb_i2c_data->eint_work);//insert cable power on.

    return 0;
    
exit:
	kfree(fusb);
    tusb_i2c_data = NULL;
    return -1;
}

static int tusb320_remove(struct i2c_client *i2c)
{
	i2c_unregister_device(i2c);
	kfree(i2c_get_clientdata(i2c));

    tusb320_power_onoff(0);

    return 0;
}

static const struct i2c_device_id tusb320_id[] = {
	{ TUSB320_I2C_NAME, 0 },
	{ }
};

static struct i2c_board_info __initdata tusb320_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO(TUSB320_I2C_NAME, (0x60)),
	},
};

static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = TUSB320_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= tusb320_probe,
	.remove		= tusb320_remove,
	#if !defined(USE_EARLY_SUSPEND)    
    .suspend            = tusb320_suspend,
    .resume             = tusb320_resume,
    #endif
	.id_table	= tusb320_id,
};
static int __init tusb320_i2c_init(void)
{
	i2c_register_board_info(3, tusb320_i2c_boardinfo, 
				ARRAY_SIZE(tusb320_i2c_boardinfo)); 
	return i2c_add_driver(&tusb320_i2c_driver);
}

static void __exit tusb320_i2c_exit(void)
{
	i2c_del_driver(&tusb320_i2c_driver);
}

module_init(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);
