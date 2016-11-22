/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include "bq2589x_reg.h"
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <mach/battery_common.h>
#include <mach/battery_meter.h>
#include <linux/earlysuspend.h>

#define CHARGING_CURRENT_LIMIT_TMEP_42 42000
#define CHARGING_CURRENT_LIMIT_TMEP_45 45000
#define CHARGING_CURRENT_LIMIT_TMEP_50 50000

#define CURRENT_MODE_HIGH 3

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
    BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_part_no{
    BQ25890 = 0x03,
    BQ25892 = 0x00,
    BQ25895 = 0x07,
};
/*extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flag, void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);*/
#define CHARGING_VOLTAGE 4400

#define  ICO_INIT_STATE                  		0x1000
#define  ICO_ISSUE_STATE                  		0x1001
#define  ICO_DONE_STATE                  		0x1002
#define  ICO_FAIL_STATE                  		0x1003
#define  ICO_FORCE_RETRY_STATE                  0x1004
#define  ICO_REG_RETRY_STATE                  	0x1005
#define  ICO_INCREASE_CURRENT_STATE				0x1006
#define  ICO_SETUP_TABLE_STATE                  0x1007
#define  ICO_CV_VOLTAGE	(CHARGING_VOLTAGE * 97)/100

#define BQ2589X_STATUS_PLUGIN      	0x0001    //plugin
#define BQ2589X_STATUS_PG          	0x0002    //power good
//#define BQ2589X_STATUS_CHG_DONE    	0x0004
#define BQ2589X_STATUS_FAULT    	0x0008
#define BQ2589X_HVDCP_DCP_EXCHANGE    	0x0010

#define BQ2589X_STATUS_EXIST		0x0100
#define BQ2589X_STATUS_CHARGE_ENABLE 0x0200
struct bq2589x {
	struct device *dev;
	struct i2c_client *client;
    
    enum   bq2589x_part_no part_no;
    int    revision;

    unsigned int    status;  //charger status:
	int		vbus_type;		//

	bool 	enabled;
	
    bool    interrupt;
	bool	use_absolute_vindpm;
	
	int		charge_volt;
	int		charge_current;

    int     vbus_volt;
    int     vbat_volt;
    int	    ico_value;

    int     vbus_volt_high_level;// voltage level that expect adapter output after tune up
    int     vbus_volt_low_level;// voltage level that expect adapter output after tune down
    int     vbat_min_volt_to_tuneup;// battery minimum voltage to tune up adapter

    int     rsoc;
	struct work_struct irq_work;
	struct work_struct adapter_in_work;
	struct work_struct adapter_out_work;
	struct delayed_work monitor_work;
	struct delayed_work ico_work;
	struct delayed_work volt_tune_work;
	struct delayed_work check_to_tuneup_work;
	struct delayed_work charger2_enable_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	struct mutex lock;
	struct power_supply usb;
	struct power_supply wall;

	wait_queue_head_t wq;
	int temp_init;
	int disable_quick_charge;
	
};

struct volt_control_t{
	bool TuneVoltwithPEplus;
	int TuneTargetVolt;
	bool toTuneDownVolt;
	bool toTuneUpVolt;
	bool TuneVoltDone;
	int TuneCounter;
	bool TuneFail;
};

static struct bq2589x *g_bq1;
static struct bq2589x *g_bq2;
static struct volt_control_t voltcontrol;
static int vbus_voltage_flag = 0;
static void bq2589x_high_dcp_set(int enable);
static int jeita_current, jeita_input_limit;
static int screen_on = 1;
static int is_max_charge = 0;
static int first_retry =0;
static int last_temp_level = 0;
static int ico_state = ICO_INIT_STATE;
void bq2589x_dump_register(struct bq2589x* bq);

//static struct task_struct *charger_thread;


static DEFINE_MUTEX(bq2589x_i2c_lock);
static DEFINE_MUTEX(suspend_lock);

extern int tusb_check_type_cable(void);
extern int g_platform_boot_mode;
extern int g_battery_thermal_throttling_flag;
static int is_power_off_charging(void)
{
	if (g_platform_boot_mode == 8 || g_platform_boot_mode == 9)
		return 1;
	else 
		return 0;
}

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
    int      ret=0;

	mutex_lock(&bq2589x_i2c_lock);
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]failed to read 0x%.2x\n", reg);
		mutex_unlock(&bq2589x_i2c_lock);
		return ret;
	}
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
   /* bq->client->ext_flag=((bq->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = reg;
    ret = i2c_master_send(bq->client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        bq->client->ext_flag=0;

        mutex_unlock(&bq2589x_i2c_lock);
        return 0;
    }
    
    readData = cmd_buf[0];
    *data = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    bq->client->ext_flag=0;*/


    *data = (u8)ret;
	mutex_unlock(&bq2589x_i2c_lock);
	
	return 0;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
    int     ret=0;

	mutex_lock(&bq2589x_i2c_lock);	
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
   /* write_data[0] = reg;
    write_data[1] = data;
    
    bq->client->ext_flag=((bq->client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(bq->client, write_data, 2);
    if (ret < 0) 
    {
       
        bq->client->ext_flag=0;
        mutex_unlock(&bq2589x_i2c_lock);
        return 0;
    }
    
    bq->client->ext_flag=0;*/

	mutex_unlock(&bq2589x_i2c_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

	if(bq == NULL)
		return 0;

	ret = bq2589x_read_byte(bq, &tmp, reg);
	
	if (ret)	
		return ret;

	tmp &= ~mask;
	tmp |= data & mask;
	
	return bq2589x_write_byte(bq, reg, tmp);
}


#define TEMP_CURRENT_TABLE_LENGTH 6
#define LEVEL0_LIMIT    200000
#define LEVEL1_LIMIT    150000
#define LEVEL2_LIMIT    100000
#define LEVEL3_LIMIT     80000

#ifdef LEECO_INDIA_CHARGE_LIMIT
static int screen_off_current_table[TEMP_CURRENT_TABLE_LENGTH][5] = {
	{30000, 450000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{33000, 400000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{36000, 350000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{38000, 300000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{40000, 200000, 0, LEVEL1_LIMIT, LEVEL1_LIMIT},
	{44000,  50000, 0, LEVEL2_LIMIT, LEVEL2_LIMIT},
};

static int screen_on_current_table[TEMP_CURRENT_TABLE_LENGTH][5] = {
	{30000, 150000, 0, LEVEL1_LIMIT, LEVEL1_LIMIT},
	{33000, 150000, 0, LEVEL1_LIMIT, LEVEL1_LIMIT},
	{36000, 100000, 0, LEVEL2_LIMIT, LEVEL2_LIMIT},
	{39000,  90000, 0, LEVEL2_LIMIT, LEVEL2_LIMIT},
	{41000,  80000, 0, LEVEL3_LIMIT, LEVEL3_LIMIT},
	{44000,  50000, 0, LEVEL3_LIMIT, LEVEL3_LIMIT},
};
#else
static int screen_off_current_table[TEMP_CURRENT_TABLE_LENGTH][5] = {
	{30000, 450000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{33000, 400000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{36000, 350000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{40000, 300000, 1, LEVEL0_LIMIT, LEVEL0_LIMIT},
	{44000, 200000, 0, LEVEL1_LIMIT, LEVEL1_LIMIT},
	{46000,  50000, 0, LEVEL2_LIMIT, LEVEL2_LIMIT},
};

static int screen_on_current_table[TEMP_CURRENT_TABLE_LENGTH][5] = {
	{30000, 150000, 0, LEVEL1_LIMIT, LEVEL1_LIMIT},
	{33000, 150000, 0, LEVEL1_LIMIT, LEVEL1_LIMIT},
	{36000, 100000, 0, LEVEL2_LIMIT, LEVEL2_LIMIT},
	{39000,  90000, 0, LEVEL2_LIMIT, LEVEL2_LIMIT},
	{41000,  80000, 0, LEVEL3_LIMIT, LEVEL3_LIMIT},
	{44000,  50000, 0, LEVEL3_LIMIT, LEVEL3_LIMIT},
};
#endif

static void temp_adjust_current_limit_init(int current_limit);
static void temp_adjust_current_limit_dump(void);
static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
    if (ret < 0) return 0;
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}

int bq25890_get_vbus_type_external(void)
{
    if(g_bq1!=NULL)
        return bq2589x_get_vbus_type(g_bq1);
    else 
        return 0; 
}
static int bq2589x_enable_otg(struct bq2589x *bq)
{
    u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
	
}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
    u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_OTG_CONFIG_MASK, val);
	
}
EXPORT_SYMBOL_GPL(bq2589x_disable_otg);

static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt )
{
    u8 val = 0;

    if (volt < BQ2589X_BOOSTV_BASE)
        volt = BQ2589X_BOOSTV_BASE;
    if (volt > BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB)
        volt = BQ2589X_BOOSTV_BASE + (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT) * BQ2589X_BOOSTV_LSB;
    

	val = ((volt - BQ2589X_BOOSTV_BASE)/BQ2589X_BOOSTV_LSB) << BQ2589X_BOOSTV_SHIFT;
	
	return bq2589x_update_bits(bq,BQ2589X_REG_0A,BQ2589X_BOOSTV_MASK,val);

}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);

static int bq2589x_set_otg_current(struct bq2589x *bq, int curr )
{
	u8 temp;

    if(curr  == 500)
        temp = BQ2589X_BOOST_LIM_500MA;
    else if(curr == 700)
        temp = BQ2589X_BOOST_LIM_700MA;
    else if(curr == 1100)
        temp = BQ2589X_BOOST_LIM_1100MA;
    else if(curr == 1600)
        temp = BQ2589X_BOOST_LIM_1600MA;
    else if(curr == 1800)
        temp = BQ2589X_BOOST_LIM_1800MA;
    else if(curr == 2100)
        temp = BQ2589X_BOOST_LIM_2100MA;
    else if(curr == 2400)
        temp = BQ2589X_BOOST_LIM_2400MA;
    else
        temp = BQ2589X_BOOST_LIM_1300MA;

    return bq2589x_update_bits(bq,BQ2589X_REG_0A,BQ2589X_BOOST_LIM_MASK,temp << BQ2589X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);

static int bq2589x_enable_charger(struct bq2589x *bq)
{
    int ret;
    u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

    ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if(ret == 0)
        bq->status |= BQ2589X_STATUS_CHARGE_ENABLE;
    return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
    int ret;
    u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_CHG_CONFIG_MASK, val);
	if(ret == 0)
        bq->status &=~BQ2589X_STATUS_CHARGE_ENABLE;
    return ret;    
}


/* interfaces that can be called by other module */
int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
    u8 val;
    int ret;

    ret = bq2589x_read_byte(bq,&val,BQ2589X_REG_02);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s failed to read register 0x02:%d\n",__func__,ret);
        return ret;
    }

    if(((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
        return 0; //is doing continuous scan
    if(oneshot)
        ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_CONV_START_MASK, BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
    else
        ret = bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
    return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x *bq)//stop continue scan 
{
    return bq2589x_update_bits(bq,BQ2589X_REG_02,BQ2589X_CONV_RATE_MASK, BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);


int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
    uint8_t val;
    int volt;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]read battery voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);


int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
    uint8_t val;
    int volt;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]read system voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
    uint8_t val;
    int volt;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]read vbus voltage failed :%d\n",ret);
        return ret;
    }
    else{
        volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

int bq2589x_adc_read_temperature(struct bq2589x *bq)
{
    uint8_t val;
    int temp;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]read temperature failed :%d\n",ret);
        return ret;
    }
    else{
        temp = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
        return temp;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);

int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
    uint8_t val;
    int volt;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]read charge current failed :%d\n",ret);
        return ret;
    }
    else{ 
        volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
        return volt;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_set_chargecurrent(struct bq2589x *bq,int curr)	
{

	u8 ichg;
    
    battery_xlog_printk(BAT_LOG_CRTI,"[battery]set current = %d\n", curr);
    ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04,BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_term_current(struct bq2589x *bq,int curr)
{
    u8 iterm;

    iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;
    
    return bq2589x_update_bits(bq, BQ2589X_REG_05,BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);


int bq2589x_set_prechg_current(struct bq2589x *bq,int curr)
{
    u8 iprechg;

    iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;
    
    return bq2589x_update_bits(bq, BQ2589X_REG_05,BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);


int bq2589x_set_ircomp(struct bq2589x *bq, int val)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_08,BQ2589X_BAT_COMP_MASK, val << BQ2589X_BAT_COMP_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_ircomp);

int bq2589x_set_vclamp(struct bq2589x *bq, int val)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_08,BQ2589X_VCLAMP_MASK, val << BQ2589X_VCLAMP_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_vclamp);

int bq2589x_set_chargevoltage(struct bq2589x *bq,int volt)
{
	u8 val;

    val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06,BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);

int bq2589x_set_force_vindpm_enable(struct bq2589x *bq,int enable)
{
    return bq2589x_update_bits(bq, BQ2589X_REG_0D,BQ2589X_FORCE_VINDPM_MASK, enable << BQ2589X_FORCE_VINDPM_SHIFT);
}

int bq2589x_set_input_volt_limit(struct bq2589x *bq,int volt)
{
	u8 val;
    val = (volt - BQ2589X_VINDPM_BASE)/BQ2589X_VINDPM_LSB;
    return bq2589x_update_bits(bq, BQ2589X_REG_0D,BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);


int bq2589x_set_input_current_limit_enable(struct bq2589x *bq,int enable)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_00,BQ2589X_ENILIM_MASK, enable << BQ2589X_ENILIM_SHIFT);

}

int bq2589x_set_input_current_limit(struct bq2589x *bq,int curr)
{

	u8 val;

	val = (curr - BQ2589X_IINLIM_BASE)/BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00,BQ2589X_IINLIM_MASK, val << BQ2589X_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);


int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;
	
	val = (offset - BQ2589X_VINDPMOS_BASE)/BQ2589X_VINDPMOS_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_01,BQ2589X_VINDPMOS_MASK, val << BQ2589X_VINDPMOS_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);


void bq2589x_start_charging(struct bq2589x *bq)
{
    bq2589x_enable_charger(bq);
}
EXPORT_SYMBOL_GPL(bq2589x_start_charging);

void bq2589x_stop_charging(struct bq2589x *bq)
{
    bq2589x_disable_charger(bq);
}
EXPORT_SYMBOL_GPL(bq2589x_stop_charging);

int bq2589x_get_charging_status(struct bq2589x *bq)
{
    u8 val = 0;
    int ret;

    ret = bq2589x_read_byte(bq,&val, BQ2589X_REG_0B);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s Failed to read register 0x0b:%d\n",__func__,ret);
        return ret;
    }
    val &= BQ2589X_CHRG_STAT_MASK;
    val >>= BQ2589X_CHRG_STAT_SHIFT;
    return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);


int bq25890_get_charging_status_external(void)
{
    if(g_bq1!=NULL)
    {
        if(bq2589x_get_charging_status(g_bq1)==3)
            return 1;
        else 
            return 0;
    }
    else
    {
        return 0;
    }
}

void bq2589x_set_otg(int enable)
{
    int ret;

    if(g_bq1 == NULL)
	    return;

    if(enable){
        ret = bq2589x_enable_otg(g_bq1);
       
        if(ret < 0){
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to enable otg-%d\n",__func__,ret);
            return;
        }
    }
    else{
        ret = bq2589x_disable_otg(g_bq1);
        if(ret < 0){
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to disable otg-%d\n",__func__,ret);
        }
    }
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg);

int bq2589x_set_watchdog_timer(struct bq2589x *bq,u8 timeout)
{
    return bq2589x_update_bits(bq,BQ2589X_REG_07,BQ2589X_WDT_MASK, (u8)(timeout) << BQ2589X_WDT_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07, BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

void bq2589x_disable_safety_timer(void)
{
	u8 val = BQ2589X_CHG_TIMER_DISABLE << BQ2589X_EN_TIMER_SHIFT;

	if(g_bq1 == NULL)
		return;

	bq2589x_update_bits(g_bq1, BQ2589X_REG_07, BQ2589X_EN_TIMER_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_safety_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03, BQ2589X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_force_dpdm_clean(struct bq2589x *bq)
{
	int ret;
	unsigned char val;

	if(bq == NULL)
		return -1;

	val= ~BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if(ret)
		return ret;

	return 0;
}

int bq2589x_force_dpdm(struct bq2589x *bq)
{
	int ret;
	unsigned char val;

	if(bq == NULL)
		return -1;

	val= BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_FORCE_DPDM_MASK, val);
	if(ret)
		return ret;

	mdelay(10);//TODO: how much time needed to finish dpdm detect?

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);


static int bq2589x_enable_ico(struct bq2589x *bq)
{
    u8 val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
	
}

static int bq2589x_disable_ico(struct bq2589x *bq)
{
    u8 val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);
	
}


static int bq2589x_enable_max(struct bq2589x *bq)
{
    u8 val = BQ2589X_MAXC_ENABLE << BQ2589X_MAXCEN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_MAXCEN_MASK, val);
	
}

static int bq2589x_disable_max(struct bq2589x *bq)
{
    u8 val = BQ2589X_MAXC_DISABLE<< BQ2589X_MAXCEN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_MAXCEN_MASK, val);
	
}

static int bq2589x_enable_hvdcp(struct bq2589x *bq)
{
    u8 val = BQ2589X_HVDCP_ENABLE << BQ2589X_HVDCPEN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_HVDCPEN_MASK, val);
	
}

static int bq2589x_disable_hvdcp(struct bq2589x *bq)
{
    u8 val = BQ2589X_HVDCP_DISABLE<< BQ2589X_HVDCPEN_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_HVDCPEN_MASK, val);
	
}


void bq2589x_set_ico(struct bq2589x *bq,int enable)
{
    int ret;
    if(enable){
        ret = bq2589x_enable_ico(bq);
        if(ret < 0){
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to enable otg-%d\n",__func__,ret);
            return;
        }
    }
    else{
        ret = bq2589x_disable_ico(bq);
        if(ret < 0){
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to disable otg-%d\n",__func__,ret);
        }
    }

}
EXPORT_SYMBOL_GPL(bq2589x_set_ico);

int bq2589x_ts_set(struct bq2589x *bq, u8 ts_enable)
{
    int ret;
	
	u8 val = ts_enable << BQ2589X_TS_PROFILE_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_TS_PROFILE_MASK, val);
    return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_ts_set);

int bq2589x_reset_chip(struct bq2589x *bq)
{
    int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14, BQ2589X_RESET_MASK, val);
    return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_ship_mode(struct bq2589x *bq)
{
    int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
    return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);

void bq2589x_enter_ship_mode_external(void)
{
    if(g_bq1!=NULL)
        bq2589x_enter_ship_mode(g_bq1);
    if(g_bq2!=NULL)
        bq2589x_enter_ship_mode(g_bq2);
}

int bq2589x_exit_ship_mode(struct bq2589x *bq)
{
    int ret;
	u8 val = BQ2589X_BATFET_ON << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DIS_MASK, val);
    return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_exit_ship_mode);

int bq2589x_enable_batfet_delay(struct bq2589x *bq)
{
    int ret;
    
	u8 val = BQ2589X_BATFET_DLY_ON << BQ2589X_BATFET_DLY_SHIFT;
	ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_BATFET_DLY_MASK, val);

    
    return ret;
}
int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x *bq,u8* state)
{
    u8 val;
    int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret) return ret;
    *state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;
	
    return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

static void bq2589x_high_dcp_set(int enable)
{
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s enable = %d , vbus_voltage_flag = %d\n", __func__, enable, vbus_voltage_flag);
	if(enable && vbus_voltage_flag)
	{
		bq2589x_enable_max(g_bq1);
		bq2589x_enable_hvdcp(g_bq1);
		mdelay(10);
		bq2589x_force_dpdm(g_bq1);
		bq2589x_set_force_vindpm_enable(g_bq1,1);

		vbus_voltage_flag = 0;
		g_bq1->status |= BQ2589X_HVDCP_DCP_EXCHANGE;

	}
	else if(!vbus_voltage_flag && !enable)
	{
		bq2589x_disable_max(g_bq1);
		bq2589x_disable_hvdcp(g_bq1);
		mdelay(10);
		bq2589x_force_dpdm(g_bq1);
		bq2589x_set_force_vindpm_enable(g_bq1,1);
		bq2589x_set_input_volt_limit(g_bq1,4600);
		if(g_bq2!=NULL)
			bq2589x_enter_hiz_mode(g_bq2);

		vbus_voltage_flag = 1;
 		g_bq1->status |= BQ2589X_HVDCP_DCP_EXCHANGE;
	}
}


int bq2589x_pumpx_enable(struct bq2589x *bq,int enable)
{
    u8 val;
    int ret;

    if(enable)
        val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
    else
        val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

    ret = bq2589x_update_bits(bq, BQ2589X_REG_04, BQ2589X_EN_PUMPX_MASK, val);
    
    return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);

int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
    u8 val;
    int ret;

    val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

    ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_UP_MASK, val);
    
    return ret;
   
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);

int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
    u8 val;
    int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
    if(ret) return ret;

    if(val & BQ2589X_PUMPX_UP_MASK) 
        return 1;   // not finished
    else
        return 0;   // pumpx up finished
   
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);

int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
    u8 val;
    int ret;

    val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

    ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_PUMPX_DOWN_MASK, val);
    
    return ret;
   
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);

int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
    u8 val;
    int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
    if(ret) return ret;

    if(val & BQ2589X_PUMPX_DOWN_MASK) 
        return 1;   // not finished
    else
        return 0;   // pumpx down finished
   
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

static int bq2589x_force_ico(struct bq2589x* bq)
{
    u8 val;
    int ret;

    val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

    ret = bq2589x_update_bits(bq, BQ2589X_REG_09, BQ2589X_FORCE_ICO_MASK, val);
    
    return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_force_ico);

static int bq2589x_check_force_ico_done(struct bq2589x* bq)
{
    u8 val;
    int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);

    battery_xlog_printk(BAT_LOG_CRTI,"[battery]bq2589x_check_force_ico_done :%d value(%x)\n",ret,val);
    
    if(ret) return ret;

    if(val & BQ2589X_ICO_OPTIMIZED_MASK) 
        return 1;  //finished
    else
        return 0;   // in progress
}
EXPORT_SYMBOL_GPL(bq2589x_check_force_ico_done);

static int bq2589x_read_idpm_limit(struct bq2589x* bq)
{
    uint8_t val;
    int curr;
    int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]read vbus voltage failed :%d\n",ret);
        return ret;
    }
    else{
        curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
        return curr;
    }
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);

static bool bq2589x_is_charge_done(struct bq2589x* bq)
{
    int ret;
    u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:read REG0B failed :%d\n",__func__,ret);
        return false;
    }
    val &= BQ2589X_CHRG_STAT_MASK;
    val >>= BQ2589X_CHRG_STAT_SHIFT;

    return(val == BQ2589X_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);


void bq2589x_charging_hw_inputcurrent_limit_external(int value)
{
    if(g_bq1!=NULL)
    {
        if(g_bq1->status & BQ2589X_STATUS_PLUGIN)
	{
                bq2589x_set_input_current_limit(g_bq1,value/100);
                bq2589x_set_input_current_limit(g_bq2,value/200);
	}
    }
}

void bq2589x_charging_set_jeita_current(int charge_current)
{
	jeita_current = charge_current;
}

void bq2589x_charging_set_jeita_input_limit(int input_limit)
{
	jeita_input_limit = input_limit;
}

void bq25890_charging_curernt_set_external(int value)
{
     if(g_bq1 == NULL)
	     return;

     battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s set current =%d, vbus_voltage_flag= 0x%x\n", __func__, value, vbus_voltage_flag);

     if(g_bq1->status & BQ2589X_STATUS_PLUGIN) {
         if(((value-225000)>0) && (!vbus_voltage_flag))
         {
		if((g_bq2!=NULL)&&(g_bq2->enabled==true))
		{
			bq2589x_set_chargecurrent(g_bq1, value / 200);
			bq2589x_set_chargecurrent(g_bq2, value / 200);
			bq2589x_enable_charger(g_bq2);
		}
         }
         else
         {
		if(vbus_voltage_flag && value > 160000)
			value = 160000;

		if((g_bq2!=NULL)&&(g_bq2->enabled==true))
			bq2589x_disable_charger(g_bq2);

		if(g_bq1!=NULL)
			bq2589x_set_chargecurrent(g_bq1,value/100);  
             
         }
    }
}

void bq2589x_dump_register(struct bq2589x* bq)
{
    int i=0;
    kal_uint8 bq2589x_reg[21] = {0};

    battery_xlog_printk(BAT_LOG_CRTI,"[battery][bq2589x] ");

#if 1
    for (i=0;i<21;i++)
    {
        bq2589x_read_byte(bq, &bq2589x_reg[i], i+0x00);
        battery_xlog_printk(BAT_LOG_CRTI,"[battery][0x%x]=0x%x \n", i, bq2589x_reg[i]);        
    }
    battery_xlog_printk(BAT_LOG_CRTI,"[battery]\n");
#endif

}


static int bq2589x_init_device(struct bq2589x *bq)
{
	
    int ret;

	ret = bq2589x_set_chargecurrent(bq,512);  
	if(ret < 0){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to set charger1 charge current:%d\n",__func__,ret);
		return ret;
	}

	ret = bq2589x_set_chargevoltage(bq,CHARGING_VOLTAGE);
	if(ret < 0){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to set charge voltage:%d\n",__func__,ret);
		return ret;
	}

    ret = bq2589x_set_vindpm_offset(bq,600);
	if(ret < 0){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to set vindpm offset:%d\n",__func__,ret);
		return ret;
	}


    bq2589x_ts_set(bq, 0);

    if(bq == g_bq1){//charger 1 specific initialization
        //bq2589x_adc_start(bq,false);
        bq2589x_set_ico(bq,1);

        ret = bq2589x_pumpx_enable(bq,0);
        if(ret){
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to enable pumpx:%d\n",__func__,ret);
            return ret;
        }
        
    	ret = bq2589x_set_term_current(bq,150);
    	
    	if(ret < 0){
    		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to set termination current:%d\n",__func__,ret);
    		return ret;
    	}
    	bq2589x_set_watchdog_timer(bq, BQ2589X_WDT_40S);
    }
    else if(bq == g_bq2){//charger2 specific initialization
        //bq2589x_adc_start(bq,false);
	ret = bq2589x_disable_charger(bq);
	if(ret < 0){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to enable charger:%d\n",__func__,ret);
		return ret;
	}
        bq2589x_set_ico(bq,0);
        
    	ret = bq2589x_set_term_current(bq, 150);
    	if(ret < 0){
    		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to set termination current:%d\n",__func__,ret);
    		return ret;
    	}

        ret = bq2589x_enter_hiz_mode(bq);//disabled by default
		if(ret < 0){
			battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to enter hiz charger 2:%d\n",__func__,ret);
			return ret;
		}

	bq2589x_disable_watchdog_timer(bq);
      
    }

    return ret;
}


static int bq2589x_charge_status(struct bq2589x * bq)
{
    u8 val = 0;

    bq2589x_read_byte(bq,&val, BQ2589X_REG_0B);
    val &= BQ2589X_CHRG_STAT_MASK;
    val >>= BQ2589X_CHRG_STAT_SHIFT;
    switch(val){
        case BQ2589X_CHRG_STAT_FASTCHG:
            return POWER_SUPPLY_CHARGE_TYPE_FAST;
        case BQ2589X_CHRG_STAT_PRECHG:
            return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
        case BQ2589X_CHRG_STAT_CHGDONE:
        case BQ2589X_CHRG_STAT_IDLE:
            return POWER_SUPPLY_CHARGE_TYPE_NONE;
        default:
            return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
    }
}

static enum power_supply_property bq2589x_charger_props[] = {
	POWER_SUPPLY_PROP_CHARGE_TYPE, /* Charger status output */
	POWER_SUPPLY_PROP_ONLINE, /* External power source */
	POWER_SUPPLY_PROP_DISABLE_QUICK_CHARGE,
};


static int bq2589x_usb_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{

    struct bq2589x *bq = container_of(psy, struct bq2589x, usb);
    u8 type = bq2589x_get_vbus_type(bq);
    
    switch(psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if(type == BQ2589X_VBUS_USB_SDP || type == BQ2589X_VBUS_USB_DCP)
            val->intval = 1;
        else
            val->intval = 0;
        break;
    case POWER_SUPPLY_PROP_CHARGE_TYPE:
        val->intval = bq2589x_charge_status(bq);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int bq2589x_wall_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{

	struct bq2589x *bq = container_of(psy, struct bq2589x, wall);
	u8 type = bq2589x_get_vbus_type(bq);

	switch(psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if(type == BQ2589X_VBUS_MAXC || type == BQ2589X_VBUS_UNKNOWN || type == BQ2589X_VBUS_NONSTAND)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_charge_status(bq);
		break;
	case POWER_SUPPLY_PROP_DISABLE_QUICK_CHARGE:
		val->intval = bq->disable_quick_charge;
		break;
	default:
        	return -EINVAL;
    }

    return 0;
}
static int bq2589x_wall_set_property(struct power_supply *psy,
		enum power_supply_property psp,
		const union power_supply_propval *val)
{
	struct bq2589x *bq = container_of(psy, struct bq2589x, wall);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_DISABLE_QUICK_CHARGE:
		bq->disable_quick_charge = val->intval;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int bq2589x_wall_property_is_writeable(struct power_supply *psy,
		enum power_supply_property psp)
{
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_DISABLE_QUICK_CHARGE:
		ret = 1;
		break;
	default:
		ret = 0;
	}

	return ret;
}

static int bq2589x_psy_register(struct bq2589x *bq)
{
    int ret;

    bq->usb.name = "bq2589x-usb";
    bq->usb.type = POWER_SUPPLY_TYPE_USB;
    bq->usb.properties = bq2589x_charger_props;
    bq->usb.num_properties = ARRAY_SIZE(bq2589x_charger_props);
    bq->usb.get_property = bq2589x_usb_get_property;
    bq->usb.external_power_changed = NULL;

    ret = power_supply_register(bq->dev, &bq->usb);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:failed to register usb psy:%d\n",__func__,ret);
        return ret;
    }


    bq->wall.name = "bq2589x-Wall";
    bq->wall.type = POWER_SUPPLY_TYPE_MAINS;
    bq->wall.properties = bq2589x_charger_props;
    bq->wall.num_properties = ARRAY_SIZE(bq2589x_charger_props);
    bq->wall.get_property = bq2589x_wall_get_property;
    bq->wall.set_property = bq2589x_wall_set_property;
    bq->wall.property_is_writeable = bq2589x_wall_property_is_writeable;
    bq->wall.external_power_changed = NULL;

    ret = power_supply_register(bq->dev, &bq->wall);
    if(ret < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:failed to register wall psy:%d\n",__func__,ret);
        goto fail_1;
    }
    
    return 0;
    
fail_1:
    power_supply_unregister(&bq->usb);
    
    return ret;
}

static void bq2589x_psy_unregister(struct bq2589x *bq)
{
    power_supply_unregister(&bq->usb);
    power_supply_unregister(&bq->wall);
}

//echo "0x88,0x99" > sys/devices/bus.2/11009000.I2C2/i2c-2/2-006a/registers
//write reg 0x88 value 0x99
static ssize_t bq2589x_store_registers(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{

	int len = 0;
	int desc[32];
	len = (size < (sizeof(desc) - 1)) ? size : (sizeof(desc) - 1);

	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s len = %d\n", __func__, len);
	sscanf(buf, "%x,%x", &desc[0], &desc[1]);

	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s addr = 0x%x, value = 0x%x\n", __func__, desc[0], desc[1]);
	bq2589x_write_byte(g_bq1, desc[0], desc[1]);

	return size; 
}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;
	
	idx = sprintf(buf,"%s:\n","Charger 1");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq1, &val, addr);
		if(ret == 0){
			len = sprintf(tmpbuf,"Reg[0x%.2x] = 0x%.2x\n",addr,val);
			memcpy(&buf[idx],tmpbuf,len);
			idx += len;
		}
	}

	idx += sprintf(&buf[idx],"%s:\n","Charger 2");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(g_bq2, &val, addr);
		if(ret == 0){
			len = sprintf(tmpbuf,"Reg[0x%.2x] = 0x%.2x\n",addr,val);
			memcpy(&buf[idx],tmpbuf,len);
			idx += len;
		}
	}

	return idx;
}

int test_current = 400000;
static ssize_t bq2589x_store_current(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	sscanf(buf, "%d", &test_current);
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s test_current = %d\n", __func__, test_current);
	bq25890_charging_curernt_set_external(test_current);

	return size; 
}

static ssize_t bq2589x_show_current(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int idx = 0;	
	idx = sprintf(buf,"test_current:%d\n", test_current);
	return idx;
}

int test_temp = 0;
static ssize_t bq2589x_store_temp(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{	
	sscanf(buf, "%d", &test_temp);
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s test_temp = %d\n", __func__, test_temp);

	return size;
}

static ssize_t bq2589x_show_temp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int idx = 0;	
	idx = sprintf(buf,"test_temp:%d\n", test_temp);
	return idx;
}

int bq2589x_cmd_set_hiz = 0;
static ssize_t bq2589x_store_charger_disable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{	
	sscanf(buf, "%d", &bq2589x_cmd_set_hiz);
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s bq2589x_cmd_set_hiz= %d\n", __func__, bq2589x_cmd_set_hiz);

	if(bq2589x_cmd_set_hiz)
	{
    		bq2589x_adc_stop(g_bq1);
    		bq2589x_adc_stop(g_bq2);
    		bq2589x_enter_hiz_mode(g_bq1);
		bq2589x_disable_watchdog_timer(g_bq1);
		bq2589x_disable_watchdog_timer(g_bq2);
	}
	else
	{
    		bq2589x_adc_start(g_bq1, false);
    		bq2589x_adc_start(g_bq2, false);
    		bq2589x_exit_hiz_mode(g_bq1);
	}

	return size;
}

static ssize_t bq2589x_show_charger_disable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int idx = 0;	
	idx = sprintf(buf,"bq2589x_cmd_set_hiz:%d\n", bq2589x_cmd_set_hiz);
	return idx;
}

static ssize_t bq2589x_show_vbus(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int idx = 0;	
	idx = sprintf(buf,"%d\n",bq2589x_adc_read_vbus_volt(g_bq1));
	return idx;
}

static DEVICE_ATTR(registers, 0660, bq2589x_show_registers, bq2589x_store_registers);
static DEVICE_ATTR(vbus, 0440, bq2589x_show_vbus, NULL);
static DEVICE_ATTR(value, 0660, bq2589x_show_current, bq2589x_store_current);
static DEVICE_ATTR(temp, 0660, bq2589x_show_temp, bq2589x_store_temp);
static DEVICE_ATTR(charger_disable, 0660, bq2589x_show_charger_disable, bq2589x_store_charger_disable);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
    &dev_attr_vbus.attr,
    &dev_attr_value.attr,
    &dev_attr_temp.attr,
    &dev_attr_charger_disable.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};


static int bq2589x_parse_dt(struct device *dev, struct bq2589x * bq)
{
    int ret;
    struct device_node *np = dev->of_node;

    ret = of_property_read_u32(np,"ti,bq2589x,vbus-volt-high-level",&bq->vbus_volt_high_level);
    if(ret) return ret;

    ret = of_property_read_u32(np,"ti,bq2589x,vbus-volt-low-level",&bq->vbus_volt_low_level);
    if(ret) return ret;

    ret = of_property_read_u32(np,"ti,bq2589x,vbat-min-volt-to-tuneup",&bq->vbat_min_volt_to_tuneup);
    if(ret) return ret;
 
    return 0;   
}

static int bq2589x_detect_device(struct bq2589x* bq)
{
    int ret;
    u8 data;

    ret = bq2589x_read_byte(bq,&data,BQ2589X_REG_14);
    battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:data:%x\n",__func__,data);
    if(ret == 0){
        bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
        bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
    }

    return ret;
}

static void bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt;
	u16 vindpm_volt;
	int ret;
	
	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if(vbus_volt < 6000)
		vindpm_volt = vbus_volt - 600;
	else	
		vindpm_volt = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(bq,vindpm_volt);
	if(ret < 0)
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Set absolute vindpm threshold %d Failed:%d\n",__func__,vindpm_volt,ret);
	else
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Set absolute vindpm threshold %d successfully\n",__func__,vindpm_volt);
	
}

static int bq2589x_set_dpdm(struct bq2589x *bq, int enable)
{
	int ret;
	u8 val;

	val = enable << BQ2589X_AUTO_DPDM_EN_SHIFT;
	ret = bq2589x_update_bits(g_bq1, BQ2589X_REG_02, BQ2589X_AUTO_DPDM_EN_MASK, val);
	if(ret)
		return ret;

	return 0;
}

int bq2589x_dpdm_enable(void)
{
	if(g_bq1 == NULL)
		return -1;

	bq2589x_set_dpdm(g_bq1, true);

	return 0;
}

int bq2589x_auto_dpdm_detect(int mtk_charger_type, int enable)
{
	static int bq2589x_dpdm_detect = 0;
	u8 status;

	if(g_bq1 == NULL || enable == bq2589x_dpdm_detect)
		return 0;

	bq2589x_read_byte(g_bq1, &status, BQ2589X_REG_0B);
	if(!(status & BQ2589X_PG_STAT_MASK) && mtk_charger_type != CHARGER_UNKNOWN)
	{
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:power not good, status=0x%x\n",__func__,status);
		return 0;
	}

	if(mtk_charger_type == CHARGER_UNKNOWN)
	{
		bq2589x_exit_hiz_mode(g_bq1);
		bq2589x_enable_charger(g_bq1);
		bq2589x_force_dpdm_clean(g_bq1);
		bq2589x_dpdm_detect = 0;
	}
	else if((mtk_charger_type == CHARGING_HOST) || ( mtk_charger_type == STANDARD_HOST))
	{
		if(mtk_charger_type == CHARGING_HOST)
			g_bq1->vbus_type = BQ2589X_VBUS_USB_CDP;
		else
			g_bq1->vbus_type = BQ2589X_VBUS_USB_SDP;

		bq2589x_dpdm_detect = 1;
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:adapter plugged in\n",__func__);
		g_bq1->status |= BQ2589X_STATUS_PLUGIN;
		schedule_work(&g_bq1->adapter_in_work);
	}
	else
	{
		bq2589x_set_dpdm(g_bq1, enable);
		bq2589x_force_dpdm(g_bq1);
		bq2589x_dpdm_detect = 1;
		g_bq1->status &= ~BQ2589X_STATUS_PLUGIN;
	}
	
	battery_xlog_printk(BAT_LOG_CRTI, "%s mtk_charger_type = %d enable = %d, bq2589x_dpdm_detect=%d\n", __func__, mtk_charger_type, enable, bq2589x_dpdm_detect);
	bq2589x_dump_register(g_bq1);
	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_auto_dpdm_detect);

static void bq2589x_adapter_in_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_in_work);
	int ret;
	
    if(bq->vbus_type != BQ2589X_VBUS_OTG)
    {
    	bq2589x_adc_start(g_bq1,false);
        if(g_bq2!=NULL)
        	bq2589x_adc_start(g_bq2,false);
    }
    bq2589x_init_device(g_bq1);
    if(g_bq2!=NULL)
    {
    	bq2589x_init_device(g_bq2);
    	ret = bq2589x_enter_hiz_mode(g_bq2);
    	if(ret < 0)
    		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: Charger 2 enter hiz mode failed\n",__func__);
    	else{
    		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Charger 2 enter Hiz mode successfully\n",__func__);
    		g_bq2->enabled = false;
    	}
    }

	if(bq->vbus_type == BQ2589X_VBUS_MAXC){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:HVDCP or Maxcharge adapter plugged in\n",__func__);

		is_max_charge = 1;

		if( ico_state == ICO_DONE_STATE || ico_state == ICO_INIT_STATE)
			schedule_delayed_work(&bq->ico_work, 2*HZ);
	}
	else if(bq->vbus_type == BQ2589X_VBUS_USB_DCP){// DCP, let's check if it is PE adapter
		//normal adapter in set charging current 2A
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:usb dcp adapter plugged in\n",__func__);
		//bq2589x_dump_register(bq);
		//schedule_delayed_work(&bq->check_to_tuneup_work,0); do not support MTK PE
		bq2589x_set_input_current_limit(g_bq1, 1600);
		bq2589x_set_chargecurrent(g_bq1, 1600);  
	}
	else if(bq->vbus_type == BQ2589X_VBUS_USB_SDP){
		//usb DRP port set charging current 500.
		//schedule_delayed_work(&bq->ico_work,0);
		bq2589x_set_chargecurrent(g_bq1,512);
	}
	else if(bq->vbus_type == BQ2589X_VBUS_USB_CDP){
		//usb DRP port set charging current 500.
		//schedule_delayed_work(&bq->ico_work,0);
		bq2589x_set_chargecurrent(g_bq1, 1500);
	}
	else{
		// otg & other mode.
		bq2589x_set_input_current_limit(g_bq1,512);
		bq2589x_set_chargecurrent(g_bq1,512);
	}

	if(bq->use_absolute_vindpm){
		bq2589x_adjust_absolute_vindpm(bq);
	}

	if(bq->vbus_type != BQ2589X_VBUS_MAXC)
	{
		bq2589x_enable_charger(g_bq1);
		schedule_delayed_work(&bq->monitor_work, 10*HZ);
	}
	bq->status &= ~BQ2589X_HVDCP_DCP_EXCHANGE;
}

static void bq2589x_adapter_out_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, adapter_out_work);
	
	bq2589x_set_force_vindpm_enable(g_bq1, 1);
	bq2589x_set_input_volt_limit(g_bq1,4600); 

	bq2589x_adc_stop(g_bq1);
	bq2589x_set_chargecurrent(g_bq1,512);  
	bq2589x_exit_hiz_mode(g_bq1);
	bq2589x_enable_charger(g_bq1);

	bq2589x_enable_max(g_bq1);
	bq2589x_enable_hvdcp(g_bq1);

	bq2589x_set_dpdm(g_bq1, false);
	bq2589x_auto_dpdm_detect(CHARGER_UNKNOWN, false);

	vbus_voltage_flag = 0;
	is_max_charge = 0;
	last_temp_level = 0;
	ico_state = ICO_INIT_STATE;
	g_bq1->ico_value = 0;
	bq->status &= ~BQ2589X_HVDCP_DCP_EXCHANGE;
	bq2589x_cmd_set_hiz = 0;
	g_bq1->temp_init = 0;
    
	if(g_bq2!=NULL)
	{        
		bq2589x_adc_stop(g_bq2);
		bq2589x_set_input_volt_limit(g_bq2,4600);
		bq2589x_set_chargecurrent(g_bq2,512);  
		bq2589x_disable_charger(g_bq2);
	}
	cancel_delayed_work(&bq->monitor_work);
	cancel_delayed_work(&bq->ico_work);
}

static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, ico_work.work);
	u8 status;
	int ret = 0;
	static int value;
	int vbus,vbat,ichg;
	static int ldpmlin =0;
	static int inputlimit= 0;
	int ico_done=0;

    if(g_bq2!=NULL)
    {	
		switch (ico_state)
		{	
			case ICO_INIT_STATE:
			default:	
				bq2589x_disable_charger(g_bq1);
				bq2589x_exit_hiz_mode(g_bq2);
				
				bq2589x_set_input_current_limit(g_bq1,500);
				bq2589x_set_input_current_limit(g_bq2,2000);
				
				bq2589x_enable_charger(g_bq2);
				
				bq2589x_set_chargecurrent(g_bq1,2250);
				if(g_bq2!=NULL)
					bq2589x_set_chargecurrent(g_bq2,2250);

				
				printk("[battery]%s ico init state",__func__);
				bq2589x_set_force_vindpm_enable(g_bq1,1);
				bq2589x_adjust_absolute_vindpm(g_bq1);

    				/* Read VINDPM/IINDPM status */
				//bq2589x_set_ico(g_bq2,1);
				bq2589x_enable_charger(g_bq2);
				
				vbus = bq2589x_adc_read_vbus_volt(g_bq2);
				vbat = bq2589x_adc_read_battery_volt(g_bq2);
				ichg = bq2589x_adc_read_charge_current(g_bq2);
				value = vbat*ichg/vbus/9*10;
				inputlimit = 500;
				
				bq2589x_set_input_current_limit(g_bq1,inputlimit);	
				
        		bq2589x_enable_charger(g_bq1);
				bq2589x_set_ico(g_bq1,1);
				
				bq2589x_dump_register(g_bq1);

				printk("[battery]%s value=%d,vbus=%d,vbat=%d,ichg=%d,ldpmlin=%d\n"
					, __func__,value,vbus,vbat,ichg, ldpmlin);
						
				if(bq2589x_charge_status(bq) != POWER_SUPPLY_CHARGE_TYPE_FAST || vbat > ICO_CV_VOLTAGE)
				{
				
					printk("[battery]%s: system power low, set 12V/1300mA,ico cv:%d\n", __func__,(int)ICO_CV_VOLTAGE);
					ico_state = ICO_SETUP_TABLE_STATE;
					ldpmlin =1300;
					schedule_delayed_work(&bq->ico_work,0);
					break;
				}

				ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);

    			if(ret < 0){
					ico_state = ICO_REG_RETRY_STATE;
    				schedule_delayed_work(&bq->ico_work,2*HZ);
    				break;
    			}
				else
				{
					
					ico_state = ICO_INCREASE_CURRENT_STATE;
					ldpmlin = (int) ( ((status & BQ2589X_IDPM_LIM_MASK)*BQ2589X_IDPM_LIM_LSB) + BQ2589X_IDPM_LIM_BASE);
    				schedule_delayed_work(&bq->ico_work,0);					
    				break;					
				}
		
				break;
			case ICO_ISSUE_STATE:

				battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO issued state\n",__func__);
    			ret = bq2589x_check_force_ico_done(bq);
    			if(ret){//ico done
					ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
					ldpmlin = (int) ( ((status & BQ2589X_IDPM_LIM_MASK)*BQ2589X_IDPM_LIM_LSB) + BQ2589X_IDPM_LIM_BASE);
					
					ico_state = ICO_INCREASE_CURRENT_STATE;
    				bq2589x_force_ico(bq);
    				battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO done!\n",__func__);
    				schedule_delayed_work(&bq->ico_work,3*HZ);
    			}				
				else
				{
					ico_state = ICO_FAIL_STATE;
					battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO not done!\n",__func__);
					schedule_delayed_work(&bq->ico_work,3*HZ);
				}
				
				break;
			case ICO_SETUP_TABLE_STATE:
				temp_adjust_current_limit_init(ldpmlin * 100);
				ico_done = value + ldpmlin;
				g_bq1->ico_value = ldpmlin;
				inputlimit = 500;
				ldpmlin = 0;
				ico_state = ICO_DONE_STATE;
				schedule_delayed_work(&bq->ico_work,0);
				battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO setup table state,ico done:%d,ldpmlin:%d\n",__func__, ico_done,ldpmlin);
				break;
			case ICO_DONE_STATE:
				temp_adjust_current_limit_dump();
				bq2589x_set_input_current_limit(g_bq1,g_bq1->ico_value);
				bq2589x_set_ico(g_bq1,1);
				bq2589x_enable_charger(g_bq1);
				
				schedule_delayed_work(&bq->charger2_enable_work,1*HZ);
				schedule_delayed_work(&bq->monitor_work,3*HZ);
				battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO done state.\n",__func__);
				//ico_state = ICO_INIT_STATE;
				break;
			case ICO_FAIL_STATE:
				ico_state = ICO_SETUP_TABLE_STATE;
				ldpmlin =1300;
				schedule_delayed_work(&bq->ico_work,0);

				battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO fail set 12V/1300mA:%d\n",__func__,ret);
				break;
			case ICO_FORCE_RETRY_STATE: 
				battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO retry state:%d\n",__func__,ret);
				ret = bq2589x_force_ico(bq);
				if(ret < 0){
					ico_state = ICO_FAIL_STATE;
					schedule_delayed_work(&bq->ico_work,3*HZ);
					battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO retry command issued failed:%d\n",__func__,ret);
				}
				else{
					ico_state = ICO_ISSUE_STATE;
					schedule_delayed_work(&bq->ico_work,3*HZ);
					battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO command issued successfully\n",__func__);
				}					
				break;
			case ICO_REG_RETRY_STATE:
				printk("[battery]%s ico reg retry state",__func__);
				ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
				
    			if(ret < 0){
					ico_state = ICO_REG_RETRY_STATE;
    				schedule_delayed_work(&bq->ico_work,2*HZ);
    				break;
    			}		
				else
				{
					ico_state = ICO_INCREASE_CURRENT_STATE;
					ldpmlin = (int) ( ((status & BQ2589X_IDPM_LIM_MASK)*BQ2589X_IDPM_LIM_LSB) + BQ2589X_IDPM_LIM_BASE);
    				schedule_delayed_work(&bq->ico_work,0);
				}
				break;
			case ICO_INCREASE_CURRENT_STATE:
				printk("[battery]%s ico increase current state:%d,%d\n", __func__,ldpmlin, inputlimit);

				if(ldpmlin >= 2000 )
				{
					ico_state = ICO_SETUP_TABLE_STATE;
    				schedule_delayed_work(&bq->ico_work,0);					
				}				
				else if(ldpmlin == inputlimit)
				{
					ret = bq2589x_force_ico(bq);
					
					if(ret < 0){
						schedule_delayed_work(&bq->ico_work,1*HZ); // retry 1 second later
						battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO command issued failed:%d\n",__func__,ret);
					}
					else{
						inputlimit += 500;
						bq2589x_set_input_current_limit(g_bq1,inputlimit);
						ico_state = ICO_ISSUE_STATE;
						schedule_delayed_work(&bq->ico_work,3*HZ);
						battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:ICO command issued successfully\n",__func__);
					}	
				}
				else
				{
					ico_state = ICO_SETUP_TABLE_STATE;
    				schedule_delayed_work(&bq->ico_work,0);
				}
				break;
		}		
    }
}

// check if need to eanble charger 2
static void bq2589x_charger2_enable_workfunc(struct work_struct *work)
{
	int ret;

	if(is_max_charge)
		bq2589x_adjust_absolute_vindpm(g_bq1);
	
	//TODO:read rsoc, or any conditons to check if needed to enable charger 2
   // if(bq->rsoc < 95){	
    	ret = bq2589x_exit_hiz_mode(g_bq2);
    	if(ret)
	    	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: charger 2 exit hiz mode failed:%d\n",__func__,ret);
    	else{
	    	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: charger 2 exit hiz mode successfully\n",__func__);
    		g_bq2->enabled = true;
    	}
        bq2589x_enable_charger(g_bq2);
  //  }
}


static void bq2589x_check_if_tuneup_workfunc(struct work_struct* work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, check_to_tuneup_work.work);
	
//	g_bq1->vbus_volt = bq2589x_adc_read_vbus_volt(g_bq1);
    g_bq1->vbat_volt = bq2589x_adc_read_battery_volt(g_bq1);
	
	if(bq->vbat_volt > g_bq1->vbat_min_volt_to_tuneup && g_bq1->vbat_volt < ICO_CV_VOLTAGE /*g_bq1->rsoc < 95*/){    
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:trying to tune up vbus voltage\n",__func__);
		voltcontrol.TuneTargetVolt = g_bq1->vbus_volt_high_level;
		voltcontrol.toTuneUpVolt = true;
		voltcontrol.toTuneDownVolt = false;
		voltcontrol.TuneVoltDone = false;
		voltcontrol.TuneCounter = 0;
		voltcontrol.TuneFail = false;	
		schedule_delayed_work(&bq->volt_tune_work,0);
	}
    else if(g_bq1->vbat_volt > ICO_CV_VOLTAGE/*g_bq1->rsoc >95*/)
    {
    	if( ico_state == ICO_DONE_STATE)
    	{
    		ico_state = ICO_INIT_STATE;
        	schedule_delayed_work(&bq->ico_work,0);
    	}
    }
	else
		schedule_delayed_work(&bq->check_to_tuneup_work, 2*HZ);
}

static void bq2589x_tune_volt_workfunc(struct work_struct * work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, volt_tune_work.work);
	int ret = 0;
	static bool pumpx_cmd_issued = false;
	
	g_bq1->vbus_volt = bq2589x_adc_read_vbus_volt(g_bq1);

    battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:vbus voltage:%d, Tune Target Volt:%d\n",__func__,g_bq1->vbus_volt,voltcontrol.TuneTargetVolt);

	if( (voltcontrol.toTuneUpVolt && g_bq1->vbus_volt > voltcontrol.TuneTargetVolt)|| 
		(voltcontrol.toTuneDownVolt && g_bq1->vbus_volt < voltcontrol.TuneTargetVolt)){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:voltage tune successfully\n",__func__);
		voltcontrol.TuneVoltDone = true;
		bq2589x_adjust_absolute_vindpm(bq);
		if(voltcontrol.toTuneUpVolt)
		{
			if( ico_state == ICO_DONE_STATE)
    		{
				ico_state = ICO_INIT_STATE;
				schedule_delayed_work(&bq->ico_work,0);
			}
		}
		return;
	}
	
	if(voltcontrol.TuneCounter > 10){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:voltage tune failed,reach max retry count\n",__func__);
		voltcontrol.TuneFail = true;
		bq2589x_adjust_absolute_vindpm(bq);

		if(voltcontrol.toTuneUpVolt)
		{
			if( ico_state == ICO_DONE_STATE)
    		{
				ico_state = ICO_INIT_STATE;
				schedule_delayed_work(&bq->ico_work,0);
			}
		}
		return;
	}

	if(!pumpx_cmd_issued){
		if(voltcontrol.toTuneUpVolt)
			ret = bq2589x_pumpx_increase_volt(bq);
		else if(voltcontrol.toTuneDownVolt)
			ret =  bq2589x_pumpx_decrease_volt(bq);
		if(ret)
			schedule_delayed_work(&bq->volt_tune_work,HZ);//retry
		else{
			battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:pumpx command issued.\n",__func__);
			pumpx_cmd_issued = true;
			voltcontrol.TuneCounter++;
			schedule_delayed_work(&bq->volt_tune_work,3*HZ);
		}
	}
	else{
		if(voltcontrol.toTuneUpVolt)
			ret = bq2589x_pumpx_increase_volt_done(bq);
		else if(voltcontrol.toTuneDownVolt)
			ret = bq2589x_pumpx_decrease_volt_done(bq);
		if(ret == 0){//finished for one step 
			battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:pumpx command finishedd!\n",__func__);
			bq2589x_adjust_absolute_vindpm(bq);
			pumpx_cmd_issued = 0;
		}			
		schedule_delayed_work(&bq->volt_tune_work,HZ);
	}
}

extern  int mtkts_AP_get_hw_temp(void);
//AP_temp * 1000, charge_current * 100, hvdcp_enable
static void temp_adjust_current_limit_init(int current_limit)
{
	int temp_level = 0;
	
	for(temp_level = 0; temp_level < TEMP_CURRENT_TABLE_LENGTH; temp_level++) {

		switch(screen_off_current_table[temp_level][4])
		{
			case LEVEL0_LIMIT:
				if(screen_off_current_table[temp_level][4] >= current_limit)
				{
					screen_off_current_table[temp_level][3] = current_limit;
				}
				else
				{
					screen_off_current_table[temp_level][3] = LEVEL0_LIMIT;
				}
				break;
			case LEVEL1_LIMIT:
				if(screen_off_current_table[temp_level][4] >= current_limit)
				{
					screen_off_current_table[temp_level][3] = current_limit;
				}				
				else
				{
					screen_off_current_table[temp_level][3] = LEVEL1_LIMIT;
				}
				break;
			case LEVEL2_LIMIT:
			default:
				if(screen_off_current_table[temp_level][4] >= current_limit)
				{
					screen_off_current_table[temp_level][3] = current_limit;
				}				
				else
				{
					screen_off_current_table[temp_level][3] = LEVEL2_LIMIT;
				}
				break;
		}
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:temp=%d,adjust limit=%d, current_limit:%d\n", 
				__func__, temp_level,screen_off_current_table[temp_level][3], current_limit);
		
	}
}

static void temp_adjust_current_limit_dump(void)
{
	int temp_level = 0;
	
	for(temp_level = 0; temp_level < TEMP_CURRENT_TABLE_LENGTH; temp_level++)
	{
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:temp=%d,adjust limit=%d\n", 
				__func__, temp_level,screen_off_current_table[temp_level][3]);

	}
}

static int bq2589x_adjust_charge_current(int temp)
{
	int hvdcp_enable;	
	int charge_current;
	int current_limit;
	int temp_level = 0;
	int battery_cap = 0;
	int disable_quick;
	static int last_temp_level = 0;
	int (*temp_current_table)[5];

	battery_cap = bat_get_ui_percentage();
	if(battery_cap >= 90 || screen_on || g_bq1->disable_quick_charge) {
		printk("rsoc = %d,screen = %d, g_bq1->disable_quick_charge=%d\n",
				battery_cap,screen_on,g_bq1->disable_quick_charge);
		disable_quick = true;
	}
	else
		disable_quick = false;

	if(disable_quick)
		temp_current_table = screen_on_current_table;
	else
		temp_current_table = screen_off_current_table;

	for(temp_level = 0; temp_level < TEMP_CURRENT_TABLE_LENGTH; temp_level++) {
		if(temp < temp_current_table[temp_level][0])
				break;
	}
	if(temp_level <= 1)
	{
		temp_level = 0;
		last_temp_level = 0;
	}
	else
		temp_level = temp_level - 1;

	if(temp_level >= last_temp_level)
		last_temp_level = temp_level;
	else
	{
		if(temp < (temp_current_table[last_temp_level][0] - 2000))
			last_temp_level = temp_level;
		else
			temp_level = last_temp_level;
	}

	charge_current = temp_current_table[temp_level][1];
	hvdcp_enable = temp_current_table[temp_level][2];
	current_limit = temp_current_table[temp_level][3];
	battery_cap = bat_get_ui_percentage();

	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:temp=%d,charge_current=%d,hvdcp_enable=%d\n", 
		__func__, temp,charge_current,hvdcp_enable);
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s jeita_current = %d, jeita_input_limit = %d, battery_cap=%d\n",
		__func__, jeita_current, jeita_input_limit, battery_cap);
	
	charge_current = min(charge_current, jeita_current);
	jeita_input_limit =  min(current_limit, jeita_input_limit);

	//letv add for MemoryLeak test
	if(g_battery_thermal_throttling_flag == 2)
	{

		hvdcp_enable = 0;
		charge_current = 160000;
		jeita_input_limit = 160000;
	}
	//letv add for MemoryLeak test end
	
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s charge_current=%d, jeita_input_limit=%d\n", __func__, charge_current, jeita_input_limit);

	bq25890_charging_curernt_set_external(charge_current);
	bq2589x_charging_hw_inputcurrent_limit_external(jeita_input_limit);
	bq2589x_high_dcp_set(hvdcp_enable);

	return 0;
}
#define BQ_TABLE_LENGTH 12
static int bq2589x_get_ap_temp(void)
{
	int temp = 0, temp_sum = 0, i =0;
	static int bq2589x_ap_temp_table[BQ_TABLE_LENGTH] = {0};
	static int count = 0;

	temp = mtkts_AP_get_hw_temp();

	if(!g_bq1->temp_init)
	{
		for(i = 0; i < BQ_TABLE_LENGTH; i++)
		{
			bq2589x_ap_temp_table[i] = 20000;
		}
		g_bq1->temp_init = 1;
	}

	if(count >= BQ_TABLE_LENGTH - 1)
		count = 0;
	else
		count++;

	bq2589x_ap_temp_table[count] = temp;

	for(i = 0; i < BQ_TABLE_LENGTH; i++)
		temp_sum += bq2589x_ap_temp_table[i];

	temp = temp_sum / BQ_TABLE_LENGTH;

	return temp;
}

extern int mtk_get_iddig_in_staus(void);
static void bq2589x_monitor_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, monitor_work.work);
	int AP_temp;
	static int frist_init = 1;
	int max_cable;

	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s\n",__func__);
	//kick watchdog
	bq2589x_reset_watchdog_timer(bq);
	if(g_bq2!=NULL)
	{
		if(g_bq2->enabled)
		bq2589x_reset_watchdog_timer(g_bq2);
	}

	if(bq->vbus_type == BQ2589X_VBUS_OTG)
	{
		if(mtk_get_iddig_in_staus() == false)
		{
			battery_xlog_printk(BAT_LOG_CRTI,"[battery] %s id is low in otg mode\n", __func__);
			goto dump;
		}
		else
		{
			battery_xlog_printk(BAT_LOG_CRTI,"[battery] %s id is high but in otg mode\n", __func__);
			return;
		}
	}

	AP_temp = bq2589x_get_ap_temp();
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]AP_temp=%d, screen_on=%d,bq->vbus_type=%d, is_max_charge=%d\n",
				AP_temp, screen_on, bq->vbus_type, is_max_charge);

	if(frist_init) {
		frist_init = 0;
		schedule_delayed_work(&bq->monitor_work,10*HZ);
		return;
	}
	max_cable = tusb_check_type_cable();
	if(jeita_input_limit > 150000 && max_cable != CURRENT_MODE_HIGH)
		jeita_input_limit = 150000;
	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s max_cable=%d jeita_input_limit=%d\n", __func__, max_cable, jeita_input_limit);

	if(is_max_charge) {
		//AP_temp = test_temp;
		if( ico_state != ICO_DONE_STATE)
		{
			schedule_delayed_work(&bq->monitor_work,10*HZ);
			return;			
		}
		
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s AP_temp = %d test_temp = %d\n", __func__, AP_temp, test_temp);
		bq2589x_adjust_charge_current(AP_temp);
	} else {
		bq25890_charging_curernt_set_external(jeita_current);
		bq2589x_charging_hw_inputcurrent_limit_external(jeita_input_limit);
	}
	
dump:
    bq2589x_dump_register(bq);
    if(g_bq2!=NULL)
        bq2589x_dump_register(g_bq2);

	schedule_delayed_work(&bq->monitor_work,10*HZ);
}


static void bq2589x_charger1_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	int ret;

	mutex_lock(&bq->lock);
    battery_xlog_printk(BAT_LOG_CRTI,"[battery]bq2589x_charger1_irq_workfunc\n");


	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) 
	{
		mutex_unlock(&bq->lock);
		return;
	}

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret) 
	{
		mutex_unlock(&bq->lock);
		return;
	}
	
	bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;
    battery_xlog_printk(BAT_LOG_CRTI,"[battery]bq->vbus_type %d reg0b:0x%x, bq->status=0x%x\n",bq->vbus_type,status, bq->status);
    if(((status & BQ2589X_VBUS_STAT_MASK) == 0) && (bq->status & BQ2589X_STATUS_PLUGIN)){// plug out
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:adapter removed\n",__func__);
        bq->status &= ~BQ2589X_STATUS_PLUGIN;
		schedule_work(&bq->adapter_out_work);
 
    }
    else if(((status & BQ2589X_VBUS_STAT_MASK) && 
	    (!(bq->status & BQ2589X_STATUS_PLUGIN)))|| (bq->status & BQ2589X_HVDCP_DCP_EXCHANGE)){
	    if(is_max_charge)
	    {
	    	if((vbus_voltage_flag && bq->vbus_type == BQ2589X_VBUS_MAXC) || 
			(!vbus_voltage_flag && bq->vbus_type == BQ2589X_VBUS_USB_DCP))
		{
        		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:vbus_voltage_flag=%d,bq->vbus_type=%d, bad charge type return\n",
					__func__, vbus_voltage_flag, bq->vbus_type);
			mutex_unlock(&bq->lock);
			return;
		}
	    }

        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:adapter plugged in\n",__func__);
        bq->status |= BQ2589X_STATUS_PLUGIN;
		schedule_work(&bq->adapter_in_work);
    }

   if(is_power_off_charging() && bq->vbus_type == BQ2589X_VBUS_MAXC && first_retry ==0)
  {
   	bq->status &= ~BQ2589X_STATUS_PLUGIN;
    	first_retry =1;
	if( is_max_charge !=1 )
	{
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:deli retry irq\n",__func__);
		schedule_work(&bq->irq_work);
	}
   }

    if((status & BQ2589X_PG_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PG)){
        bq->status |= BQ2589X_STATUS_PG;
    }
    else if(!(status & BQ2589X_PG_STAT_MASK) && (bq->status & BQ2589X_STATUS_PG)){
        bq->status &=~BQ2589X_STATUS_PG;
    }

    if(fault && !(bq->status & BQ2589X_STATUS_FAULT)){
        bq->status |= BQ2589X_STATUS_FAULT;
    }
    else if(!fault &&(bq->status &BQ2589X_STATUS_FAULT)){
        bq->status &=~BQ2589X_STATUS_FAULT;
    }

    bq->interrupt = true;
	mutex_unlock(&bq->lock);
}


static irqreturn_t bq2589x_charger1_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

static void bq2589x_early_suspend(struct early_suspend *h)
{
	mutex_lock(&suspend_lock);
	screen_on = 0;
	mutex_unlock(&suspend_lock);
}
static void bq2589x_late_resume(struct early_suspend *h)
{
	mutex_lock(&suspend_lock);
	screen_on = 1;
	mutex_unlock(&suspend_lock);
}


#define GPIO_IRQ    104
static int bq2589x_charger1_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
    int irqn;
	int ret;
	u8 status;

    bq = kzalloc(sizeof(struct bq2589x),GFP_KERNEL);
    if(!bq){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: out of memory\n",__func__);
        return -ENOMEM;
    }
    bq->dev = &client->dev;
    bq->client = client;
    i2c_set_clientdata(client,bq);
   // bq->client->timing =100;
    ret = bq2589x_detect_device(bq);
    if(ret == 0){
        if(bq->part_no == BQ25890){
			bq->status |= BQ2589X_STATUS_EXIST;
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: charger device bq25890 detected, revision:%d\n",__func__,bq->revision); 
        }
        else{
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: unexpected charger device detected %x\n",__func__,bq->part_no);
            //mt_set_gpio_mode((GPIO55 | 0x80000000),GPIO_MODE_00);  
           // mt_set_gpio_dir((GPIO55 | 0x80000000),GPIO_DIR_OUT);
           // mt_set_gpio_out((GPIO55| 0x80000000),GPIO_OUT_ONE);

            kfree(bq);
			return -ENODEV;
        }
    }
    else{
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: no bq25890 charger device found:%d\n",__func__,ret); 
        kfree(bq);
        return -ENODEV;
    }

    g_bq1 = bq;

    g_bq1->vbus_volt_high_level = 4400; //by default adapter output 5v, if >4.4v,it is ok after tune up
    g_bq1->vbus_volt_low_level = 5500; //by default adapter output 5v, if <5.5v,it is ok after tune down
    g_bq1->vbat_min_volt_to_tuneup = 3000; // by default, tune up adapter output only when bat is >3000

//    g_bq1->charge_voltage = 4208;
//    g_bq1->charge_current = 2048;

	if(client->dev.of_node)
		 bq2589x_parse_dt(&client->dev, g_bq1);

	ret = bq2589x_init_device(g_bq1);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]device init failure: %d\n", ret);
		goto err_0;
	}    // platform setup, irq,...
//mt_eint_enable_debounce(unsigned int cur_eint_num)
	/*ret = gpio_request(GPIO_IRQ, "bq2589x irq pin");
	if(ret)
	{
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: %d gpio request failed\n", __func__, GPIO_IRQ);
		goto err_0;
	}*/

	bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
	if(status & (BQ2589X_VDPM_STAT_MASK | BQ2589X_IDPM_STAT_MASK)) //VINDPM or IINDP
    		bq2589x_force_dpdm(g_bq1);

	bq2589x_disable_otg(bq);//temp solution for sometime charging without charger.
    bq2589x_exit_ship_mode(bq);
    bq2589x_adc_stop(bq);
    bq2589x_exit_hiz_mode(bq);
    bq2589x_dump_register(bq);

    mt_set_gpio_mode((GPIO104 | 0x80000000),GPIO_MODE_00);
    mt_set_gpio_dir((GPIO104 | 0x80000000),GPIO_DIR_IN);
    mt_set_gpio_pull_enable((GPIO104 | 0x80000000),GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select((GPIO104 | 0x80000000),GPIO_PULL_UP);

	irqn = mt_gpio_to_irq(104);

    
	//gpio_direction_input(GPIO_IRQ);

   // irqn = gpio_to_irq(GPIO_IRQ);
    if(irqn < 0){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:%d gpio_to_irq failed\n",__func__,irqn);
        ret = irqn;
        goto err_1;
    }
    client->irq = irqn;

	mutex_init(&bq->lock);

	INIT_WORK(&bq->irq_work, bq2589x_charger1_irq_workfunc);
	INIT_WORK(&bq->adapter_in_work, bq2589x_adapter_in_workfunc);
	INIT_WORK(&bq->adapter_out_work, bq2589x_adapter_out_workfunc);
	INIT_DELAYED_WORK(&bq->monitor_work, bq2589x_monitor_workfunc);
	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->volt_tune_work, bq2589x_tune_volt_workfunc);
	INIT_DELAYED_WORK(&bq->check_to_tuneup_work, bq2589x_check_if_tuneup_workfunc);
	INIT_DELAYED_WORK(&bq->charger2_enable_work, bq2589x_charger2_enable_workfunc );

	ret = bq2589x_psy_register(bq);
	if(ret) 
		goto err_0;

	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}
    // request irq
	ret = request_irq(client->irq, bq2589x_charger1_interrupt,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"bq2589x_charger1_irq", bq);
	if (ret) {
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Request IRQ %d failed: %d\n", __func__,client->irq, ret);
		goto err_irq;
	}
    else{
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:irq = %d\n",__func__,client->irq);
    }
#ifdef CONFIG_HAS_EARLYSUSPEND
	bq->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	bq->early_suspend.suspend = bq2589x_early_suspend;
	bq->early_suspend.resume = bq2589x_late_resume;
	register_early_suspend(&bq->early_suspend);
#endif


	/*mt_set_gpio_mode((GPIO104 | 0x80000000), 0);
    mt_set_gpio_dir((GPIO104 | 0x80000000), 0);
	mt_set_gpio_pull_enable((GPIO104 | 0x80000000), 1);
	mt_set_gpio_pull_select((GPIO104 | 0x80000000), 1);

	mt_eint_set_hw_debounce(104, 0);
	mt_eint_registration(104, 2, bq2589x_charger1_interrupt, 0);
	mt_eint_unmask(104);*/

    voltcontrol.TuneVoltwithPEplus = true; 
    schedule_work(&bq->irq_work);//in case of adapter has been in when power off
	return 0;

err_irq:
	cancel_work_sync(&bq->irq_work);
err_1:
    gpio_free(GPIO_IRQ);
err_0:
	kfree(bq);
    g_bq1 = NULL;
	return ret;
}

static void bq2589x_charger1_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: is shutdown\n",__func__);
	   
	disable_irq(client->irq);
        bq2589x_disable_otg(g_bq1);
	bq2589x_psy_unregister(bq);

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);
	cancel_delayed_work(&bq->monitor_work);
	cancel_delayed_work(&bq->ico_work);
	cancel_delayed_work(&bq->charger2_enable_work);
	bq2589x_set_chargecurrent(g_bq1,512);
	bq2589x_exit_hiz_mode(g_bq1);
	bq2589x_enable_charger(g_bq1);

	//bq2589x_enter_hiz_mode(g_bq1);
	//add for 12V to 5V shutdown.
	bq2589x_set_force_vindpm_enable(g_bq1,1);
	bq2589x_set_input_volt_limit(g_bq1,4600);
}

/* interface for other module end */
static void bq2589x_charger2_irq_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, irq_work);
	u8 status = 0;
	u8 fault = 0;
	int ret;


	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) 
		return;

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret) 
		return;

    if(((status & BQ2589X_VBUS_STAT_MASK) == 0) && (bq->status & BQ2589X_STATUS_PLUGIN)){// plug out
        bq->status &= ~BQ2589X_STATUS_PLUGIN;

    }
    else if((status & BQ2589X_VBUS_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PLUGIN)){
        bq->status |= BQ2589X_STATUS_PLUGIN;
    }

    if((status & BQ2589X_PG_STAT_MASK) && !(bq->status & BQ2589X_STATUS_PG)){
        bq->status |= BQ2589X_STATUS_PG;
    }
    else if(!(status & BQ2589X_PG_STAT_MASK) && (bq->status & BQ2589X_STATUS_PG)){
        bq->status &=~BQ2589X_STATUS_PG;
    }

    if(fault && !(bq->status & BQ2589X_STATUS_FAULT)){
        bq->status |= BQ2589X_STATUS_FAULT;
    }
    else if(!fault &&(bq->status &BQ2589X_STATUS_FAULT)){
        bq->status &=~BQ2589X_STATUS_FAULT;
    }
	
    bq->interrupt = true;
	
}
#if 0
static irqreturn_t bq2589x_charger2_interrupt(int irq, void *data)
{
	struct bq2589x *bq = data;

	schedule_work(&bq->irq_work);
	return IRQ_HANDLED;
}

#endif

static int bq2589x_charger2_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;

	int ret;

    bq = kzalloc(sizeof(struct bq2589x),GFP_KERNEL);
    if(!bq){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: out of memory\n",__func__);
        return -ENOMEM;
    }
    
    bq->dev = &client->dev;
    bq->client = client;
    i2c_set_clientdata(client,bq);
    ret = bq2589x_detect_device(bq);
    if(ret == 0){
        if(bq->part_no == BQ25892){
			bq->status |= BQ2589X_STATUS_EXIST;
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: charger device bq25892 detected, revision:%d\n",__func__,bq->revision); 
        }
        else{
            battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: unexpected charger device detected\n",__func__);
          //  mt_set_gpio_mode((GPIO56 | 0x80000000),GPIO_MODE_00);  
           // mt_set_gpio_dir((GPIO56 | 0x80000000),GPIO_DIR_OUT);
          //  mt_set_gpio_out((GPIO56| 0x80000000),GPIO_OUT_ONE);

            kfree(bq);
			return -ENODEV;
        }
    }
    else{
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: no charger device bq25892 found:%d\n",__func__,ret); 
        kfree(bq);
        return -ENODEV;
    }

    bq2589x_enter_hiz_mode(bq);
    bq2589x_exit_ship_mode(bq);
    bq2589x_adc_stop(bq);

    g_bq2 = bq;

    // initialize bq2589x, disable charger 2 by default 
	ret = bq2589x_init_device(g_bq2);
	if(ret){
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:Failed to initialize bq2589x charger\n",__func__);
	}
    else{
		battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: Initialize bq2589x charger successfully!\n",__func__);
	}
    // platform setup, irq,...
	INIT_WORK(&bq->irq_work, bq2589x_charger2_irq_workfunc);
	mt_set_gpio_mode((GPIO56 | 0x80000000),GPIO_MODE_00);  
    mt_set_gpio_pull_enable((GPIO56 | 0x80000000), GPIO_PULL_DISABLE);
    mt_set_gpio_dir((GPIO56 | 0x80000000),GPIO_DIR_OUT);
    mt_set_gpio_out((GPIO56| 0x80000000),GPIO_OUT_ZERO);

    return 0;
}

static void bq2589x_charger2_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);
    bq2589x_set_chargecurrent(g_bq2,512);  
    bq2589x_enter_hiz_mode(g_bq2);
   // bq2589x_enter_ship_mode(g_bq2);//solution of DVT2-3 can not power on

	battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s: shutdown\n",__func__);
	cancel_work_sync(&bq->irq_work);
}

static struct of_device_id bq2589x_charger1_match_table[] = {
    {.compatible = "ti,bq2589x-1",},
    {},
};


static const struct i2c_device_id bq2589x_charger1_id[] = {
	{ "bq2589x-1", 0 },
	{},
};

MODULE_DEVICE_TABLE(of, bq2589x_charger1_match_table);

static struct i2c_driver bq2589x_charger1_driver = {
	.driver		= {
		.name	= "bq2589x-1",
		.of_match_table = bq2589x_charger1_match_table,
	},
	.probe		= bq2589x_charger1_probe,
	.shutdown   = bq2589x_charger1_shutdown,
	.id_table	= bq2589x_charger1_id,
};


static struct of_device_id bq2589x_charger2_match_table[] = {
    {.compatible = "ti,bq2589x-2",},
    {},
};

static const struct i2c_device_id bq2589x_charger2_id[] = {
	{ "bq2589x-2", 0 },
	{},
};

MODULE_DEVICE_TABLE(of, bq2589x_charger2_match_table);


static struct i2c_driver bq2589x_charger2_driver = {
	.driver		= {
		.name	= "bq2589x-2",
		.of_match_table = bq2589x_charger2_match_table,
	},

	.probe		= bq2589x_charger2_probe,
	.shutdown   = bq2589x_charger2_shutdown,
	.id_table	= bq2589x_charger2_id,
};

static struct i2c_board_info __initdata i2c_bq2589x_charger1[] =
{
    { 
        I2C_BOARD_INFO("bq2589x-1",0x6A),
    },
};


static struct i2c_board_info __initdata i2c_bq2589x_charger2[] =
{
    {
        I2C_BOARD_INFO("bq2589x-2",0x6B),
    },
};


static int __init bq2589x_charger_init(void)
{

	mt_set_gpio_mode((GPIO55 | 0x80000000),GPIO_MODE_00);  
    mt_set_gpio_pull_enable((GPIO55 | 0x80000000), GPIO_PULL_DISABLE);
    mt_set_gpio_dir((GPIO55 | 0x80000000),GPIO_DIR_OUT);
    mt_set_gpio_out((GPIO55| 0x80000000),GPIO_OUT_ZERO);

    i2c_register_board_info(2,i2c_bq2589x_charger2,ARRAY_SIZE(i2c_bq2589x_charger2));
    i2c_register_board_info(2,i2c_bq2589x_charger1,ARRAY_SIZE(i2c_bq2589x_charger1));

    if(i2c_add_driver(&bq2589x_charger2_driver)){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s, failed to register bq2589x_charger2_driver.\n",__func__);
    }
    else{
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s, bq2589x_charger2_driver register successfully!\n",__func__);
    }

    if(i2c_add_driver(&bq2589x_charger1_driver)){
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s, failed to register bq2589x_charger1_driver.\n",__func__);
    }
    else{
        battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s, bq2589x_charger1_driver register successfully!\n",__func__);
    }



    return 0; 
}

static void __exit bq2589x_charger_exit(void)
{
    battery_xlog_printk(BAT_LOG_CRTI,"[battery]%s:\n",__func__);
    i2c_del_driver(&bq2589x_charger1_driver);
    i2c_del_driver(&bq2589x_charger2_driver);
}

module_init(bq2589x_charger_init);
module_exit(bq2589x_charger_exit);

MODULE_DESCRIPTION("TI BQ2589x Dual Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
