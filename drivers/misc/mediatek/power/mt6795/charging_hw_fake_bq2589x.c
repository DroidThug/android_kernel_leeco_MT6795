#include <mach/charging.h>
#include <mach/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>

#include "cust_battery_meter.h"
#include <cust_charging.h>
#include <cust_pmic.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <mach/battery_common.h>
#include <mach/mt_gpio.h>

 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //
kal_bool chargin_hw_init_done = KAL_TRUE; 
kal_bool charging_type_det_done = KAL_TRUE;
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;

// ============================================================ //
// function prototype
// ============================================================ //
 
 
// ============================================================ //
//extern variable
// ============================================================ //
 
// ============================================================ //
//extern function
// ============================================================ //
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
extern int PMIC_IMM_GetOneChannelValue(upmu_adc_chl_list_enum dwChannel, int deCount, int trimd);
extern int hw_charging_get_charger_type(void);
extern unsigned int get_pmic_mt6332_cid(void);
extern bool mt_usb_is_device(void);
extern int bq25890_get_vbus_type_external(void);
extern int bq25890_get_charging_status_external(void);
extern void bq2589x_charging_hw_inputcurrent_limit_external(int value);
extern void bq25890_charging_curernt_set_external(int value);
extern void bq2589x_charging_set_jeita_current(int charge_current);
extern void bq2589x_charging_set_jeita_input_limit(int input_limit);

 // ============================================================ //
static kal_uint32 charging_hw_init(void *data)
{
    kal_uint32 status = STATUS_OK;
    mt6332_upmu_set_rg_precc_m3_en(1);
    mt6332_upmu_set_rg_m3_en(0);
    return status;
}


static kal_uint32 charging_dump_register(void *data)
{
    kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_FPGA)     
#else 
    // HW not support
#endif    

    return status;
}
     
static void bq2589_set_charging(int value)
{
	mt_set_gpio_mode((GPIO55 | 0x80000000),GPIO_MODE_00);  
    mt_set_gpio_pull_enable((GPIO55 | 0x80000000), GPIO_PULL_DISABLE);
    mt_set_gpio_dir((GPIO55 | 0x80000000),GPIO_DIR_OUT);
    mt_set_gpio_out((GPIO55| 0x80000000),value);

	mt_set_gpio_mode((GPIO56 | 0x80000000),GPIO_MODE_00);  
    mt_set_gpio_pull_enable((GPIO56 | 0x80000000), GPIO_PULL_DISABLE);
    mt_set_gpio_dir((GPIO56 | 0x80000000),GPIO_DIR_OUT);
    mt_set_gpio_out((GPIO56| 0x80000000),value);

}

static kal_uint32 charging_enable(void *data)
{
     kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_FPGA)     
#else 
    kal_uint32 enable = *(kal_uint32*)(data);

    if(KAL_TRUE == enable)
    {
       bq2589_set_charging(0);
    }
    else
    {
       bq2589_set_charging(1);
    }
#endif
    
    return status;
}


static kal_uint32 charging_set_cv_voltage(void *data)
{
     kal_uint32 status = STATUS_OK;
     int volt=*(int *)data;

#if defined(CONFIG_MTK_FPGA)     
#else 
    //bq2589x_set_chargevoltage_external(volt);
#endif

    return status;
}     


static kal_uint32 charging_get_current(void *data)
{
    kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_FPGA)     
#else 
    // HW not support
#endif
    
    return status;
}  
  


static kal_uint32 charging_set_current(void *data)
{
     kal_uint32 status = STATUS_OK;
    // HW not support
    //bq25890_charging_curernt_set_external(*(int *)data
    bq2589x_charging_set_jeita_current(*(int *)data);

    return status;
}     


static kal_uint32 charging_set_input_current(void *data)
{
    kal_uint32 status = STATUS_OK;
    //bq2589x_charging_hw_inputcurrent_limit_external(*(int *)data);
    bq2589x_charging_set_jeita_input_limit(*(int *)data);
    return status;
}     

static kal_uint32 charging_get_charging_status(void *data)
{
   // kal_uint32 status = STATUS_OK;
    
    *(int *)data = bq25890_get_charging_status_external();

    return 0;
}

static void swchr_dump_register(void)
{
}



void set_cv_volt(void)
{
//fake charging
}

void swchr_hw_init(void)
{
}


static kal_uint32 charging_reset_watch_dog_timer(void *data)
{
    kal_uint32 status = STATUS_OK;
    
    return status;
}


static kal_uint32 charging_set_hv_threshold(void *data)
{
    kal_uint32 status = STATUS_OK;

    //HW Fixed value

    return status;
}


static kal_uint32 charging_get_hv_status(void *data)
{
      kal_uint32 status = STATUS_OK;
       
      return status;
}

        
static kal_uint32 charging_get_battery_status(void *data)
{
    kal_uint32 status = STATUS_OK;

#if 0
    //upmu_set_baton_tdet_en(1);    
    //upmu_set_rg_baton_en(1);
    //*(kal_bool*)(data) = upmu_get_rgs_baton_undet();
    *(kal_bool*)(data) = 0; // battery exist
    battery_xlog_printk(BAT_LOG_CRTI,"[charging_get_battery_status] no HW function\n");
#else
    kal_uint32 ret=0;

    pmic_config_interface(MT6332_BATON_CON0, 0x1, MT6332_PMIC_RG_BATON_EN_MASK, MT6332_PMIC_RG_BATON_EN_SHIFT);
    pmic_config_interface(MT6332_TOP_CKPDN_CON0_CLR, 0x80C0, 0xFFFF, 0); //enable BIF clock            
    pmic_config_interface(MT6332_LDO_CON2, 0x1, MT6332_PMIC_RG_VBIF28_EN_MASK, MT6332_PMIC_RG_VBIF28_EN_SHIFT);
    
    mdelay(1);
    ret = mt6332_upmu_get_bif_bat_lost();
    if(ret == 0)
    {
        *(kal_bool*)(data) = 0; // battery exist
        battery_xlog_printk(BAT_LOG_FULL,"[charging_get_battery_status] battery exist.\n");
    }
    else
    {
        *(kal_bool*)(data) = 1; // battery NOT exist
        battery_xlog_printk(BAT_LOG_CRTI,"[charging_get_battery_status] battery NOT exist.\n");
    }
#endif
   
    return status;
}


static kal_uint32 charging_get_charger_det_status(void *data)
{
	   kal_uint32 status = STATUS_OK;
 #if 0
	  // *(kal_bool*)(data) = upmu_get_rgs_chrdet();
#else
	kal_uint32 val=0,val_1=0;
    kal_uint32 is_m3_en=-1;
    
    mt6332_upmu_set_rg_precc_m3_en(1);
    mt6332_upmu_set_rg_m3_en(0);

	pmic_config_interface(0x10A, 0x1, 0xF, 8);
	pmic_config_interface(0x10A, 0x17,0xFF,0);
	pmic_read_interface(0x108,	 &val,0x1, 1);
	//*(kal_bool*)(data) = val;
    if (val == 1) // double check 
    {        
        val_1 = PMIC_IMM_GetOneChannelValue(AUX_VUSB_AP, 1, 1);         
        if(val_1 < 2500 || bq25890_get_vbus_type_external()==7)        
        {        
            battery_xlog_printk(BAT_LOG_CRTI,"CHRDET=%d but vchr=%d \n", val, val_1);
            val = 0;
        }
    }

	*(kal_bool*)(data) = val;
    
	battery_xlog_printk(BAT_LOG_CRTI,"[charging_get_charger_det_status][JJP][20140401] CHRDET status = %d\n", val);  
/*	if(val == 0)
		g_charger_type = CHARGER_UNKNOWN;*/
#endif
	   return status;
 }


kal_bool charging_type_detection_done(void)
{
     return charging_type_det_done;
}

static void ich_pre_init(void)
{
    kal_uint32 val=0;

    pmic_read_interface(0x8078,&val,0x1F,0);
    if(val==0)
    {
        //set ICH
        mt6332_upmu_set_rg_ich_sel_swen(1);

        if (g_charger_type == STANDARD_CHARGER || get_usb_current_unlimited()) 
            mt6332_upmu_set_rg_ich_sel(0xa);            
        else
            mt6332_upmu_set_rg_ich_sel(0x5);        
    }
    battery_xlog_printk(BAT_LOG_FULL,"[ich_pre_init] Reg[0x%x]=0x%x\n", 0x8078, upmu_get_reg_value(0x8078));
}

static kal_uint32 is_chr_det(void)
{
    kal_uint32 val=0;
    pmic_config_interface(0x10A, 0x1, 0xF, 8);
    pmic_config_interface(0x10A, 0x17,0xFF,0);
    pmic_read_interface(0x108,   &val,0x1, 1);

    battery_xlog_printk(BAT_LOG_CRTI,"[is_chr_det] %d\n", val);

    //ich pre-init
    //ich_pre_init();
    
    return val;
}

static kal_uint32 charging_get_charger_type(void *data)
{
    kal_uint32 status = STATUS_OK;
#if defined(CONFIG_POWER_EXT)
     *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else

    #if defined(CONFIG_MTK_WIRELESS_CHARGER_SUPPORT)
    int wireless_state = 0;
    if(wireless_charger_gpio_number!=0)
    {
        wireless_state = mt_get_gpio_in(wireless_charger_gpio_number);
        if(wireless_state == WIRELESS_CHARGER_EXIST_STATE)
        {
            *(CHARGER_TYPE*)(data) = WIRELESS_CHARGER;
            battery_xlog_printk(BAT_LOG_CRTI, "WIRELESS_CHARGER!\n");
            return status;
        }
    }
    else
    {
        battery_xlog_printk(BAT_LOG_CRTI, "wireless_charger_gpio_number=%d\n", wireless_charger_gpio_number);
    }
    
    if(g_charger_type!=CHARGER_UNKNOWN && g_charger_type!=WIRELESS_CHARGER)
    {
        *(CHARGER_TYPE*)(data) = g_charger_type;
        battery_xlog_printk(BAT_LOG_CRTI, "return %d!\n", g_charger_type);
        return status;
    }
    #endif

    if(is_chr_det()==0)
    {
        g_charger_type = CHARGER_UNKNOWN; 
        *(CHARGER_TYPE*)(data) = CHARGER_UNKNOWN;
        battery_xlog_printk(BAT_LOG_CRTI, "[charging_get_charger_type] return CHARGER_UNKNOWN\n");
        return status;
    }

    charging_type_det_done = KAL_FALSE;

    *(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();

    charging_type_det_done = KAL_TRUE;

    g_charger_type = *(CHARGER_TYPE*)(data);
    
#endif
    return status;    
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
     kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_MTK_FPGA)     
#else 
     if(slp_get_wake_reason() == WR_PCM_TIMER)
         *(kal_bool*)(data) = KAL_TRUE;
     else
         *(kal_bool*)(data) = KAL_FALSE;
 
     battery_xlog_printk(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif
        
     return status;
 }
 
 static kal_uint32 charging_set_platform_reset(void *data)
 {
     kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_MTK_FPGA)     
#else 
     #if !defined(CONFIG_ARM64)
     battery_xlog_printk(BAT_LOG_CRTI, "charging_set_platform_reset\n");  
     arch_reset(0,NULL);
     #else
     battery_xlog_printk(BAT_LOG_CRTI, "wait arch_reset ready\n"); 
     #endif
#endif
         
     return status;
 }
 
 static kal_uint32 charging_get_platfrom_boot_mode(void *data)
 {
     kal_uint32 status = STATUS_OK;
     
#if defined(CONFIG_MTK_FPGA)     
#else   
     *(kal_uint32*)(data) = get_boot_mode();
 
     battery_xlog_printk(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif
          
     return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_power_off=\n");
    //mt_power_off();
         
    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
	return STATUS_UNSUPPORTED;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_error_state(void *data)
{
        return STATUS_UNSUPPORTED;     
}


static kal_uint32 (* charging_func[CHARGING_CMD_NUMBER])(void *data)=
{
	charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
 };

 
 /*
 * FUNCTION
 *        Internal_chr_control_handler
 *
 * DESCRIPTION                                                             
 *         This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *        None
 *     
 * RETURNS
 *        
 *
 * GLOBALS AFFECTED
 *       None
 */
kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
     kal_int32 status;
     if(cmd < CHARGING_CMD_NUMBER)
         status = charging_func[cmd](data);
     else
         return STATUS_UNSUPPORTED;
 
     return status;
}
