#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "SY7804"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xC6


static struct work_struct workTimeOut;

#define FLASH_GPIO_EN GPIO15
//#define FLASH_GPIO_ENT GPIO13



/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);
extern int read_reg_byte_data(u8 * a_pSendData,u16 a_sizeSendData,u8 * a_pRecvData,u16 a_sizeRecvData,u16 i2cId);
int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
   //iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    read_reg_byte_data(buf,1,bufR,1,STROBE_DEVICE_ID);
    PK_DBG("readReg reg=0x%x val=0x%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}

int writeReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
    PK_DBG("writeReg reg=0x%x val=0x%x qq\n", buf[0],buf[1]);
    iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);

   return 0;
}
enum
{
	e_DutyNum = 16,
};
static int g_duty=0;
static int isMovieMode[e_DutyNum] = {1,0,0,0,0,0,0,0,0};

static int torchDuty[e_DutyNum] = {1,0,0,0,0,0,0,0,0};

//static int flashDuty[e_DutyNum] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
//62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 975, 937, 1000ma
static int flashDuty[e_DutyNum] = {0,1,2,3,4,5,6,7,8};//max 843mA
int m_duty1=0;
int m_duty2=0;
int LED1CloseFlag = 1;
int LED2CloseFlag = 1;


int flashEnable_LM3560_2(void)
{
	
	int err = 0;

	return err;

}
int flashDisable_LM3560_2(void)
{
	flashEnable_LM3560_2();
}


int setDuty_LM3560_2(int duty)
{

	int err = 0;
	
	return err;

}



int flashEnable_LM3560_1(void)
{
	return 0;
}
int flashDisable_LM3560_1(void)
{
	return 0;
}


int setDuty_LM3560_1(int duty)
{
	int err = 0;

	return err;
}

int init_LM3560()
{
	int err = 0;

	return err;
}


//---------------------------------------------------------//
	/*FOR SY7804 DRIVER IC*/
//---------------------------------------------------------//
int flashEnable_SY7804(void)
{
	int i = 0;
	int err = 0;
	int temp_duty = 0;

	temp_duty = readReg(0x0A);
	PK_DBG("temp_duty: %d\n",temp_duty);
	
	temp_duty &= 0xFC;

	PK_DBG("temp_duty11: %d\n",temp_duty);
	if(g_duty == 0)
	{
		err = writeReg(0x0A, 0x02);
	}
	else
	{
		err = writeReg(0x0A, 0x03);
	}
	return 0;
}
int flashDisable_SY7804(void)
{
	int err = 0;
	err = writeReg(0x0A, 0x00);
	readReg(0x0A);
	readReg(0x0B);
	readReg(0x01);
	return 0;
}


int setDuty_SY7804(int duty)
{
	int err = 0;
	g_duty = duty;

	if(g_duty == 0)
	{
		err = writeReg(0x09, 0x10);
	}
	else
	{
		err = writeReg(0x09, flashDuty[g_duty]);
	}
	return err;
}

int init_SY7804()
{
	int err = 0;
	PK_DBG(" init_SY7804 S line=%d\n",__LINE__);
	//mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ONE);
	mt_set_gpio_mode(FLASH_GPIO_EN,GPIO_MODE_GPIO);
	mt_set_gpio_dir(FLASH_GPIO_EN,GPIO_DIR_OUT);
    mt_set_gpio_pull_enable(FLASH_GPIO_EN, GPIO_PULL_DISABLE);
	mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO41,GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO41,GPIO_DIR_OUT);
    mt_set_gpio_pull_enable(GPIO41, GPIO_PULL_DISABLE);
	mt_set_gpio_out(GPIO41,GPIO_OUT_ZERO);	
	err = writeReg(0x0A, 0x00);
	err = writeReg(0x08, 0x55);
	readReg(0X0A);
	readReg(0x0B);
	readReg(0x01);
	PK_DBG(" init_SY7804 E line=%d\n",__LINE__);
	return err;
}



int FL_Enable(void)
{
    flashEnable_SY7804();
	PK_DBG(" FL_Enable line=%d\n",__LINE__);


    return 0;
}



int FL_Disable(void)
{
    flashDisable_SY7804();
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_SY7804(duty);

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
    init_SY7804();

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	//mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ZERO);
    PK_DBG(" FL_UnInit line=%d\n",__LINE__);
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static struct hrtimer g_timeOutTimer;
//static struct hrtimer g_WDResetTimer;
static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	PK_DBG("ledTimeOut_callback\n");
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
void timerInit(void)
{
	static int g_b1stInit=1;
	if(g_b1stInit==1)
    {
		g_b1stInit=0;
  INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

	}
}

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("SY7804 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);

			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		FL_dim_duty(arg);
    		break;


    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
    			FL_Enable();
				LED1CloseFlag = 0;
    		}
    		else
    		{
    			FL_Disable();
				LED1CloseFlag = 1;
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
    	    break;
			
    	case FLASH_IOC_GET_FLASH_DUTY:

			if(LED1CloseFlag == 1)
			{
	    		i4RetValue = -1;
			}
			else
			{
	    		i4RetValue = g_duty;
			}
    	    break;
			
    	case FLASH_IOC_GET_FLASH_ONOFF_STATUS:
			
			if(LED1CloseFlag == 1)
			{
	    		i4RetValue = 0;
			}
			else
			{
	    		i4RetValue = 1;
			}
    		break;


		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


