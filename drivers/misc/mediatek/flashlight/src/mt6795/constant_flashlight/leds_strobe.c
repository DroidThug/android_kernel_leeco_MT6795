#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
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
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <mach/upmu_common.h>

#include <mach/mt_gpio.h>		// For gpio control

  #include <linux/wakelock.h>
/*
0  1  2  3   4   5   6   7   8   9   10  11  12  13
25 50 75 100 125 150 300 400 500 600 700 800 900 1000
*/
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
#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)

/*#define DEBUG_LEDS_STROBE*/
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_DBG_FUNC
	#define PK_ERR PK_DBG_FUNC
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


#define STROBE_DEVICE_ID 0xA6


static struct work_struct workTimeOut;

#define FLASH_GPIO_EN GPIO15
//#define FLASH_GPIO_ENT GPIO13



/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

int readReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
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

static int isMovieMode[e_DutyNum] = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0};
#ifdef TORCH_31MA
static int torchDuty[e_DutyNum]=    {0,3,5,7,0,0,0,0,0,0,0,0,0,0,0,0};
//31,  125, 187, 250ma
#else
static int torchDuty[e_DutyNum]=    {1,3,5,7,0,0,0,0,0,0,0,0,0,0,0,0};
//62,  125, 187, 250ma
#endif

static int flashDuty[e_DutyNum]=     {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
//62, 125, 187, 250, 312, 375, 437, 500, 562, 625, 687, 750, 812, 975, 937, 1000ma
int m_duty1=0;
int m_duty2=0;
int LED1CloseFlag = 1;
int LED2CloseFlag = 1;


int flashEnable_LM3560_2(void)
{
	
	int err;
	int enable_value = 0,led1_enable = 0,led2_enable = 0;

	PK_DBG(" flashDisable_LM3560_2 S line=%d\n",__LINE__);

	enable_value = readReg(0x10);
	PK_DBG(" LED1&2_enable_value S =0x%x\n",enable_value);

	if((LED1CloseFlag == 1) && (LED2CloseFlag == 1))
	{
		err = writeReg(0x10, (enable_value & 0xE4));
		return 0;
	}

	if(LED1CloseFlag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
			err = writeReg(0x10, (enable_value & 0xE4) | 0x12);
		else
			err = writeReg(0x10, (enable_value & 0xE4) | 0x13);
	}
	else if(LED2CloseFlag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
			err = writeReg(0x10, (enable_value & 0xE4) | 0x0A);
		else
			err = writeReg(0x10, (enable_value & 0xE4) | 0x0B);
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && (isMovieMode[m_duty2] == 1))
			err = writeReg(0x10, (enable_value & 0xE4) | 0x1A);
		else
			err = writeReg(0x10, (enable_value & 0xE4) | 0x1B);			
	}

	enable_value = readReg(0x10);
	PK_DBG(" LED1&2_enable_value E =0x%x\n",enable_value);
	
	PK_DBG(" flashDisable_LM3560_2 E line=%d\n",__LINE__);
	return err;

}
int flashDisable_LM3560_2(void)
{
	flashEnable_LM3560_2();
}


int setDuty_LM3560_2(int duty)
{

	int err;
	int led1_duty = 0, duty_Value;
	int temp;
	PK_DBG(" setDuty_LM3560_2 S line=%d\n",__LINE__);

	//mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ONE);
	//if(duty<0)
	//	duty=0;
	//else if(duty>=e_DutyNum)
	//	duty=e_DutyNum-1;
	//m_duty2=duty;

	if((LED1CloseFlag == 1) && (LED2CloseFlag == 1))
	{
			
	}
	else if(LED1CloseFlag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
		{
			temp = readReg(0xA0);
			PK_DBG(" LED2_torch_register S =0x%x\n",temp);			
			err =  writeReg(0xA0, (temp & 0xC7)|(torchDuty[m_duty2] << 3));			
			temp = readReg(0xA0);
			PK_DBG(" LED2_torch_register E =0x%x\n",temp);
		}
		else
		{
			temp = readReg(0xB0);
			PK_DBG(" LED2_flash_register S =0x%x\n",temp);			
			err =  writeReg(0xB0, (temp & 0x0F)|(flashDuty[m_duty2] << 4));			
			temp = readReg(0xB0);
			PK_DBG(" LED2_flash_register E =0x%x\n",temp);
		}
	}
	else if(LED2CloseFlag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
		{
			temp = readReg(0xA0);
			PK_DBG(" LED1_torch_register S =0x%x\n",temp);			
			err =  writeReg(0xA0, (temp & 0xF8) | torchDuty[m_duty1]);		
			temp = readReg(0xA0);
			PK_DBG(" LED1_torch_register E =0x%x\n",temp);

		}
		else
		{
			temp = readReg(0xB0);
			PK_DBG(" LED1_flash_register S =0x%x\n",temp);			
			err =  writeReg(0xB0, (temp & 0xF0) | flashDuty[m_duty1]);			
			temp = readReg(0xB0);
			PK_DBG(" LED1_flash_register E =0x%x\n",temp);

		}		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1)&&((isMovieMode[m_duty2] == 1)))
		{
			temp = readReg(0xA0);
			PK_DBG(" LED1&2_torch_register S =0x%x\n",temp);			
			err =  writeReg(0xA0, (temp & 0xC0)|(torchDuty[m_duty2] << 3)|(torchDuty[m_duty1]));		
			temp = readReg(0xA0);
			PK_DBG(" LED1&2_torch_register E =0x%x\n",temp);			
		}
		else
		{
			temp = readReg(0xB0);
			PK_DBG(" LED1&2_flash_register S =0x%x\n",temp);
			err =  writeReg(0xB0, (flashDuty[m_duty2] << 4)|(flashDuty[m_duty1]));
			temp = readReg(0xB0);
			PK_DBG(" LED1&2_flash_register E =0x%x\n",temp);		
		}
	}	

	PK_DBG(" setDuty_LM3560_2 E line=%d\n",__LINE__);

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
	int err;
	int led2_duty = 0, duty_Value= 0;
	PK_DBG(" setDuty_LM3560_1 S line=%d\n",__LINE__);
	//mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ONE);
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;


	PK_DBG(" setDuty_LM3560_1 E line=%d\n",__LINE__);
	return err;
}

int init_LM3560()
{
	int err;
	
	PK_DBG(" init_LM3560 S line=%d\n",__LINE__);
	mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ONE);
	err =  writeReg(0x10, 0x00);
	//err =  writeReg(0x11, 0);
    //err =  writeReg(0x12, 0);
	//err =  writeReg(0xC0, 0x52);

	PK_DBG(" init_LM3560 E line=%d\n",__LINE__);
	return err;
}


int FL_Enable(void)
{
    flashEnable_LM3560_1();
	PK_DBG(" FL_Enable line=%d\n",__LINE__);


    return 0;
}



int FL_Disable(void)
{
    flashDisable_LM3560_1();
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_LM3560_1(duty);

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
    init_LM3560();

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	//FL_Disable();
	mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ZERO);
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
	PK_DBG("LM3560 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);

			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
    		m_duty1 = arg;
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
				LED1CloseFlag = 0;
    			FL_Enable();
    		}
    		else
    		{
    			LED1CloseFlag = 1;
    			FL_Disable();
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
	    		i4RetValue = m_duty1;
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


