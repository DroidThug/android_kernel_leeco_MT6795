#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(PFX fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   pr_debug(PFX fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif

extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{

u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4
#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


u32 pinSet[3][8] = {
                        //for main sensor
                     {  CAMERA_CMRST_PIN,
                        CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,              /* ON state */
                        GPIO_OUT_ZERO,             /* OFF state */
                        CAMERA_CMPDN_PIN,
                        CAMERA_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for sub sensor
                     {  CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     },
                     //for main_2 sensor
                     {   CAMERA_CMRST1_PIN,
                        CAMERA_CMRST1_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,               /* ON state */
                        GPIO_OUT_ZERO,              /* OFF state */
                        CAMERA_CMPDN1_PIN,
                        CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     }
                   };

    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

   
    //power ON
    if (On) {
        if(pinSetIdx == 0 ) {
            ISP_MCLK1_EN(1);
        }
        else if (pinSetIdx == 1) {
            ISP_MCLK3_EN(1);
        }
        else if (pinSetIdx == 2) {
            ISP_MCLK2_EN(1);
        }

	if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX214_MIPI_RAW, currSensorName)))
	{
		//VDD3G
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_3G, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		 //   mdelay(10);
		//VDDAF
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    //mdelay(30);
		    mdelay(1);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}

	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX214_SHARP_MIPI_RAW, currSensorName)))
	{
		//VDD3G
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_3G, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		 //   mdelay(10);
		//VDDAF
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    //mdelay(30);
		    mdelay(1);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}

	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX214_OFILM_MIPI_RAW, currSensorName)))
	{
		//VDD3G
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_3G, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		 //   mdelay(10);
		//VDDAF
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    //mdelay(30);
		    mdelay(1);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}

	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K3M2_MIPI_RAW, currSensorName)))
	{
		//VDD3G
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_3G, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		 //   mdelay(10);
		//VDDAF
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_3000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    //mdelay(30);
		    mdelay(1);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}

	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName)))
	{
		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1200,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    mdelay(10);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}
	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_SHARP_MIPI_RAW, currSensorName)))
	{
		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1200,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    mdelay(10);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}
	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_OFILM_MIPI_RAW, currSensorName)))
	{
		//AVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//DVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1200,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//IOVDD
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
		    mdelay(10);
		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(20);

		}
	}
	else
	{	//VCAM_IO
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, VOL_1800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    goto _kdCISModulePowerOn_exit_;
		}

		//VCAM_A
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    goto _kdCISModulePowerOn_exit_;
		}
	 
		//DVDD
		if (currSensorName && (0 == strcmp(currSensorName,"imx135mipiraw"))) 
		{
		    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1000,mode_name))
		    {
		         PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		         goto _kdCISModulePowerOn_exit_;
		    }
		}
		else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName)))
		{
		    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1500,mode_name))
		    {
		         PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		         goto _kdCISModulePowerOn_exit_;
		    }
		}            
		else {
		    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))
		    {
		         PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		         goto _kdCISModulePowerOn_exit_;
		    }
		
		}
	 
		//AF_VCC
		if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A2, VOL_2800,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		//enable active sensor
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(10);
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
		    mdelay(1);

		    //PDN pin
		    if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
		    if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
		    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
		}
	}
    }
    else {//power OFF
        if(pinSetIdx == 0 ) {
            ISP_MCLK1_EN(0);
        }
        else if (pinSetIdx == 1) {	
            ISP_MCLK3_EN(0);
        }
        else if (pinSetIdx == 2) {
            ISP_MCLK2_EN(0);
        }
	mdelay(10);
        //PK_DBG("[OFF]sensorIdx:%d \n",SensorIdx);
        /*if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
	mdelay(10);*/
	if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_IMX214_MIPI_RAW, currSensorName)))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_3G,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
	}
	else if(0 == strcmp(SENSOR_DRVNAME_IMX214_SHARP_MIPI_RAW, currSensorName))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_3G,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
	}
	else if(0 == strcmp(SENSOR_DRVNAME_IMX214_OFILM_MIPI_RAW, currSensorName))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_3G,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
	}
	else if(0 == strcmp(SENSOR_DRVNAME_S5K3M2_MIPI_RAW, currSensorName))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_3G,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW, currSensorName)))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_SHARP_MIPI_RAW, currSensorName)))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

	}
	else if(currSensorName && (0 == strcmp(SENSOR_DRVNAME_S5K5E2YA_OFILM_MIPI_RAW, currSensorName)))
	{
	mdelay(10);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            //if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            //if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            //if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}

	}
	else
	{
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }
	mdelay(10);
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A2,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) {
		    PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2,mode_name))
		{
		    PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
		    //return -EIO;
		    goto _kdCISModulePowerOn_exit_;
		}
	}
    }//

    return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
    
}

EXPORT_SYMBOL(kdCISModulePowerOn);

//!--
//


