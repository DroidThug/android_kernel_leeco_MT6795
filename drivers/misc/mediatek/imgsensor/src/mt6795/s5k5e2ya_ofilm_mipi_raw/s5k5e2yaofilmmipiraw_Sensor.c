/* *****************************************************************************
 *
 * Filename:
 * ---------
 *	 s5k5e2yamipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k5e2yaofilmmipiraw_Sensor.h"

/****************************Modify following Strings for debug****************************/
#define PFX "s5k5e2ya_ofilm_camera_sensor"

#define __OFILM_OTP__
#define __OFILM_OTP_WB__   //enable after wb golden value is checked

#ifdef __OFILM_OTP__

//#include "s5k5e2yamipiraw_Camera_Sensor_para.h"
//#include "s5k5e2yamipiraw_CameraCustomized.h"


#define S5K5E2YA_OFILM_DEBUG
#ifdef S5K5E2YA_OFILM_DEBUG
#define LOG_TAG (__FUNCTION__)
#define SENSORDB(fmt,arg...) xlog_printk(ANDROID_LOG_DEBUG , LOG_TAG, fmt, ##arg)  							//printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSORDB(fmt,arg...)  
#endif

#define USHORT unsigned short
#define BYTE unsigned char
#define Sleep(ms) mdelay(ms)

#define OFILM_INVALID_OTP        0x01

#define OFILM_GAIN_DEFAULT       0x0100
#define OFILM_GAIN_GREEN1_ADDR   0x020E
#define OFILM_GAIN_BLUE_ADDR     0x0212
#define OFILM_GAIN_RED_ADDR      0x0210
#define OFILM_GAIN_GREEN2_ADDR   0x0214

#define OFILM_GRG_TYPICAL   0x35B        //important, should be chencked from module test data
#define OFILM_BG_TYPICAL    0x2D2        //important, should be chencked from module test data

static kal_uint32 OFILM_tRG_Ratio_typical = OFILM_GRG_TYPICAL;
static kal_uint32 OFILM_tBG_Ratio_typical = OFILM_BG_TYPICAL;

struct S5K5E2YA_OFILM_MIPI_otp_struct
{
    kal_uint16 customer_id;
    kal_uint16 module_integrator_id;
    kal_uint16 lens_id;
    kal_uint16 rg_ratio;
    kal_uint16 bg_ratio;
    kal_uint16 user_data[5];
    kal_uint16 R_to_G;
    kal_uint16 B_to_G;
    kal_uint16 G_to_G;
    kal_uint16 R_Gain;
    kal_uint16 G_Gain;
    kal_uint16 B_Gain;
};

static kal_uint16 s5k5e2yamipiraw_ofilm_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, 0x20);

	return get_byte;
}

static void s5k5e2yamipiraw_ofilm_write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, 0x20);
}

/*************************************************************************************************
* Function    :  ofilm_start_read_otp
* Description :  before read otp , set the reading block setting  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  0, reading block setting err
                 1, reading block setting ok 
**************************************************************************************************/
static bool ofilm_start_read_otp(BYTE zone)
{
	BYTE val = 0;
	int i;
   	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x04);//make initial state
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, zone);   //Select the page to write by writing to 0xD0000A02 0x00~0x0F
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x01);   //Enter read mode by writing 01h to 0xD0000A00
	
    Sleep(2);//wait time > 47us

	return 1;
}

/*************************************************************************************************
* Function    :  ofilm_stop_read_otp
* Description :  after read otp , stop and reset otp block setting  
**************************************************************************************************/
static void ofilm_stop_read_otp()
{
   	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x04);//make initial state
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x00);//Disable NVM controller
}

/*************************************************************************************************
* Function    :  ofilm_get_otp_AWB_flag
* Description :  get otp AWB_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
static BYTE ofilm_get_otp_AWB_flag(BYTE zone)
{
    BYTE flag_AWB = 0x00;
    BYTE Flag1=0,Flag0=0;
    
    if(zone>3||zone<2)
    {
    	SENSORDB("page error \n");
    	return flag_AWB;
    }
    ofilm_start_read_otp(zone);
    
    
    Flag0=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A23);
    Flag1=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A43);
    
    
    flag_AWB = Flag0|Flag1;
    
    ofilm_stop_read_otp();

    SENSORDB("flag_AWB[page %d] := 0x%02x \n", zone, flag_AWB );
	return flag_AWB;
}

/*************************************************************************************************
* Function    :  ofilm_get_otp_LSC_flag
* Description :  get otp LSC_WRITTEN_FLAG  
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE], if 0x00 , this type has valid or empty otp data, otherwise, invalid otp data
**************************************************************************************************/
static BYTE ofilm_get_otp_LSC_flag(BYTE zone)
{
    BYTE flag_LSC = 0x00;
    ofilm_start_read_otp(zone);
    flag_LSC = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A43);
    ofilm_stop_read_otp();

    SENSORDB("page[%d] flag_LSC := 0x%02x \n", zone,flag_LSC );
	return flag_LSC;
}

/*************************************************************************************************
* Function    :  ofilm_get_otp_module_id
* Description :  get otp MID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : module ID data , TRULY ID is 0x0001            
**************************************************************************************************/
static int ofilm_get_otp_module_id(int *id)
{
	kal_uint16 Temp=0;
	kal_uint16 PageCount;
	kal_uint16 GroupFlag=0,GroupFlag0=0,GroupFlag1=0;
	
	for(PageCount = 3; PageCount>=2; PageCount--)
	{
		SENSORDB("[S5K5E2YA_OFILM] [ofilm_get_otp_module_id] PageCount=%d\n", PageCount);	
		
		if(PageCount>2)
		{//group 4,3
			//otp start control set 
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, PageCount); // page 3
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
			mdelay(5);//delay 5ms
			
			GroupFlag1 =0;
			GroupFlag0 =0;
			GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
			GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
	
			if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
			{
				if(GroupFlag1 ==0x01)
				{	//group 4 module ID
					Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a24);

					GroupFlag =4;//page 3 ->group 4,3
				}
				else
				{
					//group 3 module ID
					Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a04);
						
					GroupFlag =3;//page 3 ->group 4,3
				}
				break; //
			}
		}
		else
		{//group 2,1
			//otp start control set 
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
		  	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, PageCount); //page 2
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
			mdelay(5);//delay 5ms
			
			GroupFlag1 =0;
			GroupFlag0 =0;
			GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
			GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
			
			if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
			{
				if(GroupFlag1 ==0x01)
				{	//group 2 module ID
					Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a24);

					GroupFlag =2;//page 2 ->group 2,1
				}
				else
				{
					//group 1 module ID
					Temp=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a04);

					GroupFlag =1;//page 2 ->group 2,1
				}
					
				break;
			}	
		}
		//otp end control set 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x00); 
	}
	
    *id =(Temp&0xff);
    SENSORDB("Module ID = 0x%02x.\n",*id);
	
	if(GroupFlag==0)
	{
	    SENSORDB("ofilm_get_otp_module_id error \n");
	    return -1;
	}
	
	return 0; 
}


/*************************************************************************************************
* Function    :  ofilm_get_otp_date
* Description :  get otp date value    
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f    
**************************************************************************************************/
static bool ofilm_get_otp_date(BYTE zone) 
{
	BYTE year  = 0;
	BYTE month = 0;
	BYTE day   = 0;

	ofilm_start_read_otp(zone);

	year  = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A05);
	month = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A06);
	day   = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A07);

	ofilm_stop_read_otp();

    SENSORDB("date=%02d.%02d.%02d \n", year,month,day);

	return 1;
}



/*************************************************************************************************
* Function    :  ofilm_get_otp_lens_id
* Description :  get otp LENS_ID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : LENS ID data             
**************************************************************************************************/
static BYTE ofilm_get_otp_lens_id(BYTE zone)
{
	BYTE lens_id = 0;

	ofilm_start_read_otp(zone);
	
	lens_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A08);
	ofilm_stop_read_otp();

    SENSORDB("Lens ID = 0x%02x.\n",lens_id);

	return lens_id;
}


/*************************************************************************************************
* Function    :  ofilm_get_otp_vcm_id
* Description :  get otp VCM_ID value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : VCM ID data             
**************************************************************************************************/
static BYTE ofilm_get_otp_vcm_id(BYTE zone)
{
	BYTE vcm_id = 0;

	ofilm_start_read_otp(zone);
	
	vcm_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A09);

	ofilm_stop_read_otp();

    SENSORDB("VCM ID = 0x%02x.\n",vcm_id);

	return vcm_id;	
}


/*************************************************************************************************
* Function    :  ofilm_get_otp_driverIC_id
* Description :  get otp driverIC id value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                 other value : driver ID data             
**************************************************************************************************/
static BYTE ofilm_get_otp_driverIC_id(BYTE zone)
{
	BYTE driverIC_id = 0;

	ofilm_start_read_otp(zone);
	
	driverIC_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0A);

	ofilm_stop_read_otp();

	SENSORDB("Driver ID = 0x%02x.\n",driverIC_id);

	return driverIC_id;
}

/*************************************************************************************************
* Function    :  ofilm_get_light_id
* Description :  get otp environment light temperature value 
* Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f
* Return      :  [BYTE] 0 : OTP data fail 
                        other value : driver ID data     
			            BIT0:D65(6500K) EN
						BIT1:D50(5100K) EN
						BIT2:CWF(4000K) EN
						BIT3:A Light(2800K) EN
**************************************************************************************************/
static BYTE ofilm_get_light_id(BYTE zone)
{
	BYTE light_id = 0;

	ofilm_start_read_otp(zone);	

	light_id = s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0A0B);

	ofilm_stop_read_otp();

	SENSORDB("Light ID: 0x%02x.\n",light_id);

	return light_id;
}


/*************************************************************************************************
* Function    :  ofilm_otp_lenc_update
* Description :  Update lens correction 
* Parameters  :  
* Return      :  [bool] 0 : OTP data fail 
                        1 : otp_lenc update success            
**************************************************************************************************/
static bool ofilm_otp_lenc_update()
{
    BYTE  flag_LSC_page = 0;
    unsigned int  flag_LSC = 0;
    BYTE  temp = 0;
    flag_LSC_page = ofilm_get_otp_LSC_flag(15);
    //flag_LSC_page3 = ofilm_get_otp_LSC_flag(3);
    //flag_LSC = (flag_LSC_page2 | flag_LSC_page2);
	
    //
    if(1 == flag_LSC_page)
    {
        //LSC correct
        SENSORDB("LSC Auto Correct OK! \n");
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0B00,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3400,0x00);
    }
    //Load LSC SRAM data
    else
    {
        SENSORDB("LSC Load SRAM Data!!!!! \n");
        //Load
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x340f,0x81); //Truly Add 20140218 start
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0b00,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3400,0x00);
        Sleep(10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3457,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3b4c,0x00);//Truly Add 20140218 end

        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x56);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xeb);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x55);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x02);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x6b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xbf);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x03);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x19);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe1);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x97);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x05);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x06);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xbf);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfc);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x07);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x9c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x32);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x08);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x5c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x93);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x09);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xfb);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb2);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x7d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x22);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x42);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x5c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc2);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfc);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x95);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x96);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x71);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x8f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x24);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x11);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdc);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xdd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x12);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x13);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x97);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x7c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x14);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x99);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x16);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x15);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x88);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfb);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x16);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x32);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x17);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x76);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x18);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdf);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x19);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xde);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xfe);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xed);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc0);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x44);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xdf);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x99);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x68);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x1f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x6d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x2f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x20);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x6a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xad);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x21);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x51);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x22);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x28);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x9f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x23);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x79);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x24);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x25);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x9b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xed);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x26);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x27);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xba);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x61);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x28);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xd4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x29);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x9e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x84);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x84);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x96);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x1e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x48);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x15);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xde);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x2f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x2b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xda);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x30);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xbe);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x12);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x31);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x9b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x32);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x33);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x33);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x74);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xea);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x34);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x84);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x17);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x35);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x51);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x36);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x67);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x76);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x37);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x38);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x20);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x39);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x61);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x71);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x34);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x47);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x69);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xcf);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x13);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x09);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x11);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x3f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x01);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x88);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xed);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xef);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x41);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x62);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x42);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x43);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x44);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x9c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x20);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x45);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x67);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x46);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x17);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x47);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc1);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x7a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x48);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xeb);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x8a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x49);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x41);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x71);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf0);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x90);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc1);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x4f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x50);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x31);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x7b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x51);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x2c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x52);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x4f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x27);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x53);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x1c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x54);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x5e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x55);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x22);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x56);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe1);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x57);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x66);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x58);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x4b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x59);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xea);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xcd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc1);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x49);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x5b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x2f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x5f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xee);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x60);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xfd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x28);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x61);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x85);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x62);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x06);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x32);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x63);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x22);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x79);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x64);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x0b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x65);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x5c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x66);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x2c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x67);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x82);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc2);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x68);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x7d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x69);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x71);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x5d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x7c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc7);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x9e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xeb);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xbf);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x77);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xd8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x6f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xc5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x70);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x71);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xa8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x06);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x72);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xbc);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x8c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x73);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb1);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x96);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x74);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x35);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x21);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x75);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x1f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xfd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x76);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x6f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x83);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x77);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x3a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe0);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x78);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x60);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x54);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x79);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x21);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xa4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xc9);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x05);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x55);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x91);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x07);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x63);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xe2);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x09);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x84);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x7f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xb2);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xb5);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x80);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x74);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x71);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x81);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x7a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x73);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x82);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf6);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x94);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x83);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x36);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x2b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x84);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xdc);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x4e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x85);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xf8);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xf3);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x86);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x19);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x6f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x87);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x02);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x31);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x88);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x10);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x89);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0xcd);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x62);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x69);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x92);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x79);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x3b);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8c);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x50);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x2a);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8d);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x60);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0xe4);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3415,0x0f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3416,0x2f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3417,0x1e);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3418,0x8f);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3419,0x04);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341A,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341B,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341C,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341D,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341E,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x341F,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3420,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3421,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3422,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3423,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3424,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3425,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3426,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3427,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3428,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3429,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342A,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342B,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342C,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342D,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342E,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x342F,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3430,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3431,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3432,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3433,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3434,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3435,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3436,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3437,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3438,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3439,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343A,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343B,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343C,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343D,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343E,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x343F,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3440,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3441,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3442,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3443,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3444,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3445,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3446,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3447,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3448,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3449,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344A,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344B,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344C,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344D,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344E,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x344F,0x00);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3450,0x40);
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3451,0x00);

        //Add More
        s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x3457,0x0c);
        Sleep(10);
    }
    SENSORDB("OTP LSC Update Finished! \n");
    return 1;
}

#ifdef __OFILM_OTP_WB__

static int S5K5E2YA_OFILM_MIPI_read_otp_wb(struct S5K5E2YA_OFILM_MIPI_otp_struct *otp)
{
	kal_uint32 i,page;
	kal_uint32 R_to_G, B_to_G, G_to_G;
	kal_uint16 PageCount;
	kal_uint16 GroupFlag=0,GroupFlag0=0,GroupFlag1=0;
	kal_uint16 H_Byte=0,L_Byte=0;
	kal_uint16 RValue_C=0,GValue_C=0,BValue_C=0,RValue_G=0,GValue_G=0,BValue_G=0;
	
#if 0
for(page=2;page<=0x0f;page++)
{
	ofilm_start_read_otp(page);
	SENSORDB("page=%x\n", page);	//page 0,1,6,7 reserve
	for(i=0xa04;i<=0xa43;i++)
	{
	        SENSORDB(" TTTTT i=%x value=%x\n",i,s5k5e2yamipiraw_ofilm_read_cmos_sensor(i));	
	}
	ofilm_stop_read_otp();
}
	
#endif

	for(PageCount = 3; PageCount>=2; PageCount--)
	{
        SENSORDB("PageCount=%d\n", PageCount);	
		
		if(PageCount>2)
		{//group 4,3
			//otp start control set 
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, 0X03); // page 3
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
			mdelay(5);//delay 5ms
			
			GroupFlag1 =0;
			GroupFlag0 =0;
			GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
			GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
			
#if 0//xg test

			for(i=0xa04;i<=0xa43;i++)
			{
			    SENSORDB("xg_testpage 3 i=%x value=%x\n",i,s5k5e2yamipiraw_ofilm_read_cmos_sensor(i));	
			}
#endif

			if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
			{
				if(GroupFlag1 ==0x01)
				{	//?̡????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2B);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2C);
					RValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2D);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2E);
					BValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2F);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a30);
					GValue_C =(H_Byte<<8)+L_Byte;
					
					//???????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a32);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a33);
					RValue_G =(H_Byte<<8)+L_Byte;

					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a34);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a35);
					BValue_G =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a36);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a37);
					GValue_G =(H_Byte<<8)+L_Byte;

					GroupFlag =4;
				}
				else
				{
						//?̡????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0B);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0C);
					RValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0D);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0E);
					BValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0F);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a10);
					GValue_C =(H_Byte<<8)+L_Byte;
					
					//???????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a12);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a13);
					RValue_G =(H_Byte<<8)+L_Byte;

					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a14);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a15);
					BValue_G =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a16);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a17);
					GValue_G =(H_Byte<<8)+L_Byte;
						
					GroupFlag =3;
				}
				break; //
			}
			
		}
		else
		{//group 2,1
			//otp start control set 
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
		    s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A02, 0X02); //page 2
			s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X01);
			 
			mdelay(5);//delay 5 ms
			GroupFlag1 =0;
			GroupFlag0 =0;
			GroupFlag1 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a43);
			GroupFlag0 =s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a23);
#if 0//xg test

			for(i=0xa04;i<=0xa43;i++)
			{
			SENSORDB("xg_testpage 2 i=%x value=%x\n",i,s5k5e2yamipiraw_ofilm_read_cmos_sensor(i));	
			}
#endif
			if((GroupFlag0 ==0x01) || (GroupFlag1 ==0x01))
			{
				if(GroupFlag1 ==0x01)
				{	//?̡????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2B);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2C);
					RValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2D);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2E);
					BValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a2F);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a30);
					GValue_C =(H_Byte<<8)+L_Byte;
					
					//???????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a32);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a33);
					RValue_G =(H_Byte<<8)+L_Byte;

					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a34);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a35);
					BValue_G =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a36);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a37);
					GValue_G =(H_Byte<<8)+L_Byte;

					GroupFlag =2;
				}
				else
				{
						//?̡????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0B);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0C);
					RValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0D);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0E);
					BValue_C =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a0F);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a10);
					GValue_C =(H_Byte<<8)+L_Byte;
					
					//???????????????????? 
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a12);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a13);
					RValue_G =(H_Byte<<8)+L_Byte;

					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a14);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a15);
					BValue_G =(H_Byte<<8)+L_Byte;
					
					H_Byte=0;
					L_Byte=0;
					H_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a16);
					L_Byte=s5k5e2yamipiraw_ofilm_read_cmos_sensor(0x0a17);
					GValue_G =(H_Byte<<8)+L_Byte;
						
					GroupFlag =1;
				}
						
				break;
			}	
		}
		//otp end control set 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0X04); 
		s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0A00, 0x00); 
	}

	if((GroupFlag==0)||(RValue_C==0)||(GValue_C==0)||(GValue_C==0))
	{
		SENSORDB("otp AWB ERROR!");
		otp->R_to_G = OFILM_tRG_Ratio_typical;
		otp->B_to_G = OFILM_tBG_Ratio_typical;
		otp->G_to_G = 0x400;
		return -1;
	}
	else
	{	
		otp->R_to_G=(RValue_C*1024/GValue_C);
		otp->B_to_G=(BValue_C*1024/GValue_C);
		otp->G_to_G = 0x400;	
	}
		
	

    SENSORDB("GrouFlag =%d, otp->R_to_G=0x%x, otp->B_to_G=0x%x, otp->G_to_G=0x%x \n"
            ,GroupFlag
            ,otp->R_to_G
            ,otp->B_to_G
            ,otp->G_to_G);
    return 0;
}

static void S5K5E2YA_OFILM_MIPI_algorithm_otp_wb1(struct S5K5E2YA_OFILM_MIPI_otp_struct *otp)
{
	kal_uint32 R_to_G, B_to_G, G_to_G;
	kal_uint32 R_Gain, B_Gain, G_Gain;
	kal_uint32 G_gain_R, G_gain_B;
	
	R_to_G = otp->R_to_G;
	B_to_G = otp->B_to_G;
	G_to_G = otp->G_to_G;

    SENSORDB("R_to_G=%d, B_to_G=%d, G_to_G=%d \n",R_to_G,B_to_G,G_to_G);

	if(R_to_G < OFILM_tRG_Ratio_typical )
    {
        if(B_to_G < OFILM_tBG_Ratio_typical) 
        {
			R_Gain = 0x100 * OFILM_tRG_Ratio_typical / R_to_G;
			G_Gain = 0x100;
			B_Gain = 0x100 * OFILM_tBG_Ratio_typical / B_to_G;
        }
        else
        {
			R_Gain = 0x100 * (OFILM_tRG_Ratio_typical*B_to_G) / (OFILM_tBG_Ratio_typical*R_to_G);
			G_Gain = 0x100 *  B_to_G / OFILM_tBG_Ratio_typical;
			B_Gain = 0x100;
        }
    }
    else                      
    {		
        if(B_to_G < OFILM_tBG_Ratio_typical)
        {
			R_Gain = 0x100;
			G_Gain = 0x100 * R_to_G / OFILM_tRG_Ratio_typical;
			B_Gain = 0x100 * (OFILM_tBG_Ratio_typical*R_to_G) / (OFILM_tRG_Ratio_typical* B_to_G);
        } 
        else 
        {
    		G_gain_R = 0x100*R_to_G / OFILM_tRG_Ratio_typical;
            G_gain_B = 0x100*B_to_G / OFILM_tBG_Ratio_typical;
            if(G_gain_R > G_gain_B)						
            {						
				R_Gain = 0x100;
				G_Gain = 0x100 * R_to_G / OFILM_tRG_Ratio_typical;
				B_Gain = 0x100 * (OFILM_tBG_Ratio_typical*R_to_G) / (OFILM_tRG_Ratio_typical* B_to_G);
            } 
            else
            {		
            	R_Gain = 0x100 * (OFILM_tRG_Ratio_typical*B_to_G) / (OFILM_tBG_Ratio_typical*R_to_G);	
				G_Gain = 0x100 *  B_to_G / OFILM_tBG_Ratio_typical;
				B_Gain = 0x100;
            }
        }        
    }

	otp->R_Gain = R_Gain;
	otp->B_Gain = B_Gain;
	otp->G_Gain = G_Gain;

    SENSORDB("R_gain=0x%x, B_gain=0x%x, G_gain=0x%x \n",otp->R_Gain, otp->B_Gain, otp->G_Gain);
}



static void S5K5E2YA_OFILM_MIPI_write_otp_wb(struct S5K5E2YA_OFILM_MIPI_otp_struct *otp)
{
	kal_uint16 R_GainH, B_GainH, G_GainH;
	kal_uint16 R_GainL, B_GainL, G_GainL;
	kal_uint32 temp;

	temp = otp->R_Gain;
	R_GainH = (temp & 0xff00)>>8;
	temp = otp->R_Gain;
	R_GainL = (temp & 0x00ff);

	temp = otp->B_Gain;
	B_GainH = (temp & 0xff00)>>8;
	temp = otp->B_Gain;
	B_GainL = (temp & 0x00ff);

	temp = otp->G_Gain;
	G_GainH = (temp & 0xff00)>>8;
	temp = otp->G_Gain;
	G_GainL = (temp & 0x00ff);

    SENSORDB("G_GainH = 0x%x,G_GainL = 0x%x, R_GainH = 0x%x, R_GainL = 0x%x, B_GainH = 0x%x, B_GainL = 0x%x \n"
            ,G_GainH
            ,G_GainL
            ,R_GainH
            ,R_GainL
            ,B_GainH
            ,B_GainL);

	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x020e, G_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x020f, G_GainL);//GR
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0210, R_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0211, R_GainL);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0212, B_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0213, B_GainL);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0214, G_GainH);
	s5k5e2yamipiraw_ofilm_write_cmos_sensor(0x0215, G_GainL);//GB
}

static void S5K5E2YA_OFILM_MIPI_update_wb_register_from_otp(void)
{
    struct S5K5E2YA_OFILM_MIPI_otp_struct current_otp;

    S5K5E2YA_OFILM_MIPI_read_otp_wb(&current_otp);
    S5K5E2YA_OFILM_MIPI_algorithm_otp_wb1(&current_otp);
    S5K5E2YA_OFILM_MIPI_write_otp_wb(&current_otp);
}
#endif

#endif

#define LOG_1 LOG_INF("s5k5e2ya,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/
#define LOG_INF(format, args...)	xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);

#define MIPI_SETTLEDELAY_AUTO     0
#define MIPI_SETTLEDELAY_MANNUAL  1


//#define CAPTURE_24FPS


static imgsensor_info_struct imgsensor_info = { 
	.sensor_id = S5K5E2YA_SENSOR_ID,
	
	.checksum_value = 0x87e356d9,
	
	.pre = {
		.pclk = 179200000,				//record different mode's pclk
		.linelength = 2950,				//record different mode's linelength
		.framelength = 2000,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1280,		//record different mode's width of grabwindow
		.grabwindow_height = 960,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 23,
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 2025,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1920,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.cap1 = {
		.pclk = 144000000,
		.linelength = 2950,
		.framelength = 2025,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1920,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 240,	
	},
	.normal_video = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 2560,
		.grabwindow_height = 1440,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.hs_video = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 506,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640,
		.grabwindow_height = 480,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 1200,
	},
	.slim_video = {
		.pclk = 179200000,
		.linelength = 2950,
		.framelength = 2000,
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 1280,
		.grabwindow_height = 720,
		.mipi_data_lp2hs_settle_dc = 23,
		.max_framerate = 300,
	},
	.margin = 4,
	.min_shutter = 1,
	.max_frame_length = 0xffff,
	.ae_shut_delay_frame = 0,
	.ae_sensor_gain_delay_frame = 1,
	.ae_ispGain_delay_frame = 2,
	.ihdr_support = 0,	  //1, support; 0,not support
	.ihdr_le_firstline = 0,  //1,le first ; 0, se first
	.sensor_mode_num = 5,	  //support sensor mode num
	
	.cap_delay_frame = 3, 
	.pre_delay_frame = 3, 
	.video_delay_frame = 3,
	.hs_video_delay_frame = 3,
	.slim_video_delay_frame = 3,
	
	.isp_driving_current = ISP_DRIVING_6MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
	.mipi_settle_delay_mode = MIPI_SETTLEDELAY_AUTO,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//SENSOR_OUTPUT_FORMAT_RAW_Gr
	.mclk = 24,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.i2c_addr_table = {0x20, 0x6c, 0xff},
};


static imgsensor_struct imgsensor = {
	.mirror = IMAGE_NORMAL,				//mirrorflip information
	.sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
	.shutter = 0x3D0,					//current shutter
	.gain = 0x100,						//current gain
	.dummy_pixel = 0,					//current dummypixel
	.dummy_line = 0,					//current dummyline
	.current_fps = 0,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
	.autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
	.test_pattern = KAL_FALSE,		//test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
	.current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
	.ihdr_en = 0, //sensor need support LE, SE with HDR feature
	.i2c_write_id = 0x20,
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =	 
{{ 2576, 1936,	  8,	8, 2560, 1920, 1280,  960, 0000, 0000, 1280,  960,	  0,	0, 1280,  960}, // Preview 
 { 2576, 1936,	  8,	8, 2560, 1920, 2560, 1920, 0000, 0000, 2560, 1920,	  0,	0, 2560, 1920}, // capture 
 { 2576, 1936,	  8,  248, 2560, 1440, 2560, 1440, 0000, 0000, 2560, 1440,	  0,	0, 2560, 1440}, // video 
 { 2576, 1936,	320,  420,  640,  480,  640,  480, 0000, 0000,  640,  480,	  0,	0,  640,  480}, //hight speed video 
 { 2576, 1936,	  8,  248, 2560, 1440, 1280,  720, 0000, 0000, 1280,  720,	  0,	0, 1280,  720}};// slim video 


static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

static void write_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}

static void write_cmos_sensor_8(kal_uint16 addr, kal_uint8 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
    iWriteRegI2C(pusendcmd , 3, imgsensor.i2c_write_id);
}


static void set_dummy()
{
	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);

	write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
	write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0342, imgsensor.line_length >> 8);
	write_cmos_sensor(0x0343, imgsensor.line_length & 0xFF);
  
}	/*	set_dummy  */

static kal_uint32 return_sensor_id()
{
	return ((read_cmos_sensor(0x0000) << 8) | read_cmos_sensor(0x0001));
}


static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */


static void write_shutter(kal_uint16 shutter)
{
	kal_uint16 realtime_fps = 0;
	kal_uint32 frame_length = 0;
	   
	/* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
	/* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */
	
	// OV Recommend Solution
	// if shutter bigger than frame_length, should extend frame length first
	spin_lock(&imgsensor_drv_lock);
	if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)		
		imgsensor.frame_length = shutter + imgsensor_info.margin;
	else
		imgsensor.frame_length = imgsensor.min_frame_length;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
		imgsensor.frame_length = imgsensor_info.max_frame_length;
	spin_unlock(&imgsensor_drv_lock);
	shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
	shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;
	
	if (imgsensor.autoflicker_en) { 
		realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
		if(realtime_fps >= 297 && realtime_fps <= 305)
			set_max_framerate(296,0);
		else if(realtime_fps >= 147 && realtime_fps <= 150)
			set_max_framerate(146,0);	
		else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
		}
	} else {
		// Extend frame length
		write_cmos_sensor(0x0340, imgsensor.frame_length >> 8);
		write_cmos_sensor(0x0341, imgsensor.frame_length & 0xFF);
	}

	// Update Shutter
	//write_cmos_sensor(0x0104, 0x01);   //group hold
	write_cmos_sensor(0x0202, shutter >> 8);
	write_cmos_sensor(0x0203, shutter & 0xFF);	
	//write_cmos_sensor(0x0104, 0x00);   //group hold
	
	LOG_INF("shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

	//LOG_INF("frame_length = %d ", frame_length);
	
}	/*	write_shutter  */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void set_shutter(kal_uint16 shutter)
{
	unsigned long flags;
	spin_lock_irqsave(&imgsensor_drv_lock, flags);
	imgsensor.shutter = shutter;
	spin_unlock_irqrestore(&imgsensor_drv_lock, flags);
	
	write_shutter(shutter);
}	/*	set_shutter */



static kal_uint16 gain2reg(const kal_uint16 gain)
{
	return gain>>1;
}

/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
	kal_uint16 reg_gain;

	/* 0x350A[0:1], 0x350B[0:7] AGC real gain */
	/* [0:3] = N meams N /16 X	*/
	/* [4:9] = M meams M X		 */
	/* Total gain = M + N /16 X   */

	//
	
 
	reg_gain = gain>>1;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.gain = reg_gain; 
	spin_unlock(&imgsensor_drv_lock);
	LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

	//write_cmos_sensor(0x0104, 0x01);   //group hold
	write_cmos_sensor(0x0204, reg_gain >> 8);
	write_cmos_sensor(0x0205, reg_gain & 0xFF);  
	//write_cmos_sensor(0x0104, 0x00);
	
	return gain;
}	/*	set_gain  */



//defined but not used
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
	LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
	if (imgsensor.ihdr_en) {
		
		spin_lock(&imgsensor_drv_lock);
			if (le > imgsensor.min_frame_length - imgsensor_info.margin)		
				imgsensor.frame_length = le + imgsensor_info.margin;
			else
				imgsensor.frame_length = imgsensor.min_frame_length;
			if (imgsensor.frame_length > imgsensor_info.max_frame_length)
				imgsensor.frame_length = imgsensor_info.max_frame_length;
			spin_unlock(&imgsensor_drv_lock);
			if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
			if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;
			
			
				// Extend frame length first
				write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
				write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

		write_cmos_sensor(0x3502, (le << 4) & 0xFF);
		write_cmos_sensor(0x3501, (le >> 4) & 0xFF);	 
		write_cmos_sensor(0x3500, (le >> 12) & 0x0F);
		
		write_cmos_sensor(0x3508, (se << 4) & 0xFF); 
		write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
		write_cmos_sensor(0x3506, (se >> 12) & 0x0F); 

		set_gain(gain);
	}

}



static void set_mirror_flip(kal_uint8 image_mirror)
{
	LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
	switch (image_mirror) {
		case IMAGE_NORMAL:
			write_cmos_sensor(0x0101,0x00);
			break;
		case IMAGE_H_MIRROR:
			write_cmos_sensor(0x0101,0x01);
			break;
		case IMAGE_V_MIRROR:
			write_cmos_sensor(0x0101,0x02);		
			break;
		case IMAGE_HV_MIRROR:
			write_cmos_sensor(0x0101,0x03);
			break;
		default:
			LOG_INF("Error image_mirror setting\n");
	}

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("E\n");

   // +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x3000,0x04);       //   ct_ld_start                                    
	write_cmos_sensor(0x3002,0x03);       //   ct_sl_start                                    
	write_cmos_sensor(0x3003,0x04);       //   ct_sl_margin                                   
	write_cmos_sensor(0x3004,0x02);       //   ct_rx_start                                    
	write_cmos_sensor(0x3005,0x00);       //   ct_rx_margin (MSB)                             
	write_cmos_sensor(0x3006,0x10);       //   ct_rx_margin (LSB)                             
	write_cmos_sensor(0x3007,0x03);       //   ct_tx_start                                    
	write_cmos_sensor(0x3008,0x55);       //   ct_tx_width                                    
	write_cmos_sensor(0x3039,0x00);       //   cintc1_margin (10 --> 00)                      
	write_cmos_sensor(0x303A,0x00);       //   cintc2_margin (10 --> 00)                      
	write_cmos_sensor(0x303B,0x00);       //   offs_sh                                        
	write_cmos_sensor(0x3009,0x05);       //   ct_srx_margin                                  
	write_cmos_sensor(0x300A,0x55);       //   ct_stx_width                                   
	write_cmos_sensor(0x300B,0x38);       //   ct_dstx_width                                  
	write_cmos_sensor(0x300C,0x10);       //   ct_stx2dstx                                    
	write_cmos_sensor(0x3012,0x05);       //   ct_cds_start                                   
	write_cmos_sensor(0x3013,0x00);       //   ct_s1s_start                                   
	write_cmos_sensor(0x3014,0x22);       //   ct_s1s_end                                     
	write_cmos_sensor(0x300E,0x79);       //   ct_s3_width                                    
	write_cmos_sensor(0x3010,0x68);       //   ct_s4_width                                    
	write_cmos_sensor(0x3019,0x03);       //   ct_s4d_start                                   
	write_cmos_sensor(0x301A,0x00);       //   ct_pbr_start                                   
	write_cmos_sensor(0x301B,0x06);       //   ct_pbr_width                                   
	write_cmos_sensor(0x301C,0x00);       //   ct_pbs_start                                   
	write_cmos_sensor(0x301D,0x22);       //   ct_pbs_width                                   
	write_cmos_sensor(0x301E,0x00);       //   ct_pbr_ob_start                                
	write_cmos_sensor(0x301F,0x10);       //   ct_pbr_ob_width                                
	write_cmos_sensor(0x3020,0x00);       //   ct_pbs_ob_start                                
	write_cmos_sensor(0x3021,0x00);       //   ct_pbs_ob_width                                
	write_cmos_sensor(0x3022,0x0A);       //   ct_cds_lim_start                               
	write_cmos_sensor(0x3023,0x1E);       //   ct_crs_start                                   
	write_cmos_sensor(0x3024,0x00);       //   ct_lp_hblk_cds_start (MSB)                     
	write_cmos_sensor(0x3025,0x00);       //   ct_lp_hblk_cds_start (LSB)                     
	write_cmos_sensor(0x3026,0x00);       //   ct_lp_hblk_cds_end (MSB)                       
	write_cmos_sensor(0x3027,0x00);       //   ct_lp_hblk_cds_end (LSB)                       
	write_cmos_sensor(0x3028,0x1A);       //   ct_rmp_off_start                               
	write_cmos_sensor(0x3015,0x00);       //   ct_rmp_rst_start (MSB)                         
	write_cmos_sensor(0x3016,0x84);       //   ct_rmp_rst_start (LSB)                         
	write_cmos_sensor(0x3017,0x00);       //   ct_rmp_sig_start (MSB)                         
	write_cmos_sensor(0x3018,0xA0);       //   ct_rmp_sig_start (LSB)                         
	write_cmos_sensor(0x302B,0x10);       //   ct_cnt_margin                                  
	write_cmos_sensor(0x302C,0x0A);       //   ct_rmp_per                                     
	write_cmos_sensor(0x302D,0x06);       //   ct_cnt_ms_margin1                              
	write_cmos_sensor(0x302E,0x05);       //   ct_cnt_ms_margin2                              
	write_cmos_sensor(0x302F,0x0E);       //   rst_mx                                         
	write_cmos_sensor(0x3030,0x2F);       //   sig_mx                                         
	write_cmos_sensor(0x3031,0x08);       //   ct_latch_start                                 
	write_cmos_sensor(0x3032,0x05);       //   ct_latch_width                                 
	write_cmos_sensor(0x3033,0x09);       //   ct_hold_start                                  
	write_cmos_sensor(0x3034,0x05);       //   ct_hold_width                                  
	write_cmos_sensor(0x3035,0x00);       //   ct_lp_hblk_dbs_start (MSB)                     
	write_cmos_sensor(0x3036,0x00);       //   ct_lp_hblk_dbs_start (LSB)                     
	write_cmos_sensor(0x3037,0x00);       //   ct_lp_hblk_dbs_end (MSB)                       
	write_cmos_sensor(0x3038,0x00);       //   ct_lp_hblk_dbs_end (LSB)                       
	write_cmos_sensor(0x3088,0x06);       //   ct_lat_lsb_offset_start1                       
	write_cmos_sensor(0x308A,0x08);       //   ct_lat_lsb_offset_end1                         
	write_cmos_sensor(0x308C,0x05);       //   ct_lat_lsb_offset_start2                       
	write_cmos_sensor(0x308E,0x07);       //   ct_lat_lsb_offset_end2                         
	write_cmos_sensor(0x3090,0x06);       //   ct_conv_en_offset_start1                       
	write_cmos_sensor(0x3092,0x08);       //   ct_conv_en_offset_end1                         
	write_cmos_sensor(0x3094,0x05);       //   ct_conv_en_offset_start2                       
	write_cmos_sensor(0x3096,0x21);       //   ct_conv_en_offset_end2                         
	write_cmos_sensor(0x3099,0x0E);       //  cds_option ([3]:crs switch disable, s3,s4 streng
	write_cmos_sensor(0x3070,0x10);       //  comp1_bias (default:77)                         
	write_cmos_sensor(0x3085,0x11);       //  comp1_bias (gain1~4)                            
	write_cmos_sensor(0x3086,0x01);       //  comp1_bias (gain4~8) modified 813               
                         
	write_cmos_sensor(0x3064,0x00);       //  Multiple sampling(gainx8,x16)                   
	write_cmos_sensor(0x3062,0x08);       //  off_rst                                         
	write_cmos_sensor(0x3061,0x11);       //  dbr_tune_rd (default :08)                       
	write_cmos_sensor(0x307B,0x20);       //  dbr_tune_rgsl (default :08)                     
	write_cmos_sensor(0x3068,0x00);       //  RMP BP bias sampling                            
	write_cmos_sensor(0x3074,0x00);       //  Pixel bias sampling [2]:Default L               
	write_cmos_sensor(0x307D,0x00);       //  VREF sampling [1]                               
	write_cmos_sensor(0x3045,0x01);       //  ct_opt_l1_start                                 
	write_cmos_sensor(0x3046,0x05);       //  ct_opt_l1_width                                 
	write_cmos_sensor(0x3047,0x78);                                               
	write_cmos_sensor(0x307F,0xB1);       // RDV_OPTION[5:4], RG default high                 
	write_cmos_sensor(0x3098,0x01);       // CDS_OPTION[16] SPLA-II enable                    
	write_cmos_sensor(0x305C,0xF6);       // lob_extension[6]                                 
	write_cmos_sensor(0x306B,0x10);                                                 
	write_cmos_sensor(0x3063,0x27);       //  ADC_SAT 490mV --> 610mV                         
	write_cmos_sensor(0x3400,0x01);       //  GAS bypass                                      
	write_cmos_sensor(0x3235,0x49);       //  L/F-ADLC on                                     
	write_cmos_sensor(0x3233,0x00);       //  D-pedestal L/F ADLC off (1FC0h)                 
	write_cmos_sensor(0x3234,0x00);                                                 
	write_cmos_sensor(0x3300,0x0C);       // BPC bypass                                       
                             
	write_cmos_sensor(0x3203,0x45);       // ADC_OFFSET_EVEN                                  
	write_cmos_sensor(0x3205,0x4D);       // ADC_OFFSET_ODD                                   
	write_cmos_sensor(0x320B,0x40);       // ADC_DEFAULT                                      
	write_cmos_sensor(0x320C,0x06);       // ADC_MAX                                          
	write_cmos_sensor(0x320D,0xC0);  

	write_cmos_sensor(0x3929,0x07);       //set mipi non-continue mode
 	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
}	/*	sensor_init  */


static void preview_setting(void)
{
	// +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 990d (--> 3C8 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xD0);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x05); // x_output size : def. 1288d
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x03); // y_output size : def. 968d
	write_cmos_sensor(0x034F,0xC0); 
	
	
	//Digital Binning
	write_cmos_sensor(0x0900,0x01);	//2x2 Binning
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x03);
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 	 
}	/*	preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	if (currefps == 240) { //24fps for PIP
	// +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off

	
	// Clock Setting
	write_cmos_sensor(0x0305,0x05); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0x96); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x02); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0xD0); 
	write_cmos_sensor(0x3C1C,0x57); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xE9);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x0A); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x07); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0x80); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x01);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
	}
	else{// Reset for operation     30fps for normal capture                                                                    
	write_cmos_sensor(0x0100,0x00); //stream off
	mdelay(40);
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S3CC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xE9);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x0A); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x07); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0x80); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x01);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);

	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);

	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
	}
}

static void normal_video_setting(kal_uint16 currefps)
{
	LOG_INF("E! currefps:%d\n",currefps);
	
	//5.1.3 Video 2592x1944 30fps 24M MCLK 2lane 864Mbps/lane
	// +++++++++++++++++++++++++++//                                                               
// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off

	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30C0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode   
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xD0);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);

	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0xF8); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x06); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x97); 
	write_cmos_sensor(0x034C,0x0A); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x05); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0xA0); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x01);

	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);

	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x27);
	write_cmos_sensor(0x340C,0x01);
	write_cmos_sensor(0x340D,0xA3);
	write_cmos_sensor(0x340E,0x9E);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
		
}
static void hs_video_setting()
{
	LOG_INF("E\n");         
	//VGA 120fps

	write_cmos_sensor(0x0100,0x00); //stream off
	write_cmos_sensor(0x0136,0x18);
	write_cmos_sensor(0x0137,0x00);
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x01); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xFA);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0x08); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def.   
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x07); // y_addr_end : def.
	write_cmos_sensor(0x034B,0x87); 
	write_cmos_sensor(0x034C,0x02); // x_output size : def. 640d 
	write_cmos_sensor(0x034D,0x80); 
	write_cmos_sensor(0x034E,0x01); // y_output size : def. 480d
	write_cmos_sensor(0x034F,0xE0); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x01); //0x0 Binning
	write_cmos_sensor(0x0901,0x44);
	write_cmos_sensor(0x0383,0x03);
	write_cmos_sensor(0x0387,0x07);
	
	
	//Integration time	
	write_cmos_sensor(0x0204,0x00);
	write_cmos_sensor(0x0205,0x20);
	write_cmos_sensor(0x0202,0x01);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x00);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x00);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 



	//full size 60fps
/*
	// +++++++++++++++++++++++++++//                                                               
	// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x05); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xCA); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0xC9); 
	write_cmos_sensor(0x3C1C,0x59); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x04); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0x4F);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x01); // x_addr_start
	write_cmos_sensor(0x0345,0x40); 
	write_cmos_sensor(0x0346,0x01); // y_addr_start
	write_cmos_sensor(0x0347,0xA4); 
	write_cmos_sensor(0x0348,0x08); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0xBF); 
	write_cmos_sensor(0x034A,0x05); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0xDB); 
	write_cmos_sensor(0x034C,0x07); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x80); 
	write_cmos_sensor(0x034E,0x04); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0x38); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x00);	//0x0 Binning
	write_cmos_sensor(0x0901,0x20);
	write_cmos_sensor(0x0387,0x01);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x4C);
	write_cmos_sensor(0x3408,0x01);
	write_cmos_sensor(0x3409,0xF3);
	write_cmos_sensor(0x340A,0x24);
	write_cmos_sensor(0x340B,0x1A);
	write_cmos_sensor(0x340C,0x03);
	write_cmos_sensor(0x340D,0x6E);
	write_cmos_sensor(0x340E,0xE8);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 
*/
}

static void slim_video_setting()
{
	LOG_INF("E\n");
	// +++++++++++++++++++++++++++//                                                               
// Reset for operation                                                                         
	write_cmos_sensor(0x0100,0x00); //stream off
	
	// Clock Setting
	write_cmos_sensor(0x0305,0x06); //PLLP (def:5)
	write_cmos_sensor(0x0306,0x00);
	write_cmos_sensor(0x0307,0xE0); //PLLM (def:CCh 204d --> B3h 179d)
	write_cmos_sensor(0x3C1F,0x00); //PLLS 
	
	//S30CCC0 //dphy_band_ctrl
	
	write_cmos_sensor(0x0820,0x03); // requested link bit rate mbps : (def:3D3h 979d --> 35Bh 859d)
	write_cmos_sensor(0x0821,0x80); 
	write_cmos_sensor(0x3C1C,0x58); //dbr_div
	
	write_cmos_sensor(0x0114,0x01);  //Lane mode
	
	// Size Setting
	write_cmos_sensor(0x0340,0x07); // frame_length_lines : def. 1962d (7C2 --> 7A6 Mimnimum 22 lines)
	write_cmos_sensor(0x0341,0xD0);
	write_cmos_sensor(0x0342,0x0B); // line_length_pck : def. 2900d 
	write_cmos_sensor(0x0343,0x86);
	
	write_cmos_sensor(0x0344,0x00); // x_addr_start
	write_cmos_sensor(0x0345,0x08); 
	write_cmos_sensor(0x0346,0x00); // y_addr_start
	write_cmos_sensor(0x0347,0xF8); 
	write_cmos_sensor(0x0348,0x0A); // x_addr_end : def. 2575d  
	write_cmos_sensor(0x0349,0x07); 
	write_cmos_sensor(0x034A,0x06); // y_addr_end : def. 1936d
	write_cmos_sensor(0x034B,0x97); 
	write_cmos_sensor(0x034C,0x05); // x_output size : def. 2560d 
	write_cmos_sensor(0x034D,0x00); 
	write_cmos_sensor(0x034E,0x02); // y_output size : def. 1920d
	write_cmos_sensor(0x034F,0xD0); 
	
	//Digital Binning(default)
	write_cmos_sensor(0x0900,0x01);	//2x2 Binning
	write_cmos_sensor(0x0901,0x22);
	write_cmos_sensor(0x0383,0x01);
	write_cmos_sensor(0x0387,0x03);
	
	
	//Integration time	
	write_cmos_sensor(0x0202,0x02);  // coarse integration
	write_cmos_sensor(0x0203,0x00);
	write_cmos_sensor(0x0200,0x04);  // fine integration (AA8h --> AC4h)
	write_cmos_sensor(0x0201,0x98);
	
	write_cmos_sensor(0x3407,0x00);
	write_cmos_sensor(0x3408,0x00);
	write_cmos_sensor(0x3409,0x00);
	write_cmos_sensor(0x340A,0x00);
	write_cmos_sensor(0x340B,0x00);
	write_cmos_sensor(0x340C,0x02);
	write_cmos_sensor(0x340D,0x00);
	write_cmos_sensor(0x340E,0x00);
	write_cmos_sensor(0x3401,0x50);
	write_cmos_sensor(0x3402,0x3C);
	write_cmos_sensor(0x3403,0x03);
	write_cmos_sensor(0x3404,0x33);
	write_cmos_sensor(0x3405,0x04);
	write_cmos_sensor(0x3406,0x44);
	write_cmos_sensor(0x3458,0x03);
	write_cmos_sensor(0x3459,0x33);
	write_cmos_sensor(0x345A,0x04);
	write_cmos_sensor(0x345B,0x44);
	write_cmos_sensor(0x3400,0x01);
	
	// streaming ON
	write_cmos_sensor(0x0100,0x01); 

}

extern int Sub_Camera_MID;//
/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			*sensor_id = return_sensor_id();
			if (*sensor_id == imgsensor_info.sensor_id) {	
		#ifdef __OFILM_OTP__
		unsigned int moudile_id = 0;

		ofilm_get_otp_module_id(&moudile_id);
		SENSORDB("Read moudile_id=0x%x\n",moudile_id);
		if(moudile_id == 7)
		{
			Sub_Camera_MID = 7;// 
			*sensor_id = S5K5E2YA_OFILM_SENSOR_ID;
			SENSORDB("o-film module set output dataformat RAW_Gr!!\n");
			imgsensor_info.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr;
		}	
		else
		{
			*sensor_id = 0xFFFFFFFF;
			return ERROR_SENSOR_CONNECT_FAIL;

		}
		#endif
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);	  
				return ERROR_NONE;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		retry = 2;
	}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
	//const kal_uint8 i2c_addr[] = {IMGSENSOR_WRITE_ID_1, IMGSENSOR_WRITE_ID_2};
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	kal_uint32 sensor_id = 0; 
	LOG_1;
	LOG_2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
		spin_lock(&imgsensor_drv_lock);
		imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
		spin_unlock(&imgsensor_drv_lock);
		do {
			sensor_id = return_sensor_id();
			if (sensor_id == imgsensor_info.sensor_id) {				
				LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);	  
				break;
			}	
			LOG_INF("Read sensor id fail, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
			retry--;
		} while(retry > 0);
		i++;
		if (sensor_id == imgsensor_info.sensor_id)
			break;
		retry = 2;
	}		 
	if (imgsensor_info.sensor_id != sensor_id)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	sensor_init();
	set_mirror_flip(IMAGE_HV_MIRROR);

#ifdef __OFILM_OTP__
	unsigned int moudile_id = 0;

	ofilm_get_otp_module_id(&moudile_id);
	SENSORDB("Read moudile_id=0x%x\n",moudile_id);
	if(moudile_id == 7)
	{
		SENSORDB("o-film module set normal mirror flip!!\n");
		set_mirror_flip(IMAGE_NORMAL);
		ofilm_otp_lenc_update();
	}
	//ofilm_otp_lenc_update();
	
#ifdef __OFILM_OTP_WB__
	S5K5E2YA_OFILM_MIPI_update_wb_register_from_otp();
#endif
#endif
	  
	spin_lock(&imgsensor_drv_lock);

	imgsensor.autoflicker_en= KAL_FALSE;
	imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
	imgsensor.shutter = 0x3D0;
	imgsensor.gain = 0x100;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	imgsensor.frame_length = imgsensor_info.pre.framelength;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.dummy_pixel = 0;
	imgsensor.dummy_line = 0;
	imgsensor.ihdr_en = 0;
	imgsensor.test_pattern = KAL_FALSE;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	spin_unlock(&imgsensor_drv_lock);

	return ERROR_NONE;
}	/*	open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("E\n");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {//PIP capture: 24fps for less than 13M, 20fps for 16M,15fps for 20M
		imgsensor.pclk = imgsensor_info.cap1.pclk;
		imgsensor.line_length = imgsensor_info.cap1.linelength;
		imgsensor.frame_length = imgsensor_info.cap1.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap1.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	} else {
		if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
			LOG_INF("Warning: current_fps %d fps is not support, so use cap1's setting: %d fps!\n",imgsensor_info.cap1.max_framerate/10);
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.autoflicker_en = KAL_FALSE;
	}
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
	
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	//imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
	
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
	imgsensor.pclk = imgsensor_info.hs_video.pclk;
	//imgsensor.video_mode = KAL_TRUE;
	imgsensor.line_length = imgsensor_info.hs_video.linelength;
	imgsensor.frame_length = imgsensor_info.hs_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	hs_video_setting();
	
	return ERROR_NONE;
}	/*	hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E\n");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
	imgsensor.pclk = imgsensor_info.slim_video.pclk;
	imgsensor.line_length = imgsensor_info.slim_video.linelength;
	imgsensor.frame_length = imgsensor_info.slim_video.framelength; 
	imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
	imgsensor.dummy_line = 0;
	imgsensor.dummy_pixel = 0;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	slim_video_setting();
	
	return ERROR_NONE;
}	/*	slim_video	 */



static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
	LOG_INF("E\n");
	sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
	sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;
	
	sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
	sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

	sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
	sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;		

	
	sensor_resolution->SensorHighSpeedVideoWidth	 = imgsensor_info.hs_video.grabwindow_width;
	sensor_resolution->SensorHighSpeedVideoHeight	 = imgsensor_info.hs_video.grabwindow_height;
	
	sensor_resolution->SensorSlimVideoWidth	 = imgsensor_info.slim_video.grabwindow_width;
	sensor_resolution->SensorSlimVideoHeight	 = imgsensor_info.slim_video.grabwindow_height;
	return ERROR_NONE;
}	/*	get_resolution	*/

static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
					  MSDK_SENSOR_INFO_STRUCT *sensor_info,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	
	//sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
	//sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
	//imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

	sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
	sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
	sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	sensor_info->SensorInterruptDelayLines = 4; /* not use */
	sensor_info->SensorResetActiveHigh = FALSE; /* not use */
	sensor_info->SensorResetDelayCount = 5; /* not use */

	sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
	sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
	sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
	sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

	sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame; 
	sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame; 
	sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
	sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
	sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

	sensor_info->SensorMasterClockSwitch = 0; /* not use */
	sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;
	
	sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame; 		 /* The frame of setting shutter default 0 for TG int */
	sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;	/* The frame of setting sensor gain */
	sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;	
	sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
	sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
	sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;
	
	sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num; 
	sensor_info->SensorClockFreq = imgsensor_info.mclk;
	sensor_info->SensorClockDividCount = 3; /* not use */
	sensor_info->SensorClockRisingCount = 0;
	sensor_info->SensorClockFallingCount = 2; /* not use */
	sensor_info->SensorPixelClockCount = 3; /* not use */
	sensor_info->SensorDataLatchCount = 2; /* not use */
	
	sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
	sensor_info->SensorHightSampling = 0;	// 0 is default 1x 
	sensor_info->SensorPacketECCOrder = 1;

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			sensor_info->SensorGrabStartX = imgsensor_info.cap.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc; 

			break;	 
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			
			sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;
	   
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc; 

			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:			
			sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc; 

			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;
				  
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc; 

			break;
		default:			
			sensor_info->SensorGrabStartX = imgsensor_info.pre.startx; 
			sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;		
			
			sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
			break;
	}
	
	return ERROR_NONE;
}	/*	get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("scenario_id = %d\n", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			hs_video(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			slim_video(image_window, sensor_config_data);
			break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d\n ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
	else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
		imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{
	LOG_INF("enable = %d, framerate = %d \n", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) //enable auto flicker	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate) 
{
	kal_uint32 frame_length;
  
	LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(framerate == 0)
				return ERROR_NONE;
			frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;			
			imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
			frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
            }
			//set_dummy();
			break;	
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO:
			frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;	
			imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			break;		
		default:  //coding with  preview scenario by default
			frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
			spin_lock(&imgsensor_drv_lock);
			imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
			imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
			imgsensor.min_frame_length = imgsensor.frame_length;
			spin_unlock(&imgsensor_drv_lock);
			//set_dummy();
			LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
			break;
	}	
	return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate) 
{
	LOG_INF("scenario_id = %d\n", scenario_id);

	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			*framerate = imgsensor_info.pre.max_framerate;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*framerate = imgsensor_info.normal_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			*framerate = imgsensor_info.cap.max_framerate;
			break;		
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
			*framerate = imgsensor_info.hs_video.max_framerate;
			break;
		case MSDK_SCENARIO_ID_SLIM_VIDEO: 
			*framerate = imgsensor_info.slim_video.max_framerate;
			break;
		default:
			break;
	}

	return ERROR_NONE;
}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
	LOG_INF("enable: %d\n", enable);

	if (enable) {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601, 0x01);
	} else {
		// 0x5E00[8]: 1 enable,  0 disable
		// 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
		write_cmos_sensor(0x0601, 0x00);
	}	 
	write_cmos_sensor(0x3200, 0x00);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.test_pattern = enable;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
							 UINT8 *feature_para,UINT32 *feature_para_len)
{
	UINT16 *feature_return_para_16=(UINT16 *) feature_para;
	UINT16 *feature_data_16=(UINT16 *) feature_para;
	UINT32 *feature_return_para_32=(UINT32 *) feature_para;
	UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

	SENSOR_WINSIZE_INFO_STRUCT *wininfo;	
	MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;
 
	LOG_INF("feature_id = %d", feature_id);
	switch (feature_id) {
		case SENSOR_FEATURE_GET_PERIOD:
			*feature_return_para_16++ = imgsensor.line_length;
			*feature_return_para_16 = imgsensor.frame_length;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
			*feature_return_para_32 = imgsensor.pclk;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			if((sensor_reg_data->RegData>>8)>0)
			   write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			else
				write_cmos_sensor_8(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			get_imgsensor_id(feature_return_para_32);
			break;
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
			*feature_return_para_32 = imgsensor_info.checksum_value;
			*feature_para_len=4;
			break;
		case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", (UINT32)*feature_data);
			spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_SET_HDR:
			//LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data_16);
			LOG_INF("Warning! Not Support IHDR Feature");
			spin_lock(&imgsensor_drv_lock);
			//imgsensor.ihdr_en = (BOOL)*feature_data_16;
            imgsensor.ihdr_en = KAL_FALSE;
			spin_unlock(&imgsensor_drv_lock);
			break;
		case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", (UINT32)*feature_data);
            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

			switch (*feature_data_32) {
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_SLIM_VIDEO:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				default:
					memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
					break;
			}
            break;
		case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
			break;
		default:
			break;
	}
  
	return ERROR_NONE;
}	/*	feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
	open,
	get_info,
	get_resolution,
	feature_control,
	control,
	close
};

UINT32 S5K5E2YA_OFILM_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	OV5693_MIPI_RAW_SensorInit	*/
