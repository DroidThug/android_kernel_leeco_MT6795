/*
 * MD218A voice coil motor driver
 *
 * 
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "LC898122OFILMAF.h"
#include "../camera/kd_camera_hw.h"
#include "Ois_OFILM.h"
#include "OisDef_OFILM.h"
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/kthread.h>

// in K2, main=3, sub=main2=1
#define LENS_I2C_BUSNUM 0
static struct i2c_board_info __initdata kd_lens_dev={ I2C_BOARD_INFO("LC898122OFILMAF", 0x2F)};
 

#define LC898122OFILMAF_DRVNAME "LC898122OFILMAF"
#define LC898122OFILMAF_VCM_WRITE_ID           0x48

#define LC898122OFILMAF_DEBUG
#ifdef LC898122OFILMAF_DEBUG
#define LC898122OFILMAFDB printk
#else
#define LC898122OFILMAFDB(x,...)
#endif

static spinlock_t g_LC898122OFILMAF_SpinLock;

static struct i2c_client * g_pstLC898122OFILMAF_I2Cclient = NULL;

static dev_t g_LC898122OFILMAF_devno;
static struct cdev * g_pLC898122OFILMAF_CharDrv = NULL;
static struct class *actuator_class = NULL;

static int  g_s4LC898122OFILMAF_Opened = 0;
int  g_s4LC898122OFILMAF_Inited = 0;
static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4LC898122OFILMAF_INF = 0;
static unsigned long g_u4LC898122OFILMAF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition   = 0;
static unsigned long g_u4InitPosition   = 100;

static int g_sr = 3;

void RegReadA_Ofilm(unsigned short RegAddr, unsigned char *RegData)
{
    int  i4RetValue = 0;
    char pBuff[2] = {(char)(RegAddr >> 8) , (char)(RegAddr & 0xFF)};

    g_pstLC898122OFILMAF_I2Cclient->addr = (LC898122OFILMAF_VCM_WRITE_ID >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;

    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, pBuff, 2);
    if (i4RetValue < 0 ) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] read I2C send failed!!\n");
        return;
    }

    i4RetValue = i2c_master_recv(g_pstLC898122OFILMAF_I2Cclient, (u8*)RegData, 1);

    LC898122OFILMAFDB("[LC898122OFILMAF]I2C r (%x %x) \n",RegAddr,*RegData);
    if (i4RetValue != 1) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] I2C read failed!! \n");
        return;
    }
}

void RegWriteA_Ofilm(unsigned short RegAddr, unsigned char RegData)
{
#if 0  //snag
    unsigned char TempRegData=0;
#endif

    int  i4RetValue = 0;
    char puSendCmd[3] = {(char)((RegAddr>>8)&0xFF),(char)(RegAddr&0xFF),RegData};
    LC898122OFILMAFDB("[LC898122OFILMAF]I2C w (%x %x) \n",RegAddr,RegData);

    g_pstLC898122OFILMAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    g_pstLC898122OFILMAF_I2Cclient->addr = (LC898122OFILMAF_VCM_WRITE_ID >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;
	
    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, puSendCmd, 3);
    if (i4RetValue < 0) 
    {
        LC898122OFILMAFDB("[LC898122OFILMAF]I2C send failed!! \n");
        return;
    }

#if 0  //sang
    RegReadA_Ofilm(RegAddr, &TempRegData);
#endif
}
void RamReadA_Ofilm( unsigned short RamAddr, void * ReadData )
{
    int  i4RetValue = 0;
    char pBuff[2] = {(char)(RamAddr >> 8) , (char)(RamAddr & 0xFF)};
    unsigned short  vRcvBuff=0;
	unsigned long *pRcvBuff;
    pRcvBuff =(unsigned long *)ReadData;

    g_pstLC898122OFILMAF_I2Cclient->addr = (LC898122OFILMAF_VCM_WRITE_ID >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;

    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, pBuff, 2);
    if (i4RetValue < 0 ) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] read I2C send failed!!\n");
        return;
    }

    i4RetValue = i2c_master_recv(g_pstLC898122OFILMAF_I2Cclient, (u8*)&vRcvBuff, 2);
    if (i4RetValue != 2) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] I2C read failed!! \n");
        return;
    }
    *pRcvBuff=    ((vRcvBuff&0xFF) <<8) + ((vRcvBuff>> 8)&0xFF) ;
    
    LC898122OFILMAFDB("[LC898122OFILMAF]I2C r2 (%x %x) \n",RamAddr,(unsigned int)*pRcvBuff);

}
void RamWriteA_Ofilm( unsigned short RamAddr, unsigned short RamData )

{
#if 0  //sang
    unsigned int data;
#endif

    int  i4RetValue = 0;
    char puSendCmd[4] = {(char)((RamAddr >>  8)&0xFF), 
                         (char)( RamAddr       &0xFF),
                         (char)((RamData >>  8)&0xFF), 
                         (char)( RamData       &0xFF)};
    LC898122OFILMAFDB("[LC898122OFILMAF]I2C w2 (%x %x) \n",RamAddr,RamData);

    g_pstLC898122OFILMAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    g_pstLC898122OFILMAF_I2Cclient->addr = (LC898122OFILMAF_VCM_WRITE_ID >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;
	
    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, puSendCmd, 4);
    if (i4RetValue < 0) 
    {
        LC898122OFILMAFDB("[LC898122OFILMAF]I2C send failed!! \n");
        return;
    }

#if 0 //sang
    RamReadA_Ofilm(RamAddr, &data);
#endif
}

void SeqWriteA_Ofilm(unsigned char * pData, unsigned short lens )
{
    int ret = 0;

    g_pstLC898122OFILMAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
	g_pstLC898122OFILMAF_I2Cclient->timing = 100;
	
    ret = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, pData, lens);

    if (ret != lens) {
		LC898122OFILMAFDB("[LC898122OFILMAF] SeqWriteA_Ofilm I2C send failed ret = %d\n", ret);
    }
    return 0;
}

void RamRead32A_Ofilm(unsigned short RamAddr, void * ReadData )
{
    int  i4RetValue = 0;
    char pBuff[2] = {(char)(RamAddr >> 8) , (char)(RamAddr & 0xFF)};
    unsigned long *pRcvBuff, vRcvBuff=0;
    pRcvBuff =(unsigned long *)ReadData;

    g_pstLC898122OFILMAF_I2Cclient->addr = (LC898122OFILMAF_VCM_WRITE_ID >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;

    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, pBuff, 2);
    if (i4RetValue < 0 ) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] read I2C send failed!!\n");
        return;
    }

    i4RetValue = i2c_master_recv(g_pstLC898122OFILMAF_I2Cclient, (u8*)&vRcvBuff, 4);
    if (i4RetValue != 4) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] I2C read failed!! \n");
        return;
    }
    *pRcvBuff=   ((vRcvBuff     &0xFF) <<24) 
               +(((vRcvBuff>> 8)&0xFF) <<16) 
               +(((vRcvBuff>>16)&0xFF) << 8) 
               +(((vRcvBuff>>24)&0xFF)     );

        LC898122OFILMAFDB("[LC898122OFILMAF]I2C r4 (%x %x) \n",RamAddr,(unsigned int)*pRcvBuff);
}


void RamWrite32A_Ofilm(unsigned short RamAddr, unsigned long RamData )
{
    int  i4RetValue = 0;
#if 0  //snag
    unsigned long TempRegData=0;
#endif
    char puSendCmd[6] = {(char)((RamAddr >>  8)&0xFF), 
                         (char)( RamAddr       &0xFF),
                         (char)((RamData >> 24)&0xFF), 
                         (char)((RamData >> 16)&0xFF), 
                         (char)((RamData >>  8)&0xFF), 
                         (char)( RamData       &0xFF)};
    LC898122OFILMAFDB("[LC898122OFILMAF]I2C w4 (%x %x) \n",RamAddr,(unsigned int)RamData);

    
    g_pstLC898122OFILMAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    g_pstLC898122OFILMAF_I2Cclient->addr = (LC898122OFILMAF_VCM_WRITE_ID >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;
	
    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, puSendCmd, 6);
    if (i4RetValue < 0) 
    {
        LC898122OFILMAFDB("[LC898122OFILMAF]I2C send failed!! \n");
        return;
    }
#if 0 //sang
    RamRead32A_Ofilm(RamAddr, &TempRegData);
#endif
}
void WitTim_Ofilm(unsigned short  UsWitTim_Ofilm )
{
    msleep(UsWitTim_Ofilm);
}
static void LC898prtvalue(unsigned short  prtvalue )
{
    LC898122OFILMAFDB("[LC898122OFILMAF]printvalue ======%x   \n",prtvalue);
}

static unsigned char s4LC898OTP_ReadReg(unsigned short RegAddr)
{ 
    int  i4RetValue = 0;
    unsigned char pBuff = (unsigned char)RegAddr;
    unsigned char RegData=0xFF;

    g_pstLC898122OFILMAF_I2Cclient->addr = (0xA0 >> 1);
    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, &pBuff, 1);
    if (i4RetValue < 0 ) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] read I2C send failed!!\n");
        return 0xff;
    }

    i4RetValue = i2c_master_recv(g_pstLC898122OFILMAF_I2Cclient, &RegData, 1);

    LC898122OFILMAFDB("[LC898122OFILMAF]OTPI2C r (%x %x) \n",RegAddr,RegData);
    if (i4RetValue != 1) 
    {
        LC898122OFILMAFDB("[CAMERA SENSOR] I2C read failed!! \n");
        return 0xff;
    }
    return RegData;

}

static unsigned char s4LC898OTP_ReadReg_ofilm(unsigned short RegAddr)
{ 
    int  i4RetValue = 0;
    unsigned char pBuff = (unsigned char)RegAddr;
    unsigned char RegData=0xFF;
    int retry = 5;

    g_pstLC898122OFILMAF_I2Cclient->addr = (0xA8 >> 1);
    g_pstLC898122OFILMAF_I2Cclient->timing = 100;

    do {
	    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, &pBuff, 1);
	    if (i4RetValue < 0 ) 
	    {
	        LC898122OFILMAFDB("[LC898OTPofilm] read I2C send failed!!\n");
	    }
		else
		{
		    break;
		}
		
		udelay(100);
		
		if(retry == 0)
		{
	        LC898122OFILMAFDB("[LC898OTPofilm] retry 0 read I2C send failed!!\n");
	        return 0xff;
		}
    } while ((retry--) > 0);

	retry = 5;

    do {
	    i4RetValue = i2c_master_recv(g_pstLC898122OFILMAF_I2Cclient, &RegData, 1);

	    LC898122OFILMAFDB("[LC898122OFILMAF]OTPI2C r (%x %x) \n",RegAddr,RegData);
	    if (i4RetValue != 1) 
	    {
	        LC898122OFILMAFDB("[LC898OTPofilm] I2C read failed!! \n");
	    }
		else
		{
		    break;
		}
		
		udelay(100);

		if(retry == 0)
		{
	        LC898122OFILMAFDB("[LC898OTPofilm] retry 0 I2C read failed!!\n");
	        return 0xff;
		}
    } while ((retry--) > 0);
	
    return RegData;

}

#if 0
static void s4LC898OTP_WriteReg(unsigned short RegAddr, unsigned char RegData)
{
    int  i4RetValue = 0;
    char puSendCmd[2] = {(unsigned char)RegAddr, RegData};
    LC898122OFILMAFDB("[LC898122OFILMAF]OTPI2C w (%x %x) \n",RegAddr,RegData);

    g_pstLC898122OFILMAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    g_pstLC898122OFILMAF_I2Cclient->addr = (0xA0 >> 1);
    i4RetValue = i2c_master_send(g_pstLC898122OFILMAF_I2Cclient, puSendCmd, 2);
    if (i4RetValue < 0) 
    {
        LC898122OFILMAFDB("[LC898122OFILMAF]I2C send failed!! \n");
        return;
    }
}
#endif 
inline static int getLC898122OFILMAFInfo(__user stLC898122OFILMAF_MotorInfo * pstMotorInfo)
{
    stLC898122OFILMAF_MotorInfo stMotorInfo;
    stMotorInfo.u4MacroPosition   = g_u4LC898122OFILMAF_MACRO;
    stMotorInfo.u4InfPosition     = g_u4LC898122OFILMAF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = TRUE;

    if (g_i4MotorStatus == 1)    {stMotorInfo.bIsMotorMoving = 1;}
    else                        {stMotorInfo.bIsMotorMoving = 0;}

    if (g_s4LC898122OFILMAF_Opened >= 1)    {stMotorInfo.bIsMotorOpen = 1;}
    else                        {stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stLC898122OFILMAF_MotorInfo)))
    {
        LC898122OFILMAFDB("[LC898122OFILMAF] copy to user failed when getting motor information \n");
    }
    return 0;
}

typedef struct  ofilm_ois_otp_struct{
	
	kal_uint16 Hall_offset_x;	//Hall offset X
	kal_uint16 Hall_offset_y;	//Hall offset Y
	kal_uint16 Hall_bias_x;	//Hall bias X
	kal_uint16 Hall_bias_y;	//Hall bias Y
	kal_uint16 Hall_ad_offset_x;	//Hall AD offset X
	kal_uint16 Hall_ad_offset_y;	//Hall AD offset Y
	kal_uint16 Loop_gain_x;	//Loop gain X
	kal_uint16 Loop_gain_y;	//Loop gain Y

	kal_uint8 Gyro_offset_x_m;	//Gyro offset X M
	kal_uint8 Gyro_offset_x_l;	//Gyro offset X L
	kal_uint8 Gyro_offset_y_m;	//Gyro offset Y M
	kal_uint8 Gyro_offset_y_l;	//Gyro offset Y L

	kal_uint8 OSC;	//OSC
	
	kal_uint32 Gyro_gain_x;	//Gyro gain X
	kal_uint32 Gyro_gain_y;	//Gyro gain Y
	
	kal_uint16 AF_start;	//AF start
	kal_uint16 AF_infinit;	//AF Infinit
	kal_uint16 AF_macro;	//AF Macro

	bool valid_flag;
	
} ofilm_ois_otp_struct;
extern ofilm_ois_otp_struct g_ofilm_ois_otp_data;


void LC898122OFILMAF_init_drv_ofilm_static(void)
{
    unsigned short addrotp;
    unsigned long dataotp=0;
	
    LC898122OFILMAFDB("LC898122OFILMAF_init_drv_ofilm_static !!!\n");
    
    //IniSetAf_OFILM();
    IniSet_OFILM();
    IniSetAf_OFILM();
    //modify for ois test
    //RamWriteA_Ofilm(gxlmt3HS0_OFILM,0x3EB33333);  //0.35f
    //RamWriteA_Ofilm(gxlmt3HS1_OFILM,0x3EB33333);  //0.35f
    RamWrite32A_Ofilm(gxlmt3HS0_OFILM,0x3E4CCCCD);  //0.2f
    RamWrite32A_Ofilm(gxlmt3HS1_OFILM,0x3E4CCCCD);  //0.2f
    
    //modify end
RamAccFixMod_OFILM(ON_OFILM); //16bit Fix mode
    addrotp=0x05;dataotp=g_ofilm_ois_otp_data.Hall_offset_x;    
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall offset X 0x%x 0x%x\n", addrotp, (unsigned int)dataotp);
    RamWriteA_Ofilm(0x1479,dataotp);  //Hall offset X

    addrotp=0x07;dataotp=g_ofilm_ois_otp_data.Hall_offset_y;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall offset Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x14F9,dataotp);  //Hall offset Y

    addrotp=0x01;dataotp=g_ofilm_ois_otp_data.Hall_bias_x;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall bias X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x147A,dataotp);  //Hall bias X

    addrotp=0x03;dataotp=g_ofilm_ois_otp_data.Hall_bias_y;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall bias Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x14FA,dataotp);  //Hall bias Y

    addrotp=0x12;dataotp=g_ofilm_ois_otp_data.Hall_ad_offset_x;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall AD offset X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	//modify for ois test
	//RamWriteA(0x1450,dataotp);  //Hall AD offset X
	RamWriteA_Ofilm(0x1450,0x199A);  //Hall AD offset X set fix to 0.2f
	//RamWriteA(0x1450,0x0CCD);   //Hall AD offset X set fix to 0.1f
	//modify end

    addrotp=0x14;dataotp=g_ofilm_ois_otp_data.Hall_ad_offset_y;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall AD offset Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x14D0,dataotp);  //Hall AD offset Y

    addrotp=0x09;dataotp=g_ofilm_ois_otp_data.Loop_gain_x;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Loop gain X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x10D3,dataotp);  //Loop gain X

    addrotp=0x0B;dataotp=g_ofilm_ois_otp_data.Loop_gain_y;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Loop gain Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x11D3,dataotp);  //Loop gain Y

RamAccFixMod_OFILM(OFF_OFILM); //32bit Float mode
    addrotp=0x0D;dataotp=g_ofilm_ois_otp_data.Gyro_offset_x_m;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset X M 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a0,dataotp);  //Gyro offset X M
    addrotp=0x0E;dataotp=g_ofilm_ois_otp_data.Gyro_offset_x_l;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset X L 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a1,dataotp);  //Gyro offset X L
    addrotp=0x0F;dataotp=g_ofilm_ois_otp_data.Gyro_offset_y_m;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset Y M 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a2,dataotp);  //Gyro offset Y M
    addrotp=0x10;dataotp=g_ofilm_ois_otp_data.Gyro_offset_y_l;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset Y L 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a3,dataotp);  //Gyro offset Y L

    addrotp=0x11;dataotp=g_ofilm_ois_otp_data.OSC;  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]OSC 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x0257,dataotp);//OSC

    addrotp=0x16;
    dataotp=g_ofilm_ois_otp_data.Gyro_gain_x; 
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro gain X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);    
    RamWrite32A_Ofilm(0x1020,dataotp);  //Gyro gain X
    
    addrotp=0x1A;
    dataotp=g_ofilm_ois_otp_data.Gyro_gain_y; 
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro gain Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);    
    RamWrite32A_Ofilm(0x1120,dataotp);  //Gyro gain Y

    RamWriteA_Ofilm(TCODEH_OFILM, g_u4InitPosition); // focus position
    RtnCen_OFILM(0);
    msleep(100);
    SetPanTiltMode_OFILM(ON_OFILM);// same
    msleep(50);
    OisEna_OFILM(); //same
    //SetH1cMod_OFILM(MOVMODE_OFILM);  //movie mode
   	SetH1cMod_OFILM(0);          //still mode


    addrotp=0x11;dataotp=g_ofilm_ois_otp_data.AF_infinit;
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);

    addrotp=0x13;dataotp=g_ofilm_ois_otp_data.AF_macro;
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_init_drv_ofilm_static - End\n");
}

void LC898122OFILMAF_init_drv_ofilm(void)
{
    unsigned short addrotp;
    unsigned long dataotp=0;
	LC898122OFILMAFDB("LC898122OFILMAF_init_drv_ofilm !!!\n");
	
    //IniSetAf_OFILM();
    IniSet_OFILM();
    IniSetAf_OFILM();
    //modify for ois test
    //RamWrite32A_Ofilm(gxlmt3HS0_OFILM,0x3EB33333);  //0.35f
    //RamWrite32A_Ofilm(gxlmt3HS1_OFILM,0x3EB33333);  //0.35f
    RamWrite32A_Ofilm(gxlmt3HS0_OFILM,0x3E4CCCCD);  //0.2f
    RamWrite32A_Ofilm(gxlmt3HS1_OFILM,0x3E4CCCCD);  //0.2f
    
    //modify end
RamAccFixMod_OFILM(ON_OFILM); //16bit Fix mode
    addrotp=0x05;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);    
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall offset X 0x%x 0x%x\n", addrotp, (unsigned int)dataotp);
    RamWriteA_Ofilm(0x1479,dataotp);  //Hall offset X

    addrotp=0x07;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall offset Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x14F9,dataotp);  //Hall offset Y

    addrotp=0x01;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall bias X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x147A,dataotp);  //Hall bias X

    addrotp=0x03;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall bias Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x14FA,dataotp);  //Hall bias Y

    addrotp=0x12;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall AD offset X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    //modify for ois test
    //RamWriteA(0x1450,dataotp);  //Hall AD offset X
    RamWriteA_Ofilm(0x1450,0x199A);  //Hall AD offset X set fix to 0.2f
    //RamWriteA(0x1450,0x0CCD);   //Hall AD offset X set fix to 0.1f
    //modify end

    addrotp=0x14;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Hall AD offset Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x14D0,dataotp);  //Hall AD offset Y

    addrotp=0x09;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Loop gain X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x10D3,dataotp);  //Loop gain X

    addrotp=0x0B;dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<8)+s4LC898OTP_ReadReg_ofilm(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Loop gain Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RamWriteA_Ofilm(0x11D3,dataotp);  //Loop gain Y

RamAccFixMod_OFILM(OFF_OFILM); //32bit Float mode
    addrotp=0x0D;dataotp=s4LC898OTP_ReadReg_ofilm(addrotp);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset X M 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a0,dataotp);  //Gyro offset X M
    addrotp=0x0E;dataotp=s4LC898OTP_ReadReg_ofilm(addrotp);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset X L 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a1,dataotp);  //Gyro offset X L
    addrotp=0x0F;dataotp=s4LC898OTP_ReadReg_ofilm(addrotp);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset Y M 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a2,dataotp);  //Gyro offset Y M
    addrotp=0x10;dataotp=s4LC898OTP_ReadReg_ofilm(addrotp);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro offset Y L 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x02a3,dataotp);  //Gyro offset Y L

    addrotp=0x11;dataotp=s4LC898OTP_ReadReg_ofilm(addrotp);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]OSC 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    RegWriteA_Ofilm(0x0257,dataotp);//OSC

    addrotp=0x16;
    dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<24)
            +(s4LC898OTP_ReadReg_ofilm(addrotp+1)<<16)
            +(s4LC898OTP_ReadReg_ofilm(addrotp+2)<<8)
            +s4LC898OTP_ReadReg_ofilm(addrotp+3); 
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro gain X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);    
    RamWrite32A_Ofilm(0x1020,dataotp);  //Gyro gain X
    
    addrotp=0x1A;
    dataotp=(s4LC898OTP_ReadReg_ofilm(addrotp)<<24)
            +(s4LC898OTP_ReadReg_ofilm(addrotp+1)<<16)
            +(s4LC898OTP_ReadReg_ofilm(addrotp+2)<<8)
            +s4LC898OTP_ReadReg_ofilm(addrotp+3); 
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]Gyro gain Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);    
    RamWrite32A_Ofilm(0x1120,dataotp);  //Gyro gain Y

    RamWriteA_Ofilm(TCODEH_OFILM, g_u4InitPosition); // focus position
    RtnCen_OFILM(0);
    msleep(100);
    SetPanTiltMode_OFILM(ON_OFILM);// same
    msleep(50);
    OisEna_OFILM(); //same
    //SetH1cMod_OFILM(MOVMODE_OFILM);  //movie mode
   	SetH1cMod_OFILM(0);          //still mode


    addrotp=0x11;dataotp=(s4LC898OTP_ReadReg(addrotp)<<8)+s4LC898OTP_ReadReg(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);

    addrotp=0x13;dataotp=(s4LC898OTP_ReadReg(addrotp)<<8)+s4LC898OTP_ReadReg(addrotp+1);  
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_init_drv_ofilm -End\n");
}

inline static int moveLC898122OFILMAF(unsigned long a_u4Position)
{
    unsigned short delay_count = 0;

    if((a_u4Position > g_u4LC898122OFILMAF_MACRO) || (a_u4Position < g_u4LC898122OFILMAF_INF))
    {
        LC898122OFILMAFDB("[LC898122OFILMAF] out of range \n");
        return -EINVAL;
    }

#if 0
    if (g_s4LC898122OFILMAF_Opened == 1)
    {
        //LC898122OFILMAF_init_drv();

		{
        	//LC898122OFILMAF_init_drv_ofilm();//ofilm module
		}
        spin_lock(&g_LC898122OFILMAF_SpinLock);
        g_u4CurrPosition = g_u4InitPosition;
        g_s4LC898122OFILMAF_Opened = 2;
        spin_unlock(&g_LC898122OFILMAF_SpinLock);
    }
#else
    if (g_s4LC898122OFILMAF_Opened == 2)
    {
	    for (delay_count = 0; delay_count < 100; delay_count++) 
		{
			if(g_s4LC898122OFILMAF_Inited == 0)
			{
				mdelay(5);
				LC898122OFILMAFDB("[LC898122OFILMAF] delay 5ms for ois_ofilm init done!\n");
			}
			else
			{
				LC898122OFILMAFDB("[LC898122OFILMAF] ois_ofilm init delay_count = %d\n", delay_count);
				break;
			}
		}
        g_s4LC898122OFILMAF_Opened = 3;
    }

    if (g_s4LC898122OFILMAF_Opened == 1)
    {
        spin_lock(&g_LC898122OFILMAF_SpinLock);
        g_u4CurrPosition = g_u4InitPosition;
        g_s4LC898122OFILMAF_Opened = 2;
        spin_unlock(&g_LC898122OFILMAF_SpinLock);
    }

#endif
	
    if (g_u4CurrPosition < a_u4Position)
    {
        spin_lock(&g_LC898122OFILMAF_SpinLock);    
        g_i4Dir = 1;
        spin_unlock(&g_LC898122OFILMAF_SpinLock);    
    }
    else if (g_u4CurrPosition > a_u4Position)
    {
        spin_lock(&g_LC898122OFILMAF_SpinLock);    
        g_i4Dir = -1;
        spin_unlock(&g_LC898122OFILMAF_SpinLock);            
    }
    else   return 0; 

    spin_lock(&g_LC898122OFILMAF_SpinLock);    
    g_u4TargetPosition = a_u4Position;
    g_sr = 3;
    g_i4MotorStatus = 0;
    spin_unlock(&g_LC898122OFILMAF_SpinLock);    
	RamWriteA_Ofilm(TCODEH_OFILM, g_u4TargetPosition);

	//RamWriteA_Ofilm(TCODEH_OFILM, 200);

    spin_lock(&g_LC898122OFILMAF_SpinLock);        
    g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
    spin_unlock(&g_LC898122OFILMAF_SpinLock);                

    return 0;
}

inline static int setLC898122OFILMAFInf(unsigned long a_u4Position)
{
    spin_lock(&g_LC898122OFILMAF_SpinLock);
    g_u4LC898122OFILMAF_INF = a_u4Position;
    spin_unlock(&g_LC898122OFILMAF_SpinLock);    
    return 0;
}

inline static int setLC898122OFILMAFMacro(unsigned long a_u4Position)
{
    spin_lock(&g_LC898122OFILMAF_SpinLock);
    g_u4LC898122OFILMAF_MACRO = a_u4Position;
    spin_unlock(&g_LC898122OFILMAF_SpinLock);    
    return 0;    
}

inline static int setLC898122OFILMAFOIS_En(unsigned long a_u4Position)
{
    LC898122OFILMAFDB("[LC898122OFILMAF]setLC898122OFILMAFOIS_En a_u4Position.\n");

	if(a_u4Position&4)
	{
	LC898122OFILMAFDB("[LC898122OFILMAF]setLC898122OFILMAFOIS_En disable OIS.\n");
	    RtnCen_OFILM(0);
	}
	else
	{
    	LC898122OFILMAFDB("[LC898122OFILMAF]setLC898122OFILMAFOIS_En enable OIS.\n");
	    OisEna_OFILM();
	}
	
    return 0;    
}
////////////////////////////////////////////////////////////////
static long LC898122OFILMAF_Ioctl(
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
{
    long i4RetValue = 0;

    switch(a_u4Command)
    {
        case LC898122OFILMAFIOC_G_MOTORINFO :
            i4RetValue = getLC898122OFILMAFInfo((__user stLC898122OFILMAF_MotorInfo *)(a_u4Param));
        break;

        case LC898122OFILMAFIOC_T_MOVETO :
            i4RetValue = moveLC898122OFILMAF(a_u4Param);
        break;
 
        case LC898122OFILMAFIOC_T_SETINFPOS :
            i4RetValue = setLC898122OFILMAFInf(a_u4Param);
        break;

        case LC898122OFILMAFIOC_T_SETMACROPOS :
            i4RetValue = setLC898122OFILMAFMacro(a_u4Param);
        break;

        case LC898122OFILMAFIOC_T_SETPARA :
            i4RetValue = setLC898122OFILMAFOIS_En(a_u4Param);
        break;
		
        default :
              LC898122OFILMAFDB("[LC898122OFILMAF] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    return i4RetValue;
}
int LC898122OFILMAF_Ois_Init_Thread(void *unused)
{
	if(g_ofilm_ois_otp_data.valid_flag == false)
	{
		LC898122OFILMAF_init_drv_ofilm();//ofilm module
	}
	else
	{
		LC898122OFILMAF_init_drv_ofilm_static();//ofilm module
	}
	g_s4LC898122OFILMAF_Inited = 1;
	 
    return 0;
}

struct task_struct  *ois_ofilm_thread;

//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
// 3.Update f_op pointer.
// 4.Fill data structures into private_data
//CAM_RESET
static int LC898122OFILMAF_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Open - Start\n");
    if(g_s4LC898122OFILMAF_Opened)
    {    
        LC898122OFILMAFDB("[LC898122OFILMAF] the device is opened \n");
        return -EBUSY;
    }
	
	spin_lock(&g_LC898122OFILMAF_SpinLock);
    g_s4LC898122OFILMAF_Opened = 1;
    spin_unlock(&g_LC898122OFILMAF_SpinLock);
    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Open - End\n");
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int LC898122OFILMAF_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - Start\n");

    /*unsigned long TargetPosition = 0;	//add by malp 20130404
    unsigned long  tep_temp = 30 ;
    unsigned long  Min_Position = 200 ; //by o-film
    int i_temp=0;*/

    //RtnCen_OFILM(0);
    //msleep(50);
    //SrvCon_OFILM(X_DIR_OFILM,OFF_OFILM);
    //SrvCon_OFILM(Y_DIR_OFILM,OFF_OFILM);

    if (g_s4LC898122OFILMAF_Opened)
    {
        LC898122OFILMAFDB("[LC898122OFILMAF] feee \n");
        g_sr = 5;
	        
	/*tep_temp=20;
	Min_Position=80;
	i_temp=0;
	TargetPosition = g_u4CurrPosition;
	while (TargetPosition>Min_Position)
	{
		TargetPosition = TargetPosition - tep_temp;
		RamWriteA_Ofilm(TCODEH_OFILM, TargetPosition);
		LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release TargetPosition=%u\n",(unsigned int)TargetPosition );
		msleep(20);
		i_temp++;
		if(i_temp>=50)
		{
			break;
		}
	}*/

	//RamWrite32A_Ofilm(0x1461,0x3D4CCCCD);	//0.05

	//RamWrite32A_Ofilm(0x14E1,0x3E4CCCCD);	//0.2

	//LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - X arrow 3D4CCCCD!!!!!\n");	

	//LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - Y arrow 3E4CCCCD!!!!!\n");	

	//msleep(20);

	//RamWrite32A_Ofilm(0x1461,0x3DCCCCCD);	//0.1

	//LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - X arrow 3DCCCCCD!!!!!\n");

	//msleep(20);	

	//RamWrite32A_Ofilm(0x1461,0x3E99999A);	//0.3

	//RamWrite32A_Ofilm(0x14E1,0x3E4CCCCD);

	//LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - X arrow 3E99999A!!!!!\n");    	

	//LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - Y arrow 3E99999A!!!!!\n");

    //msleep(20);

    //RamWrite32A_Ofilm(0x1461,0x3F000000);	//0.5

	//LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - X arrow 3F000000!!!!!\n");    	

	//msleep(20);

    /*RamWrite32A_Ofilm(0x1461,0x3F0CCCCD);	//0.55

    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - X arrow 3F0CCCCD!!!!!\n");*/

    //RamWrite32A_Ofilm(0x1461,0x3F19999A);	//0.6

    //LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - X arrow 3F19999A!!!!!\n");

	//msleep(20);
                    
        spin_lock(&g_LC898122OFILMAF_SpinLock);
        g_s4LC898122OFILMAF_Opened = 0;
		//g_s4LC898122OFILMAF_Inited = 0;
        spin_unlock(&g_LC898122OFILMAF_SpinLock);

    }
    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_Release - End\n");
    //RtnCen(0);
    //SrvCon(X_DIR,OFF);
    //SrvCon(Y_DIR,OFF);
    return 0;
}

static const struct file_operations g_stLC898122OFILMAF_fops = 
{
    .owner = THIS_MODULE,
    .open = LC898122OFILMAF_Open,
    .release = LC898122OFILMAF_Release,
    .unlocked_ioctl = LC898122OFILMAF_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = LC898122OFILMAF_Ioctl,
#endif
};

inline static int Register_LC898122OFILMAF_CharDrv(void)
{
    struct device* vcm_device = NULL;

    LC898122OFILMAFDB("[LC898122OFILMAF] Register_LC898122OFILMAF_CharDrv - Start\n");

    //Allocate char driver no.
    if( alloc_chrdev_region(&g_LC898122OFILMAF_devno, 0, 1,LC898122OFILMAF_DRVNAME) )
    {
        LC898122OFILMAFDB("[LC898122OFILMAF] Allocate device no failed\n");

        return -EAGAIN;
    }

    //Allocate driver
    g_pLC898122OFILMAF_CharDrv = cdev_alloc();

    if(NULL == g_pLC898122OFILMAF_CharDrv)
    {
        unregister_chrdev_region(g_LC898122OFILMAF_devno, 1);

        LC898122OFILMAFDB("[LC898122OFILMAF] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pLC898122OFILMAF_CharDrv, &g_stLC898122OFILMAF_fops);

    g_pLC898122OFILMAF_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pLC898122OFILMAF_CharDrv, g_LC898122OFILMAF_devno, 1))
    {
        LC898122OFILMAFDB("[LC898122OFILMAF] Attatch file operation failed\n");

        unregister_chrdev_region(g_LC898122OFILMAF_devno, 1);

        return -EAGAIN;
    }

    actuator_class = class_create(THIS_MODULE, "actuatordrv_OIS_ofilm");
    if (IS_ERR(actuator_class)) {
        int ret = PTR_ERR(actuator_class);
        LC898122OFILMAFDB("Unable to create class, err = %d\n", ret);
        return ret;            
    }

    vcm_device = device_create(actuator_class, NULL, g_LC898122OFILMAF_devno, NULL, LC898122OFILMAF_DRVNAME);

    if(NULL == vcm_device)
    {
        return -EIO;
    }
    
    LC898122OFILMAFDB("[LC898122OFILMAF] Register_LC898122OFILMAF_CharDrv - End\n");    
    return 0;
}

inline static void Unregister_LC898122OFILMAF_CharDrv(void)
{
    LC898122OFILMAFDB("[LC898122OFILMAF] Unregister_LC898122OFILMAF_CharDrv - Start\n");

    //Release char driver
    cdev_del(g_pLC898122OFILMAF_CharDrv);

    unregister_chrdev_region(g_LC898122OFILMAF_devno, 1);
    
    device_destroy(actuator_class, g_LC898122OFILMAF_devno);

    class_destroy(actuator_class);

    LC898122OFILMAFDB("[LC898122OFILMAF] Unregister_LC898122OFILMAF_CharDrv - End\n");    
}

//////////////////////////////////////////////////////////////////////

static int LC898122OFILMAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int LC898122OFILMAF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id LC898122OFILMAF_i2c_id[] = {{LC898122OFILMAF_DRVNAME,0},{}};   
struct i2c_driver LC898122OFILMAF_i2c_driver = {                       
    .probe = LC898122OFILMAF_i2c_probe,                                   
    .remove = LC898122OFILMAF_i2c_remove,                           
    .driver.name = LC898122OFILMAF_DRVNAME,                 
    .id_table = LC898122OFILMAF_i2c_id,                             
};  

#if 0 
static int LC898122OFILMAF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, LC898122OFILMAF_DRVNAME);                                                         
    return 0;                                                                                       
}      
#endif 
static int LC898122OFILMAF_i2c_remove(struct i2c_client *client) {
    return 0;
}

/* Kirby: add new-style driver {*/
static int LC898122OFILMAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int i4RetValue = 0;

    LC898122OFILMAFDB("[LC898122OFILMAF] LC898122OFILMAF_i2c_probe\n");

    /* Kirby: add new-style driver { */
    g_pstLC898122OFILMAF_I2Cclient = client;
    
    g_pstLC898122OFILMAF_I2Cclient->addr = g_pstLC898122OFILMAF_I2Cclient->addr >> 1;
    g_pstLC898122OFILMAF_I2Cclient->timing = 400;
	
    //Register char driver
    i4RetValue = Register_LC898122OFILMAF_CharDrv();

    if(i4RetValue){

        LC898122OFILMAFDB("[LC898122OFILMAF] register char device failed!\n");

        return i4RetValue;
    }

    spin_lock_init(&g_LC898122OFILMAF_SpinLock);

    LC898122OFILMAFDB("[LC898122OFILMAF] Attached!! \n");

    return 0;
}

static int LC898122OFILMAF_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&LC898122OFILMAF_i2c_driver);
}

static int LC898122OFILMAF_remove(struct platform_device *pdev)
{
    i2c_del_driver(&LC898122OFILMAF_i2c_driver);
    return 0;
}

static int LC898122OFILMAF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int LC898122OFILMAF_resume(struct platform_device *pdev)
{
    return 0;
}

// platform structure
static struct platform_driver g_stLC898122OFILMAF_Driver = {
    .probe        = LC898122OFILMAF_probe,
    .remove    = LC898122OFILMAF_remove,
    .suspend    = LC898122OFILMAF_suspend,
    .resume    = LC898122OFILMAF_resume,
    .driver        = {
        .name    = "lens_actuator_ois_ofilm",
        .owner    = THIS_MODULE,
    }
};
static struct platform_device g_stAF_device = {
    .name = "lens_actuator_ois_ofilm",
    .id = 0,
    .dev = {}
};

static int __init LC898122OFILMAF_i2C_init(void)
{
    i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);

    if(platform_device_register(&g_stAF_device)){
        LC898122OFILMAFDB("failed to register AF driver\n");
        return -ENODEV;
    }

    if(platform_driver_register(&g_stLC898122OFILMAF_Driver)){
        LC898122OFILMAFDB("failed to register LC898122OFILMAF driver\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit LC898122OFILMAF_i2C_exit(void)
{
    platform_driver_unregister(&g_stLC898122OFILMAF_Driver);
}

module_init(LC898122OFILMAF_i2C_init);
module_exit(LC898122OFILMAF_i2C_exit);

MODULE_DESCRIPTION("LC898122OFILMAF lens module driver");
MODULE_AUTHOR("KY Chen <vend_james-cc.wu@Mediatek.com>");
MODULE_LICENSE("GPL");

