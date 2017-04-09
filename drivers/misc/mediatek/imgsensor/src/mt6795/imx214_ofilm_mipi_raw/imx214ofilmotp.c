/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "imx214ofilmotp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALDB printk
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0
extern u8 Ofilm_imx214_OTPData[];

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV_OFILM"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x6e>>1)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

#define LSCOTPDATASIZE 0x03c4 //964

int otp_ofilm_imx214_flag=0;

static kal_uint8 lscotpdata[LSCOTPDATASIZE];
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

static int iReadCAM_CAL_8(u8 a_u2Addr, u8 * a_puBuff, u16 i2c_id)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)(a_u2Addr)};
    int retry = 10;

    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = i2c_id>>1;
	g_pstI2Cclient->timing=100;
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
    spin_unlock(&g_CAM_CALLock); // for SMP    

    do {
	    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);

	    printk("%s, addr:%x, i2c_id:%x \n", "iReadCAM_CAL_8",  a_u2Addr, i2c_id) ;
		
	    if (i4RetValue != 1)
	    {
	        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
		    CAM_CALDB("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
	    }
		else
    	{
		    break;
		}
		
		udelay(50);
		
		if(retry == 0)
		{
	        CAM_CALDB("[CAM_CAL] retry 0 I2C send read address failed!! \n");
	        return -1;
		}
    } while ((retry--) > 0);
	
	retry = 10;

    do {
	    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
		
		CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
	    if (i4RetValue != 1)
	    {
	        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
	    } 
		else
		{
		    break;
		}

		udelay(50);
		
		if(retry == 0)
		{
	        CAM_CALDB("[CAM_CAL] retry 0 I2C read data failed!! \n");
	        return -1;
		}
    } while ((retry--) > 0);
	
    return 0;
}

kal_uint8 IMX214_Read_module_id_ofilm(void)
{	
	u8 readbuff = 0;
	u8 readbuff_RG = 0;
	u8 readbuff_BG = 0;
	static kal_bool OFILM_IMX214_MODULE = FALSE;

	if(OFILM_IMX214_MODULE == TRUE)
	{
		CAM_CALDB("OFILM_IMX214_MODULE == TRUE  !!\n");
		return 7;
	}

	CAM_CALDB("IMX214_Read_module_id_ofilm  !!\n");
	iReadCAM_CAL_8(0x80,&readbuff,0xA0);
	iReadCAM_CAL_8(0x05,&readbuff_RG,0xA0);
	iReadCAM_CAL_8(0x06,&readbuff_BG,0xA0);
	
	if(readbuff == 0x01)
	{
		if((readbuff_RG != 0)&&(readbuff_BG != 0))
		{
			CAM_CALDB("Sharp IMX214\n");
			return 1;
		}
		else
		{
			iReadCAM_CAL_8(0xDB,&readbuff,0xA6);
			
			if(readbuff == 0x07)
			{
				CAM_CALDB("Ofilm IMX214 strange\n");
				OFILM_IMX214_MODULE = TRUE;
				return 7;
			}
		}
	}
	else
	{
		iReadCAM_CAL_8(0xDB,&readbuff,0xA6);
		if(readbuff == 0x07)
		{
			CAM_CALDB("Ofilm IMX214\n");
			OFILM_IMX214_MODULE = TRUE;
			return 7;
		}
			
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

ofilm_ois_otp_struct g_ofilm_ois_otp_data = {
	
	.Hall_offset_x = 0,	//Hall offset X
	.Hall_offset_y = 0,	//Hall offset Y
	.Hall_bias_x = 0,	//Hall bias X
	.Hall_bias_y = 0,	//Hall bias Y
	.Hall_ad_offset_x = 0,	//Hall AD offset X
	.Hall_ad_offset_y = 0,	//Hall AD offset Y
	.Loop_gain_x = 0,	//Loop gain X
	.Loop_gain_y = 0,	//Loop gain Y

	.Gyro_offset_x_m = 0,	//Gyro offset X M
	.Gyro_offset_x_l = 0,	//Gyro offset X L
	.Gyro_offset_y_m = 0,	//Gyro offset Y M
	.Gyro_offset_y_l = 0,	//Gyro offset Y L

	.OSC = 0,	//OSC
	
	.Gyro_gain_x = 0,	//Gyro gain X
	.Gyro_gain_y = 0,	//Gyro gain Y
	
	.AF_start = 0,	//AF start
	.AF_infinit = 0,	//AF Infinit
	.AF_macro = 0,	//AF Macro
	.valid_flag = false,
};


static unsigned char Ofilm_LC898OTP_ReadReg(unsigned short RegAddr)
{
	u8 readbuff = 0xff;
	
	iReadCAM_CAL_8(RegAddr,&readbuff,0xA8);

	return readbuff;
	
}

static unsigned char Ofilm_LC898OTP_AF_ReadReg(unsigned short RegAddr)
{
	u8 readbuff = 0xff;
	
	iReadCAM_CAL_8(RegAddr,&readbuff,0xA0);

	return readbuff;
	
}

static kal_uint8 IMX214_Read_Ois_Otp_Ofilm(void)
{	
	unsigned short addrotp;
	unsigned long dataotp=0;

	addrotp=0x05;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);	
	CAM_CALDB("[LC898122OFILMAFOTP]Hall offset X 0x%x  0x%x\n", addrotp, (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Hall_offset_x = dataotp;	//Hall offset X

	addrotp=0x07;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Hall offset Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Hall_offset_y = dataotp;	//Hall offset Y

	addrotp=0x01;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Hall bias X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Hall_bias_x = dataotp; //Hall bias X

	addrotp=0x03;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Hall bias Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Hall_bias_y = dataotp; //Hall bias Y

	addrotp=0x12;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Hall AD offset X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Hall_ad_offset_x = dataotp;  //Hall AD offset X

	addrotp=0x14;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Hall AD offset Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Hall_ad_offset_y = dataotp;	//Hall AD offset Y

	addrotp=0x09;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Loop gain X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Loop_gain_x = dataotp; //Loop gain X

	addrotp=0x0B;dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]Loop gain Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Loop_gain_y = dataotp; //Loop gain Y

	addrotp=0x0D;dataotp=Ofilm_LC898OTP_ReadReg(addrotp);  
	CAM_CALDB("[LC898122OFILMAFOTP]Gyro offset X M 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Gyro_offset_x_m = dataotp; //Gyro offset X M
	addrotp=0x0E;dataotp=Ofilm_LC898OTP_ReadReg(addrotp);  
	CAM_CALDB("[LC898122OFILMAFOTP]Gyro offset X L 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Gyro_offset_x_l = dataotp; //Gyro offset X L
	addrotp=0x0F;dataotp=Ofilm_LC898OTP_ReadReg(addrotp);  
	CAM_CALDB("[LC898122OFILMAFOTP]Gyro offset Y M 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Gyro_offset_y_m = dataotp; //Gyro offset Y M
	addrotp=0x10;dataotp=Ofilm_LC898OTP_ReadReg(addrotp);  
	CAM_CALDB("[LC898122OFILMAFOTP]Gyro offset Y L 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.Gyro_offset_y_l = dataotp; //Gyro offset Y L

	addrotp=0x11;dataotp=Ofilm_LC898OTP_ReadReg(addrotp);  
	CAM_CALDB("[LC898122OFILMAFOTP]OSC 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.OSC = dataotp;//OSC

	addrotp=0x16;
	dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<24)
			+(Ofilm_LC898OTP_ReadReg(addrotp+1)<<16)
			+(Ofilm_LC898OTP_ReadReg(addrotp+2)<<8)
			+Ofilm_LC898OTP_ReadReg(addrotp+3); 
	CAM_CALDB("[LC898122OFILMAFOTP]Gyro gain X 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);	  
	g_ofilm_ois_otp_data.Gyro_gain_x= dataotp;	//Gyro gain X

	addrotp=0x1A;
	dataotp=(Ofilm_LC898OTP_ReadReg(addrotp)<<24)
			+(Ofilm_LC898OTP_ReadReg(addrotp+1)<<16)
			+(Ofilm_LC898OTP_ReadReg(addrotp+2)<<8)
			+Ofilm_LC898OTP_ReadReg(addrotp+3); 
	CAM_CALDB("[LC898122OFILMAFOTP]Gyro gain Y 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);	  
	g_ofilm_ois_otp_data.Gyro_gain_y= dataotp;	//Gyro gain Y

	addrotp=0x11;dataotp=(Ofilm_LC898OTP_AF_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_AF_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]AF Infinit 0x%x 0x%x\n", addrotp,  (unsigned int)dataotp);
	g_ofilm_ois_otp_data.AF_infinit = dataotp;	//AF Infinit 

	addrotp=0x13;dataotp=(Ofilm_LC898OTP_AF_ReadReg(addrotp)<<8)+Ofilm_LC898OTP_AF_ReadReg(addrotp+1);  
	CAM_CALDB("[LC898122OFILMAFOTP]AF Macro 0x%x 0x%x\n", addrotp,	(unsigned int)dataotp);
	g_ofilm_ois_otp_data.AF_macro = dataotp;  //AF Macro

	g_ofilm_ois_otp_data.valid_flag = true;  //valid flag
	CAM_CALDB("[LC898122OFILMAF] IMX214_Read_Ois_Otp_Ofilm - End\n");

	return 1;

}
static kal_uint8 IMX214_Read_AF_Otp_Ofilm(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{	
	u8 readbuff, i;
	 kal_uint16 AFOTPaddress = 0x11 ; 
	static u8 ofilm_af_otp[7] = {0,0,0,0,0,0,0};

	if(ofilm_af_otp[6] == 1)
	{
		for(i=0;i<buffersize;i++)
		{
			*(iBuffer+i) = ofilm_af_otp[i];
			CAM_CALDB("Ofilm read af static data[%d]=0x%x\n",i,iBuffer[i]);
		}
		return KAL_TRUE;
	}

	for(i=0;i<buffersize;i++)
	{
		iReadCAM_CAL_8((AFOTPaddress+i),&readbuff,0xA0);
		*(iBuffer+i) = readbuff;
		ofilm_af_otp[i] = readbuff;
		CAM_CALDB("Ofilm read af data[%d]=0x%x\n",i,iBuffer[i]);
	}

	ofilm_af_otp[6] = 1;//flag
	
	return KAL_TRUE;

}


static kal_uint8 IMX214_Read_AWB_Otp_Ofilm(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	u8 readbuff, i;
	 kal_uint16 AWBOTPaddress = 0x09 ; 
	static u8 ofilm_awb_otp[5] = {0,0,0,0,0};

	if(ofilm_awb_otp[4] == 1)
	{
		for(i=0;i<buffersize;i++)
		{
			*(iBuffer+i) = ofilm_awb_otp[i];
			CAM_CALDB("ofilm read awb static data[%d]=0x%x\n",i,iBuffer[i]);
		}
		return KAL_TRUE;
	}

	for(i=0;i<buffersize;i++)
	{
		iReadCAM_CAL_8((AWBOTPaddress+i),&readbuff,0xA0);
		*(iBuffer+i)=readbuff;
		ofilm_awb_otp[i] = readbuff;
		CAM_CALDB("ofilm read awb data[%d]=0x%x\n",i,iBuffer[i]);
	}
	
	ofilm_awb_otp[4] = 1;//flag
	
	return KAL_TRUE;
}

 
 
static kal_bool IMX214_Read_LSC_Otp_Ofilm(u16 Outdatalen,unsigned char * pOutputdata)
 {
 
 kal_uint8 page = 0;
 kal_uint16 byteperpage = 256;
 kal_uint16 number = 0;
 kal_uint16 LSCOTPaddress = 0x00 ; 
 u8 readbuff;
 int i = 0;

 if(otp_ofilm_imx214_flag){

	for(i=0;i<Outdatalen;i++)
		pOutputdata[i]=Ofilm_imx214_OTPData[i];
 }
else{

	 for(page = 0; page<=3; page++)
	 {
	   CAM_CALDB("Ofilm read page=%d\n",page);
   
	   if(page == 0){
		   for(i = 0x17; i <=0xFF;i++)
		   {
			 iReadCAM_CAL_8(LSCOTPaddress+i,&readbuff,(0xA0+2*page));
			 pOutputdata[number]=readbuff;
			 number+=1;
		   }
	   }
	   else if (page == 3)
   	   {
		   for(i = 0x00; i <=0xDA;i++)
		   {
			 iReadCAM_CAL_8(LSCOTPaddress+i,&readbuff,(0xA0+2*page));
			 pOutputdata[number]=readbuff;
			 number+=1;
		   }
	   }
	   else{
		  for(i = 0; i <=0xFF;i++)
		  {
			iReadCAM_CAL_8(LSCOTPaddress+i,&readbuff,(0xA0+2*page));
			pOutputdata[number]=readbuff;
			number+=1;
   
		  }
	   }
   
   }

}
     CAM_CALDB(" Ofilm LSC read finished number= %d\n",--number);
	 otp_ofilm_imx214_flag=0;
	 return KAL_TRUE;
 
 }

 void IMX214_ReadOtp_Ofilm(kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{
		kal_uint16 i = 0;
		u8 readbuff, base_add;
		int ret ;
		//base_add=(address/10)*16+(address%10);
			
		CAM_CALDB("[CAM_CAL]ENTER address:0x%x buffersize:%d\n ",address, buffersize);
		if (1)
		{
			for(i = 0; i<buffersize; i++)
			{				
				ret= iReadCAM_CAL_8(address+i,&readbuff,0xA6);
				CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
				*(iBuffer+i) =(unsigned char)readbuff;
			}
		}
}

//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
}

int boot_read_ofilm_2a_otp(void)
{
	u8 temp_2a_otp[8];
	
	IMX214_Read_AWB_Otp_Ofilm(0x09, temp_2a_otp, 4);
	IMX214_Read_AF_Otp_Ofilm(0x11, temp_2a_otp, 6);
	IMX214_Read_Ois_Otp_Ofilm();
}

//Burst Read Data
 int iReadData_Ofilm(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
	
	if(ui4_length ==1)
    {	
	    IMX214_ReadOtp_Ofilm(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==4)
    {
		
	    IMX214_Read_AWB_Otp_Ofilm(ui4_offset, pinputdata, ui4_length);
	   	
    }
	else if(ui4_length ==6)
    {
		
	    IMX214_Read_AF_Otp_Ofilm(ui4_offset, pinputdata, ui4_length);
	   	
   	}
	else{

		//if(IMX214_Read_LSC_Otp(ui4_length,pinputdata))

		if(IMX214_Read_module_id_ofilm() == 0x07)
		{
		    IMX214_Read_LSC_Otp_Ofilm(ui4_length,pinputdata);
		}


	}
    //2. read otp

    for(i4RetValue = 0;i4RetValue<ui4_length;i4RetValue++){
    CAM_CALDB( "[[CAM_CAL]]pinputdata[%d]=%x\n", i4RetValue,*(pinputdata+i4RetValue));}
    CAM_CALDB(" [[CAM_CAL]]ui4_length = %d,ui4_offset =%d\n ",ui4_length,ui4_offset);
    CAM_CALDB("[S24EEPORM] iReadData_Ofilm done\n" );
   return 0;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long cat24c16_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
	CAM_CALDB("[CAMERA SENSOR] cat24c16_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        CAM_CALDB("[CAMERA SENSOR] COMPAT_CAM_CALIOC_G_READ\n");
        COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
        stCAM_CAL_INFO_STRUCT __user *data;
        int err;

        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALDB("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif


/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pWorkingBuff = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
	
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB("[S24CAM_CAL] init Working buffer address 0x%p  command is 0x%8x\n", pWorkingBuff, (u32)a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif     
            CAM_CALDB("[CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[CAM_CAL] length %d \n", ptempbuf->u4Length);
            CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%p \n", pWorkingBuff);
			otp_ofilm_imx214_flag=1;
            i4RetValue = iReadData_Ofilm((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			
            CAM_CALDB("[S24CAM_CAL] After read Working buffer data  0x%4x \n", *pWorkingBuff);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     CAM_CALDB("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%p \n", pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pWorkingBuff);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = cat24c16_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_OFILM");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   
#if 0 //test110314 Please use the same I2C Group ID as Sensor
static unsigned short force[] = {CAM_CAL_I2C_GROUP_ID, IMX214OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#else
//static unsigned short force[] = {IMG_SENSOR_I2C_GROUP_ID, OV5647OTP_DEVICE_ID, I2C_CLIENT_END, I2C_CLIENT_END};   
#endif
//static const unsigned short * const forces[] = { force, NULL };              
//static struct i2c_client_address_data addr_data = { .forces = forces,}; 


static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect,                           
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             
int i4RetValue = 0;
    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = IMX214OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S24CAM_CAL] Attached!! \n");
    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{
    i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL_OFILM driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


