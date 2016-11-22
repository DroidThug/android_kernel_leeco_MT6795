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
#include "s5k3l9otp.h"
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

static DEFINE_SPINLOCK(g_CAM_3L9_CALLock); // for SMP
#define CAM_CAL_I2C_BUSNUM 0

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV_3L9"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0x5f)};

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_3L9_CALLock;
//spin_lock(&g_CAM_3L9_CALLock);
//spin_unlock(&g_CAM_3L9_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);
struct s5k3l9_otp_struct g_s5k3l9_otp_struct;



/*************************************************************************
* FUNCTION
*	iReadCAM_CAL_16
*
* DESCRIPTION
*	
*
* PARAMETERS
*	
*
* RETURNS
*	int
*
* GLOBALS AFFECTED
*
*************************************************************************/
static int iReadCAM_CAL_16(u16 a_u2Addr, u8 * a_puBuff, u16 i2c_id)
{
    int  i4RetValue = 0;
    kal_uint16 get_byte=0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8), (char)(a_u2Addr & 0xFF) };

    CAM_CALDB("iReadCAM_CAL_16 start !! \n");

    spin_lock(&g_CAM_3L9_CALLock); //for SMP
    g_pstI2Cclient->addr = i2c_id>>1;
	g_pstI2Cclient->timing=100;
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG);
    spin_unlock(&g_CAM_3L9_CALLock); // for SMP    
    CAM_CALDB("iReadCAM_CAL_16 start22 !! \n");
	
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);

    printk("%s, addr:%x, i2c_id:%x \n", "iReadCAM_CAL_16",  a_u2Addr, i2c_id) ;
	
    if (i4RetValue != 2)
    {
        CAM_CALDB("[S5K3L9_CAL] I2C send read address failed!! \n");
	    CAM_CALDB("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );
    }
	
	udelay(50);


    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	
	CAM_CALDB("[S5K3L9_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != 1)
    {
        CAM_CALDB("[S5K3L9_CAL] I2C read data failed!! \n");
    } 
	
	udelay(50);
		
    return 0;
}
/*************************************************************************
* FUNCTION
*	S5K3L9_ReadOtp
*
* DESCRIPTION
*	S5K3L9_ReadOtp
*
* PARAMETERS
*	
*
* RETURNS
*	int
*
* GLOBALS AFFECTED
*
*************************************************************************/
u8 S5K3L9_ReadOtp(kal_uint16 address)
{
	u8 readbuff;
	int ret ;
		
	CAM_CALDB("[S5K3L9_CAL]ENTER address:0x%x \n ",address);
	ret= iReadCAM_CAL_16(address,&readbuff,0xA0);
	return readbuff;
}
/*************************************************************************
* FUNCTION
*	S5K3L9_Read_module_id
*
* DESCRIPTION
*	S5K3L9_Read_module_id
*
* PARAMETERS
*	
*
* RETURNS
*	kal_uint8
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint8 S5K3L9_Read_module_id(void)
{	
	if(g_s5k3l9_otp_struct.flag == 0)
	{
		return S5K3L9_ReadOtp(0x0000);
	}
	else
	{
		g_s5k3l9_otp_struct.module_id;
	}

}

/*************************************************************************
* FUNCTION
*	S5K3L9_Read_AWBAF_Otp
*
* DESCRIPTION
*	S5K3L9_Read_AWBAF_Otp
*
* PARAMETERS
*	
*
* RETURNS
*	kal_uint8
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint8 S5K3L9_Read_AWBAF_Otp(kal_uint8 address,unsigned char *iBuffer,unsigned int buffersize)
{
	u8 readbuff, i;
	kal_uint16 AWBAFOTPaddress = 0x07 ; 

	if(buffersize != S5K3L9_AWBAF_OTP_SIZE)
	{
		CAM_CALDB("[S5K3L9_CAL]wrong AWBAFOTPSIZE:0x%x \n ",buffersize);
		return KAL_FALSE;
	}
	
	CAM_CALDB("[S5K3L9_CAL]read awbaf data from eeprom:\n ");
	for(i = 0;i < buffersize;i++)
	{
		iBuffer[i] = S5K3L9_ReadOtp(i + AWBAFOTPaddress);
	}
	
	return KAL_TRUE;
}

 
/*************************************************************************
* FUNCTION
*	S5K3L9_Read_LSC_Otp
*
* DESCRIPTION
*	S5K3L9_Read_LSC_Otp
*
* PARAMETERS
*	
*
* RETURNS
*	kal_bool
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_bool S5K3L9_Read_LSC_Otp(u16 Outdatalen,unsigned char * pOutputdata)
{
	int i;
	kal_uint16 LSCOTPaddress = 0x0017 ; 
	
	if(Outdatalen != S5K3L9_LSC_OTP_SIZE)
	{
		CAM_CALDB("[S5K3L9_CAL]wrong LSCOTPSIZE:0x%x \n ",Outdatalen);
		return KAL_FALSE;
	}
	
	if(g_s5k3l9_otp_struct.flag == 0)
	{
		CAM_CALDB("[S5K3L9_CAL]read lsc data from eeprom:\n ");
		for(i=0;i<Outdatalen;i++)
		{
			pOutputdata[i] = S5K3L9_ReadOtp(i + LSCOTPaddress);
		}
	}
	else
	{
		CAM_CALDB("[S5K3L9_CAL]read lsc static data:\n ");
		for(i=0;i<Outdatalen;i++)
		{
			pOutputdata[i] = g_s5k3l9_otp_struct.lsc_data[i];
		}
	}

	return KAL_TRUE;
}
/*************************************************************************
* FUNCTION
*	S5K3L9_Read_PDAF_Otp
*
* DESCRIPTION
*	S5K3L9_Read_PDAF_Otp
*
* PARAMETERS
*	
*
* RETURNS
*	kal_bool
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_bool S5K3L9_Read_PDAF_Otp(u16 Outdatalen,unsigned char * pOutputdata)
{
	u8 readbuff, i;
	kal_uint16 PDAFOTPaddress = 0x0763; 
	
	if(Outdatalen != S5K3L9_PDAF_OTP_SIZE)
	{
		CAM_CALDB("[S5K3L9_CAL]wrong PDAFOTPSIZE:0x%x \n ",Outdatalen);
		return KAL_FALSE;
	}
	
	if(g_s5k3l9_otp_struct.flag == 0)
	{
		CAM_CALDB("[S5K3L9_CAL]read pdaf data from eeprom:\n ");	
		for(i=0;i<Outdatalen;i++)
		{
			pOutputdata[i] = S5K3L9_ReadOtp(i + PDAFOTPaddress);
		}
	}
	else
	{
		for(i=0;i<Outdatalen;i++)
		{
			pOutputdata[i] = g_s5k3l9_otp_struct.pdaf_data[i];
		}
	}

	return KAL_TRUE;
}
/*************************************************************************
* FUNCTION
*	iWriteData
*
* DESCRIPTION
*	iWriteData
*
* PARAMETERS
*	
*
* RETURNS
*	int
*
* GLOBALS AFFECTED
*
*************************************************************************/
//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	return KAL_FALSE;
}

unsigned short read_3l9_afinfi = 0;
unsigned short read_3l9_afmacro = 0;
/*************************************************************************
* FUNCTION
*	read_s5k3l9_otp
*
* DESCRIPTION
*	read_s5k3l9_otp
*
* PARAMETERS
*	
*
* RETURNS
*	int
*
* GLOBALS AFFECTED
*
*************************************************************************/
int read_s5k3l9_otp(struct s5k3l9_otp_struct *otp_ptr)
{
	kal_uint16 i;
	
	(*otp_ptr).flag = S5K3L9_ReadOtp(0x0000);
	CAM_CALDB("[S5K3L9_CAL]flag:0x%x \n ",(*otp_ptr).flag);
	
	(*otp_ptr).cali_ver = ((S5K3L9_ReadOtp(0x0001))|(S5K3L9_ReadOtp(0x0002)<<8)
						|(S5K3L9_ReadOtp(0x0003)<<16)|(S5K3L9_ReadOtp(0x0004)<<24));
	CAM_CALDB("[S5K3L9_CAL]cali_ver:0x%x \n ",(*otp_ptr).cali_ver);
	
	(*otp_ptr).module_id = S5K3L9_ReadOtp(0x0005);
	CAM_CALDB("[S5K3L9_CAL]module_id:0x%x \n ",(*otp_ptr).module_id);
	
	(*otp_ptr).lens_id = S5K3L9_ReadOtp(0x0006);
	CAM_CALDB("[S5K3L9_CAL]lens_id:0x%x \n ",(*otp_ptr).lens_id);
	
	(*otp_ptr).awbaf_info = ((S5K3L9_ReadOtp(0x0007))|(S5K3L9_ReadOtp(0x0008)<<8));
	CAM_CALDB("[S5K3L9_CAL]awbaf_info:0x%x \n ",(*otp_ptr).awbaf_info);
	(*otp_ptr).wb_u_R = S5K3L9_ReadOtp(0x0009);
	CAM_CALDB("[S5K3L9_CAL]wb_u_R:0x%x \n ",(*otp_ptr).wb_u_R);
	(*otp_ptr).wb_u_Gr = S5K3L9_ReadOtp(0x000A);
	CAM_CALDB("[S5K3L9_CAL]wb_u_Gr:0x%x \n ",(*otp_ptr).wb_u_Gr);
	(*otp_ptr).wb_u_Gb = S5K3L9_ReadOtp(0x000B);
	CAM_CALDB("[S5K3L9_CAL]wb_u_Gb:0x%x \n ",(*otp_ptr).wb_u_Gb);
	(*otp_ptr).wb_u_B = S5K3L9_ReadOtp(0x000C);
	CAM_CALDB("[S5K3L9_CAL]wb_u_B:0x%x \n ",(*otp_ptr).wb_u_B);
	(*otp_ptr).wb_g_R = S5K3L9_ReadOtp(0x000D);
	CAM_CALDB("[S5K3L9_CAL]wb_g_R:0x%x \n ",(*otp_ptr).wb_g_R);
	(*otp_ptr).wb_g_Gr = S5K3L9_ReadOtp(0x000E);
	CAM_CALDB("[S5K3L9_CAL]wb_g_Gr:0x%x \n ",(*otp_ptr).wb_g_Gr);
	(*otp_ptr).wb_g_Gb = S5K3L9_ReadOtp(0x000F);
	CAM_CALDB("[S5K3L9_CAL]wb_g_Gb:0x%x \n ",(*otp_ptr).wb_g_Gb);
	(*otp_ptr).wb_g_B = S5K3L9_ReadOtp(0x0010);
	CAM_CALDB("[S5K3L9_CAL]wb_g_B:0x%x \n ",(*otp_ptr).wb_g_B);
	
	(*otp_ptr).af_infinit = ((S5K3L9_ReadOtp(0x0011))|(S5K3L9_ReadOtp(0x0012)<<8));
	CAM_CALDB("[S5K3L9_CAL]af_infinit:0x%x \n ",(*otp_ptr).af_infinit);
	read_3l9_afinfi = (*otp_ptr).af_infinit;
	(*otp_ptr).af_macro = ((S5K3L9_ReadOtp(0x00013))|(S5K3L9_ReadOtp(0x0014)<<8));
	CAM_CALDB("[S5K3L9_CAL]af_macro:0x%x \n ",(*otp_ptr).af_macro);
	read_3l9_afmacro = (*otp_ptr).af_macro;
	
	(*otp_ptr).lsc_size = ((S5K3L9_ReadOtp(0x00015))|(S5K3L9_ReadOtp(0x0016)<<8));
	CAM_CALDB("[S5K3L9_CAL]lsc_size:0x%x \n ",(*otp_ptr).lsc_size);
	if((*otp_ptr).lsc_size != S5K3L9_LSC_OTP_SIZE)
	{
		CAM_CALDB("[S5K3L9_CAL]wrong lsc_size:0x%x \n ",(*otp_ptr).lsc_size);
		//return KAL_FALSE;
	}

    for(i = 0;i < S5K3L9_LSC_OTP_SIZE;i++)
	{
		(*otp_ptr).lsc_data[i] = S5K3L9_ReadOtp(i + 0x0017);
	}
	
	if(get_boot_mode()==FACTORY_BOOT)
	{
		return KAL_TRUE;
	}
	
    for(i = 0;i < S5K3L9_PDAF_OTP_SIZE;i++)
	{
		(*otp_ptr).pdaf_data[i] = S5K3L9_ReadOtp(i + 0x0763);
	}

	return KAL_TRUE;
}

/*************************************************************************
* FUNCTION
*	get_3l9_dvt_id
*
* DESCRIPTION
*	get_3l9_dvt_id
*
* PARAMETERS
*	
*
* RETURNS
*	void
*
* GLOBALS AFFECTED
*
*************************************************************************/
int get_3l9_dvt_id(void)
{	
	int i = 0;

	if((g_s5k3l9_otp_struct.wb_g_R == 0x57)&&(g_s5k3l9_otp_struct.wb_g_Gr == 0xA4)&&
		(g_s5k3l9_otp_struct.wb_g_Gb == 0xA3)&&(g_s5k3l9_otp_struct.wb_g_B == 0x50))
	{
		CAM_CALDB("Samsung DVT1 3L9  !!!!\n");	
		return 1;
	}
	//else if((g_s5k3l9_otp_struct.wb_g_R == 0x59)&&(g_s5k3l9_otp_struct.wb_g_Gr == 0xA0)&&
		//(g_s5k3l9_otp_struct.wb_g_Gb == 0xA2)&&(g_s5k3l9_otp_struct.wb_g_B == 0x65))
	else
	{
		CAM_CALDB("Samsung After DVT2 3L9  !!!!\n");	
		return 2;
	}
	//else
	//{
		//CAM_CALDB("Samsung 3L9 Unknown !!!!\n");	
		//return 0;
	//}
	
}

/*************************************************************************
* FUNCTION
*	read_3l9_pdaf_data
*
* DESCRIPTION
*	read_3l9_pdaf_data
*
* PARAMETERS
*	
*
* RETURNS
*	void
*
* GLOBALS AFFECTED
*
*************************************************************************/
bool read_3l9_pdaf_data( kal_uint16 addr, BYTE* data, kal_uint32 size)
{	
	int i = 0;

	if(size < S5K3L9_PDAF_OTP_SIZE)
	{
		CAM_CALDB("[S5K3L9_CAL]wrong pdaf_size:0x%x \n ",size);	
	    return false;
	}

    for(i = 0;i < size;i++)
	{
		if(i < S5K3L9_PDAF_OTP_SIZE)
		{
			data[i] = g_s5k3l9_otp_struct.pdaf_data[i];
		}
		else
		{
			data[i] = 0;
		}
	}
    return true;
}

/*************************************************************************
* FUNCTION
*	read_s5k3l8_static_otp
*
* DESCRIPTION
*	read_s5k3l9_static_otp
*
* PARAMETERS
*	
*
* RETURNS
*	void
*
* GLOBALS AFFECTED
*
*************************************************************************/
void read_s5k3l9_static_otp(void)
{
	memset(&g_s5k3l9_otp_struct, 0, sizeof(struct s5k3l9_otp_struct));
	read_s5k3l9_otp(&g_s5k3l9_otp_struct);
}

/*************************************************************************
* FUNCTION
*	iReadData_3L9
*
* DESCRIPTION
*	iReadData_3L9
*
* PARAMETERS
*	
*
* RETURNS
*	int
*
* GLOBALS AFFECTED
*
*************************************************************************/
 int iReadData_3L9(kal_uint16 ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
	
	if(ui4_length ==1)
    {	
	    *pinputdata = S5K3L9_ReadOtp(ui4_offset);
    }
	else if(ui4_length == S5K3L9_AWBAF_OTP_SIZE)
    {
	    S5K3L9_Read_AWBAF_Otp(ui4_offset, pinputdata, ui4_length);
    }
	else if(ui4_length == S5K3L9_LSC_OTP_SIZE)
    {
		//if(get_boot_mode()==FACTORY_BOOT)
		//{
			//return -EPERM;
		//}
	    S5K3L9_Read_LSC_Otp(ui4_length,pinputdata);
   	}

    //2. read otp

    for(i4RetValue = 0;i4RetValue<ui4_length;i4RetValue++){
    CAM_CALDB( "[[S5K3L9_CAL]]pinputdata[%d]=%x\n", i4RetValue,*(pinputdata+i4RetValue));}
    CAM_CALDB(" [[S5K3L9_CAL]]ui4_length = %d,ui4_offset =%d\n ",ui4_length,ui4_offset);
    CAM_CALDB("[S24EEPORM] iReadData_S5K3L9 done\n" );
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
            CAM_CALDB("[S5K3L9_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S5K3L9_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pWorkingBuff = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL); 
    if(NULL == pWorkingBuff)
    {
        kfree(pBuff);
        CAM_CALDB("[S5K3L9_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB("[S5K3L9_CAL] init Working buffer address 0x%p  command is 0x%8x\n", pWorkingBuff, (u32)a_u4Command);

 
    if(copy_from_user((u8*)pWorkingBuff ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pWorkingBuff);
        CAM_CALDB("[S5K3L9_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[S5K3L9_CAL] Write CMD \n");
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
            CAM_CALDB("[S5K3L9_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif     
            CAM_CALDB("[S5K3L9_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[S5K3L9_CAL] length %d \n", ptempbuf->u4Length);
            CAM_CALDB("[S5K3L9_CAL] Before read Working buffer address 0x%p \n", pWorkingBuff);

            i4RetValue = iReadData_3L9((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			
            CAM_CALDB("[S5K3L9_CAL] After read Working buffer data  0x%4x \n", *pWorkingBuff);


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
      	     CAM_CALDB("[S5K3L9_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S5K3L9_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S5K3L9_CAL] to user  Working buffer address 0x%p \n", pWorkingBuff);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pWorkingBuff , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pWorkingBuff);
            CAM_CALDB("[S5K3L9_CAL] ioctl copy to user failed\n");
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
    CAM_CALDB("[S5K3L9_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_3L9_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_3L9_CALLock);
		CAM_CALDB("[S5K3L9_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_3L9_CALLock);

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_3L9_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_3L9_CALLock);

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
        CAM_CALDB("[S5K3L9_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S5K3L9_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S5K3L9_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S5K3L9_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_3L9");
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
    CAM_CALDB("[S5K3L9_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_3L9_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_3L9_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = S5K3L9OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_3L9_CALLock); // for SMP    
    
    CAM_CALDB("[S5K3L9_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S5K3L9_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S5K3L9_CAL] Attached!! \n");
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

MODULE_DESCRIPTION("CAM_CAL_3L9 driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


