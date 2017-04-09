/*****************************************************************************
 *
 * Filename:
 * ---------
 *   S-24CS64A.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   Ronnie Lai (MTK01420)
 *
 *============================================================================*/

 #include "mach/mt_boot.h"
 
#ifndef __CAM_CAL_H
#define __CAM_CAL_H

#define CAM_CAL_DEV_MAJOR_NUMBER 253

/* CAM_CAL READ/WRITE ID */
#define S5K3L9OTP_DEVICE_ID							0xA0

#define S5K3L9_AWBAF_OTP_SIZE 14
#define S5K3L9_LSC_OTP_SIZE 1868
#define S5K3L9_PDAF_OTP_SIZE 1404


typedef struct  s5k3l9_otp_struct{
	
	kal_uint8 flag;	//flag 0x0000
	kal_uint32 cali_ver;	//cali_version 0x0001
	kal_uint8 module_id;	//module_id 0x0005
	kal_uint8 lens_id;	//lens_id 0x0006

	kal_uint16 awbaf_info;	//awbaf_info 0x0007
	
	kal_uint8 wb_u_R;	//wb_u_R 0x0009
	kal_uint8 wb_u_Gr;	//wb_u_Gr 0x000A
	kal_uint8 wb_u_Gb;	//wb_u_Gb 0x000B
	kal_uint8 wb_u_B;	//wb_u_B 0x000C
	kal_uint8 wb_g_R;	//wb_g_R 0x000D
	kal_uint8 wb_g_Gr;	//wb_g_Gr 0x000E
	kal_uint8 wb_g_Gb;	//wb_g_Gb 0x000F
	kal_uint8 wb_g_B;	//wb_g_B 0x0010
	
	kal_uint16 af_infinit;	//af_infinit 0x0011
	kal_uint16 af_macro;	//af_macro 0x0013

	kal_uint16 lsc_size;	//lsc_size 0x0015
	kal_uint8 lsc_data[S5K3L9_LSC_OTP_SIZE];	//lsc_data 0x0017
	
	kal_uint8 pdaf_data[S5K3L9_PDAF_OTP_SIZE];	//pdaf_data 0x0763
	
} s5k3l9_otp_struct;

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);

#endif /* __CAM_CAL_H */

