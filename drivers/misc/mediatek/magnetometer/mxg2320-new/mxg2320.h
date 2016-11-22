/*
 *  Mxg2320 Magnetometer device driver
 *
 *  Copyright (C) 2015 joan.na <joan.na@magnachip.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef _MXG2320_H_
#define _MXG2320_H_

#include <linux/ioctl.h>

#define MXG2320_DEV_NAME                "mxg2320"

/*
     7bits     -->         8bits
0x0C 00001100  -->  00011000  0x18
0x0D 00001101  -->  00011010  0x1A
*/
#define MXG2320_I2C_ADDRESS             0x18////0x1A////
#define MXG2320_CMP1_ID                 0x20

#define MXG2320_DELAY_MOD_TRANS         100
#define MXG2320_DELAY_MAX               200
#define MXG2320_DELAY_MIN               10
#define MXG2320_VIRTUAL_DELAY_MAX       200
#define MXG2320_VIRTUAL_DELAY_MIN       10

#define MXG2320_COEF_X_MIN              -40
#define MXG2320_COEF_X_MAX              40
#define MXG2320_COEF_Y_MIN              -40
#define MXG2320_COEF_Y_MAX              40
#define MXG2320_COEF_Z_MIN              -320
#define MXG2320_COEF_Z_MAX              -80


#define MXG2320_BUFSIZE                 0x20
#define MXG2320_DATASIZE                0x100
#define MXG2320_AXIS_DATA_MAX           6
#define MXG2320_MEASUREMENT_DATA_SIZE   9
#define MXG2320_MEASUREMENT_ST1_POS     0
#define MXG2320_MEASUREMENT_TMP_POS     7
#define MXG2320_MEASUREMENT_ST2_POS     8
#define MXG2320_MEASUREMENT_RAW_POS     1

/* MXG2320 Register Address */
#define MXG2320_REG_STR1                0x00
#define MXG2320_REG_XDH                 0x01
#define MXG2320_REG_XDL                 0x02
#define MXG2320_REG_YDH                 0x03
#define MXG2320_REG_YDL                 0x04
#define MXG2320_REG_ZDH                 0x05
#define MXG2320_REG_ZDL                 0x06
#define MXG2320_REG_DMY                 0x07
#define MXG2320_REG_STR2                0x08
#define MXG2320_REG_SCTRL               0x10
#define MXG2320_REG_SWRST               0x11
#define MXG2320_REG_CMPID               0x20
#define MXG2320_REG_CINFO               0x22
#define MXG2320_REG_XCOEF               0x24
#define MXG2320_REG_YCOEF               0x25
#define MXG2320_REG_ZCOEF               0x26

/* MXG2320 Operation Mode */
#define	MXG2320_MOD_POWER_DOWN          0x00
#define MXG2320_MOD_SNG_MEASURE         0x01
#define	MXG2320_MOD_SELF_TEST           0x10
#define	MXG2320_MOD_FUSE_ACCESS         0x1F
#define	MXG2320_SOFT_RESET              0x01


/*  conversion of magnetic data (for MXG2320) to uT units
 *  #define CONVERT_M   (1.0f*0.06f)
 *  conversion of orientation data to degree units
 *  #define CONVERT_O   (1.0f/64.0f)
 */
#define CONVERT_M                       6
#define CONVERT_M_DIV                   1000 // 6/1000 = CONVERT_M
#define CONVERT_O                       1
#define CONVERT_O_DIV                   65536
#define CONVERT_FUSION                  1
#define CONVERT_FUSION_DIV              65536  // 1/64 = CONVERT_O	






typedef union{
        struct {
        	int x;
        	int y;
        	int z;
        	int status;
        	int reserved1[2];
	};
	struct {
        	int azimuth;
        	int pitch;
        	int roll;
        	int reserved2[3];
	};
	struct {
        	int rx;
        	int ry;
        	int rz;
        	int scalar;
        	int reserved3[2];
	};
	struct {
                int ux;
        	int uy;
        	int uz;
        	int ox;
        	int oy;
        	int oz;
	};	
	int data[MXG2320_AXIS_DATA_MAX];
}MXG_SENSOR_DATA;


#endif // _MXG2320_H_
