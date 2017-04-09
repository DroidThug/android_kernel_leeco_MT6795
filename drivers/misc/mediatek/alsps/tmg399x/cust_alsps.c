/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 3,
    .polling_mode_ps =0,
    .polling_mode_als =1,
    .polling_mode_gesture =0,
//    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
//    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level  = {10,150,200, 380, 1330, 2800, 4610, 6700, 8800, 9600, 10500, 11500, 11500, 11500, 11500},
    .als_value  = {70, 200, 350, 760, 2200, 4000, 6230, 9000, 11700, 12600,  17500,  20001,  20001, 20001,  20001, 20001},
    .ps_threshold_high = 80,  //0~255
    .ps_threshold_low = 50,   //0~255
//    .ps_threshold = 900,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
//TYD hj,add
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <tmg399x.h>
struct tmg399x_parameters parameters = {
    .als_time = 0xFE, /* 5.6ms */
    .als_gain = AGAIN_64,
    .wait_time = 0xFF, /* 2.78ms */
    .prox_th_min = 0,
    .prox_th_max = 255,
    .persist = PRX_PERSIST(0) | ALS_PERSIST(0),
    .als_prox_cfg1 = 0x60,
    .prox_pulse = PPLEN_4US | PRX_PULSE_CNT(6),
    .prox_gain = PGAIN_4,
    .ldrive = PDRIVE_100MA,
    .als_prox_cfg2 = LEDBOOST_150 | 0x01,
    .prox_offset_ne = 0,
    .prox_offset_sw = 0,
    .als_prox_cfg3 = 0x00,

    .ges_entry_th = 0,
    .ges_exit_th = 255,
    .ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_2,
    .ges_cfg2 = GGAIN_2 | GLDRIVE_100 | GWTIME_3,
    .ges_offset_n = 0,
    .ges_offset_s = 0,
    .ges_pulse = GPLEN_32US | GES_PULSE_CNT(16),
    .ges_offset_w = 0,
    .ges_offset_e = 0,
    .ges_dimension = GBOTH_PAIR,
};
struct tmg399x_parameters *get_tmg_arglist(void)
{
	return &parameters;
}
//e

