/*
 * Device driver for monitoring ambient light intensity in (lux)
 * proximity detection (prox), and Gesture functionality within the
 * AMS-TAOS TMG3992/3.
 *
 * Copyright (c) 2013, AMS-TAOS USA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>

#include <tmg399x.h>
#include <linux/wakelock.h>
#include <mach/mt_gpio.h>
#include <mach/irqs.h>
#include <mach/eint.h>
#include <cust_eint.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/earlysuspend.h>
#include <cust_alsps.h>
#include "gesture.h"
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <alsps.h>

#define A575
#define CCI_PSENSOR_CALIBRATION_SUPPORT

#define Gesture_function 0

#define HDEBUG
#ifdef HDEBUG
#define __MODE__ "TMG"
#define h_debug(fmt,arg...) printk(KERN_ERR"\n[ #h#j# (%s)::%s ]"fmt,__MODE__,__func__,##arg)
#else
#define h_debug(fmt,arg...)
#endif


//TYD hj ,add. for mtk
//extern int hwmsen_get_interrupt_data(int sensor, hwm_sensor_data *data);
static int mapping_als_value(struct tmg399x_chip *chip,u16 als);
//e
static u8 const tmg399x_ids[] = {
	0xAB,
	0xAA,
	0x9E,
	0x9F,
	0x84,
	0x60,
	0x49,
};

static char const *tmg399x_names[] = {
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
	"tmg399x",
};

static u8 const restorable_regs[] = {
	TMG399X_ALS_TIME,
	TMG399X_WAIT_TIME,
	TMG399X_PERSISTENCE,
	TMG399X_CONFIG_1,
	TMG399X_PRX_PULSE,
	TMG399X_GAIN,
	TMG399X_CONFIG_2,
	TMG399X_PRX_OFFSET_NE,
	TMG399X_PRX_OFFSET_SW,
	TMG399X_CONFIG_3,
};

static u8 const prox_gains[] = {
	1,
	2,
	4,
	8
};

static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const prox_pplens[] = {
	4,
	8,
	16,
	32
};

static u8 const led_drives[] = {
	100,
	50,
	25,
	12
};

static u16 const led_boosts[] = {
	100,
	150,
	200,
	300
};

static struct lux_segment segment_default[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};

static struct tmg399x_parameters param_default = {
	.als_time = 0xFE, /* 5.6ms */
	.als_gain = AGAIN_64,
	.wait_time = 0xFF, /* 2.78ms */
	.prox_th_min = 0,
	.prox_th_max = 255,
	.persist = PRX_PERSIST(0) | ALS_PERSIST(0),
	.als_prox_cfg1 = 0x60,
	.prox_pulse = PPLEN_32US | PRX_PULSE_CNT(5),
	.prox_gain = PGAIN_4,
	.ldrive = PDRIVE_100MA,
	.als_prox_cfg2 = LEDBOOST_150 | 0x01,
	.prox_offset_ne = 0,
	.prox_offset_sw = 0,
	.als_prox_cfg3 = 0,

	.ges_entry_th = 0,
	.ges_exit_th = 255,
	.ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_1,
	.ges_cfg2 = GGAIN_4 | GLDRIVE_100 | GWTIME_3,
	.ges_offset_n = 0,
	.ges_offset_s = 0,
	.ges_pulse = GPLEN_32US | GES_PULSE_CNT(5),
	.ges_offset_w = 0,
	.ges_offset_e = 0,
	.ges_dimension = GBOTH_PAIR,
};

/* for gesture and proximity offset calibartion */
static bool docalibration = true;
static bool pstatechanged = false;
static bool ges_start_flag = false;
static uint8_t nr_prox_after_ges = 0;
static u8 caloffsetstate = START_CALOFF;
static u8 caloffsetdir = DIR_NONE;
static uint8_t caloffsetspeed = CAL_FAST;
static bool firstprox = false;
static u8 callowtarget = 15;//8
static u8 calhightarget = 15;//8
static u8 last_mode = 0;
extern void process_rgbc_prox_ges_raw_data(struct tmg399x_chip *chip, u8 type, u8* data, u8 datalen);
extern void init_params_rgbc_prox_ges(void);
extern void set_visible_data_mode(struct tmg399x_chip *chip);
void tmg399x_rgbc_poll_handle(unsigned long data);

static struct tmg399x_chip *gchip;

#ifdef CCI_PSENSOR_CALIBRATION_SUPPORT
unsigned int prox_thresh_high;
unsigned int prox_thresh_low;
unsigned int prox_raw_data;
unsigned int ges_ioctl_value;
unsigned int cci_ps_cali_value = 130;
unsigned int cci_als_cali_value = 500;
unsigned int als_cali_lux = 500;
unsigned int gesture_flag = 0;
unsigned int prox_flag = 0;

struct tmg399x_threshold {
    unsigned  int result_ps_cali_value;
    unsigned  int result_als_cali_value;
};
#endif

//TYD hj,add
enum {
	USER_ANDROID=(1 << 0),
	USER_LINUX=(1 << 1),
};
static struct gesfun{
//	u8 tp_mode;
	u8 ges_user;
};
static struct gesfun gfdata;
static struct wake_lock  pslock;
static struct platform_device *pltdev;
//


static int  tmg399x_local_init(void);
static int tmg399x_local_remove(struct platform_device *pdev);
static struct alsps_init_info tmg399x_init_info = {
		.name = "tmg3993",
		.init = tmg399x_local_init,
		.uninit = tmg399x_local_remove,
};


static int tmg399x_i2c_read(struct tmg399x_chip *chip, u8 reg, u8 *val)
{
	int ret;

	s32 read;
	struct i2c_client *client = chip->client;
	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte(client, reg);
		if (ret < 0) {
			printk(KERN_ERR"%s: failed 2x to write register %x\n",
				__func__, reg);
		return ret;
		}
	}

	read = i2c_smbus_read_byte(client);
	if (read < 0) {
		mdelay(3);
		read = i2c_smbus_read_byte(client);
		if (read < 0) {
			printk(KERN_ERR"%s: failed read from register %x\n",
				__func__, reg);
		}
		return ret;
	}

	*val = (u8)read;
	return 0;
}

static int tmg399x_i2c_write(struct tmg399x_chip *chip, u8 reg, u8 val)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			printk(KERN_ERR"%s: failed to write register %x err= %d\n",
				__func__, reg, ret);
		}
	}
	return ret;
}

#if 1
static int tmg399x_i2c_modify(struct tmg399x_chip *chip, u8 reg, u8 mask, u8 val)
{
	int ret;
	u8 temp;

	ret = tmg399x_i2c_read(chip, reg, &temp);
	temp &= ~mask;
	temp |= val;
	ret |= tmg399x_i2c_write(chip, reg, temp);

	return ret;
}
#endif

static int tmg399x_i2c_ram_blk_read(struct tmg399x_chip *chip,
		u8 reg, u8 *val, int size)
{
	int ret;
	struct i2c_client *client = chip->client;

	reg += I2C_ADDR_OFFSET;
	ret =  i2c_smbus_read_i2c_block_data(client,
			reg, size, val);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

static int tmg399x_flush_regs(struct tmg399x_chip *chip)
{
	unsigned i;
	int ret;
	u8 reg;

	h_debug("%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		ret = tmg399x_i2c_write(chip, reg, chip->shadow[reg]);
		if (ret < 0) {
			printk(KERN_ERR "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return ret;
}

static int tmg399x_irq_clr(struct tmg399x_chip *chip, u8 int2clr)
{
	int ret, ret2;

	ret = i2c_smbus_write_byte(chip->client, int2clr);
	if (ret < 0) {
		mdelay(3);
		ret2 = i2c_smbus_write_byte(chip->client, int2clr);
		if (ret2 < 0) {
			printk(KERN_ERR "%s: failed 2x, int to clr %02x\n",
					__func__, int2clr);
		}
		return ret2;
	}

	return ret;
}

static int tmg399x_update_enable_reg(struct tmg399x_chip *chip)
{
	int ret;

	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL,
			chip->shadow[TMG399X_CONTROL]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_4,
			chip->shadow[TMG399X_GES_CFG_4]);

	return ret;
}

static int tmg399x_set_als_gain(struct tmg399x_chip *chip, int gain)
{
	int ret;
	u8 ctrl_reg  = chip->shadow[TMG399X_GAIN] & ~TMG399X_ALS_GAIN_MASK;

	switch (gain) {
	case 1:
		ctrl_reg |= AGAIN_1;
		break;
	case 4:
		ctrl_reg |= AGAIN_4;
		break;
	case 16:
		ctrl_reg |= AGAIN_16;
		break;
	case 64:
		ctrl_reg |= AGAIN_64;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.als_gain = ctrl_reg & TMG399X_ALS_GAIN_MASK;
	}
	return ret;
}

static void tmg399x_calc_cpl(struct tmg399x_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime = chip->shadow[TMG399X_ALS_TIME];

	cpl = 256 - chip->shadow[TMG399X_ALS_TIME];
	cpl *= TMG399X_ATIME_PER_100;
	cpl /= 100;
	cpl *= als_gains[chip->params.als_gain];

	sat = min_t(u32, MAX_ALS_VALUE, (u32)(256 - atime) << 10);
	sat = sat * 8 / 10;
	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tmg399x_get_lux(struct tmg399x_chip *chip)
{
	u32 rp1, gp1, bp1, cp1;
	u32 lux = 0;
	u32 cct;
	u32 sat;
	u32 sf;

	/* use time in ms get scaling factor */
	tmg399x_calc_cpl(chip);
	sat = chip->als_inf.saturation;
	#if defined(A77)
	tmg399x_i2c_write(chip, TMG399X_ALS_TIME,0xFB);
	#endif
   if (!chip->als_gain_auto) {
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE) {
			h_debug("%s: darkness\n", __func__);
			lux = 0;
			chip->als_inf.lux =lux ;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			h_debug("%s: saturation, keep lux & cct\n", __func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
	} else {
		u8 gain = als_gains[chip->params.als_gain];
		int ret = -EIO;

		if (gain == 16 && chip->als_inf.clear_raw >= sat) {
			ret = tmg399x_set_als_gain(chip, 1);
		} else if (gain == 16 &&
			chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL) {
			ret = tmg399x_set_als_gain(chip, 64);
		} else if ((gain == 64 &&
			chip->als_inf.clear_raw >= (sat - GAIN_SWITCH_LEVEL)) ||
			(gain == 1 && chip->als_inf.clear_raw < GAIN_SWITCH_LEVEL)) {
			ret = tmg399x_set_als_gain(chip, 16);
		}
		if (!ret) {
			printk(KERN_ERR"%s: gain adjusted, skip\n",	__func__);
			tmg399x_calc_cpl(chip);
			ret = -EAGAIN;
			lux = chip->als_inf.lux;
			goto exit;
		}
    #if defined(A596)
        if (chip->als_inf.clear_raw >= sat) {
			h_debug("%s: saturation, keep lux\n",
					__func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
    #else
		if (chip->als_inf.clear_raw <= MIN_ALS_VALUE){
			h_debug("%s: darkness\n", __func__);
			lux = 0;
			chip->als_inf.lux =lux ;
			goto exit;
		} else if (chip->als_inf.clear_raw >= sat) {
			h_debug("%s: saturation, keep lux\n",
					__func__);
			lux = chip->als_inf.lux;
			goto exit;
		}
    #endif
	}

	/* remove ir from counts*/
	rp1 = chip->als_inf.red_raw - chip->als_inf.ir;
	gp1 = chip->als_inf.green_raw - chip->als_inf.ir;
	bp1 = chip->als_inf.blue_raw - chip->als_inf.ir;
	cp1 = chip->als_inf.clear_raw - chip->als_inf.ir;

	if (!chip->als_inf.cpl) {
		h_debug("%s: zero cpl. Setting to 1\n",
				__func__);
		chip->als_inf.cpl = 1;
	}

	if (chip->als_inf.red_raw > chip->als_inf.ir)
		lux += segment_default[chip->device_index].r_coef * rp1;
	else
		printk(KERN_ERR "%s: lux rp1 = %d\n",
			__func__,
			(segment_default[chip->device_index].r_coef * rp1));

	if (chip->als_inf.green_raw > chip->als_inf.ir)
		lux += segment_default[chip->device_index].g_coef * gp1;
	else
		printk(KERN_ERR"%s: lux gp1 = %d\n",
			__func__,
			(segment_default[chip->device_index].g_coef * rp1));

	if (chip->als_inf.blue_raw > chip->als_inf.ir)
			lux += segment_default[chip->device_index].b_coef * bp1;
	else
		printk(KERN_ERR "%s: lux bp1 = %d\n",
			__func__,
			(segment_default[chip->device_index].b_coef * rp1));

	sf = chip->als_inf.cpl;

	if (sf > 131072)
		goto error;

	lux /= sf;
	lux *= segment_default[chip->device_index].d_factor;
	lux += 500;
	lux /= 1000;

	lux *= als_cali_lux;
	lux /= cci_als_cali_value;

	chip->als_inf.lux = (u16) lux;

	cct = ((segment_default[chip->device_index].ct_coef * bp1) / rp1) +
		segment_default[chip->device_index].ct_offset;

	chip->als_inf.cct = (u16) cct;

	//h_debug( "chip->als_inf.lux =%d,chip->als_inf.cct=%d\n",chip->als_inf.lux ,chip->als_inf.cct);
exit:
	return 0;

error:
	printk(KERN_ERR "ERROR Scale factor = %d", sf);

	return 1;
}

static int tmg399x_ges_init(struct tmg399x_chip *chip, int on)
{
	int ret;

	if (on) {
		if (chip->pdata) {
			chip->params.ges_entry_th = chip->pdata->parameters.ges_entry_th;
			chip->params.ges_exit_th = chip->pdata->parameters.ges_exit_th;
			chip->params.ges_cfg1 = chip->pdata->parameters.ges_cfg1;
			chip->params.ges_cfg2 = chip->pdata->parameters.ges_cfg2;
			chip->params.ges_offset_n = chip->pdata->parameters.ges_offset_n;
			chip->params.ges_offset_s = chip->pdata->parameters.ges_offset_s;
			chip->params.ges_pulse = chip->pdata->parameters.ges_pulse;
			chip->params.ges_offset_w = chip->pdata->parameters.ges_offset_w;
			chip->params.ges_offset_e = chip->pdata->parameters.ges_offset_e;
			chip->params.ges_dimension = chip->pdata->parameters.ges_dimension;
		} else {
			chip->params.ges_entry_th = param_default.ges_entry_th;
			chip->params.ges_exit_th = param_default.ges_exit_th;
			chip->params.ges_cfg1 = param_default.ges_cfg1;
			chip->params.ges_cfg2 = param_default.ges_cfg2;
			chip->params.ges_offset_n = param_default.ges_offset_n;
			chip->params.ges_offset_s = param_default.ges_offset_s;
			chip->params.ges_pulse = param_default.ges_pulse;
			chip->params.ges_offset_w = param_default.ges_offset_w;
			chip->params.ges_offset_e = param_default.ges_offset_e;
			chip->params.ges_dimension = param_default.ges_dimension;
		}
	}

	/* Initial gesture registers */
	chip->shadow[TMG399X_GES_ENTH]  = chip->params.ges_entry_th;
	chip->shadow[TMG399X_GES_EXTH]  = chip->params.ges_exit_th;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	chip->shadow[TMG399X_GES_CFG_2] = chip->params.ges_cfg2;
	chip->shadow[TMG399X_GES_OFFSET_N] = chip->params.ges_offset_n;
	chip->shadow[TMG399X_GES_OFFSET_S] = chip->params.ges_offset_s;
	chip->shadow[TMG399X_GES_PULSE] = chip->params.ges_pulse;
	chip->shadow[TMG399X_GES_OFFSET_W] = chip->params.ges_offset_w;
	chip->shadow[TMG399X_GES_OFFSET_E] = chip->params.ges_offset_e;
	chip->shadow[TMG399X_GES_CFG_3] = chip->params.ges_dimension;

	ret = tmg399x_i2c_write(chip, TMG399X_GES_ENTH,
			chip->shadow[TMG399X_GES_ENTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_EXTH,
			chip->shadow[TMG399X_GES_EXTH]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_1,
			chip->shadow[TMG399X_GES_CFG_1]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_2,
			chip->shadow[TMG399X_GES_CFG_2]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N,
			chip->shadow[TMG399X_GES_OFFSET_N]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S,
			chip->shadow[TMG399X_GES_OFFSET_S]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_PULSE,
			chip->shadow[TMG399X_GES_PULSE]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W,
			chip->shadow[TMG399X_GES_OFFSET_W]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E,
			chip->shadow[TMG399X_GES_OFFSET_E]);
	ret |= tmg399x_i2c_write(chip, TMG399X_GES_CFG_3,
			chip->shadow[TMG399X_GES_CFG_3]);

	return ret;
}

static int tmg399x_ges_enable(struct tmg399x_chip *chip, int on)
{
	int ret = 0;
	u8 init_data = 0;    //madan modify

	init_data = TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ | TMG399X_EN_PWR_ON;    //madan modify
	init_data &= chip->shadow[TMG399X_CONTROL];                           //madan modify
	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL, init_data);       //madan modify
	h_debug( "%s: on = %d\n", __func__, on);
	if (on) {
		/* initialize */
		docalibration = true;
		pstatechanged = 0;
		gesture_flag = 1;
		ret |= tmg399x_ges_init(chip, 1);
		if (ret < 0)
			return ret;

		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON |
			TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ | TMG399X_EN_GES);
                chip->shadow[TMG399X_GES_CFG_4] |= TMG399X_GES_EN_IRQ;
		ret |= tmg399x_update_enable_reg(chip);

		mdelay(3);
		set_visible_data_mode(chip);
		if(chip->als_enabled )
		{
			chip->shadow[TMG399X_ALS_TIME] = 0xEE;//10ms
			tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
		}
	} else {
		gesture_flag = 0;
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_GES;
		chip->shadow[TMG399X_GES_CFG_4] &= ~ TMG399X_GES_EN_IRQ;
		if (!chip->prx_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~(TMG399X_EN_PRX | TMG399X_EN_PRX_IRQ);
		if (!chip->prx_enabled && !chip->als_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
	
	chip->shadow[TMG399X_ALS_TIME] = 0xFE;//5.6ms
	tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);

	if(chip->als_enabled )
	{
		chip->shadow[TMG399X_ALS_TIME] = 0xEE;//50ms
		tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
	}
	
	}
	if (!ret)
		chip->ges_enabled = on;

	return ret;
}

static int tmg399x_prox_enable(struct tmg399x_chip *chip, int on)
{
	int ret;
	u8 init_data = 0;    //madan modify
	struct alsps_hw *aphw=get_cust_alsps_hw();

#ifdef CCI_PSENSOR_CALIBRATION_SUPPORT
    prox_thresh_high = cci_ps_cali_value;
	prox_thresh_low = cci_ps_cali_value - 30;
#endif
	/* Initial proximity threshold */
	chip->shadow[TMG399X_PRX_MINTHRESHLO] = prox_thresh_low;
	chip->shadow[TMG399X_PRX_MAXTHRESHHI] = prox_thresh_high;
	tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
			chip->shadow[TMG399X_PRX_MINTHRESHLO]);
	tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
			chip->shadow[TMG399X_PRX_MAXTHRESHHI]);

	init_data = TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ | TMG399X_EN_PWR_ON;    //madan modify
	init_data &= chip->shadow[TMG399X_CONTROL];                           //madan modify
	ret = tmg399x_i2c_write(chip, TMG399X_CONTROL, init_data);       //madan modify
	h_debug( "%s: on = %d\n", __func__, on);
	if (on) {
//		wake_lock(&pslock);
		firstprox = true;
		pstatechanged = 0;
		prox_flag = 1;
		chip->shadow[TMG399X_CONTROL] |= (TMG399X_EN_PWR_ON |
			TMG399X_EN_PRX );
//TYD hj,add.interrupt
		if(aphw->polling_mode_ps == 0)
			chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_PRX_IRQ;
//e
		docalibration = true;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		chip->prx_inf.raw = 0;
//		mdelay(3);
		set_visible_data_mode(chip);
	} else {
//		wake_unlock(&pslock);
		prox_flag = 0;
		if (!chip->ges_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~(TMG399X_EN_PRX_IRQ | TMG399X_EN_PRX);
		if (!chip->ges_enabled && !chip->als_enabled)
                        chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;

		chip->prx_inf.raw = 0;

		if(chip->als_enabled )
		{
			chip->shadow[TMG399X_ALS_TIME] = 0xEE;//50ms
			tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
		}		
	}
	if (!ret)
		chip->prx_enabled = on;

	return ret;
}

static int tmg399x_als_enable(struct tmg399x_chip *chip, int on)
{
	int ret;
	struct alsps_hw *aphw=get_cust_alsps_hw();

	dev_info(&chip->client->dev, "%s: on = %d\n", __func__, on);
	if (on) {
		/* set up timer for RGBC polling */
		chip->rgbc_poll_flag = false;
		chip->rgbc_timer.expires = jiffies + HZ/10;
		mod_timer(&chip->rgbc_timer,&chip->rgbc_timer.expires);
		/* use auto gain setting */
		chip->als_gain_auto = true;

		chip->shadow[TMG399X_CONTROL] |=
				(TMG399X_EN_PWR_ON | TMG399X_EN_ALS );
//TYD hj,add.interrupt
		if(aphw->polling_mode_als == 0){
			chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_ALS_IRQ;
		}
//e
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		del_timer(&chip->rgbc_timer);

		chip->shadow[TMG399X_CONTROL] &=
			~(TMG399X_EN_ALS | TMG399X_EN_ALS_IRQ);

		if (!chip->ges_enabled && !chip->prx_enabled)
			chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_PWR_ON;
		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;

		chip->als_inf.lux = 0;
		chip->als_inf.cct = 0;
	}
	if(!chip->ges_enabled )
	{
		chip->shadow[TMG399X_ALS_TIME] = 0xEE;//50ms
		tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
	}
	if (!ret)
		chip->als_enabled = on;

	return ret;
}

static int tmg399x_wait_enable(struct tmg399x_chip *chip, int on)
{
	int ret;

	h_debug( "%s: on = %d\n", __func__, on);
	if (on) {
		chip->shadow[TMG399X_CONTROL] |= TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
		mdelay(3);
	} else {
		chip->shadow[TMG399X_CONTROL] &= ~TMG399X_EN_WAIT;

		ret = tmg399x_update_enable_reg(chip);
		if (ret < 0)
			return ret;
	}
	if (!ret)
	        chip->wait_enabled = on;

	return ret;
}

static ssize_t tmg399x_ges_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->ges_enabled);
}

static ssize_t tmg399x_ges_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_ges_enable(chip, 1);
	else
		tmg399x_ges_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_enabled);
}

static ssize_t tmg399x_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_prox_enable(chip, 1);
	else
		tmg399x_prox_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmg399x_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_als_enable(chip, 1);
	else
		tmg399x_als_enable(chip, 0);

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_wait_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->wait_enabled);
}

static ssize_t tmg399x_wait_enable_store(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		tmg399x_wait_enable(chip, 1);
	else
		tmg399x_wait_enable(chip, 0);

		mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.als_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long itime;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &itime);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (itime > 712 || itime < 3) {
		printk(KERN_ERR"als integration time range [3,712]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	itime *= 100;
	itime /= TMG399X_ATIME_PER_100;
	itime = (256 - itime);
	chip->shadow[TMG399X_ALS_TIME] = (u8)itime;
	chip->params.als_time = (u8)itime;
	ret = tmg399x_i2c_write(chip, TMG399X_ALS_TIME,
		chip->shadow[TMG399X_ALS_TIME]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_wait_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int t;
	t = 256 - chip->params.wait_time;
	t *= TMG399X_ATIME_PER_100;
	t /= 100;
	if (chip->params.als_prox_cfg1 & WLONG)
		t *= 12;
	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmg399x_wait_time_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long time;
	int ret;
	u8 cfg1;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &time);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (time > 8540 || time < 3) {
		printk(KERN_ERR	"wait time range [3,8540]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	cfg1 = chip->shadow[TMG399X_CONFIG_1] & ~0x02;
	if (time > 712) {
		cfg1 |= WLONG;
		time /= 12;
	}

	time *= 100;
	time /= TMG399X_ATIME_PER_100;
	time = (256 - time);
	chip->shadow[TMG399X_WAIT_TIME] = (u8)time;
	chip->params.wait_time = (u8)time;
	chip->shadow[TMG399X_CONFIG_1] = cfg1;
	chip->params.als_prox_cfg1 = cfg1;
	ret = tmg399x_i2c_write(chip, TMG399X_WAIT_TIME,
		chip->shadow[TMG399X_WAIT_TIME]);
	ret |= tmg399x_i2c_write(chip, TMG399X_CONFIG_1,
		chip->shadow[TMG399X_CONFIG_1]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0xF0) >> 4);
}

static ssize_t tmg399x_prox_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (persist > 15) {
		dev_err(&chip->client->dev,
			"prox persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_PERSISTENCE] &= 0x0F;
	chip->shadow[TMG399X_PERSISTENCE] |= (((u8)persist << 4) & 0xF0);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	ret = tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.persist & 0x0F));
}

static ssize_t tmg399x_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long persist;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &persist);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (persist > 15) {
		printk(KERN_ERR"als persistence range [0,15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_PERSISTENCE] &= 0xF0;
	chip->shadow[TMG399X_PERSISTENCE] |= ((u8)persist & 0x0F);
	chip->params.persist = chip->shadow[TMG399X_PERSISTENCE];
	ret = tmg399x_i2c_write(chip, TMG399X_PERSISTENCE,
		chip->shadow[TMG399X_PERSISTENCE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_pulse_len_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.prox_pulse & 0xC0) >> 6;
	return snprintf(buf, PAGE_SIZE, "%duS\n", prox_pplens[i]);
}

static ssize_t tmg399x_prox_pulse_len_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long length;
	int ret;
	u8 ppulse;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &length);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (length != 4 && length != 8 &&
		length != 16 &&	length != 32) {
		printk(KERN_ERR"pulse length set: {4, 8, 16, 32}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ppulse = chip->shadow[TMG399X_PRX_PULSE] & 0x3F;
	switch (length){
	case 4:
		ppulse |= PPLEN_4US;
		break;
	case 8:
		ppulse |= PPLEN_8US;
		break;
	case 16:
		ppulse |= PPLEN_16US;
		break;
	case 32:
		ppulse |= PPLEN_32US;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_PRX_PULSE] = ppulse;
	chip->params.prox_pulse = ppulse;
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_pulse_cnt_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.prox_pulse & 0x3F) + 1);
}

static ssize_t tmg399x_prox_pulse_cnt_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long count;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &count);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (count > 64 || count == 0) {
		printk(KERN_ERR	"prox pulse count range [1,64]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	count -= 1;

	chip->shadow[TMG399X_PRX_PULSE] &= 0xC0;
	chip->shadow[TMG399X_PRX_PULSE] |= ((u8)count & 0x3F);
	chip->params.prox_pulse = chip->shadow[TMG399X_PRX_PULSE];
	ret = tmg399x_i2c_write(chip, TMG399X_PRX_PULSE,
		chip->shadow[TMG399X_PRX_PULSE]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = chip->params.prox_gain >> 2;
	return snprintf(buf, PAGE_SIZE, "%d\n", prox_gains[i]);
}

static ssize_t tmg399x_prox_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (gain != 1 && gain != 2 && gain != 4 && gain != 8) {
		printk(KERN_ERR"prox gain set: {1, 2, 4, 8, 16}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_PRX_GAIN_MASK;
	switch (gain){
	case 1:
		ctrl_reg |= PGAIN_1;
		break;
	case 2:
		ctrl_reg |= PGAIN_2;
		break;
	case 4:
		ctrl_reg |= PGAIN_4;
		break;
	case 8:
		ctrl_reg |= PGAIN_8;
		break;
	default:
		break;
	}

	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, ctrl_reg);
	if (!ret) {
		chip->shadow[TMG399X_GAIN] = ctrl_reg;
		chip->params.prox_gain = ctrl_reg & TMG399X_PRX_GAIN_MASK;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_drive_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%dmA\n",
			led_drives[chip->params.ldrive]);
}

static ssize_t tmg399x_led_drive_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long ldrive;
	int ret;
	u8 ctrl_reg;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &ldrive);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (ldrive != 100 && ldrive != 50 &&
		ldrive != 25 && ldrive != 12) {
		printk(KERN_ERR	"led drive set: {100, 50, 25, 12}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	ctrl_reg = chip->shadow[TMG399X_GAIN] & ~TMG399X_LDRIVE_MASK;
	switch (ldrive){
	case 100:
		ctrl_reg |= PDRIVE_100MA;
		chip->params.ldrive = 0;
		break;
	case 50:
		ctrl_reg |= PDRIVE_50MA;
		chip->params.ldrive = 1;
		break;
	case 25:
		ctrl_reg |= PDRIVE_25MA;
		chip->params.ldrive = 2;
		break;
	case 12:
		ctrl_reg |= PDRIVE_12MA;
		chip->params.ldrive = 3;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_GAIN] = ctrl_reg;
	ret = tmg399x_i2c_write(chip, TMG399X_GAIN, chip->shadow[TMG399X_GAIN]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[chip->params.als_gain],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long gain;
	int i = 0;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &gain);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (gain != 0 && gain != 1 && gain != 4 &&
		gain != 16 && gain != 64) {
		printk(KERN_ERR	"als gain set: {0(auto), 1, 4, 16, 64}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (gain) {
		chip->als_gain_auto = false;
		ret = tmg399x_set_als_gain(chip, als_gains[i]);
		if (!ret)
			tmg399x_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_led_boost_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i = (chip->params.als_prox_cfg2 & 0x30) >> 4;
	return snprintf(buf, PAGE_SIZE, "%d percents\n", led_boosts[i]);
}

static ssize_t tmg399x_led_boost_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long lboost;
	int ret;
	u8 cfg2;

	mutex_lock(&chip->lock);
	ret = kstrtoul(buf, 10, &lboost);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (lboost != 100 && lboost != 150 &&
		lboost != 200 && lboost != 300) {
		printk(KERN_ERR"led boost set: {100, 150, 200, 300}\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	cfg2 = chip->shadow[TMG399X_CONFIG_2] & ~0x30;
	switch (lboost){
	case 100:
		cfg2 |= LEDBOOST_100;
		break;
	case 150:
		cfg2 |= LEDBOOST_150;
		break;
	case 200:
		cfg2 |= LEDBOOST_200;
		break;
	case 300:
		cfg2 |= LEDBOOST_300;
		break;
	default:
		break;
	}
	chip->shadow[TMG399X_CONFIG_2] = cfg2;
	chip->params.als_prox_cfg2 = cfg2;
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_sat_irq_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(chip->params.als_prox_cfg2 & 0x80) >> 7);
}

static ssize_t tmg399x_sat_irq_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool psien;
	int ret;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &psien)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	chip->shadow[TMG399X_CONFIG_2] &= 0x7F;
	if (psien)
		chip->shadow[TMG399X_CONFIG_2] |= PSIEN;
	chip->params.als_prox_cfg2 = chip->shadow[TMG399X_CONFIG_2];
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_2,
		chip->shadow[TMG399X_CONFIG_2]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_offset_ne_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_ne);
}

static ssize_t tmg399x_prox_offset_ne_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_ne;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_ne);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_ne > 127 || offset_ne < -127) {
		printk(KERN_ERR "prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_ne < 0)
		offset = 128 - offset_ne;
	else
		offset = offset_ne;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
	if (!ret) {
		chip->params.prox_offset_ne = (s8)offset_ne;
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_offset_sw_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			chip->params.prox_offset_sw);
}

static ssize_t tmg399x_prox_offset_sw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	long offset_sw;
	int ret;
	u8 offset = 0;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &offset_sw);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (offset_sw > 127 || offset_sw < -127) {
		printk(KERN_ERR"prox offset range [-127, 127]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (offset_sw < 0)
		offset = 128 - offset_sw;
	else
		offset = offset_sw;

	ret = tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
	if (!ret) {
		chip->params.prox_offset_sw = (s8)offset_sw;
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
	}
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_prox_mask_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%.2x\n",
			chip->params.als_prox_cfg3 & 0x0F);
}

static ssize_t tmg399x_prox_mask_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	unsigned long prx_mask;
	int ret;

	mutex_lock(&chip->lock);
	ret = kstrtol(buf, 10, &prx_mask);
	if (ret) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (prx_mask > 15) {
		printk(KERN_ERR "prox mask range [0, 15]\n");
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if ((prx_mask == 5) || (prx_mask == 7) ||
		(prx_mask == 10) || (prx_mask == 11) ||
		(prx_mask == 13) || (prx_mask == 14))
		prx_mask |= PCMP;

	chip->shadow[TMG399X_CONFIG_3] &= 0xD0;
	chip->shadow[TMG399X_CONFIG_3] |= (u8)prx_mask;
	chip->params.als_prox_cfg3 = chip->shadow[TMG399X_CONFIG_3];
	ret = tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
		chip->shadow[TMG399X_CONFIG_3]);
	mutex_unlock(&chip->lock);
	return ret ? ret : size;
}

static ssize_t tmg399x_device_prx_raw(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmg399x_device_prx_detected(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmg399x_device_prx_cali_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", cci_ps_cali_value);
}

static ssize_t tmg399x_ges_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "n:%d s:%d w:%d e:%d\n",
			chip->params.ges_offset_n, chip->params.ges_offset_s,
			chip->params.ges_offset_w, chip->params.ges_offset_e);
}

static ssize_t tmg399x_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct lux_segment *s = chip->segment;
	int i, k;

	for (i = k = 0; i < chip->segment_num; i++)
		k += snprintf(buf + k, PAGE_SIZE - k,
				"%d:%d,%d,%d,%d,%d,%d\n", i,
				s[i].d_factor,
				s[i].r_coef,
				s[i].g_coef,
				s[i].b_coef,
				s[i].ct_coef,
				s[i].ct_offset
				);
	return k;
}

static ssize_t tmg399x_lux_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	int i;
	u32 d_factor, r_coef, g_coef, b_coef, ct_coef, ct_offset;

	mutex_lock(&chip->lock);
	if (7 != sscanf(buf, "%10d:%10d,%10d,%10d,%10d,%10d,%10d",
		&i, &d_factor, &r_coef, &g_coef, &b_coef, &ct_coef, &ct_offset)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	if (i >= chip->segment_num) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	chip->segment[i].d_factor = d_factor;
	chip->segment[i].r_coef = r_coef;
	chip->segment[i].g_coef = g_coef;
	chip->segment[i].b_coef = b_coef;
	chip->segment[i].ct_coef = ct_coef;
	chip->segment[i].ct_offset = ct_offset;
	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_auto_gain_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
				chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmg399x_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	mutex_unlock(&chip->lock);
	return size;
}

static ssize_t tmg399x_device_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.lux);
}

static ssize_t tmg399x_als_red_show(struct device *dev,
	struct device_attribute *attr, char *buf)
		{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.red_raw);
}

static ssize_t tmg399x_als_green_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.green_raw);
}

static ssize_t tmg399x_als_blue_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.blue_raw);
}

static ssize_t tmg399x_als_clear_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.clear_raw);
}

static ssize_t tmg399x_als_cct_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cct);
}

static ssize_t tmg399x_als_cali_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", cci_als_cali_value);
}

static ssize_t show_reg_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#if 1
	int i =0x0,res;
	u8 reg_value,_buff[1024 - 1]={0};
	struct tmg399x_chip *chip = dev_get_drvdata(dev);

//	while(i <= 0x2f){
		res = tmg399x_i2c_read(chip,i,&reg_value);
		if(res < 0)
			reg_value = 0;
		sprintf(_buff,"%s %x:%2x",_buff,i | 0x80,reg_value);
		if(i%0xF == 0)
			strcat(_buff,"\n");
		i++;
//	}
/*	i=0xfc;
	while(i <= 0xff){
		res = tmg399x_i2c_read(chip,i-0x80,&reg_value);
		if(res < 0)
			reg_value = 0;
		sprintf(_buff,"%s %x:%2x",_buff,i | 0x80,reg_value);
		if(i%0xF == 0)
			strcat(_buff,"\n");
		i++;
	}
*/
	return sprintf(buf,"%s\n",_buff);
#endif
}
static struct device_attribute prox_attrs[] = {
	__ATTR(reg_dump, 0666, show_reg_value,NULL),
	__ATTR(prx_power_state, 0666, tmg399x_prox_enable_show,
			tmg399x_prox_enable_store),
	__ATTR(ges_power_state, 0666, tmg399x_ges_enable_show,
			tmg399x_ges_enable_store),
	__ATTR(prx_persist, 0666, tmg399x_prox_persist_show,
			tmg399x_prox_persist_store),
	__ATTR(prx_pulse_length, 0666, tmg399x_prox_pulse_len_show,
			tmg399x_prox_pulse_len_store),
	__ATTR(prx_pulse_count, 0666, tmg399x_prox_pulse_cnt_show,
			tmg399x_prox_pulse_cnt_store),
	__ATTR(prx_gain, 0666, tmg399x_prox_gain_show,
			tmg399x_prox_gain_store),
	__ATTR(led_drive, 0666, tmg399x_led_drive_show,
			tmg399x_led_drive_store),
	__ATTR(led_boost, 0666, tmg399x_led_boost_show,
			tmg399x_led_boost_store),
	__ATTR(prx_sat_irq_en, 0666, tmg399x_sat_irq_en_show,
			tmg399x_sat_irq_en_store),
	__ATTR(prx_offset_ne, 0666, tmg399x_prox_offset_ne_show,
			tmg399x_prox_offset_ne_store),
	__ATTR(prx_offset_sw, 0666, tmg399x_prox_offset_sw_show,
			tmg399x_prox_offset_sw_store),
	__ATTR(prx_mask, 0666, tmg399x_prox_mask_show,
			tmg399x_prox_mask_store),
	__ATTR(prx_raw, 0666, tmg399x_device_prx_raw, NULL),
	__ATTR(prx_detect, 0666, tmg399x_device_prx_detected, NULL),
	__ATTR(prx_cali_value, 0666, tmg399x_device_prx_cali_value, NULL),
	__ATTR(ges_offset, 0666, tmg399x_ges_offset_show, NULL),	
};

static struct device_attribute als_attrs[] = {
	__ATTR(als_power_state, 0666, tmg399x_als_enable_show,
			tmg399x_als_enable_store),
	__ATTR(wait_time_en, 0666, tmg399x_wait_enable_show,
			tmg399x_wait_enable_store),
	__ATTR(als_Itime, 0666, tmg399x_als_itime_show,
		tmg399x_als_itime_store),
	__ATTR(wait_time, 0666, tmg399x_wait_time_show,
		tmg399x_wait_time_store),
	__ATTR(als_persist, 0666, tmg399x_als_persist_show,
			tmg399x_als_persist_store),
	__ATTR(als_gain, 0666, tmg399x_als_gain_show,
			tmg399x_als_gain_store),
	__ATTR(lux_table, 0000, tmg399x_lux_table_show,
			tmg399x_lux_table_store),
	__ATTR(als_auto_gain, 0666, tmg399x_auto_gain_enable_show,
			tmg399x_auto_gain_enable_store),
	__ATTR(als_lux, 0666, tmg399x_device_als_lux, NULL),
	__ATTR(als_red, 0666, tmg399x_als_red_show, NULL),
	__ATTR(als_green, 0666, tmg399x_als_green_show, NULL),
	__ATTR(als_blue, 0666, tmg399x_als_blue_show, NULL),
	__ATTR(als_clear, 0666, tmg399x_als_clear_show, NULL),
	__ATTR(als_cct, 0666, tmg399x_als_cct_show, NULL),
	__ATTR(als_cali_value, 0666, tmg399x_als_cali_value_show, NULL),
};

void tmg399x_report_prox(struct tmg399x_chip *chip, u8 detected)
{
//TYD hj ,add for mtk hwmsensor, interrupt mode
	struct alsps_hw *aphw=get_cust_alsps_hw();
	hwm_sensor_data hsd;
//e
	if (chip->p_idev) {
		chip->prx_inf.detected = detected;

		h_debug("JO--->chip->prx_inf.detected=%d\n",chip->prx_inf.detected);
		
		input_report_abs(chip->p_idev, ABS_DISTANCE,
				chip->prx_inf.detected ? 0 : 5);
		input_sync(chip->p_idev);
		if(0 == aphw->polling_mode_ps){
			hsd.values[0] = (chip->prx_inf.detected ? 0 : 5);
			ps_report_interrupt_data(hsd.values[0]);
		}
	}
}

void tmg399x_get_prox(struct tmg399x_chip *chip,u8 prox)
{
	int ret;


#if 0	
	ret = tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,
			&chip->shadow[TMG399X_PRX_CHAN]);
	if (ret < 0) {
		h_debug("failed to read proximity raw data\n");
		return;
	} else
//	chip->shadow[TMG399X_PRX_CHAN] = prox;
#endif
	chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];

	if ((chip->prx_inf.detected == false) || (firstprox == true)) {
		if (chip->prx_inf.raw >= prox_thresh_high) {
			chip->prx_inf.detected = true;
			chip->shadow[TMG399X_PRX_MINTHRESHLO] = prox_thresh_low;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] = 0xff;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			tmg399x_report_prox(chip,chip->prx_inf.detected );
			if (ret < 0)
				h_debug(" failed to write proximity threshold\n");
		}
	}
	if ((chip->prx_inf.detected == true) || (firstprox == true)){
		if (chip->prx_inf.raw <= prox_thresh_low) {
			chip->prx_inf.detected = false;
			chip->shadow[TMG399X_PRX_MINTHRESHLO] = 0x00;
			chip->shadow[TMG399X_PRX_MAXTHRESHHI] = prox_thresh_high;
			ret = tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
					chip->shadow[TMG399X_PRX_MINTHRESHLO]);
			ret |= tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
					chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
			tmg399x_report_prox(chip,chip->prx_inf.detected );
			if (ret < 0)
				h_debug("failed to write proximity threshold\n");
		}
	}

	if(firstprox == true)
		firstprox = false;
	
	h_debug("Demon--->prx_inf.raw=%d,prox_th_min=%d,prox_th_max=%d\n",chip->prx_inf.raw,chip->shadow[TMG399X_PRX_MINTHRESHLO],chip->shadow[TMG399X_PRX_MAXTHRESHHI]);
	
}
EXPORT_SYMBOL_GPL(tmg399x_get_prox);

EXPORT_SYMBOL_GPL(tmg399x_report_prox);
void extern_for_tmg_tpd_down(int x,int y,int value);
void extern_for_tmg_tpd_up(int x,int y);
void extern_for_tmg_input_sync(void);
void tmg399x_report_ges(struct tmg399x_chip *chip, int ges_report)
{
//TYD hj ,add for mtk hwmsensor, interrupt mode
	struct alsps_hw *aphw=get_cust_alsps_hw();
	hwm_sensor_data hsd;
	static int ctrm=0;
//e
	int x1, y1, x2, y2;
	int delta_x, delta_y, i,xa=0,ya=0,sx=0,sy=0;
	int reportv;

	x1 = y1 = x2 = y2 = 0;

	switch (ges_report) {
#if defined(A575) || defined(A596)
	case 0:
		printk(KERN_ERR"#h#j#,#North\n");
		x1 = x2 = 400;
		y1 = 300;
		y2 = 100;
		xa=0;
		ya=-1;
		reportv = GES_DOWN;
		break;
	case 2:
		printk(KERN_ERR"#h#j#,#East\n");
		x1 = 200;
		x2 = 400;
		y1 = y2 = 200;
		xa=1;
		ya=0;
		reportv = GES_LEFT;
		break;
	case 4:
		printk(KERN_ERR"#h#j#,#South\n");
		x1 = x2 = 400;
		y1 = 100;
		y2 = 300;
		xa=0;
		ya=1;
		reportv = GES_UP;
		break;
	case 6:
		printk(KERN_ERR"#h#j#,#West\n");
		x1 = 400;
		x2 = 200;
		y1 = y2 = 200;
		xa=-1;
		ya=0;
		reportv = GES_RIGHT;
		break;

#elif defined(A77)
	case 0:
		printk(KERN_ERR"#h#j#,#West\n");
		x1 = 400;
		x2 = 200;
		y1 = y2 = 200;
		xa=-1;
		ya=0;
		reportv = GES_RIGHT;
		break;
	case 2:
		printk(KERN_ERR"#h#j#,#North\n");
		x1 = x2 = 400;
		y1 = 300;
		y2 = 100;
		xa=0;
		ya=-1;
		reportv = GES_DOWN;
		break;
	case 4:
		printk(KERN_ERR"#h#j#,#East\n");
		x1 = 200;
		x2 = 400;
		y1 = y2 = 200;
		xa=1;
		ya=0;
		reportv = GES_LEFT;
		break;
	case 6:
		printk(KERN_ERR"#h#j#,#South\n");
		x1 = x2 = 400;
		y1 = 100;
		y2 = 300;
		xa=0;
		ya=1;
		reportv = GES_UP;
		break;
#endif

	default:
		return;
	}
	//TYd hj
	if(gfdata.ges_user & USER_ANDROID){
		if(0 == aphw->polling_mode_gesture){
			ctrm = (ctrm++)%0xF;
			hsd.values[0] = (ctrm << 4) | (reportv & 0x0F);
			hsd.values[1] = hsd.values[2] = 0;
			hsd.value_divide=1;
			//hwmsen_get_interrupt_data(ID_GESTURE,&hsd);
		//	ps_report_interrupt_data(hsd.values[0]);
		}
	}else if(gfdata.ges_user & USER_LINUX){
	//e
	#if 1
		delta_x = (x2 - x1) / 5;
		delta_y = (y2 - y1) / 5;

    #if defined(A575) || defined(A596)
		y1 = 640;
		x1 = 360;
    #elif defined(A77)
		y1 = 480;
		x1 = 270;
    #endif
		xa*=80;
		ya*=80;

		for (i = 0; ; i++) {
	/*		input_report_abs(chip->p_idev, ABS_MT_TRACKING_ID, 0);
			input_report_abs(chip->p_idev, ABS_MT_TOUCH_MAJOR, 200);
			input_report_abs(chip->p_idev, ABS_MT_POSITION_Y, (y1 + delta_y * i));
			input_report_abs(chip->p_idev, ABS_MT_POSITION_X, (x1 + delta_x * i));
			input_report_abs(chip->p_idev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(chip->p_idev);
			input_sync(chip->p_idev);
	*/
			sx=(x1  + (xa*i*i)/2);
			sy=(y1  + (ya*i*i)/2);
			//extern_for_tmg_tpd_down(sx,sy,1);
			msleep(1);
			//extern_for_tmg_input_sync();

        #if defined(A575) || defined(A596)
			if(sx > 720 || sx < 0 ||sy > 1280 || sy < 0)
				break;
        #elif defined(A77)
			if(sx > 540 || sx < 0 ||sy > 960 || sy < 0)
				break;
        #endif
		}

		mdelay(1);
		//extern_for_tmg_tpd_up(sx,sy);
		//extern_for_tmg_input_sync();
	    //input_mt_sync(chip->p_idev);
	    //input_sync(chip->p_idev);
    #endif
	}
}
EXPORT_SYMBOL_GPL(tmg399x_report_ges);
#define MAX_ELM 1
static unsigned short record[MAX_ELM];
static int rct=0,full=0;
static long lux_sum=0;
static int get_avg_lux(unsigned short lux)
{
	if(rct >= MAX_ELM)
		full=1;

	if(full){
		rct %= MAX_ELM;
		lux_sum -= record[rct];
	}
	lux_sum += lux;
	record[rct]=lux;
	rct++;

	return lux_sum / MAX_ELM;
}
void tmg399x_report_als(struct tmg399x_chip *chip)
{
static char count=0;
static int sum=0;
static int pre_lux=0;
	int lux;
//TYD hj ,add for mtk hwmsensor, interrupt mode
	struct alsps_hw *aphw=get_cust_alsps_hw();
	hwm_sensor_data hsd;
//e

	chip->rgbc_poll_flag = false;
h_debug("before get lux=%d",chip->als_inf.lux);
	tmg399x_get_lux(chip);
h_debug("after get lux=%d",chip->als_inf.lux);
	if(chip->als_enabled)
		mod_timer(&chip->rgbc_timer, jiffies + HZ/10);
//	printk(KERN_INFO "lux:%d cct:%d\n", chip->als_inf.lux, chip->als_inf.cct);
	lux = chip->als_inf.lux;
	input_report_abs(chip->a_idev, ABS_MISC, lux);
	input_sync(chip->a_idev);
	if(0 == aphw->polling_mode_als){
/*		if(count % 100 == 99){
			count = 0;
			lux = sum / 100;
	sum=0;
if(abs(lux - pre_lux) <=5)
	return;
	else pre_lux = lux;
*/
h_debug("avg10=%d\n",lux);
		hsd.values[0] = get_avg_lux(lux);//mapping_als_value(chip,lux); //for mtk
		hsd.values[1] = chip->als_inf.cct;
		//hsd.values[1] = chip->als_inf.red_raw;
        //hsd.values[2] = chip->als_inf.green_raw;
        //hsd.values[3] = chip->als_inf.blue_raw;
        //hsd.values[4] = chip->als_inf.clear_raw;
		hsd.value_divide=1;
		//hwmsen_get_interrupt_data(ID_LIGHT,&hsd);
		ps_report_interrupt_data(hsd.values[0]);
/*		}else{
			count ++;
			sum+=lux;
	}
	*/}
}
EXPORT_SYMBOL_GPL(tmg399x_report_als);

static u8 tmg399x_ges_nswe_min(struct tmg399x_ges_nswe nswe)
{
        u8 min = nswe.north;
        if (nswe.south < min) min = nswe.south;
        if (nswe.west < min) min = nswe.west;
        if (nswe.east < min) min = nswe.east;
        return min;
}

static u8 tmg399x_ges_nswe_max(struct tmg399x_ges_nswe nswe)
{
        u8 max = nswe.north;
        if (nswe.south > max) max = nswe.south;
        if (nswe.west > max) max = nswe.west;
        if (nswe.east > max) max = nswe.east;
        return max;
}

static bool tmg399x_change_prox_offset(struct tmg399x_chip *chip, u8 state, u8 direction,uint8_t speed)
{
	u8 offset;

	switch (state) {
	case CHECK_PROX_NE:
		if (direction == DIR_UP) {
			/* negtive offset will increase the results */
			if (chip->params.prox_offset_ne == -127)
				return false;
			if (chip->params.prox_offset_ne < -122)
				chip->params.prox_offset_ne = -127;
			else
				chip->params.prox_offset_ne -= speed;
		} else if (direction == DIR_DOWN) {
			/* positive offset will decrease the results */
			if (chip->params.prox_offset_ne == 127)
				return false;
			if (chip->params.prox_offset_ne > 122)
				chip->params.prox_offset_ne = 127;
			else
				chip->params.prox_offset_ne += speed; 
		}
		/* convert int value to offset */
		INT2OFFSET(offset, chip->params.prox_offset_ne);
		chip->shadow[TMG399X_PRX_OFFSET_NE] = offset;
		tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_NE, offset);
		break;
	case CHECK_PROX_SW:
		if (direction == DIR_UP) {
			if (chip->params.prox_offset_sw == -127)
				return false;
			if (chip->params.prox_offset_sw < -122)
				chip->params.prox_offset_sw = -127;
			else
				chip->params.prox_offset_sw -= speed;
		} else if (direction == DIR_DOWN) {
			/* positive offset will decrease the results */
			if (chip->params.prox_offset_sw == 127)
				return false;
			if (chip->params.prox_offset_sw > 122)
				chip->params.prox_offset_sw = 127;
			else
				chip->params.prox_offset_sw += speed; 
		}
		INT2OFFSET(offset, chip->params.prox_offset_sw);
		chip->shadow[TMG399X_PRX_OFFSET_SW] = offset;
		tmg399x_i2c_write(chip, TMG399X_PRX_OFFSET_SW, offset);
		break;
	default:
		break;
	}

	return true;
}

static void tmg399x_cal_prox_offset(struct tmg399x_chip *chip, u8 prox)
{
	/* start to calibrate the offset of prox */
	if (caloffsetstate == START_CALOFF) {
		/* mask south and west diode */
		chip->shadow[TMG399X_CONFIG_3] = 0x06;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
			chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetspeed = CAL_FAST;
		caloffsetstate = CHECK_PROX_NE;
		caloffsetdir = DIR_NONE;
		return;
	}

	/* calibrate north and east diode of prox */
	if (caloffsetstate == CHECK_PROX_NE) {
		/* only one direction one time, up or down */
		if(caloffsetspeed = CAL_FAST)
		{	
			if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
				caloffsetdir = DIR_UP;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_UP,4))
					return;
			} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
				caloffsetdir = DIR_DOWN;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_DOWN,4))
					return;
			}
			caloffsetspeed = CAL_SLOW;
			caloffsetdir = DIR_NONE;
			return;
		}
		else if(caloffsetspeed = CAL_SLOW)
		{	
			if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
				caloffsetdir = DIR_UP;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_UP,1))
					return;
			} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
				caloffsetdir = DIR_DOWN;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_NE, DIR_DOWN,1))
					return;
			}
			if (caloffsetdir == DIR_UP)
			{
				caloffsetdir = DIR_NONE;
				return;
				}
			caloffsetspeed = CAL_FAST;
		}

		/* north and east diode offset calibration complete, mask
		north and east diode and start to calibrate south and west diode */
		chip->shadow[TMG399X_CONFIG_3] = 0x09;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
			chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		caloffsetstate = CHECK_PROX_SW;
		caloffsetdir = DIR_NONE;
		return;
	}

	/* calibrate south and west diode of prox */
	if (caloffsetstate == CHECK_PROX_SW) {
		if(caloffsetspeed = CAL_FAST)
		{	
			if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
				caloffsetdir = DIR_UP;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_UP,4))
					return;
			} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
				caloffsetdir = DIR_DOWN;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_DOWN,4))
					return;
			}
			caloffsetspeed = CAL_SLOW;
			caloffsetdir = DIR_NONE;
			return;
		}
		else if(caloffsetspeed = CAL_SLOW)
		{	
			if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
				caloffsetdir = DIR_UP;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_UP,1))
					return;
			} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
				caloffsetdir = DIR_DOWN;
				if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_DOWN,1))
					return;
			}
			if (caloffsetdir == DIR_UP)
			{
				caloffsetdir = DIR_NONE;
				return;
				}
			caloffsetspeed = CAL_FAST;
		}

		/* north and east diode offset calibration complete, mask
		if ((caloffsetdir != DIR_DOWN) && (prox < callowtarget/2)) {
			caloffsetdir = DIR_UP;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_UP))
				return;
		} else if ((caloffsetdir != DIR_UP) && (prox > calhightarget/2)) {
			caloffsetdir = DIR_DOWN;
			if (tmg399x_change_prox_offset(chip, CHECK_PROX_SW, DIR_DOWN))
				return;
		}*/

		/* prox offset calibration complete, mask none diode,
		start to calibrate gesture offset */
		chip->shadow[TMG399X_CONFIG_3] = 0x00;
		tmg399x_i2c_write(chip, TMG399X_CONFIG_3,
			chip->shadow[TMG399X_CONFIG_3]);
		pstatechanged = true;
		tmg399x_i2c_modify(chip, TMG399X_GES_CFG_4, 0x01, 0x01);
		caloffsetstate = CHECK_NSWE_ZERO;
		caloffsetdir = DIR_NONE;
		return;
	}
}

static int tmg399x_change_ges_offset(struct tmg399x_chip *chip, struct tmg399x_ges_nswe nswe_data, u8 direction, u8 speed)
{
	u8 offset;
	int ret = false;

	/* calibrate north diode */
	if (direction == DIR_UP)
	{
		if (nswe_data.north < callowtarget)
		{
			/* negtive offset will increase the results */
			if (chip->params.ges_offset_n == -127)
				goto cal_ges_offset_south;
			if (chip->params.ges_offset_n < -122)
				chip->params.ges_offset_n = -127;
			else
				chip->params.ges_offset_n -= speed;
		}else
			goto cal_ges_offset_south;
	} 
	else if (direction == DIR_DOWN) 
	{
		if (nswe_data.north > calhightarget)
		{
			/* positive offset will decrease the results */
			if (chip->params.ges_offset_n == 127)
				goto cal_ges_offset_south;
			if (chip->params.ges_offset_n > 122)
				chip->params.ges_offset_n = 127;
			else
				chip->params.ges_offset_n += speed;
		}else
			goto cal_ges_offset_south;
	}
	/* convert int value to offset */
	INT2OFFSET(offset, chip->params.ges_offset_n);
	chip->shadow[TMG399X_GES_OFFSET_N] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_N, offset);
	ret = true;

cal_ges_offset_south:
	/* calibrate south diode */
	if (direction == DIR_UP)
	{
		if (nswe_data.south < callowtarget)
		{
			if (chip->params.ges_offset_s == -127)
				goto cal_ges_offset_west;
			if (chip->params.ges_offset_s < -122)
				chip->params.ges_offset_s = -127;
			else
				chip->params.ges_offset_s -= speed;
		}else
			goto cal_ges_offset_west;
	}
	else if (direction == DIR_DOWN)
	{
		if (nswe_data.south > calhightarget)
		{
			if (chip->params.ges_offset_s == 127)
				goto cal_ges_offset_west;
			if (chip->params.ges_offset_s > 122)
				chip->params.ges_offset_s = 127;
			else
				chip->params.ges_offset_s += speed;
		}	else
				goto cal_ges_offset_west;
	}
	INT2OFFSET(offset, chip->params.ges_offset_s);
	chip->shadow[TMG399X_GES_OFFSET_S] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_S, offset);
	ret = true;

cal_ges_offset_west:
	/* calibrate west diode */
	if (direction == DIR_UP) 
	{
		if (nswe_data.west < callowtarget) 
		{
			if (chip->params.ges_offset_w == -127)
				goto cal_ges_offset_east;
			if (chip->params.ges_offset_w < -122)
				chip->params.ges_offset_w = -127;
			else
				chip->params.ges_offset_w -= speed;
		}else
			goto cal_ges_offset_east;
	} else if (direction == DIR_DOWN) 
	{
		if (nswe_data.west > calhightarget)
		{
			if (chip->params.ges_offset_w == 127)
				goto cal_ges_offset_east;
			if (chip->params.ges_offset_w > 122)
				chip->params.ges_offset_w = 127;
			else
				chip->params.ges_offset_w += speed;
		}else
			goto cal_ges_offset_east;
	}
	INT2OFFSET(offset, chip->params.ges_offset_w);
	chip->shadow[TMG399X_GES_OFFSET_W] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_W, offset);
	ret = true;

cal_ges_offset_east:
	/* calibrate east diode */
	if (direction == DIR_UP)
	{
		if (nswe_data.east < callowtarget)
		{
			if (chip->params.ges_offset_e == -127)
				goto cal_ges_offset_exit;
			if (chip->params.ges_offset_e < -122)
				chip->params.ges_offset_e = -127;
			else
				chip->params.ges_offset_e -= speed;
		}else
			goto cal_ges_offset_exit;
	}
	 else if (direction == DIR_DOWN)
	 {
		if (nswe_data.east > calhightarget)
		{
			if (chip->params.ges_offset_e == 127)
				goto cal_ges_offset_exit;
			if (chip->params.ges_offset_e > 122)
				chip->params.ges_offset_e = 127;
			else
				chip->params.ges_offset_e += speed;
		}else
			goto cal_ges_offset_exit;
	}
	INT2OFFSET(offset, chip->params.ges_offset_e);
	chip->shadow[TMG399X_GES_OFFSET_E] = offset;
	tmg399x_i2c_write(chip, TMG399X_GES_OFFSET_E, offset);
	ret = true;

cal_ges_offset_exit:
	return ret;
}

static void tmg399x_cal_ges_offset(struct tmg399x_chip *chip, struct tmg399x_ges_nswe* nswe_data, u8 len)
{
	u8 i;
	u8 min;
	u8 max;
	for (i = 0; i < len; i++) {
		if (caloffsetstate == CHECK_NSWE_ZERO) {
			min = tmg399x_ges_nswe_min(nswe_data[i]);
			max = tmg399x_ges_nswe_max(nswe_data[i]);
			if (caloffsetspeed == CAL_FAST)
			{
				/* only one direction one time, up or down */
				if ((caloffsetdir != DIR_DOWN) && (min <= callowtarget)) {
					caloffsetdir = DIR_UP;
					if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_UP,4))
						return;
				} else if ((caloffsetdir != DIR_UP) && (max > calhightarget)) {
					caloffsetdir = DIR_DOWN;
					if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_DOWN,4))
						return;
				}
				caloffsetdir = DIR_NONE;
				caloffsetspeed = CAL_SLOW;
				return;
			}
			else if(caloffsetspeed == CAL_SLOW)
			{
				/* only one direction one time, up or down */
				if ((caloffsetdir != DIR_DOWN) && (min <= callowtarget))
				{
					caloffsetdir = DIR_UP;
					if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_UP, 1))
						return;
				}
				else if ((caloffsetdir != DIR_UP) && (max > calhightarget))
				{
					caloffsetdir = DIR_DOWN;
					if (tmg399x_change_ges_offset(chip, nswe_data[i], DIR_DOWN, 1))
						return;
				}
				if (caloffsetdir == DIR_UP)
				{
					caloffsetdir = DIR_NONE;
					return;
				}
				caloffsetspeed = CAL_FAST;
			}
			/* calibration is ok */
			caloffsetstate = CALOFF_OK;
			caloffsetdir = DIR_NONE;
			docalibration = false;
			break;
		}
	}
}

void tmg399x_start_calibration(struct tmg399x_chip *chip)
{
	docalibration = true;
	caloffsetstate = START_CALOFF;
	/* entry threshold is set min 0, exit threshold is set max 255,
	and NSWE are all masked for exit, gesture state will be force
	enter and exit every cycle */
	chip->params.ges_entry_th = 0;
	chip->shadow[TMG399X_GES_ENTH] = 0;
	tmg399x_i2c_write(chip, TMG399X_GES_ENTH, 0);
	chip->params.ges_exit_th = 255;
	chip->shadow[TMG399X_GES_EXTH] = 255;
	tmg399x_i2c_write(chip, TMG399X_GES_EXTH, 255);
	chip->params.ges_cfg1 |= GEXMSK_ALL;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	tmg399x_i2c_write(chip, TMG399X_GES_CFG_1, chip->params.ges_cfg1);

	chip->params.persist = PRX_PERSIST(0);
	chip->shadow[TMG399X_PERSISTENCE] = PRX_PERSIST(0);
	tmg399x_i2c_write(chip, TMG399X_PERSISTENCE, PRX_PERSIST(0));
}
EXPORT_SYMBOL_GPL(tmg399x_start_calibration);

void tmg399x_set_ges_thresh(struct tmg399x_chip *chip, u8 entry, u8 exit)
{
	/* set entry and exit threshold for gesture state enter and exit */
	chip->params.ges_entry_th = entry;
	chip->shadow[TMG399X_GES_ENTH] = entry;
	tmg399x_i2c_write(chip, TMG399X_GES_ENTH, entry);
	chip->params.ges_exit_th = exit;
	chip->shadow[TMG399X_GES_EXTH] = exit;
	tmg399x_i2c_write(chip, TMG399X_GES_EXTH, exit);
	chip->params.ges_cfg1 &= ~GEXMSK_ALL;
	chip->shadow[TMG399X_GES_CFG_1] = chip->params.ges_cfg1;
	tmg399x_i2c_write(chip, TMG399X_GES_CFG_1, chip->params.ges_cfg1);
}
EXPORT_SYMBOL_GPL(tmg399x_set_ges_thresh);

void tmg399x_rgbc_poll_handle(unsigned long data)
{
	struct tmg399x_chip *chip = (struct tmg399x_chip *)data;
	chip->rgbc_poll_flag = true;
}

void tmg399x_update_rgbc(struct tmg399x_chip *chip)
{
}
EXPORT_SYMBOL_GPL(tmg399x_update_rgbc);
//add by tyd-zhp
#ifdef TYD_PSENSOR_CALIBRATION_SUPPORT
static ssize_t read_offset(struct device *dev,struct device_attribute *attr, char *buf)
{
#define PS_CALI_NUM 10
#define OFFEST 90
#define LOW_PS_OFFSET 20 //TYD hj,add
	struct tmg399x_chip *chip = gchip;
	int data,i,avg=0,flg=0;//TYD hj,add a flg to save ps on/off

	if(chip == NULL)
		return sprintf(buf,"%d",0);

	if(chip->prx_enabled)
		flg = 1;
	else
		tmg399x_prox_enable(chip,1);
	mdelay(20);
//mutex_unlock(&chip->lock);
	for(i=0;i<PS_CALI_NUM;i++) {
		data = chip->shadow[TMG399X_PRX_CHAN];
		avg += data;
			mdelay(10);
	}
	avg = avg/PS_CALI_NUM;
	if(!flg)
		tmg399x_prox_enable(chip,0);

	if(avg+OFFEST>=200){
		return sprintf(buf,"%d",0);
	}
	else if(avg<255-OFFEST && avg>=0){
		chip->params.prox_th_min=avg + LOW_PS_OFFSET;
		chip->params.prox_th_max=avg+OFFEST;
		return sprintf(buf,"%d#%d#0",avg + LOW_PS_OFFSET,avg+OFFEST);
	}
}

static ssize_t write_offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmg399x_chip *chip = gchip;
	u8 far,near,zero;
	if(chip == NULL){
		return 1;
		}

	sscanf(buf,"%d#%d#%d",&far,&near,&zero);
	if(far<=0xff&&far>0 &&near<=0xFF &&near>0){
		chip->params.prox_th_min=far;
		chip->params.prox_th_max=near;
	}
	return size;
}

static ssize_t show_ps_offset(struct device *dev,struct device_attribute *attr, char *buf)
{
	return read_offset(dev,attr,buf);
}

static ssize_t store_ps_offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	return write_offset(dev,attr,buf,size);
}
//TYD hj,add
static ssize_t show_ps_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct tmg399x_chip *chip = gchip;
	return sprintf(buf,"ps:%d,avg:%d,low:%d,high:%d\n",chip->shadow[TMG399X_PRX_CHAN],chip->params.prox_th_min - LOW_PS_OFFSET,chip->params.prox_th_min,chip->params.prox_th_max);
}
static DEVICE_ATTR(ps_value,0444,show_ps_value,NULL);
//e
static DEVICE_ATTR(ps_offset,0664,show_ps_offset,store_ps_offset);
//end
#endif

static int tmg399x_check_and_report(struct tmg399x_chip *chip)
{
	int ret;
	u8 status;
	u8 numofdset;
	u8 len;
	hwm_sensor_data hsd;     //madan, for CTS Sensor Test.

	mutex_lock(&chip->lock);
	ret = tmg399x_i2c_read(chip, TMG399X_STATUS,
				&chip->shadow[TMG399X_STATUS]);
	if (ret < 0) {
		printk(KERN_ERR"%s: failed to read tmg399x status\n",
			__func__);
		goto exit_clr;
	}

	status = chip->shadow[TMG399X_STATUS];

	if ((status & TMG399X_ST_PRX_IRQ) == TMG399X_ST_PRX_IRQ) {
		/* read prox raw data */
		tmg399x_i2c_read(chip, TMG399X_PRX_CHAN,
                        &chip->shadow[TMG399X_PRX_CHAN]);
		if (chip->prx_enabled)
			chip->prx_inf.raw = chip->shadow[TMG399X_PRX_CHAN];
		/* ignore the first prox data when proximity state changed */
		h_debug("Demon========>docalibration = %d,caloffsetstate = %d,CHECK_NSWE_ZERO = %d,chip->prx_inf.raw = %d\n",docalibration,caloffsetstate,CHECK_NSWE_ZERO,chip->prx_inf.raw);
		if (pstatechanged) {
			pstatechanged = false;
		} else {
			if (docalibration && caloffsetstate != CHECK_NSWE_ZERO && (chip->prx_inf.raw < 200)) {
				
				if (caloffsetstate == START_CALOFF)
				{
					tmg399x_get_prox(chip, chip->prx_inf.raw);
				}
					h_debug("Demon============>chip->shadow[TMG399X_PRX_CHAN]  = %d\n",chip->shadow[TMG399X_PRX_CHAN]);
				if (caloffsetstate != START_CALOFF && chip->shadow[TMG399X_PRX_CHAN] > 110)
				{
					tmg399x_cal_prox_offset(chip, chip->shadow[TMG399X_PRX_CHAN]);
					caloffsetstate = START_CALOFF;
					//if (chip->prx_enabled)firstprox = true;
					chip->shadow[TMG399X_CONFIG_3] = 0x00;
					tmg399x_i2c_write(chip, TMG399X_CONFIG_3, chip->shadow[TMG399X_CONFIG_3]);
				}else{
					/* do prox offset calibration */
					tmg399x_cal_prox_offset(chip, chip->shadow[TMG399X_PRX_CHAN]);
					if(caloffsetstate == CHECK_NSWE_ZERO && !chip->ges_enabled)
					{

						chip->shadow[TMG399X_ALS_TIME] = 0xEE;//50ms
						tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
	
						chip->params.persist = PRX_PERSIST(2);
						chip->shadow[TMG399X_PERSISTENCE] = PRX_PERSIST(2);
						tmg399x_i2c_write(chip, TMG399X_PERSISTENCE, PRX_PERSIST(2));
						tmg399x_get_prox(chip, chip->prx_inf.raw);
					}
					if(chip->ges_enabled)
					{
						chip->shadow[TMG399X_ALS_TIME] = 0xFF;//50ms
						tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
					}
					if(!chip->ges_enabled)
					{
						chip->shadow[TMG399X_ALS_TIME] = 0xEE;//50ms
						tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
					}
				}
			} else {
				if(chip->ges_enabled)
				{
					chip->shadow[TMG399X_ALS_TIME] = 0xFF;//50ms
					tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
				}
				if(!chip->ges_enabled)
				{
					chip->shadow[TMG399X_ALS_TIME] = 0xEE;//50ms
					tmg399x_i2c_write(chip, TMG399X_ALS_TIME, chip->shadow[TMG399X_ALS_TIME]);
				}
				/* process prox data */
				process_rgbc_prox_ges_raw_data(chip, PROX_DATA, (u8*)&chip->shadow[TMG399X_PRX_CHAN], 1);
			}
		}
		/* clear the irq of prox */
		tmg399x_irq_clr(chip, TMG399X_CMD_PROX_INT_CLR);
	}

	if ((status & TMG399X_ST_GES_IRQ) == TMG399X_ST_GES_IRQ) {
		len = 0;
	        while(1) {
			/* get how many data sets in fifo */
	        tmg399x_i2c_read(chip, TMG399X_GES_FLVL, &numofdset);
			if (numofdset == 0) {
				/* fifo empty, skip fifo reading */
				break;
            	}

			//tyd ,hj.MTK support 8byte only
			if(numofdset >=2)
				numofdset=2;
			//
			/* read gesture data from fifo to SW buffer */
                	tmg399x_i2c_ram_blk_read(chip, TMG399X_GES_NFIFO,
                        	(u8 *)&chip->ges_raw_data[len], numofdset*4);
			/* calculate number of gesture data sets */
			len += numofdset;

			if (len > 32) {
				printk (KERN_INFO "gesture buffer overflow!\n");
				len = 0;
				mutex_unlock(&chip->lock);
				return false;
			}
        	}

		if (docalibration && caloffsetstate != CALOFF_OK) {
			/* do gesture offset calibration */
			tmg399x_cal_ges_offset(chip, (struct tmg399x_ges_nswe*)chip->ges_raw_data, len);
		} else {
			process_rgbc_prox_ges_raw_data(chip, GES_DATA, (u8*)chip->ges_raw_data, (len*4));
		}
		/* bellow mod is just for CTS Sensor Test, by madan.*/
		if(1 == gesture_flag){
			gesture_flag = 0;
			if(gfdata.ges_user & USER_ANDROID){
				hsd.values[0] = 0;
				hsd.values[1] = hsd.values[2] = 0;
				hsd.value_divide=1;
			//	hwmsen_get_interrupt_data(ID_GESTURE,&hsd);
				ps_report_interrupt_data(hsd.values[0]);
			}
		}
		/* CTS Sensor Test mod end */
	}

	if ((status & TMG399X_ST_ALS_IRQ) == TMG399X_ST_ALS_IRQ) {
		tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
                        (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		tmg399x_irq_clr(chip, TMG399X_CMD_ALS_INT_CLR);
	}

exit_clr:
	mutex_unlock(&chip->lock);

	return ret;
}

static void tmg399x_irq_work(struct work_struct *work)
{
	struct tmg399x_chip *chip =
		container_of(work, struct tmg399x_chip, irq_work);
	tmg399x_check_and_report(chip);
	h_debug("Demon------------tmg399x_irq_work\n");
	mt_eint_unmask(CUST_EINT_ALS_NUM);
//	enable_irq(chip->client->irq);
};

static irqreturn_t tmg399x_irq(void)
{
	struct device *dev = &gchip->client->dev;
	//mutex_lock(&chip->lock);
	if (gchip->in_suspend) {
		printk( "%s: in suspend\n", __func__);
//		gchip->irq_pending = 1;
//		disable_irq_nosync(gchip->client->irq);
		goto bypass;
	}

	schedule_work(&gchip->irq_work);
bypass:
	return IRQ_HANDLED;
}

static int tmg399x_set_segment_table(struct tmg399x_chip *chip,
		struct lux_segment *segment, int seg_num)
{
	int i;
	struct device *dev = &chip->client->dev;

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
/*
	if (!chip->segment) {
		printk( "%s: allocating segment table\n", __func__);
		chip->segment = kzalloc(sizeof(*chip->segment) *
				chip->seg_num_max, GFP_KERNEL);
		if (!chip->segment) {
			printk( "%s: no memory!\n", __func__);
			return -ENOMEM;
		}
	}
*/
	if (seg_num > chip->seg_num_max) {
		printk( "%s: %d segment requested, %d applied\n",
				__func__, seg_num, chip->seg_num_max);
		chip->segment_num = chip->seg_num_max;
	} else {
		chip->segment_num = seg_num;
	}
/*
	memcpy(chip->segment, segment,
			chip->segment_num * sizeof(*chip->segment));
	printk( "%s: %d segment requested, %d applied\n", __func__,
			seg_num, chip->seg_num_max);
	for (i = 0; i < chip->segment_num; i++)
		printk("seg %d: d_factor %d, r_coef %d, g_coef %d, b_coef %d, ct_coef %d ct_offset %d\n",
		i, chip->segment[i].d_factor, chip->segment[i].r_coef,
		chip->segment[i].g_coef, chip->segment[i].b_coef,
		chip->segment[i].ct_coef, chip->segment[i].ct_offset);
*/
	return 0;
}

static void tmg399x_set_defaults(struct tmg399x_chip *chip)
{
	struct device *dev = &chip->client->dev;

	if (chip->pdata) {
		h_debug( "%s: Loading pltform data\n", __func__);
		chip->params.als_time = chip->pdata->parameters.als_time;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.wait_time = chip->pdata->parameters.wait_time;
		chip->params.prox_th_min = chip->pdata->parameters.prox_th_min;
		chip->params.prox_th_max = chip->pdata->parameters.prox_th_max;
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_prox_cfg1 = chip->pdata->parameters.als_prox_cfg1;
		chip->params.prox_pulse = chip->pdata->parameters.prox_pulse;
		chip->params.prox_gain = chip->pdata->parameters.prox_gain;
		chip->params.ldrive = chip->pdata->parameters.ldrive;
		chip->params.als_prox_cfg2 = chip->pdata->parameters.als_prox_cfg2;
		chip->params.prox_offset_ne = chip->pdata->parameters.prox_offset_ne;
		chip->params.prox_offset_sw = chip->pdata->parameters.prox_offset_sw;
		chip->params.als_prox_cfg3 = chip->pdata->parameters.als_prox_cfg3;
	} else {
		h_debug( "%s: use defaults\n", __func__);
		chip->params.als_time = param_default.als_time;
		chip->params.als_gain = param_default.als_gain;
		chip->params.wait_time = param_default.wait_time;
		chip->params.prox_th_min = param_default.prox_th_min;
		chip->params.prox_th_max = param_default.prox_th_max;
		chip->params.persist = param_default.persist;
		chip->params.als_prox_cfg1 = param_default.als_prox_cfg1;
		chip->params.prox_pulse = param_default.prox_pulse;
		chip->params.prox_gain = param_default.prox_gain;
		chip->params.ldrive = param_default.ldrive;
		chip->params.als_prox_cfg2 = param_default.als_prox_cfg2;
		chip->params.prox_offset_ne = param_default.prox_offset_ne;
		chip->params.prox_offset_sw = param_default.prox_offset_sw;
		chip->params.als_prox_cfg3 = param_default.als_prox_cfg3;
	}

	chip->als_gain_auto = false;

	/* Initial proximity threshold */
	chip->shadow[TMG399X_PRX_MINTHRESHLO] = prox_thresh_low;
	chip->shadow[TMG399X_PRX_MAXTHRESHHI] = prox_thresh_high;
	tmg399x_i2c_write(chip, TMG399X_PRX_MINTHRESHLO,
			chip->shadow[TMG399X_PRX_MINTHRESHLO]);
	tmg399x_i2c_write(chip, TMG399X_PRX_MAXTHRESHHI,
			chip->shadow[TMG399X_PRX_MAXTHRESHHI]);

	tmg399x_i2c_write(chip, TMG399X_ALS_MINTHRESHLO, 0x00);
	tmg399x_i2c_write(chip, TMG399X_ALS_MINTHRESHHI, 0x00);
	tmg399x_i2c_write(chip, TMG399X_ALS_MAXTHRESHLO, 0xFF);
	tmg399x_i2c_write(chip, TMG399X_ALS_MAXTHRESHHI, 0xFF);

	chip->shadow[TMG399X_ALS_TIME]      = chip->params.als_time;
	chip->shadow[TMG399X_WAIT_TIME]     = chip->params.wait_time;
	chip->shadow[TMG399X_PERSISTENCE]   = chip->params.persist;
	chip->shadow[TMG399X_CONFIG_1]      = chip->params.als_prox_cfg1;
	chip->shadow[TMG399X_PRX_PULSE]     = chip->params.prox_pulse;
	chip->shadow[TMG399X_GAIN]          = chip->params.als_gain |
				chip->params.prox_gain | chip->params.ldrive;
	chip->shadow[TMG399X_CONFIG_2]      = chip->params.als_prox_cfg2;
	chip->shadow[TMG399X_PRX_OFFSET_NE] = chip->params.prox_offset_ne;
	chip->shadow[TMG399X_PRX_OFFSET_SW] = chip->params.prox_offset_sw;
	chip->shadow[TMG399X_CONFIG_3]      = chip->params.als_prox_cfg3;
}

static int tmg399x_get_id(struct tmg399x_chip *chip, u8 *id, u8 *rev)
{
	int ret;
	ret = tmg399x_i2c_read(chip, TMG399X_REVID, rev);
	ret |= tmg399x_i2c_read(chip, TMG399X_CHIPID, id);
	return ret;
}

static int tmg399x_pltf_power_on(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
			POWER_ON);
		mdelay(10);
	}
	chip->unpowered = ret != 0;
	return ret;
}

static int tmg399x_pltf_power_off(struct tmg399x_chip *chip)
{
	int ret = 0;
	if (chip->pdata->platform_power) {
		ret = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = ret == 0;
	} else {
		chip->unpowered = false;
	}
	return ret;
}

static int tmg399x_power_on(struct tmg399x_chip *chip)
{
	int ret;
	ret = tmg399x_pltf_power_on(chip);
	if (ret)
		return ret;
	printk("%s: chip was off, restoring regs\n",
			__func__);
	return tmg399x_flush_regs(chip);
}

static int tmg399x_prox_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool als = chip->a_idev && chip->a_idev->users;

	printk("%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_prox_enable(chip, 1);
	if (ret && !als)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_prox_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);

	h_debug( "%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_prox_enable(chip, 0);
	if (!chip->a_idev || !chip->a_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_als_idev_open(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	int ret;
	bool prox = chip->p_idev && chip->p_idev->users;

	h_debug("%s\n", __func__);
	mutex_lock(&chip->lock);
	if (chip->unpowered) {
		ret = tmg399x_power_on(chip);
		if (ret)
			goto chip_on_err;
	}
	ret = tmg399x_als_enable(chip, 1);
	if (ret && !prox)
		tmg399x_pltf_power_off(chip);
chip_on_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static void tmg399x_als_idev_close(struct input_dev *idev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(&idev->dev);
	h_debug("%s\n", __func__);
	mutex_lock(&chip->lock);
	tmg399x_als_enable(chip, 0);
	if (!chip->p_idev || !chip->p_idev->users)
		tmg399x_pltf_power_off(chip);
	mutex_unlock(&chip->lock);
}

static int tmg399x_add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	printk(KERN_ERR"%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tmg399x_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}
//MTK hwsensor,avoid code hal,add hj
static int mapping_als_value(struct tmg399x_chip *chip,u16 als)
{
	int idx;
	int invalid = 0,als_level_num;
	struct alsps_hw *hw = get_cust_alsps_hw();

	als_level_num=sizeof(hw->als_level)/sizeof(hw->als_level[0]);
	for(idx = 0; idx < als_level_num; idx++)
	{
		if(als < hw->als_level[idx])
		{
			break;
		}
	}
	if(idx >= als_level_num + 1)
	{
		printk(KERN_ERR"exceed range\n");
		idx = als_level_num;
	}
	if(chip->als_enabled){
//		h_debug("ALS: raw data %05d => value = %05d\n", als, hw->als_value[idx]);
		return hw->als_value[idx];
	}else{
//		h_debug("als not open\n");
		return -1;
	}
}

#if 0
static int gesture_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0,value;
	struct tmg399x_chip *chip = self;
	hwm_sensor_data *sensor_data;

	h_debug("cmd=%d\n",command);
	switch (command)
	{
		case SENSOR_DELAY:
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR"Enable als sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				mutex_lock(&chip->lock);
				if(value){
					err=tmg399x_ges_enable(chip, 1);
					if(!err)
						gfdata.ges_user |= USER_ANDROID;
				}else{
					if(gfdata.ges_user & USER_ANDROID){
						err=tmg399x_ges_enable(chip, 0);
						gfdata.ges_user &= ~USER_ANDROID;
					}
				}
				mutex_unlock(&chip->lock);
			}
			break;
		case SENSOR_GET_DATA:
			sensor_data = (hwm_sensor_data *)buff_out;

			sensor_data->values[0] = GES_RIGHT;
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			break;
		default:
			err=-1;
			printk(KERN_ERR"no params\n");
			break;
	}
	return err;
}

static int als_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0,value;
	struct tmg399x_chip *chip = self;
	hwm_sensor_data *sensor_data;

	h_debug("cmd=%d\n",command);
	switch (command)
	{
		case SENSOR_DELAY:
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR"Enable als sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				mutex_lock(&chip->lock);
				if(value)
					err=tmg399x_als_enable(chip, 1);
				else
					err=tmg399x_als_enable(chip, 0);
				mutex_unlock(&chip->lock);
			}
			break;
		case SENSOR_GET_DATA:
			sensor_data = (hwm_sensor_data *)buff_out;
            // tmg399x_get_als(chip);
			// err=tmg399x_get_lux(chip);

            tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
                        (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
		    process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);

            //h_debug("before get lux for polling mode======%d",chip->als_inf.lux);
	        tmg399x_get_lux(chip);
            //h_debug("after get lux for polling mode=======%d",chip->als_inf.lux);
 
			sensor_data->values[0] = get_avg_lux(chip->als_inf.lux);
			sensor_data->values[1] = chip->als_inf.cct;
			//h_debug("tmg3993 get als value======%d, cct======%d\n", sensor_data->values[0], sensor_data->values[1]);
            //sensor_data->values[1] = chip->als_inf.red_raw;
            //sensor_data->values[2] = chip->als_inf.green_raw;
            //sensor_data->values[3] = chip->als_inf.blue_raw;
            //sensor_data->values[4] = chip->als_inf.clear_raw;
			//h_debug("tmg3993 get als value=%d, red_raw=%d, green_raw=%d, blue_raw=%d, clear_raw=%d\n", sensor_data->values[0], 
               //sensor_data->values[1], sensor_data->values[2], sensor_data->values[3] , sensor_data->values[4]);
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			break;
		default:
			err=-1;
			printk(KERN_ERR"no params\n");
			break;
	}
	return err;
}
static int prx_sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0,value;
	struct tmg399x_chip *chip = self;
	hwm_sensor_data *sensor_data;
	static flag = 0;
	h_debug("cmd=%d\n",command);
	switch (command)
	{
		case SENSOR_DELAY:
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR"Enable als sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				mutex_lock(&chip->lock);
				if(value)
					err=tmg399x_prox_enable(chip, 1);
				else
					err=tmg399x_prox_enable(chip, 0);
				mutex_unlock(&chip->lock);
			}
			break;
		case SENSOR_GET_DATA:
			sensor_data = (hwm_sensor_data *)buff_out;
//			tmg399x_get_prox(chip);
			sensor_data->values[0] = !!!chip->prx_inf.detected;
			h_debug("tmg339x get als vaule = %d\n",sensor_data->values[0]);
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			break;
		default:
			err=-1;
			printk(KERN_ERR"no params\n");
			break;
	}
	return err;
}
#endif

ssize_t tp_mode_show(struct class *class, struct class_attribute *attr,
		char *buf)
{
	return sprintf(buf,"%s\n",(gfdata.ges_user & USER_LINUX)?"enable":"disable");
}
ssize_t tp_mode_store(struct class *class, struct class_attribute *attr,
		const char *buf, size_t count)
{
	struct tmg399x_chip *chip=gchip;
	bool value;

	mutex_lock(&chip->lock);
	if (strtobool(buf, &value)) {
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	if (value){
		tmg399x_ges_enable(chip, 1);
		gfdata.ges_user |= USER_LINUX;
	}
	else{
		if(gfdata.ges_user & USER_LINUX){
			tmg399x_ges_enable(chip, 0);
			gfdata.ges_user &= ~USER_LINUX;
		}
	}

	mutex_unlock(&chip->lock);
	return count;
}
static CLASS_ATTR(tpmode,0664,tp_mode_show,tp_mode_store);

#ifdef CCI_PSENSOR_CALIBRATION_SUPPORT
static int tmg399x_open(struct inode *inode, struct file *file)
{
	file->private_data = gchip->client;

	if (!file->private_data)
	{
		printk("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int tmg399x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long tmg399x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	    struct tmg399x_chip *chip = gchip;

		long err = 0;
		void __user *ptr = (void __user*) arg;
		int dat;
		uint32_t enable;

		struct tmg399x_threshold calibration_threshold;
		
		switch (cmd)
		{
			case ALSPS_SET_PS_MODE:
				if(copy_from_user(&enable, ptr, sizeof(enable)))
			    {
				    err = -EFAULT;
				    goto err_out;
			    }
			    if(enable)
			    {
				    if((err = tmg399x_prox_enable(chip, 1)))
				    {
					    printk(KERN_ERR"tmg399x enable ps fail: %ld\n", err);
					    goto err_out;
				    }	
			    }
			    else
			    {
				    if((err = tmg399x_prox_enable(chip, 0)))
				    {
					    printk(KERN_ERR"tmg399x disable ps fail: %ld\n", err); 
					    goto err_out;
				    }	
			    }
			    break;
	
			case ALSPS_GET_PS_MODE:
                enable = chip->prx_enabled;
			    if(copy_to_user(ptr, &enable, sizeof(enable)))
			    {
				    err = -EFAULT;
				    goto err_out;
			    }
				break;
	
			case ALSPS_GET_PS_DATA:
				tmg399x_check_and_report(chip);
				dat = !!!chip->prx_inf.detected;
			    if(copy_to_user(ptr, &dat, sizeof(dat)))
			    {
				    err = -EFAULT;
				    goto err_out;
			    }  
				break;
	
			case ALSPS_GET_PS_RAW_DATA:
				tmg399x_check_and_report(chip);
				dat = prox_raw_data;
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}  
		        
				break;			  
	
			case ALSPS_SET_ALS_MODE:	
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = tmg399x_als_enable(chip, 1)))
					{
						printk("tmg399x enable als fail: %ld\n", err); 
						goto err_out;
					}
				}
				else
				{
					if((err = tmg399x_als_enable(chip, 0)))
					{
						printk("tmg399x disable als fail: %ld\n", err); 
						goto err_out;
					}
				}
				break;
	
			case ALSPS_GET_ALS_MODE:
				enable = chip->als_enabled;
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;
	
			case ALSPS_GET_ALS_DATA: 
                tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
                (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
                process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);

                h_debug("ioctl before get lux for polling mode======%d",chip->als_inf.lux);
                tmg399x_get_lux(chip);
                h_debug("ioctl after get lux for polling mode=======%d",chip->als_inf.lux);
                dat = get_avg_lux(chip->als_inf.lux);
				
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
	
			case ALSPS_GET_ALS_RAW_DATA:	
                tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
                (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
                process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);

                h_debug("ioctl before get lux for polling mode======%d",chip->als_inf.lux);
                tmg399x_get_lux(chip);
                h_debug("ioctl after get lux for polling mode=======%d",chip->als_inf.lux);
                dat = chip->als_inf.lux;
				
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;

			case ALSPS_IOCTL_SET_PS_CALI:
				if(copy_from_user(&calibration_threshold, ptr, sizeof(calibration_threshold)))
				{
				    err = -EFAULT;
				    goto err_out;
			    }

			    cci_ps_cali_value = calibration_threshold.result_ps_cali_value;   //Psensor 1.5cm calibration value
			    cci_als_cali_value = 500; // calibration_threshold.result_als_cali_value; //Lsensor 500Lux calibratin value
			    printk(KERN_ERR"%s cci_ps_cali_value = %d\n", __FUNCTION__, cci_ps_cali_value);
				break;

			case ALSPS_SET_GS_MODE:
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = tmg399x_ges_enable(chip, 1)))
					{
						printk("tmg399x enable ges fail: %ld\n", err); 
						goto err_out;
					}
				}
				else
				{
					if((err = tmg399x_ges_enable(chip, 0)))
					{
						printk("tmg399x disable ges fail: %ld\n", err); 
						goto err_out;
					}
				}
				break;

			case ALSPS_GET_GS_MODE:
				enable = chip->ges_enabled;
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_GET_GS_DATA:
				tmg399x_check_and_report(chip);
				dat = ges_ioctl_value;
			    if(copy_to_user(ptr, &dat, sizeof(dat)))
			    {
				    err = -EFAULT;
				    goto err_out;
			    }  
				break;

			case ALSPS_GET_GS_RAW_DATA:
				tmg399x_check_and_report(chip);
				dat = ges_ioctl_value;
			    if(copy_to_user(ptr, &dat, sizeof(dat)))
			    {
				    err = -EFAULT;
				    goto err_out;
			    }  
				break;
				
			case ALSPS_SET_CCT_MODE:
				if(copy_from_user(&enable, ptr, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				if(enable)
				{
					if((err = tmg399x_als_enable(chip, 1)))
					{
						printk("tmg399x enable cct fail: %ld\n", err); 
						goto err_out;
					}
				}
				else
				{
					if((err = tmg399x_als_enable(chip, 0)))
					{
						printk("tmg399x disable cct fail: %ld\n", err); 
						goto err_out;
					}
				}
				break;
				
			case ALSPS_GET_CCT_MODE:
				enable = chip->als_enabled;
				if(copy_to_user(ptr, &enable, sizeof(enable)))
				{
					err = -EFAULT;
					goto err_out;
				}
				break;

			case ALSPS_GET_CCT_DATA:
				tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
                (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
                process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);

                h_debug("ioctl before get cct for polling mode======%d",chip->als_inf.cct);
                tmg399x_get_lux(chip);
                h_debug("ioctl after get cct for polling mode=======%d",chip->als_inf.cct);
                dat = chip->als_inf.cct;
				
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			   
				break;
				
			case ALSPS_GET_CCT_RAW_DATA:
				tmg399x_i2c_ram_blk_read(chip, TMG399X_CLR_CHANLO,
                (u8 *)&chip->shadow[TMG399X_CLR_CHANLO], 8);
                process_rgbc_prox_ges_raw_data(chip, RGBC_DATA, (u8*)&chip->shadow[TMG399X_CLR_CHANLO], 8);

                h_debug("ioctl before get cct for polling mode======%d",chip->als_inf.cct);
                tmg399x_get_lux(chip);
                h_debug("ioctl after get cct for polling mode=======%d",chip->als_inf.cct);
                dat = chip->als_inf.cct;
				
				if(copy_to_user(ptr, &dat, sizeof(dat)))
				{
					err = -EFAULT;
					goto err_out;
				}			  
				break;	
			
			default:
				printk("%s not supported = 0x%04x", __FUNCTION__, cmd);
				err = -ENOIOCTLCMD;
				break;
		}
	
		err_out:
		return err;    
	}

static struct file_operations tmg399x_fops = {
	.owner = THIS_MODULE,
	.open = tmg399x_open,
	.release = tmg399x_release,
	.unlocked_ioctl = tmg399x_unlocked_ioctl,
};

static struct miscdevice tmg399x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tmg399x_fops,
};
#endif

//TYD hj,add
#ifdef CONFIG_EARLYSUSPEND
static void tmg_early_suspend(struct early_suspend *h)
{
	struct tmg399x_chip *chip=gchip;
	mutex_lock(&chip->lock);
	if(gfdata.ges_user & USER_LINUX)
		tmg399x_ges_enable(chip,0);
	mutex_unlock(&chip->lock);
}
static void tmg_late_resume(struct early_suspend *h)
{
	struct tmg399x_chip *chip=gchip;
	mutex_lock(&chip->lock);
	if(gfdata.ges_user & USER_LINUX)
		tmg399x_ges_enable(chip,1);
	mutex_unlock(&chip->lock);
}

static struct early_suspend tmg_eshandle;
#endif

static int tmg399x_als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

static int tmg399x_als_enable_nodata(int en)
{
	int res = 0;

    	printk("tmg399x als enable value = %d\n", en);

	if(gchip == NULL)
	{
		printk("tmg399x  is null!!\n");
		return -1;
	}
	if(en)
	{
	res=tmg399x_als_enable(gchip, 1);
	if(res){
		printk("als_enable_nodata is failed!!\n");
		return -1;
		}
	}else{
		  res = tmg399x_als_enable(gchip, 0);
		if(res < 0)
		{
		printk("ltr579_als_enable_nodata disable als fail!\n"); 
		return -1;
		}
	}
	return 0;
}

static int tmg399x_als_set_delay(u64 ns)
{
	//udelay(ns);
	return 0;
}

static int tmg399x_als_get_data(int* value, int* status)
{
	if(!gchip)
	{
		printk("gchip is null!!\n");
		return -1;
	}
	tmg399x_i2c_ram_blk_read(gchip, TMG399X_CLR_CHANLO,(u8 *)&gchip->shadow[TMG399X_CLR_CHANLO], 8);

	process_rgbc_prox_ges_raw_data(gchip, RGBC_DATA, (u8*)&gchip->shadow[TMG399X_CLR_CHANLO], 8);

            h_debug("before get lux for polling mode======%d\n",gchip->als_inf.lux);

	tmg399x_get_lux(gchip);

	h_debug("after get lux for polling mode=======%d\n",gchip->als_inf.lux);
	*value = get_avg_lux(gchip->als_inf.lux);
	printk("tmg399x_als_get_data ,*value = %d\n",*value);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

static int tmg399x_ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}


// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int tmg399x_ps_enable_nodata(int en)
{
	int res = 0;

    	printk("tmg399x ps enable value = %d\n", en);

	if(gchip == NULL)
	{
		printk("gchip is null!!\n");
		return -1;
	}
	if(en)
	{
		
		res=tmg399x_prox_enable(gchip, 1);
		if(res){
			printk("als_enable_nodata is failed!!\n");
			return -1;
		}
	}else{
		res = tmg399x_prox_enable(gchip, 0);
		if(res < 0)
		{
			printk("disable ps:  %d\n", res);
			return -1;
		}
	}
	return 0;
}

static int tmg399x_ps_set_delay(u64 ns)
{
	//udelay(ns);
	return 0;
}

static int tmg399x_ps_get_data(int* value, int* status)
{
	 u8 ps_raw_value = 0;
	 int ret;
	if(gchip == NULL)
	{
		printk("gchip is null!!\n");
		return -1;
	}
	msleep(60);
//	tmg399x_get_prox(gchip);
	ret = tmg399x_i2c_read(gchip , TMG399X_PRX_CHAN,
			&ps_raw_value);
	if (ret < 0) {
		h_debug("failed to read proximity raw data\n");
		return ret;
	} 
	if(ps_raw_value >= prox_thresh_high-30)
		*value = 0;
	else
		*value=5;
	h_debug("ps_raw_value =%d,vaule = %d\n",ps_raw_value,*value);
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}
static int tmg399x_probe(struct i2c_client *client,
	const struct i2c_device_id *idp)
{
	int i, ret, err;
	u8 id, rev;
	struct device *dev = &client->dev;
	static struct tmg399x_chip *chip;
	struct tmg399x_i2c_platform_data *pdata = dev->platform_data;
	struct class *tmgclass;
	bool powered = 0;
//MTK way
	struct hwmsen_object obj_ps, obj_als,obj_gesture;
	struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
	struct alsps_hw *aphw=get_cust_alsps_hw();
//
	h_debug("X %s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR"%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}
	if (!pdata) {
		printk(KERN_ERR "%s: platform data required\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}
	if (!(pdata->prox_name || pdata->als_name) || client->irq < 0) {
		printk(KERN_ERR"%s: no reason to run.\n", __func__);
		ret = -EINVAL;
		goto init_failed;
	}

	if (pdata->platform_init) {
		ret = pdata->platform_init();	//board_tmg399x_init
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);//board_tmg399x_power
		if (ret) {
			printk(KERN_ERR"%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}

	chip = kmalloc(sizeof(struct tmg399x_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	memset(chip,0,sizeof(struct tmg399x_chip));

	gchip=chip;
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
#ifdef CCI_PSENSOR_CALIBRATION_SUPPORT    
	if((err = misc_register(&tmg399x_device)))
	{
		printk("tmg399x misc device register failed\n");
	}
#endif

	chip->seg_num_max = chip->pdata->segment_num ?
			chip->pdata->segment_num : ARRAY_SIZE(segment_default);
	if (chip->pdata->segment)
		ret = tmg399x_set_segment_table(chip, chip->pdata->segment,
			chip->pdata->segment_num);
	else
		ret =  tmg399x_set_segment_table(chip, segment_default,
			ARRAY_SIZE(segment_default));
	if (ret)
		goto set_segment_failed;

	ret = tmg399x_get_id(chip, &id, &rev);
	if (ret < 0)
			printk(KERN_ERR"%s: failed to get tmg399x id\n",
			__func__);

	printk(KERN_ERR"tmg339x,%s: device id:%02x device rev:%02x\n", __func__,
				id, rev);

	for (i = 0; i < ARRAY_SIZE(tmg399x_ids); i++) {
		if (id == tmg399x_ids[i])
			break;
	}
	if (i < ARRAY_SIZE(tmg399x_names)) {
		printk(KERN_ERR"%s: '%s rev. %d' detected i=%d\n", __func__,
			tmg399x_names[i], rev,i);
		chip->device_index = i;
	} else {
		printk( "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}


//TYD hj,add
	pdata->parameters.prox_th_min=aphw->ps_threshold_low;
	pdata->parameters.prox_th_max=aphw->ps_threshold_high;
//	gfdata.ges_user=0;
//	wake_lock_init(&pslock, WAKE_LOCK_SUSPEND, "pssensor");
//e
	mutex_init(&chip->lock);

	/* disable all */
	tmg399x_prox_enable(chip, 0);
//	tmg399x_ges_enable(chip, 0);
	tmg399x_als_enable(chip, 0);

	tmg399x_set_defaults(chip);
	ret = tmg399x_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;

	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	/* gesture processing initialize */
//	init_params_rgbc_prox_ges();

	if (!pdata->prox_name)
		goto bypass_prox_idev;
	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		printk(KERN_ERR"%s: no memory for input_dev '%s'\n",
				__func__, pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}

	chip->p_idev->name = pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;

#if 0
	set_bit(ABS_DISTANCE, chip->p_idev->absbit);
	input_set_abs_params(chip->p_idev, ABS_DISTANCE, 0, 1, 0, 0);
	set_bit(ABS_MT_TOUCH_MAJOR, chip->p_idev->absbit);
	set_bit(ABS_MT_POSITION_X, chip->p_idev->absbit);
	set_bit(ABS_MT_POSITION_Y, chip->p_idev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, chip->p_idev->absbit);

	input_set_abs_params(chip->p_idev,
			     ABS_MT_POSITION_X, 0, 1280, 0, 0);
	input_set_abs_params(chip->p_idev,
			     ABS_MT_POSITION_Y, 0, 720, 0, 0);
	input_set_abs_params(chip->p_idev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(chip->p_idev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	set_bit(EV_ABS, chip->p_idev->evbit);
	set_bit(EV_KEY, chip->p_idev->evbit);
	chip->p_idev->open = tmg399x_prox_idev_open;
	chip->p_idev->close = tmg399x_prox_idev_close;
#endif
	dev_set_drvdata(&chip->p_idev->dev, chip);

	ret = input_register_device(chip->p_idev);
	if (ret) {
		input_free_device(chip->p_idev);
		printk(KERN_ERR"%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_p_register_failed;
	}

	ret = tmg399x_add_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
	if (ret)
		goto input_p_sysfs_failed;

bypass_prox_idev:
	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		printk(KERN_ERR"%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}

	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
/*	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = tmg399x_als_idev_open;
	chip->a_idev->close = tmg399x_als_idev_close;
*/
	dev_set_drvdata(&chip->a_idev->dev, chip);

	ret = input_register_device(chip->a_idev);
	if (ret) {
		input_free_device(chip->a_idev);
		printk(KERN_ERR"%s: cant register input '%s'\n",
				__func__, pdata->prox_name);
		goto input_a_register_failed;
	}
	ret = tmg399x_add_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
	if (ret)
		goto input_a_sysfs_failed;
	setup_timer(&chip->rgbc_timer,
		tmg399x_rgbc_poll_handle, (unsigned long)chip);

printk("tmg399x 88888\n");
#ifdef CONFIG_EARLYSUSPEND
	tmg_eshandle.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tmg_eshandle.suspend = tmg_early_suspend;
	tmg_eshandle.resume = tmg_late_resume;
	register_early_suspend(&tmg_eshandle);
#endif

#ifdef TYD_PSENSOR_CALIBRATION_SUPPORT
	ret = device_create_file(&(pltdev->dev), &dev_attr_ps_offset);
	if(ret)
		goto input_a_sysfs_failed;
	ret = device_create_file(&(pltdev->dev), &dev_attr_ps_value);
	if(ret)
		goto cal_file_fail1;
#endif
	tmgclass = class_create(THIS_MODULE,"tmg3993");
	ret=class_create_file(tmgclass,&class_attr_tpmode);
	if(ret)
#ifdef TYD_PSENSOR_CALIBRATION_SUPPORT
		goto cal_file_fail2;
#else
		goto input_a_register_failed;
#endif
	printk("tmg399x 99999\n");
bypass_als_idev:
	INIT_WORK(&chip->irq_work, tmg399x_irq_work);
	printk("tmg399x4444\n");
#if 0
//HWDATA ,add,hj
	obj_ps.self=chip;
	obj_ps.polling=aphw->polling_mode_ps;
	obj_ps.sensor_operate = prx_sensor_operate;
	if((ret = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		printk(KERN_ERR"#h#j#  prox attach fail = %d\n", ret);
		goto input_a_sysfs_failed;
	}
	obj_als.self=chip;
	obj_als.polling=aphw->polling_mode_als;
	obj_als.sensor_operate = als_sensor_operate;
	if((ret = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		printk(KERN_ERR"#h#j# als attach fail = %d\n", ret);
		goto input_a_sysfs_failed;
	}
	obj_gesture.self=chip;
	obj_gesture.polling=aphw->polling_mode_gesture;
	obj_gesture.sensor_operate = gesture_sensor_operate;
	if((ret = hwmsen_attach(ID_GESTURE, &obj_gesture)))
	{
		printk(KERN_ERR"#h#j# gesture attach fail = %d\n", ret);
		goto input_a_sysfs_failed;
	}
//e
#else
	als_ctl.is_use_common_factory = false;
	ps_ctl.is_use_common_factory = false;
	als_ctl.enable_nodata = tmg399x_als_enable_nodata;
	als_ctl.open_report_data= tmg399x_als_open_report_data;
	printk("als_ctl.open_report_data = %p\n",als_ctl.open_report_data);
	als_ctl.set_delay  = tmg399x_als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;
	err = als_register_control_path(&als_ctl);
	if(err)
	{
		goto init_failed;
	}
	als_data.get_data = tmg399x_als_get_data;
	als_data.vender_div = 10;
	err = als_register_data_path(&als_data);	
	if(err)
	{
		goto init_failed;
	}
	ps_ctl.open_report_data = tmg399x_ps_open_report_data;
	ps_ctl.enable_nodata = tmg399x_ps_enable_nodata;
	ps_ctl.set_delay = tmg399x_ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);	
	if(err)
	{
		printk("ps register control path fail = %d\n", err);
		goto init_failed;
	}
	printk("tmg399x7777\n");
	ps_data.get_data = tmg399x_ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		printk("ps_register_data_path err = %d\n", err);
		goto init_failed;
	}
	
#endif 


//eint
#if 0
	ret = request_threaded_irq(client->irq, NULL, &tmg399x_irq,
		      IRQF_TRIGGER_LOW | IRQF_ONESHOT,
		      dev_name(dev), chip);
	if (ret) {
		printk( "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}
#else
//use mtk way,keep interrupt for gesture
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

//	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM,/*CUST_EINTF_TRIGGER_FALLING*/CUST_EINTF_TRIGGER_LOW, tmg399x_irq, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
//	mt_eint_mask(CUST_EINT_ALS_NUM);
#endif

//	INIT_WORK(&chip->work_thresh, tmg399x_beam_thread);

	h_debug("Probe ok.\n");
	return 0;

irq_register_fail:
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
#ifdef TYD_PSENSOR_CALIBRATION_SUPPORT
cal_file_fail2:
	device_remove_file(&(pltdev->dev), &dev_attr_ps_value);
cal_file_fail1:
	device_remove_file(&(pltdev->dev), &dev_attr_ps_offset);
#endif
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
input_a_register_failed:
		input_free_device(chip->a_idev);
	}
input_a_alloc_failed:
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
input_p_sysfs_failed:
		input_unregister_device(chip->p_idev);
input_p_register_failed:
		input_free_device(chip->p_idev);
	}
input_p_alloc_failed:
flush_regs_failed:
id_failed:
//	kfree(chip->segment);
set_segment_failed:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	printk(KERN_ERR"Probe failed.\n");
	return ret;
}
/*
static int tmg399x_suspend(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	struct tmg399x_i2c_platform_data *pdata = dev->platform_data;

	printk("%s\n", __func__);
	mutex_lock(&chip->lock);
	chip->in_suspend = 1;

	if (chip->p_idev && chip->p_idev->users) {
		if (pdata->proximity_can_wake) {
			printk("set wake on proximity\n");
//			chip->wake_irq = 1;
		} else {
			printk("proximity off\n");
			tmg399x_prox_enable(chip, 0);
		}
	}
	if (chip->a_idev && chip->a_idev->users) {
		if (pdata->als_can_wake) {
			printk("set wake on als\n");
//			chip->wake_irq = 1;
		} else {
			printk("als off\n");
			tmg399x_als_enable(chip, 0);
		}
	}
//	if (chip->wake_irq) {
//		irq_set_irq_wake(chip->client->irq, 1);
//	} else
	if (!chip->unpowered) {
		printk("powering off\n");
		tmg399x_pltf_power_off(chip);
	}

	mutex_unlock(&chip->lock);

	return 0;
}

static int tmg399x_resume(struct device *dev)
{
	struct tmg399x_chip *chip = dev_get_drvdata(dev);
	bool als_on, prx_on;
	int ret = 0;
	mutex_lock(&chip->lock);
	prx_on = chip->p_idev && chip->p_idev->users;
	als_on = chip->a_idev && chip->a_idev->users;
	chip->in_suspend = 0;

	h_debug( "%s: powerd %d, als: needed %d  enabled %d",
			__func__, !chip->unpowered, als_on,
			chip->als_enabled);
	h_debug(" %s: prox: needed %d  enabled %d\n",
			__func__, prx_on, chip->prx_enabled);

//	if (chip->wake_irq) {
//		irq_set_irq_wake(chip->client->irq, 0);
//		chip->wake_irq = 0;
//	}

	if (chip->unpowered && (prx_on || als_on)) {
		printk( "powering on\n");
		ret = tmg399x_power_on(chip);
		if (ret)
			goto err_power;
	}
	if (prx_on && !chip->prx_enabled)
		(void)tmg399x_prox_enable(chip, 1);
	if (als_on && !chip->als_enabled)
		(void)tmg399x_als_enable(chip, 1);
//	if (chip->irq_pending) {
//		printk("%s: pending interrupt\n", __func__);
//		chip->irq_pending = 0;
//		(void)tmg399x_check_and_report(chip);
//		enable_irq(chip->client->irq);
//	}
err_power:
	mutex_unlock(&chip->lock);

	return 0;
}
*/
static int tmg399x_remove(struct i2c_client *client)
{
	struct tmg399x_chip *chip = i2c_get_clientdata(client);
	mutex_lock(&chip->lock);
//	free_irq(client->irq, chip);
#ifdef TYD_PSENSOR_CALIBRATION_SUPPORT
	device_remove_file(&(pltdev->dev), &dev_attr_ps_value);
	device_remove_file(&(pltdev->dev), &dev_attr_ps_offset);
#endif
	if (chip->a_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->a_idev->dev,
			als_attrs, ARRAY_SIZE(als_attrs));
		input_unregister_device(chip->a_idev);
	}
	if (chip->p_idev) {
		tmg399x_remove_sysfs_interfaces(&chip->p_idev->dev,
			prox_attrs, ARRAY_SIZE(prox_attrs));
		input_unregister_device(chip->p_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
//	kfree(chip->segment);
	kfree(chip);
	mutex_unlock(&chip->lock);
	return 0;
}

static struct i2c_device_id tmg399x_idtable[] = {
	{ "tmg3993", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tmg399x_idtable);
/*
static const struct dev_pm_ops tmg399x_pm_ops = {
	.suspend = tmg399x_suspend,
	.resume  = tmg399x_resume,
};
*/
static struct i2c_driver tmg399x_driver = {
	.driver = {
		.name = "tmg3993",
//TYD hj,rm
//		.pm = &tmg399x_pm_ops,
//e
	},
	.id_table = tmg399x_idtable,
	.probe = tmg399x_probe,
	.remove = tmg399x_remove,
};
//device register
static int board_tmg399x_init(void)
{
	h_debug("board_tmg399x_init CALLED\n");

//    mt_eint_unmask(CUST_EINT_ALS_NUM);
	return 0;
}

static int board_tmg399x_power(struct device *dev, enum tmg399x_pwr_state state)
{
	h_debug("board_tmg399x_power CALLED\n");

	//setup_pin_mux(tmg399x_pin_mux);
	return 0;
}

static void board_tmg399x_teardown(struct device *dev)
{
	h_debug("board_tmg399x_teardow CALLED\n");
}
/* TYD hj ,rm
static const struct lux_segment tmg399x_segment[] = {
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
	{
		.d_factor = D_Factor1,
		.r_coef = R_Coef1,
		.g_coef = G_Coef1,
		.b_coef = B_Coef1,
		.ct_coef = CT_Coef1,
		.ct_offset = CT_Offset1,
	},
};
*/
struct tmg399x_i2c_platform_data tmg399x_data = {
	.platform_power = board_tmg399x_power,
	.platform_init = board_tmg399x_init,
	.platform_teardown = board_tmg399x_teardown,
	.prox_name = "tmg399x_proximity",
	.als_name = "tmg399x_als",
        .parameters = {
#if 0
                .als_time = 0xFE, /* 5.6ms */
                .als_gain = AGAIN_64,
                .wait_time = 0xFF, /* 2.78ms */
                .prox_th_min = 0,
                .prox_th_max = 255,
                .persist = PRX_PERSIST(0) | ALS_PERSIST(0),
                .als_prox_cfg1 = 0x60,
                .prox_pulse = PPLEN_4US | PRX_PULSE_CNT(10),
                .prox_gain = PGAIN_4,
                .ldrive = PDRIVE_100MA,
                .als_prox_cfg2 = LEDBOOST_150 | 0x01,
                .prox_offset_ne = 0,
                .prox_offset_sw = 0,
                .als_prox_cfg3 = 0x00,

                .ges_entry_th = 0x20,
                .ges_exit_th = 0x10,
                .ges_cfg1 = FIFOTH_1 | GEXMSK_ALL | GEXPERS_1,
                .ges_cfg2 = GGAIN_1 | GLDRIVE_100 | GWTIME_3,
                .ges_offset_n = 0,
                .ges_offset_s = 0,
                .ges_pulse = GPLEN_16US | GES_PULSE_CNT(32),
                .ges_offset_w = 0,
                .ges_offset_e = 0,
                .ges_dimension = GBOTH_PAIR,
#endif
        },
        .beam_settings = {
                .beam_cfg  = IRBEAM_CFG,
                .beam_carr = IRBEAM_CARR,
                .beam_ns   = IRBEAM_NS,
                .beam_isd  = IRBEAM_ISD,
                .beam_np   = IRBEAM_NP,
                .beam_ipd  = IRBEAM_IPD,
                .beam_div  = IRBEAM_DIV,
        },
        .als_can_wake = false,
        .proximity_can_wake = true,
        .segment = 0,//(struct lux_segment *) tmg399x_segment,
        .segment_num = 0,//ARRAY_SIZE(tmg399x_segment),

};
static struct i2c_board_info __initdata tmg3993=
{
       I2C_BOARD_INFO("tmg3993", 0x39),
	.irq = CUST_EINT_ALS_NUM,
	.platform_data = &tmg399x_data,
};
static inline void register_boardinfo(void)
{
	struct alsps_hw *aphw=get_cust_alsps_hw();
	struct tmg399x_parameters *parameters=get_tmg_arglist();

	memcpy(&tmg399x_data.parameters,parameters,sizeof(struct tmg399x_parameters));

	i2c_register_board_info(aphw->i2c_num, &tmg3993, 1);
}

static int  tmg399x_local_init(void)
{
	if(i2c_add_driver(&tmg399x_driver))	{
		printk(KERN_ERR"tmg register i2c driver err\n");
		return -1;
	}	
	return 0;
}

static int tmg399x_local_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tmg399x_driver);
	return 0;
}
static int i2c_tmg399x_probe(struct platform_device *pdev)
{
	pltdev = pdev;
	alsps_driver_add(&tmg399x_init_info);
	return 0;
}
static int i2c_tmg399x_remove(struct platform_device *pdev)
{
#ifdef CCI_PSENSOR_CALIBRATION_SUPPORT
    int err;
	if((err = misc_deregister(&tmg399x_device)))
	{
		printk("tmg399x misc device register fail: %d\n", err);    
	}
#endif
    
	i2c_del_driver(&tmg399x_driver);
	return 0;
}

static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};

static struct platform_driver tmg399x_alsps_driver = {
	.probe      = i2c_tmg399x_probe,
	.remove     = i2c_tmg399x_remove,
	.driver     = {
		.name  = "als_ps",
		.of_match_table = alsps_of_match,	
//		.owner = THIS_MODULE,
	}
};
//tyd hj,FIXME
//MTK register hwmdev with late_initcall,we must get this...
//
static int __init tmg399x_init(void)
{
	register_boardinfo();
	if(platform_driver_register(&tmg399x_alsps_driver))
	{
		printk(KERN_ERR"failed to register driver");
		return -ENODEV;
	}

	return 0;
}

static void __exit tmg399x_exit(void)
{
	platform_driver_unregister(&tmg399x_alsps_driver);
}

module_init(tmg399x_init);
module_exit(tmg399x_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_AUTHOR("Byron Shi<byron.shi@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmg3992/3 Ambient, Proximity, Gesture sensor driver");
MODULE_LICENSE("GPL");

