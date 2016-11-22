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
 
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <cust_mag.h>
#include <linux/hwmsen_helper.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "mxg2320.h"

#define MXG2320_MTK_NEW_SENSOR_ARCH

#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
#include <linux/batch.h>
#include "mag.h"
#endif

/* Global Identifier Declaration */
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define MXG2320_WORK_TEST

/* Global Define Declaration */
#define MXG2320_DATA_READY_CHECK(x) (x & 0x1)
#define MXG2320_DATA_OVERFLOW_CHECK(x) (x & ( 0x1 << 3 ))
#define RAW_TO_GAUSS(x, y, z) ((( x * y / 128 ) + ( x )) * z)

/* Debug Declaration */
#define MSE_TAG                 "[MSENSOR] "
#define MSE_FUN(f)              printk(MSE_TAG"%s\n", __FUNCTION__)
#define MSE_INF(fmt, args...)   printk(MSE_TAG fmt, ##args)
#define MSE_ERR(fmt, args...)   printk(KERN_ERR MSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)   printk(MSE_TAG fmt, ##args)
#define MSE_VER(fmt, args...)   ((void)0)

//#define CHECK_IC_NUM_ENABLE

#ifdef CHECK_IC_NUM_ENABLE
static u8 mxg_write_buf[10] = {0x13, 0x20, 0x01, 0xc0, 0x0b, 
                                0x08, 0x00, 0x00, 0x08, 0x01};
static u8 mxg_address_buf[10] = {0x90, 0x91, 0x98, 0x92, 0x93,
                                0x94, 0x95, 0x96, 0x97, 0x97};
#endif


typedef enum {
        MXG_FUNC_TRC_FLAG = 0x01,
        MXG_CTR_TRC_FLAG = 0x02,
        MXG_M_DATA_TRC_FLAG = 0x04,
        MXG_O_DATA_TRC_FLAG = 0x08,
        MXG_GY_DATA_TRC_FLAG = 0x10,
        MXG_RV_DATA_TRC_FLAG = 0x20,
        MXG_GV_DATA_TRC_FLAG = 0x40,
        MXG_LA_DATA_TRC_FLAG = 0x80,
} MXG_TRC;

typedef enum {
        MX_SEN_MAGNETIC = 0,
        MX_SEN_UNCAL_MAGNETIC,
        MX_SEN_ORIENTATION,
        MX_SEN_GYROSCOP,
        MX_SEN_ROTATION_VECTOR,
        MX_SEN_GRAVITY,
        MX_SEN_LINEAR_ACCELERATION,
        MX_SEN_MAX,
} MXG_SEN;

/* static variable Declaration */
static const struct i2c_device_id mxg2320_i2c_id[] = {
        {MXG2320_DEV_NAME,0},
        {}
};

static struct i2c_board_info __initdata i2c_mxg2320 = {
        I2C_BOARD_INFO("mxg2320", (MXG2320_I2C_ADDRESS>>1))
};

static struct i2c_client *this_client = NULL;
static int mxg2320_init_flag = -1;
static DEFINE_MUTEX(mxg2320_i2c_mutex);

#ifdef MXG2320_WORK_TEST
struct delayed_work mx_work;
#endif

static DECLARE_WAIT_QUEUE_HEAD(open_wq);

/* Forward Declaration of function */
static int mxg2320_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int mxg2320_i2c_remove(struct i2c_client *client);

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void mxg2320_early_suspend(struct early_suspend *h);
static void mxg2320_late_resume(struct early_suspend *h);
#else
static int mxg2320_suspend(struct i2c_client *client, pm_message_t msg);
static int mxg2320_resume(struct i2c_client *client);
#endif

struct mxg2320_conf {
        u8 coef[3];
        u8 id[2];
};

struct mxg2320_ctrl {
        rwlock_t ctrllock;
        int mode;

        int delay[MX_SEN_MAX];
        int active[MX_SEN_MAX];

        atomic_t m_flag;
        atomic_t o_flag;
        atomic_t o_count;
        atomic_t open_flag;
        atomic_t dev_open_count;
};


struct mxg2320_data {
        rwlock_t datalock;
        MXG_SENSOR_DATA mag;
        MXG_SENSOR_DATA umag;
        MXG_SENSOR_DATA ori;
        MXG_SENSOR_DATA gyr;
        MXG_SENSOR_DATA rov;
        MXG_SENSOR_DATA grv;
        MXG_SENSOR_DATA lac;
};

struct mxg2320_i2c_data {
        struct i2c_client *client;
        struct mag_hw *hw;
        struct hwmsen_convert cvt;
        struct mxg2320_data mdata;
        struct mxg2320_conf mconf;
        struct mxg2320_ctrl mcrtl;
        atomic_t layout;
        atomic_t trace;
#if defined(CONFIG_HAS_EARLYSUSPEND)
        struct early_suspend early_drv;
#endif
};


#ifdef MXG2320_MTK_NEW_SENSOR_ARCH

static int mxg2320_local_init(void);
static int mxg2320_local_uninit(void);

static struct mag_init_info mxg2320_init_info = {
        .name = "mxg2320",
        .init = mxg2320_local_init,
        .uninit = mxg2320_local_uninit,
};

#else

#ifdef CONFIG_OF
static const struct of_device_id mxg2320_of_match[] = {
        { .compatible = "mediatek,msensor", },
        {},
};
#endif

static int mxg2320_probe(struct platform_device *pdev);
static int mxg2320_remove(struct platform_device *pdev);

static struct platform_driver mxg2320_sensor_driver = {
        .probe = mxg2320_probe,
        .remove = mxg2320_remove,    
        .driver = {
                .name = "msensor",
#ifdef CONFIG_OF
		.of_match_table = mxg2320_of_match,
#endif
        }
};

#endif

static struct i2c_driver mxg2320_i2c_driver = {
        .driver = {
                .owner = THIS_MODULE, 
                .name  = MXG2320_DEV_NAME,
        },
        .probe = mxg2320_i2c_probe,
        .remove = mxg2320_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
        .suspend = mxg2320_suspend,
        .resume = mxg2320_resume,
#endif 
        .id_table = mxg2320_i2c_id,
};



static int mxg2320_i2c_write_byte(u8 reg, u8 val)
{
        int ret;

        ret = i2c_smbus_write_byte_data(this_client, reg, val);
        if ( ret < 0 ) {
                MSE_ERR("write(byte) to device fails status %x\n", ret);
                return ret;
        }       
        return 0;
}


static int mxg2320_i2c_read_block(u8 reg_addr, u8 *data, int len)
{
        int ret;

        ret = i2c_smbus_read_i2c_block_data(this_client, reg_addr, len, data);
        if ( ret < 0 ) {
                MSE_ERR("read(block) to device fails status %x\n", ret);
                goto i2c_read_block_error;
        }

i2c_read_block_error:
        return ret < 0 ? ret: 0;
}


static int mxg2320_operation_mode(u8 mode)
{
        int ret;

        ret = mxg2320_i2c_write_byte(MXG2320_REG_SCTRL, mode);
        if ( ret < 0 ) {
                MSE_ERR("change operation mode failed.\n");
        } else {
                if ( mode == MXG2320_MOD_POWER_DOWN )
                        udelay(MXG2320_DELAY_MOD_TRANS);
        }       
        return ret;
}

static void mxg2320_power(struct mag_hw *hw, unsigned int on) 
{
        static unsigned int power_on = 0;

        if ( hw->power_id != POWER_NONE_MACRO ) {
                MSE_LOG("power %s\n", on ? "on" : "off");
                if ( power_on == on ) {
                        MSE_LOG("ignore power control: %d\n", on);
                } else {
                        if ( on ) {
                                if ( !hwPowerOn(hw->power_id, hw->power_vol, "mxg2320") )
                                        MSE_ERR( "power on fails!!\n");
                        } else {
                                if ( !hwPowerDown(hw->power_id, "mxg2320") )
                                        MSE_ERR( "power off fail!!\n");
                        }
                }
        }
        
        power_on = on;
}


static int mxg2320_GetOpenStatus(struct mxg2320_i2c_data *data)
{
	wait_event_interruptible(open_wq, (atomic_read(&data->mcrtl.open_flag) != 0));
	return atomic_read(&data->mcrtl.open_flag);
}

static int mxg2320_GetCloseStatus(struct mxg2320_i2c_data *data)
{
	wait_event_interruptible(open_wq, (atomic_read(&data->mcrtl.open_flag) <= 0));
	return atomic_read(&data->mcrtl.open_flag);
}


static int mxg2320_ReadChipInfo(char *buf, int bufsize)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        
	if ( (!buf) || (bufsize <= MXG2320_BUFSIZE - 1) ) {
		return -1;
	}
	
	if ( !this_client ) {
		*buf = 0;
		return -2;
	}
	
	if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG ) {
                MSE_LOG("mxg2320 Chip \n");
        }
        
	sprintf(buf, "mxg2320 Chip");
	
	return 0;
}

static int mxg2320_ReadSensorData(char *buf, int bufsize)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( (!buf) || (bufsize <= MXG2320_BUFSIZE - 1) ) {
                return -1;
        }

        read_lock(&data->mdata.datalock);
        sprintf(buf, "%d %d %d", data->mdata.umag.ux, data->mdata.umag.uy, data->mdata.umag.uz);
        read_unlock(&data->mdata.datalock);

        if ( atomic_read(&data->trace) & MXG_M_DATA_TRC_FLAG ) {
                MSE_LOG("mxg2320 uncalibrated data x : %d y : %d z : %d \n",
                        data->mdata.umag.ux, data->mdata.umag.uy, data->mdata.umag.uz);
        }
        
        return 0;
}


static int mxg2320_ReadPostureData(char *buf, int bufsize)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( (!buf) || (bufsize <= MXG2320_BUFSIZE - 1) ) {
                return -1;
        }

        read_lock(&data->mdata.datalock);
        sprintf(buf, "%d %d %d %d", data->mdata.ori.azimuth, data->mdata.ori.pitch,
                data->mdata.ori.roll, data->mdata.ori.status);
        read_unlock(&data->mdata.datalock);

        if ( atomic_read(&data->trace) & MXG_O_DATA_TRC_FLAG ) {
                MSE_LOG("mxg2320 posture data x : %d y : %d z : %d accuracy : %d\n",
                        data->mdata.ori.azimuth, data->mdata.ori.pitch, data->mdata.ori.roll, data->mdata.ori.status);
        }
        
        return 0;
}

static int mxg2320_ReadCaliData(char *buf, int bufsize)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( (!buf) || (bufsize <= (MXG2320_BUFSIZE - 1)) ) {
                return -1;
        }

        read_lock(&data->mdata.datalock);
        sprintf(buf, "%d %d %d %d", data->mdata.mag.x, data->mdata.mag.y, 
                data->mdata.mag.z, data->mdata.mag.status);
        read_unlock(&data->mdata.datalock);

        if ( atomic_read(&data->trace) & MXG_M_DATA_TRC_FLAG ) {
                MSE_LOG("mxg2320 calibrated data x : %d y : %d z : %d accuracy : %d\n",
                        data->mdata.mag.x, data->mdata.mag.y, data->mdata.mag.z, data->mdata.mag.status);
        }
        return 0;
}

static int mxg2320_SetMode(int newmode)
{
        int mode = 0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        read_lock(&data->mcrtl.ctrllock);
        mode = data->mcrtl.mode;
        read_unlock(&data->mcrtl.ctrllock);

        if ( mode == newmode ) {
                return 0;
        } else {
                write_lock(&data->mcrtl.ctrllock);
                data->mcrtl.mode = newmode;
                write_unlock(&data->mcrtl.ctrllock);
        }

        return mxg2320_operation_mode(newmode);
}

static long mxg2320_data_measurement(struct mxg2320_i2c_data *data, int *rbuf)
{
        int i, loop, ret;
        //short raw_temp[3]={0,};
        u8 *st1, *st2, *tmps, *raw_data;
        u8 buf[MXG2320_MEASUREMENT_DATA_SIZE]={0,};

        st1 = &buf[MXG2320_MEASUREMENT_ST1_POS];
        tmps = &buf[MXG2320_MEASUREMENT_TMP_POS];
        st2 = &buf[MXG2320_MEASUREMENT_ST2_POS];
        raw_data = &buf[MXG2320_MEASUREMENT_RAW_POS];

        for ( loop = 0; loop < 10; loop++) {        
        	if ( (ret = mxg2320_i2c_read_block(MXG2320_REG_STR1, st1, 1)) )
        	{
        		MSE_ERR("read ST1 resigster failed!\n");
        		return ret;
        	}

        	if ( MXG2320_DATA_READY_CHECK(*st1) ) { 
                        break;
        	}
        	msleep(1);

        	if ( loop == 9 ) {
                        MSE_ERR("finally data is not read yet.\n");
        	}
	}
	
	if ( (ret = mxg2320_i2c_read_block(MXG2320_REG_XDH, raw_data, 6)) )
	{
		MSE_ERR("read axis data resigster failed!\n");
		return ret;
	}

	if ( (ret = mxg2320_i2c_read_block(MXG2320_REG_DMY, tmps, 2)) )
	{
		MSE_ERR("read ST2 resigster failed!\n");
		return ret;
	}
	
	
        if ( MXG2320_DATA_READY_CHECK(*st1) ) {
		for ( i = 0; i < 3; i++ ) {
		        rbuf[i] = (short)(( raw_data[(i*2)+1])|(raw_data[(i*2)] << 8));
		}
	} else {
		MSE_ERR("data is not read yet.\n");
		return -1;
	}
	
	if ( MXG2320_DATA_OVERFLOW_CHECK(*st2) )
                MSE_ERR("data is overflow.\n");
	
	return 0;
}


static int mxg2320_get_raw_data(int mode)
{	
        int i, ret;
        int strbuf[3];
        int fctbuf[3];
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

	if ( (mode != MXG2320_MOD_SNG_MEASURE) 
	        && (mode != MXG2320_MOD_SELF_TEST) ) {
	        return -1;
	}

	if ( mode == MXG2320_MOD_SELF_TEST ) {
                mxg2320_operation_mode(MXG2320_MOD_POWER_DOWN);
	}

        if ( mode == MXG2320_MOD_SELF_TEST ) {
                ret = mxg2320_operation_mode(mode);
                if ( ret < 0 ) {
                        return ret;
                }
                mdelay(1000);
	}
        
        if ( mxg2320_data_measurement(data, strbuf) ) {
                /* one more try raw data read */
                mxg2320_data_measurement(data, strbuf);
        }

        if ( mode == MXG2320_MOD_SNG_MEASURE ) {
                write_lock(&data->mdata.datalock);
                for ( i = 0; i < 3; i++ ) {
                        data->mdata.umag.data[data->cvt.map[i]] = RAW_TO_GAUSS(strbuf[i], data->mconf.coef[i], data->cvt.sign[i]);
                }
                write_unlock(&data->mdata.datalock);

                if ( atomic_read(&data->trace) & MXG_M_DATA_TRC_FLAG ) {
                        MSE_LOG("magnectic raw data x = %d, y = %d, z = %d\n",
	                        data->mdata.umag.data[0],
	                        data->mdata.umag.data[1],
	                        data->mdata.umag.data[2]);
	        }
                
        } else if ( mode == MXG2320_MOD_SELF_TEST ) {
                /* self test measure */
                for ( i = 0; i < 3; i++ ) {
                        fctbuf[i] = RAW_TO_GAUSS(strbuf[i], data->mconf.coef[i], 1);
                }
                if ( atomic_read(&data->trace) & MXG_M_DATA_TRC_FLAG ) {
                        MSE_LOG("magnectic factory test data x = %d, y = %d, z = %d\n",
        	                fctbuf[0], fctbuf[1], fctbuf[2]);
                }
  
                if ( (( fctbuf[0] > MXG2320_COEF_X_MAX ) || ( fctbuf[0] < MXG2320_COEF_X_MIN )) ||
                        (( fctbuf[1] > MXG2320_COEF_Y_MAX ) || ( fctbuf[1] < MXG2320_COEF_Y_MIN )) ||
                        (( fctbuf[2] > MXG2320_COEF_Z_MAX ) || ( fctbuf[2] < MXG2320_COEF_Z_MIN )) ) {
                        return 0;
                } else {
                        return 1;
                }
        }

        return 0;

}



/* Driver Attributes Functions Section */
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
        char strbuf[MXG2320_BUFSIZE];
        
        sprintf(strbuf, "mxg2320d");
        return sprintf(buf, "%s", strbuf);
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
        char strbuf[MXG2320_BUFSIZE];
        
        mxg2320_ReadChipInfo(strbuf, MXG2320_BUFSIZE);

        return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
        char strbuf[MXG2320_BUFSIZE];
        
        mxg2320_ReadSensorData(strbuf, MXG2320_BUFSIZE);
        
        return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
        char strbuf[MXG2320_BUFSIZE];

        mxg2320_ReadPostureData(strbuf, MXG2320_BUFSIZE);

        return sprintf(buf, "%s\n", strbuf);
}

static ssize_t show_calidata_value(struct device_driver *ddri, char *buf)
{
        char strbuf[MXG2320_BUFSIZE];
        
        mxg2320_ReadCaliData(strbuf, MXG2320_BUFSIZE);
        
        return sprintf(buf, "%s\n", strbuf);
}


static ssize_t show_factorytest_value(struct device_driver *ddri, char *buf)
{
        int ret = 0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( atomic_read(&data->mcrtl.open_flag) == 1 ) {
                MSE_ERR("self-test is busy!\r\n");
                ret = -EBUSY;
                return sprintf(buf, "%d\n", ret);
        }
        
        ret = mxg2320_get_raw_data(MXG2320_MOD_SELF_TEST);

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG ) {
                if ( ret ) {
                        MSE_ERR("self-test is success!\r\n");
                } else {
                        MSE_ERR("self-test is failed!\r\n");
                }
        }
        
        return sprintf(buf, "%d\n", ret);
}

static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{	
        ssize_t len = 0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( data->hw ) {
                len += snprintf((buf + len), (PAGE_SIZE - len), "CUST: %d %d (%d %d)\n", 
                        data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
                        
                if ( atomic_read(&data->trace) & MXG_CTR_TRC_FLAG ) {
                        MSE_INF("mxg2320 i2c num : %d,  direction : %d, power id : %d power vol : %d\n",
                                data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
                }
                
        } else {
                len += snprintf((buf + len), (PAGE_SIZE - len), "CUST: NULL\n");
        }

        return len;
}

static ssize_t show_control_value(struct device_driver *ddri, char *buf)
{
        int i = 0, temp[MX_SEN_MAX];
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        read_lock(&data->mcrtl.ctrllock);
        for( i = 0  ; i < MX_SEN_MAX; i++ ) {
                temp[i] = data->mcrtl.delay[i];
        }        
        read_unlock(&data->mcrtl.ctrllock);

        if ( atomic_read(&data->trace) & MXG_CTR_TRC_FLAG ) {
                MSE_INF("magnetic               : %s\n", (temp[0] > 0) ? "true": "false" );
                MSE_INF("uncalibrated magnetic  : %s\n", (temp[1] > 0) ? "true": "false" );
                MSE_INF("orientation            : %s\n", (temp[2] > 0) ? "true": "false" );
                MSE_INF("gyroscope              : %s\n", (temp[3] > 0) ? "true": "false" );
                MSE_INF("rotation vector        : %s\n", (temp[4] > 0) ? "true": "false" );
                MSE_INF("gravity                : %s\n", (temp[5] > 0) ? "true": "false" );
                MSE_INF("linear accelleration   : %s\n", (temp[6] > 0) ? "true": "false" );
        }              
        
        return sprintf(buf, "%d, %d, %d, %d, %d, %d, %d\n",
                temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], temp[6]);
}

static ssize_t store_control_value(struct device_driver *ddri, const char *buf, size_t count)
{
        return count;
}


static ssize_t show_mode_value(struct device_driver *ddri, char *buf)
{
        int mode=0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        read_lock(&data->mcrtl.ctrllock);
        mode = data->mcrtl.mode;
        read_unlock(&data->mcrtl.ctrllock);
        
        if ( atomic_read(&data->trace) & MXG_CTR_TRC_FLAG ) {
                MSE_LOG("mxg2320 mode : 0x%x\n", mode);
        }
        
        return sprintf(buf, "%d\n", mode);
}


static ssize_t store_mode_value(struct device_driver *ddri, const char *buf, size_t count)
{
        int mode = 0;

        sscanf(buf, "%d", &mode);

        mxg2320_SetMode(mode);
        
        return count;
}

// temp decaliration 0707
/*
static int mxg2320_m_set_delay(u64 delay);	
static int mxg2320_m_enable(int en);
static int mxg2320_o_enable(int en);
static int mxg2320_o_set_delay(u64 delay);
*/
/* end */

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        
	/* enable temp for msensor because TP is not working 0707*/
	/*
	mxg2320_m_enable(1);
	mxg2320_o_enable(1);
	mxg2320_m_set_delay(100000000);
	mxg2320_o_set_delay(100000000);
	*/
	/* end */
	
        return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
                data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0], data->cvt.sign[1],
                data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}

static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
        int layout = 0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( 1 == sscanf(buf, "%d", &layout) ) {

                atomic_set(&data->layout, layout);
                
                if ( !hwmsen_get_convert(layout, &data->cvt) ) {
                        MSE_ERR("HWMSEN_GET_CONVERT function error!\r\n");
                } else if ( !hwmsen_get_convert(data->hw->direction, &data->cvt) ) {
                        MSE_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
                } else {
                        MSE_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
                        hwmsen_get_convert(0, &data->cvt);
                }
        } else {
                MSE_ERR("invalid format = '%s'\n", buf);
        }

        return count;
}

static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
        ssize_t res;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        
        if ( NULL == data )
        {
                MSE_ERR("mxg2320_i2c_data is null!!\n");
                return 0;
        }   

        res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&data->trace));
        return res;   
}

static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
        int trace;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( NULL == data )
        {
                MSE_ERR("mxg2320_i2c_data is null!!\n");
                return 0;
        }

        if ( 1 == sscanf(buf, "0x%x", &trace) )
                atomic_set(&data->trace, trace);
        else
                MSE_ERR("invalid content: '%s', length = %zu\n", buf, count);

        return count;
}

#ifdef MXG2320_WORK_TEST
static ssize_t store_rawdata_value(struct device_driver *ddri, const char *buf, size_t count)
{
        int en = 0;

        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        
        if ( NULL == data )
        {
                MSE_ERR("mxg2320_i2c_data is null!!\n");
                return 0;
        }
        sscanf(buf, "%d", &en);

        if ( en ) {
                MSE_ERR("schedule_delayed_work is start!!\n");
                schedule_delayed_work(&mx_work, msecs_to_jiffies(10));
        } else {
                MSE_ERR("schedule_delayed_work is cancel!!\n");
                cancel_delayed_work_sync(&mx_work);
        }
        
        return count;
}

static ssize_t show_debug_value(struct device_driver *ddri, char *buf)
{
        int i = 0, temp_delay[MX_SEN_MAX], temp_work[MX_SEN_MAX];
        int o_f, m_f, open_f, count_f, o_count;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        read_lock(&data->mcrtl.ctrllock);

        for( i = 0  ; i < MX_SEN_MAX; i++ ) {
                temp_delay[i] = data->mcrtl.delay[i];
        }
        for( i = 0  ; i < MX_SEN_MAX; i++ ) {
                temp_work[i] = data->mcrtl.active[i];
        }

        m_f = atomic_read(&data->mcrtl.m_flag);
        o_f = atomic_read(&data->mcrtl.o_flag);
        open_f = atomic_read(&data->mcrtl.open_flag);
        o_count = atomic_read(&data->mcrtl.o_count);
        count_f = atomic_read(&data->mcrtl.dev_open_count);
        
        read_unlock(&data->mcrtl.ctrllock);

        if ( atomic_read(&data->trace) & MXG_CTR_TRC_FLAG ) {
                MSE_INF("magnetic               : %d\n", temp_delay[0]);
                MSE_INF("uncalibrated magnetic  : %d\n", temp_delay[1]);
                MSE_INF("orientation            : %d\n", temp_delay[2]);
                MSE_INF("gyroscope              : %d\n", temp_delay[3]);
                MSE_INF("rotation vector        : %d\n", temp_delay[4]);
                MSE_INF("gravity                : %d\n", temp_delay[5]);
                MSE_INF("linear accelleration   : %d\n", temp_delay[6]);
        
                MSE_INF("magnetic               : %s\n", (temp_work[0] > 0) ? "enable": "disable" );
                MSE_INF("uncalibrated magnetic  : %s\n", (temp_work[1] > 0) ? "enable": "disable" );
                MSE_INF("orientation            : %s\n", (temp_work[2] > 0) ? "enable": "disable" );
                MSE_INF("gyroscope              : %s\n", (temp_work[3] > 0) ? "enable": "disable" );
                MSE_INF("rotation vector        : %s\n", (temp_work[4] > 0) ? "enable": "disable" );
                MSE_INF("gravity                : %s\n", (temp_work[5] > 0) ? "enable": "disable" );
                MSE_INF("linear accelleration   : %s\n", (temp_work[6] > 0) ? "enable": "disable" );

                MSE_INF("magnetic open flag     : %d\n", m_f);
                MSE_INF("orientation open flag  : %d\n", o_f);
                MSE_INF("open flag              : %d\n", open_f);
                MSE_INF("dev open count         : %d\n", count_f);
                MSE_INF("fusion open count      : %d\n", o_count);
        }
        
        return sprintf(buf, "%d\n",0);
}

#endif

/* Driver Attributes Macro */
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(factory,     S_IRUGO, show_factorytest_value, NULL);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(calidata,    S_IRUGO, show_calidata_value, NULL);
static DRIVER_ATTR(control,     S_IRUGO | S_IWUSR, show_control_value, store_control_value );
static DRIVER_ATTR(mode,        S_IRUGO | S_IWUSR, show_mode_value, store_mode_value );
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value );
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value );
#ifdef MXG2320_WORK_TEST
static DRIVER_ATTR(rawdata,     S_IWUSR, NULL, store_rawdata_value );
static DRIVER_ATTR(debug,       S_IRUGO, show_debug_value, NULL);
#endif

static struct driver_attribute *mxg2320_attr_list[] = {
        &driver_attr_daemon,
        &driver_attr_chipinfo,
        &driver_attr_sensordata,
        &driver_attr_posturedata,
        &driver_attr_factory,
        &driver_attr_status,
        &driver_attr_calidata,
        &driver_attr_control,
        &driver_attr_mode,
        &driver_attr_layout,
        &driver_attr_trace,
#ifdef MXG2320_WORK_TEST        
        &driver_attr_rawdata,
        &driver_attr_debug,
#endif        
};

static int mxg2320_create_attr(struct device_driver *driver) 
{
        int idx, err = 0;
        int num = (int)(sizeof(mxg2320_attr_list)/sizeof(mxg2320_attr_list[0]));
        if ( driver == NULL ) {
                return -EINVAL;
        }

        for( idx = 0; idx < num; idx++ ) {
                if ( (err = driver_create_file(driver, mxg2320_attr_list[idx])) ) {
                        MSE_ERR("driver_create_file (%s) = %d\n", mxg2320_attr_list[idx]->attr.name, err);
                        break;
                }
        }
        
        return err;
}
/*----------------------------------------------------------------------------*/
static int mxg2320_delete_attr(struct device_driver *driver)
{
        int idx ,err = 0;
        int num = (int)(sizeof(mxg2320_attr_list)/sizeof(mxg2320_attr_list[0]));

        if ( driver == NULL ) {
                return -EINVAL;
        }


        for( idx = 0; idx < num; idx++ ) {
                driver_remove_file(driver, mxg2320_attr_list[idx]);
        }

        return err;
}


static int mxg2320_open(struct inode *inode, struct file *file)
{
        int ret;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        atomic_inc(&data->mcrtl.dev_open_count);

        if(atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG)
        {
                MSE_LOG("Open device node:ist8303\n");
        }
        ret = nonseekable_open(inode, file);

        return ret;
}

static int mxg2320_release(struct inode *inode, struct file *file)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        atomic_dec(&data->mcrtl.dev_open_count);
        
        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG ) {
                MSE_LOG("Release device node:ist8303\n");
        }
        
        return 0;
}

static long mxg2320_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        void __user *argp = (void __user *)arg;

        u8 addr, value;
        int length, status, ret = 0;        
        char buf[MXG2320_BUFSIZE];
        int calibrated_data[MX_SEN_MAX * MXG2320_AXIS_DATA_MAX];
        int delay[MX_SEN_MAX];
        int posture_data[4];
        int raw_data[3];

	struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

	if ( argp == NULL ) {
                MSE_ERR("invalid ioctl argument.\n");
                return -EINVAL;
        }

        if ( _IOC_DIR(cmd) & _IOC_READ ) {
                ret = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
        } else if( _IOC_DIR(cmd) & _IOC_WRITE ) {
                ret = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
        }

        if ( ret ) {
                MSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
                return -EFAULT;
        }
        
        
        switch (cmd)
        {
        case MSENSOR_IOCTL_INIT:
                MSE_ERR("MSENSOR_IOCTL_INIT\n");
                status = mxg2320_i2c_write_byte(MXG2320_REG_SWRST, MXG2320_SOFT_RESET);
                if ( copy_to_user(argp, &status, sizeof(status)) ) {
                        MSE_ERR("copy_to_user failed.");
                        return -EFAULT;
                }
                break;

        case MSENSOR_IOCTL_SET_POSTURE:
                MSE_ERR("MSENSOR_IOCTL_SET_POSTURE\n");
                if ( copy_from_user(&posture_data, argp, sizeof(posture_data)) ) {
                        return -EFAULT;
                }

                write_lock(&data->mdata.datalock);
                data->mdata.ori.azimuth = posture_data[0];
                data->mdata.ori.pitch = posture_data[1];
                data->mdata.ori.roll = posture_data[2];
                data->mdata.ori.status= posture_data[3];
                write_unlock(&data->mdata.datalock);

                break;

        case MSENSOR_IOCTL_GET_OFLAG:
                MSE_ERR("MSENSOR_IOCTL_GET_OFLAG\n");
                status = atomic_read(&data->mcrtl.o_flag);
                if ( copy_to_user(argp, &status, sizeof(status)) ) {
                        MSE_ERR("copy_to_user failed.");
                        return -EFAULT;
                }
                break;

        case MSENSOR_IOCTL_GET_MFLAG:
                MSE_ERR("MSENSOR_IOCTL_GET_MFLAG\n");
                status = atomic_read(&data->mcrtl.m_flag);
                if ( copy_to_user(argp, &status, sizeof(status)) ) {
                        MSE_ERR("copy_to_user failed.");
                        return -EFAULT;
                }

                break;

        case MSENSOR_IOCTL_GET_OPEN_STATUS:
                MSE_ERR("MSENSOR_IOCTL_GET_OPEN_STATUS\n");
                status = mxg2320_GetOpenStatus(data);
                if ( copy_to_user(argp, &status, sizeof(status)) ) {
                        MSE_LOG("copy_to_user failed.");
                        return -EFAULT;
                }
                break;
                
        case MSENSOR_IOCTL_GET_CLOSE_STATUS:
                MSE_ERR("MSENSOR_IOCTL_GET_CLOSE_STATUS\n");
                status = mxg2320_GetCloseStatus(data);
                if ( copy_to_user(argp, &status, sizeof(status)) ) {
                        MSE_LOG("copy_to_user failed.");
                        return -EFAULT;
                }
                break; 

        case MSENSOR_IOCTL_SET_CALIDATA:
                MSE_ERR("MSENSOR_IOCTL_SET_CALIDATA\n");
                if ( copy_from_user(calibrated_data, argp, sizeof(calibrated_data)) ) {
                        return -EFAULT;
                }

                write_lock(&data->mdata.datalock);
                data->mdata.mag.x       = calibrated_data[0];
                data->mdata.mag.y       = calibrated_data[1];
                data->mdata.mag.z       = calibrated_data[2];
                data->mdata.mag.status  = calibrated_data[3];
                
                data->mdata.umag.ux    = calibrated_data[6];
                data->mdata.umag.uy    = calibrated_data[7];
                data->mdata.umag.uz    = calibrated_data[8];
                data->mdata.umag.ox    = calibrated_data[9];
                data->mdata.umag.ox    = calibrated_data[10];
                data->mdata.umag.ox    = calibrated_data[11]; 
                
                data->mdata.ori.azimuth = calibrated_data[12];
                data->mdata.ori.pitch   = calibrated_data[13];
                data->mdata.ori.roll    = calibrated_data[14];
                data->mdata.ori.status  = calibrated_data[15];
                ////printk("mxg2320 ori result: Azi=%d, pitch=%d, roll=%d, accuracy=%d.\n",data->mdata.ori.azimuth, data->mdata.ori.pitch, data->mdata.ori.roll, data->mdata.ori.status);
                
                data->mdata.gyr.x       = calibrated_data[18];
                data->mdata.gyr.y       = calibrated_data[19];
                data->mdata.gyr.z       = calibrated_data[20];
                data->mdata.gyr.status  = calibrated_data[21];

                data->mdata.rov.rx      = calibrated_data[24];
                data->mdata.rov.ry      = calibrated_data[25];
                data->mdata.rov.rz      = calibrated_data[26];
                data->mdata.rov.scalar  = calibrated_data[27];

                data->mdata.grv.x       = calibrated_data[30];
                data->mdata.grv.y       = calibrated_data[31];
                data->mdata.grv.z       = calibrated_data[32];
                data->mdata.grv.status  = calibrated_data[33];

                data->mdata.lac.x       = calibrated_data[36];
                data->mdata.lac.y       = calibrated_data[37];
                data->mdata.lac.z       = calibrated_data[38];
                write_unlock(&data->mdata.datalock);
                
                if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG ) {
                        MSE_INF("magnetic               : %d, %d, %d, %d\n", data->mdata.mag.x, data->mdata.mag.y, data->mdata.mag.z, data->mdata.mag.status);
                        MSE_INF("uncalibrated magnetic  : %d, %d, %d\n", data->mdata.umag.ux, data->mdata.umag.uy, data->mdata.umag.uz);
                        MSE_INF("orientation            : %d, %d, %d, %d\n", data->mdata.ori.azimuth, data->mdata.ori.pitch, data->mdata.ori.roll, data->mdata.ori.status);         
                        MSE_INF("gyroscope              : %d, %d, %d, %d\n", data->mdata.gyr.x, data->mdata.gyr.y, data->mdata.gyr.z, data->mdata.gyr.status);
                        MSE_INF("rotation vector        : %d, %d, %d, %d\n", data->mdata.rov.rx, data->mdata.rov.ry, data->mdata.rov.rz, data->mdata.rov.scalar);
                        MSE_INF("gravity                : %d, %d, %d, %d\n", data->mdata.grv.x, data->mdata.grv.y, data->mdata.grv.z, data->mdata.grv.status);
                        MSE_INF("linear accelleration   : %d, %d, %d\n", data->mdata.lac.x, data->mdata.lac.y, data->mdata.lac.z);
                }               
                
                break;

        case MSENSOR_IOCTL_READ_CHIPINFO:
                MSE_ERR("MSENSOR_IOCTL_READ_CHIPINFO\n");
                mxg2320_ReadChipInfo(buf, MXG2320_BUFSIZE);
                if ( copy_to_user(argp, buf, strlen(buf)+1) ) {
                        return -EFAULT;
                }
                break;

        case MSENSOR_IOCTL_SENSOR_ENABLE:
                MSE_ERR("MSENSOR_IOCTL_SENSOR_ENABLE\n");
                if ( copy_from_user(&status, argp, sizeof(status)) ) {
                        MSE_ERR("copy_from_user failed.");
                        return -EFAULT;
                } else {
                        MSE_INF( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",status);
                        if ( status ) {
                	        write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_MAGNETIC] = 1;
                                write_unlock(&data->mcrtl.ctrllock);

                		atomic_set(&data->mcrtl.m_flag, 1);
                		atomic_set(&data->mcrtl.open_flag, 1);
                	} else {
                	        write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_MAGNETIC] = 0;
                                data->mcrtl.delay[MX_SEN_MAGNETIC] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                		atomic_set(&data->mcrtl.m_flag, 0);
                		if ( atomic_read(&data->mcrtl.o_flag) == 0 ) {
                			atomic_set(&data->mcrtl.open_flag, 0);
                		}
                	}
                        wake_up(&open_wq);
                }
                
                break;
        case MSENSOR_IOCTL_SNG_MEASURE:
                MSE_ERR("MSENSOR_IOCTL_SNG_MEASURE\n");
                mxg2320_operation_mode(MXG2320_MOD_SNG_MEASURE);
                break;

        case MSENSOR_IOCTL_READ_SENSORDATA:
                MSE_ERR("MSENSOR_IOCTL_READ_SENSORDATA\n");
		mxg2320_operation_mode(MXG2320_MOD_SNG_MEASURE);
                mxg2320_get_raw_data(MXG2320_MOD_SNG_MEASURE);
                read_lock(&data->mdata.datalock);
                raw_data[0] = data->mdata.umag.ux;
                raw_data[1] = data->mdata.umag.uy;
                raw_data[2] = data->mdata.umag.uz;
                read_unlock(&data->mdata.datalock);

                if ( copy_to_user(argp, raw_data, sizeof(raw_data)) ) {
                        return -EFAULT;
                }
                break;

        case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
        case MSENSOR_IOCTL_READ_POSTUREDATA:
                MSE_ERR("MSENSOR_IOCTL_READ_FACTORY_SENSORDATA & MSENSOR_IOCTL_READ_POSTUREDATA\n");
                mxg2320_ReadPostureData(buf, MXG2320_BUFSIZE);
                if ( copy_to_user(argp, buf, strlen(buf)+1) ) {
                        return -EFAULT;
                }
                break;
        case MSENSOR_IOCTL_SELF_TEST:
                MSE_ERR("MSENSOR_IOCTL_SELF_TEST\n");
                if ( atomic_read(&data->mcrtl.open_flag) == 1 ) {
                        MSE_ERR("self-test is busy!\r\n");
                        ret = -EBUSY;
                        if ( copy_to_user(argp, &ret, sizeof(ret)) ) {
                                return -EFAULT;
                        }

                        return 0;
                }
                
                ret = mxg2320_get_raw_data(MXG2320_MOD_SELF_TEST);

                if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG ) {
                        if ( ret ) {
                                MSE_ERR("self-test is success!\r\n");
                        } else {
                                MSE_ERR("self-test is failed!\r\n");
                        }
                }
                
                if ( copy_to_user(argp, &ret, sizeof(ret)) ) {
                        return -EFAULT;
                }
                break;        
        case MSENSOR_IOCTL_READ_CALIDATA:
                MSE_ERR("MSENSOR_IOCTL_READ_CALIDATA\n");
                mxg2320_ReadCaliData(buf, MXG2320_BUFSIZE);
                if ( copy_to_user(argp, buf, strlen(buf)+1) ) {
                        return -EFAULT;
                }
                break;

        case MSENSOR_IOCTL_READ_CONTROL:                
                MSE_ERR("MSENSOR_IOCTL_READ_CONTROL\n");
                read_lock(&data->mcrtl.ctrllock);
                memcpy(delay, &data->mcrtl.delay[0], sizeof(delay));
                read_unlock(&data->mcrtl.ctrllock);
                
                if ( copy_to_user(argp, delay, sizeof(delay)) ) {
                        return -EFAULT;
                }
                break;

        case MSENSOR_IOCTL_SET_CONTROL:
                break;

        case MSENSOR_IOCTL_SET_MODE:
                MSE_ERR("MSENSOR_IOCTL_SET_MODE\n");
                if ( copy_from_user(&status, argp, sizeof(status)) ) {
                        return -EFAULT;
                }

                mxg2320_SetMode(status);

                break;
                
        case MSENSOR_IOCTL_READ:
                MSE_ERR("MSENSOR_IOCTL_READ\n");
                if ( copy_from_user(buf, argp, sizeof(buf)) ) {
                        return -EFAULT;
                }

                if ( (buf[0] < 1) || (buf[0] > (MXG2320_BUFSIZE-1)) ) {
                        MSE_ERR("invalid argument.");
                        return -EINVAL;
                }
                
                addr = buf[1];
                length = buf[0];
                if ( (ret = mxg2320_i2c_read_block(addr, &buf[0], length)) ) {
                        return -EFAULT;
                }
                
                if ( copy_to_user(argp, buf, length) ) {
                        MSE_ERR("copy_to_user failed.");
                        return -EFAULT;
                }
                break;
                
        case MSENSOR_IOCTL_WRITE:
                if ( copy_from_user(buf, argp, sizeof(buf)) ) {
                        return -EFAULT;
                }

                if ( buf[1] < 1 ) {
                        MSE_ERR("invalid argument.");
                        return -EINVAL;
                }
                
                addr = buf[1];
                value = buf[0];
                if ( (ret = mxg2320_i2c_write_byte(addr, value)) ) {
                        return -EFAULT;
                }
                break;

        default:
                return -ENOIOCTLCMD;
        }
        
        return 0;
}

#ifdef CONFIG_COMPAT
static long mxg2320_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        void __user *arg32 = compat_ptr(arg);
        
        if (!file->f_op || !file->f_op->unlocked_ioctl)
            return -ENOTTY;

        switch (cmd)
        {
        case COMPAT_MSENSOR_IOCTL_INIT:
        case COMPAT_MSENSOR_IOCTL_SET_POSTURE:
        case COMPAT_MSENSOR_IOCTL_GET_OFLAG:
        case COMPAT_MSENSOR_IOCTL_GET_MFLAG:
        case COMPAT_MSENSOR_IOCTL_GET_OPEN_STATUS:
        case COMPAT_MSENSOR_IOCTL_GET_CLOSE_STATUS:
        case COMPAT_MSENSOR_IOCTL_SET_CALIDATA:
        case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
        case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
        case COMPAT_MSENSOR_IOCTL_SNG_MEASURE:
        case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
        case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
        case COMPAT_MSENSOR_IOCTL_READ_POSTUREDATA:
        case COMPAT_MSENSOR_IOCTL_SELF_TEST:
        case COMPAT_MSENSOR_IOCTL_READ_CALIDATA:
        case COMPAT_MSENSOR_IOCTL_READ_CONTROL:                
        case COMPAT_MSENSOR_IOCTL_SET_CONTROL:
        case COMPAT_MSENSOR_IOCTL_SET_MODE:
        case COMPAT_MSENSOR_IOCTL_READ:
        case COMPAT_MSENSOR_IOCTL_WRITE:
        {
                return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)(arg32));
        }
        
        default:
                return -ENOIOCTLCMD;
        }
        
        return 0;
}

#endif

static struct file_operations mxg2320_fops = {
        .owner = THIS_MODULE,
        .open = mxg2320_open,
        .release = mxg2320_release,
        .unlocked_ioctl = mxg2320_unlocked_ioctl,
#ifdef CONFIG_COMPAT
        .compat_ioctl = mxg2320_compat_ioctl,
#endif        
};

static struct miscdevice mxg2320_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "msensor",
        .fops = &mxg2320_fops,
};

int mxg2320_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
        int err = 0;
        int value;
        hwm_sensor_data* msensor_data;
        struct mxg2320_i2c_data *data = (struct mxg2320_i2c_data *)self;

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG )
        {
                MSE_FUN();
        }

        switch (command)
        {
        case SENSOR_DELAY:
                MSE_INF("mxg2320_operate for delay\n");
                if((buff_in == NULL) || (size_in < sizeof(int)))
                {
                        MSE_ERR("Set delay parameter error!\n");
                        err = -EINVAL;
                }
                else
                {
                        value = *(int *)buff_in;
                        
                        write_lock(&data->mcrtl.ctrllock);
                        if ( value <= MXG2320_DELAY_MIN ) {
                                data->mcrtl.delay[MX_SEN_MAGNETIC] = MXG2320_DELAY_MIN;
                        } else if ( value >= MXG2320_DELAY_MAX ) {
                                data->mcrtl.delay[MX_SEN_MAGNETIC] = MXG2320_DELAY_MAX;
                        } else {
                                data->mcrtl.delay[MX_SEN_MAGNETIC] = value;
                        }
                        write_unlock(&data->mcrtl.ctrllock);
                }
                break;

        case SENSOR_ENABLE:
                MSE_INF("mxg2320_operate for enable\n");
                if ( (buff_in == NULL) || (size_in < sizeof(int)) ) {
                        MSE_ERR("Set enable parameter error!\n");
                        err = -EINVAL;
                } else {
                        value = *(int *)buff_in;
                        
                        if ( value ) {
                	        write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_MAGNETIC] = 1;
                                write_unlock(&data->mcrtl.ctrllock);

                		atomic_set(&data->mcrtl.m_flag, 1);
                		atomic_set(&data->mcrtl.open_flag, 1);
                	} else {
                	        write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_MAGNETIC] = 0;
                                data->mcrtl.delay[MX_SEN_MAGNETIC] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                		atomic_set(&data->mcrtl.m_flag, 0);
                		if ( atomic_read(&data->mcrtl.o_flag) == 0 ) {
                			atomic_set(&data->mcrtl.open_flag, 0);
                		}
                	}     
                }
                wake_up(&open_wq);
                break;

        case SENSOR_GET_DATA:
                MSE_INF("mxg2320_operate for get data\n");
                if ( (buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) ) {
                        MSE_ERR("get magnetometer data parameter error!\n");
                        err = -EINVAL;
                } else {
                        msensor_data = (hwm_sensor_data *)buff_out;
                        
                        read_lock(&data->mdata.datalock);
                        msensor_data->values[0] = data->mdata.mag.x * CONVERT_M;
                        msensor_data->values[1] = data->mdata.mag.y * CONVERT_M;
                        msensor_data->values[2] = data->mdata.mag.z * CONVERT_M;
                        msensor_data->status = data->mdata.ori.status;
                        read_unlock(&data->mdata.datalock);
                        msensor_data->value_divide = CONVERT_M_DIV;

                }
                break;
                
        default:
                break;
        }

        return err;
}

int mxg2320_orientation_operate(void* self, uint32_t command, void* buff_in,
                int size_in, void* buff_out, int size_out, int* actualout)
{
        int err = 0;
        int value;
        hwm_sensor_data* osensor_data;
        struct mxg2320_i2c_data *data = (struct mxg2320_i2c_data *)self;

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG )
        {
                MSE_FUN();
        }

        switch (command)
        {
        case SENSOR_DELAY:
                MSE_INF("mxg2320_orientation_operate for delay\n");
                if((buff_in == NULL) || (size_in < sizeof(int)))
                {
                        MSE_ERR("Set delay parameter error!\n");
                        err = -EINVAL;
                }
                else
                {
                        value = *(int *)buff_in;
                        
                        write_lock(&data->mcrtl.ctrllock);
                        if ( value <= MXG2320_DELAY_MIN ) {
                                data->mcrtl.delay[MX_SEN_ORIENTATION] = MXG2320_DELAY_MIN;
                        } else if ( value >= MXG2320_DELAY_MAX ) {
                                data->mcrtl.delay[MX_SEN_ORIENTATION] = MXG2320_DELAY_MAX;
                        } else {
                                data->mcrtl.delay[MX_SEN_ORIENTATION] = value;
                        }
                        write_unlock(&data->mcrtl.ctrllock);
                }

                break;

        case SENSOR_ENABLE:
                MSE_INF("mxg2320_orientation_operate for enable\n");
                if ( (buff_in == NULL) || (size_in < sizeof(int)) ) {
                        MSE_ERR("Set enable parameter error!\n");
                        err = -EINVAL;
                } else {
                        value = *(int *)buff_in;
                        
                        if ( (atomic_read(&data->mcrtl.o_count)) <= 0 ) {
                                if ( value ) {
                                        atomic_set(&data->mcrtl.o_flag, 1);
                                        atomic_set(&data->mcrtl.open_flag, 1);
                                        
                                        
                                }
                        } else if ( (atomic_read(&data->mcrtl.o_count)) == 1 ) {
                                if ( !value ) {
                                      atomic_set(&data->mcrtl.o_flag, 0); 
                                      if ( (atomic_read(&data->mcrtl.m_flag)) == 0 ) {
                                                atomic_set(&data->mcrtl.open_flag, 0);
                                      }
                                }
                        }
                
                        if ( value ) {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_ORIENTATION] = 1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_inc(&data->mcrtl.o_count);
                        } else {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_ORIENTATION] = 0;
                                data->mcrtl.delay[MX_SEN_ORIENTATION] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_dec(&data->mcrtl.o_count);
                                if ( (atomic_read(&data->mcrtl.o_count)) < 0 ) {
                                        atomic_set(&data->mcrtl.o_count, 0); 
                                }
                        }     
                }
                wake_up(&open_wq);

                break;

        case SENSOR_GET_DATA:
                MSE_INF("mxg2320_orientation_operate for get data\n");
                if ( (buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) ) {
                        MSE_ERR("get orientation data parameter error!\n");
                        err = -EINVAL;
                } else {
                        osensor_data = (hwm_sensor_data *)buff_out;

                        read_lock(&data->mdata.datalock);
                        osensor_data->values[0] = data->mdata.ori.azimuth;
                        osensor_data->values[1] = data->mdata.ori.pitch;
                        osensor_data->values[2] = data->mdata.ori.roll;
                        osensor_data->status = data->mdata.ori.status;
                        osensor_data->value_divide = CONVERT_FUSION_DIV;
                        read_unlock(&data->mdata.datalock);
                }

                break;
        default:
                break;
        }

        return err;
}

int mxg2320_gyroscope_operate(void* self, uint32_t command, void* buff_in,
                int size_in, void* buff_out, int size_out, int* actualout)
{
        int err = 0;
        int value;
        hwm_sensor_data* gysensor_data;
        struct mxg2320_i2c_data *data = (struct mxg2320_i2c_data *)self;

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG )
        {
                MSE_FUN();
        }

        switch (command)
        {
        case SENSOR_DELAY:
                MSE_INF("mxg2320_gyroscope_operate for delay\n");
                if((buff_in == NULL) || (size_in < sizeof(int)))
                {
                        MSE_ERR("Set delay parameter error!\n");
                        err = -EINVAL;
                }
                else
                {
                        value = *(int *)buff_in;
                        
                        write_lock(&data->mcrtl.ctrllock);
                        if ( value <= MXG2320_DELAY_MIN ) {
                                data->mcrtl.delay[MX_SEN_GYROSCOP] = MXG2320_DELAY_MIN;                

                        } else if ( value >= MXG2320_DELAY_MAX ) {
                                data->mcrtl.delay[MX_SEN_GYROSCOP] = MXG2320_DELAY_MAX;
                        } else {
                                data->mcrtl.delay[MX_SEN_GYROSCOP] = value;
                        }
                        write_unlock(&data->mcrtl.ctrllock);
                }

                break;

        case SENSOR_ENABLE:
                MSE_INF("mxg2320_gyroscope_operate for enable\n");
                if ( (buff_in == NULL) || (size_in < sizeof(int)) ) {
                        MSE_ERR("Set enable parameter error!\n");
                        err = -EINVAL;
                } else {
                        value = *(int *)buff_in;
                        
                        if ( (atomic_read(&data->mcrtl.o_count)) <= 0 ) {
                                if ( value ) {                        
                                        atomic_set(&data->mcrtl.o_flag, 1);
                                        atomic_set(&data->mcrtl.open_flag, 1);
                                        
                                        
                                }
                        } else if ( (atomic_read(&data->mcrtl.o_count)) == 1 ) {
                                if ( !value ) {
                                      atomic_set(&data->mcrtl.o_flag, 0); 
                                      if ( (atomic_read(&data->mcrtl.m_flag)) == 0 ) {
                                                atomic_set(&data->mcrtl.open_flag, 0);
                                      }
                                }
                        }
                
                        if ( value ) {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_GYROSCOP] = 1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_inc(&data->mcrtl.o_count);
                        } else {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_GYROSCOP] = 0;
                                data->mcrtl.delay[MX_SEN_GYROSCOP] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_dec(&data->mcrtl.o_count);
                                if ( (atomic_read(&data->mcrtl.o_count)) < 0 ) {
                                        atomic_set(&data->mcrtl.o_count, 0); 
                                }
                        }     
                }
                wake_up(&open_wq);

                break;

        case SENSOR_GET_DATA:
                MSE_INF("mxg2320_gyroscope_operate for get data\n");
                if ( (buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) ) {
                        MSE_ERR("get gyroscope data parameter error!\n");
                        err = -EINVAL;
                } else {
                        gysensor_data = (hwm_sensor_data *)buff_out;

                        read_lock(&data->mdata.datalock);
                        gysensor_data->values[0] = data->mdata.gyr.x;
                        gysensor_data->values[1] = data->mdata.gyr.y;
                        gysensor_data->values[2] = data->mdata.gyr.z;
                        gysensor_data->status = data->mdata.gyr.status;
                        gysensor_data->value_divide = CONVERT_FUSION_DIV;
                        read_unlock(&data->mdata.datalock);
                }

                break;
        default:
                break;
        }

        return err;
}

int mxg2320_rotation_vector_operate(void* self, uint32_t command, void* buff_in,
                int size_in, void* buff_out, int size_out, int* actualout)
{
        int err = 0;
        int value;
        hwm_sensor_data* rvsensor_data;
        struct mxg2320_i2c_data *data = (struct mxg2320_i2c_data *)self;

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG )
        {
                MSE_FUN();
        }

        switch (command)
        {
        case SENSOR_DELAY:
                MSE_INF("mxg2320_rotation_vector_operate for delay\n");
                if((buff_in == NULL) || (size_in < sizeof(int)))
                {
                        MSE_ERR("Set delay parameter error!\n");
                        err = -EINVAL;
                }
                else
                {
                        value = *(int *)buff_in;
                        
                        write_lock(&data->mcrtl.ctrllock);
                        if ( value <= MXG2320_DELAY_MIN ) {
                                data->mcrtl.delay[MX_SEN_ROTATION_VECTOR] = MXG2320_DELAY_MIN;                

                        } else if ( value >= MXG2320_DELAY_MAX ) {
                                data->mcrtl.delay[MX_SEN_ROTATION_VECTOR] = MXG2320_DELAY_MAX;
                        } else {
                                data->mcrtl.delay[MX_SEN_ROTATION_VECTOR] = value;
                        }
                        write_unlock(&data->mcrtl.ctrllock);
                }

                break;

        case SENSOR_ENABLE:
                MSE_INF("mxg2320_rotation_vector_operate for enable\n");
                if ( (buff_in == NULL) || (size_in < sizeof(int)) ) {
                        MSE_ERR("Set enable parameter error!\n");
                        err = -EINVAL;
                } else {
                        value = *(int *)buff_in;
                        
                        if ( (atomic_read(&data->mcrtl.o_count)) <= 0 ) {
                                if ( value ) {
                                        atomic_set(&data->mcrtl.o_flag, 1);
                                        atomic_set(&data->mcrtl.open_flag, 1);
                                }
                        } else if ( (atomic_read(&data->mcrtl.o_count)) == 1 ) {
                                if ( !value ) {
                                      atomic_set(&data->mcrtl.o_flag, 0); 
                                      if ( (atomic_read(&data->mcrtl.m_flag)) == 0 ) {
                                                atomic_set(&data->mcrtl.open_flag, 0);
                                      }
                                }
                        }
                
                        if ( value ) {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_ROTATION_VECTOR] = 1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_inc(&data->mcrtl.o_count);
                        } else {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_ROTATION_VECTOR] = 0;
                                data->mcrtl.delay[MX_SEN_ROTATION_VECTOR] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_dec(&data->mcrtl.o_count);
                                if ( (atomic_read(&data->mcrtl.o_count)) < 0 ) {
                                        atomic_set(&data->mcrtl.o_count, 0); 
                                }
                        }
                }
                wake_up(&open_wq);

                break;

        case SENSOR_GET_DATA:
                MSE_INF("mxg2320_rotation_vector_operate for get data\n");
                if ( (buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) ) {
                        MSE_ERR("get rotation vector data parameter error!\n");
                        err = -EINVAL;
                } else {
                        rvsensor_data = (hwm_sensor_data *)buff_out;

                        read_lock(&data->mdata.datalock);
                        rvsensor_data->values[0] = data->mdata.rov.rx;
                        rvsensor_data->values[1] = data->mdata.rov.ry;
                        rvsensor_data->values[2] = data->mdata.rov.rz;
                        rvsensor_data->status = data->mdata.rov.scalar;
                        rvsensor_data->value_divide = CONVERT_FUSION_DIV;
                        read_unlock(&data->mdata.datalock);
                }

                break;
        default:
                break;
        }

        return err;
}

int mxg2320_gravity_operate(void* self, uint32_t command, void* buff_in,
                int size_in, void* buff_out, int size_out, int* actualout)
{
        int err = 0;
        int value;
        hwm_sensor_data* gvsensor_data;
        struct mxg2320_i2c_data *data = (struct mxg2320_i2c_data *)self;

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG )
        {
                MSE_FUN();
        }

        switch (command)
        {
        case SENSOR_DELAY:
                MSE_INF("mxg2320_gravity_operate for delay\n");
                if((buff_in == NULL) || (size_in < sizeof(int)))
                {
                        MSE_ERR("Set delay parameter error!\n");
                        err = -EINVAL;
                }
                else
                {
                        value = *(int *)buff_in;
                        
                        write_lock(&data->mcrtl.ctrllock);
                        if ( value <= MXG2320_DELAY_MIN ) {
                                data->mcrtl.delay[MX_SEN_GRAVITY] = MXG2320_DELAY_MIN;

                        } else if ( value >= MXG2320_DELAY_MAX ) {
                                data->mcrtl.delay[MX_SEN_GRAVITY] = MXG2320_DELAY_MAX;
                        } else {
                                data->mcrtl.delay[MX_SEN_GRAVITY] = value;
                        }
                        write_unlock(&data->mcrtl.ctrllock);
                }

                break;

        case SENSOR_ENABLE:
                MSE_INF("mxg2320_gravity_operate for enable\n");
                if ( (buff_in == NULL) || (size_in < sizeof(int)) ) {
                        MSE_ERR("Set enable parameter error!\n");
                        err = -EINVAL;
                } else {
                        value = *(int *)buff_in;
                        
                        if ( (atomic_read(&data->mcrtl.o_count)) <= 0 ) {
                                if ( value ) {
                                        atomic_set(&data->mcrtl.o_flag, 1);
                                        atomic_set(&data->mcrtl.open_flag, 1);
                                        
                                        
                                }
                        } else if ( (atomic_read(&data->mcrtl.o_count)) == 1 ) {
                                if ( !value ) {
                                      atomic_set(&data->mcrtl.o_flag, 0); 
                                      if ( (atomic_read(&data->mcrtl.m_flag)) == 0 ) {
                                                atomic_set(&data->mcrtl.open_flag, 0);
                                      }
                                }
                        }
                
                        if ( value ) {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_GRAVITY] = 1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_inc(&data->mcrtl.o_count);
                        } else {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_GRAVITY] = 0;
                                data->mcrtl.delay[MX_SEN_GRAVITY] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_dec(&data->mcrtl.o_count);
                                if ( (atomic_read(&data->mcrtl.o_count)) < 0 ) {
                                        atomic_set(&data->mcrtl.o_count, 0); 
                                }
                        }     
                }
                wake_up(&open_wq);

                break;

        case SENSOR_GET_DATA:
                MSE_INF("mxg2320_gravity_operate for get data\n");
                if ( (buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) ) {
                        MSE_ERR("get gravity data parameter error!\n");
                        err = -EINVAL;
                } else {
                        gvsensor_data = (hwm_sensor_data *)buff_out;

                        read_lock(&data->mdata.datalock);
                        gvsensor_data->values[0] = data->mdata.grv.x;
                        gvsensor_data->values[1] = data->mdata.grv.y;
                        gvsensor_data->values[2] = data->mdata.grv.z;
                        gvsensor_data->status = data->mdata.grv.status;
                        gvsensor_data->value_divide = CONVERT_FUSION_DIV;
                        read_unlock(&data->mdata.datalock);
                }
                break;
        default:
                MSE_ERR("linear acceleration operate function no this parameter error!\n");
                break;
        }

        return err;
}

int mxg2320_linear_acceleration_operate(void* self, uint32_t command, void* buff_in,
                int size_in, void* buff_out, int size_out, int* actualout)
{
        int err = 0;
        int value;
        hwm_sensor_data* lasensor_data;
        struct mxg2320_i2c_data *data = (struct mxg2320_i2c_data *)self;

        if ( atomic_read(&data->trace) & MXG_FUNC_TRC_FLAG )
        {
                MSE_FUN();
        }

        switch (command)
        {
        case SENSOR_DELAY:
                MSE_INF("mxg2320_linear_acceleration_operate for delay\n");
                if((buff_in == NULL) || (size_in < sizeof(int)))
                {
                        MSE_ERR("Set delay parameter error!\n");
                        err = -EINVAL;
                }
                else
                {
                        value = *(int *)buff_in;
                        
                        write_lock(&data->mcrtl.ctrllock);
                        if ( value <= MXG2320_DELAY_MIN ) {
                                data->mcrtl.delay[MX_SEN_LINEAR_ACCELERATION] = MXG2320_DELAY_MIN;

                        } else if ( value >= MXG2320_DELAY_MAX ) {
                                data->mcrtl.delay[MX_SEN_LINEAR_ACCELERATION] = MXG2320_DELAY_MAX;
                        } else {
                                data->mcrtl.delay[MX_SEN_LINEAR_ACCELERATION] = value;
                        }
                        write_unlock(&data->mcrtl.ctrllock);
                }

                break;

        case SENSOR_ENABLE:
                MSE_INF("mxg2320_linear_acceleration_operate for enable\n");
                if ( (buff_in == NULL) || (size_in < sizeof(int)) ) {
                        MSE_ERR("Set enable parameter error!\n");
                        err = -EINVAL;
                } else {
                        value = *(int *)buff_in;
                        
                        if ( (atomic_read(&data->mcrtl.o_count)) <= 0 ) {
                                if ( value ) {                        
                                        atomic_set(&data->mcrtl.o_flag, 1);
                                        atomic_set(&data->mcrtl.open_flag, 1);
                                        
                                        
                                }
                        } else if ( (atomic_read(&data->mcrtl.o_count)) == 1 ) {
                                if ( !value ) {
                                      atomic_set(&data->mcrtl.o_flag, 0); 
                                      if ( (atomic_read(&data->mcrtl.m_flag)) == 0 ) {
                                                atomic_set(&data->mcrtl.open_flag, 0);
                                      }
                                }
                        }
                
                        if ( value ) {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_LINEAR_ACCELERATION] = 1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_inc(&data->mcrtl.o_count);
                        } else {
                                write_lock(&data->mcrtl.ctrllock);
                                data->mcrtl.active[MX_SEN_LINEAR_ACCELERATION] = 0;
                                data->mcrtl.delay[MX_SEN_LINEAR_ACCELERATION] = -1;
                                write_unlock(&data->mcrtl.ctrllock);
                                
                                atomic_dec(&data->mcrtl.o_count);
                                if ( (atomic_read(&data->mcrtl.o_count)) < 0 ) {
                                        atomic_set(&data->mcrtl.o_count, 0); 
                                }
                        }
                }
                wake_up(&open_wq);
                
                break;

        case SENSOR_GET_DATA:
                MSE_INF("mxg2320_linear_acceleration_operate for get data\n");
                if ( (buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) ) {
                        MSE_ERR("get linear acceleration data parameter error!\n");
                        err = -EINVAL;
                } else {
                        lasensor_data = (hwm_sensor_data *)buff_out;

                        read_lock(&data->mdata.datalock);
                        lasensor_data->values[0] = data->mdata.lac.x;
                        lasensor_data->values[1] = data->mdata.lac.y;
                        lasensor_data->values[2] = data->mdata.lac.z;
                        lasensor_data->status = data->mdata.lac.status;
                        lasensor_data->value_divide = CONVERT_FUSION_DIV;
                        read_unlock(&data->mdata.datalock);
                }
                break;
                
        default:
                MSE_ERR("linear acceleration operate function no this parameter error!\n");
                break;
        }

        return err;
}

#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
static int mxg2320_m_open_report_data(int en)
{
	return 0;
}

static int mxg2320_m_set_delay(u64 delay)
{
        int value = 0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( NULL == data ) {
                MSE_ERR("mxg2320_i2c_data is null!!\n");
                return 0;
        }
        
        value = (int)delay/1000000;
        
        write_lock(&data->mcrtl.ctrllock);
        if ( value <= MXG2320_DELAY_MIN ) {
                data->mcrtl.delay[MX_SEN_MAGNETIC] = MXG2320_DELAY_MIN;
                
        } else if ( value >= MXG2320_DELAY_MAX ) {
                data->mcrtl.delay[MX_SEN_MAGNETIC] = MXG2320_DELAY_MAX;
        } else {
                data->mcrtl.delay[MX_SEN_MAGNETIC] = value;
        }
        write_unlock(&data->mcrtl.ctrllock);
        
        return 0;
}

static int mxg2320_m_enable(int en)
{
	int value = en;
	
	struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
	
	if ( value ) {
	        write_lock(&data->mcrtl.ctrllock);
                data->mcrtl.active[MX_SEN_MAGNETIC] = 1;
                write_unlock(&data->mcrtl.ctrllock);

		atomic_set(&data->mcrtl.m_flag, 1);
		atomic_set(&data->mcrtl.open_flag, 1);
	} else {
	        write_lock(&data->mcrtl.ctrllock);
                data->mcrtl.active[MX_SEN_MAGNETIC] = 0;
                data->mcrtl.delay[MX_SEN_MAGNETIC] = -1;
                write_unlock(&data->mcrtl.ctrllock);
                
		atomic_set(&data->mcrtl.m_flag, 0);
		if ( atomic_read(&data->mcrtl.o_flag) == 0 ) {
			atomic_set(&data->mcrtl.open_flag, 0);
		}
	}
	wake_up(&open_wq);

	return 0;
}

static int mxg2320_o_open_report_data(int en)
{
	return 0;
}

static int mxg2320_o_set_delay(u64 delay)
{
        int value = 0;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( NULL == data ) {
                MSE_ERR("mxg2320_i2c_data is null!!\n");
                return 0;
        }
        
        value = (int)delay/1000000;
        
        write_lock(&data->mcrtl.ctrllock);
        if ( value <= MXG2320_DELAY_MIN ) {
                data->mcrtl.delay[MX_SEN_ORIENTATION] = MXG2320_DELAY_MIN;
                
        } else if ( value >= MXG2320_DELAY_MAX ) {
                data->mcrtl.delay[MX_SEN_ORIENTATION] = MXG2320_DELAY_MAX;
        } else {
                data->mcrtl.delay[MX_SEN_ORIENTATION] = value;
        }
        write_unlock(&data->mcrtl.ctrllock);
	
	return 0;
}

static int mxg2320_o_enable(int en)
{	
        int value = en;
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        if ( (atomic_read(&data->mcrtl.o_count)) <= 0 ) {
                if ( value ) {
                        atomic_set(&data->mcrtl.o_flag, 1);
                        atomic_set(&data->mcrtl.open_flag, 1);
                }
        } else if ( (atomic_read(&data->mcrtl.o_count)) == 1 ) {
                if ( !value ) {
                      atomic_set(&data->mcrtl.o_flag, 0); 
                      if ( (atomic_read(&data->mcrtl.m_flag)) == 0 ) {
                                atomic_set(&data->mcrtl.open_flag, 0);
                      }
                }
        }

        if ( value ) {
                write_lock(&data->mcrtl.ctrllock);
                data->mcrtl.active[MX_SEN_ORIENTATION] = 1;
                write_unlock(&data->mcrtl.ctrllock);
                
                atomic_inc(&data->mcrtl.o_count);
        } else {
                write_lock(&data->mcrtl.ctrllock);
                data->mcrtl.active[MX_SEN_ORIENTATION] = 0;
                data->mcrtl.delay[MX_SEN_ORIENTATION] = -1;
                write_unlock(&data->mcrtl.ctrllock);
                
                atomic_dec(&data->mcrtl.o_count);
                if ( (atomic_read(&data->mcrtl.o_count)) < 0 ) {
                        atomic_set(&data->mcrtl.o_count, 0); 
                }
        }

        wake_up(&open_wq);

        return 0;
}

static int mxg2320_o_get_data(int* x ,int* y,int* z, int* status)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        read_lock(&data->mdata.datalock);
        *x = data->mdata.ori.azimuth;
        *y = data->mdata.ori.pitch;
        *z = data->mdata.ori.roll;
        *status = data->mdata.ori.status;        
        read_unlock(&data->mdata.datalock);

        return 0;
}

static int mxg2320_m_get_data(int* x ,int* y,int* z, int* status)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);

        read_lock(&data->mdata.datalock);
        *x = data->mdata.mag.x * CONVERT_M;
        *y = data->mdata.mag.y * CONVERT_M;
        *z = data->mdata.mag.z * CONVERT_M;
        *status = data->mdata.mag.status;
        read_unlock(&data->mdata.datalock);

	return 0;
}

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND

static void mxg2320_early_suspend(struct early_suspend *h)
{
        struct mxg2320_i2c_data *obj = container_of(h, struct mxg2320_i2c_data, early_drv);

        if ( obj == NULL ) {
		MSE_ERR("null pointer!!\n");
		return;
	}

        if ( mxg2320_operation_mode(MXG2320_MOD_POWER_DOWN) ) {
		return;
	}
	
	mxg2320_power(obj->hw, 0);
}

static void mxg2320_late_resume(struct early_suspend *h)
{
        struct mxg2320_i2c_data *obj = container_of(h, struct mxg2320_i2c_data, early_drv);

        if ( obj == NULL ) {
                MSE_ERR("null pointer!!\n");
                return;
        }

        mxg2320_power(obj->hw, 1);

        if ( mxg2320_operation_mode(MXG2320_MOD_SNG_MEASURE) ) {
                return;
        }
}

#else

static int mxg2320_suspend(struct i2c_client *client, pm_message_t msg) 
{
        int err;
        struct mxg2320_i2c_data *obj = i2c_get_clientdata(client);

        if ( obj == NULL ) {
                MSE_ERR("null pointer!!\n");
                return;
        }
        
        if ( msg.event == PM_EVENT_SUSPEND ) {
                err = mxg2320_operation_mode(MXG2320_MOD_POWER_DOWN);
                if ( err < 0 ) {
                        MSE_ERR("write power control fail!!\n");
                        return err;
                }

                mxg2320_power(obj->hw, 0);
        }

        return 0;
}

static int mxg2320_resume(struct i2c_client *client)
{
        struct mxg2320_i2c_data *obj = i2c_get_clientdata(client);

        if ( obj == NULL ) {
                MSE_ERR("null pointer!!\n");
                return -EINVAL;
        }
        
        mxg2320_power(obj->hw, 1);

        if ( mxg2320_operation_mode(MXG2320_MOD_SNG_MEASURE) ) {
                return -EFAULT;
        }
        
    return 0;
}

#endif

#ifdef MXG2320_WORK_TEST

static void mxg_work_func(struct work_struct *work)
{
        MSE_ERR("mxg_work_func start\n");
        mxg2320_operation_mode(MXG2320_MOD_SNG_MEASURE);
        mxg2320_get_raw_data(MXG2320_MOD_SNG_MEASURE);

	schedule_delayed_work(&mx_work, msecs_to_jiffies(10));
}

#endif

static int mxg2320_load_fuse_rom(struct mxg2320_i2c_data *data)
{
	int ret, i;

#ifdef CHECK_IC_NUM_ENABLE
	ret = mxg2320_i2c_read_block(MXG2320_REG_CMPID, data->mconf.id, 2);
	if ( ret ) {
                MSE_ERR("check ic number 1st. (%d)\n", ret);
		return ret;
	}

        if( (data->mconf.id[0] != MXG2320_CMP1_ID) )
        {
  	    ret = mxg2320_i2c_read_block(0X00, data->mconf.id, 2);
	    if ( ret ) {
                MSE_ERR("check ic number 2nd. (%d)\n", ret);
		return ret;
	    }
        }

	if ( (data->mconf.id[0] == 0x48) ) {
		MSE_ERR("e-fuse write start.\n");
		for ( i = 0; i < 9; i++ ) {
            mxg2320_i2c_write_byte(mxg_address_buf[i], mxg_write_buf[i]);
        }
		msleep(3);
		mxg2320_i2c_write_byte(mxg_address_buf[9], mxg_write_buf[9]);
		MSE_ERR("e-fuse write success.\n");
	}
#endif

	ret = mxg2320_operation_mode(MXG2320_MOD_FUSE_ACCESS);
	if ( ret )
		return ret;

	ret = mxg2320_i2c_read_block(MXG2320_REG_CMPID, data->mconf.id, 2);
	if ( ret ) {
                MSE_ERR("failed to read device information. (%d)\n", ret);
		return ret;
	}

	/* temp delete 0707 old mxg2320 will not match this code*/
	if ( (data->mconf.id[0] != MXG2320_CMP1_ID) ) {
		MSE_ERR("current id(0x%02X) is not for %s.", data->mconf.id[0], MXG2320_DEV_NAME);
		return -ENXIO;
	}
	//MSE_ERR("current id(0x%02X) is not for 0x%02X.", data->mconf.id[0], MXG2320_CMP1_ID);

	ret = mxg2320_i2c_read_block(MXG2320_REG_XCOEF, data->mconf.coef, 3);
	if ( ret ) {
                MSE_ERR("failed to read sensitivity. (%d)\n", ret);
		return ret;
	}

	ret = mxg2320_operation_mode(MXG2320_MOD_POWER_DOWN);
	if ( ret )
		return ret;
	
	for ( i = 0; i < 3; i++ ) {
		MSE_INF("magmetic coef[%d] : %d\n", i, data->mconf.coef[i]);
	}
	for ( i = 0; i < 2; i++ ) {
		MSE_INF("information[%d] : 0x%02X\n", i, data->mconf.id[i]);
	}

	return 0;
}



static int mxg_hw_init(struct mxg2320_i2c_data *data)
{
	int ret;

	mt_set_gpio_mode( GPIO13 | 0x80000000,  GPIO_MODE_00);
	mt_set_gpio_pull_enable(GPIO13 | 0x80000000, GPIO_PULL_DISABLE);
    	mt_set_gpio_dir( GPIO13 | 0x80000000, GPIO_DIR_OUT );
    	mt_set_gpio_out( GPIO13 | 0x80000000, GPIO_OUT_ONE );

	ret = mxg2320_operation_mode(MXG2320_MOD_POWER_DOWN);
	if ( ret )
		return ret;

        /* loading e-fuse rom data */
	ret = mxg2320_load_fuse_rom(data);

	return ret;
}

static int mxg2320_init_device(struct mxg2320_i2c_data *data)
{
        int i, ret = 0;
        /* device instance specific init */
        data->hw = get_cust_mag_hw();
                
        if ( (ret = hwmsen_get_convert(data->hw->direction, &data->cvt)) )
        {
                MSE_ERR("invalid direction: %d\n", data->hw->direction);
                return ret;
        }

        atomic_set(&data->layout, data->hw->direction);
        atomic_set(&data->trace, 0xff);
        atomic_set(&data->mcrtl.m_flag, 0);
        atomic_set(&data->mcrtl.o_flag, 0);
        atomic_set(&data->mcrtl.o_count, 0);
        atomic_set(&data->mcrtl.open_flag, 0);
        
        rwlock_init(&data->mcrtl.ctrllock);
        rwlock_init(&data->mdata.datalock);

        init_waitqueue_head(&open_wq);

        write_lock(&data->mcrtl.ctrllock);
        data->mcrtl.mode = MXG2320_MOD_POWER_DOWN;
        for ( i = 0; i < MX_SEN_MAX; i++ ) {
                data->mcrtl.delay[i] = -1;
        }
        write_unlock(&data->mcrtl.ctrllock);

#ifdef MXG2320_WORK_TEST
        /* test */
        INIT_DELAYED_WORK(&mx_work, mxg_work_func);
#endif        

        data->client = this_client;
        i2c_set_clientdata(this_client, data);


        return ret;
}


static int mxg2320_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int err = 0;
        struct mxg2320_i2c_data *data;
#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
        struct mag_control_path ctl={0};
        struct mag_data_path mag_data={0}; 
#else
        struct hwmsen_object sobj_m, sobj_o;
#endif
        struct hwmsen_object sobj_gy, sobj_rv;
        struct hwmsen_object sobj_gv, sobj_la;

	MSE_ERR("mxg2320_i2c_probe enter\n");
        if ( !(data = kzalloc(sizeof(struct mxg2320_i2c_data), GFP_KERNEL)) )
	{
		err = -ENOMEM;
		goto exit;
	}
	
        /* Global Static Variable Setting */
        this_client = client;

        mxg2320_init_device(data);

        err = mxg_hw_init(data);
        if ( err < 0 )
		goto exit_kfree;
		
#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
        if ( (err = mxg2320_create_attr(&(mxg2320_init_info.platform_diver_addr->driver))) )
#else
        if ( (err = mxg2320_create_attr(&mxg2320_sensor_driver.driver)) )
#endif
        {
                MSE_ERR("create attribute err = %d\n", err);
                goto exit_sysfs_create_group_failed;
        }

        if ( (err = misc_register(&mxg2320_device)) )
        {
                MSE_ERR("mxg2320_device register failed\n");
                goto exit_misc_device_register_failed;
        }

#ifndef MXG2320_MTK_NEW_SENSOR_ARCH
        sobj_m.self = data;
        sobj_m.polling = 1;
        sobj_m.sensor_operate = mxg2320_operate;
        if ( err = hwmsen_attach(ID_MAGNETIC, &sobj_m) )
        {
                MSE_ERR("attach magnetic fail = %d\n", err);
                goto exit_hwmsen_attach_magnetic_failed;
        }

        sobj_o.self = data;
        sobj_o.polling = 1;
        sobj_o.sensor_operate = mxg2320_orientation_operate;
        if ( err = hwmsen_attach(ID_ORIENTATION, &sobj_o) )
        {
                MSE_ERR("attach orieation fail = %d\n", err);
                goto exit_hwmsen_attach_orientation_failed;
        }
#endif

#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
        ctl.m_enable = mxg2320_m_enable;
        ctl.m_set_delay  = mxg2320_m_set_delay;
        ctl.m_open_report_data = mxg2320_m_open_report_data;
        ctl.o_enable = mxg2320_o_enable;
        ctl.o_set_delay  = mxg2320_o_set_delay;
        ctl.o_open_report_data = mxg2320_o_open_report_data;
        ctl.is_report_input_direct = false;
        ctl.is_use_common_factory = false;
        ctl.is_support_batch = data->hw->is_batch_supported;

        err = mag_register_control_path(&ctl);
        if ( err )
        {
                MSE_ERR("register mag control path err\n");
                goto exit_register_control_path_failed;
        }

        mag_data.div_m = CONVERT_M_DIV;
        mag_data.div_o = CONVERT_O_DIV;
        mag_data.get_data_o = mxg2320_o_get_data;
	mag_data.get_data_m = mxg2320_m_get_data;

        err = mag_register_data_path(&mag_data);
        if ( err )
        {
                MSE_ERR("register data control path err\n");
                goto exit_register_data_path_failed;
        }
#endif

	/*
        sobj_gy.self = data;
        sobj_gy.polling = 1;
        sobj_gy.sensor_operate = mxg2320_gyroscope_operate;
        if ( (err = hwmsen_attach(ID_GYROSCOPE, &sobj_gy)) )
        {
                MSE_ERR("attach gyroscope fail = %d\n", err);
                goto exit_hwmsen_attach_gyroscope_failed;
        }
	
        sobj_rv.self = data;
        sobj_rv.polling = 1;
        sobj_rv.sensor_operate = mxg2320_rotation_vector_operate;
        if ( (err = hwmsen_attach(ID_ROTATION_VECTOR, &sobj_rv)) )
        {
                MSE_ERR("attach rotation vector fail = %d\n", err);
                goto exit_hwmsen_attach_rotation_vector_failed;
        }

        sobj_gv.self = data;
        sobj_gv.polling = 1;
        sobj_gv.sensor_operate = mxg2320_gravity_operate;
        if ( (err = hwmsen_attach(ID_GRAVITY, &sobj_gv)) )
        {
                MSE_ERR("attach gravity fail = %d\n", err);
                goto exit_hwmsen_attach_gravity_failed;
        }

        sobj_la.self = data;
        sobj_la.polling = 1;
        sobj_la.sensor_operate = mxg2320_linear_acceleration_operate;
        if ( (err = hwmsen_attach(ID_LINEAR_ACCELERATION, &sobj_la)) )
        {
                MSE_ERR("attach linear acceleration fail = %d\n", err);
                goto exit_hwmsen_attach_linear_acceleration_failed;
        }*/

#if CONFIG_HAS_EARLYSUSPEND
        data->early_drv.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
        data->early_drv.suspend = mxg2320_early_suspend,
        data->early_drv.resume = mxg2320_late_resume,
        register_early_suspend(&data->early_drv);
#endif
        mxg2320_init_flag = 1;
        MSE_INF("%s: OK\n", __func__);
        return 0;

exit_hwmsen_attach_linear_acceleration_failed:
        hwmsen_detach(ID_GRAVITY);
exit_hwmsen_attach_gravity_failed:
        hwmsen_detach(ID_ROTATION_VECTOR);
exit_hwmsen_attach_rotation_vector_failed:
        hwmsen_detach(ID_GYROSCOPE);
exit_hwmsen_attach_gyroscope_failed:
#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
exit_register_data_path_failed:
exit_register_control_path_failed:
#endif
#ifndef MXG2320_MTK_NEW_SENSOR_ARCH
        hwmsen_detach(ID_ORIENTATION);
exit_hwmsen_attach_orientation_failed:
        hwmsen_detach(ID_MAGNETIC);
exit_hwmsen_attach_magnetic_failed:
#endif
exit_misc_device_register_failed:

#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
        mxg2320_delete_attr(&(mxg2320_init_info.platform_diver_addr->driver));
#else
        mxg2320_delete_attr(&mxg2320_sensor_driver.driver);
#endif
exit_sysfs_create_group_failed:
exit_kfree:
	MSE_ERR("exit_kfree: mxg_hw_init failed!%d\n", err);
        kfree(data);
exit:
        MSE_ERR("err mxg2320 = %d\n", err);
        return err;

}

static int mxg2320_i2c_remove(struct i2c_client *client)
{
        int err;
#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
        if ( (err = mxg2320_delete_attr(&(mxg2320_init_info.platform_diver_addr->driver))) )
#else
        if ( (err = mxg2320_delete_attr(&mxg2320_sensor_driver.driver)) )
#endif
        {
                MSE_ERR("mxg2320_delete_attr fail: %d\n", err);
        }

        this_client = NULL;
        i2c_unregister_device(client);
        kfree(i2c_get_clientdata(client));
        misc_deregister(&mxg2320_device);

        return 0;

}

#ifdef MXG2320_MTK_NEW_SENSOR_ARCH

static int mxg2320_local_init(void)
{
        struct mag_hw *hw = get_cust_mag_hw();
        
        mxg2320_power(hw, 1);
	MSE_ERR("mxg2320_local_init enter \n");
        
        if ( i2c_add_driver(&mxg2320_i2c_driver) )
        {
                MSE_ERR("add driver error\n");
                return -1;
        }

        if ( -1 == mxg2320_init_flag )
        {       
                MSE_ERR("mxg2302 init flag error\n");
                return -1;
        }
	MSE_ERR("mxg2320_local_init exit \n");

        return 0;
}

static int mxg2320_local_uninit(void)
{
        struct mag_hw *hw = get_cust_mag_hw();
        
        MSE_FUN();
        
	mxg2320_power(hw, 0); 
	i2c_del_driver(&mxg2320_i2c_driver);

        return 0;
}

#else

static int mxg2320_probe(struct platform_device *pdev) 
{
        struct mag_hw *hw = get_cust_mag_hw();

        mxg2320_power(hw, 1);

        if(i2c_add_driver(&mxg2320_i2c_driver))
        {
                MSE_ERR("add driver error\n");
                return -1;
        } 
        return 0;

}
static int mxg2320_remove(struct platform_device *pdev)
{
        struct mxg2320_i2c_data *data = i2c_get_clientdata(this_client);
        struct mag_hw *hw = get_cust_mag_hw();

        MSE_FUN();
        
        mxg2320_power(hw, 0);
        atomic_set(&data->mcrtl.dev_open_count, 0);
        i2c_del_driver(&mxg2320_i2c_driver);

        return 0;
}

#endif

static int __init mxg2320_init(void)
{
	int ret_val = 0;
        struct mag_hw *hw = get_cust_mag_hw();
        ret_val = i2c_register_board_info(hw->i2c_num, &i2c_mxg2320, 1);
////aaaaaaaaaaaaaaaafff
#ifdef MXG2320_MTK_NEW_SENSOR_ARCH
        MSE_INF("MXG2320 init new mtk architecture... and i2c_register_board_info func ret_value:%d\n", ret_val);
        mag_driver_add(&mxg2320_init_info);
#else
        MSE_INF("MXG2320 init old mtk architecture...\n");
        if ( platform_driver_register(&mxg2320_sensor_driver) )
        {
                MSE_ERR("failed to register driver");
                return -ENODEV;
        }
#endif
        return 0;
}

static void __exit mxg2320_exit(void)
{
#ifndef MXG2320_MTK_NEW_SENSOR_ARCH
    platform_driver_unregister(&mxg_sensor_driver);
#endif
}

module_init(mxg2320_init);
module_exit(mxg2320_exit);

MODULE_AUTHOR("Magnacip");
MODULE_DESCRIPTION("MXG2320 M-Sensor driver");
MODULE_LICENSE("GPL");
