/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2014 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include "atmel_mxt_ts_letv.h"
//#include "tpd_custom_synaptics.h"
#include <cust_eint.h>
#include <linux/gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/kthread.h>
#include "cust_gpio_usage.h"

#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>
#include "cust_gpio_usage.h"
#include <mach/mt_gpio.h>
#include <mach/mt_reg_base.h>
#include <mach/mt_typedefs.h>
#include <mach/eint.h>
#include <mach/ext_wd_drv.h>
#include <linux/watchdog.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_wdt.h>
#include <mach/wd_api.h>
#include "tpd.h"
#include <pmic_drv.h>
#include <linux/stringify.h>
#include <asm/uaccess.h>
#include <mach/mt_boot.h>

#define GTP_DMA_MAX_TRANSACTION_LENGTH  255   // for DMA mode
static u8 *gpDMABuf_va = NULL;
static dma_addr_t gpDMABuf_pa = 0;
static struct mxt_platform_data mxt_platform_data;

#define CONFIG_FB_PM
#define AM(a, args...) printk("mxt %s %d " a, __func__, __LINE__, ##args);
#if defined(CONFIG_FB_PM)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
extern char* mtkfb_find_lcm_driver(void);
static int tpd_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static struct task_struct *thread = NULL;

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif

#define CONFIG_MXT_I2C_DMA
#define MAX1_WAKEUP_GESTURE_ENABLE

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"
#define FIRMWARE_VERSION    "2F.02"

static unsigned int g_Multouch;
/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	255
#define MXT_MAX_BLOCK_READ	255
//frank
//#define MXT_MAX_BLOCK_WRITE	7
//#define MXT_MAX_BLOCK_READ	7


/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9		9
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_USER_DATA_T38		38
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_ACTIVE_STYLUS_T63	63
#define MXT_OBJ_NOISE_T72               72
#define MXT_UNLOCK_GESTURE_T81		81
#define MXT_TOUCH_SEQUENCE_PROCESSOR_T93	93
#define MXT_TOUCH_MULTITOUCHSCREEN_T100		100
#define MXT_TOUCH_SELF_CAPACITANCE_CONFIG_T111	111
#define MXT_SYMBOL_GESTURE_PROCESSOR_T115	115

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

#define MXT_T6_CMD_PAGE_UP		0x01
#define MXT_T6_CMD_PAGE_DOWN		0x02
#define MXT_T6_CMD_DELTAS		0x10
#define MXT_T6_CMD_REFS			0x11
#define MXT_T6_CMD_DEVICE_ID		0x80
#define MXT_T6_CMD_TOUCH_THRESH		0xF4


/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

#define THRESHOLD_MAX 27500
#define THRESHOLD_MIN 19000

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55
#define MXT_MUTUAL_REFERENCE_MODE	0x11

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS	1

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS	(1 << 0)
#define MXT_T63_STYLUS_RELEASE	(1 << 1)
#define MXT_T63_STYLUS_MOVE		(1 << 2)
#define MXT_T63_STYLUS_SUPPRESS	(1 << 3)

#define MXT_T63_STYLUS_DETECT	(1 << 4)
#define MXT_T63_STYLUS_TIP		(1 << 5)
#define MXT_T63_STYLUS_ERASER	(1 << 6)
#define MXT_T63_STYLUS_BARREL	(1 << 7)

#define MXT_T63_STYLUS_PRESSURE_MASK	0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_TYPE_MASK	0x70
#define MXT_T100_TYPE_STYLUS	0x20

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
//#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIME		500	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	3000	/* msec */
//#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	5	/* msec */
#define MXT_CHG_DELAY	        100	/* msec */
#define MXT_POWERON_DELAY	150	/* msec */
#define MXT_CALIBRATE_DELAY	100	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#define DEBUG_MSG_MAX		200

/* UPLOAD FW */
#define MXT_FW_NAME_SIZE	17
#define MXT_CFG_OFFSET_TYPE	1
#define MXT_CFG_OFFSET_INSTANCE	3
#define MXT_CFG_OFFSET_SIZE	5
#define MXT_T100_DISABLE_MASK	0xfd
#define MXT_T100_ENABLE_MASK	0x2
#define MXT_T81_ENABLE_MASK	0x1
#define MXT_T81_DISABLE_MASK	0xfe
#define MXT_T38_INFO_SIZE	10

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
/* GESTURE */
#define MXT_GESTURE_UP			0x56
#define MXT_GESTURE_DOWN		0x55
#define MXT_GESTURE_LEFT		0x57
#define MXT_GESTURE_RIGHT		0x58
#define MXT_GESTURE_DOUBLE_CLICK_MASK	0x02

#define RIGHT    1
#define LEFT     2
#define DOWN     4
#define UP       8
#define DOUBLE   3
#endif

#define ADD_MINUS_PERSENT 30
#define TX_NUM 26
#define RX_NUM 15
#define PAGE 128
#define RAW_DATA_SIZE (TX_NUM * RX_NUM)

//amin
enum mxt_fw_update{ 
    IS_UPDATINGE, 
    HAVE_UPDATED, 
};
struct proc_file{                                                                                                                                 
    struct file_operations fop;
    unsigned int mode;
    char *name; 
};

#define procify(propname) (&proc_##propname)

#define __PROC_FILE(_name, _mode, _read, _write) {\
    .name = __stringify(_name),\
    .fop = {.read =_read, .write = _write},\
    .mode = _mode,\
}
#define PROC_FILE(_name, _mode, _read, _write) struct proc_file proc_##_name = __PROC_FILE(_name, _mode, _read, _write)
static int mxt_update_fw_and_cfg(void *args);

//frank
#define CONFIG_MXT_I2C_EXTFLAG
#define TPD_I2C_BUS		    2
#define TPD_I2C_ADDR		0x4a    //7 bits, without  r/w bit
#define TPD_POWER_SOURCE	PMIC_APP_CAP_TOUCH_VDD
#define I2C_ACCESS_NO_REG   (1 << 4)  // no reg address, directly access i2c reg 
#define I2C_ACCESS_NO_CACHE   (1 << 5)  //no dma cache need
#define I2C_ACCESS_R_REG_FIXED   (1 << 0)   //don't mov reg address if read len is too long

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct t81_configuration{
	u8 ctrl;
	u8 distlsbyte;
	u8 distmsbyte;
	u8 startxmin;
	u8 startymin;
	u8 startxsize;
	u8 startysize;
	u8 endxmin;
	u8 endymin;
	u8 endxsize;
	u8 endysize;
	u8 movlimmin;
	u8 movlimmax;
	u8 movhyst;
	u8 maxarea;
	u8 maxnumtch;
	u8 angle;
	u8 unused;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

/* For touch panel compatebility */
enum hw_pattern_type{
	old_pattern,
	new_pattern
};

struct mxt_cfg_version{
	u8 year;
	u8 month;
	u8 date;
};

/* To store the cfg version in the hardware */
struct mxt_cfg_info{
	enum hw_pattern_type type;
	u16 fw_version;
	struct mxt_cfg_version cfg_version;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct mxt_platform_data *pdata;
	enum hw_pattern_type pattern_type;
	struct mxt_cfg_info config_info;
	u8 *t38_config;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	bool debug_v2_enabled;
	u8 *debug_msg_data;
	u16 debug_msg_count;
	struct bin_attribute debug_msg_attr;
	struct mutex debug_msg_lock;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
//frank
//	u8 bootloader_addr;
#if defined(CONFIG_MXT_I2C_DMA)
	unsigned short bootloader_addr;
	void *i2c_dma_va;
	dma_addr_t i2c_dma_pa;
#else		
	u8 bootloader_addr;
#endif
	struct t7_config t7_cfg;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	u8 num_stylusids;
	unsigned long t15_keystatus;
	bool use_retrigen_workaround;
	bool use_regulator;
	struct regulator *reg_vdd;
	struct regulator *reg_avdd;
	struct regulator *reg_vdd_io;
	struct regulator *reg_vcc_i2c;
	char *fw_name;
	char *cfg_name;

	/* Cached T8 configuration */
	struct t81_configuration t81_cfg;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u16 T7_size;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u16 T38_address;
	u8 T38_size;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T63_reportid_min;
	u8 T63_reportid_max;
	u16 T81_address;
	u8 T81_size;
	u8 T81_reportid_min;
	u8 T81_reportid_max;
	u16 T100_address;
	u8 T100_reportid_min;
	u8 T100_reportid_max;

	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for config update handling */
	struct completion crc_completion;

	/* Indicates whether device is in suspend */
	bool suspended;

	/* Indicates whether device is updating configuration */
	bool updating_config;

#if defined(CONFIG_FB_PM)
	struct notifier_block fb_notif;
	struct work_struct fb_notify_work;
#endif
	u16 raw_data_16[RAW_DATA_SIZE];
	u16 raw_data_avg;
	
	/* Protect access to the T37 object buffer, add by morven */	
	struct mutex T37_buf_mutex;	
	u8 *T37_buf;	
	size_t T37_buf_size;
	struct early_suspend early_suspend;

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	u16 T93_address;
	u8 T93_reportid;
	u16 T115_address;
	u8 T115_reportid;
	bool enable_wakeup_gesture;
	bool double_click_wake_enable;
	bool down_to_up_wake_enable;
	bool up_to_down_wake_enable;
	bool right_to_left_wake_enable;
	bool left_to_right_wake_enable;
	int detected_gesture;
#endif
    bool staying;
    bool update_force;
	struct mutex bus_access_mutex;
};

struct mxt_data *mxt_i2c_data;
struct mxt_sharp{
    void *dev;
    int (*mxt_sharp_suspend)(struct device *dev);
    int (*mxt_sharp_resume)(struct device *dev);
}mxt_sharp;

static void mxt_tpd_suspend(struct early_suspend *h);
static void mxt_tpd_resume(struct early_suspend *h);
static int mxt_suspend(struct device *h);
static int mxt_resume(struct device *h);

#if defined(CONFIG_FB_PM)
static void fb_notify_resume_work(struct work_struct *work);
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#endif

static int mxt_process_messages_until_invalid(struct mxt_data *data);

static size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_TOUCH_MULTITOUCHSCREEN_T100:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
        case MXT_TOUCH_SELF_CAPACITANCE_CONFIG_T111:
	case MXT_OBJ_NOISE_T72:
		return true;
	default:
		return false;
	}
}
static inline void reinit_completion(struct completion *x)
{
	init_completion(x);
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	printk("mxt MSG: %*ph\n", data->T5_msg_size, message);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (data->debug_v2_enabled)
		return;

	mutex_lock(&data->debug_msg_lock);

	data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->debug_msg_data) {
		dev_err(&data->client->dev, "Failed to allocate buffer\n");
		return;
	}

	data->debug_v2_enabled = true;
	mutex_unlock(&data->debug_msg_lock);

	dev_info(dev, "Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;

	if (!data->debug_v2_enabled)
		return;

	dev_info(dev, "disabling message output\n");
	data->debug_v2_enabled = false;

	mutex_lock(&data->debug_msg_lock);
	kfree(data->debug_msg_data);
	data->debug_msg_data = NULL;
	data->debug_msg_count = 0;
	mutex_unlock(&data->debug_msg_lock);
	dev_info(dev, "Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;

	mutex_lock(&data->debug_msg_lock);

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return;
	}

	if (data->debug_msg_count < DEBUG_MSG_MAX) {
		memcpy(data->debug_msg_data +
		       data->debug_msg_count * data->T5_msg_size,
		       msg,
		       data->T5_msg_size);
		data->debug_msg_count++;
	} else {
		printk("Discarding %u messages\n", data->debug_msg_count);
		data->debug_msg_count = 0;
	}

	mutex_unlock(&data->debug_msg_lock);

	sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int count;
	size_t bytes_read;

	if (!data->debug_msg_data) {
		dev_err(dev, "No buffer!\n");
		return 0;
	}

	count = bytes / data->T5_msg_size;

	if (count > DEBUG_MSG_MAX)
		count = DEBUG_MSG_MAX;

	mutex_lock(&data->debug_msg_lock);

	if (count > data->debug_msg_count)
		count = data->debug_msg_count;

	bytes_read = count * data->T5_msg_size;

	memcpy(buf, data->debug_msg_data, bytes_read);
	data->debug_msg_count = 0;

	mutex_unlock(&data->debug_msg_lock);

	return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
	sysfs_bin_attr_init(&data->debug_msg_attr);
	data->debug_msg_attr.attr.name = "debug_msg";
	data->debug_msg_attr.attr.mode = 0666;
	data->debug_msg_attr.read = mxt_debug_msg_read;
	data->debug_msg_attr.write = mxt_debug_msg_write;
	data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

	if (sysfs_create_bin_file(&data->client->dev.kobj,
				  &data->debug_msg_attr) < 0) {
		dev_err(&data->client->dev, "Failed to create %s\n",
			data->debug_msg_attr.attr.name);
		return -EINVAL;
	}

	return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
	if (data->debug_msg_attr.attr.name)
		sysfs_remove_bin_file(&data->client->dev.kobj,
				      &data->debug_msg_attr);
}

//frank 
static inline unsigned long test_flag_8bit(unsigned long mask, const volatile unsigned char *addr)
{
	return ((*addr) & mask) != 0;
}

static inline unsigned long test_flag(unsigned long mask, const volatile unsigned long *addr)
{
	return ((*addr) & mask) != 0;
}

static int mxt_wait_for_completion(struct mxt_data *data,
				   struct completion *comp,
				   unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		return ret;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}
static int __mxt_write_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len, const void *val, unsigned long flag);
static int __mxt_read_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len, void *val, unsigned long flag);
#if 0
static int mxt_bootloader_read(struct mxt_data *data, u8 *val, unsigned int count)
{/*{{{*/
	int ret;
	struct i2c_msg msg;
	bool retry = false;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;
    //frank
    msg.ext_flag = 0;
    msg.timing = 100;

retry_read:
	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if (!retry) {
			retry = true;
			dev_err(&data->client->dev, "%s: i2c recv failed (%d), retry\n",
				__func__, ret);
			msleep(30);
			goto retry_read;
		} else {
			ret = ret < 0 ? ret : -EIO;
			dev_err(&data->client->dev, "%s: i2c recv failed (%d)\n",
				__func__, ret);
		}
	}

	return ret;
}/*}}}*/
static int mxt_bootloader_write(struct mxt_data *data, const u8 * const val, unsigned int count)
{/*{{{*/
	int ret;
	struct i2c_msg msg;
	bool retry = false;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;
    //frank
    msg.ext_flag = 0;
    msg.timing = 100;

write_retry:
	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		if (!retry) {
			retry = true;
			dev_err(&data->client->dev, "%s: i2c send failed (%d), retry\n",
			__func__, ret);
			msleep(30);
			goto write_retry;
		} else {
			ret = ret < 0 ? ret : -EIO;
			dev_err(&data->client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
		}
	}

	return ret;
}/*}}}*/
#endif
static int mxt_bootloader_read(struct mxt_data *data, u8 *val, unsigned int count)
{
	struct i2c_client *client = data->client;

	return __mxt_read_reg_ext(client, data->bootloader_addr, 0, count, val, I2C_ACCESS_NO_REG);
}
static int mxt_bootloader_write(struct mxt_data *data, const u8 * const val, unsigned int count)
{
	struct i2c_client *client = data->client;

	return __mxt_write_reg_ext(client, data->bootloader_addr, 0, count, val, I2C_ACCESS_NO_REG);
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = data->info ? data->info->family_id : 0;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* Chips after 1664S use different scheme */
		if (retry || family_id >= 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
#if defined(CONFIG_MXT_I2C_DMA)	
	data->bootloader_addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
#endif
    //frank dbg
    dev_err(&data->client->dev, "mxt: bootloader i2c address 0x%02x \n", bootloader);
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool retry)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;

	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return val;
		}

		dev_err(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		dev_err(dev, "Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state,
				bool wait)
{
	struct device *dev = &data->client->dev;
	u8 val;
	int ret;

recheck:
    AM("\n");
if (wait) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
        mt_eint_unmask(6);
		ret = mxt_wait_for_completion(data, &data->bl_completion,
					      MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -ERESTARTSYS better by terminating
			 * fw update process before returning to userspace
			 * by writing length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			dev_err(dev, "mxg: Update wait error %d\n", ret);
//			return ret;
		}
	}
	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

#if 0
//comment compal code
static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int ret;
	bool retry = false;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;
	xfer[0].ext_flag = 0;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].ext_flag = 0;
	xfer[1].len = len;
	xfer[1].buf = val;

retry_read:
	ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_read;
		} else {
			dev_err(&client->dev, "%s: i2c transfer failed (%d)\n",
				__func__, ret);
			return -EIO;
		}
	}

	return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int ret;
	bool retry = false;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

retry_write:
	ret = i2c_master_send(client, buf, count);
	if (ret != count) {
		if (!retry) {
			dev_dbg(&client->dev, "%s: i2c retry\n", __func__);
			msleep(MXT_WAKEUP_TIME);
			retry = true;
			goto retry_write;
		} else {
			dev_err(&client->dev, "%s: i2c send failed (%d)\n",
				__func__, ret);
			ret = -EIO;
		}
	} else {
		dev_dbg(&client->dev, "%s:successfully to send i2c message\n",
				__func__);
		ret = 0;
	}

	kfree(buf);
	return ret;
}
#endif



//frank
static int __mxt_cache_read(struct i2c_client *client,u16 addr, 
				u16 reg, u16 len, void *val, u8 *r_cache, u8 *r_cache_pa, u8 *w_cache, u8 *w_cache_pa, unsigned long flag)
{
	struct i2c_msg *msgs;
	int num;

	struct i2c_msg xfer[2];
	char buf[2];
	u16 transferred;
	int retry = 3;
	int ret;

	if (test_flag(I2C_ACCESS_NO_CACHE,&flag)) {
		w_cache = w_cache_pa = buf;
		r_cache = r_cache_pa = val;
	}

	if (test_flag(I2C_ACCESS_NO_REG,&flag)) {
		msgs = &xfer[1];
		num = 1;
	}else{
		w_cache[0] = reg & 0xff;
		w_cache[1] = (reg >> 8) & 0xff;

		msgs = &xfer[0];
		num = ARRAY_SIZE(xfer);
//        printk("amin %s %d num: %d\n", __func__, __LINE__, num);
		/* Write register */
		xfer[0].addr = addr;
		xfer[0].flags = 0;
		xfer[0].len = 2;
		xfer[0].buf = w_cache_pa;
	}

	/* Read data */
	xfer[1].addr = addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = r_cache_pa;

#if defined(CONFIG_MXT_I2C_EXTFLAG)
	xfer[1].ext_flag = xfer[0].ext_flag = client->addr & 0xff00;
	xfer[1].timing = xfer[0].timing = 100;
	dev_dbg(&client->dev, "%s: i2c transfer(r)  (addr %x extflag %x) reg %d len %d\n",
		__func__, client->addr, xfer[0].ext_flag, reg, len);
#endif

	transferred = 0;
	while(transferred < len) {
		if (!test_flag(I2C_ACCESS_NO_REG | I2C_ACCESS_R_REG_FIXED,&flag)) {
			w_cache[0] = (reg +  transferred) & 0xff;
			w_cache[1] = ((reg + transferred) >> 8) & 0xff;
		}

		if (test_flag(I2C_ACCESS_NO_CACHE,&flag))
			xfer[1].buf = r_cache_pa + transferred;
		xfer[1].len = len - transferred;
		if (xfer[1].len > MXT_MAX_BLOCK_READ)
			xfer[1].len = MXT_MAX_BLOCK_READ;
	retry_read:
		ret = i2c_transfer(client->adapter, msgs, num);
		if (ret != num) {
			if (retry) {
				dev_dbg(&client->dev, "%s: i2c transfer(r) retry, reg %d\n", __func__, reg);
				msleep(MXT_WAKEUP_TIME);
				retry--;
				goto retry_read;
			} else {
				dev_err(&client->dev, "%s: i2c transfer(r) failed (%d) reg %d len %d transferred %d\n",
					__func__, ret, reg, len, transferred);
				return -EIO;
			}
		}
		if (!test_flag(I2C_ACCESS_NO_CACHE,&flag))
			memcpy(val + transferred, r_cache, xfer[1].len);
		transferred += xfer[1].len;

#if (DBG_LEVEL > 1)
		dev_dbg(&client->dev, "[mxt] i2c transfer(r) reg %d len %d current %d transferred %d\n",
			reg, len, xfer[1].len, transferred);
		print_hex_dump(KERN_DEBUG, "[mxt] r:", DUMP_PREFIX_NONE, 16, 1,
					test_flag(I2C_ACCESS_NO_CACHE,&flag) ? xfer[1].buf : r_cache, xfer[1].len, false);
#endif
	}
	return 0;
}

//frank
static int __mxt_cache_write(struct i2c_client *client,u16 addr, 
				u16 reg, u16 len, const void *val, u8 *w_cache, u8 *w_cache_pa, unsigned long flag)
{
	struct i2c_msg xfer;
	void *buf = NULL;
	u16 transferred,extend;
	int retry = 3;
	int ret;

	if (test_flag(I2C_ACCESS_NO_REG,&flag)) {
		extend = 0;
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag))
			w_cache = w_cache_pa = (u8 *)val;
	}else {
		extend = 2;
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag)) {
			buf = kmalloc( len + extend, GFP_KERNEL);
			if (!buf)
				return -ENOMEM;
			w_cache = w_cache_pa = buf;
		}

		w_cache[0] = reg & 0xff;
		w_cache[1] = (reg >> 8) & 0xff;
	}

	/* Write register */
	xfer.addr = addr;
	xfer.flags = 0;
	xfer.buf = w_cache_pa;

#if defined(CONFIG_MXT_I2C_EXTFLAG)
	xfer.ext_flag = client->addr & 0xff00;
	xfer.timing = 100;
	dev_dbg(&client->dev, "%s: i2c transfer(w) (addr %x extflag %x) reg %d len %d\n",
		__func__, client->addr , xfer.ext_flag, reg, len);
#endif

	transferred = 0;
	while(transferred < len) {
		xfer.len = len - transferred+ extend;
		if (xfer.len> MXT_MAX_BLOCK_WRITE)
			xfer.len = MXT_MAX_BLOCK_WRITE;
	
		if (test_flag(I2C_ACCESS_NO_CACHE,&flag) &&
			test_flag(I2C_ACCESS_NO_REG,&flag))
			xfer.buf = w_cache_pa + transferred;
		else
			memcpy(w_cache + extend, val + transferred, xfer.len - extend);

		if (extend) {
			w_cache[0] = (reg +  transferred) & 0xff;
			w_cache[1] = ((reg + transferred) >> 8) & 0xff;
		}

	retry_write:
		ret = i2c_transfer(client->adapter, &xfer, 1);
		if (ret != 1) {
			if (retry) {
				dev_dbg(&client->dev, "%s: i2c transfer(w) retry, reg %d\n", __func__, reg);
				msleep(MXT_WAKEUP_TIME);
				retry--;
				goto retry_write;
			} else {
				dev_err(&client->dev, "%s: i2c transfer(w) failed (%d) reg %d len %d transferred %d\n",
					__func__, ret, reg, len, transferred);
				if (buf)
					kfree(buf);
				return -EIO;
			}
		}

		transferred += xfer.len -extend;

#if (DBG_LEVEL > 1)
		dev_dbg(&client->dev, "[mxt] i2c transfer(w) reg %d len %d current %d transferred %d\n",
			reg, len, xfer.len -extend, transferred);
		print_hex_dump(KERN_DEBUG, "[mxt] w:", DUMP_PREFIX_NONE, 16, 1,
					test_flag(I2C_ACCESS_NO_CACHE,&flag) ? xfer.buf : w_cache, xfer.len, false);
#endif
	}

	if (buf)
		kfree(buf);
	return 0;
}

//frank
static int __mxt_read_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len, void *val, unsigned long flag)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	u8 *r_cache,*r_cache_pa,*w_cache,*w_cache_pa;
	int ret;

#if defined(CONFIG_MXT_I2C_DMA)	
	r_cache = data->i2c_dma_va;
	r_cache_pa = (void *)data->i2c_dma_pa;

	w_cache = data->i2c_dma_va + PAGE_SIZE;
	w_cache_pa = (void *)data->i2c_dma_pa + PAGE_SIZE;
#else
	r_cache_pa = r_cache = NULL;
	w_cache_pa = w_cache = NULL;

	flag |= I2C_ACCESS_NO_CACHE;
#endif

	mutex_lock(&data->bus_access_mutex);
	ret = __mxt_cache_read(client, addr, reg, len, val, r_cache, r_cache_pa, w_cache, w_cache_pa, flag);
	mutex_unlock(&data->bus_access_mutex);

	return ret;
}


//frank
static int __mxt_write_reg_ext(struct i2c_client *client, u16 addr, u16 reg, u16 len,
				const void *val, unsigned long flag)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	u8 *w_cache,*w_cache_pa;
	int ret;

#if defined(CONFIG_MXT_I2C_DMA)	
	w_cache = data->i2c_dma_va + PAGE_SIZE;
	w_cache_pa = (void *)data->i2c_dma_pa + PAGE_SIZE;

#else
	w_cache_pa = w_cache = NULL;

	flag |= I2C_ACCESS_NO_CACHE;
#endif
	mutex_lock(&data->bus_access_mutex);
	ret = __mxt_cache_write(client, addr, reg, len, val, w_cache, w_cache_pa, flag);
	mutex_unlock(&data->bus_access_mutex);

	return ret;
}



//frank
static int __mxt_read_reg(struct i2c_client *client, u16 reg, u16 len,
				void *val)
{
	return __mxt_read_reg_ext(client, client->addr, reg, len, val, 0);
}

//frank
static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
				const void *val)
{
	return __mxt_write_reg_ext(client, client->addr, reg, len, val, 0);
}



static inline void mxt_config_ctrl_set(struct mxt_data *mxt, u16 addr, u8 mask)
{
	int error, ctrl;

	error = __mxt_read_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
	ctrl |= mask;
	error = __mxt_write_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
}

static inline void mxt_config_ctrl_clear(struct mxt_data *mxt, u16 addr,
					u8 mask)
{
	int error, ctrl;

	error = __mxt_read_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
	ctrl &= ~mask;
	error = __mxt_write_reg(mxt->client, addr, 1, &ctrl);
	if (error)
		dev_err(&mxt->client->dev, "%s:%d: i2c sent failed\n",
				__func__, __LINE__);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_warn(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

	complete(&data->crc_completion);

	if (crc != data->config_crc) {
		data->config_crc = crc;
		printk("T6 Config Checksum: 0x%06X\n", crc);
	}

	/* Detect reset */
	if (status & MXT_T6_STATUS_RESET)
		complete(&data->reset_completion);

	printk("mxt: process t6 message: 0x%x\n", status);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		printk("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			status == 0 ? " OK" : "",
			status & MXT_T6_STATUS_RESET ? " RESET" : "",
			status & MXT_T6_STATUS_OFL ? " OFL" : "",
			status & MXT_T6_STATUS_SIGERR ? " SIGERR" : "",
			status & MXT_T6_STATUS_CAL ? " CAL" : "",
			status & MXT_T6_STATUS_CFGERR ? " CFGERR" : "",
			status & MXT_T6_STATUS_COMSERR ? " COMSERR" : "");

	/* Save current status */
	data->t6_status = status;
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	bool button;
	int i;

	/* Active-low switch */
	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;
		button = !(message[1] & (1 << i));
		input_report_key(input, pdata->t19_keymap[i], button);
	}
}

static void mxt_input_sync(struct mxt_data *data)
{
	input_mt_report_pointer_emulation(data->input_dev,
					  data->pdata->t19_num_keys);
	input_sync(data->input_dev);
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	printk( "[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n", id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);

	input_mt_slot(input_dev, id);

	if (status & MXT_T9_DETECT) {
		/*
		 * Multiple bits may be set if the host is slow to read
		 * the status messages, indicating all the events that
		 * have happened.
		 */
		if (status & MXT_T9_RELEASE) {
			input_mt_report_slot_state(input_dev,
						   MT_TOOL_FINGER, 0);
			mxt_input_sync(data);
		}

		/* A size of zero indicates touch is from a linked T47 Stylus */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_T47_STYLUS;
			tool = MT_TOOL_PEN;
		} else {
			tool = MT_TOOL_FINGER;
		}

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}

static void mxt_proc_t81_message(struct mxt_data *data, u8 *message)
{
	int id ;
	u8 status;
	int xdelta, ydelta;
	struct input_dev *dev = data->input_dev;

	/* we don't enable this function now 141119 */
	return;

	if (message == NULL)
		return;
	id = message[0];
	status = message[1];
	xdelta = (message[3]<< 8) | message[2];
	ydelta = (message[5]<< 8) | message[4];

	if (xdelta && ydelta) {
		input_report_key(dev, KEY_POWER, 1);
		input_sync(dev);
		input_report_key(dev, KEY_POWER, 0);
		input_sync(dev);
		dev_info(&data->client->dev, "Report a power key event\n");
	}

	dev_info(&data->client->dev, "%s: id: 0x%x, status: 0x%x, xdelta: 0x%x, ydelta: 0x%x\n",
		__func__, id, status, xdelta, ydelta);

	return;
}
static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
    static char pointer[10] = {0};
    int cnt = 0;
	int id;
    int i;
	u8 status;
	int x;
	int y;
	int tool;

    if(data->staying == IS_UPDATINGE)
    {
        printk("Updating fw or cfg!\n");
        return;
    }
	id = message[0] - data->T100_reportid_min - 2;

	/* ignore SCRSTATUS events */
	if (id < 0)
		return;
    else
        pointer[id] = 1;

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	printk("[%u] status:%02X x:%u y:%u area:%02X amp:%02X vec:%02X\n",
		id,
		status,
		x, y,
		data->t100_aux_area ? message[data->t100_aux_area] : 0,
		data->t100_aux_ampl ? message[data->t100_aux_ampl] : 0,
		data->t100_aux_vect ? message[data->t100_aux_vect] : 0);

	input_mt_slot(input_dev, id);


	if (status & MXT_T100_DETECT) {
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
			tool = MT_TOOL_PEN;
		else
			tool = MT_TOOL_FINGER;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

		if (data->t100_aux_ampl)
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 message[data->t100_aux_ampl]);

		if (data->t100_aux_area) {
			if (tool == MT_TOOL_PEN)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 MXT_TOUCH_MAJOR_T47_STYLUS);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 message[data->t100_aux_area]);
		}

		if (data->t100_aux_vect)
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
					 message[data->t100_aux_vect]);
	} else {
        pointer[id] = 0;
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
    for(i = 0; i < 10; i++)
    {
        cnt += pointer[i];
    }
    g_Multouch = cnt;

	data->update_input = true;
}

static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	u8 num_keys;  //len is NUM_KEY_TYPE
	const unsigned int *keymap;

	unsigned long keystates = le32_to_cpu(msg[2]);
    if(keystates == 3)
    {
        printk("mxt invalid t15 message!!, keystates: %lu\n", keystates);
        return;
    }

	num_keys = data->pdata->num_keys[T15_T97_KEY];
	keymap = data->pdata->keymap[T15_T97_KEY];
    AM("num_keys: %d\n", num_keys);
	for (key = 0; key < 3; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

        printk("mxt t15 key: %d, curr_state: %d, new_state: %d\n", key, curr_state, new_state);
		if (!curr_state && new_state) {
			__set_bit(key, &data->t15_keystatus);

//			__set_bit(key, input_dev->keybit);
            AM("mxt keymap: %u\n", keymap[key]);
			input_event(input_dev, EV_KEY, keymap[key], 1);
            AM("mxt input_event\n");
			sync = true;
		} else if (curr_state && !new_state) {
			__clear_bit(key, &data->t15_keystatus);

//			__clear_bit(key, input_dev->keybit);
            AM("mxt data->t15_keystatus: %lu\n", data->t15_keystatus);
			input_event(input_dev, EV_KEY, keymap[key], 0);
            AM("mxt input_event\n");
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_info(dev, "T42 suppress\n");
	else
		dev_info(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	printk("T48 state %d status %02X %s%s%s%s%s\n", state, status,
		status & 0x01 ? "FREQCHG " : "",
		status & 0x02 ? "APXCHG " : "",
		status & 0x04 ? "ALGOERR " : "",
		status & 0x10 ? "STATCHG " : "",
		status & 0x20 ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	u8 id;
	u16 x, y;
	u8 pressure;

	/* stylus slots come after touch slots */
	id = data->num_touchids + (msg[0] - data->T63_reportid_min);

	if (id < 0 || id > (data->num_touchids + data->num_stylusids)) {
		dev_err(dev, "invalid stylus id %d, max slot is %d\n",
			id, data->num_stylusids);
		return;
	}

	x = msg[3] | (msg[4] << 8);
	y = msg[5] | (msg[6] << 8);
	pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

	printk(dev,
		"[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
		id,
		msg[1] & MXT_T63_STYLUS_SUPPRESS ? 'S' : '.',
		msg[1] & MXT_T63_STYLUS_MOVE     ? 'M' : '.',
		msg[1] & MXT_T63_STYLUS_RELEASE  ? 'R' : '.',
		msg[1] & MXT_T63_STYLUS_PRESS    ? 'P' : '.',
		x, y, pressure,
		msg[2] & MXT_T63_STYLUS_BARREL   ? 'B' : '.',
		msg[2] & MXT_T63_STYLUS_ERASER   ? 'E' : '.',
		msg[2] & MXT_T63_STYLUS_TIP      ? 'T' : '.',
		msg[2] & MXT_T63_STYLUS_DETECT   ? 'D' : '.');

	input_mt_slot(input_dev, id);

	if (msg[2] & MXT_T63_STYLUS_DETECT) {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
	} else {
		input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
	}

	input_report_key(input_dev, BTN_STYLUS,
			 (msg[2] & MXT_T63_STYLUS_ERASER));
	input_report_key(input_dev, BTN_STYLUS2,
			 (msg[2] & MXT_T63_STYLUS_BARREL));

	mxt_input_sync(data);
}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
static void mxt_proc_t93_messages(struct mxt_data *data, u8 *msg)
{
    AM("\n");
	struct input_dev *dev = data->input_dev;

	if ((msg[0] == data->T93_reportid) &&
		(msg[1] & MXT_GESTURE_DOUBLE_CLICK_MASK)) {
		data->detected_gesture = DOUBLE;
		input_report_key(dev, KEY_POWER, 1);
		input_sync(dev);
		input_report_key(dev, KEY_POWER, 0);
		input_sync(dev);
	}
}

static void mxt_proc_t115_messages(struct mxt_data *data, u8 *msg)
{
	int gesture_detected = 0;
	struct input_dev *dev = data->input_dev;
	if (msg[0] != data->T115_reportid)
		return;

	switch (msg[1]) {
	case MXT_GESTURE_DOWN:
		if (data->up_to_down_wake_enable) {
			data->detected_gesture = DOWN;
			gesture_detected = 1;
		}
		break;
	case MXT_GESTURE_UP:
		if (data->down_to_up_wake_enable) {
			data->detected_gesture = UP;
			gesture_detected = 1;
		}
		break;
	case MXT_GESTURE_LEFT:
		if (data->right_to_left_wake_enable) {
			data->detected_gesture = LEFT;
			gesture_detected = 1;
		}
		break;
	case MXT_GESTURE_RIGHT:
		if (data->left_to_right_wake_enable) {
			data->detected_gesture = RIGHT;
			gesture_detected = 1;
		}
		break;
	default:
		break;
	}

	if (gesture_detected) {
		input_report_key(dev, KEY_POWER, 1);
		input_sync(dev);
		input_report_key(dev, KEY_POWER, 0);
		input_sync(dev);
	}
}
#endif

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

    //printk("mxt_debug_dump:\n");
    //mxt_dump_message(data, message);
    //printk("mxt: reportid %d\n",report_id );
    //printk("mxt_debug_dump end\n");

        AM("data->T93_reportid: 0x%x, data->suspended: %x, data->double_click_wake_enable: %x\n", data->T93_reportid, data->suspended, data->double_click_wake_enable);
	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id >= data->T9_reportid_min
	    && report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id == data->T81_reportid_min) {
		mxt_proc_t81_message(data, message);
	} else if (report_id >= data->T100_reportid_min
	    && report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T63_reportid_min && report_id <= data->T63_reportid_max) {
		mxt_proc_t63_messages(data, message);
	} else if (report_id >= data->T15_reportid_min && report_id <= data->T15_reportid_max) {
		mxt_dump_message(data, message);
		mxt_proc_t15_messages(data, message);
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	} else if ((report_id == data->T115_reportid) && data->suspended) {
		mxt_proc_t115_messages(data, message);
	} else if (report_id == data->T93_reportid && data->suspended && data->double_click_wake_enable) {
		mxt_proc_t93_messages(data, message);
#endif
	} else if (!data->input_dev|| (data->suspended && !data->enable_wakeup_gesture)) {
		/*
		 * do not report events if input device is not
		 * yet registered or returning from suspend
		 */
		mxt_dump_message(data, message);
	}else {
		dump = true;
        printk("mxt: not valid reportid 0x%x\n", report_id);
	}

	if (dump) {
		//mxt_dump_message(data, message);
    }
	if (data->debug_v2_enabled)
		mxt_debug_msg_add(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}

	/* return number of messages read */
	return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 count, num_left;

    AM("mxt_process_messages_t44 proccessing\n");
	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "mxt: Failed to read T44 and T5 (%d)\n", ret);
		return IRQ_NONE;
	}

	count = data->msg_buf[0];

    printk("mxt:mxt_process_messages_t44 read T44 count: %d\n", count);

	if (count == 0) {
		printk("Interrupt triggered but zero messages\n");
        //frank 
        //mxt_process_messages_until_invalid(data);
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		printk("T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		printk("mxt:Unexpected invalid message\n");
		return IRQ_NONE;
	}
    AM("mxt_process_messages_t44 first message handled\n");

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
        printk("mxt_process_messages_t44 going to handle left 0x%x messages\n", num_left);
		ret = mxt_read_and_process_messages(data, num_left);
		if (ret < 0){
            printk("mxt_process_messages_t44 handle left messages error\n");
			goto end;
        }
		else if (ret != num_left)
			printk("mxt: Unexpected invalid message in left messages\n");
	}

end:
	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	u8 count = data->last_message_count;
	int total_handled = 0;
    int num_handled = 0;
    AM("\n");
	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* keep reading two msgs until one is invalid or reportid limit */
	do {
    AM("\n");
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
    AM("\n");
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->update_input) {
		mxt_input_sync(data);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

    
static void mxt_interrupt(void)
{
    mt_eint_mask(6);
	tpd_flag=1;
    AM("\n");
	wake_up_interruptible(&waiter);
    return;
}

static int touch_event_handler(void *pdata)
{
	struct mxt_data *data = pdata;
//	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    do{
        //handler:
        set_current_state(TASK_INTERRUPTIBLE);

        AM("handler waiting for event\n");
        wait_event_interruptible(waiter, tpd_flag != 0);
        tpd_flag = 0;
        set_current_state(TASK_RUNNING);

        //sched_setscheduler(current, SCHED_RR, &param);
        //AM("date->in_bootloader: %d\n", data->in_bootloader);
        if (data->in_bootloader) {
            /* bootloader state transition completion */
            complete(&data->bl_completion);
            //            continue;
            //    goto handler;
        }

        if (!data->object_table) {
            //printk("mxt: object_table is NULL!\n");
            //  continue;
            //  goto handler;
        }
        if(!data->in_bootloader)
        {
            if (data->T44_address) {
                mxt_process_messages_t44(data);
            } else {
                mxt_process_messages(data);
            }
            mt_eint_unmask(6);
        }
        //mt_eint_unmask(6);
    }while(!kthread_should_stop());
	return 0;
}
static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret) {
		dev_err(&data->client->dev, "%s: Reg writing failed!\n",
				__func__);
		return ret;
	}

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while (command_register != 0 && timeout_counter++ <= 100);

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "%s: Command failed!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting chip\n");

	printk("mxt: resetting chip\n");

	reinit_completion(&data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/*
	 * On failure, CRC is set to 0 and config will always be
	 * downloaded.
	 */
	data->config_crc = 0;
	reinit_completion(&data->crc_completion);

	mxt_t6_command(data, cmd, value, true);

	/*
	 * Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded.
	 */
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	//if (irq_get_trigger_type(6) & IRQF_TRIGGER_LOW)
	if(1)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_update_cfg - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_update_cfg(struct mxt_data *data, const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	struct mxt_object *object;
	int ret;
	int offset;
	int data_pos;
	int byte_offset;
	int i;
	int cfg_start_ofs;
	u32 info_crc, config_crc, calculated_crc;
	u8 *config_mem;
	size_t config_mem_size;
	unsigned int type, instance, size;
	u8 val;
	u16 reg;

	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC))) {
		dev_err(dev, "Unrecognised config file\n");
		ret = -EINVAL;
		goto release;
	}

	data_pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++) {
		ret = sscanf(cfg->data + data_pos, "%hhx%n",
			     (unsigned char *)&cfg_info + i,
			     &offset);
		if (ret != 1) {
			dev_err(dev, "Bad format\n");
			ret = -EINVAL;
			goto release;
		}

		data_pos += offset;
	}

	if (cfg_info.family_id != data->info->family_id) {
		dev_err(dev, "Family ID mismatch!\n");
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.variant_id != data->info->variant_id) {
		dev_err(dev, "Variant ID mismatch!\n");
		ret = -EINVAL;
		goto release;
	}

	/* Read CRCs */
	ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Info CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
	if (ret != 1) {
		dev_err(dev, "Bad format: failed to parse Config CRC\n");
		ret = -EINVAL;
		goto release;
	}
	data_pos += offset;

	/*
	 * The Info Block CRC is calculated over mxt_info and the object
	 * table. If it does not match then we are trying to load the
	 * configuration from a different chip or firmware version, so
	 * the configuration CRC is invalid anyway.
	 */
	if (info_crc == data->info_crc) {
		if (config_crc == 0 || data->config_crc == 0) {
			dev_info(dev, "CRC zero, attempting to apply config\n");
		} else if (config_crc == data->config_crc) {
			dev_info(dev, "Config CRC 0x%06X: OK\n",
				data->config_crc);
			ret = 0;
			goto release;
		} else {
			dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X\n",
				 data->config_crc, config_crc);
		}
	} else {
		dev_warn(dev,
			 "Warning: Info CRC error - device=0x%06X file=0x%06X\n",
			 data->info_crc, info_crc);
	}

	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +
			MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		dev_err(dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto release;
	}

	while (data_pos < cfg->size) {
		/* Read type, instance, length */
		ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
			     &type, &instance, &size, &offset);
		if (ret == 0) {
			/* EOF */
			break;
		} else if (ret != 3) {
			dev_err(dev, "Bad format: failed to parse object\n");
			ret = -EINVAL;
			goto release_mem;
		}
		data_pos += offset;

		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			for (i = 0; i < size; i++) {
				ret = sscanf(cfg->data + data_pos, "%hhx%n",
					     &val,
					     &offset);
				data_pos += offset;
			}
			continue;
		}

		if (size > mxt_obj_size(object)) {
			/*
			 * Either we are in fallback mode due to wrong
			 * config or config from a later fw version,
			 * or the file is corrupt or hand-edited.
			 */
			dev_warn(dev, "Discarding %zu byte(s) in T%u\n",
				 size - mxt_obj_size(object), type);
		} else if (mxt_obj_size(object) > size) {
			/*
			 * If firmware is upgraded, new bytes may be added to
			 * end of objects. It is generally forward compatible
			 * to zero these bytes - previous behaviour will be
			 * retained. However this does invalidate the CRC and
			 * will force fallback mode until the configuration is
			 * updated. We warn here but do nothing else - the
			 * malloc has zeroed the entire configuration.
			 */
			dev_warn(dev, "Zeroing %zu byte(s) in T%d\n",
				 mxt_obj_size(object) - size, type);
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			ret = -EINVAL;
			goto release_mem;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;

		for (i = 0; i < size; i++) {
			ret = sscanf(cfg->data + data_pos, "%hhx%n",
				     &val,
				     &offset);
			if (ret != 1) {
				dev_err(dev, "Bad format in T%d\n", type);
				ret = -EINVAL;
				goto release_mem;
			}
			data_pos += offset;

			if (i > mxt_obj_size(object))
				continue;

			byte_offset = reg + i - cfg_start_ofs;

			if ((byte_offset >= 0)
			    && (byte_offset <= config_mem_size)) {
				*(config_mem + byte_offset) = val;
			} else {
				dev_err(dev, "Bad object: reg:%d, T%d, ofs=%d\n",
					reg, object->type, byte_offset);
				ret = -EINVAL;
				goto release_mem;
			}
		}

		if (type == MXT_USER_DATA_T38 && data->t38_config) {
				memcpy(config_mem + reg - cfg_start_ofs + MXT_T38_INFO_SIZE, data->t38_config + MXT_T38_INFO_SIZE,
					data->T38_size - MXT_T38_INFO_SIZE);
		}
	}

	/* Calculate crc of the received configs (not the raw config file) */
	if (data->T7_address < cfg_start_ofs) {
		dev_err(dev, "Bad T7 address, T7addr = %x, config offset %x\n",
			data->T7_address, cfg_start_ofs);
		ret = 0;
		goto release_mem;
	}

	calculated_crc = mxt_calculate_crc(config_mem,
					   data->T7_address - cfg_start_ofs,
					   config_mem_size);

	if (config_crc > 0 && (config_crc != calculated_crc))
		dev_warn(dev, "Config CRC error, calculated=%06X, file=%06X\n",
			 calculated_crc, config_crc);

	/* Write configuration as blocks */
	byte_offset = 0;
	while (byte_offset < config_mem_size) {
		size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		ret = __mxt_write_reg(data->client,
				      cfg_start_ofs + byte_offset,
				      size, config_mem + byte_offset);
		if (ret != 0) {
			dev_err(dev, "Config write error, ret=%d\n", ret);
			goto release_mem;
		}

		byte_offset += size;
	}

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

	ret = mxt_check_retrigen(data);
	if (ret)
		goto release_mem;

	ret = mxt_soft_reset(data);
	if (ret)
		goto release_mem;

	dev_info(dev, "Config successfully updated\n");

	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

release_mem:
	kfree(config_mem);
release:
	release_firmware(cfg);
	return ret;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 48, .idle = 48 };
	struct t7_config active_mode = { .active = 255, .idle = 255 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &active_mode;
        
	error = __mxt_write_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), new_config);
	if (error)
		return error;

	printk("mxt: Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			printk("T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
			printk("T7 cfg zero after reset, overriding\n");
			data->t7_cfg.active = 20;
			data->t7_cfg.idle = 100;
			return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	}

	printk("Initialized power cfg: ACTV %d, IDLE %d\n",
		data->t7_cfg.active, data->t7_cfg.idle);
	return 0;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	mt_eint_unmask(6);

	if (data->use_retrigen_workaround) {
            AM("\n");
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	mxt_debug_msg_remove(data);
	mxt_free_input_device(data);

	data->object_table = NULL;
	data->info = NULL;

	kfree(data->raw_info_block);
	data->raw_info_block = NULL;

	kfree(data->msg_buf);
	data->msg_buf = NULL;

	/* free the t38 configuration mem */
	if (data->t38_config) {
		kfree(data->t38_config);
		data->t38_config = NULL;
	}

	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T63_reportid_min = 0;
	data->T63_reportid_max = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data,
				  struct mxt_object *object_table)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		printk( "mxt: T%u Start:%u Size:%zu Instances:%zu Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80) {
				/*
				 * On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned
				 */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			data->T7_size = object->size_minus_one + 1;
			break;
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_USER_DATA_T38:
			data->T38_address = object->start_address;
			data->T38_size = object->size_minus_one + 1;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_PROCG_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_ACTIVE_STYLUS_T63:
			/* Only handle messages from first T63 instance */
			data->T63_reportid_min = min_id;
			data->T63_reportid_max = min_id;
			data->num_stylusids = 1;
			break;
		case MXT_UNLOCK_GESTURE_T81:
			data->T81_address = object->start_address;
			data->T81_size = object->size_minus_one + 1;
			data->T81_reportid_min = min_id;
			data->T81_reportid_max = min_id;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->T100_address = object->start_address;
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
		case MXT_TOUCH_SEQUENCE_PROCESSOR_T93:
			data->T93_address = object->start_address;
			data->T93_reportid = min_id;
			break;
		case MXT_SYMBOL_GESTURE_PROCESSOR_T115:
			data->T115_address = object->start_address;
			data->T115_reportid = min_id;
			break;
#endif
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_t38_object(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	struct mxt_cfg_info *config_info = &data->config_info;
	size_t t38_size = data->T38_size;
	u8 *cfg_buf;
	u8 *info;
	int error = 0;

	if (data->t38_config) {
		kfree(data->t38_config);
		data->t38_config = NULL;
	}

	cfg_buf = kzalloc(t38_size, GFP_KERNEL);
	if (!cfg_buf) {
		dev_err(dev, "%s: Do not have enough memory\n", __func__);
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, data->T38_address, t38_size, cfg_buf);
	if (error) {
		dev_err(dev, "%s Failed to read t38 object\n", __func__);
		goto err_free_mem;
	}


	/* store the config info */
	info = (u8 *)cfg_buf;
	config_info->type = info[0];
	config_info->fw_version = info[2] << 8 | info[1];
	config_info->cfg_version.year = info[3];
	config_info->cfg_version.month = info[4];
	config_info->cfg_version.date = info[5];
	data->t38_config = info;

	printk("%s: T38 address: 0x%x\n"
		"data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
		__func__, data->T38_address, info[0], info[1],
		info[2], info[3], info[4], info[5]);

	/* store the pattern type info */
	data->pattern_type = info[0];

	return 0;
err_free_mem:
	kfree(data->t38_config);
	data->t38_config = NULL;
	return error;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size, byte_offset, read_size;
	void *id_buf, *buf;
	uint8_t num_objects;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	id_buf = kzalloc(size, GFP_KERNEL);
	if (!id_buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	error = __mxt_read_reg(client, 0, size, id_buf);
	if (error) {
        AM("\n");
		kfree(id_buf);
		return error;
	}

	/* Resize buffer to give space for rest of info block */
	num_objects = ((struct mxt_info *)id_buf)->object_num;
    AM("num_objects: %d\n", num_objects);
	size += (num_objects * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	memcpy(buf, id_buf, MXT_OBJECT_START);
	kfree(id_buf);

	/* Read rest of info block */
	byte_offset = MXT_OBJECT_START;
	while (byte_offset < size) {
		if (size - byte_offset > MXT_MAX_BLOCK_READ)
			read_size = MXT_MAX_BLOCK_READ;
		else
			read_size = size - byte_offset;

		error = __mxt_read_reg(client, byte_offset, read_size,
				buf + byte_offset);
		if (error)
			goto err_free_mem;

		byte_offset += read_size;
	}

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/*
	 * CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol (eg i2c-hid)
	 */
	if ((data->info_crc == 0) || (data->info_crc != calculated_crc)) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			calculated_crc, data->info_crc);
		error = -EIO;
		goto err_free_mem;
	}

	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;

		printk("Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id,
		 data->info->version >> 4, data->info->version & 0xf,
		 data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data, buf + MXT_OBJECT_START);
	if (error) {
		dev_err(&client->dev, "Error %d parsing object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	error = mxt_read_t38_object(data);
	if (error) {
		dev_err(&client->dev, "%s: Failed to read t38 object\n",__func__);
		return error;
	}

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	int error;
	struct device *dev = &data->client->dev;

	gpio_direction_output(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->reg_avdd);
	if (error) {
		dev_err(dev, "regulator_enable failed  reg_avdd\n");
		return;
	}

	/*
	 * MOBILEP-16222 vdd_io is not applied and it's removed from
	 * device tree.
	 *
	error = regulator_enable(data->reg_vdd_io);
	if (error) {
		dev_err(dev, "regulator_enable failed reg_vdd_io\n");
		return;
	}
	*/

	msleep(MXT_REGULATOR_DELAY);
	gpio_direction_output(data->pdata->gpio_reset, 1);
	msleep(MXT_CHG_DELAY);

retry_wait:
	reinit_completion(&data->bl_completion);
	data->in_bootloader = true;
	error = mxt_wait_for_completion(data, &data->bl_completion,
					MXT_POWERON_DELAY);
	if (error == -EINTR)
		goto retry_wait;
	data->in_bootloader = false;
}

static void mxt_regulator_restore(struct mxt_data *data)
{
	int error;
	struct device *dev = &data->client->dev;
#if 0
	gpio_direction_output(data->pdata->gpio_reset, 0);

	error = regulator_enable(data->reg_avdd);
	if (error) {
		dev_err(dev, "regulator_enable failed  reg_avdd \n");
		return;
	}

	msleep(MXT_REGULATOR_DELAY);
	gpio_direction_output(data->pdata->gpio_reset, 1);
	msleep(MXT_CHG_DELAY);
	mt_eint_unmask(6);
#else
    AM("\n");
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");
    msleep(50);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(100);
	mt_eint_unmask(6);
#endif

#if 0
retry_wait:
	reinit_completion(&data->bl_completion);
	data->in_bootloader = true;
	error = mxt_wait_for_completion(data, &data->bl_completion,
					MXT_POWERON_DELAY);
	if (error == -EINTR) {
		dev_err(dev, "retry to wait bootloader completion\n");
		goto retry_wait;
	}

	data->in_bootloader = false;
#endif
}

static void mxt_regulator_disable(struct mxt_data *data)
{
	if(data->pdata->gpio_reset)
		gpio_direction_output(data->pdata->gpio_reset, 1);

	if(data->reg_avdd)
		regulator_disable(data->reg_avdd);
}

static void mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/*
	 * According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage
	 */
//	if (!gpio_is_valid(data->pdata->gpio_reset)) {
//		printk("Must gpio_reset  GPIO to use regulator support\n");
//		goto fail;
//	}else
	{
		error = gpio_request(data->pdata->gpio_reset, "mxt_reset_gpio");
		if (error) {
			dev_err(dev, "unable to request gpio [%lu]\n",
						data->pdata->gpio_reset);
			goto fail;
		}else
			dev_info(dev, "successfully request reset_gpio %lu\n", data->pdata->gpio_reset);

		error = gpio_direction_output(data->pdata->gpio_reset, 0);
		if (error) {
			dev_err(dev,
				"unable to set direction for gpio [%lu]\n",
				data->pdata->gpio_reset);
			goto fail;
		}

	}

	/*
	 * MOBILEP-16222 vdd_io is never needed to apply and it's removed from
	 * device tree.
	 *
	data->reg_vdd_io = regulator_get(dev, "vdd_io");
	if (IS_ERR(data->reg_vdd_io)) {
		error = PTR_ERR(data->reg_vdd_io);
		dev_err(dev, "Error %d getting vdd_io regulator\n", error);
		goto fail_release;
	}
	*/

	data->reg_avdd = regulator_get(dev, "avdd");
	if (IS_ERR(data->reg_avdd)) {
		error = PTR_ERR(data->reg_avdd);
		dev_err(dev, "Error %d getting avdd regulator\n", error);
		goto fail_release;
	}

	if (regulator_count_voltages(data->reg_avdd) > 0) {
		error = regulator_set_voltage(data->reg_avdd, 3300000,
							3300000);
		if (error) {
			dev_err(dev,
				"regulator set_vtg reg_avdd failed error=%d\n", error);
			goto fail_release;
		}
	}

	data->use_regulator = true;
	mxt_regulator_enable(data);

	printk("Initialised regulators\n");
	return;

fail_release:
	regulator_put(data->reg_vdd);
fail:
	data->reg_vdd = NULL;
	data->reg_avdd = NULL;
	data->use_regulator = false;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(&range.x);
	le16_to_cpus(&range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	printk(&client->dev,
		"Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static void mxt_start(struct mxt_data *data);
static void mxt_stop(struct mxt_data *data);
static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	unsigned int mt_flags = 0;
	int i;

	error = mxt_read_t9_resolution(data);
	if (error)
		dev_warn(dev, "Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						     pdata->t19_keymap[i]);

		mt_flags |= INPUT_MT_POINTER;

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);

		input_dev->name = "Atmel maXTouch Touchpad";
	} else {
		mt_flags |= INPUT_MT_DIRECT;
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids + data->num_stylusids;
	error = input_mt_init_slots(input_dev, num_mt_slots, mt_flags);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	/* For T63 active stylus */
	if (data->T63_reportid_min) {
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
		input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
		input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
			0, MT_TOOL_MAX, 0, 0);
	}

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;
        AM("t15_num_keys: %d\n", data->pdata->t15_num_keys);
		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					     data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(&range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(&range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_info(&client->dev,
		 "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;
    int key;
	char num_keys;  //len is NUM_KEY_TYPE
	const unsigned int *keymap;

	error = mxt_read_t100_config(data);
	if (error)
		dev_err(dev, "Failed to initialize T00 resolution\n");

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (data->pdata->input_name){
		input_dev->name = data->pdata->input_name;
    }
	else{
		input_dev->name = "atmel_mxt_T100";
    }

	mutex_init(&input_dev->mutex);
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, KEY_POWER);

    //frank
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
#if 1
	num_keys = data->pdata->num_keys[T15_T97_KEY];
	keymap = data->pdata->keymap[T15_T97_KEY];
    AM("num_keys: %d\n", num_keys);
    for (key = 0; key < num_keys; key++) {
        printk("mxt T15 key press: %u\n", key);

        input_set_capability(input_dev, EV_KEY, keymap[key]);
        AM("mxt keymap[%d]: %d\n", key, keymap[key]);
    }
#endif
	input_set_capability(input_dev, EV_KEY, KEY_POWER);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_PRESSURE,
				     0, 255, 0, 0);

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids,
				    INPUT_MT_DIRECT);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	//input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	if (data->t100_aux_area)
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				     0, MXT_MAX_AREA, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				     0, 255, 0, 0);

	if (data->t100_aux_vect)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg);

/* Delete config cb function
static void mxt_config_cb(const struct firmware *cfg, void *ctx)
{
	mxt_configure_objects(ctx, cfg);
}
*/

static int strtobyte(const char *data, u8 *value)
{
	char str[3];

	str[0] = data[0];
	str[1] = data[1];
	str[2] = '\0';

	return kstrtou8(str,16, value);
}

static size_t mxt_convert_text_to_binary(u8 *buffer, size_t len)
{
	int ret;
	int i;
	int j = 0;

	for (i = 0; i < len; i+=2) {
		ret = strtobyte(&buffer[i], &buffer[j]);
		if (ret)
			return -EINVAL;
		j++;
	}

	return (size_t)j;
}

static int mxt_update_fw(struct mxt_data * data);
static int mxt_parse_cfg_and_load(struct mxt_data *data,
	struct mxt_config_info *info, bool force);

static int mxt_register_input_device(struct mxt_data *data);
static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	bool alt_bootloader_addr = false;
	bool retry = false;

	mt_eint_mask(6);
retry_info:
	error = mxt_read_info_block(data);
	if (error) {
retry_bootloader:
		error = mxt_probe_bootloader(data, alt_bootloader_addr);
		if (error) {
			if (alt_bootloader_addr) {
				/* Chip is not in appmode or bootloader mode */
				return error;
			}

			dev_info(&client->dev, "Trying alternate bootloader address\n");
			alt_bootloader_addr = true;
			goto retry_bootloader;
		} else {
			if (retry) {
				dev_err(&client->dev, "mxt: Could not recover from bootloader mode, try to flash a firmware image to TP anyway\n");
				/*
				 * We can reflash from this state, so do not
				 * abort init
				 */
				data->in_bootloader = true;

				/* Flash a new firmware anyway */
				error = mxt_update_fw(data);
				if (error){
					dev_err(&client->dev, "Failed to update fw\n");
					return error;
				}

				/* update the corresponding config data */
				error = mxt_parse_cfg_and_load(data,&data->pdata->info, true);
				if (error) {
					dev_err(&client->dev, "Failed to update the corresponding config data\n");
					return error;
				}

				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
			retry = true;
			goto retry_info;
		}
	}

//frank added
#if 0
    AM("board going force a reset after gpio init\n");
	mxt_soft_reset(data);
#endif

	/* Enable general touch event reporting */
//	mxt_config_ctrl_set(data, data->T100_address, 0x02);
//  mxt_config_ctrl_clear(data, data->T100_address, 0x02);

	error = mxt_check_retrigen(data);
	if (error)
		goto err_free_object_table;

	error = mxt_register_input_device(data);
	if (error)
		dev_err(&data->client->dev, "Failed to register input device\n");

	error = mxt_acquire_irq(data);
	if (error)
		goto err_free_object_table;

	error = mxt_debug_msg_init(data);
	if (error)
		goto err_free_object_table;

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

static int mxt_register_input_device(struct mxt_data *data)
{
	int ret = 0;
	struct device *dev = &data->client->dev;
	if (!data->T9_reportid_min && !data->T100_reportid_min) {
		dev_err(dev, "%s, invalid parameters\n", __func__);
		return -EINVAL;
	}
	mxt_free_input_device(data);
	if (data->T9_reportid_min) {
		ret = mxt_initialize_t9_input_device(data);
		if (ret) {
			dev_err(dev, "Failed to register t9 input device\n");
			return ret;
		}
	} else if (data->T100_reportid_min) {
		ret = mxt_initialize_t100_input_device(data);
		if (ret) {
			dev_err(dev, "Failed to register t100 input device\n");
			return ret;
		}
	}else
		dev_err(dev, "Failed to find T9 or T100 object\n");

	return ret;
}

static int mxt_configure_objects(struct mxt_data *data,
				 const struct firmware *cfg)
{
	struct device *dev = &data->client->dev;
	int error;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(dev, "Failed to initialize power cfg\n");
		goto err_free_object_table;
	}

	if (cfg) {
		error = mxt_update_cfg(data, cfg);
		if (error)
			dev_warn(dev, "Error %d updating config\n", error);
	}

	error = mxt_register_input_device(data);
	if (error) {
		dev_err(dev, "Failed to register input device\n");
		return error;
	}

	return 0;

err_free_object_table:
	mxt_free_object_table(data);
	return error;
}

/* Configuration crc check sum is returned as hex xxxxxx */
static ssize_t mxt_config_csum_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%06x\n", data->config_crc);
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%02X.%02X\n",
			 data->info->version, data->info->build);
}
static ssize_t mxt_fw_version_check_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    const char *fw_name = data->pdata->fw_version;
    struct mxt_info *info = data->info;
    struct mxt_config_info *config_info = &data->pdata->info;
    struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;
    int cnt = 0;
    unsigned int version, build;

    while(data->staying == IS_UPDATINGE)
    {
        msleep(1000);
    }
    if(sscanf(fw_name, "%2x%2x", &version, &build) != 2)
    {
	    return scnprintf(buf, PAGE_SIZE, "Error\n");
    }

    if (info->version != version || info->build != build) {
        cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "Image FW: %X.%X, IC FW: %X.%X\n", version, build,
                info->version, info->build);
    }

    if(cfg_version->year != config_info->config_year ||
            cfg_version->month != config_info->config_month ||
            cfg_version->date != config_info->config_date)
    {
        cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "IC Config: %d.%d.%d, Image Config: %d.%d.%d",
                cfg_version->year, cfg_version->month, cfg_version->date,
                config_info->config_year, config_info->config_month, config_info->config_date);
    }
    if(cnt == 0)
        cnt += snprintf(buf + cnt, PAGE_SIZE - cnt, "Pass\n");
    return cnt;
}
/* Firmware Config Version is returned as YearMonthDay */
static ssize_t mxt_cfg_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;
	return scnprintf(buf, PAGE_SIZE, "%02d%02d%02d\n",
			 cfg_version->year,
			 cfg_version->month,
			 cfg_version->date);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/*
	 * To convert file try:
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw
	 */
	dev_err(dev, "Firmware file isn't in binary format\n");

	return -EINVAL;
}

static void mtk_kick_wdt(void)
{
    
    mtk_wdt_restart(WD_TYPE_NORMAL);
    mtk_wdt_restart(WD_TYPE_NOLOCK);

}
static int mxt_load_fw(struct device *dev)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;
	size_t len = 0;
	u8 *buffer  = NULL;
	ret = request_firmware(&fw, data->fw_name, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", data->fw_name);
		return ret;
	}
	buffer = kmalloc(fw->size, GFP_KERNEL);
	if (!buffer) {
		dev_err(dev, "unable to allocate the memory for fw buffer\n");
		return -ENOMEM;
	}
	memcpy(buffer, fw->data, fw->size);
	len = fw->size;

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret) {
		dev_info(dev, "converted to binary image\n");
		len = mxt_convert_text_to_binary(buffer, len);
		if (len < 0)
			goto release_firmware;
	}

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);
        AM("\n");
		mt_eint_unmask(6);
		data->suspended = false;
	}

    if (!data->in_bootloader) {
        /* Change to the bootloader mode */
        data->in_bootloader = true;

        ret = mxt_t6_command(data, MXT_COMMAND_RESET,
                MXT_BOOT_VALUE, false);
        if (ret)
            goto release_firmware;

        msleep(MXT_RESET_TIME);

        /* Do not need to scan since we know family ID */
        ret = mxt_lookup_bootloader_address(data, 0);
        if (ret)
            goto release_firmware;
    } else {
		mt_eint_unmask(6);
	}

	mxt_free_object_table(data);
	reinit_completion(&data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
	if (ret) {
		/* Bootloader may still be unlocked from previous attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, false);
    AM("\n");
		if (ret)
			goto mt_eint_mask;
	} else {
		printk("Unlocking bootloader\n");

		msleep(100);

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		msleep(MXT_RESET_TIME);
		if (ret)
			goto mt_eint_mask;
        AM("\n");
	}

	while (pos < len) {
        
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, true);
		if (ret)
			goto mt_eint_mask;

		frame_size = ((*(buffer + pos) << 8) | *(buffer + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, buffer + pos, frame_size);
		if (ret)
			goto mt_eint_mask;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, true);
		if (ret) {
			retry++;
            AM("retry: %d\n", retry);
			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto mt_eint_mask;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		//if (frame % 50 == 0)
		if (frame % 10 == 0)
        {
            mtk_kick_wdt();
            printk("mxt: sent %d frames, %d/%zd bytes\n",
                    frame, pos, fw->size);
        }
    }
    mt_eint_unmask(6);
	/* Wait for flash. */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		goto mt_eint_mask;

	printk("mxt: sent %d frames, %d bytes\n", frame, pos);
	printk("mxt: update fw successfully\n");

	/*
	 * Wait for device to reset. Some bootloader versions do not assert
	 * the CHG line after bootloading has finished, so ignore potential
	 * errors.
	 */
//	mxt_wait_for_completion(data, &data->bl_completion, MXT_FW_RESET_TIME);

	data->in_bootloader = false;

mt_eint_mask:
	mt_eint_mask(6);
release_firmware:
	release_firmware(fw);
	if (buffer)
		kfree(buffer);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		msleep(200);

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return count;
}

static int __mxt_update_fw(struct device *dev,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;

	error = mxt_update_file_name(dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(dev);
	if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		return error;
	} else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		msleep(200);

		error = mxt_initialize(data);
		if (error)
			return error;
	}

	return 0;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	const struct firmware *cfg;
	int ret;

	if (data->in_bootloader) {
		dev_err(dev, "Not in appmode\n");
		return -EINVAL;
	}

	ret = mxt_update_file_name(dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	ret = request_firmware(&cfg, data->cfg_name, dev);
	if (ret < 0) {
		dev_err(dev, "Failure to request config file %s\n",
			data->cfg_name);
		ret = -ENOENT;
		goto out;
	}

	data->updating_config = true;

	mxt_free_input_device(data);

	if (data->suspended) {
		if (data->use_regulator) {
			mt_eint_unmask(6);
			mxt_regulator_enable(data);
		} else {
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
			mxt_acquire_irq(data);
		}

		data->suspended = false;
	}

	ret = mxt_configure_objects(data, cfg);
	if (ret)
		goto out;

	ret = count;
out:
	data->updating_config = false;
	return ret;
}

static ssize_t mxt_update_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret;
    data->update_force = true;
    ret = mxt_update_fw_and_cfg(data);
    if (ret) {
        return snprintf(buf, PAGE_SIZE, "Update Failed\n");
    }
    else{
        return snprintf(buf, PAGE_SIZE, "Update Pass\n");
    }
}


static ssize_t mxt_reset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int ret = 1;
	unsigned int reset;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;

	if (reset != 1)
		return -EINVAL;

	msleep(MXT_CALIBRATE_DELAY);
	/* Recalibrate to avoid touch panel chaos */
	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);

	dev_err(dev, "%s: after calibrate\n", __func__);

	/*
	ret = mxt_soft_reset(data);
	if (ret)
		dev_err(dev, "%s, reset failed\n",__func__);
	*/

	return ret;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	char c;

	c = data->debug_enabled ? '1' : '0';
	return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		if (i == 1)
			mxt_debug_msg_enable(data);
		else
			mxt_debug_msg_disable(data);

		return count;
	} else {
		printk("debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		printk("%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		printk("debug_enabled write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_sys_suspend_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct mxt_object *t81_obj= mxt_get_object(data,MXT_UNLOCK_GESTURE_T81);
	int error = 0;

	if (!t81_obj) {
		dev_err(&client->dev, "There is no t81 object\n");
		return 0;
	}

	error = __mxt_read_reg(client,t81_obj->start_address,
		t81_obj->size_minus_one + 1,&data->t81_cfg);
	if (error)
		dev_err(&client->dev, "%s: i2c_sent  failed\n",__func__);

	return scnprintf(buf, PAGE_SIZE,"0x%x\n", data->t81_cfg.ctrl);
}

static ssize_t mxt_sys_suspend_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	struct mxt_object *t81_obj = mxt_get_object(data, MXT_UNLOCK_GESTURE_T81);
	struct mxt_object *t100_obj = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	u8 ctrl = 0, mask1 = 0xfe, mask2 = 0x1;
	u8 mask3 = 0xfd, mask4 = 0x2;
	u8 error = 0, value = 0;
	u8 *cmd = NULL;

	if (!t81_obj) {
		dev_err(&client->dev, "There is no object\n");
		return 0;
	}

	cmd = kmalloc(count + 1, GFP_KERNEL);
	if (cmd == NULL)
		goto release_error;

	memcpy(cmd, buf, count);

	if (cmd[count-1] == '\n')
		cmd[count-1] = '\0';
	else
		cmd[count] = '\0';

	if (!strcmp(cmd, "true")) {
		error = __mxt_read_reg(client,t100_obj->start_address, 1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		value = ctrl & mask3;

		dev_info(dev, "%s: t100 configuration write: 0x%x\n",__func__, value);

		error = __mxt_write_reg(client,t100_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		error = __mxt_read_reg(client, t81_obj->start_address, 1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);
		value = ctrl | mask2;
		dev_info(&client->dev, "%s: t81_configuration write:0x%x\n",__func__,value);

		error = __mxt_write_reg(client,t81_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);
	} else if (!strcmp(cmd, "false")) {
		error = __mxt_read_reg(client,t81_obj->start_address,1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		value = ctrl & mask1;
		dev_info(&client->dev, "%s: else t81 configuration write 0x%x\n",__func__,value);

		error = __mxt_write_reg(client,t81_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		error = __mxt_read_reg(client,t100_obj->start_address, 1, &ctrl);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);
		value = ctrl | mask4;
		dev_err(&client->dev, "%s: else t100 configuration write: 0x%x\n",__func__, value);

		error = __mxt_write_reg(client,t100_obj->start_address, 1,&value);
		if (error)
			dev_err(&client->dev, "%s: i2c sent failed\n", __func__);

		}
release_error:
	return count;
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

//modified by morven

/*
 * Helper function for performing a T6 diagnostic command
 */
static int mxt_T6_diag_cmd(struct mxt_data *data, struct mxt_object *T6,
			   u8 cmd)
{
	int ret;
	u16 addr = T6->start_address + MXT_COMMAND_DIAGNOSTIC;

	ret = mxt_write_reg(data->client, addr, cmd);
	if (ret)
		return ret;

	/*
	 * Poll T6.diag until it returns 0x00, which indicates command has
	 * completed.
	 */
	while (cmd != 0) {
		ret = __mxt_read_reg(data->client, addr, 1, &cmd);
		if (ret)
			return ret;
	}
	return 0;
}

//modified by morven
void read_raw_data(struct mxt_data *data, u8 mode)
{	
	struct mxt_object *T6, *T37;
	u8 *obuf;
	ssize_t ret = 0;
	size_t i,j;
	u8 lsb, msb;
	size_t T37_buf_size, num_pages;
	size_t pos;

	if (!data || !data->object_table)
		return -ENODEV;

	T6 = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	T37 = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!T6 || mxt_obj_size(T6) < 6 || !T37 || mxt_obj_size(T37) < 3) {
		dev_err(&data->client->dev, "Invalid T6 or T37 object\n");
		return -ENODEV;
	}

	/* Something has gone wrong if T37_buf is already allocated */
	if (data->T37_buf)
		return -EINVAL;

	T37_buf_size = data->info->matrix_xsize * data->info->matrix_ysize * sizeof(__le16);
	data->T37_buf_size = T37_buf_size;
	data->T37_buf = kmalloc(data->T37_buf_size, GFP_KERNEL);
	if (!data->T37_buf)
		return -ENOMEM;

	/* Temporary buffer used to fetch one T37 page */
	obuf = kmalloc(mxt_obj_size(T37), GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

//	disable_irq(data->irq);
    mt_eint_unmask(6);
	num_pages = DIV_ROUND_UP(T37_buf_size, mxt_obj_size(T37) - 2);
	pos = 0;
	for (i = 0; i < num_pages; i++) {
		u8 cmd;
		size_t chunk_len;

		/* For first page, send mode as cmd, otherwise PageUp */
		cmd = (i == 0) ? mode : MXT_T6_CMD_PAGE_UP;
		ret = mxt_T6_diag_cmd(data, T6, cmd);
		if (ret)
			goto err_free_T37_buf;

		ret = __mxt_read_reg(data->client, T37->start_address,
				mxt_obj_size(T37), obuf);
		if (ret)
			goto err_free_T37_buf;

		/* Verify first two bytes are current mode and page # */
		if (obuf[0] != mode) {
			dev_err(&data->client->dev,
				"Unexpected mode (%u != %u)\n", obuf[0], mode);
			ret = -EIO;
			goto err_free_T37_buf;
		}

		if (obuf[1] != i) {
			dev_err(&data->client->dev,
				"Unexpected page (%u != %zu)\n", obuf[1], i);
			ret = -EIO;
			goto err_free_T37_buf;
		}

		/*
		 * Copy the data portion of the page, or however many bytes are
		 * left, whichever is less.
		 */
		chunk_len = min(mxt_obj_size(T37) - 2, T37_buf_size - pos);
		memcpy(&data->T37_buf[pos], &obuf[2], chunk_len);
		pos += chunk_len;
	}
	
    i = 0;
    for(j = 0; j < data->info->matrix_xsize * data->info->matrix_ysize * 2; j += 2)
    {	   
		lsb = data->T37_buf[j] & 0xff;
		msb = data->T37_buf[j+1] & 0xff;
        if((lsb | (msb << 8)) == 0)
            continue;
		data->raw_data_16[i] = lsb | (msb << 8);
		i++;
    }

	goto out;

err_free_T37_buf:
	kfree(data->T37_buf);
	data->T37_buf = NULL;
	data->T37_buf_size = 0;
out:
	kfree(obuf);
	enable_irq(data->irq);
	return ret ?: 0;
}


static ssize_t mxt_short_circuit_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
//do not nedd this
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}


static ssize_t mxt_do_calibration_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static ssize_t mxt_calibration_data_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static ssize_t mxt_calibration_data_freq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "Sharp Module,Can't need read calibration data\n");
}

static ssize_t mxt_open_circuit_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ii;
	unsigned int jj;
	short *report_data_16, limit_b, limit_l;

	read_raw_data(data, MXT_T6_CMD_REFS);
	report_data_16 = data->raw_data_16;
	limit_b = data->raw_data_avg + 3500;
	limit_l = data->raw_data_avg - 3500;

#if 0 // temp solution
	for (ii = 0; ii < TX_NUM; ii++) {
		for (jj = 0; jj < RX_NUM; jj++) {
			if (*report_data_16 > limit_b ||
				*report_data_16 < limit_l ||
				*report_data_16 > THRESHOLD_MAX ||
				*report_data_16 < THRESHOLD_MIN) {
				return snprintf(buf, PAGE_SIZE,
					"TP Open Circuit Detected,\nTx:%d,Rx:%d,raw:%d, threshold:%d\n",
					ii, jj,
					*report_data_16, data->raw_data_avg);
			}
			report_data_16++;
		}
	}
#endif
	
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static ssize_t mxt_raw_cap_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ii;
	unsigned int jj;
	int cnt = 0;
	int count = 0;

	unsigned int sum = 0;
	short max = 0;
	short min = 0;
	u16 *report_data_16;

    struct mxt_info *info = data->info;
    struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;

	read_raw_data(data, MXT_T6_CMD_REFS);
	report_data_16 = data->raw_data_16;

	for (ii = 0; ii < TX_NUM; ii++) {
		for (jj = 0; jj < RX_NUM; jj++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d, ",
					*report_data_16);

			sum += *report_data_16;

			if (max < *report_data_16)
				max = *report_data_16;

			if (ii == 0 && jj == 0)
				min = *report_data_16;
			else if (*report_data_16 < min)
				min = *report_data_16;

			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
	}
	cnt = snprintf(buf, PAGE_SIZE - count, "\n");
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n",
			TX_NUM, RX_NUM);
	buf += cnt;
    count += cnt;

    cnt = snprintf(buf, PAGE_SIZE - count, "FW: %X.%X\t, Config: %d.%d.%d\n", info->version, info->build,
            cfg_version->year, cfg_version->month, cfg_version->date);
    buf += cnt;
    count += cnt;

    data->raw_data_avg = sum/RAW_DATA_SIZE;
    cnt = snprintf(buf, PAGE_SIZE - count,
			"max = %d, min = %d, average = %d\n",
			max, min, data->raw_data_avg);
	buf += cnt;
	count += cnt;

	return count;
}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
static ssize_t mxt_double_click_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "double_click_wake_enable:%u ; enable_wakeup_gesture:%u\n",
			data->double_click_wake_enable, data->enable_wakeup_gesture);
}

static ssize_t mxt_double_click_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->double_click_wake_enable = input;

    AM("irq: %d\n", data->client->irq);
	if (data->double_click_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

    if(data->enable_wakeup_gesture == true){
        enable_irq_wake(data->client->irq);
    } else {
        disable_irq_wake(data->client->irq);
    }

	return count;
}

static ssize_t mxt_down_to_up_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->down_to_up_wake_enable);
}

static ssize_t mxt_down_to_up_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->down_to_up_wake_enable = input;

	if (data->down_to_up_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else 
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_up_to_down_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->up_to_down_wake_enable);
}

static ssize_t mxt_up_to_down_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->up_to_down_wake_enable = input;

	if (data->up_to_down_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_right_to_left_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->right_to_left_wake_enable);
}

static ssize_t mxt_right_to_left_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->right_to_left_wake_enable = input;

	if (data->right_to_left_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else 
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_left_to_right_wake_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->left_to_right_wake_enable);
}

static ssize_t mxt_left_to_right_wake_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct mxt_data *data = dev_get_drvdata(dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	data->left_to_right_wake_enable = input;

	if (data->left_to_right_wake_enable)
		data->enable_wakeup_gesture = true;
	else
		data->enable_wakeup_gesture = false;

	if(data->enable_wakeup_gesture == true)
		enable_irq_wake(data->client->irq);
	else 
		disable_irq_wake(data->client->irq);

	return count;
}

static ssize_t mxt_detected_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			data->detected_gesture);
}
#endif

static ssize_t touch_debug_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int cmd, arg;
	struct mxt_data *data = dev_get_drvdata(dev);
	struct input_dev *input_dev = data->input_dev;
    if(sscanf(buf,"%d%d", &cmd, &arg) < 0)
        return -EINVAL;
    switch(cmd)
    {
        case 0:   
                    input_report_key(input_dev, KEY_APPSELECT, 1);
                    input_sync(input_dev);
                    input_report_key(input_dev, KEY_APPSELECT, 0);
                    input_sync(input_dev);
                    break;
        case 1:    
                    input_report_key(input_dev, KEY_HOMEPAGE, 1);
                    input_sync(input_dev);
                    input_report_key(input_dev, KEY_HOMEPAGE, 0);
                    input_sync(input_dev);
                    break;
        case 2:     
                    input_report_key(input_dev, KEY_BACK, 1);
                    input_sync(input_dev);
                    input_report_key(input_dev, KEY_BACK, 0);
                    input_sync(input_dev);
                    break;
        case 3:
                    mxt_suspend(dev);
                    break;
        case 4:
                    mxt_resume(dev);
                    break;
        case 5:
                    input_report_key(dev, KEY_POWER, 1);
                    input_sync(dev);
                    input_report_key(dev, KEY_POWER, 0);
                    input_sync(dev);
                    break;
        case 6:
                    data->enable_wakeup_gesture = true;
                    break;
        default:
                    break;
    }
    return count;
}
static ssize_t mulTouchtest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count = sprintf(buf,"%d\n", g_Multouch);
    return count; 
}
static ssize_t mxt_stay_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = sprintf(buf,"%s\n", data->staying ? "staying disable": "staying enable");
    return count; 
}
static ssize_t mxt_stay_wake_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i = 0;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->staying = (i == 0);

		printk("%s\n", i ? "staying disable" : "staying enable");
		return count;
	} else {
		printk("staying enabled write error\n");
		return -EINVAL;
	}
}
static DEVICE_ATTR(touch_debug, S_IRUGO|S_IWUSR, NULL, touch_debug_store);
static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(fw_version_check, S_IRUGO, mxt_fw_version_check_show, NULL);
static DEVICE_ATTR(cfg_version, S_IRUGO, mxt_cfg_version_show, NULL);
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(mxt_update, S_IRUSR, mxt_update_show, NULL);
static DEVICE_ATTR(reset, S_IWUGO, NULL, mxt_reset_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL,
	mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
	mxt_debug_enable_store);
static DEVICE_ATTR(config_csum, S_IRUGO, mxt_config_csum_show, NULL);

static DEVICE_ATTR(sys_suspend, S_IWUSR, mxt_sys_suspend_show, mxt_sys_suspend_store);

static DEVICE_ATTR(open_circuit_test, S_IRUGO, mxt_open_circuit_test_show, NULL);
static DEVICE_ATTR(short_circuit_test, S_IRUGO, mxt_short_circuit_test_show, NULL);
static DEVICE_ATTR(raw_cap_data, S_IRUGO, mxt_raw_cap_data_show, NULL);

static DEVICE_ATTR(calibration_data_test, S_IRUGO, mxt_calibration_data_test_show, NULL);
static DEVICE_ATTR(calibration_data_freq, S_IRUGO, mxt_calibration_data_freq_show, NULL);
static DEVICE_ATTR(do_calibration, S_IRUGO, mxt_do_calibration_show, NULL);
static DEVICE_ATTR(mulTouchtest, S_IRUGO | S_IWUGO, mulTouchtest_show, NULL);
static DEVICE_ATTR(stay_awake, S_IRUGO | S_IWUGO, mxt_stay_wake_show, mxt_stay_wake_store);

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
static DEVICE_ATTR(double_click_wake_enable, 0666,
	mxt_double_click_wake_enable_show, mxt_double_click_wake_enable_store);
static DEVICE_ATTR(down_to_up_wake_enable, 0666,
	mxt_down_to_up_wake_enable_show, mxt_down_to_up_wake_enable_store);
static DEVICE_ATTR(up_to_down_wake_enable, 0666,
	mxt_up_to_down_wake_enable_show, mxt_up_to_down_wake_enable_store);
static DEVICE_ATTR(right_to_left_wake_enable, 0666,
	mxt_right_to_left_wake_enable_show,
	mxt_right_to_left_wake_enable_store);
static DEVICE_ATTR(left_to_right_wake_enable, 0666,
	mxt_left_to_right_wake_enable_show,
	mxt_left_to_right_wake_enable_store);
static DEVICE_ATTR(detected_gesture, 0444, mxt_detected_gesture_show, NULL);
#endif

static struct attribute *mxt_attrs[] = {
	&dev_attr_fw_version.attr,
	&dev_attr_touch_debug.attr,
	&dev_attr_cfg_version.attr,
	&dev_attr_hw_version.attr,
	&dev_attr_object.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_update_cfg.attr,
	&dev_attr_reset.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_debug_v2_enable.attr,
	&dev_attr_debug_notify.attr,
	&dev_attr_config_csum.attr,
	&dev_attr_sys_suspend.attr,
	&dev_attr_open_circuit_test.attr,
	&dev_attr_raw_cap_data.attr,
	&dev_attr_calibration_data_test.attr,
	&dev_attr_calibration_data_freq.attr,
	&dev_attr_do_calibration.attr,
	&dev_attr_mxt_update.attr,
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	&dev_attr_double_click_wake_enable.attr,
	&dev_attr_down_to_up_wake_enable.attr,
	&dev_attr_up_to_down_wake_enable.attr,
	&dev_attr_right_to_left_wake_enable.attr,
	&dev_attr_left_to_right_wake_enable.attr,
	&dev_attr_detected_gesture.attr,
#endif
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static int mxt_mulTouchtest_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)
    {
        return 0;
    }
    ptr += mulTouchtest_show(&data->client->dev, NULL, ptr);

    *ppos += ptr - page;
    return (ptr - page);
}
static int mxt_raw_cap_data_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_raw_cap_data_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}

static int mxt_calibration_data_test_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_calibration_data_test_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}

static int mxt_calibration_data_freq0_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_calibration_data_freq_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}

static int mxt_calibration_data_freq1_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
      ptr += mxt_calibration_data_freq_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}

static int mxt_calibration_data_freq2_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
	
 ptr += mxt_calibration_data_freq_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}

static int mxt_do_calibration_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_do_calibration_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}

static int mxt_open_circuit_test_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_open_circuit_test_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}
static int mxt_short_circuit_test_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_short_circuit_test_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}
static int mxt_fw_version_check_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }
    ptr += mxt_fw_version_check_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}
static int mxt_stay_wake_read_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char *ptr = page;
	struct mxt_data *data = mxt_i2c_data;
    
    if (*ppos)  // CMD call again
    {
        return 0;
    }

    ptr += mxt_stay_wake_show(&data->client->dev, NULL, ptr);
    
    *ppos += ptr - page;
    return (ptr - page);
}
static int mxt_stay_wake_write_proc(struct file *file, char __user *page, size_t count, loff_t *ppos)
{
    char value[10]={0};
	struct mxt_data *data = mxt_i2c_data;
    
    if (copy_from_user(value, page, count))
    {
		dev_err(&data->client->dev, "%s: copy from user fail\n", __func__);
        return -EFAULT;
    }

    mxt_stay_wake_store(&data->client->dev, NULL, value, count);
    
    return count;
}

static PROC_FILE(synaptics_fw_version_check, S_IRUGO | S_IWUGO, mxt_fw_version_check_proc, NULL);
static PROC_FILE(open_circuit_test, S_IRUGO | S_IWUGO, mxt_open_circuit_test_proc, NULL);
static PROC_FILE(short_circuit_test, S_IRUGO | S_IWUGO, mxt_short_circuit_test_proc, NULL);
static PROC_FILE(mulTouchtest, S_IRUGO | S_IWUGO, mxt_mulTouchtest_proc, NULL);
static PROC_FILE(raw_cap_data, S_IRUGO | S_IWUGO, mxt_raw_cap_data_proc, NULL);
static PROC_FILE(do_calibration, S_IRUGO | S_IWUGO, mxt_do_calibration_proc, NULL);
static PROC_FILE(calibration_data_test, S_IRUGO | S_IWUGO, mxt_calibration_data_test_proc, NULL);
static PROC_FILE(calibration_data_freq0, S_IRUGO | S_IWUGO, mxt_calibration_data_freq0_proc, NULL);
static PROC_FILE(calibration_data_freq1, S_IRUGO | S_IWUGO, mxt_calibration_data_freq1_proc, NULL);
static PROC_FILE(calibration_data_freq2, S_IRUGO | S_IWUGO, mxt_calibration_data_freq2_proc, NULL);
static PROC_FILE(stay_wake, S_IRUGO | S_IWUGO, mxt_stay_wake_read_proc, mxt_stay_wake_write_proc);

static struct proc_file *sproc_file[] = {
    procify(synaptics_fw_version_check),
    procify(open_circuit_test),
    procify(short_circuit_test),
    procify(mulTouchtest),
    procify(raw_cap_data),
    procify(stay_wake),
    procify(do_calibration),
    procify(calibration_data_test),
    procify(calibration_data_freq0),
    procify(calibration_data_freq1),
    procify(calibration_data_freq2),
};
static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
	unsigned int num_mt_slots;
	int id;
	bool curr_state;
	const unsigned int *keymap;
	if (!input_dev)
		return;
	keymap = data->pdata->keymap[T15_T97_KEY];
	num_mt_slots = data->num_touchids + data->num_stylusids;
            AM("num_mt_slots: %d\n", num_mt_slots);

	for (id = 0; id < num_mt_slots; id++) {
            AM("\n");
		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}
	mxt_input_sync(data);
	for (id = 0; id < 3; id++) {
	curr_state = test_bit(id, &data->t15_keystatus);
	if (curr_state){
	input_event(input_dev, EV_KEY, keymap[id], 0);
	input_sync(input_dev);
	__clear_bit(id, &data->t15_keystatus);
		}
	}
}

static void mxt_start(struct mxt_data *data)
{
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
    if (data->enable_wakeup_gesture) {
        mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
        mxt_t6_command(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE, false);
        AM("\n");
        return;
    }
#endif

    if (!data->suspended || data->in_bootloader)
        return;

        mxt_regulator_restore(data);
//    if (data->use_regulator) {
//        AM("\n");
//        mxt_regulator_restore(data);
//    } else {
//        AM("\n");
//        /*
//         * Discard any messages still in message buffer
//         * from before chip went to sleep
//         */
//        mxt_process_messages_until_invalid(data);
//
//        mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
//
//        /* Recalibrate since chip has been in deep sleep */
//        mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
//
//        mxt_acquire_irq(data);
//    }

    data->suspended = false;
}

static void mxt_stop(struct mxt_data *data)
{
    u8 value;
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
    if (data->enable_wakeup_gesture) {
        mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
        mxt_t6_command(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE, false);
        data->suspended = true;
        return;
    }
#endif
    if (data->suspended || data->in_bootloader || data->updating_config)
        return;

    __mxt_read_reg(data->client, MXT_TOUCH_SELF_CAPACITANCE_CONFIG_T111 + 21, 1, &value);
    if(value == 255) {
        value = 60;
        __mxt_write_reg(data->client, MXT_TOUCH_SELF_CAPACITANCE_CONFIG_T111 + 21, 1, &value);
    }

    mt_eint_mask(6);

//    if (data->use_regulator)
//    {
//        mxt_regulator_disable(data);
//    }
//    else
//    {
//        mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
//    }

    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(10);
    hwPowerDown(TPD_POWER_SOURCE, "TP");

    AM("\n");
    mxt_reset_slots(data);
    data->suspended = true;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

//	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	struct device *device = &data->client->dev;

	dev_err(device, "%s\n", __func__);
	//mxt_stop(data);
}

#ifdef CONFIG_OF
static int mxt_parse_config_dt(struct mxt_data *data,
		struct device_node *np, struct mxt_config_info *info)
{
	int rc;
	u32 temp_val;
	struct device *dev = &data->client->dev;
	struct property *prop;

	rc = of_property_read_u32(np, "atmel,family-id", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read family-id\n", __func__);
		return rc;
	} else
		info->family_id = (u8) temp_val;

	rc = of_property_read_u32(np, "atmel,variant-id", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read variant-id\n", __func__);
		return rc;
	} else
		info->variant_id = (u8) temp_val;

	rc = of_property_read_u32(np, "atmel,version", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read version\n", __func__);
		return rc;
	} else
		info->version = (u8) temp_val;

	rc = of_property_read_u32(np, "atmel,build", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read build\n", __func__);
		return rc;
	} else
		info->build = (u8) temp_val;

	rc = of_property_read_u32(np, "atmel,config-year", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read config-year\n", __func__);
		return rc;
	} else
		info->config_year = (u8) temp_val;

	rc = of_property_read_u32(np, "atmel,config-month", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read config-month\n", __func__);
		return rc;
	} else
		info->config_month = (u8) temp_val;

	rc = of_property_read_u32(np, "atmel,config-date", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s, Unable to read config-date\n", __func__);
		return rc;
	} else
		info->config_date = (u8) temp_val;

	prop = of_find_property(np, "atmel,config", &info->config_length);
	if (!prop) {
		dev_err(dev, "Looking up %s property in node %s failed\n",
			"atmel,config", np->full_name);
		return -ENODEV;
	}

	/* Allocate mem, it will be released in mxt_release_config_mem function */
	info->config = kzalloc(info->config_length * sizeof(u8), GFP_KERNEL);
	if (!info->config) {
		dev_err(dev, "Failed to alloc memory for config data\n");
		return -ENOMEM;
	}

	dev_info(dev, "%s: config_data: 0x%x, config_month: 0x%x, config_year: 0x%x\n",
		__func__, info->config_date, info->config_month, info->config_year);

	/* Get config data */
	memcpy(info->config, prop->value, info->config_length);

	return 0;
}

static void mxt_release_config_mem(struct mxt_config_info *info)
{
	if (info->config)
		kfree(info->config);

	return;
}

static struct device_node *mxt_find_cfg_node(struct mxt_data *data, struct device_node *np)
{
	struct device *dev = &data->client->dev;
	struct mxt_config_info *info = &data->pdata->info;
	struct device_node *cfg_np = NULL;
	u32 temp_val, rc;
	enum hw_pattern_type type;

	cfg_np = of_get_next_child(np, NULL);
	if (cfg_np == NULL) {
		dev_err(dev, "%s, have no cfg node\n", __func__);
		return NULL;
	}

	do {
		rc = of_property_read_u32(cfg_np, "atmel,type", &temp_val);
		if (rc && (rc != -EINVAL)) {
			dev_err(dev,"TP: %s, Unable to read atmel,type\n", __func__);
			return NULL;
		}
        AM("temp_val: %d\n", temp_val);
		type = temp_val ? new_pattern: old_pattern;

        if(data->pattern_type != old_pattern && data->pattern_type != new_pattern)
            data->pattern_type = old_pattern;

		if (type == data->pattern_type) {
			printk("type: 0x%x, data->pattern_type: 0x%x\n", type, data->pattern_type);
			info->type = (u8) type;
			return cfg_np;
        }
    
		cfg_np = of_get_next_child(np , cfg_np);
	} while (cfg_np!= NULL);

	dev_err(dev, "No found cfg node, temp_val: 0x%x, data->pattern_type: 0x%x\n", type, data->pattern_type);
	return NULL;
}

static int mxt_parse_cfg_and_load(struct mxt_data *data,
	struct mxt_config_info *info, bool force)
{
	int rc, offset = 0, byte_offset = 0, reg;
	u8 *temp_cfg;
	int cfg_start_ofs;
	u8 *config_mem = NULL;
	size_t config_mem_size;
	struct mxt_object *object;
	unsigned int type, instance, size;
	struct device_node *cfg_np;
	struct device *dev = &data->client->dev;
	struct device_node *np = dev->of_node;
	struct mxt_cfg_version *cfg_version = &data->config_info.cfg_version;

    if(!np)
    {
        dev_err(dev, "%s, dev->of_node is NULL\n", __func__);
        return 0;
    }
	cfg_np = mxt_find_cfg_node(data, np);
	if (!cfg_np) {
		dev_err(dev, "%s, Not found touch config node\n", __func__);
		return -EIO;
	}

	rc = mxt_parse_config_dt(data, cfg_np, info);
	if (!rc)
		temp_cfg = info->config;
	else
		return -EIO;

	printk("config: year %d, month %d, date %d, hw: year %d, month %d, date %d\n",
		info->config_year, info->config_month, info->config_date, cfg_version->year, cfg_version->month, cfg_version->date);

	if (force)
		goto directly_update;

	if (info->version != data->info->version || info->build != data->info->build) {
		dev_err(dev, "%s: config corresponding fw version doesn't match\n", __func__);
//		return 0;
	}



	if (info->config_year != cfg_version->year || info->config_month != cfg_version->month
		|| info->config_date != cfg_version->date)
		printk("%s, ready to update config\n", __func__);
	else {
		printk("%s, the same cfg version\n", __func__);
		return 0;
	}

directly_update:
	/* Malloc memory to store configuration */
	cfg_start_ofs = MXT_OBJECT_START +
			data->info->object_num * sizeof(struct mxt_object) +MXT_INFO_CHECKSUM_SIZE;
	config_mem_size = data->mem_size - cfg_start_ofs;
	config_mem = kzalloc(config_mem_size, GFP_KERNEL);
	if (!config_mem) {
		dev_err(dev, "Failed to alloc memory for config mem\n");
		return -ENOMEM;
	}

	while (offset < info->config_length) {
		type = temp_cfg[offset + MXT_CFG_OFFSET_TYPE];
		instance = temp_cfg[offset + MXT_CFG_OFFSET_INSTANCE];
		size = temp_cfg[offset + MXT_CFG_OFFSET_SIZE];
		object = mxt_get_object(data, type);
		if (!object) {
			/* Skip object */
			offset = offset + MXT_CFG_OFFSET_SIZE + size + 1;
			continue;
		}

		if (size > mxt_obj_size(object)) {
			dev_err(dev, "redundant parameters\n");
			offset = offset + MXT_CFG_OFFSET_SIZE + size + 1;
			continue;
		} else if (mxt_obj_size(object) > size) {
			dev_err(dev, "object type: 0x%x, less parameters\n", type);
			offset = offset + MXT_CFG_OFFSET_SIZE + size + 1;
			continue;
		}

		if (instance >= mxt_obj_instances(object)) {
			dev_err(dev, "Object instances exceeded!\n");
			goto release_mem;
		}

		reg = object->start_address + mxt_obj_size(object) * instance;
		byte_offset = reg - cfg_start_ofs;

		if (type == MXT_USER_DATA_T38) {
			if (data->t38_config) {
				memcpy(config_mem + byte_offset, temp_cfg + offset + MXT_CFG_OFFSET_SIZE + 1, MXT_T38_INFO_SIZE);
				memcpy(config_mem + byte_offset + MXT_T38_INFO_SIZE, data->t38_config + MXT_T38_INFO_SIZE,
					data->T38_size - MXT_T38_INFO_SIZE);
			} else
				memcpy(config_mem + byte_offset, temp_cfg + offset + MXT_CFG_OFFSET_SIZE + 1, size);
		} else
			memcpy(config_mem + byte_offset, temp_cfg + offset + MXT_CFG_OFFSET_SIZE + 1, size);

		offset = offset + MXT_CFG_OFFSET_SIZE + size + 1;
	}

	/* Write configuration as blocks */
	byte_offset = 0;
	while (byte_offset < config_mem_size) {
		size = config_mem_size - byte_offset;

		if (size > MXT_MAX_BLOCK_WRITE)
			size = MXT_MAX_BLOCK_WRITE;

		rc = __mxt_write_reg(data->client,
				      cfg_start_ofs + byte_offset,
				      size, config_mem + byte_offset);
		if (rc != 0) {
			dev_err(dev, "Config write error, ret=%d\n", rc);
			goto release_mem;
		}

		byte_offset += size;
	}

	mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);


	rc = mxt_check_retrigen(data);
	if (rc)
		goto release_mem;

	rc = mxt_soft_reset(data);
	if (rc)
		dev_err(dev, "%s, reset failed\n",__func__);


	/* T7 config may have changed */
	mxt_init_t7_power_cfg(data);

//	rc = mxt_initialize(data);
//	if (rc) {
//		dev_err(dev, "Failed to initialize device\n");
//        return -1;
//	}
	rc = mxt_read_t38_object(data);
	if (rc) {
		dev_err(dev, "%s: Failed to read t38 object\n",__func__);
		return rc;
	}
	rc = mxt_register_input_device(data);
	if (rc) {
		dev_err(dev, "Failed to register input device\n");
		return rc;
	}
	printk("mxt: update config successfully\n");

release_mem:
	mxt_release_config_mem(info);
	if (config_mem)
		kfree(config_mem);
	return 0;
}
static struct mxt_platform_data *mxt_parse_dt(struct i2c_client *client)
{
	struct mxt_platform_data *pdata;
	struct device *dev = &client->dev;
	struct property *prop;
	unsigned int *keymap;
	int proplen, ret;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	/* reset gpio */
	pdata->gpio_reset = of_get_named_gpio_flags(dev->of_node,
		"atmel,reset-gpio", 0, NULL);

	/*power gpio*/
	pdata->gpio_vdd = of_get_named_gpio_flags(dev->of_node,
		"atmel,vdd-gpio", 0, NULL);
	of_property_read_string(dev->of_node, "atmel,cfg_name",
				&pdata->cfg_name);

	of_property_read_string(dev->of_node, "atmel,input_name",
				&pdata->input_name);

	of_property_read_string(dev->of_node, "atmel,fw_version",
				&pdata->fw_version);

	prop = of_find_property(dev->of_node, "linux,gpio-keymap", &proplen);
	if (prop) {
		pdata->t19_num_keys = proplen / sizeof(u32);

		keymap = devm_kzalloc(dev,
			pdata->t19_num_keys * sizeof(u32), GFP_KERNEL);
		if (!keymap)
			return NULL;

		pdata->t19_keymap = keymap;

		ret = of_property_read_u32_array(client->dev.of_node,
			"linux,gpio-keymap", keymap, pdata->t19_num_keys);
		if (ret) {
			dev_err(dev,
				"Unable to read device tree key codes: %d\n",
				 ret);
			return NULL;
		}
	}

	return pdata;
}
#endif

static bool is_need_to_update_fw(struct mxt_data *data)
{
    struct mxt_info *info = data->info;
    unsigned int version, build;
    int rc;
    const char *fw_name = data->pdata->fw_version;
    struct device *dev = &data->client->dev;

    if(data->update_force)
    {
        printk("force update fw and cfg!\n");
        return true;
    }
    rc = sscanf(fw_name, "%2x%2x", &version, &build);
    printk("info->version: 0x%x, info->build: 0x%x, version: 0x%x, build: 0x%x\n",
            info->version, info->build, version, build);
    if (rc != 2) {
        dev_err(dev, "Can't get the fw version from DTS file\n");
        return false;
    }

    if (info->version == version && info->build == build) {
        dev_info(dev, "The same fw version\n");
        return false;
    }
#if 0  //amin
    if (info->version < version) {
        printk("mxt: TP: version: need to update fw\n");
        return true;
    } else if (info->version == version && info->build < build) {
        printk("mxt: TP: build: need to update fw\n");
        return true;
    }
#endif
    if((info->version < version) ||
		((info->version == version) && (info->build < build)))
    {
        printk("TP: build: need to update fw\n");
        return true;
    }
    printk("info->version: 0x%x, info->build: 0x%x, version: 0x%x, build: 0x%x\n",
            info->version, info->build, version, build);
    return false; // amin
}

static int mxt_update_fw(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	const char *fw_version = data->pdata->fw_version;
	char fw_name[MXT_FW_NAME_SIZE];
	int ret;

	/* Get the fw name from parsing dts file */
//	ret = snprintf(fw_name, MXT_FW_NAME_SIZE, "mXT640T_V%4s.fw", fw_version);  amin
	ret = snprintf(fw_name, MXT_FW_NAME_SIZE, "%4s", fw_version);
	if (ret != MXT_FW_NAME_SIZE) {
//		dev_err(dev, "TP: %s, Failed to get fw name from DTS\n", __func__);
//		return ret;
	}

	printk("mxt: %s: fw_name: %s, ret: %d\n", __func__, fw_name, ret);

	ret = __mxt_update_fw(&client->dev, fw_name, strlen(fw_name));
	if (ret) {
		dev_err(dev, "unable to update firmware\n");
		return ret;
	}

	return 0;
}

static int mxt_update_fw_and_cfg(void *args)
{
	int error;
    struct mxt_data *data = (struct mxt_data *)args;
	struct device *dev = &data->client->dev;
    char boot_mode = get_boot_mode();
	if (boot_mode == META_BOOT) {
		data->staying = HAVE_UPDATED;
		dev_dbg(dev, "%s: Don't update fw in META_BOOT\n", __func__);
		return 0;
	}
	
    data->staying = IS_UPDATINGE;
	if (is_need_to_update_fw(data)) {
		error = mxt_update_fw(data);
		if (error) {
			dev_err(dev, "mxt: %s: Failed to update fw\n", __func__);
            goto update_fail;
		}

        error = mxt_parse_cfg_and_load(data,&data->pdata->info, true);
        if (error) {
            dev_err(dev, "mxt: Failed to update config, after firmware updating\n");
            goto update_fail;
        }
	} else {
		error = mxt_parse_cfg_and_load(data, &data->pdata->info, false);
		if (error) {
			dev_err(dev, "mxt: Failed to update config\n");
            goto update_fail;
		}
	}
	
//	error = mxt_initialize(data);
//	if (error) {
//		dev_err(dev, "Failed to initialize device\n");
//        return -1;
//	}
    data->staying = HAVE_UPDATED;
	return 0;
	
update_fail:
	data->staying = HAVE_UPDATED;
	return -EIO;
}

static inline void board_gpio_init(const struct mxt_platform_data *pdata)
{

    // if gpio init in board, or use regulator , skip this function

    /* set irq pin input/pull high */
    /*	 power off vdd/avdd   */
    /*		msleep(100);		*/
    /* 	  set reset output 0 	*/
    /*	 power up vdd/avdd	*/
    /*		msleep(50);			*/
    /*	   set reset output 1 	*/
    /*		msleep(200);		*/


}
static int mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct mxt_data *data;
	unsigned char attr_count;
	struct proc_dir_entry *mxt_config_proc = NULL;
	int error;
    int retval;
	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
    mxt_i2c_data = data;
    //frank
	mutex_init(&data->bus_access_mutex);

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	data->client = client;
//	6 = client->irq;
//   AM("6: %d\n", 6);
//	data->pdata = dev_get_platdata(&client->dev);
    data->pdata = &mxt_platform_data;
    if(!data->pdata)
    {
        AM("\n");
    }
    //data->pdata = kzalloc(sizeof(struct mxt_platform_data), GFP_KERNEL);
	i2c_set_clientdata(client, data);

#if defined(CONFIG_MXT_I2C_DMA)
	client->addr |= I2C_RS_FLAG | I2C_ENEXT_FLAG | I2C_DMA_FLAG;
	data->i2c_dma_va = (u8 *)dma_alloc_coherent(&client->dev, PAGE_SIZE * 2, &data->i2c_dma_pa, GFP_KERNEL);
	if (!data->i2c_dma_va)	{
		error = -ENOMEM;
		dev_err(&client->dev, "Allocate DMA I2C Buffer failed!\n");
		goto err_free_mem;
	}
#endif
#ifdef CONFIG_OF
    if (!data->pdata && client->dev.of_node)
    {
        data->pdata = mxt_parse_dt(client);
    }
#endif

	if (!data->pdata) {
		data->pdata = devm_kzalloc(&client->dev, sizeof(*data->pdata),
					   GFP_KERNEL);
		if (!data->pdata) {
			dev_err(&client->dev, "Failed to allocate pdata\n");
			error = -ENOMEM;
			goto err_free_mem;
		}

		/* Set default parameters */
//		data->pdata->irqflags = IRQF_TRIGGER_LOW;
	}


	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);
	mutex_init(&data->debug_msg_lock);

//	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
//				     data->p6flags | IRQF_ONESHOT,
//				     client->name, data);
    //    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
//	if (error) {
//		dev_err(&client->dev, "Failed to register interrupt\n");
//		goto err_free_irq;
//	}
	thread = kthread_run(touch_event_handler, data, "atmel-tpd");
	if ( IS_ERR(thread) ) {
		retval = PTR_ERR(thread);
		pr_err(" %s: failed to create kernel thread: %d\n",__func__, retval);
	}
	/* power supply */
//	mxt_probe_regulators(data);

	/* read the information block, initiate the mxt_data struct */

    //power up is included
//    board_gpio_init(data->pdata);
#if 1
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
    msleep(10);
    hwPowerOn(TPD_POWER_SOURCE, VOL_3300, "TP");
    //	hwPowerOn(MT6323_POWER_LDO_VGP2, VOL_3300, "TP");
    msleep(50);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(200);
//retry_wait:
//	reinit_completion(&data->bl_completion);
//	data->in_bootloader = true;
//	error = mxt_wait_for_completion(data, &data->bl_completion,
//					MXT_POWERON_DELAY);
//	if (error == -EINTR)
//		goto retry_wait;
//	data->in_bootloader = false;
#endif


    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
    //mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	error = mxt_initialize(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize device\n");
		goto err_free_object;
	}
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINTF_TRIGGER_LOW, mxt_interrupt, 0);
	/* Determine whether to update fw and config */
   //amin
   
    thread = kthread_run(mxt_update_fw_and_cfg, (void *)data, "atmel_fw_update");
    if (IS_ERR(thread))
    {
		retval = PTR_ERR(thread);
		pr_err(" %s: failed to create kernel thread: %d\n",__func__, retval);
        return -1;
    }
#if 0   //amin
	error = mxt_update_fw_and_cfg(data);
	if (error)
		dev_err(&data->client->dev, "Failed to update fw and cfg\n");
#endif



	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failure %d creating sysfs group\n",
			error);
		goto err_free_object;
	}

    for(attr_count = 0; attr_count < ARRAY_SIZE(sproc_file); attr_count++)
    {
        mxt_config_proc = proc_create(sproc_file[attr_count]->name, 
                sproc_file[attr_count]->mode, NULL, &(sproc_file[attr_count]->fop));
        if(!mxt_config_proc)
        {
            dev_err(&client->dev,
                    "%s: Failed to create procfs entry\n",
                    __func__);
        }
    }
	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_remove_sysfs_group;
	}

	/*register input device*/
#if 0
	error = mxt_register_input_device(data);
	if (error)
		dev_err(&data->client->dev, "Failed to register input device\n");
#endif

#if defined(CONFIG_FB_PM)
	INIT_WORK(&data->fb_notify_work, fb_notify_resume_work);
	data->fb_notif.notifier_call = fb_notifier_callback;
	error = fb_register_client(&data->fb_notif);
	if (error) {
		dev_err(&client->dev,
				"Unable to register fb_notifier: %d\n", error);
		goto err_remove_mem_access_attr;
	}
#endif
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_tpd_suspend;
	data->early_suspend.resume = mxt_tpd_resume;
	register_early_suspend(&data->early_suspend);
	tpd_load_status = 1;

//frank only for debug use
//	data->enable_wakeup_gesture = 1;
//	data->double_click_wake_enable = 1;
//frank only for debug use
//
	return 0;

#if defined(CONFIG_FB_PM)
err_remove_mem_access_attr:
    if(!gpDMABuf_va){
        dma_free_coherent(&(client->dev), GTP_DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);
        gpDMABuf_va = NULL;
        gpDMABuf_pa = 0;
        return -1;
    }
	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
#endif
err_remove_sysfs_group:
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
err_free_object:
	mxt_free_object_table(data);
err_free_irq:
	free_irq(client->irq, data);
err_free_mem:
#if defined(CONFIG_MXT_I2C_DMA)	
	dma_free_coherent(&client->dev, PAGE_SIZE * 2, data->i2c_dma_va, data->i2c_dma_pa);
#endif
	kfree(data);
	return error;
}

static int mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
	tpd_load_status = 0;
	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);

	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
#if defined(CONFIG_MXT_I2C_DMA)
	dma_free_coherent(&client->dev, PAGE_SIZE * 2, data->i2c_dma_va, data->i2c_dma_pa);
#endif	
	free_irq(6, data);
	regulator_put(data->reg_avdd);
	regulator_put(data->reg_vdd);
	mxt_free_object_table(data);
	kfree(data);

	return 0;
}

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
void mxt_set_gesture(struct mxt_data *mxt, bool enable)
{
	if (enable) {
		/* Enable gesture, double click, event reporting */
		mxt_config_ctrl_set(mxt, mxt->T93_address, 0x02);

		/* Enable gesture, up, down, left, right, event reporting */
		mxt_config_ctrl_set(mxt, mxt->T115_address, 0x02);
	} else {
		/* Disable gesture, double click, event reporting */
		mxt_config_ctrl_clear(mxt, mxt->T93_address, 0x02);

		/* Disable gesture, up, down, left, right, event reporting */
		mxt_config_ctrl_clear(mxt, mxt->T115_address, 0x02);
	}
}
#endif

#if defined(CONFIG_FB_PM)
static void fb_notify_resume_work(struct work_struct *work)
{
	struct mxt_data *mxt =
		 container_of(work,
			struct mxt_data, fb_notify_work);
	mxt_resume(&(mxt->input_dev->dev));

	/* Enable general touch event reporting */
	mxt_config_ctrl_set(mxt, mxt->T100_address, 0x02);
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
	if (mxt->enable_wakeup_gesture)
		mxt_set_gesture(mxt, false);
#endif
}

static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct mxt_data *mxt = container_of(self, struct mxt_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
				mxt && mxt->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
#ifdef MAX1_WAKEUP_GESTURE_ENABLE
			mxt->suspended = 0;
#endif
			printk(&mxt->client->dev,
					"TP: %s(), FB_BLANK_UNBLANK\n",
					__func__);

			schedule_work(&mxt->fb_notify_work);

		} else if (*blank == FB_BLANK_POWERDOWN) {
			if (flush_work(&mxt->fb_notify_work))
				pr_warn("%s: waited resume worker finished\n",
				__func__);

			printk(&mxt->client->dev,
					"TP: %s(), FB_BLANK_POWERDOWN\n",
					__func__);

			/* Disable general touch event reporting */
			mxt_config_ctrl_clear(mxt, mxt->T100_address, 0x02);

#ifdef MAX1_WAKEUP_GESTURE_ENABLE
			mxt->suspended = 1;
			if (mxt->enable_wakeup_gesture)
				mxt_set_gesture(mxt, true);
#endif
			mxt_reset_slots(mxt);

			mxt_suspend(&(mxt->input_dev->dev));
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
#if 1
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

//	mutex_lock(&input_dev->mutex);
    AM("input_dev->users: %d\n", input_dev->users);
	if (input_dev->users)
    {
		mxt_stop(data);
    }

//	mutex_unlock(&input_dev->mutex);
#endif
	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

//	mutex_lock(&input_dev->mutex);

    AM("input_dev->users: %d\n", input_dev->users);
        if (input_dev->users)
        {
            mxt_start(data);
        }
//	mutex_unlock(&input_dev->mutex);
	return 0;
}

static void mxt_tpd_suspend(struct early_suspend *h)
{
#if 1
	struct mxt_data *data = mxt_i2c_data;

	printk("[mxt] tpd_suspend\n");
    if(data->staying == IS_UPDATINGE)
    {
        printk("%s staying!!\n", __func__);
        return;
    }

	if(data)
    {
		mxt_suspend(&(data->client->dev));
    }
    else
    {
        AM("data is NULL\n");
    }
#endif
}

static void mxt_tpd_resume(struct early_suspend *h)
{
#if 1
	printk("[mxt] tpd_resume\n");
	struct mxt_data *data = mxt_i2c_data;
	
    if(data->staying == IS_UPDATINGE)
    {
        printk("%s staying!!\n", __func__);
        return;
    }
	if(data)
		mxt_resume(&data->client->dev);
#endif
}

#endif

static const struct of_device_id mxt_i2c_table[] = {
    {.compatible = "atmel,atmel_mxt_ts",},
	{},
};
static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(of, mxt_i2c_table);

static struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
        .of_match_table = mxt_i2c_table,
	},
	.probe		= mxt_probe,
	.remove		= mxt_remove,
	.id_table	= mxt_id,
};

static int tpd_local_init(void)
{
	TPD_DMESG("Atmel I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
		TPD_DMESG("tangjie Error unable to add i2c driver.\n");
		return -1;
	}
#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif 
	if(tpd_load_status == 0) {  // disable auto load touch driver for linux3.0 porting
		printk("mxt atmel add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	return 0;
}

static struct tpd_driver_t atmel_mxt_driver = {
	.tpd_device_name = "atmel_mxt_ts",
	.tpd_local_init = tpd_local_init,
	.suspend = mxt_tpd_suspend,
	.resume = mxt_tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif		
};

static struct mxt_platform_data mxt_platform_data = {
	.irqflags = IRQF_TRIGGER_LOW/*IRQF_TRIGGER_FALLING*/,  //the flag here should be matched with board_init_irq() irq triggle method
	.num_keys = mxts_num_keys,
	.keymap = mxts_keys,
//	.t15_num_keys = 3,
//	.t15_keymap = mxt_keymap,
#if defined(CONFIG_MXT_REPORT_VIRTUAL_KEY_SLOT_NUM)
	.max_y_t = 1919,     //Max value of Touch AA, asix more than this value will process by VirtualKeyHit
	.vkey_space_ratio = {5,8,15,10},
#endif
#if defined(CONFIG_MXT_SELFCAP_TUNE)
	.config_array = mxt_config_array,
#endif
	.fw_version = "30AA.fw",
    .cfg_name = "30AA.raw"
};
static struct i2c_board_info __initdata i2c_tpd[]={
    {I2C_BOARD_INFO("atmel_mxt_ts", (TPD_I2C_ADDR)),
        .platform_data = &mxt_platform_data,
        .irq = 6,
    },
};

static int __init atmel_mxt_init(void)
{
	if(!strcmp(mtkfb_find_lcm_driver(),"nt35596_hd1080_vdo_5A_tps65132")){
		i2c_register_board_info(TPD_I2C_BUS, i2c_tpd, ARRAY_SIZE(i2c_tpd));
#if 1
		if(i2c_add_driver(&tpd_i2c_driver)!=0)
		{
			TPD_DMESG("tangjie Error unable to add i2c driver.\n");
			return -1;
		}
#else
		if(tpd_driver_add(&atmel_mxt_driver) < 0){
			pr_err("Fail to add tpd driver\n");
			return -1;
		}
#endif
	}
	return 0;
}
static void __exit atmel_mxt_exit(void)
{
	tpd_driver_remove(&atmel_mxt_driver);
	return;
}
#if 1
late_initcall(atmel_mxt_init);
#else
module_init(atmel_mxt_init);
#endif
module_exit(atmel_mxt_exit);
/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");

