/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
 #if BUILD_DEBUG
#define DEBUG
#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/ctype.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"

#define SYSFS_FOLDER_NAME "f54"

#define GET_REPORT_TIMEOUT_S 2
#define COMMAND_TIMEOUT_100MS 20

#define STATUS_IDLE 0
#define STATUS_BUSY 1
#define STATUS_ERROR 2

#define REPORT_INDEX_OFFSET 1
#define REPORT_DATA_OFFSET 3

#define SENSOR_RX_MAPPING_OFFSET 1
#define SENSOR_TX_MAPPING_OFFSET 2

#define COMMAND_GET_REPORT 1
#define COMMAND_FORCE_CAL 2
#define COMMAND_FORCE_UPDATE 4

#define CONTROL_NO_AUTO_CAL 1

#define CONTROL_0_SIZE 1
#define CONTROL_1_SIZE 1
#define CONTROL_2_SIZE 2
#define CONTROL_3_SIZE 1
#define CONTROL_4_6_SIZE 3
#define CONTROL_7_SIZE 1
#define CONTROL_8_9_SIZE 3
#define CONTROL_10_SIZE 1
#define CONTROL_11_SIZE 2
#define CONTROL_12_13_SIZE 2
#define CONTROL_14_SIZE 1
#define CONTROL_15_SIZE 1
#define CONTROL_16_SIZE 1
#define CONTROL_17_SIZE 1
#define CONTROL_18_SIZE 1
#define CONTROL_19_SIZE 1
#define CONTROL_20_SIZE 1
#define CONTROL_21_SIZE 2
#define CONTROL_22_26_SIZE 7
#define CONTROL_27_SIZE 1
#define CONTROL_28_SIZE 2
#define CONTROL_29_SIZE 1
#define CONTROL_30_SIZE 1
#define CONTROL_31_SIZE 1
#define CONTROL_32_35_SIZE 8
#define CONTROL_36_SIZE 1
#define CONTROL_37_SIZE 1
#define CONTROL_38_SIZE 1
#define CONTROL_39_SIZE 1
#define CONTROL_40_SIZE 1
#define CONTROL_41_SIZE 1
#define CONTROL_42_SIZE 2
#define CONTROL_43_54_SIZE 13
#define CONTROL_55_56_SIZE 2
#define CONTROL_57_SIZE 1
#define CONTROL_58_SIZE 1
#define CONTROL_59_SIZE 2
#define CONTROL_60_62_SIZE 3
#define CONTROL_63_SIZE 1
#define CONTROL_64_67_SIZE 4
#define CONTROL_68_73_SIZE 8
#define CONTROL_74_SIZE 2
#define CONTROL_75_SIZE 1
#define CONTROL_76_SIZE 1
#define CONTROL_77_78_SIZE 2
#define CONTROL_79_83_SIZE 5
#define CONTROL_84_85_SIZE 2
#define CONTROL_86_SIZE 1
#define CONTROL_87_SIZE 1
#define CONTROL_88_SIZE 1

#define HIGH_RESISTANCE_DATA_SIZE 6
#define FULL_RAW_CAP_MIN_MAX_DATA_SIZE 4
#define TRX_OPEN_SHORT_DATA_SIZE 7

#define concat(a, b) a##b

#define attrify(propname) (&dev_attr_##propname.attr)

#define show_prototype(propname)\
static ssize_t concat(test_sysfs, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf);\
\
struct device_attribute dev_attr_##propname =\
		__ATTR(propname, S_IRUGO,\
		concat(test_sysfs, _##propname##_show),\
		synaptics_rmi4_store_error);

#define store_prototype(propname)\
static ssize_t concat(test_sysfs, _##propname##_store)(\
		struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t count);\
\
struct device_attribute dev_attr_##propname =\
		__ATTR(propname, S_IWUGO,\
		synaptics_rmi4_show_error,\
		concat(test_sysfs, _##propname##_store));

#define show_store_prototype(propname)\
static ssize_t concat(test_sysfs, _##propname##_show)(\
		struct device *dev,\
		struct device_attribute *attr,\
		char *buf);\
\
static ssize_t concat(test_sysfs, _##propname##_store)(\
		struct device *dev,\
		struct device_attribute *attr,\
		const char *buf, size_t count);\
\
struct device_attribute dev_attr_##propname =\
		__ATTR(propname, (S_IRUGO | S_IWUGO),\
		concat(test_sysfs, _##propname##_show),\
		concat(test_sysfs, _##propname##_store));

#define disable_cbc(ctrl_num)\
do {\
	retval = synaptics_rmi4_i2c_read(rmi4_data,\
			f54->control.ctrl_num->address,\
			f54->control.ctrl_num->data,\
			sizeof(f54->control.ctrl_num->data));\
	if (retval < 0) {\
		dev_err(&rmi4_data->i2c_client->dev,\
				"%s: Failed to disable CBC (" #ctrl_num ")\n",\
				__func__);\
		return retval;\
	} \
	f54->control.ctrl_num->cbc_tx_carrier_selection = 0;\
	retval = synaptics_rmi4_i2c_write(rmi4_data,\
			f54->control.ctrl_num->address,\
			f54->control.ctrl_num->data,\
			sizeof(f54->control.ctrl_num->data));\
	if (retval < 0) {\
		dev_err(&rmi4_data->i2c_client->dev,\
				"%s: Failed to disable CBC (" #ctrl_num ")\n",\
				__func__);\
		return retval;\
	} \
} while (0)

enum f54_report_types {
	F54_8BIT_IMAGE = 1,
	F54_16BIT_IMAGE = 2,
	F54_RAW_16BIT_IMAGE = 3,
	F54_HIGH_RESISTANCE = 4,
	F54_TX_TO_TX_SHORTS = 5,
	F54_RX_TO_RX_SHORTS_1 = 7,
	F54_TRUE_BASELINE = 9,
	F54_FULL_RAW_CAP_MIN_MAX = 13,
	F54_RX_OPENS_1 = 14,
	F54_TX_OPENS = 15,
	F54_TX_TO_GND_SHORTS = 16,
	F54_RX_TO_RX_SHORTS_2 = 17,
	F54_RX_OPENS_2 = 18,
	F54_FULL_RAW_CAP = 19,
	F54_FULL_RAW_CAP_NO_RX_COUPLING = 20,
	F54_SENSOR_SPEED = 22,
	F54_ADC_RANGE = 23,
	F54_TRX_OPENS = 24,
	F54_TRX_TO_GND_SHORTS = 25,
	F54_TRX_SHORTS = 26,
	F54_ABS_RAW_CAP = 38,
	F54_ABS_DELTA_CAP = 40,
	INVALID_REPORT_TYPE = -1,
};

struct f54_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;

			/* query 1 */
			unsigned char num_of_tx_electrodes;

			/* query 2 */
			unsigned char f54_query2_b0__1:2;
			unsigned char has_baseline:1;
			unsigned char has_image8:1;
			unsigned char f54_query2_b4__5:2;
			unsigned char has_image16:1;
			unsigned char f54_query2_b7:1;

			/* queries 3.0 and 3.1 */
			unsigned short clock_rate;

			/* query 4 */
			unsigned char touch_controller_family;

			/* query 5 */
			unsigned char has_pixel_touch_threshold_adjustment:1;
			unsigned char f54_query5_b1__7:7;

			/* query 6 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_interference_metric:1;
			unsigned char has_sense_frequency_control:1;
			unsigned char has_firmware_noise_mitigation:1;
			unsigned char has_ctrl11:1;
			unsigned char has_two_byte_report_rate:1;
			unsigned char has_one_byte_report_rate:1;
			unsigned char has_relaxation_control:1;

			/* query 7 */
			unsigned char curve_compensation_mode:2;
			unsigned char f54_query7_b2__7:6;

			/* query 8 */
			unsigned char f54_query8_b0:1;
			unsigned char has_iir_filter:1;
			unsigned char has_cmn_removal:1;
			unsigned char has_cmn_maximum:1;
			unsigned char has_touch_hysteresis:1;
			unsigned char has_edge_compensation:1;
			unsigned char has_per_frequency_noise_control:1;
			unsigned char has_enhanced_stretch:1;

			/* query 9 */
			unsigned char has_force_fast_relaxation:1;
			unsigned char has_multi_metric_state_machine:1;
			unsigned char has_signal_clarity:1;
			unsigned char has_variance_metric:1;
			unsigned char has_0d_relaxation_control:1;
			unsigned char has_0d_acquisition_control:1;
			unsigned char has_status:1;
			unsigned char has_slew_metric:1;

			/* query 10 */
			unsigned char has_h_blank:1;
			unsigned char has_v_blank:1;
			unsigned char has_long_h_blank:1;
			unsigned char has_startup_fast_relaxation:1;
			unsigned char has_esd_control:1;
			unsigned char has_noise_mitigation2:1;
			unsigned char has_noise_state:1;
			unsigned char has_energy_ratio_relaxation:1;

			/* query 11 */
			unsigned char has_excessive_noise_reporting:1;
			unsigned char has_slew_option:1;
			unsigned char has_two_overhead_bursts:1;
			unsigned char has_query13:1;
			unsigned char has_one_overhead_burst:1;
			unsigned char f54_query11_b5:1;
			unsigned char has_ctrl88:1;
			unsigned char has_query15:1;

			/* query 12 */
			unsigned char number_of_sensing_frequencies:4;
			unsigned char f54_query12_b4__7:4;
		} __packed;
		unsigned char data[14];
	};
};

struct f54_query_13 {
	union {
		struct {
			unsigned char has_ctrl86:1;
			unsigned char has_ctrl87:1;
			unsigned char has_ctrl87_sub0:1;
			unsigned char has_ctrl87_sub1:1;
			unsigned char has_ctrl87_sub2:1;
			unsigned char has_cidim:1;
			unsigned char has_noise_mitigation_enhancement:1;
			unsigned char has_rail_im:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_15 {
	union {
		struct {
			unsigned char has_ctrl90:1;
			unsigned char has_transmit_strength:1;
			unsigned char has_ctrl87_sub3:1;
			unsigned char has_query16:1;
			unsigned char has_query20:1;
			unsigned char has_query21:1;
			unsigned char has_query22:1;
			unsigned char has_query25:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_16 {
	union {
		struct {
			unsigned char has_query17:1;
			unsigned char has_data17:1;
			unsigned char has_ctrl92:1;
			unsigned char has_ctrl93:1;
			unsigned char has_ctrl94_query18:1;
			unsigned char has_ctrl95_query19:1;
			unsigned char has_ctrl99:1;
			unsigned char has_ctrl100:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_query_21 {
	union {
		struct {
			unsigned char has_abs_rx:1;
			unsigned char has_abs_tx:1;
			unsigned char has_ctrl91:1;
			unsigned char has_ctrl96:1;
			unsigned char has_ctrl97:1;
			unsigned char has_ctrl98:1;
			unsigned char has_data19:1;
			unsigned char has_query24_data18:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f54_control_7 {
	union {
		struct {
			unsigned char cbc_cap:3;
			unsigned char cbc_polarity:1;
			unsigned char cbc_tx_carrier_selection:1;
			unsigned char f54_ctrl7_b5__7:3;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_41 {
	union {
		struct {
			unsigned char no_signal_clarity:1;
			unsigned char f54_ctrl41_b1__7:7;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_57 {
	union {
		struct {
			unsigned char cbc_cap:3;
			unsigned char cbc_polarity:1;
			unsigned char cbc_tx_carrier_selection:1;
			unsigned char f54_ctrl57_b5__7:3;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control_88 {
	union {
		struct {
			unsigned char tx_low_reference_polarity:1;
			unsigned char tx_high_reference_polarity:1;
			unsigned char abs_low_reference_polarity:1;
			unsigned char abs_polarity:1;
			unsigned char cbc_polarity:1;
			unsigned char cbc_tx_carrier_selection:1;
			unsigned char charge_pump_enable:1;
			unsigned char cbc_abs_auto_servo:1;
		} __packed;
		struct {
			unsigned char data[1];
			unsigned short address;
		} __packed;
	};
};

struct f54_control {
	struct f54_control_7 *reg_7;
	struct f54_control_41 *reg_41;
	struct f54_control_57 *reg_57;
	struct f54_control_88 *reg_88;
};

struct synaptics_rmi4_f54_handle {
	bool no_auto_cal;
	unsigned char status;
	unsigned char intr_mask;
	unsigned char intr_reg_num;
	unsigned char tx_assigned;
	unsigned char rx_assigned;
	unsigned char *report_data;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	unsigned short fifoindex;
	size_t report_size;
	unsigned int data_buffer_size;
	unsigned int data_pos;
	enum f54_report_types report_type;
	struct f54_query query;
	struct f54_query_13 query_13;
	struct f54_query_15 query_15;
	struct f54_query_16 query_16;
	struct f54_query_21 query_21;
	struct f54_control control;
	struct mutex status_mutex;
	struct kobject *sysfs_dir;
	struct hrtimer watchdog;
	struct work_struct timeout_work;
	struct work_struct test_report_work;
	struct workqueue_struct *test_report_workqueue;
	struct synaptics_rmi4_data *rmi4_data;
};

struct f55_query {
	union {
		struct {
			/* query 0 */
			unsigned char num_of_rx_electrodes;

			/* query 1 */
			unsigned char num_of_tx_electrodes;

			/* query 2 */
			unsigned char has_sensor_assignment:1;
			unsigned char has_edge_compensation:1;
			unsigned char curve_compensation_mode:2;
			unsigned char has_ctrl6:1;
			unsigned char has_alternate_transmitter_assignment:1;
			unsigned char has_single_layer_multi_touch:1;
			unsigned char has_query5:1;
		} __packed;
		unsigned char data[3];
	};
};

struct synaptics_rmi4_f55_handle {
	unsigned char *tx_assignment;
	unsigned char *rx_assignment;
	unsigned short query_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned short command_base_addr;
	struct f55_query query;
};

show_prototype(num_of_mapped_tx)
show_prototype(num_of_mapped_rx)
show_prototype(tx_mapping)
show_prototype(rx_mapping)
show_prototype(report_size)
show_prototype(f54status)
show_prototype(short_circuit_test)
show_prototype(open_circuit_test)
show_prototype(raw_cap_data)
store_prototype(do_preparation)
store_prototype(force_cal)
store_prototype(get_report)
store_prototype(resume_touch)
show_store_prototype(report_type)
show_store_prototype(fifoindex)
show_store_prototype(no_auto_cal)
show_store_prototype(read_report)

static struct attribute *attrs[] = {
	attrify(num_of_mapped_tx),
	attrify(num_of_mapped_rx),
	attrify(tx_mapping),
	attrify(rx_mapping),
	attrify(report_size),
	attrify(f54status),
	attrify(do_preparation),
	attrify(force_cal),
	attrify(get_report),
	attrify(resume_touch),
	attrify(report_type),
	attrify(fifoindex),
	attrify(no_auto_cal),
	attrify(read_report),
	attrify(short_circuit_test),
	attrify(open_circuit_test),
	attrify(raw_cap_data),
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data,
		unsigned short length);

static ssize_t test_sysfs_data_read(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static struct bin_attribute test_report_data = {
	.attr = {
		.name = "report_data",
		.mode = S_IRUGO,
	},
	.size = 0,
	.read = test_sysfs_data_read,
};

static struct synaptics_rmi4_f54_handle *f54;
static struct synaptics_rmi4_f55_handle *f55;

DECLARE_COMPLETION(test_remove_complete);

static bool test_report_type_valid(enum f54_report_types report_type)
{
	switch (report_type) {
	case F54_8BIT_IMAGE:
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_HIGH_RESISTANCE:
	case F54_TX_TO_TX_SHORTS:
	case F54_RX_TO_RX_SHORTS_1:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_RX_OPENS_1:
	case F54_TX_OPENS:
	case F54_TX_TO_GND_SHORTS:
	case F54_RX_TO_RX_SHORTS_2:
	case F54_RX_OPENS_2:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_NO_RX_COUPLING:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_TRX_OPENS:
	case F54_TRX_TO_GND_SHORTS:
	case F54_TRX_SHORTS:
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
		return true;
		break;
	default:
		f54->report_type = INVALID_REPORT_TYPE;
		f54->report_size = 0;
		return false;
	}
}

static void test_set_report_size(void)
{
	int retval;
	unsigned char tx = f54->tx_assigned;
	unsigned char rx = f54->rx_assigned;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	switch (f54->report_type) {
	case F54_8BIT_IMAGE:
		f54->report_size = tx * rx;
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_NO_RX_COUPLING:
	case F54_SENSOR_SPEED:
		f54->report_size = 2 * tx * rx;
		break;
	case F54_HIGH_RESISTANCE:
		f54->report_size = HIGH_RESISTANCE_DATA_SIZE;
		break;
	case F54_TX_TO_TX_SHORTS:
	case F54_TX_OPENS:
	case F54_TX_TO_GND_SHORTS:
		f54->report_size = (tx + 7) / 8;
		break;
	case F54_RX_TO_RX_SHORTS_1:
	case F54_RX_OPENS_1:
		if (rx < tx)
			f54->report_size = 2 * rx * rx;
		else
			f54->report_size = 2 * tx * rx;
		break;
	case F54_FULL_RAW_CAP_MIN_MAX:
		f54->report_size = FULL_RAW_CAP_MIN_MAX_DATA_SIZE;
		break;
	case F54_RX_TO_RX_SHORTS_2:
	case F54_RX_OPENS_2:
		if (rx <= tx)
			f54->report_size = 0;
		else
			f54->report_size = 2 * rx * (rx - tx);
		break;
	case F54_ADC_RANGE:
		if (f54->query.has_signal_clarity) {
			retval = synaptics_rmi4_i2c_read(rmi4_data,
					f54->control.reg_41->address,
					f54->control.reg_41->data,
					sizeof(f54->control.reg_41->data));
			if (retval < 0) {
				dev_dbg(&rmi4_data->i2c_client->dev,
						"%s: Failed to read control reg_41\n",
						__func__);
				f54->report_size = 0;
				break;
			}
			if (!f54->control.reg_41->no_signal_clarity) {
				if (tx % 4)
					tx += 4 - (tx % 4);
			}
		}
		f54->report_size = 2 * tx * rx;
		break;
	case F54_TRX_OPENS:
	case F54_TRX_TO_GND_SHORTS:
	case F54_TRX_SHORTS:
		f54->report_size = TRX_OPEN_SHORT_DATA_SIZE;
		break;
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
		f54->report_size = 4 * (tx + rx);
		break;
	default:
		f54->report_size = 0;
	}

	return;
}

static int test_set_interrupt(bool set)
{
	int retval;
	unsigned char ii;
	unsigned char zero = 0x00;
	unsigned char *intr_mask;
	unsigned short f01_ctrl_reg;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	intr_mask = rmi4_data->intr_mask;
	f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (!set) {
		retval = synaptics_rmi4_i2c_write(rmi4_data,
				f01_ctrl_reg,
				&zero,
				sizeof(zero));
		if (retval < 0)
			return retval;
	}

	for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
		if (intr_mask[ii] != 0x00) {
			f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + ii;
			if (set) {
				retval = synaptics_rmi4_i2c_write(rmi4_data,
						f01_ctrl_reg,
						&zero,
						sizeof(zero));
				if (retval < 0)
					return retval;
			} else {
				retval = synaptics_rmi4_i2c_write(rmi4_data,
						f01_ctrl_reg,
						&(intr_mask[ii]),
						sizeof(intr_mask[ii]));
				if (retval < 0)
					return retval;
			}
		}
	}

	f01_ctrl_reg = rmi4_data->f01_ctrl_base_addr + 1 + f54->intr_reg_num;

	if (set) {
		retval = synaptics_rmi4_i2c_write(rmi4_data,
				f01_ctrl_reg,
				&f54->intr_mask,
				1);
		if (retval < 0)
			return retval;
	}

	return 0;
}

static int test_wait_for_command_completion(void)
{
	int retval;
	unsigned char value;
	unsigned char timeout_count;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	timeout_count = 0;
	do {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				f54->command_base_addr,
				&value,
				sizeof(value));
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to read command register\n",
					__func__);
			return retval;
		}

		if (value == 0x00)
			break;

		msleep(100);
		timeout_count++;
	} while (timeout_count < COMMAND_TIMEOUT_100MS);

	if (timeout_count == COMMAND_TIMEOUT_100MS) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Timed out waiting for command completion\n",
				__func__);
		return -ETIMEDOUT;
	}

	return 0;
}

static int test_do_command(unsigned char command)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write command\n",
				__func__);
		return retval;
	}

	retval = test_wait_for_command_completion();
	if (retval < 0)
		return retval;

	return 0;
}

static int test_do_preparation(void)
{
	int retval;
	unsigned char value;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	if (f54->query.touch_controller_family == 1)
		disable_cbc(reg_7);
	else if (f54->query.has_ctrl88 == 1)
		disable_cbc(reg_88);

	if (f54->query.has_0d_acquisition_control)
		disable_cbc(reg_57);

	if (f54->query.has_signal_clarity) {
		value = 1;
		retval = synaptics_rmi4_i2c_write(rmi4_data,
				f54->control.reg_41->address,
				&value,
				sizeof(f54->control.reg_41->data));
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to disable signal clarity\n",
					__func__);
			return retval;
		}
	}

	retval = test_do_command(COMMAND_FORCE_UPDATE);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to do force update\n",
				__func__);
		return retval;
	}

	retval = test_do_command(COMMAND_FORCE_CAL);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to do force cal\n",
				__func__);
		return retval;
	}

	return 0;
}

static int test_check_for_idle_status(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	switch (f54->status) {
	case STATUS_IDLE:
		retval = 0;
		break;
	case STATUS_BUSY:
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Status busy\n",
				__func__);
		retval = -EINVAL;
		break;
	case STATUS_ERROR:
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Status error\n",
				__func__);
		retval = -EINVAL;
		break;
	default:
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Invalid status (%d)\n",
				__func__, f54->status);
		retval = -EINVAL;
	}

	return retval;
}

static void test_timeout_work(struct work_struct *work)
{
	int retval;
	unsigned char command;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	mutex_lock(&f54->status_mutex);

	if (f54->status == STATUS_BUSY) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				f54->command_base_addr,
				&command,
				sizeof(command));
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to read command register\n",
					__func__);
		} else if (command & COMMAND_GET_REPORT) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Report type not supported by FW\n",
					__func__);
		} else {
			queue_work(f54->test_report_workqueue,
					&f54->test_report_work);
			goto exit;
		}
		f54->status = STATUS_ERROR;
		f54->report_size = 0;
	}

exit:
	mutex_unlock(&f54->status_mutex);

	return;
}

static enum hrtimer_restart test_get_report_timeout(struct hrtimer *timer)
{
	schedule_work(&(f54->timeout_work));

	return HRTIMER_NORESTART;
}

static ssize_t test_sysfs_num_of_mapped_tx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->tx_assigned);
}

static ssize_t test_sysfs_num_of_mapped_rx_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->rx_assigned);
}

static ssize_t test_sysfs_tx_mapping_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cnt;
	int count = 0;
	unsigned char ii;
	unsigned char tx_num;
	unsigned char tx_electrodes = f54->query.num_of_tx_electrodes;

	if (!f55)
		return -EINVAL;

	for (ii = 0; ii < tx_electrodes; ii++) {
		tx_num = f55->tx_assignment[ii];
		if (tx_num == 0xff)
			cnt = snprintf(buf, PAGE_SIZE - count, "xx ");
		else
			cnt = snprintf(buf, PAGE_SIZE - count, "%02u ", tx_num);
		buf += cnt;
		count += cnt;
	}

	snprintf(buf, PAGE_SIZE - count, "\n");
	count++;

	return count;
}

static ssize_t test_sysfs_rx_mapping_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cnt;
	int count = 0;
	unsigned char ii;
	unsigned char rx_num;
	unsigned char rx_electrodes = f54->query.num_of_rx_electrodes;

	if (!f55)
		return -EINVAL;

	for (ii = 0; ii < rx_electrodes; ii++) {
		rx_num = f55->rx_assignment[ii];
		if (rx_num == 0xff)
			cnt = snprintf(buf, PAGE_SIZE - count, "xx ");
		else
			cnt = snprintf(buf, PAGE_SIZE - count, "%02u ", rx_num);
		buf += cnt;
		count += cnt;
	}

	snprintf(buf, PAGE_SIZE - count, "\n");
	count++;

	return count;
}

static ssize_t test_sysfs_report_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", (unsigned int)f54->report_size);
}

static ssize_t test_sysfs_f54status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;

	mutex_lock(&f54->status_mutex);

	retval = snprintf(buf, PAGE_SIZE, "%u\n", f54->status);

	mutex_unlock(&f54->status_mutex);

	return retval;
}

static ssize_t test_sysfs_short_circuit_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char report_type[10] = { 0};
	int ii;
	
	sprintf(report_type, "%d", F54_TX_TO_TX_SHORTS);
	test_sysfs_read_report_store(dev, attr, report_type, sizeof(report_type));
	
	for (ii = 0; ii < f54->report_size; ii++) {
		if(f54->report_data[ii]){
			return snprintf(buf, PAGE_SIZE, "TP Short Circuit Detected,\nType-5,%03d: 0x%02x\n",
					ii, f54->report_data[ii]);
		}
	}
	sprintf(report_type, "%d", F54_TRX_SHORTS);
	test_sysfs_read_report_store(dev, attr, report_type, sizeof(report_type));
	
	for (ii = 0; ii < f54->report_size; ii++) {
		if(f54->report_data[ii]){
			return snprintf(buf, PAGE_SIZE, "TP Short Circuit Detected,\nType-26,%03d: 0x%02x\n",
					ii, f54->report_data[ii]);
		}
	}
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static short open_circuit_threshold_ofilm[15][26] = {
#if 0
2535,2519,2492,2467,2443,2417,2395,2369,2348,2322,2305,1924,1891,1864,1826,1807,1783,1761,1740,1724,1708,1695,1676,1668,1658,14,
2748,2733,2699,2666,2637,2606,2580,2551,2526,2496,2474,2212,2180,2152,2112,2094,2070,2048,2025,2008,1992,1978,1958,1951,1945,191,
2693,2680,2643,2608,2578,2543,2515,2482,2454,2422,2396,2195,2150,2122,2082,2062,2038,2014,1992,1974,1957,1942,1921,1913,1931,1719,
2624,2606,2571,2537,2508,2475,2447,2414,2387,2355,2329,2161,2131,2104,2066,2048,2026,2004,1984,1967,1951,1937,1917,1910,1927,1851,
2627,2612,2574,2541,2512,2479,2450,2417,2391,2360,2334,2199,2171,2147,2109,2093,2072,2053,2034,2019,2005,1993,1975,1969,1965,138,
2688,2670,2630,2592,2558,2519,2487,2450,2420,2385,2355,2253,2223,2197,2158,2141,2119,2112,2079,2064,2049,2037,2018,2012,2011,68,
2659,2647,2607,2568,2532,2494,2460,2424,2393,2357,2327,2268,2240,2215,2179,2163,2155,2123,2104,2090,2076,2065,2047,2042,2040,190,
2589,2573,2533,2493,2458,2420,2387,2350,2320,2284,2254,2232,2204,2180,2145,2142,2108,2089,2070,2056,2042,2030,2012,2007,2025,1838,
2593,2580,2541,2503,2469,2432,2399,2363,2333,2299,2269,2281,2256,2235,2203,2190,2171,2154,2138,2126,2126,2103,2087,2083,2077,139,
2660,2643,2601,2559,2520,2477,2442,2402,2370,2332,2300,2350,2323,2302,2269,2255,2236,2218,2201,2188,2175,2164,2148,2144,2140,83,
2629,2612,2569,2528,2491,2449,2414,2376,2343,2306,2275,2363,2338,2318,2288,2276,2257,2241,2225,2212,2200,2203,2175,2172,2167,190,
2562,2544,2502,2461,2425,2384,2350,2311,2280,2244,2212,2336,2311,2292,2274,2248,2231,2214,2198,2186,2173,2163,2149,2145,2161,1830,
2572,2566,2510,2469,2432,2392,2357,2318,2286,2249,2217,2379,2355,2337,2307,2295,2278,2262,2246,2234,2220,2212,2198,2195,2210,1798,
2644,2626,2583,2540,2501,2458,2421,2382,2349,2310,2277,2500,2470,2454,2425,2415,2401,2388,2374,2363,2351,2344,2331,2329,2321,233,
2372,2334,2290,2246,2205,2162,2124,2082,2050,2012,1977,2309,2318,2267,2238,2227,2212,2198,2186,2174,2163,2155,2143,2141,2133,147
#else
2586,2578,2548,2518,2495,2466,2441,2409,2390,2355,2334,1938,1904,1873,1832,1809,1778,1753,1727,1699,1673,1651,1610,1584,1550,23,
2799,2803,2776,2740,2711,2676,2645,2614,2593,2557,2528,2257,2223,2193,2149,2127,2095,2065,2040,2009,1976,1952,1908,1881,1844,216,
2753,2755,2721,2683,2650,2612,2575,2537,2510,2472,2437,2230,2182,2151,2107,2085,2056,2021,1994,1962,1931,1913,1869,1843,1827,1793,
2693,2692,2659,2619,2585,2546,2507,2473,2451,2412,2380,2208,2174,2140,2099,2080,2053,2025,2001,1971,1938,1919,1877,1852,1839,1939,
2692,2692,2658,2621,2587,2549,2508,2476,2454,2415,2384,2249,2216,2185,2145,2129,2103,2078,2057,2030,2006,1992,1952,1929,1898,172,
2755,2755,2718,2675,2638,2596,2550,2514,2490,2446,2410,2310,2277,2248,2206,2188,2158,2143,2107,2084,2061,2048,2009,1986,1959,84,
2714,2711,2673,2631,2593,2551,2505,2470,2442,2399,2364,2309,2280,2252,2212,2193,2179,2138,2116,2096,2076,2061,2024,2003,1975,212,
2646,2640,2600,2555,2517,2474,2429,2393,2368,2325,2289,2275,2245,2219,2182,2176,2134,2104,2081,2064,2044,2031,1994,1974,1967,1935,
2654,2653,2611,2568,2529,2484,2436,2402,2379,2337,2301,2325,2298,2275,2241,2222,2196,2170,2146,2125,2122,2100,2069,2052,2025,172,
2714,2710,2669,2620,2578,2533,2483,2446,2421,2374,2335,2400,2371,2346,2310,2292,2267,2242,2221,2204,2187,2177,2146,2131,2106,102,
2678,2674,2634,2587,2545,2498,2450,2414,2389,2345,2306,2412,2387,2365,2333,2318,2298,2276,2258,2242,2224,2229,2187,2174,2150,212,
2618,2616,2577,2531,2489,2441,2394,2359,2334,2290,2251,2395,2369,2349,2330,2303,2282,2263,2247,2231,2210,2202,2177,2166,2160,1966,
2629,2637,2585,2538,2497,2453,2403,2368,2343,2298,2259,2445,2420,2401,2370,2357,2338,2319,2304,2289,2269,2257,2233,2224,2218,1918,
2689,2688,2648,2601,2559,2510,2459,2424,2398,2351,2308,2556,2526,2510,2481,2470,2454,2439,2427,2415,2400,2394,2371,2364,2341,268,
2405,2374,2335,2286,2244,2195,2144,2108,2081,2036,1994,2356,2365,2313,2283,2272,2256,2241,2229,2218,2204,2198,2178,2175,2164,167
#endif
};

static short open_circuit_threshold_lens[14][26] = {
1400,1350,1292,1249,1201,1165,1124,1093,1058,1045,991 ,974 ,956 ,950 ,956 ,958 ,967 ,979 ,991 ,1016,1059,1072,1107,1145,1187,155  
,1377,1337,1282,1239,1191,1154,1118,1086,1048,1019,975 ,953 ,935 ,927 ,929 ,925 ,944 ,936 ,943 ,961 ,984 ,1005,1034,1063,1116,1292 
,1385,1348,1296,1251,1202,1162,1128,1092,1057,1026,985 ,964 ,946 ,936 ,937 ,933 ,940 ,944 ,952 ,966 ,993 ,1025,1040,1069,1120,1285 
,1392,1359,1308,1267,1218,1180,1149,1118,1085,1056,1018,998 ,980 ,971 ,972 ,971 ,977 ,983 ,992 ,1007,1034,1052,1083,1124,1150,603  
,1401,1371,1323,1283,1235,1202,1168,1139,1107,1081,1039,1019,1012,990 ,993 ,990 ,997 ,997 ,1006,1019,1045,1065,1090,1117,1148,584  
,1406,1377,1333,1294,1252,1216,1184,1156,1125,1099,1056,1041,1019,1011,1013,1022,1014,1012,1020,1032,1056,1076,1102,1126,1154,635  
,1408,1379,1333,1296,1253,1215,1180,1151,1120,1094,1049,1033,1007,999 ,999 ,991 ,995 ,993 ,997 ,1019,1029,1040,1067,1092,1131,1454 
,1417,1389,1342,1304,1260,1225,1192,1159,1133,1109,1068,1046,1025,1015,1012,1007,1009,1008,1010,1019,1039,1052,1076,1101,1136,1408 
,1429,1404,1363,1326,1285,1253,1224,1193,1165,1147,1107,1091,1070,1060,1061,1056,1057,1068,1059,1068,1089,1103,1127,1150,1170,672  
,1433,1411,1370,1336,1299,1268,1238,1209,1181,1165,1127,1109,1089,1078,1075,1070,1072,1069,1074,1083,1103,1118,1151,1160,1176,643  
,1446,1429,1390,1355,1318,1287,1257,1231,1204,1187,1154,1134,1114,1106,1101,1095,1096,1095,1114,1108,1126,1142,1162,1180,1192,732  
,1458,1436,1395,1361,1323,1291,1257,1229,1201,1181,1147,1125,1106,1107,1091,1085,1085,1081,1083,1091,1107,1119,1140,1158,1181,1575 
,1476,1454,1419,1391,1359,1332,1304,1279,1252,1234,1206,1189,1165,1158,1167,1146,1145,1138,1139,1146,1158,1165,1180,1192,1217,1609 
,1488,1462,1432,1413,1384,1366,1333,1311,1290,1276,1260,1243,1216,1209,1209,1204,1203,1199,1199,1209,1224,1231,1247,1264,1292,1078
};

static short open_circuit_threshold_lens_gff[15][26] = {
#if 0
2534,2546,2525,2503,2480,2468,2447,2440,2419,2409,2402,2122,2094,2063,2042,2016,2004,1990,1983,1962,1957,1945,1945,1940,1907,-7  
,2774,2810,2786,2755,2727,2706,2689,2671,2640,2623,2602,2430,2412,2383,2369,2343,2329,2317,2308,2280,2284,2280,2282,2275,2228,115 
,2697,2729,2704,2675,2647,2628,2607,2586,2562,2548,2522,2404,2367,2343,2331,2303,2289,2270,2261,2244,2242,2235,2232,2223,2202,1797
,2638,2668,2647,2623,2600,2581,2558,2541,2527,2503,2489,2374,2350,2329,2313,2291,2277,2261,2256,2237,2230,2218,2223,2223,2200,1893
,2661,2697,2673,2647,2628,2609,2591,2567,2548,2534,2515,2421,2397,2374,2360,2334,2324,2298,2301,2282,2277,2270,2275,2275,2237,113 
,2727,2765,2741,2718,2699,2678,2661,2633,2602,2588,2562,2487,2463,2440,2426,2395,2381,2376,2357,2336,2341,2331,2336,2336,2294,51  
,2685,2715,2694,2671,2642,2623,2598,2572,2541,2529,2508,2461,2449,2421,2402,2381,2376,2348,2346,2320,2324,2322,2331,2329,2277,131 
,2619,2640,2616,2588,2562,2536,2515,2501,2463,2454,2430,2404,2383,2360,2350,2343,2308,2294,2289,2270,2275,2268,2280,2275,2256,1943
,2631,2671,2647,2619,2600,2581,2558,2539,2508,2492,2477,2461,2449,2426,2416,2393,2381,2364,2355,2346,2360,2341,2346,2348,2296,113 
,2708,2744,2720,2689,2659,2638,2619,2595,2569,2550,2525,2543,2525,2501,2494,2466,2452,2440,2435,2414,2412,2407,2412,2412,2371,65  
,2668,2699,2682,2652,2628,2602,2586,2567,2536,2515,2496,2534,2515,2494,2492,2466,2452,2435,2430,2416,2409,2416,2416,2412,2369,131 
,2605,2633,2614,2595,2572,2548,2520,2501,2473,2456,2440,2496,2477,2454,2456,2423,2409,2393,2383,2376,2371,2364,2379,2374,2334,1882
,2628,2671,2635,2614,2593,2572,2543,2522,2489,2473,2452,2525,2515,2492,2477,2456,2447,2440,2428,2416,2409,2402,2412,2402,2362,1839
,2706,2737,2711,2689,2661,2638,2605,2586,2555,2534,2506,2621,2609,2581,2583,2562,2548,2536,2529,2517,2513,2508,2520,2508,2447,179 
,2407,2404,2362,2336,2317,2298,2273,2261,2228,2216,2185,2421,2430,2374,2364,2346,2334,2320,2327,2313,2306,2298,2308,2303,2261,143
#else
2661,2658,2649,2645,2636,2629,2621,2615,2603,2599,2600,2329,2317,2303,2289,2291,2267,2256,2248,2238,2230,2225,2223,2217,2215,-11,
2810,2796,2784,2773,2761,2748,2740,2728,2716,2706,2694,2568,2571,2546,2534,2525,2513,2503,2495,2487,2479,2474,2471,2467,2464,37,
2813,2798,2784,2772,2757,2742,2732,2718,2705,2694,2678,2571,2559,2546,2534,2523,2511,2501,2504,2482,2474,2468,2464,2461,2489,2211,
2800,2783,2768,2754,2740,2724,2713,2698,2683,2671,2656,2568,2558,2544,2530,2521,2509,2498,2490,2480,2471,2465,2462,2471,2487,2250,
2784,2768,2753,2739,2724,2708,2697,2683,2669,2656,2643,2579,2569,2569,2544,2535,2525,2515,2507,2500,2492,2487,2485,2483,2484,93,
2783,2768,2753,2738,2723,2707,2696,2682,2667,2655,2641,2596,2586,2574,2562,2554,2557,2535,2527,2520,2513,2509,2507,2505,2507,61,
2791,2776,2761,2745,2729,2713,2700,2685,2671,2657,2642,2630,2609,2597,2585,2578,2568,2559,2552,2544,2538,2534,2533,2530,2531,103,
2775,2757,2741,2723,2706,2689,2676,2660,2644,2630,2614,2609,2599,2588,2576,2568,2558,2548,2541,2533,2526,2522,2520,2518,2546,2326,
2759,2745,2729,2713,2697,2680,2669,2654,2640,2627,2612,2627,2619,2609,2598,2590,2582,2574,2568,2562,2556,2556,2552,2551,2549,96,
2759,2744,2728,2712,2696,2679,2668,2653,2638,2626,2610,2646,2638,2628,2617,2611,2603,2595,2590,2584,2578,2580,2576,2581,2574,64,
2768,2751,2735,2719,2703,2686,2675,2659,2645,2632,2616,2671,2663,2654,2644,2638,2630,2623,2617,2612,2607,2603,2602,2601,2610,103,
2758,2740,2724,2707,2690,2672,2659,2643,2628,2616,2599,2675,2667,2658,2648,2642,2635,2640,2621,2615,2610,2606,2605,2604,2627,2262,
2747,2729,2712,2696,2680,2662,2650,2635,2620,2608,2592,2693,2685,2676,2668,2662,2654,2647,2641,2636,2631,2627,2627,2626,2648,2279,
2752,2738,2723,2708,2693,2676,2666,2652,2638,2627,2612,2731,2725,2719,2724,2707,2703,2697,2693,2690,2687,2683,2683,2681,2676,110,
2504,2488,2472,2456,2440,2423,2411,2395,2381,2368,2353,2622,2614,2607,2597,2594,2588,2583,2578,2576,2584,2571,2569,2570,2564,144
#endif
};

static ssize_t test_sysfs_open_circuit_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#if 1
	char report_type[10] = { 0};
	int ii;
	unsigned int jj;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	short *report_data_16;
	
	sprintf(report_type, "%d", F54_FULL_RAW_CAP_NO_RX_COUPLING);
	test_sysfs_read_report_store(dev, attr, report_type, sizeof(report_type));
	
	report_data_16 = (short *)f54->report_data;

	for (ii = 0; ii < tx_num; ii++) {
		for (jj = 0; jj < rx_num; jj++) {

			if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "LL96_01")){ //ofilm
				if(jj == 25 &&( ii == 0 || ii == 1 || ii == 4 || ii == 5 || ii == 6 ||
					ii == 8 || ii == 9 || ii == 10 || ii == 13 || ii == 14) ){ //skip empty node in Button area
					
				}else{
					if( *report_data_16 > open_circuit_threshold_ofilm[ii][jj]*(100+25)/100 ||
					     *report_data_16 < open_circuit_threshold_ofilm[ii][jj]*(100-25)/100 )
					     return snprintf(buf, PAGE_SIZE, "TP Open Circuit Detected,\nTx:%d,Rx:%d,raw:%d,threshold:%d\n",
					ii, jj, *report_data_16, open_circuit_threshold_ofilm[ii][jj]);
					
				}
			}
			else if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "LL96_00")){ //lens
				return snprintf(buf, PAGE_SIZE, "Pass\n");//PVT not use it again,so pass directly
				if(jj == 25 &&( ii == 0 || ii == 3 || ii == 4 || ii == 5 ||
					ii == 8 || ii == 9 || ii == 10 || ii == 13) ){ //skip empty node in Button area
					
				}else{
					
					if( *report_data_16 > open_circuit_threshold_lens[ii][jj]*(100+25)/100  ||
					     *report_data_16 < open_circuit_threshold_lens[ii][jj]*(100-25)/100  )
					     return snprintf(buf, PAGE_SIZE, "TP Open Circuit Detected,\nTx:%d,Rx:%d,raw:%d,threshold:%d\n",
					ii, jj, *report_data_16, open_circuit_threshold_lens[ii][jj]);
				}
			}
			else if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "S1_L02")){ //lens GFF
				if(jj == 25 &&( ii == 0 || ii == 1 || ii == 4 || ii == 5 || ii == 6 ||
					ii == 8 || ii == 9 || ii == 10 || ii == 13 || ii == 14) ){ //skip empty node in Button area
					
				}else{
					if( *report_data_16 > open_circuit_threshold_lens_gff[ii][jj]*(100+25)/100 ||
					     *report_data_16 < open_circuit_threshold_lens_gff[ii][jj]*(100-25)/100 )
					     return snprintf(buf, PAGE_SIZE, "TP Open Circuit Detected,\nTx:%d,Rx:%d,raw:%d,threshold:%d\n",
					ii, jj, *report_data_16, open_circuit_threshold_lens_gff[ii][jj]);
					
				}
			}
			else
				return snprintf(buf, PAGE_SIZE, "Invalid product id:%s\n", f54->rmi4_data->rmi4_mod_info.product_id_string);
			
			report_data_16++;

		}

	}
#endif
	return snprintf(buf, PAGE_SIZE, "Pass\n");
}

static ssize_t test_sysfs_raw_cap_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char report_type[10] = { 0};
	int ii;
	unsigned int jj;
	int cnt;
	int count = 0;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	short *report_data_16;

	unsigned int sum = 0;
	short max = 0;
	short min = 0;
	
	sprintf(report_type, "%d", F54_FULL_RAW_CAP_NO_RX_COUPLING);
	test_sysfs_read_report_store(dev, attr, report_type, sizeof(report_type));
	
	report_data_16 = (short *)f54->report_data;

	for (ii = 0; ii < tx_num; ii++) {
		for (jj = 0; jj < rx_num; jj++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d ",
					*report_data_16);

			if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "LL96_01")){ //ofilm
				if(jj == 25 &&( ii == 0 || ii == 1 || ii == 4 || ii == 5 || ii == 6 ||
					ii == 8 || ii == 9 || ii == 10 || ii == 13 || ii == 14) ){ //skip empty node in Button area
					
				}else{
					sum+=*report_data_16;
					
					if(max < *report_data_16)
						max = *report_data_16;

					if(ii == 0 && jj == 0)
						min = *report_data_16;
					else if( *report_data_16 < min)
						min = *report_data_16;
				}
			}
			else if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "LL96_00")){ //lens
				if(jj == 25 &&( ii == 0 || ii == 3 || ii == 4 || ii == 5 ||
					ii == 8 || ii == 9 || ii == 10 || ii == 13) ){ //skip empty node in Button area
					
				}else{
					sum+=*report_data_16;
					
					if(max < *report_data_16)
						max = *report_data_16;

					if(ii == 0 && jj == 0)
						min = *report_data_16;
					else if( *report_data_16 < min)
						min = *report_data_16;
				}
			}
			else if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "S1_L02")){ //lens GFF
				if(jj == 25 &&( ii == 0 || ii == 1 || ii == 4 || ii == 5 || ii == 6 ||
					ii == 8 || ii == 9 || ii == 10 || ii == 13 || ii == 14) ){ //skip empty node in Button area
					
				}else{
					sum+=*report_data_16;
					
					if(max < *report_data_16)
						max = *report_data_16;

					if(ii == 0 && jj == 0)
						min = *report_data_16;
					else if( *report_data_16 < min)
						min = *report_data_16;
				}
			}
			else
				return snprintf(buf, PAGE_SIZE, "Invalid product id:%s\n", f54->rmi4_data->rmi4_mod_info.product_id_string);
			
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
			tx_num, rx_num);
	buf += cnt;
	count += cnt;

	cnt = snprintf(buf, PAGE_SIZE - count, "product_id:%s\n",
			f54->rmi4_data->rmi4_mod_info.product_id_string);
	buf += cnt;
	count += cnt;

	if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "LL96_01")){
		cnt = snprintf(buf, PAGE_SIZE - count, "max = %d, min = %d, average = %d\n",
				max, min, sum/(tx_num*rx_num -10));
		buf += cnt;
		count += cnt;
	}
	else if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "LL96_00")){
		cnt = snprintf(buf, PAGE_SIZE - count, "max = %d, min = %d, average = %d\n",
				max, min, sum/(tx_num*rx_num -8));
		buf += cnt;
		count += cnt;
	}
	else if(!strcmp(f54->rmi4_data->rmi4_mod_info.product_id_string, "S1_L02")){
		cnt = snprintf(buf, PAGE_SIZE - count, "max = %d, min = %d, average = %d\n",
				max, min, sum/(tx_num*rx_num -10));
		buf += cnt;
		count += cnt;
	}
	else
		return snprintf(buf, PAGE_SIZE, "Invalid product id:%s\n", f54->rmi4_data->rmi4_mod_info.product_id_string);

	return count;
}

static ssize_t test_sysfs_do_preparation_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	mutex_lock(&f54->status_mutex);

	retval = test_check_for_idle_status();
	if (retval < 0)
		goto exit;

	retval = test_do_preparation();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to do preparation\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&f54->status_mutex);

	return retval;
}

static ssize_t test_sysfs_force_cal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	mutex_lock(&f54->status_mutex);

	retval = test_check_for_idle_status();
	if (retval < 0)
		goto exit;

	retval = test_do_command(COMMAND_FORCE_CAL);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to do force cal\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&f54->status_mutex);

	return retval;
}

static ssize_t test_sysfs_get_report_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char command;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	mutex_lock(&f54->status_mutex);

	retval = test_check_for_idle_status();
	if (retval < 0)
		goto exit;

	if (!test_report_type_valid(f54->report_type)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Invalid report type\n",
				__func__);
		retval = -EINVAL;
		goto exit;
	}

	test_set_interrupt(true);

	command = (unsigned char)COMMAND_GET_REPORT;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f54->command_base_addr,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write get report command\n",
				__func__);
		goto exit;
	}

	f54->status = STATUS_BUSY;

	f54->data_pos = 0;

	hrtimer_start(&f54->watchdog,
			ktime_set(GET_REPORT_TIMEOUT_S, 0),
			HRTIMER_MODE_REL);

	retval = count;

exit:
	mutex_unlock(&f54->status_mutex);

	return retval;
}

static ssize_t test_sysfs_resume_touch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long setting;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting != 1)
		return -EINVAL;

	test_set_interrupt(false);

	return count;
}

static ssize_t test_sysfs_report_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->report_type);
}

static ssize_t test_sysfs_report_type_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	mutex_lock(&f54->status_mutex);

	retval = test_check_for_idle_status();
	if (retval < 0)
		goto exit;

	if (!test_report_type_valid((enum f54_report_types)setting)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Report type not supported by driver\n",
				__func__);
		retval = -EINVAL;
		goto exit;
	}

	f54->report_type = (enum f54_report_types)setting;
	data = (unsigned char)setting;
	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f54->data_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write report type\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	mutex_unlock(&f54->status_mutex);

	return retval;
}

static ssize_t test_sysfs_fifoindex_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int retval;
	unsigned char data[2];
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f54->data_base_addr + REPORT_INDEX_OFFSET,
			data,
			sizeof(data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read report index\n",
				__func__);
		return retval;
	}

	batohs(&f54->fifoindex, data);

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->fifoindex);
}

static ssize_t test_sysfs_fifoindex_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data[2];
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	f54->fifoindex = setting;

	hstoba(data, (unsigned short)setting);

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f54->data_base_addr + REPORT_INDEX_OFFSET,
			data,
			sizeof(data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write report index\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t test_sysfs_no_auto_cal_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", f54->no_auto_cal);
}

static ssize_t test_sysfs_no_auto_cal_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char data;
	unsigned long setting;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = sstrtoul(buf, 10, &setting);
	if (retval)
		return retval;

	if (setting > 1)
		return -EINVAL;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f54->control_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read no auto cal setting\n",
				__func__);
		return retval;
	}

	if (setting)
		data |= CONTROL_NO_AUTO_CAL;
	else
		data &= ~CONTROL_NO_AUTO_CAL;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f54->control_base_addr,
			&data,
			sizeof(data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write no auto cal setting\n",
				__func__);
		return retval;
	}

	f54->no_auto_cal = (setting == 1);

	return count;
}

static ssize_t test_sysfs_read_report_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int ii;
	unsigned int jj;
	int cnt;
	int count = 0;
	int tx_num = f54->tx_assigned;
	int rx_num = f54->rx_assigned;
	char *report_data_8;
	short *report_data_16;
	int *report_data_32;
	unsigned int *report_data_u32;

	switch (f54->report_type) {
	case F54_8BIT_IMAGE:
		report_data_8 = (char *)f54->report_data;
		for (ii = 0; ii < f54->report_size; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%03d: %d\n",
					ii, *report_data_8);
			report_data_8++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_NO_RX_COUPLING:
	case F54_SENSOR_SPEED:
		report_data_16 = (short *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "tx = %d\nrx = %d\n",
				tx_num, rx_num);
		buf += cnt;
		count += cnt;

		for (ii = 0; ii < tx_num; ii++) {
			for (jj = 0; jj < (rx_num - 1); jj++) {
				cnt = snprintf(buf, PAGE_SIZE - count, "%-4d ",
						*report_data_16);
				report_data_16++;
				buf += cnt;
				count += cnt;
			}
			cnt = snprintf(buf, PAGE_SIZE - count, "%-4d\n",
					*report_data_16);
			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_HIGH_RESISTANCE:
	case F54_FULL_RAW_CAP_MIN_MAX:
		report_data_16 = (short *)f54->report_data;
		for (ii = 0; ii < f54->report_size; ii += 2) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%03d: %d\n",
					ii / 2, *report_data_16);
			report_data_16++;
			buf += cnt;
			count += cnt;
		}
		break;
	case F54_ABS_RAW_CAP:
		report_data_u32 = (unsigned int *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "rx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5u",
					*report_data_u32);
			report_data_u32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "tx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5u",
					*report_data_u32);
			report_data_u32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
		break;
	case F54_ABS_DELTA_CAP:
		report_data_32 = (int *)f54->report_data;
		cnt = snprintf(buf, PAGE_SIZE - count, "rx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < rx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5d",
					*report_data_32);
			report_data_32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "tx ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "     %2d", ii);
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;

		cnt = snprintf(buf, PAGE_SIZE - count, "   ");
		buf += cnt;
		count += cnt;
		for (ii = 0; ii < tx_num; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "  %5d",
					*report_data_32);
			report_data_32++;
			buf += cnt;
			count += cnt;
		}
		cnt = snprintf(buf, PAGE_SIZE - count, "\n");
		buf += cnt;
		count += cnt;
		break;
	default:
		for (ii = 0; ii < f54->report_size; ii++) {
			cnt = snprintf(buf, PAGE_SIZE - count, "%03d: 0x%02x\n",
					ii, f54->report_data[ii]);
			buf += cnt;
			count += cnt;
		}
		break;
	}

	snprintf(buf, PAGE_SIZE - count, "\n");
	count++;

	return count;
}

static ssize_t test_sysfs_read_report_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned char timeout = GET_REPORT_TIMEOUT_S * 10;
	unsigned char timeout_count;
	const char cmd[] = {'1', 0};
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = test_sysfs_report_type_store(dev, attr, buf, count);
	if (retval < 0)
		goto exit;

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
		break;
	default:
		retval = test_sysfs_do_preparation_store(dev, attr, cmd, 1);
		if (retval < 0)
			goto exit;
		break;
	}

	retval = test_sysfs_get_report_store(dev, attr, cmd, 1);
	if (retval < 0)
		goto exit;

	timeout_count = 0;
	do {
		if (f54->status != STATUS_BUSY)
			break;
		msleep(100);
		timeout_count++;
	} while (timeout_count < timeout);

	if ((f54->status != STATUS_IDLE) || (f54->report_size == 0)) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read report\n",
				__func__);
		retval = -EINVAL;
		goto exit;
	}

	switch (f54->report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_ABS_RAW_CAP:
	case F54_ABS_DELTA_CAP:
		retval = test_sysfs_resume_touch_store(dev, attr, cmd, 1);
		if (retval < 0)
			goto exit;
		break;
	default:
		rmi4_data->reset_device(rmi4_data);
		break;
	}

	return count;

exit:
	rmi4_data->reset_device(rmi4_data);

	return retval;
}

static ssize_t test_sysfs_data_read(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	int retval;
	unsigned int read_size;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	mutex_lock(&f54->status_mutex);

	retval = test_check_for_idle_status();
	if (retval < 0)
		goto exit;

	if (!f54->report_data) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Report type %d data not available\n",
				__func__, f54->report_type);
		retval = -EINVAL;
		goto exit;
	}

	if ((f54->data_pos + count) > f54->report_size)
		read_size = f54->report_size - f54->data_pos;
	else
		read_size = min(count,  f54->report_size);

	memcpy(buf, f54->report_data + f54->data_pos, read_size);
	f54->data_pos += read_size;
	retval = read_size;

exit:
	mutex_unlock(&f54->status_mutex);

	return retval;
}

static void test_report_work(struct work_struct *work)
{
	int retval;
	unsigned char report_index[2];
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	mutex_lock(&f54->status_mutex);

	if (f54->status != STATUS_BUSY)
		goto exit;

	retval = test_wait_for_command_completion();
	if (retval < 0) {
		f54->status = STATUS_ERROR;
		goto exit;
	}

	test_set_report_size();
	if (f54->report_size == 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Report data size = 0\n",
				__func__);
		f54->status = STATUS_ERROR;
		goto exit;
	}

	if (f54->data_buffer_size < f54->report_size) {
		if (f54->data_buffer_size)
			kfree(f54->report_data);
		f54->report_data = kzalloc(f54->report_size, GFP_KERNEL);
		if (!f54->report_data) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to alloc mem for data buffer\n",
					__func__);
			f54->data_buffer_size = 0;
			f54->status = STATUS_ERROR;
			goto exit;
		}
		f54->data_buffer_size = f54->report_size;
	}

	report_index[0] = 0;
	report_index[1] = 0;

	retval = synaptics_rmi4_i2c_write(rmi4_data,
			f54->data_base_addr + REPORT_INDEX_OFFSET,
			report_index,
			sizeof(report_index));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write report data index\n",
				__func__);
		f54->status = STATUS_ERROR;
		goto exit;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f54->data_base_addr + REPORT_DATA_OFFSET,
			f54->report_data,
			f54->report_size);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read report data\n",
				__func__);
		f54->status = STATUS_ERROR;
		goto exit;
	}

	f54->status = STATUS_IDLE;

exit:
	mutex_unlock(&f54->status_mutex);

	if (f54->status == STATUS_ERROR)
		f54->report_size = 0;

	return;
}

static void test_remove_sysfs(void)
{
	sysfs_remove_group(f54->sysfs_dir, &attr_group);
	sysfs_remove_bin_file(f54->sysfs_dir, &test_report_data);
	kobject_put(f54->sysfs_dir);

	return;
}

static int test_set_sysfs(void)
{
	int retval;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	f54->sysfs_dir = kobject_create_and_add(SYSFS_FOLDER_NAME,
			&(rmi4_data->i2c_client->dev.kobj));
	if (!f54->sysfs_dir) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs directory\n",
				__func__);
		goto exit_directory;
	}

	retval = sysfs_create_bin_file(f54->sysfs_dir, &test_report_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_bin_file;
	}

	retval = sysfs_create_group(f54->sysfs_dir, &attr_group);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs attributes\n",
				__func__);
		goto exit_attributes;
	}

	return 0;

exit_attributes:
	sysfs_remove_group(f54->sysfs_dir, &attr_group);
	sysfs_remove_bin_file(f54->sysfs_dir, &test_report_data);

exit_bin_file:
	kobject_put(f54->sysfs_dir);

exit_directory:
	return -ENODEV;
}

static void test_free_control_mem(void)
{
	struct f54_control control = f54->control;

	kfree(control.reg_7);
	kfree(control.reg_41);
	kfree(control.reg_57);
	kfree(control.reg_88);

	return;
}

static int test_set_controls(void)
{
	unsigned char length;
	unsigned char num_of_sensing_freqs;
	unsigned short reg_addr = f54->control_base_addr;
	struct f54_control *control = &f54->control;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	num_of_sensing_freqs = f54->query.number_of_sensing_frequencies;

	/* control 0 */
	reg_addr += CONTROL_0_SIZE;

	/* control 1 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1))
		reg_addr += CONTROL_1_SIZE;

	/* control 2 */
	reg_addr += CONTROL_2_SIZE;

	/* control 3 */
	if (f54->query.has_pixel_touch_threshold_adjustment == 1)
		reg_addr += CONTROL_3_SIZE;

	/* controls 4 5 6 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1))
		reg_addr += CONTROL_4_6_SIZE;

	/* control 7 */
	if (f54->query.touch_controller_family == 1) {
		control->reg_7 = kzalloc(sizeof(*(control->reg_7)),
				GFP_KERNEL);
		if (!control->reg_7)
			goto exit_no_mem;
		control->reg_7->address = reg_addr;
		reg_addr += CONTROL_7_SIZE;
	}

	/* controls 8 9 */
	if ((f54->query.touch_controller_family == 0) ||
			(f54->query.touch_controller_family == 1))
		reg_addr += CONTROL_8_9_SIZE;

	/* control 10 */
	if (f54->query.has_interference_metric == 1)
		reg_addr += CONTROL_10_SIZE;

	/* control 11 */
	if (f54->query.has_ctrl11 == 1)
		reg_addr += CONTROL_11_SIZE;

	/* controls 12 13 */
	if (f54->query.has_relaxation_control == 1)
		reg_addr += CONTROL_12_13_SIZE;

	/* controls 14 15 16 */
	if (f54->query.has_sensor_assignment == 1) {
		reg_addr += CONTROL_14_SIZE;
		reg_addr += CONTROL_15_SIZE * f54->query.num_of_rx_electrodes;
		reg_addr += CONTROL_16_SIZE * f54->query.num_of_tx_electrodes;
	}

	/* controls 17 18 19 */
	if (f54->query.has_sense_frequency_control == 1) {
		reg_addr += CONTROL_17_SIZE * num_of_sensing_freqs;
		reg_addr += CONTROL_18_SIZE * num_of_sensing_freqs;
		reg_addr += CONTROL_19_SIZE * num_of_sensing_freqs;
	}

	/* control 20 */
	reg_addr += CONTROL_20_SIZE;

	/* control 21 */
	if (f54->query.has_sense_frequency_control == 1)
		reg_addr += CONTROL_21_SIZE;

	/* controls 22 23 24 25 26 */
	if (f54->query.has_firmware_noise_mitigation == 1)
		reg_addr += CONTROL_22_26_SIZE;

	/* control 27 */
	if (f54->query.has_iir_filter == 1)
		reg_addr += CONTROL_27_SIZE;

	/* control 28 */
	if (f54->query.has_firmware_noise_mitigation == 1)
		reg_addr += CONTROL_28_SIZE;

	/* control 29 */
	if (f54->query.has_cmn_removal == 1)
		reg_addr += CONTROL_29_SIZE;

	/* control 30 */
	if (f54->query.has_cmn_maximum == 1)
		reg_addr += CONTROL_30_SIZE;

	/* control 31 */
	if (f54->query.has_touch_hysteresis == 1)
		reg_addr += CONTROL_31_SIZE;

	/* controls 32 33 34 35 */
	if (f54->query.has_edge_compensation == 1)
		reg_addr += CONTROL_32_35_SIZE;

	/* control 36 */
	if ((f54->query.curve_compensation_mode == 1) ||
			(f54->query.curve_compensation_mode == 2)) {
		if (f54->query.curve_compensation_mode == 1) {
			length = max(f54->query.num_of_rx_electrodes,
					f54->query.num_of_tx_electrodes);
		} else if (f54->query.curve_compensation_mode == 2) {
			length = f54->query.num_of_rx_electrodes;
		}
		reg_addr += CONTROL_36_SIZE * length;
	}

	/* control 37 */
	if (f54->query.curve_compensation_mode == 2)
		reg_addr += CONTROL_37_SIZE * f54->query.num_of_tx_electrodes;

	/* controls 38 39 40 */
	if (f54->query.has_per_frequency_noise_control == 1) {
		reg_addr += CONTROL_38_SIZE * num_of_sensing_freqs;
		reg_addr += CONTROL_39_SIZE * num_of_sensing_freqs;
		reg_addr += CONTROL_40_SIZE * num_of_sensing_freqs;
	}

	/* control 41 */
	if (f54->query.has_signal_clarity == 1) {
		control->reg_41 = kzalloc(sizeof(*(control->reg_41)),
				GFP_KERNEL);
		if (!control->reg_41)
			goto exit_no_mem;
		control->reg_41->address = reg_addr;
		reg_addr += CONTROL_41_SIZE;
	}

	/* control 42 */
	if (f54->query.has_variance_metric == 1)
		reg_addr += CONTROL_42_SIZE;

	/* controls 43 44 45 46 47 48 49 50 51 52 53 54 */
	if (f54->query.has_multi_metric_state_machine == 1)
		reg_addr += CONTROL_43_54_SIZE;

	/* controls 55 56 */
	if (f54->query.has_0d_relaxation_control == 1)
		reg_addr += CONTROL_55_56_SIZE;

	/* control 57 */
	if (f54->query.has_0d_acquisition_control == 1) {
		control->reg_57 = kzalloc(sizeof(*(control->reg_57)),
				GFP_KERNEL);
		if (!control->reg_57)
			goto exit_no_mem;
		control->reg_57->address = reg_addr;
		reg_addr += CONTROL_57_SIZE;
	}

	/* control 58 */
	if (f54->query.has_0d_acquisition_control == 1)
		reg_addr += CONTROL_58_SIZE;

	/* control 59 */
	if (f54->query.has_h_blank == 1)
		reg_addr += CONTROL_59_SIZE;

	/* controls 60 61 62 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_60_62_SIZE;

	/* control 63 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1) ||
			(f54->query.has_slew_metric == 1) ||
			(f54->query.has_slew_option == 1) ||
			(f54->query.has_noise_mitigation2 == 1))
		reg_addr += CONTROL_63_SIZE;

	/* controls 64 65 66 67 */
	if (f54->query.has_h_blank == 1)
		reg_addr += CONTROL_64_67_SIZE * 7;
	else if ((f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_64_67_SIZE;

	/* controls 68 69 70 71 72 73 */
	if ((f54->query.has_h_blank == 1) ||
			(f54->query.has_v_blank == 1) ||
			(f54->query.has_long_h_blank == 1))
		reg_addr += CONTROL_68_73_SIZE;

	/* control 74 */
	if (f54->query.has_slew_metric == 1)
		reg_addr += CONTROL_74_SIZE;

	/* control 75 */
	if (f54->query.has_enhanced_stretch == 1)
		reg_addr += CONTROL_75_SIZE * num_of_sensing_freqs;

	/* control 76 */
	if (f54->query.has_startup_fast_relaxation == 1)
		reg_addr += CONTROL_76_SIZE;

	/* controls 77 78 */
	if (f54->query.has_esd_control == 1)
		reg_addr += CONTROL_77_78_SIZE;

	/* controls 79 80 81 82 83 */
	if (f54->query.has_noise_mitigation2 == 1)
		reg_addr += CONTROL_79_83_SIZE;

	/* controls 84 85 */
	if (f54->query.has_energy_ratio_relaxation == 1)
		reg_addr += CONTROL_84_85_SIZE;

	/* control 86 */
	if ((f54->query.has_query13 == 1) && (f54->query_13.has_ctrl86 == 1))
		reg_addr += CONTROL_86_SIZE;

	/* control 87 */
	if ((f54->query.has_query13 == 1) && (f54->query_13.has_ctrl87 == 1))
		reg_addr += CONTROL_87_SIZE;

	/* control 88 */
	if (f54->query.has_ctrl88 == 1) {
		control->reg_88 = kzalloc(sizeof(*(control->reg_88)),
				GFP_KERNEL);
		if (!control->reg_88)
			goto exit_no_mem;
		control->reg_88->address = reg_addr;
		reg_addr += CONTROL_88_SIZE;
	}

	return 0;

exit_no_mem:
	dev_err(&rmi4_data->i2c_client->dev,
			"%s: Failed to alloc mem for control registers\n",
			__func__);
	return -ENOMEM;
}

static int test_set_queries(void)
{
	int retval;
	unsigned char offset;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f54->query_base_addr,
			f54->query.data,
			sizeof(f54->query.data));
	if (retval < 0)
		return retval;

	offset = sizeof(f54->query.data);

	/* query 12 */
	if (f54->query.has_sense_frequency_control == 0)
		offset -= 1;

	/* query 13 */
	if (f54->query.has_query13) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_13.data,
				sizeof(f54->query_13.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 14 */
	if ((f54->query.has_query13) && (f54->query_13.has_ctrl87))
		offset += 1;

	/* query 15 */
	if (f54->query.has_query15) {
		retval = synaptics_rmi4_i2c_read(rmi4_data,
				f54->query_base_addr + offset,
				f54->query_15.data,
				sizeof(f54->query_15.data));
		if (retval < 0)
			return retval;
		offset += 1;
	}

	/* query 16 */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f54->query_base_addr + offset,
			f54->query_16.data,
			sizeof(f54->query_16.data));
	if (retval < 0)
		return retval;
	offset += 1;

	/* query 17 */
	if (f54->query_16.has_query17)
		offset += 1;

	/* query 18 */
	if (f54->query_16.has_ctrl94_query18)
		offset += 1;

	/* query 19 */
	if (f54->query_16.has_ctrl95_query19)
		offset += 1;

	/* query 20 */
	if ((f54->query.has_query15) && (f54->query_15.has_query20))
		offset += 1;

	/* query 21 */
	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f54->query_base_addr + offset,
			f54->query_21.data,
			sizeof(f54->query_21.data));
	if (retval < 0)
		return retval;

	return 0;
}

static void test_f54_set_regs(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned int intr_count,
		unsigned char page)
{
	unsigned char ii;
	unsigned char intr_offset;

	f54->query_base_addr = fd->query_base_addr | (page << 8);
	f54->control_base_addr = fd->ctrl_base_addr | (page << 8);
	f54->data_base_addr = fd->data_base_addr | (page << 8);
	f54->command_base_addr = fd->cmd_base_addr | (page << 8);

	f54->intr_reg_num = (intr_count + 7) / 8;
	if (f54->intr_reg_num != 0)
		f54->intr_reg_num -= 1;

	f54->intr_mask = 0;
	intr_offset = intr_count % 8;
	for (ii = intr_offset;
			ii < ((fd->intr_src_count & MASK_3BIT) +
			intr_offset);
			ii++) {
		f54->intr_mask |= 1 << ii;
	}

	return;
}

static void test_f55_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char ii;
	unsigned char rx_electrodes = f54->query.num_of_rx_electrodes;
	unsigned char tx_electrodes = f54->query.num_of_tx_electrodes;

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f55->query_base_addr,
			f55->query.data,
			sizeof(f55->query.data));
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read f55 query registers\n",
				__func__);
		return;
	}

	if (!f55->query.has_sensor_assignment)
		return;

	f55->tx_assignment = kzalloc(tx_electrodes, GFP_KERNEL);
	f55->rx_assignment = kzalloc(rx_electrodes, GFP_KERNEL);

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f55->control_base_addr + SENSOR_TX_MAPPING_OFFSET,
			f55->tx_assignment,
			tx_electrodes);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read f55 tx assignment\n",
				__func__);
		return;
	}

	retval = synaptics_rmi4_i2c_read(rmi4_data,
			f55->control_base_addr + SENSOR_RX_MAPPING_OFFSET,
			f55->rx_assignment,
			rx_electrodes);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read f55 rx assignment\n",
				__func__);
		return;
	}

	f54->tx_assigned = 0;
	for (ii = 0; ii < tx_electrodes; ii++) {
		if (f55->tx_assignment[ii] != 0xff)
			f54->tx_assigned++;
	}

	f54->rx_assigned = 0;
	for (ii = 0; ii < rx_electrodes; ii++) {
		if (f55->rx_assignment[ii] != 0xff)
			f54->rx_assigned++;
	}

	return;
}

static void test_f55_set_regs(struct synaptics_rmi4_data *rmi4_data,
		struct synaptics_rmi4_fn_desc *fd,
		unsigned char page)
{
	f55 = kzalloc(sizeof(*f55), GFP_KERNEL);
	if (!f55) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for f55\n",
				__func__);
		return;
	}

	f55->query_base_addr = fd->query_base_addr | (page << 8);
	f55->control_base_addr = fd->ctrl_base_addr | (page << 8);
	f55->data_base_addr = fd->data_base_addr | (page << 8);
	f55->command_base_addr = fd->cmd_base_addr | (page << 8);

	return;
}

static int test_scan_pdt(void)
{
	int retval;
	unsigned char intr_count = 0;
	unsigned char page;
	unsigned short addr;
	bool f54found = false;
	bool f55found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;
	struct synaptics_rmi4_data *rmi4_data = f54->rmi4_data;

	for (page = 0; page < PAGES_TO_SERVICE; page++) {
		for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
			addr |= (page << 8);

			retval = synaptics_rmi4_i2c_read(rmi4_data,
					addr,
					(unsigned char *)&rmi_fd,
					sizeof(rmi_fd));
			if (retval < 0)
				return retval;

			addr &= ~(MASK_8BIT << 8);

			if (!rmi_fd.fn_number)
				break;

			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F54:
				test_f54_set_regs(rmi4_data,
						&rmi_fd, intr_count, page);
				f54found = true;
				break;
			case SYNAPTICS_RMI4_F55:
				test_f55_set_regs(rmi4_data,
						&rmi_fd, page);
				f55found = true;
				break;
			default:
				break;
			}

			if (f54found && f55found)
				goto pdt_done;

			intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
		}
	}

	if (!f54found) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to find F54\n",
				__func__);
		return -EINVAL;
	}

pdt_done:
	return 0;
}

static void synaptics_rmi4_test_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (!f54)
		return;

	if (f54->intr_mask & intr_mask)
		queue_work(f54->test_report_workqueue, &f54->test_report_work);

	return;
}

static int synaptics_rmi4_test_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	f54 = kzalloc(sizeof(*f54), GFP_KERNEL);
	if (!f54) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for f54\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	f54->rmi4_data = rmi4_data;

	f55 = NULL;

	retval = test_scan_pdt();
	if (retval < 0)
		goto exit_free_mem;

	retval = test_set_queries();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read f54 query registers\n",
				__func__);
		goto exit_free_mem;
	}

	f54->tx_assigned = f54->query.num_of_tx_electrodes;
	f54->rx_assigned = f54->query.num_of_rx_electrodes;

	retval = test_set_controls();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to set up f54 control registers\n",
				__func__);
		goto exit_free_control;
	}

	if (f55)
		test_f55_init(rmi4_data);

	retval = test_set_sysfs();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs entries\n",
				__func__);
		goto exit_sysfs;
	}

	f54->test_report_workqueue =
			create_singlethread_workqueue("test_report_workqueue");
	INIT_WORK(&f54->test_report_work, test_report_work);

	hrtimer_init(&f54->watchdog, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	f54->watchdog.function = test_get_report_timeout;
	INIT_WORK(&f54->timeout_work, test_timeout_work);

	mutex_init(&f54->status_mutex);
	f54->status = STATUS_IDLE;

	return 0;

exit_sysfs:
	if (f55) {
		kfree(f55->tx_assignment);
		kfree(f55->rx_assignment);
	}

exit_free_control:
	test_free_control_mem();

exit_free_mem:
	kfree(f55);
	f55 = NULL;
	kfree(f54);
	f54 = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_test_remove(struct synaptics_rmi4_data *rmi4_data)
{
	if (!f54)
		goto exit;

	hrtimer_cancel(&f54->watchdog);

	cancel_work_sync(&f54->test_report_work);
	flush_workqueue(f54->test_report_workqueue);
	destroy_workqueue(f54->test_report_workqueue);

	test_remove_sysfs();

	if (f55) {
		kfree(f55->tx_assignment);
		kfree(f55->rx_assignment);
	}

	test_free_control_mem();

	kfree(f55);
	f55 = NULL;

	if (f54->data_buffer_size)
		kfree(f54->report_data);

	kfree(f54);
	f54 = NULL;

exit:
	complete(&test_remove_complete);

	return;
}

static void synaptics_rmi4_test_reset(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;

	if (!f54) {
		synaptics_rmi4_test_init(rmi4_data);
		return;
	}

	if (f55) {
		kfree(f55->tx_assignment);
		kfree(f55->rx_assignment);
	}

	test_free_control_mem();

	kfree(f55);
	f55 = NULL;

	retval = test_scan_pdt();
	if (retval < 0)
		goto exit_free_mem;

	retval = test_set_queries();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read f54 query registers\n",
				__func__);
		goto exit_free_mem;
	}

	f54->tx_assigned = f54->query.num_of_tx_electrodes;
	f54->rx_assigned = f54->query.num_of_rx_electrodes;

	retval = test_set_controls();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to set up f54 control registers\n",
				__func__);
		goto exit_free_control;
	}

	if (f55)
		test_f55_init(rmi4_data);

	f54->status = STATUS_IDLE;

	return;

exit_free_control:
	test_free_control_mem();

exit_free_mem:
	kfree(f55);
	f55 = NULL;
	kfree(f54);
	f54 = NULL;

	return;
}

static struct synaptics_rmi4_exp_fn test_module = {
	.fn_type = RMI_TEST_REPORTING,
	.init = synaptics_rmi4_test_init,
	.remove = synaptics_rmi4_test_remove,
	.reset = synaptics_rmi4_test_reset,
	.reinit = NULL,
	.early_suspend = NULL,
	.suspend = NULL,
	.resume = NULL,
	.late_resume = NULL,
	.attn = synaptics_rmi4_test_attn,
};

static int __init rmi4_test_module_init(void)
{
	synaptics_rmi4_new_function(&test_module, true);

	return 0;
}

static void __exit rmi4_test_module_exit(void)
{
	synaptics_rmi4_new_function(&test_module, false);

	wait_for_completion(&test_remove_complete);

	return;
}

module_init(rmi4_test_module_init);
module_exit(rmi4_test_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX Test Reporting Module");
MODULE_LICENSE("GPL v2");
