/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_laser_focus.h"
#include "msm_laser_focus_vl6180x_def.h"
#include "sysfs/Laser_forcus_sysfs.h"
#include "msm_cci.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

//DEFINE_MSM_MUTEX(msm_laser_focus_mutex);

#undef CDBG
#define CDBG(fmt, args...) //pr_info(fmt, ##args)

//static struct v4l2_file_operations msm_laser_focus_v4l2_subdev_fops;
static int32_t VL6180x_power_up(struct msm_laser_focus_ctrl_t *a_ctrl);
static int32_t VL6180x_power_down(struct msm_laser_focus_ctrl_t *a_ctrl);
static int VL6180x_match_id(struct msm_laser_focus_ctrl_t *s_ctrl);
static int VL6180x_init(struct msm_laser_focus_ctrl_t *a_ctrl);
static int VL6180x_deinit(struct msm_laser_focus_ctrl_t *a_ctrl);

struct msm_laser_focus_ctrl_t *vl6180x_t = NULL;

/*For LaserFocus STATUS Controll+++*/
#define	STATUS_PROC_FILE				"driver/LaserFocus_Status"
#define	DEVICE_TURN_ON_FILE			"driver/LaserFocus_on"
#define	DEVICE_GET_VALUE				"driver/LaserFocus_value"
#define	DEVICE_SET_CALIBRATION			"driver/LaserFocus_CalStart"
#define	DEVICE_DUMP_REGISTER_VALUE	"driver/LaserFocus_regiser_dump"
static struct proc_dir_entry *status_proc_file;
static struct proc_dir_entry *device_trun_on_file;
static struct proc_dir_entry *device_get_value_file;
static struct proc_dir_entry *device_set_calibration_file;
static struct proc_dir_entry *dump_laser_focus_register_file;
static int ATD_status;

static int VL6180x_WrByte(uint32_t register_addr, uint16_t i2c_write_data)
{
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				i2c_write_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	CDBG("%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
	return status;
}

static uint16_t VL6180x_RdByte(uint32_t register_addr)
{
	int status;
	uint16_t data;
	
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
			&data, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	CDBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, data);
	return data;
}

static int VL6180x_WrDWord(uint32_t register_addr, uint16_t i2c_write_data)
{
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, register_addr,
				i2c_write_data, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	CDBG("%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
	return status;
}

static int VL6180x_RdDWord(uint32_t register_addr, uint16_t *i2c_read_data)
{
	int status;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, register_addr,
		i2c_read_data, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	CDBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

static int VL6180x_device_UpscaleRegInit(void)
{
	int status = 0;
	uint16_t p2p_offset = 0;

	VL6180x_WrByte(0x0207, 0x01);
	VL6180x_WrByte(0x0208, 0x01);
	VL6180x_WrByte(0x0133, 0x01);
	VL6180x_WrByte(0x0096, 0x00);
	VL6180x_WrByte(0x0097, 0x54);
	VL6180x_WrByte(0x00E3, 0x00);
	VL6180x_WrByte(0x00E4, 0x04);
	VL6180x_WrByte(0x00E5, 0x02);
	VL6180x_WrByte(0x00E6, 0x01);
	VL6180x_WrByte(0x00E7, 0x03);
	VL6180x_WrByte(0x00F5, 0x02);
	VL6180x_WrByte(0x00D9, 0x05);
	VL6180x_WrByte(0x00DB, 0xCE);
	VL6180x_WrByte(0x00DC, 0x03);
	VL6180x_WrByte(0x00DD, 0xF8);
	VL6180x_WrByte(0x009F, 0x00);
	VL6180x_WrByte(0x00A3, 0x28);
	VL6180x_WrByte(0x00B7, 0x00);
	VL6180x_WrByte(0x00BB, 0x28);
	VL6180x_WrByte(0x00B2, 0x09);
	VL6180x_WrByte(0x00CA, 0x09);
	VL6180x_WrByte(0x0198, 0x01);
	VL6180x_WrByte(0x01B0, 0x17);
	VL6180x_WrByte(0x01AD, 0x00);
	VL6180x_WrByte(0x00FF, 0x05);
	VL6180x_WrByte(0x0100, 0x05);
	VL6180x_WrByte(0x0199, 0x05);
	VL6180x_WrByte(0x0109, 0x07);
	VL6180x_WrByte(0x010A, 0x30);
	VL6180x_WrByte(0x003F, 0x46);
	VL6180x_WrByte(0x01A6, 0x1B);
	VL6180x_WrByte(0x01AC, 0x3E);
	VL6180x_WrByte(0x01A7, 0x1F);
	VL6180x_WrByte(0x0103, 0x01);
	VL6180x_WrByte(0x0030, 0x00);
	VL6180x_WrByte(0x001B, 0x0A);
	VL6180x_WrByte(0x003E, 0x0A);
	VL6180x_WrByte(0x0131, 0x04);
	VL6180x_WrByte(0x0011, 0x10);
	VL6180x_WrByte(0x0014, 0x24);
	VL6180x_WrByte(0x0031, 0xFF);
	VL6180x_WrByte(0x00D2, 0x01);
	VL6180x_WrByte(0x00F2, 0x01);
	VL6180x_WrByte(0x001C, 0x3F);

	/* Apply calibration data */
	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION || 
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_INIT_CCI)	{
		/* Read & Apply Calibration data */
		if(Laser_Forcus_sysfs_read_offset() != -ENOENT
			&& Laser_Forcus_sysfs_read_offset() != -ENXIO
			&& Laser_Forcus_sysfs_read_offset() != -EINVAL)
		{
			vl6180x_t->laser_focus_offset_value = Laser_Forcus_sysfs_read_offset();
			VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, vl6180x_t->laser_focus_offset_value);
		}
		else
		{
			// recount p2p calibration offset to fit ranging ratio
			p2p_offset = VL6180x_RdByte(0x0024);

			if( p2p_offset > 0x7F )
			{
				p2p_offset -= 0xFF;
			}
			p2p_offset /= 3;
			p2p_offset += 1;

			VL6180x_WrByte(0x24, p2p_offset);
		}
		
		if(Laser_Forcus_sysfs_read_cross_talk_offset() != -ENOENT
			&& Laser_Forcus_sysfs_read_cross_talk_offset() != -ENXIO
			&& Laser_Forcus_sysfs_read_cross_talk_offset() != -EINVAL)
		{
			vl6180x_t->laser_focus_cross_talk_offset_value = Laser_Forcus_sysfs_read_cross_talk_offset();
			VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, vl6180x_t->laser_focus_cross_talk_offset_value);
		}
		
		//vl6180x_t->laser_focus_offset_value = Laser_Forcus_sysfs_read_offset();
		//vl6180x_t->laser_focus_cross_talk_offset_value = Laser_Forcus_sysfs_read_cross_talk_offset();

		/* Apply Calibration value */
		//VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, vl6180x_t->laser_focus_offset_value);
		//VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, vl6180x_t->laser_focus_cross_talk_offset_value);
	}
	else{
		// recount p2p calibration offset to fit ranging ratio
		p2p_offset = VL6180x_RdByte(0x0024);

		if( p2p_offset > 0x7F )
		{
			p2p_offset -= 0xFF;
		}
		p2p_offset /= 3;
		p2p_offset += 1;

		VL6180x_WrByte(0x24, p2p_offset);
	}

	return status;
}

static int VL6180x_WaitDeviceBooted(void)
{
	uint16_t FreshOutReset;
	int status, retry_time = 0;
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}	

	do	{
		status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, SYSTEM_FRESH_OUT_OF_RESET,
			&FreshOutReset, MSM_CAMERA_I2C_BYTE_DATA);
		if (status < 0) {
			pr_err("%s: read register(0x%x) failed\n", __func__, SYSTEM_FRESH_OUT_OF_RESET);
			return status;
		}
		CDBG("%s: read register(0x%x) : 0x%x \n", __func__, SYSTEM_FRESH_OUT_OF_RESET, FreshOutReset);

		if (FreshOutReset == 0X01)	{
			/* Setting SYSTEM_FRESH_OUT_OF_RESET to 0 */
			status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, SYSTEM_FRESH_OUT_OF_RESET,
				0, MSM_CAMERA_I2C_BYTE_DATA);
			if (status < 0) {
				pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSTEM_FRESH_OUT_OF_RESET);
				return status;
			}
			CDBG("%s: wirte register(0x%x) : 0x00 \n", __func__, SYSTEM_FRESH_OUT_OF_RESET);
		}
		msleep(5);
		retry_time++;
		if (retry_time == 10)
			break;
	}
	while (FreshOutReset != 0x00);

	return status;
}

static ssize_t ATD_VL6180x_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8];
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (vl6180x_t->device_state == val)	{
		printk("%s Setting same commond (%d) !!\n", __func__, val);
		return -EINVAL;
	}
	switch (val) {
	case MSM_LASER_FOCUS_DEVICE_OFF:
		rc = VL6180x_power_down(vl6180x_t);
		rc = VL6180x_deinit(vl6180x_t);
		vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		break;
	case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		if (vl6180x_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
			rc = VL6180x_power_down(vl6180x_t);
			rc = VL6180x_deinit(vl6180x_t);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}
		vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
		rc = VL6180x_power_up(vl6180x_t);
		rc = VL6180x_init(vl6180x_t);
		rc = VL6180x_WaitDeviceBooted();
		if (rc < 0)	{
			printk("%s Device trun on fail !!\n", __func__);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			return -EIO;
		} else	{
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			printk("%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
			VL6180x_device_UpscaleRegInit();

			/* Setting SYSRANGE_START to 0x01 */
			VL6180x_WrByte(SYSRANGE_START, 0x01);
		}
		break;
	case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:
		if (vl6180x_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
			rc = VL6180x_power_down(vl6180x_t);
			rc = VL6180x_deinit(vl6180x_t);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}
		vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
		rc = VL6180x_power_up(vl6180x_t);
		rc = VL6180x_init(vl6180x_t);
		rc = VL6180x_WaitDeviceBooted();
		if (rc < 0)	{
			printk("%s Device trun on fail !!\n", __func__);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			return -EIO;
		} else	{
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			printk("%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
			VL6180x_device_UpscaleRegInit();

			/* Setting SYSRANGE_START to 0x01 */
			VL6180x_WrByte(SYSRANGE_START, 0x01);
		}
		break;
	case MSM_LASER_FOCUS_DEVICE_INIT_CCI:
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			rc = VL6180x_init(vl6180x_t);
			rc = VL6180x_WaitDeviceBooted();
			if(rc < 0){
				printk("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} else {
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
				printk("%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
				VL6180x_device_UpscaleRegInit();

				/* Setting SYSRANGE_START to 0x01 */
				VL6180x_WrByte(SYSRANGE_START, 0X01);
			}
			break;
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			rc = VL6180x_deinit(vl6180x_t);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_DEINIT_CCI;
			break;
	default:
		printk("%s commond fail !!\n", __func__);
		break;
	}
	return len;
}

static int ATD_VL6180x_device_enable_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", vl6180x_t->device_state);
	return 0;
}

static int ATD_VL6180x_device_enable_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_device_enable_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_enable_open,
	.write = ATD_VL6180x_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_VL6180x_device_read_range(void)
{
	uint16_t RawRange, Data_value;
	int status, i = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	/* Setting i2c client */
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	/* Setting Range meansurement in single-shot mode */	
	/* Check RESULT_RANGE_STATUS bit0 is 0 */
	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, RESULT_RANGE_STATUS, &Data_value, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, RESULT_RANGE_STATUS);
		return status;
	}
	//CDBG("%s: read register(0x%x) : 0x%x \n", __func__, RESULT_RANGE_STATUS, Data_value);

	/* Setting SYSRANGE_START to 0x01 */
	Data_value = 0x01;
	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, SYSRANGE_START, Data_value, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSRANGE_START);
		return status;
	}
	CDBG("%s: wirte register(0x%x) : 0x%x \n", __func__, SYSRANGE_START, Data_value);
#if 0	
	/* Check RESULT_INTERRUPT_STATUS_GPIO */
	Data_value = 0x00;
	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, RESULT_INTERRUPT_STATUS_GPIO, &Data_value, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, RESULT_INTERRUPT_STATUS_GPIO);
		return status;
	}
	CDBG("%s: read register(0x%x) : 0x%x \n", __func__, RESULT_INTERRUPT_STATUS_GPIO, Data_value);

	/* Check SYSRANGE_CROSSTALK_COMPENSATION_RATE */
	Data_value = 0x00;
	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, SYSRANGE_CROSSTALK_COMPENSATION_RATE, &Data_value, MSM_CAMERA_I2C_WORD_DATA);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, SYSRANGE_CROSSTALK_COMPENSATION_RATE);
		return status;
	}
	CDBG("%s: read register(0x%x) : 0x%x \n", __func__, SYSRANGE_CROSSTALK_COMPENSATION_RATE, Data_value);

	/* Check SYSRANGE_PART_TO_PART_RANGE_OFFSET */
	Data_value = 0x00;
	status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, SYSRANGE_PART_TO_PART_RANGE_OFFSET, &Data_value, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, SYSRANGE_PART_TO_PART_RANGE_OFFSET);
		return status;
	}
	CDBG("%s: read register(0x%x) : 0x%x \n", __func__, SYSRANGE_PART_TO_PART_RANGE_OFFSET, Data_value);
#endif

	/* Get Sensor detect distance */
	RawRange = 0;
	for (i = 0; i <10; i++)	{
		/* Check RESULT_INTERRUPT_STATUS_GPIO */
		Data_value = 0x00;
		status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, RESULT_INTERRUPT_STATUS_GPIO, &Data_value, MSM_CAMERA_I2C_BYTE_DATA);
		if (status < 0) {
			pr_err("%s: read register(0x%x) failed\n", __func__, RESULT_INTERRUPT_STATUS_GPIO);
			return status;
		}
		CDBG("%s: read register(0x%x) : 0x%x \n", __func__, RESULT_INTERRUPT_STATUS_GPIO, Data_value);

		if (Data_value == RES_INT_STAT_GPIO_NEW_SAMPLE_READY)	{
			status = sensor_i2c_client->i2c_func_tbl->i2c_read(sensor_i2c_client, RESULT_RANGE_VAL, &RawRange, MSM_CAMERA_I2C_BYTE_DATA);
			if (status < 0) {
				pr_err("%s: read register(0x%x) failed\n", __func__, RESULT_RANGE_VAL);
				return status;
			} else	{
				CDBG("%s: read register(0x%x) : %d \n", __func__, RESULT_RANGE_VAL, RawRange);
				break;
			}
		}
		msleep(20);
	}

	/* Setting SYSTEM_INTERRUPT_CLEAR to 0x01 */
	Data_value = 0x01;
	status = sensor_i2c_client->i2c_func_tbl->i2c_write(sensor_i2c_client, SYSTEM_INTERRUPT_CLEAR, Data_value, MSM_CAMERA_I2C_BYTE_DATA);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSTEM_INTERRUPT_CLEAR);
		return status;
	}
	//CDBG("%s: wirte register(0x%x) : 0x%x \n", __func__, SYSTEM_INTERRUPT_CLEAR, Data_value);
	
	return (int)RawRange*3;
}

static int ATD_VL6180x_device_get_range_read(struct seq_file *buf, void *v)
{
	int RawRange = 0;

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		seq_printf(buf, "%d\n", 0);
		//return -EBUSY;
		return 0;
	}
	
	RawRange = ATD_VL6180x_device_read_range();
	//RawRange = RawRange*3;

	printk("%s Test Data (%d)  Device (%d)\n", __func__, RawRange , vl6180x_t->device_state);

	if (RawRange < 0) {
		pr_err("%s: ead_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}

	seq_printf(buf, "%d\n", RawRange);

	return 0;
}
 
static int ATD_VL6180x_device_get_range_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_device_get_range_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_VL6180x_device_clibration_offset(void)
{
	int i = 0, RawRange = 0, sum = 0;
	uint16_t distance;
	int16_t offset;

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		return -EBUSY;
	}

	/* Set calibratioon Setting */
	/*VL6180x_WrByte(0x0097, 0xFD);
	VL6180x_WrByte(0x00A3, 0x3C);
	VL6180x_WrByte(0x00BB, 0x3C);*/

	printk("%s:%d: VL6180x: 0x24 %x\n", __func__, __LINE__,VL6180x_RdByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET));
	
	/* Clean system offset */
	VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x00);
	VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);

	for(i=0; i<STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{
		RawRange = (int)ATD_VL6180x_device_read_range();
		printk("%s:%d: VL6180x: RawRange %d\n", __func__, __LINE__,RawRange);
		msleep(50);
		sum += RawRange;
	}
	distance = (uint16_t)(sum / STMVL6180_RUNTIMES_OFFSET_CAL);
	//printk("The measure distanec is %d mm\n", distance);
	printk("%s:%d: VL6180x: distance %d\n", __func__, __LINE__,distance);

	if((VL6180_OFFSET_CAL_RANGE - distance)<0){
		offset = (VL6180_OFFSET_CAL_RANGE - distance)/3;
		offset = 256+offset;
	}else{
		offset = (VL6180_OFFSET_CAL_RANGE - distance)/3;
	}

	printk("%s:%d: VL6180x: offset %d\n", __func__, __LINE__,offset);

	if((distance>=(100+3)||distance<=(100-3)))
	{	
		VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);

		/* Write calibration file */
		vl6180x_t->laser_focus_offset_value = offset;
		if (Laser_Forcus_sysfs_write_offset(offset) == false)
			return -ENOENT;
	}

	/* Set calibratioon Setting */
	/*VL6180x_WrByte(0x097, 0x54);
	VL6180x_WrByte(0x0A3, 0x28);
	VL6180x_WrByte(0x0BB, 0x28);*/
	
	return 0;
}

static int ATD_VL6180x_device_clibration_crosstalkoffset(void)
{
	int i = 0, RawRange = 0;
	int xtalk_sum  = 0, xrtn_sum = 0;
	uint16_t XtalkCompRate;
	uint16_t rtnRate = 0;

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		return -EBUSY;
	}

	/* Clean crosstalk offset */
	VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);

	for(i = 0; i < STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{	
		//RawRange = (int)VL6180x_RdByte(RESULT_RANGE_RAW);
		RawRange = ATD_VL6180x_device_read_range();
		VL6180x_RdDWord(RESULT_RANGE_SIGNAL_RATE, &rtnRate);

		printk("%s:%d: VL6180x: RawRange %d\n", __func__, __LINE__,RawRange);
		printk("%s:%d: VL6180x: rtnRate %d\n", __func__, __LINE__,(int)rtnRate);

		xtalk_sum += RawRange;
		xrtn_sum += (int)rtnRate;

		msleep(30);
	}
	printk("Crosstalk compensation rate is %d\n", xtalk_sum);
	printk("%s:%d: VL6180x: xrtn_sum %d\n", __func__, __LINE__,xrtn_sum);

	//XtalkCompRate = (uint16_t)((rtnRate/STMVL6180_RUNTIMES_OFFSET_CAL)/128) * (1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*3000/VL6180_CROSSTALK_CAL_RANGE));
	//XtalkCompRate = XtalkCompRate*128;

	XtalkCompRate = (uint16_t)(xrtn_sum/STMVL6180_RUNTIMES_OFFSET_CAL) * (1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*1000/VL6180_CROSSTALK_CAL_RANGE));
	XtalkCompRate = XtalkCompRate;

	XtalkCompRate = XtalkCompRate/1000;

	if(VL6180x_RdByte(0x21)==20)
	{
		VL6180x_WrByte(0x21,0x6);
	}
	
	printk("Crosstalk compensation rate is %d\n", XtalkCompRate);

	VL6180x_WrDWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, XtalkCompRate);

	/* Write calibration file */
	vl6180x_t->laser_focus_cross_talk_offset_value = XtalkCompRate;
	if (Laser_Forcus_sysfs_write_cross_talk_offset(XtalkCompRate) == false)
		return -ENOENT;

	return 0;
}
 
static ssize_t ATD_VL6180x_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8];
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	printk("%s commond : %d\n", __func__, val);
	switch (val) {
	case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		ret = ATD_VL6180x_device_clibration_offset();
		if (ret < 0)
			return ret;
		break;
	case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
		ret = ATD_VL6180x_device_clibration_crosstalkoffset();
		if (ret < 0)
			return ret;
		break;
	default:
		printk("%s commond fail(%d) !!\n", __func__, val);
		return -EINVAL;
		break;
	}
	return len;
}

static const struct file_operations ATD_laser_focus_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_get_range_open,
	.write = ATD_VL6180x_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_VL6180x_I2C_status_check(struct msm_laser_focus_ctrl_t *s_ctrl){
	int32_t rc;

	rc = VL6180x_power_up(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_power_up failed %d\n", __func__, __LINE__);
		return 0;
	}
	VL6180x_init(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_init failed %d\n", __func__, __LINE__);
		return 0;
	}
	
	rc = VL6180x_match_id(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_match_id failed %d\n", __func__, __LINE__);
		return 0;
	}
	rc = VL6180x_deinit(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_deinit failed %d\n", __func__, __LINE__);
		return 0;
	}
	rc = VL6180x_power_down(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
		return 0;
	}

	return 1;
}

static int ATD_VL6180x_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = ATD_VL6180x_I2C_status_check(vl6180x_t);
	
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

static int ATD_VL6180x_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_VL6180x_register_read(struct seq_file *buf, void *v)
{
	int status, i = 0;
	uint16_t register_value = 0;
	
	if(vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF)
	{
		printk("VL6180x device_state is OFF!\n");
		return 0;
	}

	for (i = 0; i <0x100; i++)	{
		register_value = 0;
		status = VL6180x_RdDWord(i, &register_value);
		Laser_Forcus_sysfs_write_register_ze550kl(i, register_value);
	}
	seq_printf(buf, "%d\n", 0);
	return 0;
}

static int dump_VL6180x_laser_focus_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_VL6180x_register_read, NULL);
}

static const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_VL6180x_laser_focus_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static void VL6180x_create_proc_file(void)
{
	status_proc_file = proc_create(STATUS_PROC_FILE, 0776, NULL, &ATD_I2C_status_check_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
	}
	
	device_trun_on_file = proc_create(DEVICE_TURN_ON_FILE, 0776, NULL, &ATD_laser_focus_device_enable_fops);
	if (device_trun_on_file) {
		printk("%s device_trun_on_file sucessed!\n", __func__);
	} else {
		printk("%s device_trun_on_file failed!\n", __func__);
	}

	device_get_value_file = proc_create(DEVICE_GET_VALUE, 0776, NULL, &ATD_laser_focus_device_get_range_fos);
	if (device_get_value_file) {
		printk("%s device_get_value_file sucessed!\n", __func__);
	} else {
		printk("%s device_get_value_file failed!\n", __func__);
	}

	device_set_calibration_file = proc_create(DEVICE_SET_CALIBRATION, 0776, NULL, &ATD_laser_focus_device_calibration_fops);
	if (device_set_calibration_file) {
		printk("%s device_set_calibration_file sucessed!\n", __func__);
	} else {
		printk("%s device_set_calibration_file failed!\n", __func__);
	}

	dump_laser_focus_register_file = proc_create(DEVICE_DUMP_REGISTER_VALUE, 0776, NULL, &dump_laser_focus_register_fops);
	if (dump_laser_focus_register_file) {
		printk("%s dump_laser_focus_register_file sucessed!\n", __func__);
	} else {
		printk("%s dump_laser_focus_register_file failed!\n", __func__);
	}
}

int VL6180x_match_id(struct msm_laser_focus_ctrl_t *s_ctrl)
{
	int rc = 0;
	uint16_t chipid = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct msm_camera_slave_info *slave_info;
	const char *sensor_name;

	if (!s_ctrl) {
		pr_err("%s:%d failed: %p\n",
			__func__, __LINE__, s_ctrl);
		return -EINVAL;
	}
	sensor_i2c_client = s_ctrl->i2c_client;
	slave_info = s_ctrl->sensordata->slave_info;
	sensor_name = s_ctrl->sensordata->sensor_name;

	if (!sensor_i2c_client || !slave_info || !sensor_name) {
		pr_err("%s:%d failed: %p %p %p\n",
			__func__, __LINE__, sensor_i2c_client, slave_info,
			sensor_name);
		return -EINVAL;
	}

	rc = sensor_i2c_client->i2c_func_tbl->i2c_read(
		sensor_i2c_client, slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: read id failed\n", __func__, sensor_name);
		return rc;
	}

	CDBG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
		slave_info->sensor_id);
	if (chipid != slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}

static int32_t VL6180x_vreg_control(struct msm_laser_focus_ctrl_t *a_ctrl,
							int config)
{
	int rc = 0, i, cnt;
	struct msm_laser_focus_vreg *vreg_cfg;

	vreg_cfg = &a_ctrl->vreg_cfg;
	cnt = vreg_cfg->num_vreg;
	if (!cnt)
		return 0;

	if (cnt >= CAM_VREG_MAX) {
		pr_err("%s failed %d cnt %d\n", __func__, __LINE__, cnt);
		return -EINVAL;
	}

	for (i = 0; i < cnt; i++) {
		rc = msm_camera_config_single_vreg(&(a_ctrl->pdev->dev),
			&vreg_cfg->cam_vreg[i],
			(struct regulator **)&vreg_cfg->data[i],
			config);
	}
	return rc;
}

static int32_t VL6180x_power_down(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (a_ctrl->laser_focus_state != LASER_FOCUS_POWER_DOWN) {

		rc = VL6180x_vreg_control(a_ctrl, 0);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			return rc;
		}

		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		a_ctrl->i2c_tbl_index = 0;
		a_ctrl->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}
	CDBG("Exit\n");
	return rc;
}

static int VL6180x_init(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	// CCI Init 
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client->i2c_func_tbl->i2c_util(
			a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
	CDBG("Exit\n");
	return rc;
}

static int VL6180x_deinit(struct msm_laser_focus_ctrl_t *a_ctrl) {
	int rc = 0;
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client->i2c_func_tbl->i2c_util(
			a_ctrl->i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_init failed\n");
		
	}


	CDBG("Exit\n");
	return rc;
}

/*
static int32_t msm_laser_focus_get_subdev_id(struct msm_laser_focus_ctrl_t *a_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = a_ctrl->pdev->id;
	else
		*subdev_id = a_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}
*/

static int32_t VL6180x_get_dt_data(struct device_node *of_node,
		struct msm_laser_focus_ctrl_t *fctrl)
{
	int32_t rc = 0;
	struct msm_camera_sensor_board_info *sensordata = NULL;
	uint32_t id_info[3];
	struct msm_laser_focus_vreg *vreg_cfg;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->sensordata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->sensordata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	sensordata = fctrl->sensordata;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&sensordata->sensor_name);
	CDBG("%s label %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_master = MASTER_0;
		rc = 0;
	}

	if (of_find_property(of_node,
			"qcom,cam-vreg-name", NULL)) {
		vreg_cfg = &fctrl->vreg_cfg;
		rc = msm_camera_get_dt_vreg_data(of_node,
			&vreg_cfg->cam_vreg, &vreg_cfg->num_vreg);
		if (rc < 0) {
			kfree(fctrl);
			pr_err("failed rc %d\n", rc);
			return rc;
		}
	}

	sensordata->slave_info =
		kzalloc(sizeof(struct msm_camera_slave_info),
			GFP_KERNEL);
	if (!sensordata->slave_info) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto ERROR;
	}

	rc = of_property_read_u32_array(of_node, "qcom,slave-id",
		id_info, 3);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}
	fctrl->sensordata->slave_info->sensor_slave_addr = id_info[0];
	fctrl->sensordata->slave_info->sensor_id_reg_addr = id_info[1];
	fctrl->sensordata->slave_info->sensor_id = id_info[2];

		CDBG("%s:%d slave addr 0x%x sensor reg 0x%x id 0x%x\n",
		__func__, __LINE__,
		fctrl->sensordata->slave_info->sensor_slave_addr,
		fctrl->sensordata->slave_info->sensor_id_reg_addr,
		fctrl->sensordata->slave_info->sensor_id);

	return rc;

ERROR:
		kfree(fctrl->sensordata->slave_info);
	return rc;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static const struct v4l2_subdev_internal_ops msm_laser_focus_internal_ops;

static int32_t VL6180x_power_up(struct msm_laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	rc = VL6180x_vreg_control(a_ctrl, 1);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	a_ctrl->laser_focus_state = LASER_FOCUS_POWER_UP;

	CDBG("Exit\n");
	return rc;
}

/*
static int32_t msm_laser_focus_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_laser_focus_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");

	if (on)
		rc = VL6180x_power_up(a_ctrl);
	else
		rc = VL6180x_power_down(a_ctrl);

	CDBG("Exit\n");
	return rc;
}
*/

static struct v4l2_subdev_core_ops msm_laser_focus_subdev_core_ops = {
	.ioctl = NULL,
	//.s_power = msm_laser_focus_power,
};

static struct v4l2_subdev_ops msm_laser_focus_subdev_ops = {
	.core = &msm_laser_focus_subdev_core_ops,
};

static struct msm_camera_i2c_client msm_laser_focus_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};
static const struct of_device_id msm_laser_focus_dt_match[] = {
	{.compatible = "qcom,ois", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, msm_laser_focus_dt_match);

static int32_t VL6180x_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;

	const struct of_device_id *match;
	struct msm_camera_cci_client *cci_client = NULL;

	CDBG("Probe Start\n");
	ATD_status = 0;

	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if (!match) {
		pr_err("device not match\n");
		return -EFAULT;
	}

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	vl6180x_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),
		GFP_KERNEL);
	if (!vl6180x_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* Set platform device handle */
	vl6180x_t->pdev = pdev;

	rc = VL6180x_get_dt_data(pdev->dev.of_node, vl6180x_t);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	/* Assign name for sub device */
	snprintf(vl6180x_t->msm_sd.sd.name, sizeof(vl6180x_t->msm_sd.sd.name),
			"%s", vl6180x_t->sensordata->sensor_name);

	vl6180x_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;

	/* Set device type as platform device */
	vl6180x_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	vl6180x_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == vl6180x_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}
	if (!vl6180x_t->i2c_client->i2c_func_tbl)
		vl6180x_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	vl6180x_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!vl6180x_t->i2c_client->cci_client) {
		kfree(vl6180x_t->vreg_cfg.cam_vreg);
		kfree(vl6180x_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
	//vl6180x_t->i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	cci_client = vl6180x_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = vl6180x_t->cci_master;
	if (vl6180x_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = vl6180x_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	
	cci_client->i2c_freq_mode = I2C_FAST_MODE; //sean ++
	
	v4l2_subdev_init(&vl6180x_t->msm_sd.sd,
		vl6180x_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&vl6180x_t->msm_sd.sd, vl6180x_t);
	vl6180x_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	vl6180x_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(vl6180x_t->msm_sd.sd.name,
		ARRAY_SIZE(vl6180x_t->msm_sd.sd.name), "msm_laser_focus");
	media_entity_init(&vl6180x_t->msm_sd.sd.entity, 0, NULL, 0);
	vl6180x_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	vl6180x_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	vl6180x_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&vl6180x_t->msm_sd);
	vl6180x_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;

	/* Init data struct */
	vl6180x_t->laser_focus_cross_talk_offset_value = 0;
	vl6180x_t->laser_focus_offset_value = 0;
	vl6180x_t->laser_focus_state = MSM_LASER_FOCUS_DEVICE_OFF;

	/*rc = VL6180x_power_up(vl6180x_t);
	if (rc < 0) {
		pr_err("%s VL6180x_power_up failed %d\n", __func__, __LINE__);
		goto probe_failure;
	}
	VL6180x_init(vl6180x_t);
	if (rc < 0) {
		pr_err("%s VL6180x_init failed %d\n", __func__, __LINE__);
		goto probe_failure;
	}
	rc = VL6180x_match_id(vl6180x_t);
	if (rc < 0) {
		pr_err("%s VL6180x_match_id failed %d\n", __func__, __LINE__);
		goto probe_failure;
	}
	rc = VL6180x_deinit(vl6180x_t);
	if (rc < 0) {
		pr_err("%s VL6180x_deinit failed %d\n", __func__, __LINE__);
		goto probe_failure;
	}
	rc = VL6180x_power_down(vl6180x_t);
	if (rc < 0) {
		pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
		goto probe_failure;
	}*/
	
	if(ATD_VL6180x_I2C_status_check(vl6180x_t) == 0)
		goto probe_failure;
	
	ATD_status = 1;
	VL6180x_create_proc_file();
		
	CDBG("Probe Success\n");
	return 0;
probe_failure:
	CDBG("%s Probe failed\n", __func__);
	return rc;
}

static struct platform_driver msm_laser_focus_platform_driver = {
	.driver = {
		.name = "qcom,ois",
		.owner = THIS_MODULE,
		.of_match_table = msm_laser_focus_dt_match,
	},
};

static int __init VL6180x_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_probe(&msm_laser_focus_platform_driver,
		VL6180x_platform_probe);
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);

	return rc;
}
static void __exit VL6180x_driver_exit(void)
{
	CDBG("Enter");
	platform_driver_unregister(&msm_laser_focus_platform_driver);
	return;
}


module_init(VL6180x_init_module);
module_exit(VL6180x_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
