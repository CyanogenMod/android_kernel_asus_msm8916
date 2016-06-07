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
//#include "msm_laser_focus_vl6180x_def.h"
#include "sysfs/Laser_forcus_sysfs.h"
#include "msm_cci.h"
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include "msm_laser_focus_vl6180x_api.h"
#include "kernel_driver_timer.h"

#include "vl6180x_api.h"

//DEFINE_MSM_MUTEX(msm_laser_focus_mutex);

#undef CDBG
#define CDBG(fmt, args...) //pr_info(fmt, ##args)

#undef DBG_LOG
#define DBG_LOG(fmt, args...) //pr_info(fmt, ##args)

#undef REG_RW_DBG
#define REG_RW_DBG(fmt, args...) //pr_info(fmt, ##args)

#undef API_DBG
#define API_DBG(fmt, args...) //pr_info(fmt, ##args)

/* Out of range */
#define OUT_OF_RANGE 765

/* Time out value: mm */
#define TIMEOUT_VAL 80	//ZE550KL define 80ms,other project can define again

#define VL6180_API_ERROR_COUNT_MAX		5

//static struct v4l2_file_operations msm_laser_focus_v4l2_subdev_fops;
static int32_t VL6180x_power_up(struct msm_laser_focus_ctrl_t *a_ctrl);
static int32_t VL6180x_power_down(struct msm_laser_focus_ctrl_t *a_ctrl);
static int VL6180x_match_id(struct msm_laser_focus_ctrl_t *s_ctrl);
static int VL6180x_init(struct msm_laser_focus_ctrl_t *a_ctrl);
static int VL6180x_deinit(struct msm_laser_focus_ctrl_t *a_ctrl);

struct msm_laser_focus_ctrl_t *vl6180x_t = NULL;
struct timeval timer, timer2;
bool camera_on_flag = false;
int vl6180x_check_status = 0;

static int DMax = 0;
static int errorStatus = 16;
struct mutex vl6180x_mutex;

/*For LaserFocus STATUS Controll+++*/
#define	STATUS_PROC_FILE				"driver/LaserFocus_Status"
#define	STATUS_PROC_FILE_FOR_CAMERA	"driver/LaserFocus_Status_For_Camera"
#define	DEVICE_TURN_ON_FILE			"driver/LaserFocus_on"
#define	DEVICE_GET_VALUE				"driver/LaserFocus_value"
#define	DEVICE_GET_MORE_VALUE				"driver/LaserFocus_value_more_info"
#define	DEVICE_SET_CALIBRATION			"driver/LaserFocus_CalStart"
#define	DEVICE_DUMP_REGISTER_VALUE	"driver/LaserFocus_regiser_dump"
#define	DEVICE_SET_REGISTER_VALUE	"driver/LaserFocus_regiser_set"
#define DEVICE_DUMP_DEBUG_REGISTER_VALUE        "driver/LaserFocus_debug_dump"
static struct proc_dir_entry *status_proc_file;
static struct proc_dir_entry *device_trun_on_file;
static struct proc_dir_entry *device_get_value_file;
static struct proc_dir_entry *device_get_more_value_file;
static struct proc_dir_entry *device_set_calibration_file;
static struct proc_dir_entry *dump_laser_focus_register_file;
static struct proc_dir_entry *set_laser_focus_register_file;
static struct proc_dir_entry *dump_laser_focus_debug_file;
static int ATD_status;

int ASUS_VL6180x_WrByte(uint32_t register_addr, uint16_t i2c_write_data)
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
	REG_RW_DBG("%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
	return status;
}

int ASUS_VL6180x_RdByte(uint32_t register_addr, uint16_t *i2c_read_data)
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
                        i2c_read_data, MSM_CAMERA_I2C_BYTE_DATA);
        if (status < 0) {
                pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
                return status;
        }
        REG_RW_DBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);
        return status;
}

int ASUS_VL6180x_WrWord(uint32_t register_addr, uint16_t i2c_write_data)
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
	REG_RW_DBG("%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
	return status;
}

int ASUS_VL6180x_RdWord(uint32_t register_addr, uint16_t *i2c_read_data)
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
	REG_RW_DBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

int ASUS_VL6180x_WrDWord(uint32_t register_addr, uint32_t i2c_write_data){
	int status;
	uint16_t high_val, low_val;
	
	high_val = (uint16_t)((i2c_write_data&0xFFFF0000)>>16);
	low_val = (uint16_t)(i2c_write_data&0x0000FFFF);
	
	status = ASUS_VL6180x_WrWord(register_addr, high_val);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	status = ASUS_VL6180x_WrWord(register_addr+2, low_val);
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr+2);
		return status;
	}

	REG_RW_DBG("%s: wirte register(0x%x) : 0x%x\n", __func__, register_addr, i2c_write_data);	

	return status;
}

#if 0
int ASUS_VL6180x_WrDWord(uint32_t register_addr, uint32_t i2c_write_data, uint16_t num_byte)
{
	int status;
	uint16_t Data_value = 0;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_addr = register_addr;
	reg_setting.reg_data[0] = (uint8_t)((i2c_write_data & 0xFF000000) >> 24);
	reg_setting.reg_data[1] = (uint8_t)((i2c_write_data & 0x00FF0000) >> 16);
	reg_setting.reg_data[2] = (uint8_t)((i2c_write_data & 0x0000FF00) >> 8);
	reg_setting.reg_data[3] = (uint8_t)(i2c_write_data & 0x000000FF);
	reg_setting.reg_data_size = num_byte;

	printk("[Debug] write reg_data[0]:0x%x ; reg_data[1]:0x%x ; reg_data[2]:0x%x ; reg_data[3]:0x%x\n", 
		reg_setting.reg_data[0], reg_setting.reg_data[1], reg_setting.reg_data[2], reg_setting.reg_data[3]);
	
	//status = (int)msm_camera_cci_i2c_write_seq(sensor_i2c_client, register_addr, 
	//	reg_setting.reg_data, reg_setting.reg_data_size);
	status = sensor_i2c_client->i2c_func_tbl->i2c_write_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);
	
	status = sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);
	printk("[Debug] read reg_data[0]:0x%x ; reg_data[1]:0x%x ; reg_data[2]:0x%x ; reg_data[3]:0x%x\n", 
		reg_setting.reg_data[0], reg_setting.reg_data[1], reg_setting.reg_data[2], reg_setting.reg_data[3]);
	ASUS_VL6180x_RdWord(register_addr, &Data_value);
	
	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	REG_RW_DBG("%s: wirte register(0x%x) : 0x%x\n", __func__, register_addr, i2c_write_data);
	return status;
}
#endif

int ASUS_VL6180x_RdDWord(uint32_t register_addr, uint32_t *i2c_read_data, uint16_t num_byte)
{
	int status;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_data_size = num_byte;

	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		reg_setting.reg_data, reg_setting.reg_data_size);

	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	*i2c_read_data=((uint32_t)reg_setting.reg_data[0]<<24)|((uint32_t)reg_setting.reg_data[1]<<16)|((uint32_t)reg_setting.reg_data[2]<<8)|((uint32_t)reg_setting.reg_data[3]);
	REG_RW_DBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}

int ASUS_VL6180x_UpdateByte(uint32_t register_addr, uint8_t AndData, uint8_t OrData){
	int status;
	uint16_t i2c_read_data, i2c_write_data;

	status = ASUS_VL6180x_RdByte(register_addr, &i2c_read_data);
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

	i2c_write_data = ((uint8_t)(i2c_read_data&0x00FF)&AndData) | OrData;

	status = ASUS_VL6180x_WrByte(register_addr, i2c_write_data);

	if (status < 0) {
		pr_err("%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

 	REG_RW_DBG("%s: update register(0x%x) from 0x%x to 0x%x(AndData:0x%x;OrData:0x%x)\n", __func__, register_addr, i2c_read_data, i2c_write_data,AndData,OrData);

	return status;
}
#if 0
static bool sysfs_write(char *filename, int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err(" File open (%s) fail\n", filename);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err(" File strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	//printk(" write value %s to %s\n", buf, filename);

	return true;
}


static void debug_read_range(VL6180x_RangeData_t *pRangeData){

	uint16_t reg_data = 0;
	
	ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, &reg_data);
	//printk(" register(0x%x) : 0x%x\n", SYSRANGE_CROSSTALK_COMPENSATION_RATE, reg_data);
	sysfs_write("/factory/LaserFocus_0x1e.txt",reg_data);
	ASUS_VL6180x_RdByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, &reg_data);
	//printk(" register(0x%x) : 0x%x\n", SYSRANGE_PART_TO_PART_RANGE_OFFSET, reg_data);
	sysfs_write("/factory/LaserFocus_0x24.txt",reg_data);
	ASUS_VL6180x_RdByte(RESULT_RANGE_STATUS, &reg_data);
	//printk(" register(0x%x) : 0x%x\n", RESULT_RANGE_STATUS, reg_data);
	sysfs_write("/factory/LaserFocus_0x4d.txt",reg_data);
	ASUS_VL6180x_RdByte(RESULT_RANGE_VAL, &reg_data);
	//printk(" register(0x%x) : 0x%x\n", RESULT_RANGE_VAL, reg_data);
	sysfs_write("/factory/LaserFocus_0x62.txt",reg_data);
	ASUS_VL6180x_RdByte(RESULT_RANGE_RAW, &reg_data);
	//printk(" register(0x%x) : 0x%x\n", RESULT_RANGE_RAW, reg_data);
	sysfs_write("/factory/LaserFocus_0x64.txt",reg_data);
	ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE, &reg_data);
	//printk(" register(0x%x) : 0x%x\n", RESULT_RANGE_SIGNAL_RATE, reg_data);
	sysfs_write("/factory/LaserFocus_0x66.txt",reg_data);
	//printk(" DMax : %d0\n", pRangeData->DMax);
	sysfs_write("/factory/LaserFocus_DMax.txt",pRangeData->DMax);
	//printk(" errorCode : %d\n", pRangeData->errorStatus);
	sysfs_write("/factory/LaserFocus_errorStatus.txt",pRangeData->errorStatus);
}
#endif
static int VL6180x_device_Load_Calibration_Value(void){
	int status = 0;
	bool Factory_folder_file;

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_INIT_CCI)	{
			
		if(vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION){
			Factory_folder_file = true;
		}
		else{
			Factory_folder_file = false;
		}
				
		/* Read Calibration data */
		vl6180x_t->laser_focus_offset_value = Laser_Forcus_sysfs_read_offset(Factory_folder_file);
		vl6180x_t->laser_focus_cross_talk_offset_value = Laser_Forcus_sysfs_read_cross_talk_offset(Factory_folder_file);

		/* Apply Calibration value */
		//VL6180x_SetOffsetCalibrationData(0, vl6180x_t->laser_focus_offset_value);
		
		status = ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, vl6180x_t->laser_focus_offset_value);
		if (status < 0) {
			pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSRANGE_PART_TO_PART_RANGE_OFFSET);
			return status;
		}
		
		VL6180x_SetXTalkCompensationRate(0, vl6180x_t->laser_focus_cross_talk_offset_value);
		/*
		status = ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, vl6180x_t->laser_focus_cross_talk_offset_value);
		if (status < 0) {
			pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSRANGE_CROSSTALK_COMPENSATION_RATE);
			return status;
		}
		*/
	}	
	
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
			mutex_lock(&vl6180x_mutex);
			if(camera_on_flag){
				CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				mutex_unlock(&vl6180x_mutex);
				break;
			}
			rc = VL6180x_deinit(vl6180x_t);
			rc = VL6180x_power_down(vl6180x_t);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			mutex_unlock(&vl6180x_mutex);
			break;
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
			if(camera_on_flag){
				CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				break;
			}
			if (vl6180x_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
				rc = VL6180x_deinit(vl6180x_t);
				rc = VL6180x_power_down(vl6180x_t);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			}
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			rc = VL6180x_power_up(vl6180x_t);
			rc = VL6180x_init(vl6180x_t);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} 

			API_DBG("%s: VL6180x_InitData Start\n", __func__);
			rc = VL6180x_InitData(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("%s: VL6180x_InitData Success\n", __func__);

			API_DBG("%s: VL6180x_Prepare Start\n", __func__);
			rc = VL6180x_Prepare(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("%s: VL6180x_Prepare Success\n", __func__);


			rc = VL6180x_device_Load_Calibration_Value();
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}

			API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
			//rc = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
			/*if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);
			*/
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			printk("%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
		
			break;
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:
			if(camera_on_flag){
				CDBG("%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				break;
			}
			if (vl6180x_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
				rc = VL6180x_deinit(vl6180x_t);
				rc = VL6180x_power_down(vl6180x_t);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			}
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			rc = VL6180x_power_up(vl6180x_t);
			rc = VL6180x_init(vl6180x_t);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} 

			API_DBG("%s: VL6180x_InitData Start\n", __func__);
			rc = VL6180x_InitData(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("%s: VL6180x_InitData Success\n", __func__);

			API_DBG("%s: VL6180x_Prepare Start\n", __func__);
			rc = VL6180x_Prepare(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("%s: VL6180x_Prepare Success\n", __func__);


			API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
			/*rc = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);*/

			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			printk("%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
		
			break;
		case MSM_LASER_FOCUS_DEVICE_INIT_CCI:
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			rc = VL6180x_init(vl6180x_t);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} 

			API_DBG("%s: VL6180x_InitData Start\n", __func__);
			rc = VL6180x_InitData(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
			API_DBG("%s: VL6180x_InitData Success\n", __func__);

			API_DBG("%s: VL6180x_Prepare Start\n", __func__);
			rc = VL6180x_Prepare(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
			API_DBG("%s: VL6180x_Prepare Success\n", __func__);

			rc = VL6180x_device_Load_Calibration_Value();
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
				
			API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
			/*rc = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
			API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);*/

			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			printk("%s Init Device (%d)\n", __func__, vl6180x_t->device_state);

			camera_on_flag = true;
			
			break;
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			mutex_lock(&vl6180x_mutex);
			rc = VL6180x_deinit(vl6180x_t);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_DEINIT_CCI;
			camera_on_flag = false;
			mutex_unlock(&vl6180x_mutex);
			break;
		default:
			printk("%s commond fail !!\n", __func__);
			break;
	}
	return len;

DEVICE_TURN_ON_ERROR:
	rc = VL6180x_deinit(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_deinit failed %d\n", __func__, __LINE__);
	}
		
	rc = VL6180x_power_down(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
	}
	return -EIO;
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

static int ATD_VL6180x_device_read_range(VL6180x_RangeData_t *pRangeData)
{
	uint16_t RawRange;
	int status, i = 0;
	struct msm_camera_i2c_client *sensor_i2c_client;
	uint8_t intStatus;
	//VL6180x_RangeData_t RangeData;

	timer = get_current_time();
	
	/* Setting i2c client */
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	/* Setting Range meansurement in single-shot mode */	
	API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	status = VL6180x_RangeClearInterrupt(0);
	if (status < 0) {
		pr_err("%s: rVL6180x_RangeClearInterrupt failed\n", __func__);
		return status;
	}
	API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	API_DBG("%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	status = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
	if (status < 0) {
		pr_err("%s: VL6180x_RangeSetSystemMode failed\n", __func__);
		return status;
	}
	API_DBG("%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	/* Get Sensor detect distance */
	RawRange = 0;
	for (i = 0; i <1000; i++)	{
		/* Check RESULT_INTERRUPT_STATUS_GPIO */
		API_DBG("%s: VL6180x_RangeGetInterruptStatus Start\n", __func__);
		status = VL6180x_RangeGetInterruptStatus(0, &intStatus);
		if (status < 0) {
			pr_err("%s: VL6180x_RangeGetInterruptStatus failed\n", __func__);
			return status;
		}
		
		API_DBG("%s: VL6180x_RangeGetInterruptStatus Success\n", __func__);
		
		if (intStatus == RES_INT_STAT_GPIO_NEW_SAMPLE_READY){
			API_DBG("%s: VL6180x sensor ready! after loop:%d times GetInterruptStatus\n", __func__,i);
			break;
		}
		
		timer2 = get_current_time();
		if((((timer2.tv_sec*1000000)+timer2.tv_usec)-((timer.tv_sec*1000000)+timer.tv_usec)) > (TIMEOUT_VAL*1000)){
			printk("%s: Timeout: Out Of Range!!\n", __func__);
			return OUT_OF_RANGE;
		}
	}
	
	if(i < 1000){		
		status = VL6180x_RangeGetMeasurement(0, pRangeData);
		if (status < 0) {
			pr_err("%s: VL6180x_RangeGetMeasurement failed\n", __func__);
			return status;
		}
		
		if (pRangeData->errorStatus == 0){
			DBG_LOG("%s: Read range:%d\n", __func__, (int)pRangeData->range_mm);
			API_DBG("%s: VL6180x_RangeGetMeasurement Success\n", __func__);
		}
		else{
			API_DBG("%s: VL6180x_RangeGetMeasurement Failed: errorStatus(%d)\n", __func__, pRangeData->errorStatus);
			pRangeData->range_mm = OUT_OF_RANGE;
		}
	}
	else
	{
		API_DBG("%s: VL6180x sensor no ready!\n", __func__);
		pRangeData->range_mm = OUT_OF_RANGE;
	}

	/* Setting SYSTEM_INTERRUPT_CLEAR to 0x01 */
	API_DBG("%s: VL6180x_RangeClearInterrupt Start\n", __func__);
	status = VL6180x_RangeClearInterrupt(0);
	if (status < 0) {
		pr_err("%s: VL6180x_RangeClearInterrupt failed\n", __func__);
		return status;
	}
	API_DBG("%s: VL6180x_RangeClearInterrupt Success\n", __func__);
	
#if VL6180x_WRAP_AROUND_FILTER_SUPPORT
	printk("[LF][vl6180x]: %d (%d:%d:%d)\n", pRangeData->range_mm, 
		pRangeData->FilteredData.filterError,
		pRangeData->DMax,
		pRangeData->errorStatus);
#else
	printk("[LF][vl6180x]: %d (%d:%d)\n", pRangeData->range_mm, 
		pRangeData->DMax,
		pRangeData->errorStatus);
#endif

	return (int)pRangeData->range_mm;
}

static int ATD_VL6180x_device_get_range_read(struct seq_file *buf, void *v)
{
	int RawRange = 0;
	
	uint16_t test_0x1e_Value = 0;
	uint8_t test_0x24_Value = 0;
	uint8_t test_0x4d_Value = 0;
	uint8_t test_RangeValue = 0;
	uint8_t test_RawRange = 0;
	uint16_t test_rtnRate = 0;
	uint32_t test_DMax = 0;
	VL6180x_RangeData_t RangeData;

	mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		DBG_LOG("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_unlock(&vl6180x_mutex);
		return 0;
		//return -EBUSY;
	}
	
	RawRange = ATD_VL6180x_device_read_range(&RangeData);
	//RawRange = RawRange*3;

	DBG_LOG("%s Test Data (%d)  Device (%d)\n", __func__, RawRange , vl6180x_t->device_state);

	if (RawRange < 0) {
		pr_err("%s: read_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}

	DMax = RangeData.DMax;
	errorStatus = RangeData.errorStatus;

	seq_printf(buf, "%d\n", RawRange);
	
	
	//create reg file
	
	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION){
		ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE,&test_0x1e_Value);//0x1e
		VL6180x_RdByte(0,SYSRANGE_PART_TO_PART_RANGE_OFFSET,&test_0x24_Value);//0x24
		VL6180x_RdByte(0,RESULT_RANGE_STATUS,&test_0x4d_Value);//0x4d
		VL6180x_RdByte(0,RESULT_RANGE_VAL,&test_RangeValue);//0x62
		VL6180x_RdByte(0,RESULT_RANGE_RAW,&test_RawRange);	//0x64
		ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE,&test_rtnRate);//0x66
		
		test_DMax = RangeData.DMax;//DMax
		
		DBG_LOG("[LF][vl6180x] reg value: (0x1e):%d,(0x24):%d,(0x4d):%d,(0x62):%d,(0x64):%d,(0x66):%d,(DMax):%d",test_0x1e_Value,test_0x24_Value,test_0x4d_Value,test_RangeValue,test_RawRange,test_rtnRate,test_DMax);
		
		if (Laser_Forcus_sysfs_write_0x1e(test_0x1e_Value) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x24(test_0x24_Value) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x4d(test_0x4d_Value) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x62(test_RangeValue) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x64(test_RawRange) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x66(test_rtnRate) == false)
			return -ENOENT;

		if (Laser_Forcus_sysfs_write_DMax(test_DMax) == false)
			return -ENOENT;
	}
#if 0
	if(camera_on_flag == false){
		debug_read_range(&RangeData);
	}
#endif	

	mutex_unlock(&vl6180x_mutex);

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

static int ATD_VL6180x_device_get_more_value_read(struct seq_file *buf, void *v)
{
	int RawRange = 0;
	
	uint16_t test_0x1e_Value = 0;
	uint8_t test_0x24_Value = 0;
	uint8_t test_0x4d_Value = 0;
	uint8_t test_RangeValue = 0;
	uint8_t test_RawRange = 0;
	uint16_t test_rtnRate = 0;
	uint32_t test_DMax = 0;
	VL6180x_RangeData_t RangeData;

	mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		DBG_LOG("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_unlock(&vl6180x_mutex);
		return 0;
		//return -EBUSY;
	}
	
	RawRange = ATD_VL6180x_device_read_range(&RangeData);
	//RawRange = RawRange*3;

	DBG_LOG("%s Test Data (%d)  Device (%d)\n", __func__, RawRange , vl6180x_t->device_state);

	if (RawRange < 0) {
		pr_err("%s: read_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}

	DMax = RangeData.DMax;
	errorStatus = RangeData.errorStatus;

	seq_printf(buf, "%d#%d#%d\n", RawRange, RangeData.DMax, RangeData.errorStatus);
	//create reg file
	
	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION){
		ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE,&test_0x1e_Value);//0x1e
		VL6180x_RdByte(0,SYSRANGE_PART_TO_PART_RANGE_OFFSET,&test_0x24_Value);//0x24
		VL6180x_RdByte(0,RESULT_RANGE_STATUS,&test_0x4d_Value);//0x4d
		VL6180x_RdByte(0,RESULT_RANGE_VAL,&test_RangeValue);//0x62
		VL6180x_RdByte(0,RESULT_RANGE_RAW,&test_RawRange);	//0x64
		ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE,&test_rtnRate);//0x66
		
		test_DMax = RangeData.DMax;//DMax
		
		DBG_LOG("[LF][vl6180x] reg value: (0x1e):%d,(0x24):%d,(0x4d):%d,(0x62):%d,(0x64):%d,(0x66):%d,(DMax):%d",test_0x1e_Value,test_0x24_Value,test_0x4d_Value,test_RangeValue,test_RawRange,test_rtnRate,test_DMax);
		
		if (Laser_Forcus_sysfs_write_0x1e(test_0x1e_Value) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x24(test_0x24_Value) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x4d(test_0x4d_Value) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x62(test_RangeValue) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x64(test_RawRange) == false)
			return -ENOENT;
		
		if (Laser_Forcus_sysfs_write_0x66(test_rtnRate) == false)
			return -ENOENT;

		if (Laser_Forcus_sysfs_write_DMax(test_DMax) == false)
			return -ENOENT;
	}
#if 0
	if(camera_on_flag == false){
		debug_read_range(&RangeData);
	}
#endif	
	mutex_unlock(&vl6180x_mutex);

	return 0;
}

static int ATD_VL6180x_device_get_more_value_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_device_get_more_value_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_more_value_fos = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_get_more_value_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_VL6180x_device_clibration_offset(void)
{
	int i = 0, RawRange = 0, sum = 0;
	uint16_t distance;
	int16_t offset;
	int status= 0;
	VL6180x_RangeData_t RangeData;
	int errorCount = 0;

	mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		DBG_LOG("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		mutex_unlock(&vl6180x_mutex);
		return -EBUSY;
	}
	
	/* Clean system offset */
	//VL6180x_SetOffsetCalibrationData(0, 0);
	VL6180x_SetXTalkCompensationRate(0, 0);
	ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x00);
	//ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);
	
	for(i=0; i<STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{
		RawRange = (int)ATD_VL6180x_device_read_range(&RangeData);
		//msleep(50);
		printk("[LF][vl6180x]: %d, %d\n", RangeData.range_mm, RangeData.signalRate_mcps);
		if( RangeData.errorStatus != 0 )
		{
			errorCount++;
			if( i >= 0 )
			{
				i--;
			}			
			
			if( errorCount > VL6180_API_ERROR_COUNT_MAX )
			{
				DBG_LOG("[LF][vl6180x]: too many 765 detected in offset calibration (%d)\n", errorCount);
				goto error;
			}
			
			continue;
		}
		sum += RawRange;
	}
	distance = (uint16_t)(sum / STMVL6180_RUNTIMES_OFFSET_CAL);
	DBG_LOG("The measure distanec is %d mm\n", distance);

	if((VL6180_OFFSET_CAL_RANGE - distance)<0){
		offset = (VL6180_OFFSET_CAL_RANGE - distance)/3;
		offset = 256+offset;
	}else{
		offset = (VL6180_OFFSET_CAL_RANGE - distance)/3;
	}
	
	//VL6180x_SetOffsetCalibrationData(0, offset);
	status = ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
	if (status < 0) {
		pr_err("%s: write register(0x%x) failed\n", __func__, SYSRANGE_PART_TO_PART_RANGE_OFFSET);
		return status;
	}

	/* Write calibration file */
	vl6180x_t->laser_focus_offset_value = offset;
	if (Laser_Forcus_sysfs_write_offset(offset) == false){
		mutex_unlock(&vl6180x_mutex);
		return -ENOENT;
	}
	/*
	if (Laser_Forcus_sysfs_write_offset_to_persist(offset) == false){
		mutex_unlock(&vl6180x_mutex);
		return -ENOENT;
	}*/	

	DBG_LOG("The measure distance is %d mm; The offset value is %d\n", distance, offset);

	mutex_unlock(&vl6180x_mutex);
	return 0;
	
error:
	
	ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x00);

	/* Write calibration file */
	vl6180x_t->laser_focus_offset_value = 0;
	Laser_Forcus_sysfs_write_offset(-1);
	Laser_Forcus_sysfs_write_offset_to_persist(-1);

	mutex_unlock(&vl6180x_mutex);
	
	return -ENOENT;
}

static int ATD_VL6180x_device_clibration_crosstalkoffset(void)
{
	int i = 0, RawRange = 0;
	int xtalk_sum  = 0, xrtn_sum = 0;
	uint16_t XtalkCompRate;
	uint16_t rtnRate = 0;
	uint16_t Data_value = 0;
	VL6180x_RangeData_t RangeData;
	int errorCount = 0;
	uint16_t temp = 0;

	mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		DBG_LOG("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		mutex_unlock(&vl6180x_mutex);
		return -EBUSY;
	}

	/* Clean crosstalk offset */
	VL6180x_SetXTalkCompensationRate(0, 0);
	//ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);

	for(i = 0; i < STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{	
		RawRange = ATD_VL6180x_device_read_range(&RangeData);
		printk("[LF][vl6180x]: %d, %d\n", RangeData.range_mm, RangeData.signalRate_mcps);
		if( RangeData.errorStatus != 0 )
		{
			errorCount++;
			if( i >= 0 )
			{
				i--;
			}			
			
			if( errorCount > VL6180_API_ERROR_COUNT_MAX )
			{
				DBG_LOG("[LF][vl6180x]: too many 765 detected in xtalk calibration (%d)\n", errorCount);
				goto error;
			}
			
			continue;
		}
		//ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE, &rtnRate);
		rtnRate = RangeData.signalRate_mcps;
		xtalk_sum += RawRange;
		xrtn_sum += (int)rtnRate;

		//msleep(30);
	}
	printk("Crosstalk compensation rate(sum) is %d\n", xtalk_sum);
	
	//XtalkCompRate = (uint16_t)((rtnRate/STMVL6180_RUNTIMES_OFFSET_CAL)/128) * (1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*3000/VL6180_CROSSTALK_CAL_RANGE));
	//XtalkCompRate = XtalkCompRate*128;

	//Sean_Lu ++++ 0812 for factory test
	if((1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*1000/VL6180_CROSSTALK_CAL_RANGE) < 0))
	{
		printk("Crosstalk  is negative value !\n");
		ASUS_VL6180x_RdByte(0x21,&Data_value);

		if(Data_value==20)
		{
			ASUS_VL6180x_WrByte(0x21,0x6);
		}

		mutex_unlock(&vl6180x_mutex);

		goto negative;
	}
	else
	{
	
		XtalkCompRate = (uint16_t)(xrtn_sum/STMVL6180_RUNTIMES_OFFSET_CAL) * (1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*1000/VL6180_CROSSTALK_CAL_RANGE));
		temp = XtalkCompRate;
		
		printk("Crosstalk compensation rate(after arithmetic) is %d\n", XtalkCompRate);

		if( asus_PRJ_ID == 3 )  // 3 is ZD550KL
		{
			printk("Crosstalk compensation rate is %d( *1000 )\n", XtalkCompRate);
			if( XtalkCompRate > 500  && XtalkCompRate < 1000 ) 
				XtalkCompRate = 1000;
		}
		XtalkCompRate = XtalkCompRate/1000;

		ASUS_VL6180x_RdByte(0x21,&Data_value);

		if(Data_value==20)
		{
			ASUS_VL6180x_WrByte(0x21,0x6);
		}

		printk("Crosstalk compensation rate(after/1000) is %d\n", XtalkCompRate);

		//ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, XtalkCompRate);
		VL6180x_SetXTalkCompensationRate(0, XtalkCompRate);

		/* Write calibration file */
		vl6180x_t->laser_focus_cross_talk_offset_value = XtalkCompRate;
		if (Laser_Forcus_sysfs_write_cross_talk_offset(XtalkCompRate) == false){
			mutex_unlock(&vl6180x_mutex);
			return -ENOENT;
		}
	/*
		if (Laser_Forcus_sysfs_write_cross_talk_offset_to_persist(XtalkCompRate) == false){
			mutex_unlock(&vl6180x_mutex);
			return -ENOENT;
		}
*/
		DBG_LOG("Crosstalk compensation rate is %d\n", XtalkCompRate);

		mutex_unlock(&vl6180x_mutex);
		return 0;
	}

negative:

	VL6180x_SetXTalkCompensationRate(0, 0);

	/* Write calibration file */
	vl6180x_t->laser_focus_cross_talk_offset_value = 0;
	Laser_Forcus_sysfs_write_cross_talk_offset(0);
	Laser_Forcus_sysfs_write_cross_talk_offset_to_persist(0);

	mutex_unlock(&vl6180x_mutex);

	return 0;
	
error:
	
	VL6180x_SetXTalkCompensationRate(0, 0);

	/* Write calibration file */
	vl6180x_t->laser_focus_cross_talk_offset_value = 0;
	Laser_Forcus_sysfs_write_cross_talk_offset(-1);
	Laser_Forcus_sysfs_write_cross_talk_offset_to_persist(-1); 

	mutex_unlock(&vl6180x_mutex);
	
	return -ENOENT;
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
	
	/* VL6180x only */
	if(g_ASUS_laserID == 1){
			rc = VL6180x_power_up(vl6180x_t);
			if (rc < 0) {
				//kfree(vl6180x_t);
				pr_err("%s VL6180x_power_up failed %d\n", __func__, __LINE__);
				return 0;
			}
	}
	else{
		rc = VL6180x_power_up(vl6180x_t);
		if (rc < 0) {
			//kfree(vl6180x_t);
			pr_err("%s VL6180x_power_up failed %d\n", __func__, __LINE__);
			return 0;
		}
	}
	VL6180x_init(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_init failed %d\n", __func__, __LINE__);
		rc = VL6180x_power_down(vl6180x_t);
		if (rc < 0)
			pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
		else
			pr_err("%s VL6180x_power_down Success %d\n", __func__, __LINE__);
		return 0;
	}
	
	rc = VL6180x_match_id(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_match_id failed %d\n", __func__, __LINE__);
		rc = VL6180x_deinit(vl6180x_t);
		if (rc < 0)
			pr_err("%s VL6180x_deinit failed %d\n", __func__, __LINE__);
		else
			pr_err("%s VL6180x_deinit Success %d\n", __func__, __LINE__);

		rc = VL6180x_power_down(vl6180x_t);
		if (rc < 0) 
			pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
		else
			pr_err("%s VL6180x_power_down Success %d\n", __func__, __LINE__);
		return 0;
	}
	rc = VL6180x_deinit(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_deinit failed %d\n", __func__, __LINE__);
		return 0;
	}
	
	/* VL6180x only */
	if(g_ASUS_laserID == 1){
			rc = VL6180x_power_down(vl6180x_t);
			if (rc < 0) {
				//kfree(vl6180x_t);
				pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
				return 0;
			}
	}else{
		rc = VL6180x_power_down(vl6180x_t);
		if (rc < 0) {
			//kfree(vl6180x_t);
			pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
			return 0;
		}
	}

	vl6180x_check_status = 1;

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

static int VL6180x_I2C_status_check_via_prob(struct msm_laser_focus_ctrl_t *s_ctrl){
	return vl6180x_check_status;
}

static int VL6180x_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = VL6180x_I2C_status_check_via_prob(vl6180x_t);
	
	seq_printf(buf, "%d\n", ATD_status);
	return 0;
}

static int VL6180x_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, VL6180x_I2C_status_check_proc_read, NULL);
}

static const struct file_operations I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = VL6180x_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_VL6180x_register_read(struct seq_file *buf, void *v)
{
	int status, i = 0;
	uint16_t register_value = 0;

	for (i = 0; i <0x100; i++)	{
		register_value = 0;
		status = ASUS_VL6180x_RdWord(i, &register_value);
		if (status < 0) {
			pr_err("%s: read register(0x%x) failed\n", __func__, i);
			return status;
		}
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

static int dump_VL6180x_debug_register_read(struct seq_file *buf, void *v)
{
	uint16_t reg_data = 0;
	
	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		DBG_LOG("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		return -EBUSY;
	}
	
	ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", SYSRANGE_CROSSTALK_COMPENSATION_RATE, reg_data);
	ASUS_VL6180x_RdByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", SYSRANGE_PART_TO_PART_RANGE_OFFSET, reg_data);
	ASUS_VL6180x_RdByte(RESULT_RANGE_STATUS, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", RESULT_RANGE_STATUS, reg_data);
	ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE, &reg_data);
	seq_printf(buf, "register(0x%x) : 0x%x\n", RESULT_RANGE_SIGNAL_RATE, reg_data);
	seq_printf(buf, "DMax : %d\n", DMax);
	seq_printf(buf, "errorStatus : %d\n", errorStatus);
	return 0;
}

static int dump_VL6180x_laser_focus_debug_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_VL6180x_debug_register_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_VL6180x_laser_focus_debug_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static ssize_t set_VL6180x_laser_focus_register_write(struct file *dev, const char *buff, size_t count, loff_t *loff)
{
	uint32_t reg_addr;
	uint16_t reg_val;
	int mode;
	int reg_addr_store;
	int reg_val_store;
	sscanf(buff, "%d %x %x", &mode, &reg_addr_store, &reg_val_store);
	
	reg_val = reg_val_store & 0xFFFF;
    reg_addr = reg_addr_store & 0xFFFFFFFF;
    
    switch(mode){
		case 0:{			
			printk("[LF][vl6180x] %s reg_addr:%x,reg_val:%x", __func__,reg_addr, reg_val);
			VL6180x_WrWord( 0, reg_addr, reg_val);
			msleep(30);	
			VL6180x_RdWord(0,reg_addr, &reg_val); 
			printk("[LF][vl6180x] %s reg_addr:%x,reg_val:%x", __func__,reg_addr, reg_val);			
			break;
		}
		case 1:{
			
			VL6180x_WrWord( 0, 0x0026, 0x26);
			VL6180x_WrByte( 0, 0x002d, 0x12);
			
			break;
		}
		case 2:{
			
			VL6180x_WrByte( 0, 0x0025, 0xff);
			
			break;
		}
		default:{
		
			break;
		}
	}
	
	return count;
}

static const struct file_operations set_laser_focus_register_fops = {
	.write = set_VL6180x_laser_focus_register_write,
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
	
	device_get_more_value_file = proc_create(DEVICE_GET_MORE_VALUE, 0776, NULL, &ATD_laser_focus_device_get_more_value_fos);
	if (device_get_more_value_file) {
		printk("%s device_get_more_value_file sucessed!\n", __func__);
	} else {
		printk("%s device_get_more_value_file failed!\n", __func__);
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

	dump_laser_focus_debug_file = proc_create(DEVICE_DUMP_DEBUG_REGISTER_VALUE, 0664, NULL, &dump_laser_focus_debug_register_fops);
	if (dump_laser_focus_debug_file) {
		printk("%s dump_laser_focus_debug_register_file sucessed!\n", __func__);
	} else {
		printk("%s dump_laser_focus_debug_register_file failed!\n", __func__);
	}
	
	set_laser_focus_register_file = proc_create(DEVICE_SET_REGISTER_VALUE, 0776, NULL, &set_laser_focus_register_fops);
	if (set_laser_focus_register_file) {
		printk("%s set_laser_focus_register_file sucessed!\n", __func__);
	} else {
		printk("%s set_laser_focus_register_file failed!\n", __func__);
	}

	status_proc_file = proc_create(STATUS_PROC_FILE_FOR_CAMERA, 0776, NULL, &I2C_status_check_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
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

	DBG_LOG("%s: read id: 0x%x expected id 0x%x:\n", __func__, chipid,
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
			pr_err("cci_deinit failed\n");
		
	}


	CDBG("Exit\n");
	return rc;
}

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

	DBG_LOG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&sensordata->sensor_name);
	DBG_LOG("%s label %s, rc %d\n", __func__,
		sensordata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_master);
	DBG_LOG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_master,
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
	
	/*VL6180x only */
	if(g_ASUS_laserID == 0){
				printk("[LASER_FOCUS] It is Laura sensor, do nothing!!\n");
				return -1;
	}
	
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
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
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
	mutex_init(&vl6180x_mutex);
	rc = platform_driver_probe(&msm_laser_focus_platform_driver,
		VL6180x_platform_probe);
	DBG_LOG("%s:%d rc %d\n", __func__, __LINE__, rc);

	return rc;
}
static void __exit VL6180x_driver_exit(void)
{
	CDBG("Enter");
	platform_driver_unregister(&msm_laser_focus_platform_driver);
	return;
}

int ASUS_VL6180x_RdMulti(uint32_t register_addr, uint8_t *i2c_read_data, uint16_t num_byte)
{
	int status;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	
	/* Setting i2c client */
	struct msm_camera_i2c_client *sensor_i2c_client;
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}

	reg_setting.reg_data_size = num_byte;

	status = (int)sensor_i2c_client->i2c_func_tbl->i2c_read_seq(sensor_i2c_client, register_addr, 
		i2c_read_data, reg_setting.reg_data_size);
	
	if (status < 0) {
		pr_err("%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}
	
	//*i2c_read_data=reg_setting.reg_data;
	REG_RW_DBG("%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);

	return status;
}



module_init(VL6180x_init_module);
module_exit(VL6180x_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
