/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"
#include "../msm_sensor.h"
#include "msm_led_flash.h"
#include "../cci/msm_cci.h"
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>

#define FLASH_NAME "camera-led-flash"
#define CAM_FLASH_PINCTRL_STATE_SLEEP "cam_flash_suspend"
#define CAM_FLASH_PINCTRL_STATE_DEFAULT "cam_flash_default"
static struct mutex 								flash_lock;
static struct mutex 								flashlight_lock;

static bool asus_flash_status;

enum led_direction_t {
	REAR_LED = 0,
	FRONT_LED = 1,
};

static enum led_direction_t led_direction = REAR_LED;   //
#define ENABLE_FLASH_SELECT_PROC 1
#define	FLASH_SELECT_PROC_FILE "driver/Flash_Select"
static struct proc_dir_entry *flash_select_proc_file;


/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#define CDBG(fmt, args...) pr_info(fmt, ##args)

struct msm_led_flash_ctrl_t *g_fctrl = NULL;
int msm_flash_led_low_current_set(struct msm_led_flash_ctrl_t *fctrl, int intensity1, int intensity2);
int msm_flash_led_high_current_set(struct msm_led_flash_ctrl_t *fctrl, int intensity1, int intensity2);


bool is_ZD550KL( void )
{
	switch (asus_PRJ_ID) {
        case 0://ASUS_ZE550KL
            printk("ASUS_ZE550KL platform_data\n");
            return false;
        case 1://ASUS_ZE600KL
		printk("ASUS_ZE600KL platform_data\n");
            return false;
        case 2://ASUS_ZX550KL
		printk("ASUS_ZX550KL platform_data\n");
            return false;
        case 3://ASUS_ZD550KL
		printk("ASUS_ZD550KL platform_data\n");
            return true;
        default:
		printk("default platform_data\n");
            return false;
    }

/*
	// [ro.product.model]: [ASUS_Z00UD]
	char prop[PATH_MAX];
	if(property_get("ro.product.model", prop, NULL) > 0) {
		printk("%s\t %s leong_p", __FUNCTION__, prop );
        if((strncmp(prop, "ASUS_Z00UD", 10) == 0 ){
		printk( "is ZD500 leong_p\n" );
			return true;
		}
    }
    return false;*/
}

bool is_ZE600KL_ZE601KL (void) {
    switch (asus_PRJ_ID) {
        case 0://ASUS_ZE550KL
                printk("ASUS_ZE550KL platform_data\n");
                return false;
        case 1://ASUS_ZE600KL
                printk("ASUS_ZE600KL platform_data\n");
                return true;
        case 2://ASUS_ZX550KL
                printk("ASUS_ZX550KL platform_data\n");
                return false;
        case 3://ASUS_ZD550KL
                printk("ASUS_ZD550KL platform_data\n");
                return false;
        default:
                printk("default platform_data\n");
                return false;
        }
}


#if ENABLE_FLASH_SELECT_PROC
static int flash_select_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", led_direction);
    return 0;
}

static ssize_t flash_select_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
//	int val;
	char messages[8];
        int i =0;
        for(i=0;i<8;i++) messages[i]=0;
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	pr_err("%s\t messages : %s  buff : %s  leong_p_flash\n", __func__, messages, buff );

	if( strncmp(messages, "REAR", 4) == 0 )
		led_direction = REAR_LED;
	else
		led_direction = FRONT_LED;
	pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
/*
	val = (int)simple_strtol(messages, NULL, 10);
	if( val < 0 )
	{
		pr_err("%s\t messages:%s simple_strtol fail.",__func__, messages);
		return 0;
	}

	printk("%s commond : %d\n", __func__, val);

	if( val == REAR_LED || val == FRONT_LED )
		led_direction = val;
*/
	return len;
}

static int flash_select_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, flash_select_proc_read, NULL);
}

static const struct file_operations flash_select_fops = {
	.owner = THIS_MODULE,
	.open = flash_select_proc_open,
	.write = flash_select_proc_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_flash_select_proc_file(void)
{
	flash_select_proc_file = proc_create(FLASH_SELECT_PROC_FILE, 0776, NULL, &flash_select_fops);
	if (flash_select_proc_file) {
		printk("%s (sky81296)flash_select_proc_file sucessed!\n", __func__);
	} else {
		printk("%s (sky81296)flash_select_proc_file failed!\n", __func__);
	}
}

#endif  // ENABLE_FLASH_SELECT_PROC end


int32_t msm_led_i2c_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	*subdev_id = fctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	return 0;
}

int32_t msm_led_i2c_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data)
{
	int rc = 0;
	//int i = 0;
	struct msm_camera_led_cfg_t *cfg = (struct msm_camera_led_cfg_t *)data;
	CDBG("called led_state %d led_direction %d \n", cfg->cfgtype, led_direction);

	if (!fctrl->func_tbl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	switch (cfg->cfgtype) {

	case MSM_CAMERA_LED_INIT:
		mutex_lock(&flashlight_lock);
		if (g_fctrl->flashlight_state == MSM_CAMERA_LED_INIT) {
			printk(" now flashlight already init, so release it first\n");
			msm_flash_led_release(g_fctrl);
			g_fctrl->flashlight_state = MSM_CAMERA_LED_RELEASE;
		}
		if (fctrl->func_tbl->flash_led_init)
			rc = fctrl->func_tbl->flash_led_init(fctrl);
		/*for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			cfg->flash_current[i] =
				fctrl->flash_max_current[i];
			cfg->flash_duration[i] =
				fctrl->flash_max_duration[i];
			cfg->torch_current[i] =
				fctrl->torch_max_current[i];
		}*/
		g_fctrl->led_state = MSM_CAMERA_LED_INIT;
		fctrl->led_state = MSM_CAMERA_LED_INIT;
		mutex_unlock(&flashlight_lock);
		break;

	case MSM_CAMERA_LED_RELEASE:
		if (fctrl->func_tbl->flash_led_release)
			rc = fctrl->func_tbl->
				flash_led_release(fctrl);
		g_fctrl->led_state = MSM_CAMERA_LED_RELEASE;
		fctrl->led_state = MSM_CAMERA_LED_RELEASE;
		break;

	case MSM_CAMERA_LED_OFF:
		if (fctrl->func_tbl->flash_led_off)
			rc = fctrl->func_tbl->flash_led_off(fctrl);
		break;

	case MSM_CAMERA_LED_LOW:
		/*for (i = 0; i < torch_num_sources; i++) {
			if (fctrl->torch_max_current[i] > 0) {
				fctrl->torch_op_current[i] =
					(cfg->torch_current[i] < fctrl->torch_max_current[i]) ?
					cfg->torch_current[i] : fctrl->torch_max_current[i];
				CDBG("torch source%d: op_current %d max_current %d\n",
					i, fctrl->torch_op_current[i], fctrl->torch_max_current[i]);
			}
		}*/
		msm_flash_led_low_current_set(fctrl,cfg->torch_current[0],cfg->torch_current[1]);
		if (cfg->torch_current[0] != 0 && cfg->torch_current[1] != 0) {
			if (fctrl->func_tbl->flash_led_low)
				rc = fctrl->func_tbl->flash_led_low(fctrl);
			break;
		} else if (cfg->torch_current[0] != 0 && cfg->torch_current[1] == 0) {
			if (fctrl->func_tbl->flash_led_low_first)
				rc = fctrl->func_tbl->flash_led_low_first(fctrl);
			break;
		} else if (cfg->torch_current[0] == 0 && cfg->torch_current[1] != 0) {
			if (fctrl->func_tbl->flash_led_low_second)
				rc = fctrl->func_tbl->flash_led_low_second(fctrl);
			break;
		} else {
			if (fctrl->func_tbl->flash_led_off)
				rc = fctrl->func_tbl->flash_led_off(fctrl);
			break;
		}
	case MSM_CAMERA_LED_HIGH:
		/*for (i = 0; i < fctrl->flash_num_sources; i++) {
			if (fctrl->flash_max_current[i] > 0) {
				fctrl->flash_op_current[i] =
					(cfg->flash_current[i] < fctrl->flash_max_current[i]) ?
					cfg->flash_current[i] : fctrl->flash_max_current[i];
				CDBG("flash source%d: op_current %d max_current %d\n",
					i, fctrl->flash_op_current[i], fctrl->flash_max_current[i]);
			}
		}*/
		//msm_flash_led_high_timer_set(fctrl,cfg->flash_duration[0],cfg->flash_duration[1]);
		msm_flash_led_high_current_set(fctrl,cfg->flash_current[0],cfg->flash_current[1]);
		if (cfg->flash_current[0] != 0 && cfg->flash_current[1] != 0) {
		if (fctrl->func_tbl->flash_led_high)
			rc = fctrl->func_tbl->flash_led_high(fctrl);
		break;
		} else if (cfg->flash_current[0] != 0 && cfg->flash_current[1] == 0) {
			if (fctrl->func_tbl->flash_led_high_first)
				rc = fctrl->func_tbl->flash_led_high_first(fctrl);
			break;
		} else if (cfg->flash_current[0] == 0 && cfg->flash_current[1] != 0) {
			if (fctrl->func_tbl->flash_led_high_second)
				rc = fctrl->func_tbl->flash_led_high_second(fctrl);
			break;
		} else {
			if (fctrl->func_tbl->flash_led_off)
				rc = fctrl->func_tbl->flash_led_off(fctrl);
			break;
		}
	default:
		rc = -EFAULT;
		break;
	}
	CDBG("flash_set_led_state: return %d\n", rc);
	return rc;
}
static int msm_flash_pinctrl_init(struct msm_led_flash_ctrl_t *ctrl)
{
	struct msm_pinctrl_info *flash_pctrl = NULL;
	pr_info("%s entry\n", __func__);
	flash_pctrl = &ctrl->pinctrl_info;
	flash_pctrl->pinctrl = devm_pinctrl_get(&ctrl->pdev->dev);

	if (IS_ERR_OR_NULL(flash_pctrl->pinctrl)) {
		pr_err("%s:%d Getting pinctrl handle failed\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_active = pinctrl_lookup_state(
					       flash_pctrl->pinctrl,
					       CAM_FLASH_PINCTRL_STATE_DEFAULT);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_active)) {
		pr_err("%s:%d Failed to get the active state pinctrl handle\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	flash_pctrl->gpio_state_suspend = pinctrl_lookup_state(
						flash_pctrl->pinctrl,
						CAM_FLASH_PINCTRL_STATE_SLEEP);

	if (IS_ERR_OR_NULL(flash_pctrl->gpio_state_suspend)) {
		pr_err("%s:%d Failed to get the suspend state pinctrl handle\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	//asus-Andrew
     if(is_ZE600KL_ZE601KL()) {
          #define CAM_FLASH_PINCTRL_STATE_INIT "flash_err_default"
             struct pinctrl_state *init_flash_pin = NULL;
          init_flash_pin = pinctrl_lookup_state(
             flash_pctrl->pinctrl,
             CAM_FLASH_PINCTRL_STATE_INIT);
          pinctrl_select_state(flash_pctrl->pinctrl, init_flash_pin);
     }

	return 0;
}


int msm_flash_led_init(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s:%d called led_direction %d\n", __func__, __LINE__, led_direction);
	mutex_lock(&flash_lock);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}

	if (fctrl->led_state == MSM_CAMERA_LED_INIT&&fctrl->flashlight_state == MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	fctrl->flashlight_state = MSM_CAMERA_LED_RELEASE;

	if (power_info->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
	}

	/* CCI Init */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("cci_init failed\n");
			mutex_unlock(&flash_lock);
			return rc;
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		mutex_unlock(&flash_lock);
		return rc;
	}

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		//CDBG("%s:%d PC:: flash pins setting to active state",
		//		__func__, __LINE__);
		rc = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_active);
		if (rc < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to active state",
					__func__, __LINE__);
		}
	}
	msleep(20);
/*
	CDBG("before FL_RESET\n");
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_HIGH);
*/
//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_LOW);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_LOW);
		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

//<asus-leong_un20150401<<<<<<<<<+
	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->init_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	//fctrl->led_state = MSM_CAMERA_LED_INIT;
	mutex_unlock(&flash_lock);
	CDBG("%s:%d end\n", __func__, __LINE__);
	return rc;
}

int msm_flash_led_release(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0, ret = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if (fctrl->led_state != MSM_CAMERA_LED_INIT&& fctrl->flashlight_state != MSM_CAMERA_LED_INIT) {
		pr_err("%s:%d invalid led state\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}

	if( is_ZD550KL() )
	{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_LOW);

			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_LOW);

	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_LOW);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
	if (power_info->gpio_conf->gpio_num_info->
			valid[SENSOR_GPIO_FL_RESET] == 1)
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_RESET],
			GPIO_OUT_LOW);

	if (fctrl->pinctrl_info.use_pinctrl == true) {
		ret = pinctrl_select_state(fctrl->pinctrl_info.pinctrl,
				fctrl->pinctrl_info.gpio_state_suspend);
		if (ret < 0) {
			devm_pinctrl_put(fctrl->pinctrl_info.pinctrl);
			pr_err("%s:%d cannot set pin to suspend state",
				__func__, __LINE__);
		}
	}
	rc = msm_camera_request_gpio_table(
		power_info->gpio_conf->cam_gpio_req_tbl,
		power_info->gpio_conf->cam_gpio_req_tbl_size, 0);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		mutex_unlock(&flash_lock);
		return rc;
	}

	//fctrl->led_state = MSM_CAMERA_LED_RELEASE;
	/* CCI deInit */
	if (fctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_util(
			fctrl->flash_i2c_client, MSM_CCI_RELEASE);
		if (rc < 0)
			pr_err("cci_deinit failed\n");
	}

	asus_flash_status = false;

	mutex_unlock(&flash_lock);
	CDBG("%s:%d end\n", __func__, __LINE__);
	return 0;
}

int msm_flash_led_off(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);
	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->off_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = false;

/*
	if( is_ZD550KL() )
	{
		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_1_EN],
			GPIO_OUT_LOW);

		gpio_set_value_cansleep(
			power_info->gpio_conf->gpio_num_info->
			gpio_num[SENSOR_GPIO_FL_EN],
			GPIO_OUT_LOW);

	}


	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_LOW);
*/
	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_led_low(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called led_direction %d\n", __func__, __LINE__, led_direction);
	mutex_lock(&flash_lock);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		/**/
		CDBG("%s  led_direction %d\n", __func__, led_direction);
		if( led_direction == FRONT_LED )
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
//<asus-leong_un20150401<<<<<<<<<+

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = true;

	mutex_unlock(&flash_lock);
	return rc;
}

/*For ASUS FLASH+++*/
int msm_flash_led_low_first(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{

			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
//<asus-leong_un20150401<<<<<<<<<+

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_first_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = true;	

	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_led_low_second(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);

	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		mutex_unlock(&flash_lock);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
//<asus-leong_un20150401<<<<<<<<<+

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);


	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->low_second_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = true;

	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_led_low_current_set(struct msm_led_flash_ctrl_t *fctrl, int intensity1, int intensity2)
{
	int rc = 0,i = 0;
	int val[2];
	int torch_map_offset = 25;

	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	mutex_lock(&flash_lock);
	CDBG("%s:%d called led_direction %d\n", __func__, __LINE__, led_direction);
	val[0] = intensity1;
	val[1] = intensity2;

	for (i=0;i<2;i++) {
		val[i] = (val[i] / torch_map_offset)-1;
		if (val[i] < 0)
			val[i] = 0;
		if(val[i] > SKY81296_TORCH_CURRENT_250MA)
			val[i] = SKY81296_TORCH_CURRENT_250MA;
		else if (val[i] < SKY81296_TORCH_CURRENT_25MA)
			val[i] = SKY81296_TORCH_CURRENT_25MA;
	}
	//printk(KERN_INFO "[AsusFlash] Real set torch current  to %d\n", val);
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_LOW);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_LOW);
		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);

    if( is_ZD550KL() )
	{
		if (fctrl->flash_i2c_client && fctrl->reg_setting) {
			pr_err("%s:%d flash_i2c_client&& fctrl->reg_setting = true  =0x\n", __func__, __LINE__);
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
				fctrl->flash_i2c_client,
				fctrl->reg_setting->init_setting);
			if (rc < 0)
				pr_err("%s:%d failed\n", __func__, __LINE__);
		}
	}

	printk("[AsusFlash] Set Dual Torch current %u \n", ( val[1] << 4 | val[0] ));
	if (fctrl->flash_i2c_client) {
		//CDBG("%s:%d flash_i2c_client = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl->flash_i2c_client,
			SKY81296_MOVIE_MODE_CURRENT,
			( val[1] << 4 | val[0] ),
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
/*
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl->flash_i2c_client,
			SKY81296_CONTROL1,
			0x11,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
*/
	}
	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_led_high_current_set(struct msm_led_flash_ctrl_t *fctrl, int intensity1, int intensity2)
{
	int rc = 0,i = 0;
	int val[2];
	int flash_current_map_offset = 50, flash_current_base = 250, flash_current_default = 1000;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;

	mutex_lock(&flash_lock);
	CDBG("%s:%d called\n", __func__, __LINE__);
	val[0] = intensity1;
	val[1] = intensity2;
	for (i=0;i<2;i++) {
		if (val[i] < 1001)
			val[i] = ((val[i] - flash_current_base) / flash_current_map_offset);
		else
			val[i] = SKY81296_FLASH_CURRENT_1000MA + (val[i] - flash_current_default) / 100;
		if (val[i] < 0)
			val[i] = 0;
		if(val[i] > SKY81296_FLASH_CURRENT_1000MA)
			val[i] = SKY81296_FLASH_CURRENT_1000MA;
		else if (val[i] < SKY81296_FLASH_CURRENT_250MA)
			val[i] = SKY81296_FLASH_CURRENT_250MA;
	}
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_LOW);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_LOW);
		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);

    if( is_ZD550KL() )
	{
		if (fctrl->flash_i2c_client && fctrl->reg_setting) {
			pr_err("%s:%d flash_i2c_client&& fctrl->reg_setting = true  =0x\n", __func__, __LINE__);
			rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
				fctrl->flash_i2c_client,
				fctrl->reg_setting->init_setting);
			if (rc < 0)
				pr_err("%s:%d failed\n", __func__, __LINE__);
		}
	}

	printk("[AsusFlash] Set Dual Flash current %u %u \n", val[0],val[1] );
	if (fctrl->flash_i2c_client) {
		//CDBG("%s:%d flash_i2c_client = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl->flash_i2c_client,
			SKY81296_FLASH1_CURRENT,
			val[0],
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl->flash_i2c_client,
			SKY81296_FLASH2_CURRENT,
			val[1],
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_led_high_timer_set(struct msm_led_flash_ctrl_t *fctrl, int timer1, int timer2)
{
	int rc = 0,i = 0;
	int val[2];
	int flash_timeout_map_offset = 95;
	//struct msm_camera_sensor_board_info *flashdata = NULL;
	//struct msm_camera_power_ctrl_t *power_info = NULL;

	CDBG("%s:%d called\n", __func__, __LINE__);
	val[0] = timer1;
	val[1] = timer2;
	for (i=0;i<2;i++) {
		val[i] = (val[i] / flash_timeout_map_offset);
		if (val[i] < 0)
			val[i] = 0;
		if(val[i] > SKY81296_FLASHTIMEOUT_1045MS)
			val[i] = SKY81296_FLASHTIMEOUT_1045MS;
	}
	if (!fctrl) {
		pr_err("%s:%d fctrl NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	/*
	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	if( is_ZD550KL() )
	{
		if( led_direction == FRONT_LED )
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		GPIO_OUT_HIGH);
	*/
	printk("[AsusFlash] Set Dual Flash time1 %u time2 %u \n", val[0],val[1] );
	if (fctrl->flash_i2c_client) {
		CDBG("%s:%d flash_i2c_client = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write(
			fctrl->flash_i2c_client,
			SKY81296_FLASH_TIMER,
			( val[1] << 4 | val[0] ),
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}
	return rc;
}

/*For ASUS FLASH---*/

int msm_flash_led_high(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		/**/
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{

			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
//<asus-leong_un20150401<<<<<<<<<+

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = true;

	mutex_unlock(&flash_lock);
	return rc;
}

/*For ASUS FLASH+++*/
int msm_flash_led_high_first(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{

			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

//<asus-leong_un20150401<<<<<<<<<+

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_first_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = true;

	mutex_unlock(&flash_lock);
	return rc;
}

int msm_flash_led_high_second(struct msm_led_flash_ctrl_t *fctrl)
{
	int rc = 0;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	CDBG("%s:%d called\n", __func__, __LINE__);
	mutex_lock(&flash_lock);

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

//<asus-leong_un20150401>>>>>>>>>+
	if( is_ZD550KL() )
	{
		pr_err("%s:%d called led_direction %d \n", __func__, __LINE__, led_direction);
		if( led_direction == FRONT_LED )
		{

			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_1_EN],
				GPIO_OUT_HIGH);
		}
		else
		{
			gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);

		}
	}
	else
		gpio_set_value_cansleep(
				power_info->gpio_conf->gpio_num_info->
				gpio_num[SENSOR_GPIO_FL_EN],
				GPIO_OUT_HIGH);
//<asus-leong_un20150401<<<<<<<<<+

	gpio_set_value_cansleep(
		power_info->gpio_conf->gpio_num_info->
		gpio_num[SENSOR_GPIO_FL_NOW],
		//GPIO_OUT_HIGH);
		GPIO_OUT_LOW);

	if (fctrl->flash_i2c_client && fctrl->reg_setting) {
		//CDBG("%s:%d flash_i2c_client&& fctrl->reg_setting = true\n", __func__, __LINE__);
		rc = fctrl->flash_i2c_client->i2c_func_tbl->i2c_write_table(
			fctrl->flash_i2c_client,
			fctrl->reg_setting->high_second_setting);
		if (rc < 0)
			pr_err("%s:%d failed\n", __func__, __LINE__);
	}

	asus_flash_status = true;

	mutex_unlock(&flash_lock);
	return rc;
}
/*For ASUS FLASH---*/

#define	FLASH_BRIGHTNESS_PROC_FILE	"driver/asus_flash_brightness"
#define	STATUS_PROC_FILE	"driver/flash_status"
#define	DUMP_PROC_FILE	"driver/flash_dump_reg"
static struct proc_dir_entry *flash_brightness_proc_file;
static struct proc_dir_entry *status_proc_file;
static struct proc_dir_entry *dump_proc_file;
static int last_flash_brightness_value;
static int ATD_status;
//static bool asus_flash_status;

static int flash_brightness_proc_read(struct seq_file *buf, void *v)
{
    seq_printf(buf, "%d\n", last_flash_brightness_value);
    return 0;
}

static int flash_brightness_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, flash_brightness_proc_read, NULL);
}

static ssize_t flash_brightness_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int set_val = -1,now_flash_brightness_value = -1,rc;
	int MAX_FLASHLIGHT_CURRENT = 100;
	char messages[8];
        int i =0;
        for(i=0;i<8;i++) messages[i]=0;

	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}
/*
	if( is_ZD550KL() )
	{
		if( led_direction != FRONT_LED )
			led_direction = REAR_LED;
	}
*/
	now_flash_brightness_value = (int)simple_strtol(messages, NULL, 10);
	if(now_flash_brightness_value > 99) now_flash_brightness_value = 99;
	set_val = now_flash_brightness_value * MAX_FLASHLIGHT_CURRENT / 99;
	mutex_lock(&flashlight_lock);
	pr_info("[AsusFlashBrightness]flash brightness value=%d now_flash_brightness_value=%d\n", set_val,now_flash_brightness_value);
	if (g_fctrl->led_state == MSM_CAMERA_LED_INIT) {
		printk(KERN_INFO "[AsusFlashBrightness] CameraFlash already init, so donothing\n");
		last_flash_brightness_value = now_flash_brightness_value;
		asus_flash_status = false;
		mutex_unlock(&flashlight_lock);
		return len;
	}
	if (last_flash_brightness_value == 0&&(now_flash_brightness_value>0&&now_flash_brightness_value<=99)) {
		rc = msm_flash_led_init(g_fctrl);
		asus_flash_status = true;
		g_fctrl->flashlight_state = MSM_CAMERA_LED_INIT;
		if (rc  < 0) {
			printk(KERN_INFO "[AsusFlashBrightness] msm_flash_led_init fail\n");
			msm_flash_led_release(g_fctrl);
			g_fctrl->flashlight_state = MSM_CAMERA_LED_RELEASE;
			last_flash_brightness_value = now_flash_brightness_value;
			mutex_unlock(&flashlight_lock);
			return rc;
		}
	} else if (last_flash_brightness_value == now_flash_brightness_value||(now_flash_brightness_value<0||now_flash_brightness_value>99) || (g_fctrl->flashlight_state == MSM_CAMERA_LED_RELEASE)) {
		printk(KERN_INFO "[AsusFlashBrightness] now_flash_brightness_value = last_flash_brightness_value or now_flash_brightness_value out range or flashlight_state invalid so donothing\n");
		last_flash_brightness_value = now_flash_brightness_value;
		asus_flash_status = true;
		mutex_unlock(&flashlight_lock);
		return len;
	}

	last_flash_brightness_value = now_flash_brightness_value;

	if (set_val > MAX_FLASHLIGHT_CURRENT) {
		msm_flash_led_low_current_set(g_fctrl, MAX_FLASHLIGHT_CURRENT, MAX_FLASHLIGHT_CURRENT);
		msm_flash_led_low_first(g_fctrl);
		asus_flash_status = true;
		//map_num = SKY81296_TORCH_CURRENT_200MA;
	} else if (set_val <= 0) {
		if (g_fctrl->flashlight_state == MSM_CAMERA_LED_RELEASE) {
			printk(KERN_INFO "[AsusFlashBrightness] flashlight already release, so don't need release again\n");
			asus_flash_status = false;
			mutex_unlock(&flashlight_lock);
			return len;
		}
		msm_flash_led_off(g_fctrl);
		msm_flash_led_release(g_fctrl);
		g_fctrl->flashlight_state = MSM_CAMERA_LED_RELEASE;
	} else if (0 < set_val && set_val < (MAX_FLASHLIGHT_CURRENT + 1)) {
		printk(KERN_INFO "[AsusFlashBrightness] current now in 1~%d", MAX_FLASHLIGHT_CURRENT);
		msm_flash_led_low_current_set(g_fctrl, set_val,set_val);
		//msm_flash_led_low(g_fctrl);
		asus_flash_status = true;
		msm_flash_led_low_first(g_fctrl);
	} else {
		if (g_fctrl->flashlight_state == MSM_CAMERA_LED_RELEASE) {
			printk(KERN_INFO "[AsusFlashBrightness] flashlight already release, so don't need release again\n");
			mutex_unlock(&flashlight_lock);
			return len;
		}
		msm_flash_led_off(g_fctrl);
		asus_flash_status = false;
		msm_flash_led_release(g_fctrl);
		last_flash_brightness_value = 0;
		g_fctrl->flashlight_state = MSM_CAMERA_LED_RELEASE;
		mutex_unlock(&flashlight_lock);
		return -1;
	}
	//printk(KERN_INFO "[AsusFlashBrightness] Real set torch current  to %d\n", set_val);
	mutex_unlock(&flashlight_lock);
	return len;
}

static const struct file_operations flash_brightness_fops = {
	.owner = THIS_MODULE,
	.open = flash_brightness_proc_open,
	.read = seq_read,
	.write = flash_brightness_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_proc_read(struct seq_file *buf, void *v)
{
    uint16_t i,value=0;
    int rc=0;
    uint16_t flash_i2c_interface[12]={0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B};
    if (g_fctrl->led_state != MSM_CAMERA_LED_INIT&&g_fctrl->flashlight_state != MSM_CAMERA_LED_INIT) {
	    seq_printf(buf, "Please open Camera or Flashlight then try again\n");
	    return 0;
    }
    for (i=0;i<sizeof(flash_i2c_interface)/sizeof(uint16_t);i++) {
	    rc = g_fctrl->flash_i2c_client->i2c_func_tbl->i2c_read(
		g_fctrl->flash_i2c_client,	flash_i2c_interface[i], &value,
		MSM_CAMERA_I2C_BYTE_DATA);
	    if (rc < 0)
		pr_err("%s:%d failed\n", __func__, __LINE__);

	    seq_printf(buf, "0x%x=0x%x\n",flash_i2c_interface[i],value);
    }
    return 0;
}

static int dump_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, dump_proc_read, NULL);
}


static const struct file_operations dump_fops = {
	.owner = THIS_MODULE,
	.open = dump_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int status_proc_read(struct seq_file *buf, void *v)
{
//    seq_printf(buf, "%d\n", ATD_status);
//	ATD_status = 0;
    seq_printf(buf, "%d\n", asus_flash_status);
    return 0;
}

static int status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, status_proc_read, NULL);
}


static const struct file_operations status_fops = {
	.owner = THIS_MODULE,
	.open = status_proc_open,
	.read = seq_read,
	//.write = status_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_proc_file(void)
{
    ATD_status = 0;
    status_proc_file = proc_create(STATUS_PROC_FILE, 0666, NULL, &status_fops);
    if (status_proc_file) {
	printk("%s sucessed!\n", __func__);
    } else {
	printk("%s failed!\n", __func__);
    }
    asus_flash_status = false;
    last_flash_brightness_value = 0;
    flash_brightness_proc_file = proc_create(FLASH_BRIGHTNESS_PROC_FILE, 0666, NULL, &flash_brightness_fops);
    if (flash_brightness_proc_file) {
	printk("%s sucessed!\n", __func__);
    } else {
	printk("%s failed!\n", __func__);
    }
    dump_proc_file = proc_create(DUMP_PROC_FILE, 0666, NULL, &dump_fops);
    if (dump_proc_file) {
	printk("%s sucessed!\n", __func__);
    } else {
	printk("%s failed!\n", __func__);
    }
}

static ssize_t asus_flash_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t asus_flash_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	int mode = -1, set_val = -1, set_val2 = -1;
	//int map_num = -1;
	//int torch_map_offset = 25;

    led_direction = REAR_LED;
	CDBG("%s:%d called ( %s )\n", __func__, __LINE__ , ( led_direction == REAR_LED )? "REAR_LED": "FRONT_LED" );
	sscanf(buf, "%d %d %d", &mode, &set_val, &set_val2);
	pr_info("[AsusFlash]flash mode=%d value=%d value2=%d\n", mode, set_val, set_val2);
	if(asus_flash_status == false)
		msm_flash_led_init(g_fctrl);

	pr_info("[AsusFlash]init finished\n");
	if(mode == 0) {
		if (set_val < 0 || set_val > 250 || set_val == 1) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_low_first(g_fctrl);
		} else if (set_val == 0 ) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if(0 < set_val && set_val < 251) {
			printk(KERN_INFO "[AsusFlash] current now in 1~250");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		//printk(KERN_INFO "[AsusFlash] Real set torch current  to %d\n", map_num);
	} else if(mode == 1) {
		if (set_val == 1 || set_val < 0 || set_val > 1500) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_high_first(g_fctrl);
		} else if (set_val == 0) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if (0 < set_val && set_val < 1501) {
			printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		//printk(KERN_INFO "[AsusFlash] Real set flash current to %d\n", map_num);
		/*if (set_val2 == -1) {
			set_val2 = 1;
		}
		if (set_val2 < 0 || set_val2 > 1425 || set_val2 == 1) {
			msm_flash_lm3642_led_high(&fctrl);
		} else if (set_val2 == 0) {
			msm_flash_lm3642_led_off(&fctrl);
		} else if (0 < set_val2 && set_val2 < 1426) {
			printk(KERN_INFO "[AsusFlash] Flash time out now in 1~1425");
		} else {
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash time out to %d\n", map_num);*/
	} else if(mode==2) {
		if (set_val == 0) {
			if (set_val2 < 0 || set_val2 > 250 || set_val2 == 1) {
				//msm_flash_led_off(fctrl);
				asus_flash_status = true;
				msm_flash_led_low_current_set(g_fctrl, SKY81296_TORCH_CURRENT_125MA, SKY81296_TORCH_CURRENT_125MA);
				msm_flash_led_low(g_fctrl);
			} else if (set_val2 == 0 ) {
			       asus_flash_status = false;
				msm_flash_led_off(g_fctrl);
				msm_flash_led_release(g_fctrl);
			} else if(0 < set_val2 && set_val2 < 251) {
				printk(KERN_INFO "[AsusFlash] current now in 1~250");
				//msm_flash_led_off(fctrl);
				asus_flash_status = true;
				msm_flash_led_low_current_set(g_fctrl,set_val2, set_val2);
				msm_flash_led_low(g_fctrl);
			} else {
			       asus_flash_status = false;
				msm_flash_led_off(g_fctrl);
				msm_flash_led_release(g_fctrl);
				return -1;
			}
		}
	} else {
		return -1;
	}

	return count;
}

static const struct file_operations asus_flash_proc_fops = {
	.read = asus_flash_show,
	.write = asus_flash_store,
};

/*#define	ASUS_FLASH_PROC_FILE	"driver/asus_flash"
static struct proc_dir_entry *asus_flash_proc_file;
static int asus_flash_value;*/

static ssize_t asus_flash_second_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t asus_flash_second_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	int mode = -1, set_val = -1;
	int map_num = -1;


    led_direction = REAR_LED;
	CDBG("%s:%d called ( %s )\n", __func__, __LINE__ , ( led_direction == REAR_LED )? "REAR_LED": "FRONT_LED" );
	sscanf(buf, "%d %d", &mode, &set_val);
	pr_info("[AsusFlash]flash mode=%d value=%d\n", mode, set_val);
	if(asus_flash_status == false)
		msm_flash_led_init(g_fctrl);

	pr_info("[AsusFlash]init finished\n");
	if(mode == 0) {
		if (set_val < 0 || set_val > 250 || set_val == 1) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_low_second(g_fctrl);
		} else if (set_val == 0 ) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if(0 < set_val && set_val < 251) {
			printk(KERN_INFO "[AsusFlash] current now in 1~250");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set torch current  to %d\n", map_num);
	} else if(mode == 1) {
		if (set_val == 1 || set_val < 0 || set_val > 1500) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_high_second(g_fctrl);
		} else if (set_val == 0) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if (0 < set_val && set_val < 1501) {
			printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash current to %d\n", map_num);
		/*if (set_val2 == -1) {
			set_val2 = 1;
		}
		if (set_val2 < 0 || set_val2 > 1425 || set_val2 == 1) {
			msm_flash_lm3642_led_high(&fctrl);
		} else if (set_val2 == 0) {
			msm_flash_lm3642_led_off(&fctrl);
		} else if (0 < set_val2 && set_val2 < 1426) {
			printk(KERN_INFO "[AsusFlash] Flash time out now in 1~1425");
		} else {
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash time out to %d\n", map_num);*/
	} else {
		return -1;
	}

	return count;
}

static const struct file_operations asus_flash_second_proc_fops = {
	.read = asus_flash_second_show,
	.write = asus_flash_second_store,
};



static ssize_t asus_flash_third_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t asus_flash_third_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	int mode = -1, set_val = -1, set_val2 = -1;
	//int map_num = -1;
	//int torch_map_offset = 25;

    led_direction = FRONT_LED;
	CDBG("%s:%d called ( %s )\n", __func__, __LINE__ , ( led_direction == REAR_LED )? "REAR_LED": "FRONT_LED" );
	sscanf(buf, "%d %d %d", &mode, &set_val, &set_val2);
	pr_info("[AsusFlash]flash mode=%d value=%d value2=%d\n", mode, set_val, set_val2);
	if(asus_flash_status == false)
		msm_flash_led_init(g_fctrl);

	pr_info("[AsusFlash]init finished\n");
	if(mode == 0) {
		if (set_val < 0 || set_val > 250 || set_val == 1) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_low_first(g_fctrl);
		} else if (set_val == 0 ) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if(0 < set_val && set_val < 251) {
			printk(KERN_INFO "[AsusFlash] current now in 1~250");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		//printk(KERN_INFO "[AsusFlash] Real set torch current  to %d\n", map_num);
	} else if(mode == 1) {
		if (set_val == 1 || set_val < 0 || set_val > 1500) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_high_first(g_fctrl);
		} else if (set_val == 0) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if (0 < set_val && set_val < 1501) {
			printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		//printk(KERN_INFO "[AsusFlash] Real set flash current to %d\n", map_num);
		/*if (set_val2 == -1) {
			set_val2 = 1;
		}
		if (set_val2 < 0 || set_val2 > 1425 || set_val2 == 1) {
			msm_flash_lm3642_led_high(&g_fctrl);
		} else if (set_val2 == 0) {
			msm_flash_lm3642_led_off(&g_fctrl);
		} else if (0 < set_val2 && set_val2 < 1426) {
			printk(KERN_INFO "[AsusFlash] Flash time out now in 1~1425");
		} else {
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash time out to %d\n", map_num);*/
	} else if(mode==2) {
		if (set_val == 0) {
			if (set_val2 < 0 || set_val2 > 250 || set_val2 == 1) {
				//msm_flash_led_off(g_fctrl);
				asus_flash_status = true;
				msm_flash_led_low_current_set(g_fctrl, SKY81296_TORCH_CURRENT_125MA, SKY81296_TORCH_CURRENT_125MA);
				msm_flash_led_low(g_fctrl);
			} else if (set_val2 == 0 ) {
			       asus_flash_status = false;
				msm_flash_led_off(g_fctrl);
				msm_flash_led_release(g_fctrl);
			} else if(0 < set_val2 && set_val2 < 251) {
				printk(KERN_INFO "[AsusFlash] current now in 1~250");
				//msm_flash_led_off(g_fctrl);
				asus_flash_status = true;
				msm_flash_led_low_current_set(g_fctrl,set_val2, set_val2);
				msm_flash_led_low(g_fctrl);
			} else {
			       asus_flash_status = false;
				msm_flash_led_off(g_fctrl);
				msm_flash_led_release(g_fctrl);
				return -1;
			}
		}
	} else {
		return -1;
	}

	return count;
}

static const struct file_operations asus_flash_third_proc_fops = {
	.read = asus_flash_third_show,
	.write = asus_flash_third_store,
};

/*#define	ASUS_FLASH_PROC_FILE	"driver/asus_flash"
static struct proc_dir_entry *asus_flash_proc_file;
static int asus_flash_value;*/

static ssize_t asus_flash_fourth_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t asus_flash_fourth_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
	int mode = -1, set_val = -1;
	int map_num = -1;


    led_direction = FRONT_LED;
	CDBG("%s:%d called ( %s )\n", __func__, __LINE__ , ( led_direction == REAR_LED )? "REAR_LED": "FRONT_LED" );
	sscanf(buf, "%d %d", &mode, &set_val);
	pr_info("[AsusFlash]flash mode=%d value=%d\n", mode, set_val);
	if(asus_flash_status == false)
		msm_flash_led_init(g_fctrl);

	pr_info("[AsusFlash]init finished\n");
	if(mode == 0) {
		if (set_val < 0 || set_val > 250 || set_val == 1) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_low_second(g_fctrl);
		} else if (set_val == 0 ) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if(0 < set_val && set_val < 251) {
			printk(KERN_INFO "[AsusFlash] current now in 1~250");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set torch current  to %d\n", map_num);
	} else if(mode == 1) {
		if (set_val == 1 || set_val < 0 || set_val > 1500) {
			ATD_status = 1;
			asus_flash_status = true;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_high_second(g_fctrl);
		} else if (set_val == 0) {
			ATD_status = 1;
			asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
		} else if (0 < set_val && set_val < 1501) {
			printk(KERN_INFO "[AsusFlash] Flash current now in 1~1500");
		} else {
		       asus_flash_status = false;
			msm_flash_led_off(g_fctrl);
			msm_flash_led_release(g_fctrl);
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash current to %d\n", map_num);
		/*if (set_val2 == -1) {
			set_val2 = 1;
		}
		if (set_val2 < 0 || set_val2 > 1425 || set_val2 == 1) {
			msm_flash_lm3642_led_high(&g_fctrl);
		} else if (set_val2 == 0) {
			msm_flash_lm3642_led_off(&g_fctrl);
		} else if (0 < set_val2 && set_val2 < 1426) {
			printk(KERN_INFO "[AsusFlash] Flash time out now in 1~1425");
		} else {
			return -1;
		}
		printk(KERN_INFO "[AsusFlash] Real set flash time out to %d\n", map_num);*/
	} else {
		return -1;
	}

	return count;
}

static const struct file_operations asus_flash_fourth_proc_fops = {
	.read = asus_flash_fourth_show,
	.write = asus_flash_fourth_store,
};


/*For ASUS FLASH---*/

static int32_t msm_led_get_dt_data(struct device_node *of_node,
		struct msm_led_flash_ctrl_t *fctrl)
{
	int32_t rc = 0, i = 0;
	struct msm_camera_gpio_conf *gconf = NULL;
	struct device_node *flash_src_node = NULL;
	struct msm_camera_sensor_board_info *flashdata = NULL;
	struct msm_camera_power_ctrl_t *power_info = NULL;
	uint32_t count = 0;
	uint16_t *gpio_array = NULL;
	uint16_t gpio_array_size = 0;
	uint32_t id_info[3];

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	fctrl->flashdata = kzalloc(sizeof(
		struct msm_camera_sensor_board_info),
		GFP_KERNEL);
	if (!fctrl->flashdata) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	flashdata = fctrl->flashdata;
	power_info = &flashdata->power_info;

	rc = of_property_read_u32(of_node, "cell-index", &fctrl->subdev_id);
	if (rc < 0) {
		pr_err("failed\n");
		return -EINVAL;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	rc = of_property_read_string(of_node, "label",
		&flashdata->sensor_name);
	CDBG("%s label %s, rc %d\n", __func__,
		flashdata->sensor_name, rc);
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		goto ERROR1;
	}

	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	}

	fctrl->pinctrl_info.use_pinctrl = false;
	fctrl->pinctrl_info.use_pinctrl = of_property_read_bool(of_node,
						"qcom,enable_pinctrl");
	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("failed\n");
			return -EINVAL;
		}
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"linux,default-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				pr_err("failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			CDBG("default trigger %s\n",
				 fctrl->flash_trigger_name[i]);

			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				pr_err("failed rc %d\n", rc);
				of_node_put(flash_src_node);
				continue;
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}

	} else { /*Handle LED Flash Ctrl by GPIO*/
		power_info->gpio_conf =
			 kzalloc(sizeof(struct msm_camera_gpio_conf),
				 GFP_KERNEL);
		if (!power_info->gpio_conf) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			return rc;
		}
		gconf = power_info->gpio_conf;

		gpio_array_size = of_gpio_count(of_node);
		CDBG("%s gpio count %d\n", __func__, gpio_array_size);

		if (gpio_array_size) {
			gpio_array = kzalloc(sizeof(uint16_t) * gpio_array_size,
				GFP_KERNEL);
			if (!gpio_array) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				rc = -ENOMEM;
				goto ERROR4;
			}
			for (i = 0; i < gpio_array_size; i++) {
				gpio_array[i] = of_get_gpio(of_node, i);
				CDBG("%s gpio_array[%d] = %d\n", __func__, i,
					gpio_array[i]);
			}

			rc = msm_camera_get_dt_gpio_req_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR4;
			}

			rc = msm_camera_get_dt_gpio_set_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR5;
			}

			rc = msm_camera_init_gpio_pin_tbl(of_node, gconf,
				gpio_array, gpio_array_size);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR6;
			}
		}

		/* Read the max current for an LED if present */
		if (of_get_property(of_node, "qcom,max-current", &count)) {
			count /= sizeof(uint32_t);

			if (count > MAX_LED_TRIGGERS) {
				pr_err("failed\n");
				rc = -EINVAL;
				goto ERROR8;
			}

			fctrl->flash_num_sources = count;
			fctrl->torch_num_sources = count;

			rc = of_property_read_u32_array(of_node,
				"qcom,max-current",
				fctrl->flash_max_current, count);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR8;
			}

			for (; count < MAX_LED_TRIGGERS; count++)
				fctrl->flash_max_current[count] = 0;

			for (count = 0; count < MAX_LED_TRIGGERS; count++)
				fctrl->torch_max_current[count] =
					fctrl->flash_max_current[count] >> 1;
		}

		/* Read the max duration for an LED if present */
		if (of_get_property(of_node, "qcom,max-duration", &count)) {
			count /= sizeof(uint32_t);

			if (count > MAX_LED_TRIGGERS) {
				pr_err("failed\n");
				rc = -EINVAL;
				goto ERROR8;
			}

			rc = of_property_read_u32_array(of_node,
				"qcom,max-duration",
				fctrl->flash_max_duration, count);
			if (rc < 0) {
				pr_err("%s failed %d\n", __func__, __LINE__);
				goto ERROR8;
			}

			for (; count < MAX_LED_TRIGGERS; count++)
				fctrl->flash_max_duration[count] = 0;
		}

		flashdata->slave_info =
			kzalloc(sizeof(struct msm_camera_slave_info),
				GFP_KERNEL);
		if (!flashdata->slave_info) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			rc = -ENOMEM;
			goto ERROR8;
		}

		rc = of_property_read_u32_array(of_node, "qcom,slave-id",
			id_info, 3);
		if (rc < 0) {
			pr_err("%s failed %d\n", __func__, __LINE__);
			goto ERROR9;
		}
		fctrl->flashdata->slave_info->sensor_slave_addr = id_info[0];
		fctrl->flashdata->slave_info->sensor_id_reg_addr = id_info[1];
		fctrl->flashdata->slave_info->sensor_id = id_info[2];

		kfree(gpio_array);
		return rc;
ERROR9:
		kfree(fctrl->flashdata->slave_info);
ERROR8:
		kfree(fctrl->flashdata->power_info.gpio_conf->gpio_num_info);
ERROR6:
		kfree(gconf->cam_gpio_set_tbl);
ERROR5:
		kfree(gconf->cam_gpio_req_tbl);
ERROR4:
		kfree(gconf);
ERROR1:
		kfree(fctrl->flashdata);
		kfree(gpio_array);
	}
	return rc;
}

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_write_conf_tbl = msm_camera_cci_i2c_write_conf_tbl,
};

#ifdef CONFIG_DEBUG_FS
static int set_led_status(void *data, u64 val)
{
	struct msm_led_flash_ctrl_t *fctrl =
		 (struct msm_led_flash_ctrl_t *)data;
	int rc = -1;
	pr_debug("set_led_status: Enter val: %llu", val);
	if (!fctrl) {
		pr_err("set_led_status: fctrl is NULL");
		return rc;
	}
	if (!fctrl->func_tbl) {
		pr_err("set_led_status: fctrl->func_tbl is NULL");
		return rc;
	}
	if (val == 0) {
		pr_debug("set_led_status: val is disable");
		rc = msm_flash_led_off(fctrl);
	} else {
		pr_debug("set_led_status: val is enable");
		rc = msm_flash_led_low(fctrl);
	}

	return rc;
}

DEFINE_SIMPLE_ATTRIBUTE(ledflashdbg_fops,
	NULL, set_led_status, "%llu\n");
#endif

int msm_flash_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	/*For ASUS FLASH+++*/
	struct proc_dir_entry* proc_entry_flash;
	struct proc_dir_entry* proc_entry_flash_second;
	struct proc_dir_entry* proc_entry_flash_third;
	struct proc_dir_entry* proc_entry_flash_fourth;
	void* dummy = NULL;
	void* dummy2 = NULL;
	void* dummy3 = NULL;
	void* dummy4 = NULL;
	/*For ASUS FLASH---*/
	int rc = 0;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dentry;
#endif
	pr_info("%s entry\n", __func__);
	ATD_status = 0;
	create_proc_file();
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		rc = -35;
		goto probe_failure;
	}

	g_fctrl = (struct msm_led_flash_ctrl_t *)(id->driver_data);
	if (g_fctrl->flash_i2c_client)
		g_fctrl->flash_i2c_client->client = client;
	/* Set device type as I2C */
	g_fctrl->flash_device_type = MSM_CAMERA_I2C_DEVICE;

	/* Assign name for sub device */
	snprintf(g_fctrl->msm_sd.sd.name, sizeof(g_fctrl->msm_sd.sd.name),
		"%s", id->name);

	rc = msm_led_get_dt_data(client->dev.of_node, g_fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d\n", __func__, __LINE__);
		return rc;
	}

	if (g_fctrl->pinctrl_info.use_pinctrl == true)
		msm_flash_pinctrl_init(g_fctrl);

	if (g_fctrl->flash_i2c_client != NULL) {
		g_fctrl->flash_i2c_client->client = client;
		if (g_fctrl->flashdata->slave_info->sensor_slave_addr)
			g_fctrl->flash_i2c_client->client->addr =
				g_fctrl->flashdata->slave_info->
				sensor_slave_addr;
	} else {
		pr_err("%s %s sensor_i2c_client NULL\n",
			__func__, client->name);
		rc = -EFAULT;
		return rc;
	}

	if (!g_fctrl->flash_i2c_client->i2c_func_tbl)
		g_fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_qup_func_tbl;

	rc = msm_led_i2c_flash_create_v4lsubdev(g_fctrl);
	if (rc < 0) {
		pr_err("%s msm_led_i2c_flash_create_v4lsubdev failed line %d\n", __func__, __LINE__);
		return rc;
	}
#ifdef CONFIG_DEBUG_FS
	dentry = debugfs_create_file("ledflash", S_IRUGO, NULL, (void *)g_fctrl,
		&ledflashdbg_fops);
	if (!dentry)
		pr_err("Failed to create the debugfs ledflash file");
#endif
	/*For ASUS FLASH+++*/
	proc_entry_flash = proc_create_data("driver/asus_flash", 0666, NULL, &asus_flash_proc_fops, dummy);
	proc_set_user(proc_entry_flash, 1000, 1000);
	proc_entry_flash_second = proc_create_data("driver/asus_flash2", 0666, NULL, &asus_flash_second_proc_fops, dummy2);
	proc_set_user(proc_entry_flash_second, 1000, 1000);
	if( is_ZD550KL() ){
		proc_entry_flash_third = proc_create_data("driver/asus_flash3", 0666, NULL, &asus_flash_third_proc_fops, dummy3);
		proc_set_user(proc_entry_flash_third, 1000, 1000);
		proc_entry_flash_fourth = proc_create_data("driver/asus_flash4", 0666, NULL, &asus_flash_fourth_proc_fops, dummy4);
		proc_set_user(proc_entry_flash_fourth, 1000, 1000);
	}
	/*For ASUS FLASH---*/
	pr_info("%s:%d probe success\n", __func__, __LINE__);
	return 0;

probe_failure:
	pr_info("%s:%d probe failed\n", __func__, __LINE__);
	return rc;
}

int msm_flash_probe(struct platform_device *pdev,
	const void *data)
{
	/*For ASUS FLASH+++*/
	struct proc_dir_entry* proc_entry_flash;
	struct proc_dir_entry* proc_entry_flash_second;
	struct proc_dir_entry* proc_entry_flash_third;
	struct proc_dir_entry* proc_entry_flash_fourth;
	void* dummy = NULL;
	void* dummy2 = NULL;
	void* dummy3 = NULL;
	void* dummy4 = NULL;
	/*For ASUS FLASH---*/
	int rc = 0;
	/*struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;*/
	struct device_node *of_node = pdev->dev.of_node;
	struct msm_camera_cci_client *cci_client = NULL;

	pr_info("%s entry\n", __func__);
	create_proc_file();

#if ENABLE_FLASH_SELECT_PROC
	if( is_ZD550KL() ){
		create_flash_select_proc_file();
	}
#endif

	if (!of_node) {
		pr_err("of_node NULL\n");
		goto probe_failure;
	}
	g_fctrl = (struct msm_led_flash_ctrl_t *)data;
	g_fctrl->pdev = pdev;

	rc = msm_led_get_dt_data(pdev->dev.of_node, g_fctrl);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}

	if (g_fctrl->pinctrl_info.use_pinctrl == true)
		msm_flash_pinctrl_init(g_fctrl);

	/* Assign name for sub device */
	snprintf(g_fctrl->msm_sd.sd.name, sizeof(g_fctrl->msm_sd.sd.name),
			"%s", g_fctrl->flashdata->sensor_name);
	/* Set device type as Platform*/
	g_fctrl->flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;

	if (NULL == g_fctrl->flash_i2c_client) {
		pr_err("%s flash_i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}

	g_fctrl->flash_i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!g_fctrl->flash_i2c_client->cci_client) {
		pr_err("%s failed line %d kzalloc failed\n",
			__func__, __LINE__);
		return rc;
	}

	cci_client = g_fctrl->flash_i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = g_fctrl->cci_i2c_master;
	if (g_fctrl->flashdata->slave_info->sensor_slave_addr)
		cci_client->sid =
			g_fctrl->flashdata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;

	cci_client->i2c_freq_mode = I2C_FAST_MODE; //sean ++

	if (!g_fctrl->flash_i2c_client->i2c_func_tbl)
		g_fctrl->flash_i2c_client->i2c_func_tbl =
			&msm_sensor_cci_func_tbl;

	rc = msm_led_flash_create_v4lsubdev(pdev, g_fctrl);

	/*For ASUS FLASH+++*/
	proc_entry_flash = proc_create_data("driver/asus_flash", 0666, NULL, &asus_flash_proc_fops, dummy);
	proc_set_user(proc_entry_flash, 1000, 1000);
	proc_entry_flash_second = proc_create_data("driver/asus_flash2", 0666, NULL, &asus_flash_second_proc_fops, dummy2);
	proc_set_user(proc_entry_flash_second, 1000, 1000);
	if( is_ZD550KL() ){
		proc_entry_flash_third = proc_create_data("driver/asus_flash3", 0666, NULL, &asus_flash_third_proc_fops, dummy3);
		proc_set_user(proc_entry_flash_third, 1000, 1000);
		proc_entry_flash_fourth = proc_create_data("driver/asus_flash4", 0666, NULL, &asus_flash_fourth_proc_fops, dummy4);
		proc_set_user(proc_entry_flash_fourth, 1000, 1000);
	}
	/*For ASUS FLASH---*/
	mutex_init(&flash_lock);
	mutex_init(&flashlight_lock);
	CDBG("%s: probe success\n", __func__);
	return 0;

probe_failure:
	CDBG("%s probe failed\n", __func__);
	return rc;
}
