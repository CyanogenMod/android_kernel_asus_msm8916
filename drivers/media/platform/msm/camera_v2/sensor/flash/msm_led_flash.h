/* Copyright (c) 2009-2014, The Linux Foundation. All rights reserved.
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
#ifndef MSM_LED_FLASH_H
#define MSM_LED_FLASH_H

#include <linux/leds.h>
#include <linux/platform_device.h>
#include <media/v4l2-subdev.h>
#include <media/msm_cam_sensor.h>
#include <soc/qcom/camera2.h>
#include "msm_camera_i2c.h"
#include "msm_sd.h"

#define SKY81296_FLASH1_CURRENT		0x00
#define SKY81296_FLASH2_CURRENT		0x01
#define SKY81296_FLASH_TIMER		0x02
#define SKY81296_MOVIE_MODE_CURRENT	0x03
#define SKY81296_CONTROL1		0x04
#define SKY81296_CONTROL2		0x05
#define SKY81296_CONTROL3		0x06

struct msm_led_flash_ctrl_t;

struct msm_flash_fn_t {
	int32_t (*flash_get_subdev_id)(struct msm_led_flash_ctrl_t *, void *);
	int32_t (*flash_led_config)(struct msm_led_flash_ctrl_t *, void *);
	int32_t (*flash_led_init)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_release)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_off)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_low)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_low_first)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_low_second)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_high)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_high_first)(struct msm_led_flash_ctrl_t *);
	int32_t (*flash_led_high_second)(struct msm_led_flash_ctrl_t *);
};

struct msm_led_flash_reg_t {
	struct msm_camera_i2c_reg_setting *init_setting;
	struct msm_camera_i2c_reg_setting *off_setting;
	struct msm_camera_i2c_reg_setting *release_setting;
	struct msm_camera_i2c_reg_setting *low_setting;
	struct msm_camera_i2c_reg_setting *low_first_setting;
	struct msm_camera_i2c_reg_setting *low_second_setting;
	struct msm_camera_i2c_reg_setting *high_setting;
	struct msm_camera_i2c_reg_setting *high_first_setting;
	struct msm_camera_i2c_reg_setting *high_second_setting;
};

struct msm_led_flash_ctrl_t {
	struct msm_camera_i2c_client *flash_i2c_client;
	struct msm_sd_subdev msm_sd;
	struct platform_device *pdev;
	struct msm_flash_fn_t *func_tbl;
	struct msm_camera_sensor_board_info *flashdata;
	struct msm_led_flash_reg_t *reg_setting;
	/* Flash */
	const char *flash_trigger_name[MAX_LED_TRIGGERS];
	struct led_trigger *flash_trigger[MAX_LED_TRIGGERS];
	uint32_t flash_num_sources;
	uint32_t flash_op_current[MAX_LED_TRIGGERS];
	uint32_t flash_max_current[MAX_LED_TRIGGERS];
	uint32_t flash_max_duration[MAX_LED_TRIGGERS];
	/* Torch */
	const char *torch_trigger_name[MAX_LED_TRIGGERS];
	struct led_trigger *torch_trigger[MAX_LED_TRIGGERS];
	uint32_t torch_num_sources;
	uint32_t torch_op_current[MAX_LED_TRIGGERS];
	uint32_t torch_max_current[MAX_LED_TRIGGERS];

	void *data;
	enum msm_camera_device_type_t flash_device_type;
	enum cci_i2c_master_t cci_i2c_master;
	enum msm_camera_led_config_t led_state;
	enum msm_camera_led_config_t flashlight_state;
	uint32_t subdev_id;
	struct msm_pinctrl_info pinctrl_info;
};

int msm_flash_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);

int msm_flash_probe(struct platform_device *pdev, const void *data);

int32_t msm_led_flash_create_v4lsubdev(struct platform_device *pdev,
	void *data);
int32_t msm_led_i2c_flash_create_v4lsubdev(void *data);

int32_t msm_led_i2c_trigger_get_subdev_id(struct msm_led_flash_ctrl_t *fctrl,
	void *arg);

int32_t msm_led_i2c_trigger_config(struct msm_led_flash_ctrl_t *fctrl,
	void *data);

int msm_flash_led_init(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_release(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_off(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_low(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_low_first(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_low_second(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_high(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_high_first(struct msm_led_flash_ctrl_t *fctrl);
int msm_flash_led_high_second(struct msm_led_flash_ctrl_t *fctrl);

enum sky81296_flash_timeout{
	SKY81296_FLASHTIMEOUT_OFF, //0
	SKY81296_FLASHTIMEOUT_95MS,
	SKY81296_FLASHTIMEOUT_190MS,
	SKY81296_FLASHTIMEOUT_285MS,
	SKY81296_FLASHTIMEOUT_380MS,
	SKY81296_FLASHTIMEOUT_475MS,
	SKY81296_FLASHTIMEOUT_570MS,
	SKY81296_FLASHTIMEOUT_665MS,
	SKY81296_FLASHTIMEOUT_760MS,
	SKY81296_FLASHTIMEOUT_855MS,
	SKY81296_FLASHTIMEOUT_950MS,
	SKY81296_FLASHTIMEOUT_1045MS,
	SKY81296_FLASHTIMEOUT_1140MS,
	SKY81296_FLASHTIMEOUT_1235MS,
	SKY81296_FLASHTIMEOUT_1330MS,
	SKY81296_FLASHTIMEOUT_1425MS, //15
	SKY81296_FLASHTIMEOUT_NUM,
};

enum sky81296_flash_current{
	SKY81296_FLASH_CURRENT_250MA, //0
	SKY81296_FLASH_CURRENT_300MA,
	SKY81296_FLASH_CURRENT_350MA,
	SKY81296_FLASH_CURRENT_400MA,
	SKY81296_FLASH_CURRENT_450MA,
	SKY81296_FLASH_CURRENT_500MA,
	SKY81296_FLASH_CURRENT_550MA,
	SKY81296_FLASH_CURRENT_600MA,
	SKY81296_FLASH_CURRENT_650MA,
	SKY81296_FLASH_CURRENT_700MA,
	SKY81296_FLASH_CURRENT_750MA,
	SKY81296_FLASH_CURRENT_800MA,
	SKY81296_FLASH_CURRENT_850MA,
	SKY81296_FLASH_CURRENT_900MA,
	SKY81296_FLASH_CURRENT_950MA,
	SKY81296_FLASH_CURRENT_1000MA,
	SKY81296_FLASH_CURRENT_1100MA,
	SKY81296_FLASH_CURRENT_1200MA,
	SKY81296_FLASH_CURRENT_1300MA,
	SKY81296_FLASH_CURRENT_1400MA,
	SKY81296_FLASH_CURRENT_1500MA, //20
};

enum sky81296_torch_current{
	SKY81296_TORCH_CURRENT_25MA, //0
	SKY81296_TORCH_CURRENT_50MA,
	SKY81296_TORCH_CURRENT_75MA,
	SKY81296_TORCH_CURRENT_100MA,
	SKY81296_TORCH_CURRENT_125MA,
	SKY81296_TORCH_CURRENT_150MA,
	SKY81296_TORCH_CURRENT_175MA,
	SKY81296_TORCH_CURRENT_200MA,
	SKY81296_TORCH_CURRENT_225MA,
	SKY81296_TORCH_CURRENT_250MA, // 9
	SKY81296_TORCH_CURRENT_NUM,
};
#endif
