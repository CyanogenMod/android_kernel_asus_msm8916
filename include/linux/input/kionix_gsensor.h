/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __KXTJ9_H__
#define __KXTJ9_H__

#define KXTJ9_I2C_ADDR		0x0E
#define KX022_I2C_ADDR		0x1F

#define ASUS_GSENSOR_NAME			"kx022"

#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define STATUS_REG		0x15
#define INT_REL			0x17
/* Kxtj02 reg
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define CTRL_REG2		0x1D
#define INT_CTRL1		0x1E
#define INT_CTRL2		0x1F
#define DATA_CTRL		0x21
*/
/*  Kx022 reg */
#define CTRL_REG1		0x18
#define CTRL_REG2		0x19
#define INT_CTRL1		0x1C
#define INT_CTRL2		0x1D
#define INT_CTRL3		0x1E
#define INT_CTRL4		0x1F
#define DATA_CTRL		0x1B
// for enable motion detect , added by cheng_kao 2014.02.12 ++
#define WAKEUP_TIMER	0x29
#define WAKEUP_THRES	0x6A

#define MOTION_DETECT_PERCENT	10
#define MOTION_DETECT_COUNT	25

#define WUFE_ON			(1 << 1)
#define WUFE_OFF		0xFD
#define DRDYE_OFF		0xDF

#define INT_MODE_DRDY	0
#define INT_MODE_WUFE	1

#define WUFE12_5F		4
#define WUFE25F			5
#define WUFE50F			6
#define WUFE100F		7
// for enable motion detect , added by cheng_kao 2014.02.12 --


/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
#define RES_12bit			(1 << 6)
#define RES_16bit			(1 << 6)
#define RES_8bit			0xBF

#define GRP4_G_4G		(1 << 3)

/* DATA CONTROL REGISTER BITS */
#define ODR0_781F		8
#define ODR12_5F			0
#define ODR25F			1
#define ODR50F			2
#define ODR100F			3
#define ODR200F			4
#define ODR400F			5
#define ODR800F			6
#define ODR1600F			7
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL			(1 << 3)
#define KXTJ9_IEA			(1 << 4)
#define KXTJ9_IEN			(1 << 5)
#define KX022_IEA			(1 << 4)
#define KX022_IEN			(1 << 5)
#define KX022_DRDYI1		(1 << 4)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

//use to define raw data by the chip location
#define KXTJ9_CHIP_LOCATION_EVB_ME372CL		0
#define KXTJ9_CHIP_LOCATION_SR_ME372CG		1	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ9_CHIP_LOCATION_SR_ME372CL		2	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_ME175CG		3	/*	X(0 , 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_PF400CG		4	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_A400CG		5	/*	X(-1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_ER_ME372CL		6	/*	X(0 , 0 , 0 );	Y(0 , -1 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_FE380CG		7	/*	X(0 , 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_A450CG		8	/*	X(0 , 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , -1) */
#define KXTJ2_CHIP_LOCATION_SR_PF450CL		9	/*	X(1 , 0 , 0 );	Y(0 , 1 , 0);	Z(0 , 0 , 1) */
#define KXTJ2_CHIP_LOCATION_SR_ZC500CL		10	/*	X(0, -1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , 1) */
#define KX022_CHIP_LOCATION_SR_ZC500KL		11	/*	X(0, 1 , 0 );	Y(1 , 0 , 0);	Z(0 , 0 , 1) */
int g_ilocation=0;

#define WHOAMI_VALUE_FOR_KXTJ9	0x8
#define WHOAMI_VALUE_FOR_KXTJ2	0x9
#define WHOAMI_VALUE_FOR_KX022	0x14

#define RAW_LIMIT_PERCENT			35

#define KX022_VALUE_FOR_NOT_NEED_RESET	0
#define KX022_RESET_FOR_SAME_RAWDATA	1
#define KX022_RESET_FOR_ZERO_RAWDATA	2

#define KXTJ9_RES_8BIT	0
#define KXTJ9_RES_12BIT	1

#define KX022_RES_16BIT	1

//define for Camera added by cheng_Kao 2013.10.31 ++
int g_kxtj_for_camera_x =0;
int g_kxtj_for_camera_y =0;
int g_kxtj_for_camera_z =0;

struct kxtj9_platform_data {
	unsigned int min_interval;	/* minimum poll interval (in milli-seconds) */
	unsigned int init_interval;	/* initial poll interval (in milli-seconds) */

	/*
	 * By default, x is axis 0, y is axis 1, z is axis 2; these can be
	 * changed to account for sensor orientation within the host device.
	 */
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	/*
	 * Each axis can be negated to account for sensor orientation within
	 * the host device.
	 */
	bool negate_x;
	bool negate_y;
	bool negate_z;

	/* CTRL_REG1: set resolution, g-range, data ready enable */
	/* Output resolution: 8-bit valid or 12-bit valid */
	#define RES_8BIT		0
	#define RES_12BIT		(1 << 6)
	u8 res_12bit;
	/* Output g-range: +/-2g, 4g, or 8g */
	#define KXTJ9_G_2G		0
	#define KXTJ9_G_4G		(1 << 3)
	#define KXTJ9_G_8G		(1 << 4)
	u8 g_range;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	int gpio;
};
#endif  /* __KXTJ9_H__ */
