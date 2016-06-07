/*
 * This file is part of the Dyna-Image AP3426 sensor driver for MTK platform.
 * AP3426 is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3426.h
 *
 * Summary:
 *	AP3426 sensor dirver header file.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 06/03/14 John		 Original Creation (Test version:1.0)
 * 
 */

/*
 * Definitions for ap3426 als/ps sensor chip.
 */
#ifndef __AP3426_H__
#define __AP3426_H__

#include <linux/ioctl.h>
#include <linux/types.h>


#define AP3426_SUCCESS					0
#define AP3426_ERR_I2C					-1
#define AP3426_ERR_STATUS				-3
#define AP3426_ERR_SETUP_FAILURE			-4
#define AP3426_ERR_GETGSENSORDATA			-5
#define AP3426_ERR_IDENTIFICATION			-6


#define AP3426_NUM_CACHABLE_REGS	28
#define AP3426_SYS_RST_ENABLE 0x4

/* ap3426 control registers */
/*============================================================================*/
#define AP3426_REG_SYS_CONF        0x00
#define AP3426_REG_SYS_CONF_SHIFT	(0)
#define AP3426_REG_SYS_CONF_MASK	0x07

#define AP3426_REG_SYS_INT_PS_MASK	0x02
#define AP3426_REG_SYS_INT_PS_SHIFT	(1)
#define AP3426_REG_SYS_INT_AL_MASK	0x01
#define AP3426_REG_SYS_INT_AL_SHIFT	(0)
 
#define AP3426_REG_SYS_INTSTATUS   0x01
#define AP3426_REG_SYS_INT_SHIFT	(0)
#define AP3426_REG_SYS_INT_PS_SHIFT	(1)
#define AP3426_REG_SYS_INT_LS_SHIFT	(0)
#define AP3426_REG_SYS_INT_MASK		0x03
#define AP3426_REG_SYS_INT_PMASK		0x02
#define AP3426_REG_SYS_INT_AMASK		0x01

#define AP3426_OBJ_COMMAND	0x01
#define AP3426_OBJ_MASK		0x10
#define AP3426_OBJ_SHIFT	(4)

#define AP3426_REG_SYS_INTCTRL     0x02
#define AP3426_REG_SYS_WAITTIME    0x06

/* ap3426 data registers */
#define AP3426_REG_IR_DATA_LOW     0x0A
#define AP3426_REG_IR_DATA_LOW_SHIFT     (0)
#define AP3426_REG_IR_DATA_LOW_MASK 0xFF     

#define AP3426_REG_IR_DATA_HIGH    0x0B
#define AP3426_REG_IR_DATA_HIGH_SHIFT    (0)
#define AP3426_REG_IR_DATA_HIGH_MASK    0x03

#define AP3426_REG_ALS_DATA_LOW    0x0C
#define AP3426_REG_ALS_DATA_HIGH   0x0D

#define AP3426_REG_PS_DATA_LOW     0x0E
#define AP3426_REG_PS_DATA_LOW_SHIFT     (0)
#define	AL3426_REG_PS_DATA_LOW_MASK	   0xFF
#define AP3426_REG_PS_DATA_HIGH    0x0F
#define AP3426_REG_PS_DATA_HIGH_SHIFT    (0)
#define	AL3426_REG_PS_DATA_HIGH_MASK	   0x03
/*----------------------------------------------------------------------------*/
#define AP3426_REG_ALS_CONF        0x10 /*ALS GAIN*/

#define AP3426_REG_ALS_PERSIS      0x14
//#define AP3426_REG_ALS_CAL         0x19

#define AP3426_REG_ALS_THDL_L      0x1A
#define AP3426_REG_ALS_THDL_L_SHIFT	(0)
#define AP3426_REG_ALS_THDL_L_MASK	0xFF

#define AP3426_REG_ALS_THDL_H      0x1B
#define AP3426_REG_ALS_THDL_H_SHIFT	(0)
#define AP3426_REG_ALS_THDL_H_MASK	0xFF

#define AP3426_REG_ALS_THDH_L      0x1C
#define AP3426_REG_ALS_THDH_L_SHIFT	(0)
#define AP3426_REG_ALS_THDH_L_MASK	0xFF

#define AP3426_REG_ALS_THDH_H      0x1D
#define AP3426_REG_ALS_THDH_H_SHIFT	(0)
#define AP3426_REG_ALS_THDH_H_MASK	0xFF


/*----------------------------------------------------------------------------*/
/* ap3426 PS CONFIG registers */
#define AP3426_REG_PS_CONF         0x20 /*PS GAIN*/
#define AP3426_REG_PS_CONF_SHIFT         (2) 
#define AP3426_REG_PS_CONF_MASK         0x0C 

#define AP3426_REG_PS_LEDD         0x21 /*PS LED DRIVER*/
#define AP3426_REG_PS_LEDD_SHIFT         (0) 
#define AP3426_REG_PS_LEDD_MASK         0x03

#define AP3426_REG_PS_IFORM        0x22 /* PS INT Mode*/

#define AP3426_REG_PS_MEAN         0x23
#define AP3426_REG_PS_MEAN_SHIFT         (0)
#define AP3426_REG_PS_MEAN_MASK         0x03

#define AP3426_REG_PS_SMARTINT     0x24 /* PS Smart INT for low power */
#define AP3426_REG_PS_INTEGR       0x25
#define AP3426_REG_PS_INTEGR_SHIFT         (0)
#define AP3426_REG_PS_INTEGR_MASK         0x3F
#define AP3426_REG_PS_PERSIS       0x26
#define AP3426_REG_PS_PERSIS_SHIFT       (0)
#define AP3426_REG_PS_PERSIS_MASK       0x3F
#define AP3426_REG_PS_CAL_L        0x28
#define AP3426_REG_PS_CAL_L_SHIFT	(0)
#define AP3426_REG_PS_CAL_L_MASK		0xFF
#define AP3426_REG_PS_CAL_H        0x29
#define AP3426_REG_PS_CAL_H_SHIFT	(0)
#define AP3426_REG_PS_CAL_H_MASK		0x01

#define AP3426_REG_PS_THDL_L       0x2A
#define AP3426_REG_PS_THDL_L_SHIFT	(0)
#define AP3426_REG_PS_THDL_L_MASK		0xFF

#define AP3426_REG_PS_THDL_H       0x2B
#define AP3426_REG_PS_THDL_H_SHIFT	(0)
#define AP3426_REG_PS_THDL_H_MASK		0x03

#define AP3426_REG_PS_THDH_L       0x2C
#define AP3426_REG_PS_THDH_L_SHIFT	(0)
#define AP3426_REG_PS_THDH_L_MASK		0xFF

#define AP3426_REG_PS_THDH_H       0x2D
#define AP3426_REG_PS_THDH_H_SHIFT	(0)
#define AP3426_REG_PS_THDH_H_MASK		0x03


/*============================================================================*/
//SYSTEM MODE (AP3426_REG_SYS_CONF)
#define	AP3426_SYS_DEV_DOWN        0x00
#define	AP3426_SYS_ALS_ENABLE      0x01
#define	AP3426_SYS_PS_ENABLE       0x02
#define	AP3426_SYS_ALS_PS_ENABLE   0x03
#define	AP3426_SYS_DEV_RESET       0x04
/*----------------------------------------------------------------------------*/
//INT FLAG BIT MASK
#define	AP3426_SYS_ALS_INT_TRI     0x01
#define	AP3426_SYS_PS_INT_TRI      0x02
#define	AP3426_SYS_PS_INT_OBJ      0x10
#define	AP3426_SYS_PS_INT_IROV     0x20
/*----------------------------------------------------------------------------*/
//INT CLEAN Mode
#define	AP3426_SYS_ICLEAN_AUTO     0x00
#define	AP3426_SYS_ICLEAN_MANUAL   0x01
/*----------------------------------------------------------------------------*/
//ALS CONFIG
#define AP3426_ALS_RANGE_0         0x00	/* Full range 32768 lux (0.5lux/count) */
#define AP3426_ALS_RANGE_1         0x01	/* Full range 8192 lux */
#define AP3426_ALS_RANGE_2         0x02	/* Full range 2048 lux */
#define AP3426_ALS_RANGE_3         0x03	/* Full range 512 lux */
#define AP3426_ALS_RANGE_MASK		0x30
#define AP3426_ALS_RANGE_SHIFT	(4)
#define AP3426_ALS_PERSIST_MASK	0x0F

/*----------------------------------------------------------------------------*/
//PS CONFIG
#define AP3426_PS_GAIN_1           0x00 /* PS resulation * 1 */
#define AP3426_PS_GAIN_2           0x01 /* PS resulation * 2 */
#define AP3426_PS_GAIN_4           0x02 /* PS resulation * 4 */
#define AP3426_PS_GAIN_8           0x03 /* PS resulation * 8 */
#define AP3426_PS_PERSIST_1            0x00
#define AP3426_PS_PERSIST_2            0x01
#define AP3426_PS_PERSIST_4            0x02
#define AP3426_PS_PERSIST_8            0x03
/*----------------------------------------------------------------------------*/
//PS LED Control
#define AP3426_PS_LED_P0        0x00	/* 0 puls */
#define AP3426_PS_LED_P1         0x01	/* 1 puls (default)*/
#define AP3426_PS_LED_P2         0x02	/* 2 puls  */
#define AP3426_PS_LED_P3         0x03	/* 3 puls  */
#define AP3426_PS_DEIVER_167         0x00	/* 16.7% */
#define AP3426_PS_DEIVER_333         0x01	/* 33.3% */
#define AP3426_PS_DEIVER_667         0x02	/* 66.7% */
#define AP3426_PS_DEIVER_1000         0x03	/* 100% (default)*/
/*----------------------------------------------------------------------------*/
//PS MEAN
#define AP3426_PS_MEAN_0         0x00	/* 5ms @2T*/
#define AP3426_PS_MEAN_1         0x01	/* 9.6ms @2T*/
#define AP3426_PS_MEAN_2         0x02	/* 14.1ms @2T*/
#define AP3426_PS_MEAN_3         0x03	/* 18.7ms @2T*/
/*----------------------------------------------------------------------------*/
#define DISABLE                     0x00
#define ENABLE                      0x01
/*============================================================================*/


/*----------------------------------------------------------------------------*/
//PS Engineering Registers
#define AP3426_REG_PS_DC_1         0x30 /*Only in Engineering chip, couldn't find in datasheet*/
#define AP3426_REG_PS_DC_1_SHIFT         (0) 
#define AP3426_REG_PS_DC_1_MASK         0xFF 
#define AP3426_REG_PS_DC_2         0x32 /*Only in Engineering chip, couldn't find in datasheet*/
#define AP3426_REG_PS_DC_2_SHIFT         (0) 
#define AP3426_REG_PS_DC_2_MASK         0xFF 

/*----------------------------------------------------------------------------*/
#define PROXIMITYSENSOR_IOCTL_MAGIC 'c'
#define PROXIMITYSENSOR_IOCTL_GET_ENABLED \
		_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 1, int *)
#define PROXIMITYSENSOR_IOCTL_ENABLE \
		_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 2, int *)

/*ASUS - bevis chen-- add +*/
#define PBUFF_SIZE				16	/* Rx buffer size */

#define ASUS_PSENSOR_IOCTL_CLOSE		         _IO(PROXIMITYSENSOR_IOCTL_MAGIC, 0x12)
#define ASUS_PSENSOR_IOCTL_START		         _IO(PROXIMITYSENSOR_IOCTL_MAGIC, 0x13)
#define ASUS_PSENSOR_IOCTL_GETDATA             _IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 0x14, char[PBUFF_SIZE+1])

#define ASUS_PSENSOR_SETCALI_DATA	         _IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 0x15, int[2])
#define ASUS_PSENSOR_EN_CALIBRATION           _IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 0x16, char)
/*ASUS - bevis chen-- add -*/


#define LIGHTSENSOR_IOCTL_MAGIC 'l'

#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)

/*ASUS - bevis chen-- add +*/
#define LBUFF_SIZE				16	/* Rx buffer size */

#define ASUS_LIGHTSENSOR_IOCTL_CLOSE		         _IO(LIGHTSENSOR_IOCTL_MAGIC, 0x12)
#define ASUS_LIGHTSENSOR_IOCTL_START		         _IO(LIGHTSENSOR_IOCTL_MAGIC, 0x13)
#define ASUS_LIGHTSENSOR_IOCTL_GETDATA             _IOR(LIGHTSENSOR_IOCTL_MAGIC, 0x14, char[LBUFF_SIZE+1])

#define ASUS_LIGHTSENSOR_SETCALI_DATA	         _IOW(LIGHTSENSOR_IOCTL_MAGIC, 0x15, int[2])
#define ASUS_LIGHTSENSOR_EN_CALIBRATION           _IOW(LIGHTSENSOR_IOCTL_MAGIC, 0x16, char)
/*ASUS - bevis chen-- add -*/


#endif

