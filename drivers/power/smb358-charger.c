/* Copyright (c) 2014 The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "SMB358 %s: " fmt, __func__
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/qpnp/qpnp-adc.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include "asus_battery.h"
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <linux/alarmtimer.h>

#define _SMB358_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB358_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB358_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

#define _SMB358_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define SMB358_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
		_SMB358_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
			(RIGHT_BIT_POS))

/* Config/Control registers */
#define CHG_CURRENT_CTRL_REG		0x0
#define CHG_OTH_CURRENT_CTRL_REG	0x1
#define VARIOUS_FUNC_REG		0x2
#define VFLOAT_REG			0x3
#define CHG_CTRL_REG			0x4
#define STAT_AND_TIMER_CTRL_REG		0x5
#define CHG_PIN_EN_CTRL_REG		0x6
#define THERM_A_CTRL_REG		0x7
#define SYSOK_AND_USB3_REG		0x8
#define OTHER_CTRL_REG			0x9
#define OTG_TLIM_THERM_CNTRL_REG	0x0A	/* Non-Volatile + mirror */
#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG    0x0B
#define CHARGE_CURRENT_COMPENSATION         SMB358_MASK(7, 6)
#define FLOAT_VOLTAGE_MASK                SMB358_MASK(5, 0)
#define SMB358_FLOAT_VOLTAGE_VALUE_4380mV    (BIT(0)|BIT(2)|BIT(3)|BIT(5))
#define SMB358_FLOAT_VOLTAGE_VALUE_4340mV    (BIT(1)|BIT(3)|BIT(5))
#define SMB358_FLOAT_VOLTAGE_VALUE_4140mV    BIT(5)
#define SMB358_FLOAT_VOLTAGE_VALUE_4100mV    (BIT(1)|BIT(2)|BIT(3)|BIT(4))
#define SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA    BIT(7)
#define SMB358_RECHARGE_VRECH_VOLTAGE BIT(2)
#define FAULT_INT_REG			0xC
#define STATUS_INT_REG			0xD
#define HARD_LIMIT_HOT_CELL_TEMP_MASK            SMB358_MASK(5, 4)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK            SMB358_MASK(3, 2)
#define SMB358_FAST_CHG_CURRENT_VALUE_600mA    BIT(6)
#define SMB358_FAST_CHG_CURRENT_VALUE_900mA    (BIT(6)|BIT(5))
#define SMB358_FAST_CHG_CURRENT_VALUE_1300mA    BIT(7)
#define SMB358_FAST_CHG_CURRENT_VALUE_2000mA    (BIT(7)|BIT(6)|BIT(5))

#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK    0x03
/* Command registers */
#define CMD_A_REG			0x30
#define CMD_B_REG			0x31

/* IRQ status registers */
#define IRQ_A_REG			0x35
#define IRQ_B_REG			0x36
#define IRQ_C_REG			0x37
#define IRQ_D_REG			0x38
#define IRQ_E_REG			0x39
#define IRQ_F_REG			0x3A

/* Status registers */
#define STATUS_C_REG			0x3D
#define STATUS_D_REG			0x3E
#define STATUS_E_REG			0x3F

/* Config bits */
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT            BIT(4)
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_ENABLE				0x20
#define CFG_PIN_EN_DISABLE				0x00
#define CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE    BIT(4)
#define CFG_CURRENT_LIMIT_SMB358_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB358_VALUE_1000 0x30
#define CFG_CURRENT_LIMIT_SMB358_VALUE_500  0x10
#define CFG_CURRENT_LIMIT_SMB358_VALUE_700  0x20
#define CFG_CURRENT_LIMIT_SMB358_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB358_VALUE_2000 0x70
#define CHG_INHI_EN_MASK			BIT(1)
#define CHG_INHI_EN_BIT				BIT(1)
#define CMD_A_CHG_ENABLE_BIT			BIT(1)
#define CMD_A_VOLATILE_W_PERM_BIT		BIT(7)
#define CMD_A_CHG_SUSP_EN_BIT			BIT(2)
#define CMD_A_CHG_SUSP_EN_MASK			BIT(2)
#define CMD_A_OTG_ENABLE_BIT			BIT(4)
#define CMD_A_OTG_ENABLE_MASK			BIT(4)
#define CMD_B_CHG_HC_ENABLE_BIT			BIT(0)
#define USB3_ENABLE_BIT				BIT(5)
#define USB3_ENABLE_MASK			BIT(5)
#define CMD_B_CHG_USB_500_900_ENABLE_BIT	BIT(1)
#define CHG_CTRL_AUTO_RECHARGE_ENABLE_BIT	0x0
#define CHG_CTRL_CURR_TERM_END_CHG_BIT		0x0
#define CHG_CTRL_BATT_MISSING_DET_THERM_IO	SMB358_MASK(5, 4)
#define CHG_CTRL_AUTO_RECHARGE_MASK		BIT(7)
#define CHG_AUTO_RECHARGE_DIS_BIT		BIT(7)
#define CHG_CTRL_CURR_TERM_END_MASK		BIT(6)
#define CHG_CTRL_BATT_MISSING_DET_MASK		SMB358_MASK(5, 4)
#define CHG_CTRL_APSD_EN_BIT			BIT(2)
#define CHG_CTRL_APSD_EN_MASK			BIT(2)
#define CHG_ITERM_MASK				0x07
#define CHG_PIN_CTRL_USBCS_REG_BIT		0x0
#define SMB358_FAST_CHG_CURRENT_MASK            SMB358_MASK(7, 5)
#define SMB358_TERMINATION_CURRENT_MASK         SMB358_MASK(2, 0)
#define SMB358_TERMINATION_CURRENT_VALUE_100mA    BIT(2)
#define SMB358_TERMINATION_CURRENT_VALUE_80mA    (BIT(0)|BIT(1))
#define SMB358_FAST_CHG_CURRENT_VALUE_1300mA    BIT(7)
/* This is to select if use external pin EN to control CHG */
#define CHG_PIN_CTRL_CHG_EN_LOW_PIN_BIT		SMB358_MASK(6, 5)
#define CHG_PIN_CTRL_CHG_EN_LOW_REG_BIT		0x0
#define CHG_PIN_CTRL_CHG_EN_MASK		SMB358_MASK(6, 5)
#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB358_MASK(3, 2)

#define RECHARGE_VOLTAGE_MASK    SMB358_MASK(3, 2)
#define CHG_LOW_BATT_THRESHOLD \
				SMB358_MASK(3, 0)
#define CHG_PIN_CTRL_USBCS_REG_MASK		BIT(4)
#define CHG_PIN_CTRL_APSD_IRQ_BIT		BIT(1)
#define CHG_PIN_CTRL_APSD_IRQ_MASK		BIT(1)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_BIT		BIT(2)
#define CHG_PIN_CTRL_CHG_ERR_IRQ_MASK		BIT(2)
#define VARIOUS_FUNC_USB_SUSP_EN_REG_BIT	BIT(6)
#define VARIOUS_FUNC_USB_SUSP_MASK		BIT(6)
#define FAULT_INT_HOT_COLD_HARD_BIT		BIT(7)
#define FAULT_INT_HOT_COLD_SOFT_BIT		BIT(6)
#define FAULT_INT_INPUT_OV_BIT			BIT(3)
#define FAULT_INT_INPUT_UV_BIT			BIT(2)
#define FAULT_INT_AICL_COMPLETE_BIT		BIT(1)
#define STATUS_INT_CHG_TIMEOUT_BIT		BIT(7)
#define STATUS_INT_OTG_DETECT_BIT		BIT(6)
#define STATUS_INT_BATT_OV_BIT			BIT(5)
#define STATUS_INT_CHGING_BIT			BIT(4)
#define STATUS_INT_CHG_INHI_BIT			BIT(3)
#define STATUS_INT_INOK_BIT			BIT(2)
#define STATUS_INT_MISSING_BATT_BIT		BIT(1)
#define STATUS_INT_LOW_BATT_BIT			BIT(0)
#define THERM_A_THERM_MONITOR_EN_BIT		0x0
#define THERM_A_THERM_MONITOR_EN_MASK		BIT(4)
#define VFLOAT_MASK				0x3F
#define RECHARGER_FLAG				0x20

/* IRQ status bits */
#define IRQ_A_HOT_HARD_BIT			BIT(6)
#define IRQ_A_COLD_HARD_BIT			BIT(4)
#define IRQ_A_HOT_SOFT_BIT			BIT(2)
#define IRQ_A_COLD_SOFT_BIT			BIT(0)
#define IRQ_B_BATT_MISSING_BIT			BIT(4)
#define IRQ_B_BATT_LOW_BIT			BIT(2)
#define IRQ_B_BATT_OV_BIT			BIT(6)
#define IRQ_B_PRE_FAST_CHG_BIT			BIT(0)
#define IRQ_C_TAPER_CHG_BIT			BIT(2)
#define IRQ_C_TERM_BIT				BIT(0)
#define IRQ_C_INT_OVER_TEMP_BIT			BIT(6)
#define IRQ_D_CHG_TIMEOUT_BIT			(BIT(0) | BIT(2))
#define IRQ_D_AICL_DONE_BIT			BIT(4)
#define IRQ_D_APSD_COMPLETE			BIT(6)
#define IRQ_E_INPUT_UV_BIT			BIT(0)
#define IRQ_E_INPUT_OV_BIT			BIT(2)
#define IRQ_E_AFVC_ACTIVE                       BIT(4)
#define IRQ_F_OTG_VALID_BIT			BIT(2)
#define IRQ_F_OTG_BATT_FAIL_BIT			BIT(4)
#define IRQ_F_OTG_OC_BIT			BIT(6)
#define IRQ_F_POWER_OK				BIT(0)
#define SMB358_VFLT_50mA    0x00
#define SMB358_VFLT_100mA    BIT(2)
#define SMB358_VFLT_200mA    BIT(3)
#define SMB358_HARD_HOT_LIMIT_59_degree    BIT(4)
#define SMB358_OTG_CURRENT_LIMIT_500mA    BIT(2)
//<asus-guc20150503+>
#define SMB358_OTG_CURRENT_LIMIT_250mA    0x00
//<asus-guc20150503->
/* Status  bits */
#define STATUS_C_CHARGING_MASK			SMB358_MASK(2, 1)
#define STATUS_C_FAST_CHARGING			BIT(2)
#define STATUS_C_PRE_CHARGING			BIT(1)
#define STATUS_C_TAPER_CHARGING			SMB358_MASK(2, 1)
#define STATUS_C_CHG_ERR_STATUS_BIT		BIT(6)
#define STATUS_C_CHG_ENABLE_STATUS_BIT		BIT(0)
#define STATUS_C_CHG_HOLD_OFF_BIT		BIT(3)
#define STATUS_D_CHARGING_PORT_MASK \
				SMB358_MASK(3, 0)
#define STATUS_D_PORT_ACA_DOCK			BIT(3)
#define STATUS_D_PORT_SDP			BIT(2)
#define STATUS_D_PORT_DCP			BIT(1)
#define STATUS_D_PORT_CDP			BIT(0)
#define STATUS_D_PORT_OTHER			SMB358_MASK(1, 0)
#define STATUS_D_PORT_ACA_A			(BIT(2) | BIT(0))
#define STATUS_D_PORT_ACA_B			SMB358_MASK(2, 1)
#define STATUS_D_PORT_ACA_C			SMB358_MASK(2, 0)

/* constants */
#define USB2_MIN_CURRENT_MA		100
#define USB2_MAX_CURRENT_MA		500
#define USB3_MIN_CURRENT_MA		150
#define USB3_MAX_CURRENT_MA		900
#define AC_CHG_CURRENT_MASK		0x70
#define AC_CHG_CURRENT_SHIFT		4
#define SMB358_IRQ_REG_COUNT		6
#define SMB358_FAST_CHG_MIN_MA		200
#define SMB358_FAST_CHG_MAX_MA		2000
#define SMB358_FAST_CHG_SHIFT		5
#define SMB_FAST_CHG_CURRENT_MASK	0xE0
#define SMB358_DEFAULT_BATT_CAPACITY	50
#define SMB358_BATT_GOOD_THRE_2P5	0x1
#define REPORT_CAPACITY_POLLING_TIME (180)
#define AICL_MINIMUM_INTERVAL (30)
#define SMB358_BATTID_INITIAL -1
static int VCH_VALUE=SMB358_FLOAT_VOLTAGE_VALUE_4380mV;
int64_t battID = SMB358_BATTID_INITIAL;
//BSP Ben disable g_Iqc
//int g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;		
int cfg_current_limit = CFG_CURRENT_LIMIT_SMB358_VALUE_2000; 
static bool JEITA_Flag=false;    //flag for 5W adapter by kerwin
static bool fast_chg_flag = false;    //flag for fast_chg_flag by kerwin
//extern char asus_lcd_id[2];//for ze550 and ze551 [0]='0' or '1' ze550kl, [0]='2','3'
void smb358_update_aicl_work(int time);
static int Thermal_Policy(int *thermal_policy_current);
static int Thermal_Policy_ze550(int *thermal_policy_current);
//kerwin for fast charger thermal policy
extern int Thermal_Level;
static int Thermal_Level_old=0;
/*factory*/
//#define ASUS_FACTORY_BUILD
bool Batt_ID_reday = false;
bool Batt_ID_ERROR=false;
extern bool thermal_abnormal;
//BSP Ben add adapter_id for 5w (BZ sku) porting
//adapter_id:   0 = 5w
//              1 = 10w
extern int asus_project_ADAPTER_ID;

enum {
	USER	= BIT(0),
	THERMAL = BIT(1),
	CURRENT = BIT(2),
	SOC	= BIT(3),
};

struct smb358_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

#if defined(ASUS_FACTORY_BUILD)
bool eng_charging_limit;
bool g_charging_toggle_for_charging_limit;
bool charger_suspend_for_charging_limit;	
int charger_limit_setting;
#endif
/*********************************************
 *******Project Oriented Function List********
 *********************************************
 *
 *  smb358_charging_toggle_zd550_ze600(charging_toggle_level_t level, bool on)
 *  smb358_pre_config_zd550_ze600(void)
 *  smb3xx_config_max_current_zd550_ze600(int usb_state)
 *  smb358_soc_detect_batt_tempr_zd550_ze600(int usb_state)
 *  smb358_set_recharge_voltage_zd550_ze600(void)
 *
 *
 ************5W BZ sku porting*****************
 *  smb358_pre_config_5wBZsku_zd550(void)
 *  smb3xx_config_max_current_5wBZsku_zd550(int usb_state)
 *  smb358_soc_detect_batt_tempr_5wBZsku_zd550(int usb_state)
 *
 */

/*****************************************
  ************default for ze551kl and so on, add special function for ze550kl
 *  smb358_charging_toggle_ze550(charging_toggle_level_t level, bool on)
 *  smb358_pre_config_ze550(void)
 *  smb3xx_config_max_current_ze550(int usb_state)
 *  smb358_soc_detect_batt_tempr_ze550(int usb_state)
 *  smb358_set_recharge_voltage_ze550(void)             
*/

static int gp_sdio_2_clk;
static int gp_inok;
static int g_usb_state = CABLE_OUT;
static struct wake_lock inok_wakelock;
static struct smb358_charger *smb358_dev;
DEFINE_MUTEX(g_usb_state_lock);
DEFINE_MUTEX(g_charging_toggle_lock);
static bool g_charging_toggle = true;
struct smb358_charger {
	struct i2c_client	*client;
	struct device		*dev;

	bool			inhibit_disabled;
	bool			recharge_disabled;
	int			recharge_mv;
	bool			iterm_disabled;
	int			iterm_ma;
	int			vfloat_mv;
	int			chg_valid_gpio;
	int			chg_valid_act_low;
	int			chg_present;
	int			fake_battery_soc;
	bool			chg_autonomous_mode;
	bool			disable_apsd;
	bool			using_pmic_therm;
	bool			battery_missing;
	const char		*bms_psy_name;
	bool			resume_completed;
	bool			irq_waiting;
	bool			bms_controlled_charging;
	struct mutex		read_write_lock;
	struct mutex		path_suspend_lock;
	struct mutex		irq_complete;
	u8			irq_cfg_mask[2];
	int			irq_gpio;
	int			charging_disabled;
	int			fastchg_current_max_ma;
	unsigned int		cool_bat_ma;
	unsigned int		warm_bat_ma;
	unsigned int		cool_bat_mv;
	unsigned int		warm_bat_mv;

	/* debugfs related */
#if defined(CONFIG_DEBUG_FS)
	struct dentry		*debug_root;
	u32			peek_poke_address;
#endif
	/* status tracking */
	bool			batt_full;
	bool			batt_hot;
	bool			batt_cold;
	bool			batt_warm;
	bool			batt_cool;
	bool			jeita_supported;
	int			charging_disabled_status;
	int			usb_suspended;

	/* power supply */
	struct power_supply	*usb_psy;
	struct power_supply	*bms_psy;
	struct power_supply	batt_psy;

	/* otg 5V regulator */
	struct smb358_regulator	otg_vreg;

	/* adc_tm paramters */
	struct qpnp_vadc_chip	*vadc_dev;
	struct qpnp_adc_tm_chip	*adc_tm_dev;
	struct qpnp_adc_tm_btm_param	adc_param;
	int			cold_bat_decidegc;
	int			hot_bat_decidegc;
	int			cool_bat_decidegc;
	int			warm_bat_decidegc;
	int			bat_present_decidegc;
	/* i2c pull up regulator */
	struct regulator	*vcc_i2c;
};


/* Input current limit in mA */
static const unsigned int icl_tbl[] = {
	300,
	500,
	700,
	1000,
	1200,
	1300,
	1800,
	2000,
};

//<asus-guc20150423+>
struct smb_adaptive_factors {
	/*voltage and current*/
	int		VCH_VALUE_RANGE_06;
	int		g_Iqc_RANGE_01;
	/*battery's temperature*/
	int		batt_tempr_00;
	int		batt_tempr_01;
	int		batt_tempr_02;
	int		batt_tempr_03;
	int		batt_tempr_04;
	/*battery's temperature with 30 units of hysteresis*/
	int		batt_tempr_00_hyst30;
	int		batt_tempr_01_hyst30;
	int		batt_tempr_02_hyst30;
	int		batt_tempr_03_hyst30;
	int		batt_tempr_04_hyst30;
	/*frequency of SET_VCH_VALUE*/
	int		SET_VCH_FREQ;

};
static struct smb_adaptive_factors PROJ_ASUS_ZE550KL = {
	/*voltage and current*/
	SMB358_FLOAT_VOLTAGE_VALUE_4140mV, 
	SMB358_FAST_CHG_CURRENT_VALUE_600mA, 
	/*battery's temperature*/
	15,
	100,
	200,
	500,
	600,
	/*battery's temperature with 30 units of hysteresis*/
	45, 
	130, 
	230, 
	480, 
	570,
	/*frequency of SET_VCH_VALUE*/
	3
};

static struct smb_adaptive_factors PROJ_ASUS_ZE600KL = {
	/*voltage and current*/
	SMB358_FLOAT_VOLTAGE_VALUE_4380mV,
	SMB358_FAST_CHG_CURRENT_VALUE_1300mA, 
	/*battery's temperature*/
	10,
	100,
	200,
	500,
	600,
	/*battery's temperature with 30 units of hysteresis*/
	40, 
	130, 
	230, 
	470, 
	570, 
	/*frequency of SET_VCH_VALUE*/
	5		// (??) Yet to be verified
};

static struct smb_adaptive_factors PROJ_ASUS_ZX550KL = {
	/*voltage and current*/
	SMB358_FLOAT_VOLTAGE_VALUE_4140mV, 
	SMB358_FAST_CHG_CURRENT_VALUE_600mA, 
	/*battery's temperature*/
	15,
	100,
	200,
	500,
	600,
	/*battery's temperature with 30 units of hysteresis*/
	45, 
	130, 
	230, 
	480, 
	570,
	/*frequency of SET_VCH_VALUE*/
	3
};

static struct smb_adaptive_factors PROJ_ASUS_ZD550KL = {
	/*voltage and current*/
	SMB358_FLOAT_VOLTAGE_VALUE_4380mV, 
	SMB358_FAST_CHG_CURRENT_VALUE_1300mA, 
	/*battery's temperature*/
	10,
	100,
	200,
	500,
	600,
	/*battery's temperature with 30 units of hysteresis*/
	40, 
	130, 
	230, 
	470, 
	570,
	/*frequency of SET_VCH_VALUE*/
	5		// (??) Yet to be verified
};

struct	smb_adaptive_factors	*charger_factor;

//<asus-guc20150427+>
static void determine_project_id(int pid)
{
	switch(pid) {
		case ASUS_ZE600KL:
			charger_factor = &PROJ_ASUS_ZE600KL;
			BAT_DBG("%s: Using ZE600KL's setting\n", __FUNCTION__);
			break;
		case ASUS_ZX550KL:
			charger_factor = &PROJ_ASUS_ZX550KL;
			BAT_DBG("%s: Using ZX550KL's setting\n", __FUNCTION__);
			break;
		case ASUS_ZD550KL:
			charger_factor = &PROJ_ASUS_ZD550KL;
			BAT_DBG("%s: Using ZD550KL's setting\n", __FUNCTION__);
			break;
		case ASUS_ZE550KL:
		default:
			charger_factor = &PROJ_ASUS_ZE550KL;
			BAT_DBG("%s: Using ZE550KL's setting\n", __FUNCTION__);
			break;
	}
}
//<asus-guc20150427->
//<asus-guc20150424->

static uint8_t led_type = 0; //ASUS BSP Austin_T : LED charger mode +++

static int __smb358_read_reg(struct smb358_charger *chip, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c read fail: can't read from %02x: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int __smb358_write_reg(struct smb358_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		dev_err(chip->dev,
			"i2c write fail: can't write %02x to %02x: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb358_read_reg(struct smb358_charger *chip, int reg,
						u8 *val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_write_reg(struct smb358_charger *chip, int reg,
						u8 val)
{
	int rc;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_write_reg(chip, reg, val);
	mutex_unlock(&chip->read_write_lock);

	return rc;
}

static int smb358_masked_write(struct smb358_charger *chip, int reg,
							u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	mutex_lock(&chip->read_write_lock);
	rc = __smb358_read_reg(chip, reg, &temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_read_reg Failed: reg=%03X, rc=%d\n", reg, rc);
		goto out;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = __smb358_write_reg(chip, reg, temp);
	if (rc) {
		dev_err(chip->dev,
			"smb358_write Failed: reg=%03X, rc=%d\n", reg, rc);
	}
out:
	mutex_unlock(&chip->read_write_lock);
	return rc;
}

static int smb358_enable_volatile_writes(struct smb358_charger *chip)
{
	int rc;

	rc = smb358_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_PERM_BIT,
						CMD_A_VOLATILE_W_PERM_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n",
				rc);

	return rc;
}

static int config_otg_regs(int toggle)
{
	int ret;

	dev_err(smb358_dev->dev, "%s++\n", __FUNCTION__);
	if (toggle) {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}
		/* Toggle to enable OTG function: output High */
		gpio_set_value(gp_sdio_2_clk, 1);

		/* Set OTG current limit to 500mA: 0Ah[3:2]="01" */
		ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			SMB358_OTG_CURRENT_LIMIT_500mA);
		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
			return ret;
		}
	} else{

		/* Set OTG current limit to 500mA: 0Ah[3:2]="00" (OTG current limit at USBIN == 250mA)*/
		ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			SMB358_OTG_CURRENT_LIMIT_250mA);	//<asus-guc20150427>	BIT(2)
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(gp_sdio_2_clk, 0);

	}
	
	dev_err(smb358_dev->dev, "%s--\n", __FUNCTION__);
    return ret;
}

/*----------------------------------------------------------------------------*/
static int otg(int toggle)
{
	int ret;

	dev_err(smb358_dev->dev, "%s++\n", __FUNCTION__);
	if (!smb358_dev) {
		pr_info("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}


	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;


	ret = config_otg_regs(toggle);
	if (ret < 0)
		return ret;

	dev_err(smb358_dev->dev, "%s--\n", __FUNCTION__);
	return 0;
}
/*+++BSP David Charger Proc Data Structure+++*/
struct ChargerReg_command{
	u8 addr;
	enum readwrite{
		E_READ = 0,
		E_WRITE = 1,
		E_READWRITE = 2,
		E_NOUSE = 3,
	} rw;
};

struct ChargerReg_command ChargerReg_CMD_Table[] = {
	{0x00,	E_READWRITE},
	{0x01,	E_READWRITE},
	{0x02,	E_READWRITE},
	{0x03,	E_READWRITE},
	{0x04,	E_READ},
	{0x05,	E_READ},
	{0x06,	E_READWRITE},
	{0x07,	E_READWRITE},
	{0x08,	E_READ},
	{0x09,	E_READWRITE},
	{0x0a,	E_READWRITE},
	{0x0b,	E_READWRITE},
	{0x0c,	E_READ},
	{0x0d,	E_READ},
	{0x0e,	E_READ},
	{0x30,	E_READWRITE},
	{0x31,	E_READWRITE},
	{0x33,	E_READ},
	{0x35,	E_READ},
	{0x36,	E_READ},
	{0x37,	E_READ},
	{0x38,	E_READ},
	{0x39,	E_READ},
	{0x3a,	E_READ},
	{0x3b,	E_READ},
	{0x3c,	E_READ},
	{0x3d,	E_READ},
	{0x3e,	E_READ},
	{0x3f,	E_READ},
};
/*---BSP David Charger Proc Data Structure---*/
static int ChargerReg_proc_read_By_Table(int cmd)
{
	u8 Result;
	if (cmd >= 0 && cmd < ARRAY_SIZE(ChargerReg_CMD_Table)) {
		if (E_READ == ChargerReg_CMD_Table[cmd].rw || E_READWRITE == ChargerReg_CMD_Table[cmd].rw) {
			if (smb358_dev) {
				smb358_read_reg(smb358_dev, ChargerReg_CMD_Table[cmd].addr, &Result);
			} else{
				/*do nothing*/
			}
		} else{
			/*do nothing*/
		}
	} else{
		/*do nothing*/
	}
	return Result;
}

void ChargerRegDump(char tempString[], int length)
{
	int i = 0;
	int Charger_reg_value = -1;
	BAT_DBG("Enter %s!", __func__);
	for (i = 0; i < ARRAY_SIZE(ChargerReg_CMD_Table); i++) {
		Charger_reg_value = ChargerReg_proc_read_By_Table(i);
		snprintf(tempString, length, "%s%02x", tempString, Charger_reg_value);
	}
}

extern int get_charger_type(void)
{
	int ret;

	if (!smb358_dev) {
		pr_err("Warning: smb358_dev is null due to probe function has error\n");
		return -1;
	}
	ret = g_usb_state;
	return ret;
}
struct delayed_work LED_ChargerMode;
struct delayed_work USB_3s_retry_kicker_work;
struct delayed_work USB_3s_retry_work;
extern void led_set_charger_mode(uint8_t led_type); //ASUS BSP Austin_T : LED charger mode +++
extern void focal_usb_detection(bool plugin);          //ASUS BSP Jacob_kung : notify touch cable in +++
/* write 06h[6:5]="00" or "11" */
int smb358_charging_toggle(charging_toggle_level_t level, bool on)
{
	int ret = 0;
	u8 reg;
	int old_toggle;
	int result_toggle;
	bool reject = false;
	static charging_toggle_level_t old_lvl = JEITA;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb358_dev) {
		pr_info("Warning: smb358_dev is null "
			"due to probe function has error\n");
		return 1;
	}

	mutex_lock(&g_charging_toggle_lock);
	old_toggle = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

	if (Batt_ID_ERROR || (Thermal_Level == 3)){
		if(Thermal_Level == 3){
			BAT_DBG("Thermal_Level is %d, set discharging!\n", Thermal_Level);
		} else if (Batt_ID_ERROR){
		BAT_DBG("BattID is out of range, set discharging!\n");
		}
		result_toggle = false;
		//focal_usb_detection(false); //ASUS BSP Jacob_kung : notify touch cable out +++
		schedule_delayed_work(&LED_ChargerMode, 0*HZ);
	} else {
		result_toggle = on;
	}
	/* do charging or not? */
	if (level != FLAGS) {
		if (on) {
			/* want to start charging? */
			if (level == JEITA) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != JEITA) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else if (level == BALANCE) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != BALANCE) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else {
				/* what the hell are you? */
			}
		} else {
			/* want to stop charging? just do it! */
		}
	} else {
		/* it's the highest level. just do it! */
	}

	BAT_DBG("%s: old_lvl:%s, request_lv:%s, old_toggle:%s, request_toggle:%s, result:%s\n",
		__FUNCTION__,
		level_str[old_lvl],
		level_str[level],
		old_toggle ? "ON" : "OFF",
		on ? "ON" : "OFF",
		result_toggle ? "ON" : "OFF");

	/* level value assignment */
	if (!reject) {
		old_lvl = level;
	}

	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;

	/* Config CFG_PIN register */

	ret = smb358_read_reg(smb358_dev, CHG_PIN_EN_CTRL_REG, &reg);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	reg &= ~CFG_PIN_EN_CTRL_MASK;
	if (result_toggle) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		reg |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
		    . I2C Control - "0" in Command Register disables charger */
	}
	ret = smb358_write_reg(smb358_dev, CHG_PIN_EN_CTRL_REG, reg);
	if (ret < 0)
		goto out;

	mutex_lock(&g_charging_toggle_lock);
	g_charging_toggle = result_toggle;
	mutex_unlock(&g_charging_toggle_lock);

out:
	return ret;
}

int smb358_charging_toggle_ze550(charging_toggle_level_t level, bool on)
{
	int ret = 0;
	u8 reg;
	int old_toggle;
	int result_toggle;
	bool reject = false;
	static charging_toggle_level_t old_lvl = JEITA;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb358_dev) {
		pr_info("Warning: smb358_dev is null "
			"due to probe function has error\n");
		return 1;
	}

	mutex_lock(&g_charging_toggle_lock);
	old_toggle = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

	if (Batt_ID_ERROR || (Thermal_Level == 3)){
		if(Thermal_Level == 3){
			BAT_DBG("Thermal_Level is %d, set discharging!\n", Thermal_Level);
		} else if (Batt_ID_ERROR){
		BAT_DBG("BattID is out of range, set discharging!\n");
		}
		result_toggle = false;
		//focal_usb_detection(false); //ASUS BSP Jacob_kung : notify touch cable out +++
		schedule_delayed_work(&LED_ChargerMode, 0*HZ);
	} else {
		result_toggle = on;
	}
	/* do charging or not? */
	if (level != FLAGS) {
		if (on) {
			/* want to start charging? */
			if (level == JEITA) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != JEITA) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else if (level == BALANCE) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != BALANCE) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else {
				/* what the hell are you? */
			}
		} else {
			/* want to stop charging? just do it! */
		}
	} else {
		/* it's the highest level. just do it! */
	}

	BAT_DBG("%s: old_lvl:%s, request_lv:%s, old_toggle:%s, request_toggle:%s, result:%s\n",
		__FUNCTION__,
		level_str[old_lvl],
		level_str[level],
		old_toggle ? "ON" : "OFF",
		on ? "ON" : "OFF",
		result_toggle ? "ON" : "OFF");

	/* level value assignment */
	if (!reject) {
		old_lvl = level;
	}

	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;

	/* Config CFG_PIN register */

	ret = smb358_read_reg(smb358_dev, CHG_PIN_EN_CTRL_REG, &reg);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	reg &= ~CFG_PIN_EN_CTRL_MASK;
	if (result_toggle) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		reg |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
		    . I2C Control - "0" in Command Register disables charger */
	}
	ret = smb358_write_reg(smb358_dev, CHG_PIN_EN_CTRL_REG, reg);
	if (ret < 0)
		goto out;

	mutex_lock(&g_charging_toggle_lock);
	g_charging_toggle = result_toggle;
	mutex_unlock(&g_charging_toggle_lock);

out:
	return ret;
}


int smb358_charging_toggle_zd550_ze600(charging_toggle_level_t level, bool on)
{
	int ret = 0;
	u8 reg;
	int old_toggle;
	int result_toggle;
	bool reject = false;
	static charging_toggle_level_t old_lvl = JEITA;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb358_dev) {
		pr_info("Warning: smb358_dev is null "
			"due to probe function has error\n");
		return 1;
	}

	mutex_lock(&g_charging_toggle_lock);
	old_toggle = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

        if (Batt_ID_ERROR || (Thermal_Level == 3)){
		if(Thermal_Level == 3){
			BAT_DBG("Thermal_Level is %d, set discharging!\n", Thermal_Level);
		} else if (Batt_ID_ERROR){
		BAT_DBG("BattID is out of range, set discharging!\n");
		}
		result_toggle = false;
		//focal_usb_detection(false); //ASUS BSP Jacob_kung : notify touch cable out +++
		schedule_delayed_work(&LED_ChargerMode, 0*HZ);
	} else {
		result_toggle = on;
	}

	/* do charging or not? */
	if (level != FLAGS) {
		if (on) {
			/* want to start charging? */
			if (level == JEITA) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != JEITA) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else if (level == BALANCE) {
				if (!old_toggle) {
					/* want to restart charging? */
					if (old_lvl != BALANCE) {
						/* reject the request! someone stop charging before */
						result_toggle = false;
						reject = true;
					}
				}
			} else {
				/* what the hell are you? */
			}
		} else {
			/* want to stop charging? just do it! */
		}
	} else {
		/* it's the highest level. just do it! */
	}

	BAT_DBG("%s: old_lvl:%s, request_lv:%s, old_toggle:%s, request_toggle:%s, result:%s\n",
		__FUNCTION__,
		level_str[old_lvl],
		level_str[level],
		old_toggle ? "ON" : "OFF",
		on ? "ON" : "OFF",
		result_toggle ? "ON" : "OFF");

	/* level value assignment */
	if (!reject) {
		old_lvl = level;
	}

	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;

	/* Config CFG_PIN register */

	ret = smb358_read_reg(smb358_dev, CHG_PIN_EN_CTRL_REG, &reg);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	reg &= ~CFG_PIN_EN_CTRL_MASK;
	if (result_toggle) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		reg |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
		    . I2C Control - "0" in Command Register disables charger */
	}
	ret = smb358_write_reg(smb358_dev, CHG_PIN_EN_CTRL_REG, reg);
	if (ret < 0)
		goto out;

	mutex_lock(&g_charging_toggle_lock);
	g_charging_toggle = result_toggle;
	mutex_unlock(&g_charging_toggle_lock);

out:
	return ret;
}



static int smb358_pre_config(void)
{
	int ret;
	//u8 reg;

	 /* set fast charge current: 2000mA set 20150507 by kerwin*/
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_FAST_CHG_CURRENT_MASK,
			SMB358_FAST_CHG_CURRENT_VALUE_2000mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set fast charge current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set termination current: 80mA */
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_TERMINATION_CURRENT_MASK,
			SMB358_TERMINATION_CURRENT_VALUE_80mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set termination current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set cold soft limit current: 600mA */
	ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set cold soft limit current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*disable charger*/
	ret = smb358_charging_toggle(JEITA, false);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*set float voltage*/
	ret = smb358_masked_write(smb358_dev,
		VFLOAT_REG,
		FLOAT_VOLTAGE_MASK,
		VCH_VALUE);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set charger voltage, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*enable charger*/
	ret = smb358_charging_toggle(JEITA, true);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*wait aicl complete*/
	/*do{
		//check if AICL completed here, already delay in config input current phase
		ret = smb358_read_reg(smb358_dev, STATUS_E_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E(" %s: fail to read STATUS_E_REG reg\n", __FUNCTION__);
		} else{
			reg = reg & AUTOMATIC_INPUT_CURR_LIMIT_BIT;
			if (reg > 0) {
				reg = 1;
			} else{
				reg = 0;
			}
		}
	}while(reg == 0);*/
fail:
	return ret;
}

static int smb358_pre_config_ze550(void)
{
	int ret;

	 /* set fast charge current: 1300mA set 20150507 by kerwin*/
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_FAST_CHG_CURRENT_MASK,
			SMB358_FAST_CHG_CURRENT_VALUE_1300mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set fast charge current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set termination current: 80mA */
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_TERMINATION_CURRENT_MASK,
			SMB358_TERMINATION_CURRENT_VALUE_80mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set termination current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set cold soft limit current: 600mA */
	ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set cold soft limit current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*disable charger*/
	ret = smb358_charging_toggle_ze550(JEITA, false);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*set float voltage*/
	ret = smb358_masked_write(smb358_dev,
		VFLOAT_REG,
		FLOAT_VOLTAGE_MASK,
		VCH_VALUE);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set charger voltage, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*enable charger*/
	ret = smb358_charging_toggle_ze550(JEITA, true);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
fail:
	return ret;
}


static int smb358_pre_config_zd550_ze600(void)
{
	int ret;
	//u8 reg;

	 /* set fast charge current: 2000mA set 20150507 by kerwin*/
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_FAST_CHG_CURRENT_MASK,
			SMB358_FAST_CHG_CURRENT_VALUE_2000mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set fast charge current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set termination current: 80mA */
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_TERMINATION_CURRENT_MASK,
			SMB358_TERMINATION_CURRENT_VALUE_80mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set termination current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set cold soft limit current: 600mA */
	ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set cold soft limit current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*disable charger*/
	ret = smb358_charging_toggle_zd550_ze600(JEITA, false);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*set float voltage*/
	ret = smb358_masked_write(smb358_dev,
		VFLOAT_REG,
		FLOAT_VOLTAGE_MASK,
		VCH_VALUE);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set charger voltage, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*enable charger*/
	ret = smb358_charging_toggle_zd550_ze600(JEITA, true);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*wait aicl complete*/
	//do{
		/*
		 * ZD550KL BSP Ben:ZD550KL don't wait for AICL result complete bit 3fH[5]
		 * Because fast plug in and out might hang
                 * <asus-guc20150512> ZE600KL to dodge the same issue. 
                 */
		BAT_DBG_E(" %s: [ZD550KL][ZE600KL]don't wait for aicl result OK!\n", __FUNCTION__);
		/*check if AICL completed here, already delay in config input current phase*/
		/*ret = smb358_read_reg(smb358_dev, STATUS_E_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E(" %s: fail to read STATUS_E_REG reg\n", __FUNCTION__);
		} else{
			reg = reg & AUTOMATIC_INPUT_CURR_LIMIT_BIT;
			if (reg > 0) {
				reg = 1;
			} else{
				reg = 0;
			}
		}*/
	//}while(reg == 0);
fail:
	return ret;
}

static int smb358_pre_config_5wBZsku_zd550(void)
{
	int ret;
	//u8 reg;

	 /* set fast charge current: 900mA set 20150729 by BSP Ben*/
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_FAST_CHG_CURRENT_MASK,
			SMB358_FAST_CHG_CURRENT_VALUE_900mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set fast charge current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set termination current: 80mA */
	ret = smb358_masked_write(smb358_dev,
			CHG_CURRENT_CTRL_REG,
			SMB358_TERMINATION_CURRENT_MASK,
			SMB358_TERMINATION_CURRENT_VALUE_80mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set termination current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}

	/* set cold soft limit current: 600mA */
	ret = smb358_masked_write(smb358_dev,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			SMB358_COLD_SOFT_LIMIT_CURRENT_VALUE_600mA);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set cold soft limit current, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*disable charger*/
	ret = smb358_charging_toggle_zd550_ze600(JEITA, false);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*set float voltage*/
	ret = smb358_masked_write(smb358_dev,
		VFLOAT_REG,
		FLOAT_VOLTAGE_MASK,
		VCH_VALUE);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to set charger voltage, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*enable charger*/
	ret = smb358_charging_toggle_zd550_ze600(JEITA, true);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		goto fail;
	}
	/*wait aicl complete*/
	//do{
		/*
		 * ZD550KL BSP Ben:ZD550KL don't wait for AICL result complete bit 3fH[5]
		 * Because fast plug in and out might hang
                 * <asus-guc20150512> ZE600KL to dodge the same issue. 
                 */
		BAT_DBG_E(" %s: [ZD550KL][SMB358][5wBZsku]don't wait for aicl result OK!\n", __FUNCTION__);
		/*check if AICL completed here, already delay in config input current phase*/
		/*ret = smb358_read_reg(smb358_dev, STATUS_E_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E(" %s: fail to read STATUS_E_REG reg\n", __FUNCTION__);
		} else{
			reg = reg & AUTOMATIC_INPUT_CURR_LIMIT_BIT;
			if (reg > 0) {
				reg = 1;
			} else{
				reg = 0;
			}
		}*/
	//}while(reg == 0);
fail:
	return ret;
}



extern int64_t get_battery_id(void);

struct delayed_work Set_vch_val;
void SET_VCH_VALUE(struct work_struct *dat){
		int ret;
        BAT_DBG("BattID: %lld\n", battID);
        if (battID == SMB358_BATTID_INITIAL){
                schedule_delayed_work(&Set_vch_val,1*HZ);
                return;
        }

	if( battID < 500000 && battID >= 200000 ){// ER battery NVT+ATL 23.2K
		VCH_VALUE = SMB358_FLOAT_VOLTAGE_VALUE_4380mV;
		//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
		cfg_current_limit = CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
		BAT_DBG("ER BAT_ID=BAT4, VCH=4.38V\n");
		Batt_ID_ERROR = false;
		// Enable charger if BattID == 4, e.g. 06h[6:5] = "01"
		ret = smb358_masked_write(smb358_dev, CHG_PIN_EN_CTRL_REG, CFG_PIN_EN_CTRL_MASK,
						CFG_PIN_EN_ENABLE);
		if (ret){
			BAT_DBG("Failed to enable charger's pin control\n");
			return;
		}
	}else{
		if(battID < 900000 && battID >= 500000){//SMP+Sony
			VCH_VALUE = SMB358_FLOAT_VOLTAGE_VALUE_4380mV;
			//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
			cfg_current_limit = CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
			BAT_DBG("ER BAT_ID=BAT3, VCH=4.38V\n");
			Batt_ID_ERROR = false;
			// Enable charger's pin control if BattID == 3, e.g. 06h[6:5] = "01"
			ret = smb358_masked_write(smb358_dev, CHG_PIN_EN_CTRL_REG, CFG_PIN_EN_CTRL_MASK,
							CFG_PIN_EN_ENABLE);
			if (ret){
				BAT_DBG("Failed to enable charger's pin control\n");
				return;
			}
		}else{
			if(battID < 1200000 && battID >= 900000){//SMP+Cos
				VCH_VALUE = SMB358_FLOAT_VOLTAGE_VALUE_4380mV;
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				cfg_current_limit = CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
				BAT_DBG("ER BAT_ID=BAT2, VCH=4.38V\n");
				Batt_ID_ERROR = false;
				// Enable charger's pin control if BattID == 2, e.g. 06h[6:5] = "01"
				ret = smb358_masked_write(smb358_dev, CHG_PIN_EN_CTRL_REG, CFG_PIN_EN_CTRL_MASK,
								CFG_PIN_EN_ENABLE);
				if (ret){
					BAT_DBG("Failed to enable charger's pin control\n");
					return;
				}
			}else{
				if(battID < 1500000 && battID >= 1200000){//Cos+Cos
					VCH_VALUE = SMB358_FLOAT_VOLTAGE_VALUE_4380mV;
					//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
					cfg_current_limit = CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
					BAT_DBG("ER BAT_ID=BAT1, VCH=4.38V\n");
					Batt_ID_ERROR = true;
					// Disable charger's pin control if BattID == 1, e.g. 06h[6:5] = "00"
					ret = smb358_masked_write(smb358_dev, CHG_PIN_EN_CTRL_REG, CFG_PIN_EN_CTRL_MASK,
									CFG_PIN_EN_DISABLE);
					if (ret){
						BAT_DBG("Failed to disable charger's pin control\n");
						return;
					}
				}else{//other cell
					VCH_VALUE=SMB358_FLOAT_VOLTAGE_VALUE_4380mV;
					//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
					cfg_current_limit = CFG_CURRENT_LIMIT_SMB358_VALUE_1200;
					BAT_DBG("Other battery cell fail\n");
					Batt_ID_ERROR=true;
					// Disable charger's pin control if BattID belongs to none of above, e.g. 06h[6:5] = "00"
					ret = smb358_masked_write(smb358_dev, CHG_PIN_EN_CTRL_REG, CFG_PIN_EN_CTRL_MASK,
									CFG_PIN_EN_DISABLE);
					if (ret){
						BAT_DBG("Failed to disable charger's pin control\n");
						return;
					}
				}
			}
		}
	}	
	Batt_ID_reday = true;
}

//--0623 charger mode led--
extern bool g_Charger_mode;
extern bool g_bat_full;
extern int get_bms_calculated_soc(int *rsoc);
static bool charger_mode_cable_out = false;
void LED_ChargerMode_Set(struct work_struct *dat){
	int ret = 0;
	int cap;
	static uint8_t OldLedType = 0xFF;

        if(!g_Charger_mode)
		return;
	
	if(Batt_ID_reday){
		ret = get_bms_calculated_soc(&cap);
		if(ret){
			printk("Read Battery Capacity fail\n");
		}

		if(Batt_ID_ERROR){
			led_type = 0;
			if(OldLedType != led_type)
				led_set_charger_mode(led_type);	
			OldLedType = led_type;
			return;
		}
		
		if((cap < 10) && (cap >= 0))
			led_type = 3;
		else{
			if(g_bat_full)
				led_type = 2;
			else{
					if((cap <= 99) && (cap >= 10))
						led_type = 1;
				}
			}
                if(charger_mode_cable_out)
                {
                        printk("Charger mode Cable Out! set LED off\n");
                        led_type = 0;
                }
		if(OldLedType != led_type)
		        led_set_charger_mode(led_type);
                OldLedType = led_type;
	}else
		schedule_delayed_work(&LED_ChargerMode, 1*HZ);
}
//--0623 charger mode led--

static inline struct power_supply *get_psy_usb(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (!strncmp(pst->name,"usb",3)) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);
        BAT_DBG("%s: Can't get usb psy!\n", __FUNCTION__);

	return NULL;
}
static inline int get_usb_charger_type(int *usb_state)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_usb();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TYPE, &val);
	if (!ret){
		switch(val.intval){
                        case POWER_SUPPLY_TYPE_USB:
                                *usb_state = USB_IN;
                                BAT_DBG("[SMB358]%s: detect USB plugging\n", __FUNCTION__);
                                break;
                        case POWER_SUPPLY_TYPE_USB_DCP:
                                *usb_state = AC_IN;
                                BAT_DBG("[SMB358]%s: detect USB plugging\n", __FUNCTION__);
                                break;
                        case POWER_SUPPLY_TYPE_USB_CDP:
                                *usb_state = AC_IN;
                                BAT_DBG("[SMB358]%s: detect USB plugging\n", __FUNCTION__);
                                break;
                        default:
                                BAT_DBG("[SMB358]%s: detect Unknown plugging\n", __FUNCTION__);
                                *usb_state = UNKNOWN_IN;
                                break;
                }
        }
	return ret;
}
//BSP Ben USB_SDP 3s retry
void USB_3s_retry_kicker(struct work_struct *dat){

        BAT_DBG("[SMB358]%s: +++\n", __FUNCTION__);
        cancel_delayed_work_sync(&USB_3s_retry_work);
	schedule_delayed_work(&USB_3s_retry_work, 3*HZ);
        BAT_DBG("[SMB358]%s: ---\n", __FUNCTION__);

}

void USB_3s_retry_set(struct work_struct *dat){
	int chg_type;
        int ret;
        extern int ASUSEvtlog_Power(int);
        BAT_DBG("[SMB358]%s: +++\n", __FUNCTION__);
        ret = get_usb_charger_type(&chg_type);
                if(ret){
                        BAT_DBG("[SMB358]%s: get_usb_charger_type fail!\n", __FUNCTION__);
                        return;
                }
        BAT_DBG("[SMB358]%s: chg_type = %d\n", __FUNCTION__,chg_type);
        if(chg_type == AC_IN){
                BAT_DBG("[SMB358]%s:chg_type= %d ,3s retry sucessed, set charger to AC_IN porting\n", __FUNCTION__,chg_type);
                setSMB358Charger(AC_IN);
                ASUSEvtlog_Power(chg_type);
        }

        BAT_DBG("[SMB358]%s: ---\n", __FUNCTION__);

}


/*+++BSP David proc battID_read Interface+++*/
static int battID_read_proc_read(struct seq_file *buf, void *v)
{
	int64_t battID;
	battID = get_battery_id();
	return seq_printf(buf, "%lld\n", battID);
}
static ssize_t battID_read_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]%s: %d\n", __FUNCTION__, val);

	return len;
}

static int battID_read_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, battID_read_proc_read, NULL);
}

static const struct file_operations battID_read_fops = {
	.owner = THIS_MODULE,
	.open =  battID_read_proc_open,
	.write = battID_read_proc_write,
	.read = seq_read,
	.release = single_release,
};

static void create_battID_read_proc_file(void)
{
	struct proc_dir_entry *battID_read_proc_file = proc_create("driver/battID_read", 0666, NULL, &battID_read_fops);

	if (battID_read_proc_file) {
		printk("[BAT][CHG][SMB][Proc]battID_read create ok!\n");
	} else{
		printk("[BAT][CHG][SMB][Proc]battID_read create failed!\n");
	}
	return;
}
/*---BSP David proc battID_read Interface---*/
/* enable/disable AICL function */
static int smb358_OptiCharge_Toggle(bool on)
{
	int ret;
	u8 reg;

	if (!smb358_dev) {
		pr_info("%s: smb358_dev is null due to driver probed isn't ready\n",
			__FUNCTION__);

		return -1;
	}
	ret = smb358_read_reg(smb358_dev, VARIOUS_FUNC_REG, &reg);
	if (ret < 0)
		goto fail;

	if (on)
		reg |= CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;
	else
		reg &= ~CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;

	ret = smb358_write_reg(smb358_dev, VARIOUS_FUNC_REG, reg);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}
static int smb358_set_current_limits(int usb_state, bool is_twinsheaded)
{
	int ret;

	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;

	if (usb_state == AC_IN) {
		return smb358_masked_write(smb358_dev,
				CHG_OTH_CURRENT_CTRL_REG,
				CFG_CURRENT_LIMIT_SMB358_MASK,
				cfg_current_limit);//update CFG_CURRENT_LIMIT_SMB358_VALUE_2000
	} else if (usb_state == SE1_IN) {
		return smb358_masked_write(smb358_dev,
				CHG_OTH_CURRENT_CTRL_REG,
				CFG_CURRENT_LIMIT_SMB358_MASK,
				CFG_CURRENT_LIMIT_SMB358_VALUE_1000);
	} else if (usb_state == USB_IN) {
	} else if (usb_state == UNKNOWN_IN) {
	}

	return ret;
}
/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl,
		size_t size, unsigned int val)
{
	if (val >= size)
		return tbl[size-1];
	return tbl[val];
}
/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
static int get_aicl_results(void)
{
	int ret;
	u8 reg;
	ret = smb358_read_reg(smb358_dev, STATUS_E_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
		return ret;
	}

	reg &= 0x0F;
	return hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), reg);
}
static void smb3xx_config_max_current(int usb_state)
{
	int aicl_result;

	BAT_DBG("%s: usb_state = %d\n", __FUNCTION__, usb_state);
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/*BSP david: if AICL result is already 2000 mA, don't config
					to prevent low battery capacity shutdown when booting*/
		/*get AICL result*/
		aicl_result = get_aicl_results();
		BAT_DBG(" %s: AICL result = %d\n", __FUNCTION__, aicl_result);
		if (aicl_result >= 2000) {
			BAT_DBG(" %s: AICL result is already 2000 mA, don't config USB_IN!\n", __FUNCTION__);
			return;
		}
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=2000mA - Write 01h[7:4]="0111" */
		if (smb358_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	} else if (usb_state == USB_IN) {
		/*do nothing when USB_IN, using default setting 500mA*/
	} else if (usb_state == UNKNOWN_IN) {
		/*do nothing when UNKNOWN_IN, using default setting 500mA*/
	} else if (usb_state == SE1_IN) {
		/*do nothing when SE1_IN, using setting 1000mA*/
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=1000mA - Write 01h[7:4]="0011" */
		if (smb358_set_current_limits(SE1_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	}
}

static void smb3xx_config_max_current_ze550(int usb_state)
{
	int aicl_result;

	BAT_DBG("%s: usb_state = %d\n", __FUNCTION__, usb_state);
	/* USB Mode Detection (by SOC)  do not diff AC and SE1*/
	if ((usb_state == AC_IN) ||(usb_state == SE1_IN)){
		/*get AICL result*/
		aicl_result = get_aicl_results();
		BAT_DBG(" %s: AICL result = %d\n", __FUNCTION__, aicl_result);
		if (aicl_result >= 1000) {
			BAT_DBG(" %s: AICL result is already 1000 mA, don't config USB_IN!\n", __FUNCTION__);
			return;
		}
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=2000mA - Write 01h[7:4]="0111" */
		if (smb358_set_current_limits(SE1_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	} else if (usb_state == USB_IN) {
		/*do nothing when USB_IN, using default setting 500mA*/
	} else if (usb_state == UNKNOWN_IN) {
		/*do nothing when UNKNOWN_IN, using default setting 500mA*/
	} 
}


static void smb3xx_config_max_current_zd550_ze600(int usb_state)
{
	int aicl_result;

	BAT_DBG("%s: usb_state = %d\n", __FUNCTION__, usb_state);
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/*BSP david: if AICL result is already 2000 mA, don't config
					to prevent low battery capacity shutdown when booting*/
		/*get AICL result*/
		aicl_result = get_aicl_results();
		BAT_DBG(" %s: AICL result = %d\n", __FUNCTION__, aicl_result);
		if (aicl_result >= 2000) {
			BAT_DBG(" %s: AICL result is already 2000 mA, don't config USB_IN!\n", __FUNCTION__);
			return;
		}
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=2000mA - Write 01h[7:4]="0111" */
		if (smb358_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	} else if (usb_state == USB_IN) {
		/*do nothing when USB_IN, using default setting 500mA*/
	} else if (usb_state == UNKNOWN_IN) {
		/*do nothing when UNKNOWN_IN, using default setting 500mA*/
	} else if (usb_state == SE1_IN) {
		/*do nothing when SE1_IN, using setting 1000mA*/
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=1000mA - Write 01h[7:4]="0011" */
		if (smb358_set_current_limits(SE1_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	}
}

static void smb3xx_config_max_current_5wBZsku_zd550(int usb_state)
{
	int aicl_result;

	BAT_DBG("%s: usb_state = %d\n", __FUNCTION__, usb_state);
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/*BSP david: if AICL result is already 1000 mA, don't config
					to prevent low battery capacity shutdown when booting*/
		/*get AICL result*/
		aicl_result = get_aicl_results();
		BAT_DBG(" %s: AICL result = %d\n", __FUNCTION__, aicl_result);
		if (aicl_result >= 1000) {
			BAT_DBG(" %s: AICL result is already 1000 mA, don't config USB_IN!\n", __FUNCTION__);
			return;
		}
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=1000mA - Write 01h[7:4]="0011" */
		if (smb358_set_current_limits(SE1_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	} else if (usb_state == USB_IN) {
		/*do nothing when USB_IN, using default setting 500mA*/
	} else if (usb_state == UNKNOWN_IN) {
		/*do nothing when UNKNOWN_IN, using default setting 500mA*/
	} else if (usb_state == SE1_IN) {
		/*do nothing when SE1_IN, using setting 1000mA*/
		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __FUNCTION__);
			return;
		}

		/* Set I_USB_IN=1000mA - Write 01h[7:4]="0011" */
		if (smb358_set_current_limits(SE1_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__FUNCTION__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __FUNCTION__);
			return;
		}
	}
}

bool smb358_is_charging(int usb_state)
{	
	if (usb_state == USB_IN || usb_state == AC_IN ||usb_state == UNKNOWN_IN || usb_state == SE1_IN || usb_state == PAD_SUPPLY) {
		return true;
	} else{
		return false;
	}
}
static void smb358_config_max_current(int usb_state)
{
	if (!smb358_is_charging(usb_state))
		return;

	if (!smb358_dev) {
		pr_err("%s: smb358_dev is null due to driver probed isn't ready\n",
			__FUNCTION__);

		return;
	}

	/* Allow violate register can be written - Write 30h[7]="1" */
	if (smb358_enable_volatile_writes(smb358_dev) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_enable_volatile_writes failed!\n", __FUNCTION__);
		return;
	}
        //ZD550KL & ZE600KL Porting
        if(asus_PRJ_ID==ASUS_ZD550KL||asus_PRJ_ID==ASUS_ZE600KL)
        {
                if(asus_project_ADAPTER_ID)
                {
	                BAT_DBG("%s: use ZD550KL & ZE600KL charger porting!\n", __FUNCTION__);
                        smb358_pre_config_zd550_ze600();
                        smb3xx_config_max_current_zd550_ze600(usb_state);
                }else{
	                BAT_DBG("%s:[5wBZsku] use ZD550KL & ZE600KL charger porting!\n", __FUNCTION__);
                        smb358_pre_config_5wBZsku_zd550();
                        smb3xx_config_max_current_5wBZsku_zd550(usb_state);
                }
        //ZE550KL & ZE551KL( porting)
        }else if(asus_PRJ_ID==ASUS_ZE550KL){
        	//if((asus_lcd_id[0]=='0') ||(asus_lcd_id[0]=='1')){
	        	BAT_DBG("%s: use default(ZE550KL) charger porting!\n", __FUNCTION__);
                smb358_pre_config_ze550();
	        	smb3xx_config_max_current_ze550(usb_state);
			/*}else if((asus_lcd_id[0]=='2') ||(asus_lcd_id[0]=='3')){
				BAT_DBG("%s: use default(ZE551KL) charger porting!\n", __FUNCTION__);
                smb358_pre_config();
	        	smb3xx_config_max_current(usb_state);
				}*/
        }else{
         		BAT_DBG("%s: use default charger porting!\n", __FUNCTION__);
                smb358_pre_config();
	        	smb3xx_config_max_current(usb_state);
        	}
}
static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}
static inline int get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = val.intval / 1000;

	return ret;
}
static inline int get_battery_temperature(int *tempr)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret)
		*tempr = val.intval;
	else
		printk("%s: using fake temperature: 250\n", __FUNCTION__);
	return ret;
}
static inline int get_battery_rsoc(int *rsoc)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret)
		*rsoc = val.intval;

	return ret;
}
bool smb358_get_full_status(void)
{
	int ret;
	u8 reg;
	bool result = false;
	if (smb358_dev) {
		ret = smb358_read_reg(smb358_dev, STATUS_C_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E(" %s: fail to read STATUS_C_REG reg\n", __FUNCTION__);
		}
		reg = reg & RECHARGER_FLAG;
		if (reg >= 1) {
			result = true;
		}
	}
	return result;
}
int smb358_set_recharge_voltage(void)
{
	int ret;
	u8 reg;  // for 3Dh value
	int cap; //for battery capacity
	ret = get_bms_calculated_soc(&cap);
	if(ret){
		BAT_DBG_E("Read Battery Capacity fail\n");
	}
	ret = smb358_read_reg(smb358_dev, STATUS_C_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STATUS_C_REG reg\n", __FUNCTION__);
	}
	reg = reg & RECHARGER_FLAG;

	/* BSP Clay: Capacity <= 98% and Battery status had been changed to FULL*/
	if (cap <= 98 && reg > 0){
		ret = smb358_charging_toggle(JEITA, false);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		}
		ret = smb358_charging_toggle(JEITA, true);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		}
	}
	return 0;
}

int smb358_set_recharge_voltage_ze550(void)
{
	int ret;
	u8 reg;  // for 3Dh value
	int cap; //for battery capacity
	ret = get_bms_calculated_soc(&cap);
	if(ret){
		BAT_DBG_E("Read Battery Capacity fail\n");
	}
	ret = smb358_read_reg(smb358_dev, STATUS_C_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STATUS_C_REG reg\n", __FUNCTION__);
	}
	reg = reg & RECHARGER_FLAG;

	/* BSP Clay: Capacity <= 98% and Battery status had been changed to FULL*/
	if (cap <= 98 && reg > 0){
		ret = smb358_charging_toggle_ze550(JEITA, false);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		}
		ret = smb358_charging_toggle_ze550(JEITA, true);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		}
	}
	return 0;
}


int smb358_set_recharge_voltage_zd550_ze600(void)
{
	int ret;
	u8 reg;  // for 3Dh value
	int cap; //for battery capacity
	ret = get_bms_calculated_soc(&cap);
	if(ret){
		BAT_DBG_E("Read Battery Capacity fail\n");
	}
	ret = smb358_read_reg(smb358_dev, STATUS_C_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STATUS_C_REG reg\n", __FUNCTION__);
	}
	reg = reg & RECHARGER_FLAG;

	/* BSP Clay: Capacity <= 98% and Battery status had been changed to FULL*/
	if (cap <= 98 && reg > 0){
		ret = smb358_charging_toggle_zd550_ze600(JEITA, false);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to stop charging, ret = %d\n", __FUNCTION__, ret);
		}
		ret = smb358_charging_toggle_zd550_ze600(JEITA, true);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to start charging, ret = %d\n", __FUNCTION__, ret);
		}
	}
	return 0;
}

int smb358_soc_control_jeita(void)
{
	int ret;

	if (!smb358_dev) {
		pr_err("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;

	/* write 0bh[5:4]="11" */
	ret = smb358_masked_write(smb358_dev, HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG, HARD_LIMIT_HOT_CELL_TEMP_MASK,
						0xff);
	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	/* write 07h[1:0]="00" */
	ret = smb358_masked_write(smb358_dev, THERM_A_CTRL_REG, CFG_THERM_SOFT_HOT_COMPENSATION_MASK,
						0);
	if (ret) {
		pr_err("fail to set CFG_THERM_SOFT_HOT_COMPENSATION_MASK ret=%d\n", ret);
		return ret;
	}
	/* write 07h[3:2]="00" */
	ret = smb358_masked_write(smb358_dev, THERM_A_CTRL_REG, SOFT_LIMIT_COLD_CELL_TEMP_MASK,
						0);
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_COLD_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	return ret;
}

#if defined(ASUS_FACTORY_BUILD)
static bool asus_battery_charging_limit(void)
{
	//int recharging_soc = 59;
	//int discharging_soc = 60;
	int percentage;
	int recharging_soc=charger_limit_setting-1;
	int discharging_soc=charger_limit_setting;

	BAT_DBG("%s,charger_limit_setting=%d\n",__FUNCTION__,charger_limit_setting);
	if (get_battery_rsoc(&percentage)) {
		BAT_DBG_E(" %s: * fail to get battery rsoc *\n", __func__);
		g_charging_toggle_for_charging_limit = true;
		charger_suspend_for_charging_limit=false;
	} else{
		if (eng_charging_limit) {
			/*BSP david: enable charging when soc <= recharging soc*/
			if (percentage <= recharging_soc) {
				BAT_DBG("%s, soc: %d <= recharging soc: %d , enable charging\n", __FUNCTION__, percentage, recharging_soc);
				g_charging_toggle_for_charging_limit = true;
				charger_suspend_for_charging_limit=false;				
				BAT_DBG("%s, soc: %d <= recharging soc: %d , charger not suspend\n",
					__FUNCTION__, percentage, recharging_soc);
			/*BSP david: disable charging when soc >= discharging soc*/
			} else if (percentage >= discharging_soc) {
				BAT_DBG("%s, soc: %d >= discharging soc: %d , disable charging\n", __FUNCTION__, percentage, discharging_soc);
				g_charging_toggle_for_charging_limit = false;
				if(percentage==discharging_soc){	
			      charger_suspend_for_charging_limit=false;	
			      BAT_DBG("%s, soc: %d == discharging soc: %d , charger not suspend\n", __FUNCTION__, percentage, discharging_soc);	
			       }else{	
			       charger_suspend_for_charging_limit=true;	
			       BAT_DBG("%s, soc: %d > discharging soc: %d , charger suspend\n", __FUNCTION__, percentage, discharging_soc);	
			       }
			} else{
				BAT_DBG("%s, soc: %d, between %d and %d, maintain original charging toggle:%d\n", __FUNCTION__, percentage, recharging_soc, discharging_soc, g_charging_toggle_for_charging_limit);
			}
		} else{
			BAT_DBG("%s, charging limit disable, enable charging!\n", __FUNCTION__);
			g_charging_toggle_for_charging_limit = true;
			charger_suspend_for_charging_limit = false;
			BAT_DBG("%s, charging limit disable, charger not suspend!\n", __FUNCTION__);
		}
	}
	return g_charging_toggle_for_charging_limit;
}
#endif

static int __smb358_path_suspend(struct smb358_charger *chip,bool suspend)
{
	int rc;

	rc= smb358_enable_volatile_writes(smb358_dev);
	if (rc < 0)
		return rc;

	rc=smb358_masked_write(chip,CMD_A_REG,CMD_A_CHG_SUSP_EN_MASK,suspend ? CMD_A_CHG_SUSP_EN_BIT : 0);

	if(rc<0)
		dev_err(chip->dev,"couldn't set CMD_A reg ,rc=%d\n",rc);
	return rc;
}


int smb358_charger_control_jeita(void)
{
	int ret;
	bool charging_enable = true;
	BAT_DBG("%s:+++\n", __FUNCTION__);

	if (!smb358_dev) {
		BAT_DBG_E("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}
	ret = smb358_enable_volatile_writes(smb358_dev);
	if (ret < 0)
		return ret;

	/* write 0bh[5:4]= "00" */
	ret = smb358_masked_write(smb358_dev,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			HARD_LIMIT_HOT_CELL_TEMP_MASK,
			0x00);
	if (ret) {
		BAT_DBG_E("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}

	/* write 07h[1:0]="10" */
	ret = smb358_masked_write(smb358_dev,
			THERM_A_CTRL_REG,
			CFG_THERM_SOFT_HOT_COMPENSATION_MASK,
			0x02);
	if (ret) {
		BAT_DBG_E("fail to set Soft Hot Limit Behavior to Float Voltage Compensation ret=%d\n", ret);
		return ret;
	}

	/* write 07h[3:2]="01" */
	ret = smb358_masked_write(smb358_dev,
			THERM_A_CTRL_REG,
			SOFT_LIMIT_COLD_CELL_TEMP_MASK,
			0x04);
	if (ret) {
		BAT_DBG_E("fail to set Soft Cold Temp Limit to Charge Current Compensation ret=%d\n", ret);
		return ret;
	}

#if defined(ASUS_FACTORY_BUILD)
	/*BSP david: do 5060 when FACTORY BUILD defined*/
	__smb358_path_suspend(smb358_dev,charger_suspend_for_charging_limit);
	if (charging_enable) {
		charging_enable = asus_battery_charging_limit();
		}
	else{
		BAT_DBG_E("%s: charging disabled by JEITA!\n", __FUNCTION__);
		}
#endif

	if(asus_PRJ_ID==ASUS_ZD550KL||asus_PRJ_ID==ASUS_ZE600KL){
                ret = smb358_charging_toggle_zd550_ze600(JEITA, charging_enable);
    		}else if(asus_PRJ_ID==ASUS_ZE550KL){
    			ret = smb358_charging_toggle_ze550(JEITA, charging_enable);
	}else {
                ret = smb358_charging_toggle(JEITA, charging_enable);
        }
        if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		return ret;
	}
	/*Set recharge voltage*/
	/* write 01h[3:2]="01"*/
	ret = smb358_masked_write(smb358_dev,
			CHG_OTH_CURRENT_CTRL_REG,
			RECHARGE_VOLTAGE_MASK,
			SMB358_VFLT_100mA);
	if (ret) {
		BAT_DBG_E("fail to set RECHARGE_VOLTAGE_MASK ret=%d\n", ret);
		return ret;
	}

	BAT_DBG("%s:---\n", __FUNCTION__);

	return ret;
};
//<asus-guc20150427+> The enum below will no longer be used.
/*enum JEITA_state {
	JEITA_STATE_INITIAL,
	JEITA_STATE_LESS_THAN_15,
	JEITA_STATE_RANGE_15_to_100,
	JEITA_STATE_RANGE_100_to_200,
	JEITA_STATE_RANGE_200_to_500,
	JEITA_STATE_RANGE_500_to_600,
	JEITA_STATE_LARGER_THAN_600,
};*/

/*battery's temperature in the range of... */
enum JEITA_state_for_all {
	JEITA_STATE_INITIAL,	// Intitial state shall not be changed
	JEITA_STATE_RANGE_01,	// It denotes either below 10 or below 15, depends on the project
	JEITA_STATE_RANGE_02,	// It denotes either between 10 to 100 or beteen 15 to 100, depends on the project
	JEITA_STATE_RANGE_03,	// It denotes between 100 to 200
	JEITA_STATE_RANGE_04,	// It denotes between 200 to 500
	JEITA_STATE_RANGE_05,	// It denotes between 500 to 600
	JEITA_STATE_RANGE_06,	// It denotes beyond 600
};
//<asus-guc20150427->

int smb358_JEITA_judge_state(int old_State, int batt_tempr)
{
	int result_State;

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	/*batt_tempr < 1.5*/
	if (batt_tempr < charger_factor->batt_tempr_00) {				//<asus-guc20150427> 15
		result_State = JEITA_STATE_RANGE_01;						//<asus-guc20150427> JEITA_STATE_LESS_THAN_15
	/*1.5 <= batt_tempr < 10*/
	} else if (batt_tempr < charger_factor->batt_tempr_01) {		//<asus-guc20150427> 100
		result_State = JEITA_STATE_RANGE_02;						//<asus-guc20150427> JEITA_STATE_RANGE_15_to_100
	/*10 <= batt_tempr < 20*/
	} else if (batt_tempr < charger_factor->batt_tempr_02) {		//<asus-guc20150427> 200
		result_State = JEITA_STATE_RANGE_03;						//<asus-guc20150427> JEITA_STATE_RANGE_100_to_200
	/*20 <= batt_tempr < 50*/
	} else if (batt_tempr < charger_factor->batt_tempr_03) {		//<asus-guc20150427> 500
		result_State = JEITA_STATE_RANGE_04;						//<asus-guc20150427> JEITA_STATE_RANGE_200_to_500
	/*50 <= batt_tempr < 60*/
	} else if (batt_tempr < charger_factor->batt_tempr_04) {		//<asus-guc20150427> 600
		result_State = JEITA_STATE_RANGE_05;						//<asus-guc20150427> JEITA_STATE_RANGE_500_to_600
	/*60 <= batt_tempr*/
	} else{
		result_State = JEITA_STATE_RANGE_06;						//<asus-guc20150427> JEITA_STATE_LARGER_THAN_600
	}

	/*BSP david: do 3 degree hysteresis*/
	if (old_State == JEITA_STATE_RANGE_01 && result_State == JEITA_STATE_RANGE_02) {
		if (batt_tempr <= charger_factor->batt_tempr_00_hyst30) {	//<asus-guc20150427> 45
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_02 && result_State == JEITA_STATE_RANGE_03) {
		if (batt_tempr <= charger_factor->batt_tempr_01_hyst30) {	//<asus-guc20150427> 130
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_03 && result_State == JEITA_STATE_RANGE_04) {
		if (batt_tempr <= charger_factor->batt_tempr_02_hyst30) {	//<asus-guc20150427> 230
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_05 && result_State == JEITA_STATE_RANGE_04) {
		if (batt_tempr >= charger_factor->batt_tempr_03_hyst30) {	//<asus-guc20150427> 470
			result_State = old_State;
		}
	}
	if (old_State == JEITA_STATE_RANGE_06 && result_State == JEITA_STATE_RANGE_05) {
		if (batt_tempr >= charger_factor->batt_tempr_04_hyst30) {	//<asus-guc20150427> 570
			result_State = old_State;
		}
	}
	return result_State;
}
static int smb358_soc_detect_batt_tempr(int usb_state)
{
	int ret;
	u8 reg;
	static int state = JEITA_STATE_INITIAL;
	int fast_chg_reg_value = 0;/*set fast chg reg value*/
	int Vchg_reg_value = 0;/*set Vchg reg value*/
	bool charging_enable = false;
	int batt_tempr = 250;/* unit: C  */
	int batt_volt = 4000;/* unit: mV */
	int chg_volt_reg = 0;/*read reg value*/
	bool Vrech = false; /* recharge set */
	int usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;/*unit: mA*/
	int thermal_policy_ic=0;
	int raw_cap=0; //add by kerwin for fast chager JEITA_flag
	int aicl_time=60;//schedule time
	struct power_supply *psy;
	if (!smb358_is_charging(usb_state))
		return aicl_time;
	/*BSP david: do soc control jeita setting*/
	smb358_soc_control_jeita();
	
	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __FUNCTION__);
		return aicl_time;
	} else{
	}

	/*acquire battery capability here 20150507*/
	ret = get_bms_calculated_soc(&raw_cap);
	if (ret < 0) {
		BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
		return aicl_time;
	}

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __FUNCTION__);
		return aicl_time;
	} else{
	}
	
	/* Vchg here */
	ret = smb358_read_reg(smb358_dev, VFLOAT_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
		return aicl_time;
	}

	chg_volt_reg = reg & FLOAT_VOLTAGE_MASK;
	BAT_DBG("%s: battery temperature: %d, battery voltage: %d, Vchg: %x,raw=%d\n", __FUNCTION__, batt_tempr, batt_volt, chg_volt_reg,raw_cap);

	/*BSP david: judge current jeita state, do 3 degree hysteresis*/
	state = smb358_JEITA_judge_state(state, batt_tempr);

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	switch (state) {
	case JEITA_STATE_RANGE_01:
		charging_enable = false;
		usb_in_current=0;
		fast_chg_reg_value = charger_factor->g_Iqc_RANGE_01;	//<asus-guc20150427> SMB358_FAST_CHG_CURRENT_VALUE_600mA
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00);
		break;
	case JEITA_STATE_RANGE_02:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_600mA;
		usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
		JEITA_Flag=true;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00, charger_factor->batt_tempr_01);
		smb358_set_recharge_voltage();
		break;
	case JEITA_STATE_RANGE_03:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
		JEITA_Flag=true;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_01, charger_factor->batt_tempr_02);
		smb358_set_recharge_voltage();
		break;
	case JEITA_STATE_RANGE_04:
		charging_enable = true;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d with fast_chg_flag = %d\n", __FUNCTION__, charger_factor->batt_tempr_02, charger_factor->batt_tempr_03, fast_chg_flag ? 1 : 0);
		if (fast_chg_flag) {
			if ((batt_volt < 4150) && (raw_cap <70)) { //add rsoc avoid wrong 5W detection
				fast_chg_flag = false;
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;				
				if(JEITA_Flag){
					usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
					JEITA_Flag=false;
				}
			} else{
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
				usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
				JEITA_Flag=true;
			}
		} else{
			if ((batt_volt < 4250) && (raw_cap <70)) {  //add rsoc avoid wrong 5W detection
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;				
				if(JEITA_Flag){
					usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
					JEITA_Flag=false;
				}
				
			} else{
				fast_chg_flag = true;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
				usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
				JEITA_Flag=true;
			}
		}
		smb358_set_recharge_voltage();
		break;
	case JEITA_STATE_RANGE_05:
		if (chg_volt_reg == VCH_VALUE && batt_volt >= 4110) {
			charging_enable = false;
			usb_in_current=0;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = VCH_VALUE;
			BAT_DBG("%s: %d <= temperature < %d, Vchg == 4.38 && Vbat >= 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		} else{
			charging_enable = true;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4100mV;
			JEITA_Flag=true;
			usb_in_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
			Vrech = true;
			BAT_DBG("%s: %d <= temperature < %d, Vchg != 4.38 || Vbat < 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		}
		break;
	case JEITA_STATE_RANGE_06:
		charging_enable = false;
		usb_in_current=0;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = charger_factor->VCH_VALUE_RANGE_06;	//<asus-guc20150427> SMB358_FLOAT_VOLTAGE_VALUE_4140mV
		BAT_DBG("%s: %d <= temperature\n", __FUNCTION__, charger_factor->batt_tempr_04);
		break;
	default:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
		Vchg_reg_value = VCH_VALUE;
		JEITA_Flag=true;
		BAT_DBG("%s: wrong state!\n", __FUNCTION__);
		break;
	}
	
#if defined(ASUS_FACTORY_BUILD)
	/*BSP david: do 5060 when FACTORY BUILD defined*/
	if (charging_enable) {
		charging_enable = asus_battery_charging_limit();
	} else{
		BAT_DBG_E("%s: charging disabled by JEITA!\n", __FUNCTION__);
	}
#endif
	/*do set reg value after depend on above decision*/	
	/*Set Vchg*/
	ret = smb358_masked_write(smb358_dev, VFLOAT_REG, FLOAT_VOLTAGE_MASK,
						Vchg_reg_value);
	if (ret) {
		pr_err("fail to set Vchg ret=%d\n", ret);
		goto finish;
	}
	/*Set Fast charge current*/
	ret = smb358_masked_write(smb358_dev, CHG_CURRENT_CTRL_REG, SMB358_FAST_CHG_CURRENT_MASK,
						fast_chg_reg_value);
	if (ret) {
		pr_err("fail to set FAST_CHG_CURRENT ret=%d\n", ret);
		goto finish;
	}

	/*set charger current limit by kerwin*/
	//usb_in_current=0 disable charging
	BAT_DBG("%s: JEITA_IC=%d, JEITA_flag=%d, fast_chg_flag=%d\n",
							__FUNCTION__,usb_in_current,JEITA_Flag, fast_chg_flag);
	if(g_usb_state == AC_IN && usb_in_current != 0){
			//get thermal policy current set, compare and chose the small value
	ret = Thermal_Policy(&thermal_policy_ic);
	if(ret < 0){
			  BAT_DBG_E(" %s: fail to set current according to Thermal Policy\n", __FUNCTION__);
	}
				ret = smb358_read_reg(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, &reg);
				if (ret < 0) {
								BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
								return aicl_time;
				}
				reg=reg & CFG_CURRENT_LIMIT_SMB358_MASK;
				BAT_DBG("%s: Thermal_Level=%d, TP_IC=%d, JEITA_IC=%d, CURRENT_LIMIT_READ=%d, JEITA_flag=%d, fast_chg_flag=%d\n",
								__FUNCTION__, Thermal_Level, thermal_policy_ic,usb_in_current,reg,JEITA_Flag, fast_chg_flag);
				if(thermal_policy_ic !=0){
						if(thermal_policy_ic < usb_in_current)
										usb_in_current=thermal_policy_ic;
				}
				if(JEITA_Flag){ 
					if(reg != usb_in_current){
								printk("%s is in JEITA_flag=1\n",__FUNCTION__);
								if(reg < usb_in_current){ /* from small to big_Input_Current */
										smb358_OptiCharge_Toggle(false);
										ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
										usb_in_current);
										if (ret) {
												pr_err("fail to set current limit ret=%d\n", ret);
												goto finish;
										}
										smb358_OptiCharge_Toggle(true);
								}else {
										ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
										usb_in_current);
										if (ret) {
												pr_err("fail to set current limit ret=%d\n", ret);
												goto finish;
										}
								}
								
						}
				}else{//JEITA_flag false usb_in limit =2000mA
						if(reg != usb_in_current){		
								printk("%s is in JEITA_flag=0\n",__FUNCTION__);
								smb358_OptiCharge_Toggle(false);
						ret=smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
							 usb_in_current);
								if (ret) {
										pr_err("fail to set current limit ret=%d\n", ret);
										goto finish;
								}
								aicl_time=5;
						smb358_OptiCharge_Toggle(true);
						//smb358_update_aicl_work(5);
						//wake_lock_timeout(&inok_wakelock, 5 * HZ);
						}
				}
		}
	
	/*set enable charging reg*/
	ret = smb358_charging_toggle(JEITA, charging_enable);
	if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		goto finish;
	}
	/*Set Vrech=Vflt-100mv*/
	if (Vrech){
		ret = smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, RECHARGE_VOLTAGE_MASK,
						SMB358_RECHARGE_VRECH_VOLTAGE);
		if (ret) {
			pr_err("fail to set Vrech ret=%d\n", ret);
			goto finish;
		}
	}
	
finish:
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);		
	}
	printk("%s aicl_time=%d\n",__FUNCTION__,aicl_time);
	return aicl_time;
}

static int smb358_soc_detect_batt_tempr_ze550(int usb_state)
{
	int ret;
	u8 reg;
	static int state = JEITA_STATE_INITIAL;
	int fast_chg_reg_value = 0;/*set fast chg reg value*/
	int Vchg_reg_value = 0;/*set Vchg reg value*/
	bool charging_enable = false;
	int batt_tempr = 250;/* unit: C  */
	int batt_volt = 4000;/* unit: mV */
	int chg_volt_reg = 0;/*read reg value*/
	//bool Vrech = false; /* recharge set */
	int thermal_policy_ic=0;
	int raw_cap=0; //add by kerwin for fast chager JEITA_flag
	int aicl_time=60;//schedule time
	struct power_supply *psy;
	if (!smb358_is_charging(usb_state))
		return aicl_time;
	/*BSP david: do soc control jeita setting*/
	smb358_soc_control_jeita();
	
	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __FUNCTION__);
		return aicl_time;
	} else{
	}

	/*acquire battery capability here 20150507*/
	ret = get_bms_calculated_soc(&raw_cap);
	if (ret < 0) {
		BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
		return aicl_time;
	}

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __FUNCTION__);
		return aicl_time;
	} else{
	}
	
	/* Vchg here */
	ret = smb358_read_reg(smb358_dev, VFLOAT_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
		return aicl_time;
	}

	chg_volt_reg = reg & FLOAT_VOLTAGE_MASK;
	BAT_DBG("%s: battery temperature: %d, battery voltage: %d, Vchg: %x,raw=%d\n", __FUNCTION__, batt_tempr, batt_volt, chg_volt_reg,raw_cap);

	/*BSP david: judge current jeita state, do 3 degree hysteresis*/
	state = smb358_JEITA_judge_state(state, batt_tempr);

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	switch (state) {
	case JEITA_STATE_RANGE_01:
		charging_enable = false;
		fast_chg_reg_value = charger_factor->g_Iqc_RANGE_01;	//<asus-guc20150427> SMB358_FAST_CHG_CURRENT_VALUE_600mA
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00);
		break;
	case JEITA_STATE_RANGE_02:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_600mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00, charger_factor->batt_tempr_01);
		smb358_set_recharge_voltage_ze550();
		break;
	case JEITA_STATE_RANGE_03:
	case JEITA_STATE_RANGE_04:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_01, charger_factor->batt_tempr_03);
		smb358_set_recharge_voltage_ze550();
		break;
	case JEITA_STATE_RANGE_05:
		if (chg_volt_reg == VCH_VALUE && batt_volt >= 4110) {
			charging_enable = false;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = VCH_VALUE;
			BAT_DBG("%s: %d <= temperature < %d, Vchg == 4.38 && Vbat >= 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		} else{
			charging_enable = true;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4100mV;
			//Vrech = true;
			BAT_DBG("%s: %d <= temperature < %d, Vchg != 4.38 || Vbat < 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		}
		break;
	case JEITA_STATE_RANGE_06:
		charging_enable = false;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = charger_factor->VCH_VALUE_RANGE_06;	//<asus-guc20150427> SMB358_FLOAT_VOLTAGE_VALUE_4140mV
		BAT_DBG("%s: %d <= temperature\n", __FUNCTION__, charger_factor->batt_tempr_04);
		break;
	default:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: wrong state!\n", __FUNCTION__);
		break;
	}
	
#if defined(ASUS_FACTORY_BUILD)
	/*BSP david: do 5060 when FACTORY BUILD defined*/
	if (charging_enable) {
		charging_enable = asus_battery_charging_limit();
	} else{
		BAT_DBG_E("%s: charging disabled by JEITA!\n", __FUNCTION__);
	}
#endif
	/*do set reg value after depend on above decision*/	
	/*Set Vchg*/
	ret = smb358_masked_write(smb358_dev, VFLOAT_REG, FLOAT_VOLTAGE_MASK,
						Vchg_reg_value);
	if (ret) {
		pr_err("fail to set Vchg ret=%d\n", ret);
		goto finish;
	}
	/*Set Fast charge current*/
	ret = smb358_masked_write(smb358_dev, CHG_CURRENT_CTRL_REG, SMB358_FAST_CHG_CURRENT_MASK,
						fast_chg_reg_value);
	if (ret) {
		pr_err("fail to set FAST_CHG_CURRENT ret=%d\n", ret);
		goto finish;
	}

	/*set charger current limit by kerwin*/
	if(g_usb_state == AC_IN){
		ret = Thermal_Policy_ze550(&thermal_policy_ic);
		if(ret < 0){
			  BAT_DBG_E(" %s: fail to set current according to Thermal Policy\n", __FUNCTION__);
		}
		ret = smb358_read_reg(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
			return aicl_time;
		}
		reg=reg & CFG_CURRENT_LIMIT_SMB358_MASK;
		BAT_DBG("%s: Thermal_Level=%d, TP_IC=%d, CURRENT_LIMIT_READ=%d\n",
								__FUNCTION__, Thermal_Level, thermal_policy_ic,reg);

		if(reg != thermal_policy_ic){
			if(Thermal_Level !=3){	 //avoid Thermal_Level !=3	to set usb_in limit=0mA
				if(thermal_policy_ic!=0){
						if(reg < thermal_policy_ic){ /* from small to big_Input_Current */
									smb358_OptiCharge_Toggle(false);
									ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
												thermal_policy_ic);
									if (ret) {
											pr_err("fail to set current limit ret=%d\n", ret);
											goto finish;
									}
									smb358_OptiCharge_Toggle(true);
						}else {
									ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
												thermal_policy_ic);
									if (ret) {
											pr_err("fail to set current limit ret=%d\n", ret);
											goto finish;
									}
						}
				}
			}else{
						if(reg != thermal_policy_ic){		
								smb358_OptiCharge_Toggle(false);
								ret=smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
							 			thermal_policy_ic);
								if (ret) {
										pr_err("fail to set current limit ret=%d\n", ret);
										goto finish;
								}
								aicl_time=60;
								smb358_OptiCharge_Toggle(true);
						}
			}
		}
	}
	
	/*set enable charging reg*/
	ret = smb358_charging_toggle_ze550(JEITA, charging_enable);
	if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		goto finish;
	}
	/*Set Vrech=Vflt-100mv*/
	/*if (Vrech){
		ret = smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, RECHARGE_VOLTAGE_MASK,
						SMB358_RECHARGE_VRECH_VOLTAGE);
		if (ret) {
			pr_err("fail to set Vrech ret=%d\n", ret);
			goto finish;
		}
	}*/
	
finish:
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);		
	}
	printk("%s aicl_time=%d\n",__FUNCTION__,aicl_time);
	return aicl_time;
}


static int smb358_soc_detect_batt_tempr_zd550_ze600(int usb_state)
{
	int ret;
	u8 reg;
	static int state = JEITA_STATE_INITIAL;
	int fast_chg_reg_value = 0;/*set fast chg reg value*/
	int Vchg_reg_value = 0;/*set Vchg reg value*/
	bool charging_enable = false;
	bool recharge_enable = false;
	int batt_tempr = 250;/* unit: C  */
	int batt_volt = 4000;/* unit: mV */
	int chg_volt_reg = 0;/*read reg value*/
	bool Vrech = false; /* recharge set */
	int raw_cap=0; //add by kerwin for fast chager JEITA_flag
	int aicl_time=60;//schedule time
	int thermal_policy_ic=0;
	struct power_supply *psy;
	if (!smb358_is_charging(usb_state))
		return aicl_time;
	/*BSP david: do soc control jeita setting*/
	smb358_soc_control_jeita();
	
	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __FUNCTION__);
		return aicl_time;
	} else{
	}

	/*acquire battery capability here 20150507*/
	ret = get_bms_calculated_soc(&raw_cap);
	if (ret < 0) {
		BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
		return aicl_time;
	}

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __FUNCTION__);
		return aicl_time;
	} else{
	}
	
	/* Vchg here */
	ret = smb358_read_reg(smb358_dev, VFLOAT_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
		return aicl_time;
	}

	chg_volt_reg = reg & FLOAT_VOLTAGE_MASK;
	BAT_DBG("%s: battery temperature: %d, battery voltage: %d, Vchg: %x,raw=%d\n", __FUNCTION__, batt_tempr, batt_volt, chg_volt_reg,raw_cap);

	/*BSP david: judge current jeita state, do 3 degree hysteresis*/
	state = smb358_JEITA_judge_state(state, batt_tempr);

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	switch (state) {
	case JEITA_STATE_RANGE_01:
		charging_enable = false;
		fast_chg_reg_value = charger_factor->g_Iqc_RANGE_01;	//<asus-guc20150427> SMB358_FAST_CHG_CURRENT_VALUE_600mA
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00);
		break;
	case JEITA_STATE_RANGE_02:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_600mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00, charger_factor->batt_tempr_01);
	        recharge_enable = true;
		break;
	case JEITA_STATE_RANGE_03:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_01, charger_factor->batt_tempr_02);
	        recharge_enable = true;
		break;
	case JEITA_STATE_RANGE_04:
		charging_enable = true;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d with fast_chg_flag = %d\n", __FUNCTION__, charger_factor->batt_tempr_02, charger_factor->batt_tempr_03, fast_chg_flag ? 1 : 0);
		if (fast_chg_flag) {
			if (batt_volt < 4100) {
				fast_chg_flag = false;
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;				
			} else{
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			}
		} else{
			if (batt_volt < 4250) {
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;				
				
			} else{
				fast_chg_flag = true;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			}
		}
	        recharge_enable = true;
		break;
	case JEITA_STATE_RANGE_05:
		if (chg_volt_reg == VCH_VALUE && batt_volt >= 4110) {
			charging_enable = false;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = VCH_VALUE;
			BAT_DBG("%s: %d <= temperature < %d, Vchg == 4.38 && Vbat >= 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		} else{
			charging_enable = true;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
			Vchg_reg_value = SMB358_FLOAT_VOLTAGE_VALUE_4100mV;
			Vrech = true;
			BAT_DBG("%s: %d <= temperature < %d, Vchg != 4.38 || Vbat < 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		}
		break;
	case JEITA_STATE_RANGE_06:
		charging_enable = false;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_1300mA;
		Vchg_reg_value = charger_factor->VCH_VALUE_RANGE_06;	//<asus-guc20150427> SMB358_FLOAT_VOLTAGE_VALUE_4140mV
		BAT_DBG("%s: %d <= temperature\n", __FUNCTION__, charger_factor->batt_tempr_04);
		break;
	default:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: wrong state!\n", __FUNCTION__);
		break;
	}
	
#if defined(ASUS_FACTORY_BUILD)
	/*BSP david: do 5060 when FACTORY BUILD defined*/
	if (charging_enable) {
		charging_enable = asus_battery_charging_limit();
	} else{
		BAT_DBG_E("%s: charging disabled by JEITA!\n", __FUNCTION__);
	}
#endif
	/*do set reg value after depend on above decision*/	
	/*Set Vchg*/
	ret = smb358_masked_write(smb358_dev, VFLOAT_REG, FLOAT_VOLTAGE_MASK,
						Vchg_reg_value);
	if (ret) {
		pr_err("fail to set Vchg ret=%d\n", ret);
		goto finish;
	}
	/*Set Fast charge current*/
	ret = smb358_masked_write(smb358_dev, CHG_CURRENT_CTRL_REG, SMB358_FAST_CHG_CURRENT_MASK,
						fast_chg_reg_value);
	if (ret) {
		pr_err("fail to set FAST_CHG_CURRENT ret=%d\n", ret);
		goto finish;
	}

	/*set charger current limit by kerwin*/
	if(g_usb_state == AC_IN ){
	    ret = Thermal_Policy(&thermal_policy_ic);
		if(ret < 0){
			BAT_DBG_E(" %s: fail to set current according to Thermal Policy\n", __FUNCTION__);
		}
		ret = smb358_read_reg(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
			return aicl_time;
		}
		reg = reg & CFG_CURRENT_LIMIT_SMB358_MASK;
		BAT_DBG("%s: Thermal_Level=%d, TP_IC=%d, CURRENT_LIMIT_READ=%d, fast_chg_flag=%d\n",
					__FUNCTION__, Thermal_Level, thermal_policy_ic, reg, fast_chg_flag);

        if(reg != thermal_policy_ic){
	        if(Thermal_Level !=3){	 //avoid Thermal_Level !=3	to set usb_in limit=0mA
			if(thermal_policy_ic!=0){
				if(reg < thermal_policy_ic){ /* from small to big_Input_Currt */
					smb358_OptiCharge_Toggle(false);
					ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,
                                                                        CFG_CURRENT_LIMIT_SMB358_MASK,thermal_policy_ic);
					if(ret){
					        pr_err("fail to set current limit ret=%d\n", ret);
						goto finish;
					}
						smb358_OptiCharge_Toggle(true);
					}else{
					        ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
												thermal_policy_ic);
					        if(ret){
						        pr_err("fail to set current limit ret=%d\n", ret);
					                goto finish;
						}
					}
				}
			}else{
			        if(reg != thermal_policy_ic){
					smb358_OptiCharge_Toggle(false);
					ret=smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
							 	thermal_policy_ic);
					if (ret) {
					        pr_err("fail to set current limit ret=%d\n", ret);
						goto finish;
					}
					aicl_time=60;
					smb358_OptiCharge_Toggle(true);
				}
			}
		}
	}
	
	/*set enable charging reg*/
	ret = smb358_charging_toggle_zd550_ze600(JEITA, charging_enable);
	if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		goto finish;
	}
	/*Set Vrech=Vflt-100mv*/
	if (Vrech){
		ret = smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, RECHARGE_VOLTAGE_MASK,
						SMB358_RECHARGE_VRECH_VOLTAGE);
		if (ret) {
			pr_err("fail to set Vrech ret=%d\n", ret);
			goto finish;
		}
	}
        if(recharge_enable){
		smb358_set_recharge_voltage_zd550_ze600();
		BAT_DBG("%s: smb358_set_recharge_voltage_zd550_ze600e!\n", __FUNCTION__);
        }
	
finish:
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);		
	}
	printk("%s aicl_time=%d\n",__FUNCTION__,aicl_time);
	return aicl_time;
}

static int smb358_soc_detect_batt_tempr_5wBZsku_zd550(int usb_state)
{
	int ret;
	u8 reg;
	static int state = JEITA_STATE_INITIAL;
	int fast_chg_reg_value = 0;/*set fast chg reg value*/
	int Vchg_reg_value = 0;/*set Vchg reg value*/
	bool charging_enable = false;
	bool recharge_enable = false;
	int batt_tempr = 250;/* unit: C  */
	int batt_volt = 4000;/* unit: mV */
	int chg_volt_reg = 0;/*read reg value*/
	bool Vrech = false; /* recharge set */
	int raw_cap=0; //add by kerwin for fast chager JEITA_flag
	int aicl_time=60;//schedule time
	int thermal_policy_ic=0;
	struct power_supply *psy;
	if (!smb358_is_charging(usb_state))
		return aicl_time;
	/*BSP david: do soc control jeita setting*/
	smb358_soc_control_jeita();
	
	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __FUNCTION__);
		return aicl_time;
	} else{
	}

	/*acquire battery capability here 20150507*/
	ret = get_bms_calculated_soc(&raw_cap);
	if (ret < 0) {
		BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
		return aicl_time;
	}

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __FUNCTION__);
		return aicl_time;
	} else{
	}
	
	/* Vchg here */
	ret = smb358_read_reg(smb358_dev, VFLOAT_REG, &reg);
	if (ret < 0) {
		BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
		return aicl_time;
	}

	chg_volt_reg = reg & FLOAT_VOLTAGE_MASK;
	BAT_DBG("%s: battery temperature: %d, battery voltage: %d, Vchg: %x,raw=%d\n", __FUNCTION__, batt_tempr, batt_volt, chg_volt_reg,raw_cap);

	/*BSP david: judge current jeita state, do 3 degree hysteresis*/
	state = smb358_JEITA_judge_state(state, batt_tempr);

	/*decide value to set each reg (Vchg, Charging enable, Fast charge current)*/
	switch (state) {
	case JEITA_STATE_RANGE_01:
		charging_enable = false;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;  
                Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00);
		break;
	case JEITA_STATE_RANGE_02:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_600mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_00, charger_factor->batt_tempr_01);
	        recharge_enable = true;
		break;
	case JEITA_STATE_RANGE_03:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d\n", __FUNCTION__, charger_factor->batt_tempr_01, charger_factor->batt_tempr_02);
	        recharge_enable = true;
		break;
	case JEITA_STATE_RANGE_04:
		charging_enable = true;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: %d <= temperature < %d with fast_chg_flag = %d\n", __FUNCTION__, charger_factor->batt_tempr_02, charger_factor->batt_tempr_03, fast_chg_flag ? 1 : 0);
		if (fast_chg_flag) {
			if (batt_volt < 4100) {
				fast_chg_flag = false;
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;				
			} else{
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
			}
		} else{
			if (batt_volt < 4250) {
				//g_Iqc = SMB358_FAST_CHG_CURRENT_VALUE_2000mA;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;				
				
			} else{
				fast_chg_flag = true;
				fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
			}
		}
	        recharge_enable = true;
		break;
	case JEITA_STATE_RANGE_05:
		if (chg_volt_reg == VCH_VALUE && batt_volt >= 4110) {
			charging_enable = false;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
			Vchg_reg_value = VCH_VALUE;
			BAT_DBG("%s: %d <= temperature < %d, Vchg == 4.38 && Vbat >= 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		} else{
			charging_enable = true;
			fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
			BAT_DBG("%s: %d <= temperature < %d, Vchg != 4.38 || Vbat < 4.11\n", __FUNCTION__, charger_factor->batt_tempr_03, charger_factor->batt_tempr_04);
		}
		break;
	case JEITA_STATE_RANGE_06:
		charging_enable = false;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
		Vchg_reg_value = charger_factor->VCH_VALUE_RANGE_06;	//<asus-guc20150427> SMB358_FLOAT_VOLTAGE_VALUE_4140mV
		BAT_DBG("%s: %d <= temperature\n", __FUNCTION__, charger_factor->batt_tempr_04);
		break;
	default:
		charging_enable = true;
		fast_chg_reg_value = SMB358_FAST_CHG_CURRENT_VALUE_900mA;
		Vchg_reg_value = VCH_VALUE;
		BAT_DBG("%s: wrong state!\n", __FUNCTION__);
		break;
	}
	
#if defined(ASUS_FACTORY_BUILD)
	/*BSP david: do 5060 when FACTORY BUILD defined*/
	if (charging_enable) {
		charging_enable = asus_battery_charging_limit();
	} else{
		BAT_DBG_E("%s: charging disabled by JEITA!\n", __FUNCTION__);
	}
#endif
	/*do set reg value after depend on above decision*/	
	/*Set Vchg*/
	ret = smb358_masked_write(smb358_dev, VFLOAT_REG, FLOAT_VOLTAGE_MASK,
						Vchg_reg_value);
	if (ret) {
		pr_err("fail to set Vchg ret=%d\n", ret);
		goto finish;
	}
	/*Set Fast charge current*/
	ret = smb358_masked_write(smb358_dev, CHG_CURRENT_CTRL_REG, SMB358_FAST_CHG_CURRENT_MASK,
						fast_chg_reg_value);
	if (ret) {
		pr_err("fail to set FAST_CHG_CURRENT ret=%d\n", ret);
		goto finish;
	}

	/*set charger current limit by kerwin*/
	if(g_usb_state == AC_IN ){
	    ret = Thermal_Policy_ze550(&thermal_policy_ic);
		if(ret < 0){
			BAT_DBG_E(" %s: fail to set current according to Thermal Policy\n", __FUNCTION__);
		}
		ret = smb358_read_reg(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, &reg);
		if (ret < 0) {
			BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
			return aicl_time;
		}
		reg = reg & CFG_CURRENT_LIMIT_SMB358_MASK;
		BAT_DBG("%s: Thermal_Level=%d, TP_IC=%d, CURRENT_LIMIT_READ=%d, fast_chg_flag=%d\n",
					__FUNCTION__, Thermal_Level, thermal_policy_ic, reg, fast_chg_flag);

        if(reg != thermal_policy_ic){
	        if(Thermal_Level !=3){	 //avoid Thermal_Level !=3	to set usb_in limit=0mA
			if(thermal_policy_ic!=0){
				if(reg < thermal_policy_ic){ /* from small to big_Input_Currt */
					smb358_OptiCharge_Toggle(false);
					ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,
                                                                        CFG_CURRENT_LIMIT_SMB358_MASK,thermal_policy_ic);
					if(ret){
					        pr_err("fail to set current limit ret=%d\n", ret);
						goto finish;
					}
						smb358_OptiCharge_Toggle(true);
					}else{
					        ret = smb358_masked_write(smb358_dev,CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
												thermal_policy_ic);
					        if(ret){
						        pr_err("fail to set current limit ret=%d\n", ret);
					                goto finish;
						}
					}
				}
			}else{
			        if(reg != thermal_policy_ic){
					smb358_OptiCharge_Toggle(false);
					ret=smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG,CFG_CURRENT_LIMIT_SMB358_MASK,
							 	thermal_policy_ic);
					if (ret) {
					        pr_err("fail to set current limit ret=%d\n", ret);
						goto finish;
					}
					aicl_time=60;
					smb358_OptiCharge_Toggle(true);
				}
			}
		}
	}
	
	/*set enable charging reg*/
	ret = smb358_charging_toggle_zd550_ze600(JEITA, charging_enable);
	if (ret) {
		pr_err("fail to set charging enable ret=%d\n", ret);
		goto finish;
	}
	/*Set Vrech=Vflt-100mv*/
	if (Vrech){
		ret = smb358_masked_write(smb358_dev, CHG_OTH_CURRENT_CTRL_REG, RECHARGE_VOLTAGE_MASK,
						SMB358_RECHARGE_VRECH_VOLTAGE);
		if (ret) {
			pr_err("fail to set Vrech ret=%d\n", ret);
			goto finish;
		}
	}
        if(recharge_enable){
		smb358_set_recharge_voltage_zd550_ze600();
		BAT_DBG("%s: smb358_set_recharge_voltage_zd550_ze600e!\n", __FUNCTION__);
        }
	
finish:
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);		
	}
	printk("%s aicl_time=%d\n",__FUNCTION__,aicl_time);
	return aicl_time;
}




extern bool get_sw_charging_toggle(void)
{
	bool ret;

	mutex_lock(&g_charging_toggle_lock);
	ret = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

	return ret;
}
struct battery_info_reply {
u32 capacity;
int rawsoc;
int Rsoc;
u32 voltage_now;
int current_now;
char *temperature_negative_sign;
int temperature;
int temperature10;
u32 cable_status;
int status;
int fcc;
bool charging_toggle;
};
/*BSP david: can only used for printing, since the info may not up to date*/
struct battery_info_reply batt_info;
int asus_battery_update_status(void)
{
	int ret,cap;
	int status = POWER_SUPPLY_STATUS_UNKNOWN;
	bool charging_toggle;
	u32 cable_status; 
	cable_status = get_charger_type();
	charging_toggle = get_sw_charging_toggle();
	schedule_delayed_work(&LED_ChargerMode, 0*HZ);
		if (Batt_ID_ERROR){
			return status;
		}
		else if (smb358_is_charging(cable_status)) {
			if (g_bat_full) {
			status = POWER_SUPPLY_STATUS_FULL;
		} else if (charging_toggle) {
			ret = get_bms_calculated_soc(&cap);
			if(ret){
				BAT_DBG_E("Read Battery Capacity fail\n");
			}
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else{
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	} else{
		status = POWER_SUPPLY_STATUS_DISCHARGING;
 	}
	return status;
}
extern int get_driver_soc(void);
extern int get_vm_bms_fcc(void);
static int asus_update_all(void)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;
	int batt_temp;
	int charging_status;

	psy = get_psy_battery();
	if (!psy) {
		BAT_DBG_E("%s: can't get battery psy!\n", __FUNCTION__);
		return -EINVAL;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret) {
		batt_info.capacity = val.intval;
	} else{
		BAT_DBG_E("%s: can't get capacity, ret = %d!\n", __FUNCTION__, ret);
		batt_info.capacity = ret;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret) {
		batt_info.voltage_now = val.intval /1000;
	} else{
		BAT_DBG_E("%s: can't get voltage_now, ret = %d!\n", __FUNCTION__, ret);
		batt_info.voltage_now = ret;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret) {
		batt_info.current_now = val.intval / 1000;
	} else{
		BAT_DBG_E("%s: can't get current_now, ret = %d!\n", __FUNCTION__, ret);
		batt_info.current_now = ret;
	}
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret) {
		batt_temp = val.intval;
		if (batt_temp >= 10 || batt_temp <= -10) {
			batt_info.temperature10 = batt_temp / 10;
			batt_info.temperature = batt_temp - (batt_info.temperature10 * 10);
			if (batt_info.temperature < 0) {
				batt_info.temperature = -batt_info.temperature;
			}
			batt_info.temperature_negative_sign = "";
		} else {
			batt_info.temperature10 = 0;
			batt_info.temperature = batt_temp < 0 ? -batt_temp : batt_temp;
			if (batt_temp >= 0) {
				batt_info.temperature_negative_sign = "";
			}
		}
	} else{
		BAT_DBG_E("%s: can't get bat_temp, ret = %d!\n", __FUNCTION__, ret);
		batt_temp = ret;
	}

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_STATUS, &val);
	if (!ret) {
		batt_info.status = val.intval;
	} else{
		BAT_DBG_E("%s: can't get charging_status, ret = %d!\n", __FUNCTION__, ret);
		charging_status = ret;
	}
	ret = get_bms_calculated_soc(&batt_info.rawsoc);
	if (ret < 0) {
		BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
	}
	batt_info.cable_status = get_charger_type();
	batt_info.charging_toggle = get_sw_charging_toggle();
	batt_info.fcc = get_vm_bms_fcc();
	batt_info.Rsoc = get_driver_soc();
	return 0;
}
static struct timespec g_last_print_time;
static int asus_print_all(void)
{
	int ret;
	char chargerReg[128] = "";
	char battInfo[256];
	/*  UNKN means UNKOWN, CHRG mean CHARGING, DISC means DISCHARGING,
	    NOTC means NOT CHARGING
	*/
	char batt_status_str[5][5] = {
		"UNKN",
		"CHRG",
		"DISC",
		"NOTC",
		"FULL"
	};
	char cable_status_str[8][5] = {
		"USB",
		"AC",
		"NO",
		"EN5V",
		"DS5V",
		"PAD",
		"UNKN",
		"SE1",
	};
	ret = asus_update_all();
	if (ret < 0) {
		BAT_DBG_E("%s: can't update battery info!\n", __FUNCTION__);
		return -1;
	}

	ChargerRegDump(chargerReg, 128);
	
	snprintf(battInfo, sizeof(battInfo), "report Capacity ==>%d, FCC:%dmAh, BMS:%d, V:%dmV, Cur:%dmA, ",
		batt_info.capacity,
		batt_info.fcc,
		batt_info.capacity,
		batt_info.voltage_now,
		batt_info.current_now);
	snprintf(battInfo, sizeof(battInfo), "%sTemp:%s%d.%dC, Cable:%d(%s), Status:%s, Charging:%s, Rsoc:%d, BATID:%lld, TLevel:%d, ",
		battInfo,
		batt_info.temperature_negative_sign,
		batt_info.temperature10,
		batt_info.temperature,
		batt_info.cable_status,
		cable_status_str[batt_info.cable_status],
		batt_status_str[batt_info.status],
		batt_info.charging_toggle ? "ON" : "OFF",
		batt_info.Rsoc,
		battID,
		Thermal_Level);
	snprintf(battInfo, sizeof(battInfo), "%sReg:%s\n",
		battInfo,
		chargerReg);
	BAT_DBG("%s", battInfo);
	ASUSEvtlog("[BAT][Ser]report Capacity:%d,%d,%d,%d,%d,%s%d.%d,%d==>%d\n", batt_info.capacity,
		batt_info.fcc,
		batt_info.Rsoc,
		batt_info.voltage_now,
		batt_info.current_now,
		batt_info.temperature_negative_sign,
		batt_info.temperature10,
		batt_info.temperature,
		batt_info.cable_status,
		batt_info.capacity);
	//BAT_DBG("[BATT][CHARGER]Thermal_Level %d\n",Thermal_Level);
	g_last_print_time = current_kernel_time();
	return 0;
}
static struct workqueue_struct *chrgr_work_queue;
static struct delayed_work aicl_dete_work;
struct delayed_work SetBatRTCWorker;

void smb358_update_aicl_work(int time)
{
	int usb_state = g_usb_state;
	if (smb358_is_charging(usb_state)) {
		cancel_delayed_work(&aicl_dete_work);
	        flush_workqueue(chrgr_work_queue);
		queue_delayed_work(chrgr_work_queue,
			&aicl_dete_work,
			time * HZ);

	}
}
static int Thermal_Policy(int *thermal_policy_current){
         int usb_state;
         int ret;
         int cap;
         
         usb_state = g_usb_state;
         /* BSP Clay: Thermal policy stage 2 and 3 */
         if ( usb_state == AC_IN ){
                 if(Thermal_Level_old != Thermal_Level && Thermal_Level == 3){
                         BAT_DBG("Thermal_Level old:%d new:%d\n", Thermal_Level_old, Thermal_Level);
                         BAT_DBG("Suspend adapater to let Iusb_in current = 0mA!\n");
                         //under 02h[7]=1, 
                         *thermal_policy_current=0;
                         ret = __smb358_path_suspend(smb358_dev,1);
                         if(ret){
                                 BAT_DBG_E("%s: Set adapter to suspend mode fail!\n",__FUNCTION__);
								 return -1;
                         }
                         Thermal_Level_old = Thermal_Level;
                 }else if (Thermal_Level_old != Thermal_Level || Thermal_Level == 2){
                         BAT_DBG("Thermal_Level old:%d new:%d\n", Thermal_Level_old, Thermal_Level);
                         if(Thermal_Level_old == 3){
                                 ret =__smb358_path_suspend(smb358_dev,0);
                                 if (ret){
                                 		BAT_DBG_E("%s: Set adapter to normal mode fail!\n",__FUNCTION__);
										return -1;
                                 }
                         }

                         // 0 update aicl_worker, and wake_luck 6s
                         /* Thermal policy 1: Normal AC charging setting */
                         if (Thermal_Level == 0){
                                 BAT_DBG("Thermal_Level = %d, set Iusb_in current = 2000mA\n", Thermal_Level);
								 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_2000;
                                 //smb358_update_aicl_work(5);
                                 //wake_lock_timeout(&inok_wakelock, 5 * HZ);
                         } else if (Thermal_Level == 1){
                                 BAT_DBG("Thermal_Level = %d, set Iusb_in current = 1000mA\n", Thermal_Level);
								 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
                         } else if (Thermal_Level == 2){
                                 ret = get_bms_calculated_soc(&cap);
                                 if (ret < 0) {
                                         BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
                                         return -1;
                                 }

                                 if (cap >= 15){
                                         BAT_DBG("Thermal_Level = %d, cap = %d, set Iusb_in current = 500mA\n", Thermal_Level, cap);
										 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_500;
                                 } else if (cap >= 8){
                                         BAT_DBG("Thermal_Level = %d, cap = %d, set Iusb_in current = 700mA\n", Thermal_Level, cap);
										 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_700;

                                 } else if (cap < 8){
                                         BAT_DBG("Thermal_Level = %d, cap = %d, set Iusb_in current = 1000mA\n", Thermal_Level, cap);
										 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
                                 } 
                         }
                         Thermal_Level_old = Thermal_Level;
                 }
         }
         return 0;
 }	

static int Thermal_Policy_ze550(int *thermal_policy_current){
         int usb_state;
         int ret;
         int cap;
         
         usb_state = g_usb_state;
         /* BSP Clay: Thermal policy stage 2 and 3 */
         if ( usb_state == AC_IN ){
                 if(Thermal_Level_old != Thermal_Level && Thermal_Level == 3){
                         BAT_DBG("Thermal_Level old:%d new:%d\n", Thermal_Level_old, Thermal_Level);
                         BAT_DBG("Suspend adapater to let Iusb_in current = 0mA!\n");
                         //under 02h[7]=1, 
                         *thermal_policy_current=0;
                         ret = __smb358_path_suspend(smb358_dev,1);
                         if(ret){
                                 BAT_DBG_E("%s: Set adapter to suspend mode fail!\n",__FUNCTION__);
								 return -1;
                         }
                         Thermal_Level_old = Thermal_Level;
                 }else if (Thermal_Level_old != Thermal_Level || Thermal_Level == 2){
                         BAT_DBG("Thermal_Level old:%d new:%d\n", Thermal_Level_old, Thermal_Level);
                         if(Thermal_Level_old == 3){
                                 ret =__smb358_path_suspend(smb358_dev,0);
                                 if (ret){
                                 		BAT_DBG_E("%s: Set adapter to normal mode fail!\n",__FUNCTION__);
										return -1;
                                 }
                         }

                         // 0 update aicl_worker, and wake_luck 6s
                         /* Thermal policy 1: Normal AC charging setting */
                         if (Thermal_Level == 0){
                                 BAT_DBG("Thermal_Level = %d, set Iusb_in current = 1000mA\n", Thermal_Level);
								 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
                                 //smb358_update_aicl_work(5);
                                 //wake_lock_timeout(&inok_wakelock, 5 * HZ);
                         } else if (Thermal_Level == 1){
                                 BAT_DBG("Thermal_Level = %d, set Iusb_in current = 1000mA\n", Thermal_Level);
								 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
                         } else if (Thermal_Level == 2){
                                 ret = get_bms_calculated_soc(&cap);
                                 if (ret < 0) {
                                         BAT_DBG_E("%s: can't get raw bms soc, ret = %d\n", __FUNCTION__, ret);
                                         return -1;
                                 }

                                 if (cap >= 15){
                                         BAT_DBG("Thermal_Level = %d, cap = %d, set Iusb_in current = 500mA\n", Thermal_Level, cap);
										 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_500;
                                 } else if (cap >= 8){
                                         BAT_DBG("Thermal_Level = %d, cap = %d, set Iusb_in current = 700mA\n", Thermal_Level, cap);
										 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_700;

                                 } else if (cap < 8){
                                         BAT_DBG("Thermal_Level = %d, cap = %d, set Iusb_in current = 1000mA\n", Thermal_Level, cap);
										 *thermal_policy_current=CFG_CURRENT_LIMIT_SMB358_VALUE_1000;
                                 } 
                         }
                         Thermal_Level_old = Thermal_Level;
                 }
         }
         return 0;
 }	


static struct timespec g_last_aicl_time;
void aicl_dete_worker(struct work_struct *dat)
{
	int aicl_result;
	int usb_state;
	int ret;
	u8 reg;
	int aicl_time=60;

	BAT_DBG(" %s, dat: %s\n", __FUNCTION__, dat != NULL ? "YES" : "NO");
	if (!smb358_dev) {
		pr_err("%s: smb358_dev is null due to driver probed isn't ready\n",
			__FUNCTION__);
		return;
	}

	ret = smb358_read_reg(smb358_dev, CHG_CURRENT_CTRL_REG, &reg);
	if (ret < 0) {
			BAT_DBG_E("%s: fail to read charger voltage reg\n", __FUNCTION__);
			return ;
	}
	printk("%s CHG_CURRENT_CTRL_REG=%#x\n",__FUNCTION__,reg);

	usb_state = g_usb_state;
	/*BSP david: don't need to do any config if no cable!*/
	if (!smb358_is_charging(usb_state))
		return;

	/*BSP David -if AC_IN & AICL result <= 500, do initial config and skip JEITA*/
	if (usb_state == AC_IN) {
		/*get AICL result*/
		aicl_result = get_aicl_results();
		BAT_DBG("%s: aicl result(%dmA) with AICL status:%d\n", __FUNCTION__, aicl_result, reg);
		/*if AICL result <= 500, then config input current but don't do JEITA*/
		if(asus_PRJ_ID==ASUS_ZD550KL||asus_PRJ_ID==ASUS_ZE600KL){
			BAT_DBG("%s: ZD550KL & ZE600KL flow\n", __FUNCTION__);
                  	if (aicl_result <= 500 && Thermal_Level < 2) {
			        BAT_DBG("%s: don't do JEITA when aicl result(%dmA) <= 500mA.\n", __FUNCTION__, aicl_result);
			        smb358_config_max_current(usb_state);
				goto skip_jeita;
	        	} else if (aicl_result <= 1500) {
		        	smb358_masked_write(smb358_dev,
			        	CHG_OTH_CURRENT_CTRL_REG,
				        CFG_CURRENT_LIMIT_SMB358_MASK,
				        CFG_CURRENT_LIMIT_SMB358_VALUE_1000);
			        BAT_DBG("normal charger\n");
		        }else{
			        BAT_DBG("%s:Iusb_in 2000mA.\n", __FUNCTION__);
		        }
                }else{
          		//if((asus_PRJ_ID==ASUS_ZE550KL)&&((asus_lcd_id[0]=='0')||(asus_lcd_id[0]=='1'))){
          		if(asus_PRJ_ID==ASUS_ZE550KL){
					BAT_DBG("%s: ze550kl flow\n", __FUNCTION__);
					if (aicl_result <= 500 && Thermal_Level < 2) {
							BAT_DBG("%s: don't do JEITA when aicl result(%dmA) <= 500mA.\n", __FUNCTION__, aicl_result);
							smb358_config_max_current(usb_state);
							goto skip_jeita;
					}

				}else{
					BAT_DBG("%s: Default flow\n", __FUNCTION__);
                        
                  	if (aicl_result <= 500 && Thermal_Level < 2) {
			        BAT_DBG("%s: don't do JEITA when aicl result(%dmA) <= 500mA.\n", __FUNCTION__, aicl_result);
			        smb358_config_max_current(usb_state);
			        goto skip_jeita;
	        		} else if (aicl_result <= 1500) {
		        		smb358_masked_write(smb358_dev,
			        		CHG_OTH_CURRENT_CTRL_REG,
				        	CFG_CURRENT_LIMIT_SMB358_MASK,
				        	CFG_CURRENT_LIMIT_SMB358_VALUE_1000);
				        	printk("normal charger\n");
		       		 }
				}
        	}
	}
	
        if(asus_PRJ_ID==ASUS_ZD550KL||asus_PRJ_ID==ASUS_ZE600KL){
                if(asus_project_ADAPTER_ID){
			BAT_DBG("%s:use zd550 ze600 JEITA setting.\n", __FUNCTION__);
                        aicl_time = smb358_soc_detect_batt_tempr_zd550_ze600(usb_state);
                }else{
			BAT_DBG("%s:[5wBZsku] use 5wBZsku zd550 ze600 JEITA setting.\n", __FUNCTION__);
                        aicl_time = smb358_soc_detect_batt_tempr_5wBZsku_zd550(usb_state);
                }
        }else if(asus_PRJ_ID==ASUS_ZE550KL){
        		//if((asus_lcd_id[0]=='0') ||(asus_lcd_id[0]=='1')){   //ze550kl
                	aicl_time=smb358_soc_detect_batt_tempr_ze550(usb_state);
        		/*}else if((asus_lcd_id[0]=='2') ||(asus_lcd_id[0]=='3')){     //ze551kl
        			aicl_time=smb358_soc_detect_batt_tempr(usb_state);
        		}*/
        }else{
        		aicl_time=smb358_soc_detect_batt_tempr(usb_state);
        	}
        printk("%s:aicl_time=%d\n",__FUNCTION__,aicl_time);
skip_jeita:
	g_last_aicl_time = current_kernel_time();
	//smb358_update_aicl_work(60);
	if(aicl_time==60){
		printk("%s:queue_delayed_work 60s\n",__FUNCTION__);
		queue_delayed_work(chrgr_work_queue,
	                &aicl_dete_work,
	                60*HZ);
		}
	
		else if(aicl_time==5)
				{
				printk("%s:queue_delayed_work 5s\n",__FUNCTION__);
				//smb358_update_aicl_work(5);
				queue_delayed_work(chrgr_work_queue,
						&aicl_dete_work,
						5*HZ);
				wake_lock_timeout(&inok_wakelock, 5 * HZ);
				}
	schedule_delayed_work(&SetBatRTCWorker, 0);
}
EXPORT_SYMBOL(aicl_dete_worker);
struct delayed_work battery_poll_data_work;
void smb358_polling_battery_data_work(int time);
void asus_polling_data(struct work_struct *dat)
{
	int ret;
	int usb_state;
	usb_state = g_usb_state;
	ret = asus_print_all();
	/*BSP david: if report capacity fail, do it after 5s*/
	if (!ret) {
		//smb358_polling_battery_data_work(180);
		schedule_delayed_work(&battery_poll_data_work, 180 * HZ);
		if (!smb358_is_charging(usb_state)) {
			schedule_delayed_work(&SetBatRTCWorker, 0);
		}
	} else{
		printk("%s: gauge not ready yet, delay 5s!\n", __FUNCTION__);
		//smb358_polling_battery_data_work(5);
		schedule_delayed_work(&battery_poll_data_work, 5 * HZ);
	}
}
void smb358_polling_battery_data_work(int time)
{
	cancel_delayed_work(&battery_poll_data_work);
	flush_delayed_work(&battery_poll_data_work);
	schedule_delayed_work(&battery_poll_data_work, time * HZ);
}
static int smb358_routine_aicl_control(void)
{
	BAT_DBG(" %s\n", __FUNCTION__);

	INIT_DELAYED_WORK(&aicl_dete_work,
		aicl_dete_worker);
	chrgr_work_queue =	create_singlethread_workqueue("smb358_wq");
	if (!chrgr_work_queue) {
		BAT_DBG_E(" fail to create smb358_wq\n");
		return -ENOMEM;
	}
	return 0;
}
/*+++BSP David AC power supply+++*/
static char *supply_list[] = {
	"battery",
};

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};


static int smb358_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	usb_state = g_usb_state;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = (usb_state == AC_IN || usb_state == UNKNOWN_IN || usb_state == SE1_IN) ? 1 : 0;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
static struct power_supply smb358_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb358_power_get_property,
	},
};
static int smb358_register_power_supply(struct device *dev)
{
	int ret;
	ret = power_supply_register(dev, &smb358_power_supplies[0]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}
	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb358_power_supplies[0]);
	return ret;
}
/*---BSP David AC power supply---*/
static struct alarm bat_alarm;
struct wake_lock bat_alarm_wake_lock;
struct wake_lock UsbCable_Lock;
static DEFINE_SPINLOCK(bat_alarm_slock);
static enum alarmtimer_restart batAlarm_handler(struct alarm *alarm, ktime_t now)
{
	printk("[BAT][alarm]batAlarm triggered\n");
	return ALARMTIMER_NORESTART;
}
void BatTriggeredSetRTC(struct work_struct *dat)
{
	unsigned long batflags;
	struct timespec new_batAlarm_time;
	struct timespec mtNow;
	int RTCSetInterval = 180;
	int cableType;
	int soc;
	int ret;

	if (!smb358_dev) {
		BAT_DBG("%s: driver not ready yet!\n", __FUNCTION__);
		return;
	}

	mtNow = current_kernel_time();
	new_batAlarm_time.tv_sec = 0;
	new_batAlarm_time.tv_nsec = 0;

	cableType = get_charger_type();

	if (smb358_is_charging(cableType)) {
		RTCSetInterval = 60;
	} else{
		ret = get_battery_rsoc(&soc);
		if (!ret) {
			if (soc <= 1) {
				RTCSetInterval = 300;
			} else{
				RTCSetInterval = soc * 450;
				if (RTCSetInterval < 450) {
					RTCSetInterval = 450;
				}
				if (RTCSetInterval > 36000) {
					RTCSetInterval = 36000;
				}
			}
		} else{
			BAT_DBG_E("%s: NO SOC INFO! Set minimum alarm time!\n", __FUNCTION__);
			RTCSetInterval = 300;
		}
	}
	new_batAlarm_time.tv_sec = mtNow.tv_sec + RTCSetInterval;
	BAT_DBG("%s: alarm start after %ds\n", __FUNCTION__, RTCSetInterval);
	spin_lock_irqsave(&bat_alarm_slock, batflags);
	alarm_start(&bat_alarm,
		timespec_to_ktime(new_batAlarm_time));
	spin_unlock_irqrestore(&bat_alarm_slock, batflags);
}

/* BSP Clay: AC charger debounce policy */
static struct timespec CABLE_OUT_CHECK;
static struct timespec MTNOW;
bool AC_IN_EVER = false;
bool CABLE_OUT_IS_TRUE = false;
struct delayed_work AC_unstable_det;

/* BSP Clay: Cable out function */
void AC_unstable_detect(struct work_struct *dat){
	struct power_supply *psy;
	int ret;
	Thermal_Level_old=0;
	ret=__smb358_path_suspend(smb358_dev,0);
	if(ret)
		pr_err("%s: fail to disable suspend adapter register 30h[2]\n", __FUNCTION__);
	//reset flag when cable out by kerwin bsp
	fast_chg_flag = false; 
	JEITA_Flag=false;
	g_usb_state = CABLE_OUT;
	AC_IN_EVER = false;
	BAT_DBG("Clay: CABLE_OUT operation is done!\n");
	if (smb358_dev) {
		cancel_delayed_work(&aicl_dete_work);
		flush_delayed_work(&aicl_dete_work);
		schedule_delayed_work(&SetBatRTCWorker, 0);
	}
	/*BSP david: update power_supply after cable type changed*/
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);		
	}
	if (smb358_dev) {
		power_supply_changed(&smb358_power_supplies[0]);
	}
}
int setSMB358Charger(int usb_state)
{
	int ret = 0;
	struct power_supply *psy;
	static long time_cal = 0;
	char usb_status_str[8][11] = {
		"USB_IN",
		"AC_IN",
		"CABLE_OUT",
		"ENABLE_5V",
		"DISABLE_5V",
		"PAD_SUPPLY",
		"UNKNOWN_IN",
		"SE1_IN",
	};
	if(asus_PRJ_ID==ASUS_ZE550KL)
		g_usb_state = usb_state;
	else{
		if( usb_state != CABLE_OUT )
			g_usb_state = usb_state;
	}
	printk(KERN_EMERG "setSMB358Charger  usb_state:%d\n",usb_state);
	if (usb_state >= 0 && usb_state <= 7) {
		BAT_DBG("%s:%s\n", __FUNCTION__, usb_status_str[usb_state]);
	}

	/*BSP david: update power_supply after cable type changed*/
	psy = get_psy_battery();
	if (psy) {
		power_supply_changed(psy);
	} else{
		pr_err("%s: fail to request power supply changed\n", __FUNCTION__);		
	}
	if (smb358_dev) {
		power_supply_changed(&smb358_power_supplies[0]);
		//under 02h[7]=0,  prevent cable_plug & reboot when suspend adapter
		ret=__smb358_path_suspend(smb358_dev,0);
		if (ret){
                BAT_DBG_E("%s: Set adapter to normal mode fail!\n",__FUNCTION__);
			//	return -1;
                }
	}
	switch (usb_state) {
	case AC_IN:
		if (smb358_dev) {			
			if(!wake_lock_active(&UsbCable_Lock))
				wake_lock(&UsbCable_Lock);
			if(asus_PRJ_ID == ASUS_ZD550KL) {
				focal_usb_detection(true);
			}
			if(asus_PRJ_ID == ASUS_ZE600KL) {
				focal_usb_detection(true);
			}
			if(asus_PRJ_ID==ASUS_ZE550KL){
				mutex_lock(&g_usb_state_lock);
				/*BSP david : do initial setting & config current*/
				smb358_config_max_current(g_usb_state);
				mutex_unlock(&g_usb_state_lock);
				/*BSP david : do JEITA*/
				smb358_update_aicl_work(5);
				focal_usb_detection(true);
			}else{
				/* BSP Clay: AC debounce policy +++*/
				if ( AC_IN_EVER ){
					MTNOW = current_kernel_time();
					time_cal = MTNOW.tv_sec - CABLE_OUT_CHECK.tv_sec;
					time_cal = time_cal * 1000000000 + (MTNOW.tv_nsec - CABLE_OUT_CHECK.tv_nsec);
					if ( time_cal <= 500000000 ){
						BAT_DBG("AC debounce policy: The time of CABLE_OUT change to AC_IN is only %ld\n", time_cal);
						cancel_delayed_work_sync(&AC_unstable_det);
						g_usb_state = SE1_IN;
						BAT_DBG("Cancel CABLE_OUT operation and Change to SE1_IN setting!\n");
					}
				}
				AC_IN_EVER = true;
				/* BSP Clay: AC debounce policy ---*/
				mutex_lock(&g_usb_state_lock);
				/*BSP david : do initial setting & config current*/
				smb358_config_max_current(g_usb_state);
				mutex_unlock(&g_usb_state_lock);
				/*BSP david : do JEITA*/
				if (g_usb_state == SE1_IN) {
					smb358_update_aicl_work(0);
				} else{
					smb358_update_aicl_work(5);
				}
			}
			schedule_delayed_work(&LED_ChargerMode, 0*HZ);
		}
		break;
	case USB_IN:
	case UNKNOWN_IN:
	case SE1_IN:
		if(asus_PRJ_ID == ASUS_ZD550KL) {
			if(usb_state==USB_IN)
				focal_usb_detection(true);
		}
		if(asus_PRJ_ID == ASUS_ZE600KL) {
			if(usb_state==USB_IN)
				focal_usb_detection(true);
		}
		if(asus_PRJ_ID==ASUS_ZE550KL){
			if(usb_state==USB_IN)
				focal_usb_detection(true);
		}else{	
			/* BSP Clay: AC debounce policy */
			if(AC_IN_EVER == true){
				AC_IN_EVER = false;
				cancel_delayed_work_sync(&AC_unstable_det);
			}
		}
		if (smb358_dev) {
			if(!wake_lock_active(&UsbCable_Lock))
				wake_lock(&UsbCable_Lock);
			mutex_lock(&g_usb_state_lock);
			/*BSP david : do initial setting & config current*/
			smb358_config_max_current(usb_state);
			mutex_unlock(&g_usb_state_lock);
			/*BSP david : do JEITA*/
			smb358_update_aicl_work(0);
			schedule_delayed_work(&LED_ChargerMode, 0*HZ);
	                schedule_delayed_work(&USB_3s_retry_kicker_work, 0*HZ);
		}
		break; 
	case CABLE_OUT:
		if (smb358_dev) {	
			if(wake_lock_active(&UsbCable_Lock))
				wake_unlock(&UsbCable_Lock);
			if(asus_PRJ_ID == ASUS_ZD550KL) {
				focal_usb_detection(false);
			}
			if(asus_PRJ_ID == ASUS_ZE600KL) {
				focal_usb_detection(false);
			}
			if(asus_PRJ_ID==ASUS_ZE550KL){
				schedule_delayed_work(&AC_unstable_det, 0);
				focal_usb_detection(false);
			}else{
				/* BSP_Clay: AC debounce policy */
				if ( AC_IN_EVER ){
					CABLE_OUT_CHECK = current_kernel_time();
					schedule_delayed_work(&AC_unstable_det, HZ/2);
				}else{ /* BSP Clay: Normal Cable out */
					schedule_delayed_work(&AC_unstable_det, 0);
				}
			}
			JEITA_Flag=false;
                        //set LED off in charger mode
                        charger_mode_cable_out = true;
			schedule_delayed_work(&LED_ChargerMode, 0*HZ);
		}
		break;
	case ENABLE_5V:
		if (smb358_dev) {
			if(asus_PRJ_ID==ASUS_ZE550KL){
			}else{
				/* BSP Clay: AC debounce policy */
				if(AC_IN_EVER == true){
					AC_IN_EVER = false;
					cancel_delayed_work_sync(&AC_unstable_det);
				}
			}
			ret = otg(1);
		}
		break;
	case DISABLE_5V:
		if (smb358_dev) {
			if(asus_PRJ_ID==ASUS_ZE550KL){
			}else{
				/* BSP Clay: AC debounce policy */
				if(AC_IN_EVER == true){
					BAT_DBG("Clay: Cancel CABLE_OUT operation!At USB_IN\n");
					AC_IN_EVER = false;
					cancel_delayed_work_sync(&AC_unstable_det);
				}
			}
			ret = otg(0);
		}
		break;
	default:
		BAT_DBG(" %s: not support usb state = %d\n", __FUNCTION__, usb_state);
		ret = 1;
	}
	return ret;
}
EXPORT_SYMBOL(setSMB358Charger);
/*+++BSP david: charger register dump proc+++*/

#define BYTETOBINARY(byte) \
	(byte & 0x80 ? 1 : 0), \
	(byte & 0x40 ? 1 : 0), \
	(byte & 0x20 ? 1 : 0), \
	(byte & 0x10 ? 1 : 0), \
	(byte & 0x08 ? 1 : 0), \
	(byte & 0x04 ? 1 : 0), \
	(byte & 0x02 ? 1 : 0), \
	(byte & 0x01 ? 1 : 0)

static int ChargerRegDump_read_proc(struct seq_file *buf, void *v)
{
	int i = 0;
	int Charger_reg_value = -1;
	char tempString[512] = "";
	for (i = 0; i < ARRAY_SIZE(ChargerReg_CMD_Table); i++) {
		Charger_reg_value = ChargerReg_proc_read_By_Table(i);
		snprintf(tempString, sizeof(tempString), "%s0x%02x=%d%d%d%d%d%d%d%d\n", tempString, ChargerReg_CMD_Table[i].addr, BYTETOBINARY(Charger_reg_value));
	}
	return seq_printf(buf, "%s", tempString);
}
static ssize_t ChargerRegDump_write_proc(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]%s: %d\n", __FUNCTION__, val);

	return len;
}

static int ChargerRegDump_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ChargerRegDump_read_proc, NULL);
}

static const struct file_operations ChargerRegDump_fops = {
	.owner = THIS_MODULE,
	.open =  ChargerRegDump_proc_open,
	.write = ChargerRegDump_write_proc,
	.read = seq_read,
	.release = single_release,
};

static void create_ChargerRegDump_proc_file(void)
{
	struct proc_dir_entry *ChargerReg_proc_file = proc_create("driver/Charger_Reg_Dump", 0666, NULL, &ChargerRegDump_fops);

	if (ChargerReg_proc_file) {
		printk("[BAT][Proc]%s sucessed!\n", __FUNCTION__);
	} else{
		printk("[BAT][Proc]%s failed!\n", __FUNCTION__);
	}

	return;
}

bool inok_status(void){
 	return	gpio_get_value(gp_inok);
}

/*---BSP david: charger register dump proc---*/
static irqreturn_t smb358_inok_interrupt(int irq, void *data)
{
	struct smb358_charger *smb = data;

	/* wake lock to prevent system instantly
	   enter S3 while it's in resuming flow */
	BAT_DBG("%s: wake up system!\n", __FUNCTION__);
	pm_runtime_get_sync(&smb->client->dev);
	if (gpio_get_value(gp_inok)) {
		wake_lock_timeout(&inok_wakelock, 2*HZ);
		dev_warn(&smb->client->dev,
			"%s: >>> INOK pin (HIGH) <<<\n", __FUNCTION__);
	} else {
		/*BSP david: set wake lock 5s to prevent system suspend 
				before reading aicl result when cable in*/
		wake_lock_timeout(&inok_wakelock, 6 * HZ);
		dev_warn(&smb->client->dev,
			"%s: >>> INOK pin (LOW) <<<\n", __FUNCTION__);
	}
	pm_runtime_put_sync(&smb->client->dev);
	return IRQ_HANDLED;
}
/*BSP david: set pinctrl to enable inok gpio*/
static void smb358_set_pinctrl(void)
{
	int ret;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	
	key_pinctrl = devm_pinctrl_get(smb358_dev->dev);
	set_state = pinctrl_lookup_state(key_pinctrl, "smb358_nfc_disable_active");
	ret = pinctrl_select_state(key_pinctrl, set_state);
	printk("%s: pinctrl_select_state = %d\n", __FUNCTION__, ret);
}
static int smb358_inok_gpio_init(struct smb358_charger *smb)
{
	int ret, irq;
	printk("%s: gp_inok = %d\n", __FUNCTION__, gp_inok);
	if ((!gpio_is_valid(gp_inok))) {
		printk("%s: gp_inok is not valid!\n", __FUNCTION__);
		goto fail;
	}
	/*BSP david: set pinctrl to enable inok gpio*/
	smb358_set_pinctrl();
	if (gpio_get_value(gp_inok)) {
		BAT_DBG("%s:>>> INOK (HIGH) <<<\n", __FUNCTION__);
	} else{
		BAT_DBG("%s:>>> INOK (LOW) <<<\n", __FUNCTION__);
	}
	ret = gpio_request_one(gp_inok, GPIOF_IN, "smb358_inok");
	if (ret < 0) {
		printk("%s: request INOK gpio fail!\n", __FUNCTION__);
		goto fail;
	}
	irq = gpio_to_irq(gp_inok);

	ret = request_threaded_irq(irq, NULL, smb358_inok_interrupt,
			IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
			IRQF_ONESHOT |
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			smb->client->name,
			smb);
	if (ret < 0) {
		BAT_DBG_E("%s: config INOK gpio as IRQ fail!\n", __FUNCTION__);
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(gp_inok);
fail:
	smb->client->irq = 0;
	return ret;
}
/*+++BSP David BMMI Adb Interface+++*/
#define	chargerIC_status_PROC_FILE	"chargerIC_status"
#define charger_version	"0.0.5"
static struct proc_dir_entry *chargerIC_status_proc_file;
static int chargerIC_status_proc_read(struct seq_file *buf, void *v)
{
	int ret = -1;
	u8 reg;
	char *str_ok="ACK: i2c r/w test ok";
	char *str_err="ERROR: i2c r/w test fail";
	ret = smb358_read_reg(smb358_dev, 0x01, &reg);
	if (ret) {
		ret = 0;
		seq_printf(buf, "%s\n%s\n", str_err,charger_version);
	} else{
		ret = 1;
		seq_printf(buf, "%s\n%s\n", str_ok,charger_version);
	}
	//seq_printf(buf, "%d\n", ret);
	return 0;
}
static int chargerIC_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, chargerIC_status_proc_read, NULL);
}
static ssize_t chargerIC_status_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]ChargerIC Proc File: %d\n", val);

	return len;
}

static const struct file_operations chargerIC_status_fops = {
	.owner = THIS_MODULE,
	.open = chargerIC_status_proc_open,
	.write = chargerIC_status_proc_write,
	.read = seq_read,
	.release = single_release,
};
void static create_chargerIC_status_proc_file(void)
{
	chargerIC_status_proc_file = proc_create(chargerIC_status_PROC_FILE, 0644, NULL, &chargerIC_status_fops);

	if (chargerIC_status_proc_file) {
		BAT_DBG("[Proc]%s sucessed!\n", __FUNCTION__);
	} else{
		BAT_DBG("[Proc]%s failed!\n", __FUNCTION__);
	}
}

#if defined(ASUS_FACTORY_BUILD)
#define	batt_current_PROC_FILE	"batt_current_now"
static struct proc_dir_entry *batt_current_proc_file;
static int batt_current_proc_read(struct seq_file *buf, void *v)
{
	int ret = -1;
	struct power_supply *psy;
	union power_supply_propval val;
	int current_now=0;

	psy = get_psy_battery();
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret) {
		current_now = val.intval / 1000;
		current_now=-current_now;
		seq_printf(buf, "%d\n", current_now);
	} else{
		BAT_DBG_E("%s: can't get current_now, ret = %d!\n", __FUNCTION__, ret);
		current_now = ret;
	}

	return 0;
}

static int batt_current_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, batt_current_proc_read, NULL);
}

static ssize_t batt_current_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]batt_current Proc File: %d\n", val);

	return len;
}

static const struct file_operations batt_current_fops = {
	.owner = THIS_MODULE,
	.open = batt_current_proc_open,
	.write = batt_current_proc_write,
	.read = seq_read,
	.release = single_release,
};
void static create_batt_current_proc_file(void)
{
	batt_current_proc_file = proc_create(batt_current_PROC_FILE, 0644, NULL, &batt_current_fops);

	if (batt_current_proc_file) {
		BAT_DBG("[Proc]%s sucessed!\n", __FUNCTION__);
	} else{
		BAT_DBG("[Proc]%s failed!\n", __FUNCTION__);
	}
}


#define	batt_voltage_PROC_FILE	"batt_voltage_now"
static struct proc_dir_entry *batt_voltage_proc_file;
static int batt_voltage_proc_read(struct seq_file *buf, void *v)
{
	int ret = -1;
	struct power_supply *psy;
	union power_supply_propval val;
	int voltage_now=0;

	psy = get_psy_battery();
	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret) {
		voltage_now = val.intval / 1000;
		seq_printf(buf, "%d\n", voltage_now);
	} else{
		BAT_DBG_E("%s: can't get voltage_now, ret = %d!\n", __FUNCTION__, ret);
		voltage_now = ret;
	}

	return 0;
}

static int batt_voltage_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, batt_voltage_proc_read, NULL);
}

static ssize_t batt_voltage_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	printk("[BAT][CHG][SMB][Proc]batt_voltage Proc File: %d\n", val);

	return len;
}

static const struct file_operations batt_voltage_fops = {
	.owner = THIS_MODULE,
	.open = batt_voltage_proc_open,
	.write = batt_voltage_proc_write,
	.read = seq_read,
	.release = single_release,
};
void static create_batt_voltage_proc_file(void)
{
	batt_voltage_proc_file = proc_create(batt_voltage_PROC_FILE, 0644, NULL, &batt_voltage_fops);

	if (batt_voltage_proc_file) {
		BAT_DBG("[Proc]%s sucessed!\n", __FUNCTION__);
	} else{
		BAT_DBG("[Proc]%s failed!\n", __FUNCTION__);
	}
}


/*---BSP David BMMI Adb Interface---*/
/*+++BSP David proc charger_limit_enable Interface+++*/
static int charger_limit_enable_proc_read(struct seq_file *buf, void *v)
{
	if (eng_charging_limit) {
		seq_printf(buf, "charging limit enable\n");
	} else{
		seq_printf(buf, "charging limit disable\n");
	}
	return 0;
}

static ssize_t charger_limit_enable_proc_write(struct file *filp, const char __user *buff,
		size_t len, loff_t *data)
{
	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	if (buff[0] == '1') {
		eng_charging_limit = true;
		g_charging_toggle_for_charging_limit = true;
		charger_suspend_for_charging_limit = false;
		/* turn on charging limit in eng mode */
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable:%d\n", 1);
	} else if (buff[0] == '0') {
		eng_charging_limit = false;
		g_charging_toggle_for_charging_limit = true;
		charger_suspend_for_charging_limit = false;
		/* turn off charging limit in eng mode */
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable:%d\n", 0);
	}

	//kerwin for charger limit
	sscanf(buff,"%s %d",messages,&charger_limit_setting);
	if(charger_limit_setting<10)
		charger_limit_setting=10;
	else if(charger_limit_setting>100)
		charger_limit_setting=100;
	printk("[BAT][CHG][SMB][Proc]charger_limit_setting:%d\n", charger_limit_setting);
	smb358_update_aicl_work(0);
	return len;
}

static int charger_limit_enable_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, charger_limit_enable_proc_read, NULL);
}

static const struct file_operations charger_limit_enable_fops = {
	.owner = THIS_MODULE,
	.open =  charger_limit_enable_proc_open,
	.write = charger_limit_enable_proc_write,
	.read = seq_read,
	.release = single_release,
};

static void create_charger_limit_enable_proc_file(void)
{
	struct proc_dir_entry *charger_limit_enable_proc_file = proc_create("driver/charger_limit_enable", 0666, NULL, &charger_limit_enable_fops);

	if (charger_limit_enable_proc_file) {
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable create ok!\n");
	} else{
		printk("[BAT][CHG][SMB][Proc]charger_limit_enable create failed!\n");
	}
	return;
}
/*---BSP David proc charger_limit_enable Interface---*/
#endif
static int smb358_charger_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret;
	int usb_state;
	struct smb358_charger *chip;
	struct power_supply *usb_psy;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;

	printk("%s++\n", __FUNCTION__);
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;
	chip->fake_battery_soc = -EINVAL;
	ret = smb358_register_power_supply(&client->dev);
	if (ret < 0)
		return ret;
	INIT_DELAYED_WORK(&SetBatRTCWorker, BatTriggeredSetRTC);
	wake_lock_init(&bat_alarm_wake_lock, WAKE_LOCK_SUSPEND, "bat_alarm_wake");
	wake_lock_init(&UsbCable_Lock, WAKE_LOCK_SUSPEND, "UsbCable_Lock_Wake");
	alarm_init(&bat_alarm, ALARM_REALTIME, batAlarm_handler);
	smb358_dev = chip;

	mutex_init(&chip->irq_complete);
	mutex_init(&chip->read_write_lock);
	mutex_init(&chip->path_suspend_lock);
	i2c_set_clientdata(client, chip);
	/* early for VADC get, defer probe if needed */
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");

	INIT_DELAYED_WORK(&battery_poll_data_work, asus_polling_data);
	ret = smb358_routine_aicl_control();
	if (ret < 0)
		return ret;
	gp_sdio_2_clk = of_get_named_gpio(np, "qcom,otg-gpio", 0);
	gp_inok = of_get_named_gpio(np, "qcom,inok-gpio", 0);
	printk("%s: gp_sdio_2_clk = %d\n", __FUNCTION__, gp_sdio_2_clk);
	if ((!gpio_is_valid(gp_sdio_2_clk))) {
		printk("%s: gp_sdio_2_clk is not valid!\n", __FUNCTION__);
		return -EINVAL;
	}
	ret = gpio_request(gp_sdio_2_clk,	"smb358_otg");
	if (ret < 0) {
		printk("%s: request CHG_OTG gpio fail!\n", __FUNCTION__);
	}

	gpio_direction_output(gp_sdio_2_clk, 0);
	if (ret < 0) {
		printk("%s: set direction of CHG_OTG gpio fail!\n", __FUNCTION__);
	}
	
	wake_lock_init(&inok_wakelock,
			WAKE_LOCK_SUSPEND, "smb358_wakelock");
	/* INOK pin configuration */
	ret = smb358_inok_gpio_init(smb358_dev);
	if (ret < 0) {
		dev_warn(dev,
			"fail to initialize INOK gpio: %d\n",
			ret);
	}
//BSP Ben add adapter_id for 5w (BZ sku) porting
//adapter_id:   0 = 5w
//              1 = 10w
        printk("%s:ADAPTER_ID=<%d>\n",__func__,asus_project_ADAPTER_ID);

#if defined(ASUS_FACTORY_BUILD)
	eng_charging_limit = false;
	g_charging_toggle_for_charging_limit = true;
	charger_suspend_for_charging_limit=false;
	charger_limit_setting=60;
	create_charger_limit_enable_proc_file();
	create_batt_current_proc_file();
	create_batt_voltage_proc_file();
#endif
	create_ChargerRegDump_proc_file();
	create_chargerIC_status_proc_file();
	create_battID_read_proc_file();
	determine_project_id(asus_PRJ_ID);

	smb358_enable_volatile_writes(smb358_dev);

	INIT_DELAYED_WORK(&Set_vch_val, SET_VCH_VALUE);
	INIT_DELAYED_WORK(&AC_unstable_det, AC_unstable_detect);
	INIT_DELAYED_WORK(&LED_ChargerMode, LED_ChargerMode_Set);
	INIT_DELAYED_WORK(&USB_3s_retry_kicker_work, USB_3s_retry_kicker);
	INIT_DELAYED_WORK(&USB_3s_retry_work, USB_3s_retry_set);
	schedule_delayed_work(&Set_vch_val, (unsigned long)(charger_factor->SET_VCH_FREQ)* HZ);	//<asus-guc20150427>	3
	usb_state = g_usb_state;
	/*BSP david: do JEITA if the usb state has changed*/
	if (smb358_is_charging(usb_state)) {
		mutex_lock(&g_usb_state_lock);
		//under 02h[7]=0,  prevent cable_plug & reboot when suspend adapter
		ret=__smb358_path_suspend(smb358_dev,0);
		if (ret){
                BAT_DBG_E("%s: Set adapter to normal mode fail!\n",__FUNCTION__);
				return -1;
        }
		smb358_config_max_current(g_usb_state);
		mutex_unlock(&g_usb_state_lock);
		if (usb_state == AC_IN) {
			smb358_update_aicl_work(5);
		} else{
			smb358_update_aicl_work(0);
		}
	}
	smb358_polling_battery_data_work(0);
	printk("%s--\n", __FUNCTION__);
	return 0;
}

static int smb358_charger_remove(struct i2c_client *client)
{
	struct smb358_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);
	if (gpio_is_valid(chip->chg_valid_gpio))
		gpio_free(chip->chg_valid_gpio);

	if (chip->vcc_i2c)
		regulator_disable(chip->vcc_i2c);

	mutex_destroy(&chip->irq_complete);
	wake_lock_destroy(&inok_wakelock);
	debugfs_remove_recursive(chip->debug_root);
	return 0;
}

static int smb358_suspend(struct device *dev)
{
	cancel_delayed_work(&aicl_dete_work);
	flush_delayed_work(&aicl_dete_work);
	return 0;
}

static int smb358_suspend_noirq(struct device *dev)
{
	return 0;
}

static int smb358_resume(struct device *dev)
{
	struct timespec mtNow;
	int nextAICLinterval;
	int usb_state = g_usb_state;
	BAT_DBG("%s\n", __FUNCTION__);
	mtNow = current_kernel_time();
	/*BSP david: if not update for more than 180s, do report capacity*/
	if (mtNow.tv_sec - g_last_print_time.tv_sec >= REPORT_CAPACITY_POLLING_TIME) {
		smb358_polling_battery_data_work(0);
	}
	
	if (smb358_is_charging(usb_state)) {
		/*BSP david: if next AICL time less than 30s, do aicl
				(next AICL time = last aicl time + 60s)*/
		nextAICLinterval = 60 - (mtNow.tv_sec - g_last_aicl_time.tv_sec);
		if (nextAICLinterval <= AICL_MINIMUM_INTERVAL) {
			smb358_update_aicl_work(0);
		} else{
			smb358_update_aicl_work(nextAICLinterval);
		}
	}
	return 0;
}

static void smb358_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\n", __FUNCTION__);

}
static const struct dev_pm_ops smb358_pm_ops = {
	.suspend	= smb358_suspend,
	.suspend_noirq	= smb358_suspend_noirq,
	.resume		= smb358_resume,
};

static struct of_device_id smb358_match_table[] = {
	{ .compatible = "qcom,smb358-charger",},
	{ },
};

static const struct i2c_device_id smb358_charger_id[] = {
	{"smb358-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb358_charger_id);

static struct i2c_driver smb358_charger_driver = {
	.driver		= {
		.name		= "smb358-charger",
		.owner		= THIS_MODULE,
		.of_match_table = smb358_match_table,
		.pm		= &smb358_pm_ops,
	},
	.probe		= smb358_charger_probe,
	.remove		= smb358_charger_remove,
	.shutdown    = smb358_shutdown,
	.id_table	= smb358_charger_id,
};

module_i2c_driver(smb358_charger_driver);

MODULE_DESCRIPTION("SMB358 Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:smb358-charger");
