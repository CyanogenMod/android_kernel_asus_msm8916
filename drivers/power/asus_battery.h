/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Chris Chang chris1_chang@asus.com
 */
#include <linux/i2c.h>
#include <linux/seq_file.h>
enum {
	USB_IN,
	AC_IN,
	CABLE_OUT,
	ENABLE_5V,
	DISABLE_5V,
	PAD_SUPPLY,
	UNKNOWN_IN,
//ASUS_BSP+++ Landice "[ZE500KL][USB][NA][spec] Separate SE1 from DCP from charger types"
	SE1_IN,
//ASUS_BSP--- Landice "[ZE500KL][USB][NA][spec] Separate SE1 from DCP from charger types"
};

typedef enum {
	BALANCE = 0,
	JEITA,
	FLAGS,
} charging_toggle_level_t;

/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB358Charger(int usb_state);

#define HIGH_TEMPERATURE_TO_STOP_CHARGING  (550)
#define LOW_TEMPERATURE_TO_STOP_CHARGING   (0)

#define BATTERY_TAG "<BATT>"
#define BAT_DBG(...)  printk(KERN_INFO BATTERY_TAG __VA_ARGS__)
#define BAT_DBG_L(level, ...)  printk(level BATTERY_TAG __VA_ARGS__)
#define BAT_DBG_E(...)  printk(KERN_ERR BATTERY_TAG __VA_ARGS__)

#define BYTETOBINARYPATTERN "%d%d%d%d-%d%d%d%db"
#define BYTETOBINARY(byte) \
	(byte & 0x80 ? 1 : 0), \
	(byte & 0x40 ? 1 : 0), \
	(byte & 0x20 ? 1 : 0), \
	(byte & 0x10 ? 1 : 0), \
	(byte & 0x08 ? 1 : 0), \
	(byte & 0x04 ? 1 : 0), \
	(byte & 0x02 ? 1 : 0), \
	(byte & 0x01 ? 1 : 0)
