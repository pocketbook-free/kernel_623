/* fp9928.h
 *
 * Copyright (c) 2011 Foxconn TMSBG Co., Ltd.
 *
 * Head file for Fitipower 9928 EPD PMIC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef __LINUX_REGULATOR_FP9928_H_
#define __LINUX_REGULATOR_FP9928_H_

/*
 * PMIC Register Addresses
 */
enum { 
	REG_TMST_VALUE  = 0x00,
	REG_FUNC_ADJUST,
	REG_VCOM_SETTING,
};

/*
 * Regulator configure
 */
enum {
    /* In alphabetical order */
	FP9928_DISPLAY,
	FP9928_VCOM,
	FP9928_NUM_REGULATORS,	
};

/*
 * PMIC Driver data structure
 */
struct fp9928_data {
	struct device *dev;

	/* Platform connection */
	struct i2c_client *i2c_client;
	
	/* read/write mutex lock */
	struct mutex rwlock;

	/* Client devices */
	struct platform_device *pdev[FP9928_NUM_REGULATORS];

	/* GPIOs */
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;

	/* fp9928 part variables */
	int vcom_uV;

	/* One-time VCOM setup marker */
	bool vcom_setup;
	bool init_done;

	/* powerup/powerdown wait time in ms */
	int max_wait;
};

/*
 * Platform data structure
 */
struct fp9928_platform_data {
	int gpio_pmic_vcom_ctrl;
	int gpio_pmic_wakeup;
	struct regulator_init_data *regulator_init;
	int (*init)(struct fp9928_data *);
};

/*
 * Declarations
 */
struct regulator_init_data;

int fp9928_register_regulator(struct fp9928_data *fp9928_dev, int id,
				     struct regulator_init_data *initdata);

int fp9928_i2c_device_write(u8 reg, u8 value);
int fp9928_i2c_device_read(u8 reg, u8 *dest);

#endif
