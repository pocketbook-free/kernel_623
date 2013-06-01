/*
 * Copyright 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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
 */

/*!
 * @file pmic/core/mc13892.c
 * @brief This file contains MC13892 specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mfd/mc13892/core.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <mach/iomux-mx50.h>
#include <mach/iomux-v3.h>
#include <linux/led/pmic_leds.h>
#include "pmic.h"
#include "linux/hw_ops.h"
#ifndef CONFIG_MXC_PMIC_I2C
struct i2c_client *mc13892_client;
#endif

#define WDI (5*32 + 28) /* GPIO_6_28 */
#define FRONT_LIGHT_POWER_OFF	gpio_direction_output(169,0)
extern int power_off_flag;
void *mc13892_alloc_data(struct device *dev)
{
	struct mc13892 *mc13892;

	mc13892 = kzalloc(sizeof(struct mc13892), GFP_KERNEL);
	if (mc13892 == NULL)
		return NULL;

	mc13892->dev = dev;

	return (void *)mc13892;
}
EXPORT_SYMBOL(mc13892_alloc_data);

int mc13892_init_registers(void)
{
	CHECK_ERROR(pmic_write(REG_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_STATUS0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_INT_STATUS1, 0xFFFFFF));
	/* disable auto charge */
	if (machine_is_mx51_3ds())
		CHECK_ERROR(pmic_write(REG_CHARGE, 0xB40003));

	pm_power_off = mc13892_power_off;
	hw_set_device_poweroff(mc13892_power_off);
	return PMIC_SUCCESS;
}
EXPORT_SYMBOL(mc13892_init_registers);

/*!
 * This function returns the PMIC version in system.
 *
 * @param 	ver	pointer to the pmic_version_t structure
 *
 * @return       This function returns PMIC version.
 */
void mc13892_get_revision(pmic_version_t *ver)
{
	int rev_id = 0;
	int rev1 = 0;
	int rev2 = 0;
	int finid = 0;
	int icid = 0;

	ver->id = PMIC_MC13892;
	pmic_read(REG_IDENTIFICATION, &rev_id);

	rev1 = (rev_id & 0x018) >> 3;
	rev2 = (rev_id & 0x007);
	icid = (rev_id & 0x01C0) >> 6;
	finid = (rev_id & 0x01E00) >> 9;

	ver->revision = ((rev1 * 10) + rev2);
	printk(KERN_INFO "mc13892 Rev %d.%d FinVer %x detected\n", rev1,
	       rev2, finid);
}
EXPORT_SYMBOL(mc13892_get_revision);

#define PMIC_BUTTON_DEBOUNCE_VALUE	0x2
#define PMIC_BUTTON_DEBOUNCE_MASK	0x3

static void powerbutton_debounce(void)
{
	/* Configure debounce time for power button 1 */
	pmic_write_reg(REG_POWER_CTL2, (PMIC_BUTTON_DEBOUNCE_VALUE << 4),
				(PMIC_BUTTON_DEBOUNCE_MASK << 4));

	/* Configure debounce time for power button 2 */
	//pmic_write_reg(REG_POWER_CTL2, (PMIC_BUTTON_DEBOUNCE_VALUE << 6),
	//			(PMIC_BUTTON_DEBOUNCE_MASK << 6));
	/* Configure debounce time for power button 3 */
	//pmic_write_reg(REG_POWER_CTL2, (PMIC_BUTTON_DEBOUNCE_VALUE << 8),
	//			(PMIC_BUTTON_DEBOUNCE_MASK << 8));
	pmic_write_reg(REG_POWER_CTL2, (0 << 3), (1 << 3));
	//pmic_write_reg(REG_POWER_CTL2, (0 << 1), (1 << 1));
	//pmic_write_reg(REG_POWER_CTL2, (0 << 2), (1 << 2));
}

void mc13892_power_off(void)
{
	unsigned int value;
	powerbutton_debounce();
	pmic_read_reg(REG_INT_SENSE0, &value, 0xffffff);
	if(!((value >> 6) & 0x1)){
	  	/* Clear bit #0 */
		pmic_write_reg(REG_MEM_A, (0 << 0), (1 << 0));	
		/* configure the PMIC for WDIRESET, power off */
		pmic_write_reg(REG_POWER_CTL2, (0 << 12), (1 << 12));
		pmic_write_reg(REG_POWER_CTL2, (1 << 0), (1 << 0));
		mxc_iomux_v3_setup_pad((MX50_PAD_WDOG__GPIO_6_28 & ~MUX_PAD_CTRL_MASK) | PAD_CTL_DSE_HIGH);
		gpio_request(WDI, "wdog_reset");
		/* Direction output */
		gpio_direction_output(WDI, 0);
		gpio_set_value(WDI, 0);
	}
	else
	{
		printk("Charging at power off state.\n");
		gpio_set_led_on();
		FRONT_LIGHT_POWER_OFF;
		power_off_flag = 1;
		/* Mark bit #0 */
		pmic_write_reg(REG_MEM_A, (1 << 0), (1 << 0));
		//arm_pm_restart('h', "restart");
	}
	/*
	pmic_read_reg(REG_POWER_CTL0, &value, 0xffffff);

	value |= 0x000008;

	pmic_write_reg(REG_POWER_CTL0, value, 0xffffff);*/
}
