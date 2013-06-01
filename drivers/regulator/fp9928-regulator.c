/* fp9928-regulator.c
 *
 * Copyright (c) 2011 Foxconn TMSBG Co., Ltd.
 *
 * Regulator driver for Fitipower 9928 EPD PMIC
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/fp9928.h>
#include <linux/gpio.h>

/* Default VCOM voltage in uV unit */
static long unsigned int fp9928_vcom = -1250000;

struct fp9928_vcom_programming_data {
	int vcom_min_uV;
	int vcom_max_uV;
	int vcom_step_uV;
};

static struct fp9928_vcom_programming_data vcom_data = {
	-4000000,
	-300000,
	10780,
};

/* Convert uV to the VCOM register setting value*/
static inline u8 vcom_uV_to_rs(int uV)
{
	return 28 + (((((vcom_data.vcom_max_uV - uV) * 10) 
					/ vcom_data.vcom_step_uV) + 5) / 10);
}

/* Convert the VCOM register value to uV */
static inline int vcom_rs_to_uV(u8 rs)
{
	return vcom_data.vcom_max_uV - 
		((rs - 28) * vcom_data.vcom_step_uV);
}

static int fp9928_vcom_set_voltage(struct regulator_dev *reg,
					int minuV, int uV)
{
	u8 reg_val;

	if ((uV < vcom_data.vcom_min_uV)
		|| (uV > vcom_data.vcom_max_uV)) {
		pr_err("Warning : supported VCOM voltage range : -4000mv ~ -300mv\n");
		return -EINVAL;
	}

	printk("%s, %d\n", __func__, uV);
	if(uV >= -2501000){
		fp9928_i2c_device_write(REG_FUNC_ADJUST, 0x01);//digital VCOM 
		reg_val = vcom_uV_to_rs(uV);
		fp9928_i2c_device_write(REG_VCOM_SETTING, reg_val);

		msleep(10);
	
		fp9928_i2c_device_read(REG_VCOM_SETTING, &reg_val);
		printk("read register raw value : %d\n", reg_val);
		printk("Set VCOM output voltage to %dmV\n", uV / 1000);
	}
	else{
		printk("Could not adjust vcom voltage via REG_VCOM_SETING register.\n");
		printk("You can adjust it by OPIN pin\n");
		fp9928_i2c_device_write(REG_FUNC_ADJUST, 0x00);//analog VCOM
	}

	return 0;
}

static int fp9928_vcom_get_voltage(struct regulator_dev *reg)
{
	u8 reg_val;
	int vol_val;

	fp9928_i2c_device_read(REG_FUNC_ADJUST, &reg_val);
	if(reg_val == 0x01){
		fp9928_i2c_device_read(REG_VCOM_SETTING, &reg_val);

		vol_val = vcom_rs_to_uV(reg_val);	
	
		if (vol_val < -2500000) {
			vol_val = vcom_data.vcom_min_uV;
		}
	}
	else
		printk("VCOM voltage from OPIN \n");
	return vol_val;
}

static int fp9928_vcom_enable(struct regulator_dev *reg)
{
	struct fp9928_data *fp9928 = rdev_get_drvdata(reg);

	/*
	 * Check to see if we need to set the VCOM voltage.
	 * Should only be done one time.
	 */
	if (!fp9928->vcom_setup) {
		fp9928_vcom_set_voltage(reg, fp9928->vcom_uV,
							    fp9928->vcom_uV);
		fp9928->vcom_setup = true;
	}

	/* enable VCOM regulator output */
	gpio_set_value(fp9928->gpio_pmic_vcom_ctrl, 1);
	
	return 0;
}

static int fp9928_vcom_disable(struct regulator_dev *reg)
{
	struct fp9928_data *fp9928 = rdev_get_drvdata(reg);
	
	gpio_set_value(fp9928->gpio_pmic_vcom_ctrl, 0);

	return 0;
}

static int fp9928_vcom_is_enabled(struct regulator_dev *reg)
{
	struct fp9928_data *fp9928 = rdev_get_drvdata(reg);

	int gpio = gpio_get_value(fp9928->gpio_pmic_vcom_ctrl);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

static int fp9928_display_enable(struct regulator_dev *reg)
{
	struct fp9928_data *fp9928 = rdev_get_drvdata(reg);
	gpio_set_value(fp9928->gpio_pmic_wakeup, 1);

	return 0;
}

static int fp9928_display_disable(struct regulator_dev *reg)
{
	struct fp9928_data *fp9928 = rdev_get_drvdata(reg);
	gpio_set_value(fp9928->gpio_pmic_wakeup, 0);

	msleep(fp9928->max_wait);

	return 0;
}

static int fp9928_display_is_enabled(struct regulator_dev *reg)
{
	struct fp9928_data *fp9928 = rdev_get_drvdata(reg);
	
	int gpio = gpio_get_value(fp9928->gpio_pmic_wakeup);

	if (gpio == 0)
		return 0;
	else
		return 1;
}

/*
 * Regulator operations
 */
static struct regulator_ops fp9928_display_ops = {
	.enable 	= fp9928_display_enable,
	.disable	= fp9928_display_disable,
	.is_enabled = fp9928_display_is_enabled,
};

static struct regulator_ops fp9928_vcom_ops = {
	.enable 	 = fp9928_vcom_enable,
	.disable	 = fp9928_vcom_disable,
	.get_voltage = fp9928_vcom_get_voltage,
	.set_voltage = fp9928_vcom_set_voltage,
	.is_enabled  = fp9928_vcom_is_enabled,
};

static struct regulator_desc fp9928_regulator[FP9928_NUM_REGULATORS] = {
	{
		.name = "DISPLAY",
		.id   = FP9928_DISPLAY,
		.ops  = &fp9928_display_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "VCOM",
		.id   = FP9928_VCOM,
		.ops  = &fp9928_vcom_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int fp9928_regulator_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	
	pr_info("%s\n", __func__);
	
	rdev = regulator_register(&fp9928_regulator[pdev->id], &pdev->dev,
				  pdev->dev.platform_data,
				  dev_get_drvdata(&pdev->dev));
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			fp9928_regulator[pdev->id].name);
		return PTR_ERR(rdev);
	}

	platform_set_drvdata(pdev, rdev);
	
	return 0;
}

static int fp9928_regulator_remove(struct platform_device *pdev)
{
	struct regulator_dev *rdev = platform_get_drvdata(pdev);
	regulator_unregister(rdev);
	return 0;
}

int fp9928_register_regulator(struct fp9928_data *fp9928_dev, int id,
				     	struct regulator_init_data *initdata)
{
	struct platform_device *pdev;
	int ret;
	
	if (!fp9928_dev || !fp9928_dev->i2c_client)
		return -ENODEV;

	if (fp9928_dev->pdev[id])
		return -EBUSY;

	pdev = platform_device_alloc("fp9928-regulator", id);
	if (!pdev) {
		dev_err(fp9928_dev->dev,
		       "Failed to alloc platform device fp9928-regulator.%d\n",	id);
		return -ENOMEM;
	}
	
	fp9928_dev->pdev[id] = pdev;

	initdata->driver_data = fp9928_dev;

	pdev->dev.platform_data = initdata;
	pdev->dev.parent = fp9928_dev->dev;
	platform_set_drvdata(pdev, fp9928_dev);

	ret = platform_device_add(pdev);

	if (ret != 0) {
		dev_err(fp9928_dev->dev,
		       "Failed to register regulator %d: %d\n",
			id, ret);
		platform_device_del(pdev);
		fp9928_dev->pdev[id] = NULL;
	}

	printk("fp9928_vcom value from uboot is %d\n", fp9928_vcom);
	if (!fp9928_dev->init_done) {
		fp9928_dev->vcom_uV = fp9928_vcom;
		fp9928_dev->init_done = true;
	}

	return ret;
}
EXPORT_SYMBOL(fp9928_register_regulator);

static struct platform_driver fp9928_regulator_driver = {
	.driver = {
		.name = "fp9928-regulator",
	},
	.probe  = fp9928_regulator_probe,
	.remove = fp9928_regulator_remove,
};

static int __init fp9928_regulator_init(void)
{
	pr_info("%s\n", __func__);
	return platform_driver_register(&fp9928_regulator_driver);
}
subsys_initcall(fp9928_regulator_init);

static void __exit fp9928_regulator_exit(void)
{
	platform_driver_unregister(&fp9928_regulator_driver);
}
module_exit(fp9928_regulator_exit);

/*
 * Parse user specified options (`vcom=')
 * Example:
 * 		vcom=-1250000
 * Note: 
 * 		vcom options in uV uint
 */
static int __init fp9928_setup(char *opt)
{
	int ret;
	int offs = 0;

	printk("opt is %s\n", opt);
	if (opt[0] == '-')
		offs = 1;
		
	ret = strict_strtoul(opt + offs, 0, &fp9928_vcom);
	if (ret < 0)
		return ret;
		
	fp9928_vcom = -fp9928_vcom;

	return 1;
}
__setup("vcom=", fp9928_setup);

MODULE_AUTHOR("Jerry, Luo <luokun237234@gmail.com>");
MODULE_DESCRIPTION("fp9928 voltage regulator driver");
MODULE_LICENSE("GPL");
