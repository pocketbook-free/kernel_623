/*
 * Copyright 2009-2011 Freescale Semiconductor, Inc. All Rights Reserved.
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

/*
 * Includes
 */
//#define DEBUG 
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/mach-types.h>
#include <linux/pmic_battery.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_status.h>
#include <linux/reboot.h>
#include <linux/pmic_external.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <mach/iomux-mx50.h>
#include <linux/led/pmic_leds.h>
#include <linux/ep7a/charge_core.h>
#include <linux/hw_ops.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/rtc.h>

int power_off_flag = 0;
int bind_flag = 0;
EXPORT_SYMBOL(bind_flag);
int  usb_adapter_state(void);
//#define EP_TEST
#ifdef EP_TEST
	#define TEST(x, args...) 	printk(x, ##args)
#else
	#define TEST(x, args...)	NULL 
#endif	

#define UART2_RTS (5*32 + 13) /*GPIO_6_13*/

#define BIT_CHG_VOL_LSH		0
#define BIT_CHG_VOL_WID		3

#define BIT_CHG_CURR_LSH		3
#define BIT_CHG_CURR_WID		4

#define BIT_CHG_PLIM_LSH		15
#define BIT_CHG_PLIM_WID		2

#define BIT_CHG_DETS_LSH 6
#define BIT_CHG_DETS_WID 1
#define BIT_CHG_CURRS_LSH 11
#define BIT_CHG_CURRS_WID 1

#define BIT_CHG_LEDEN_LSH 18
#define BIT_CHG_LEDEN_WID 1

#define BIT_CHG_SELECT_LSH 10
#define BIT_CHG_SELECT_WID 1

#define BIT_CHG_FAULT_LSH 8
#define BIT_CHG_FAULT_WID 2

#define TRICKLE_CHG_EN_LSH	7
#define	LOW_POWER_BOOT_ACK_LSH	8
#define BAT_TH_CHECK_DIS_LSH	9
#define	BATTFET_CTL_EN_LSH	10
#define BATTFET_CTL_LSH		11
#define	REV_MOD_EN_LSH		13
#define PLIM_DIS_LSH		17
#define	CHG_LED_EN_LSH		18
#define CHGTMRRST_LSH		19
#define RESTART_CHG_STAT_LSH	20
#define	AUTO_CHG_DIS_LSH	21
#define CYCLING_DIS_LSH		22
#define	VI_PROGRAM_EN_LSH	23

#define TRICKLE_CHG_EN_WID	1
#define	LOW_POWER_BOOT_ACK_WID	1
#define BAT_TH_CHECK_DIS_WID	1
#define	BATTFET_CTL_EN_WID	1
#define BATTFET_CTL_WID		1
#define	REV_MOD_EN_WID		1
#define PLIM_DIS_WID		1
#define	CHG_LED_EN_WID		1
#define CHGTMRRST_WID		1
#define RESTART_CHG_STAT_WID	1
#define	AUTO_CHG_DIS_WID	1
#define CYCLING_DIS_WID		1
#define	VI_PROGRAM_EN_WID	1

#define ACC_STARTCC_LSH		0
#define ACC_STARTCC_WID		1
#define ACC_RSTCC_LSH		1
#define ACC_RSTCC_WID		1
#define ACC_CCFAULT_LSH		7
#define ACC_CCFAULT_WID		7
#define ACC_CCOUT_LSH		8
#define ACC_CCOUT_WID		16
#define ACC1_ONEC_LSH		0
#define ACC1_ONEC_WID		15

#define ACC_CALIBRATION 0x17
#define ACC_START_COUNTER 0x07
#define ACC_STOP_COUNTER 0x2
#define ACC_CONTROL_BIT_MASK 0x1f
#define ACC_ONEC_VALUE 2621
#define ACC_COULOMB_PER_LSB 1
#define ACC_CALIBRATION_DURATION_MSECS 20

#define BAT_VOLTAGE_UNIT_UV (4800000/1023)
#define BAT_CURRENT_UNIT_UA (3000000/511)
#define CHG_VOLTAGE_UINT_UV 23474
#define CHG_MIN_CURRENT_UA 3500

//ADD FOR SCP
#define SCP_EN_LSH      0
#define SCP_EN_WID      1
#define VGEN1_ENABLE_LSH	0
#define VGEN1_ENABLE_WID	1
#define VGEN2_ENABLE_LSH	12
#define VGEN2_ENABLE_WID	1

#define COULOMB_TO_UAH(c) (10000 * c / 36)

/* Add pmic charger time-out 4h, the total charing time should not exceed 6h */
//#define BAT_CAP_MAH 1200UL
#define BAT_CAP_MAH 2400UL //8h
#define CHG_CUR_MA 300UL   

static pmic_event_callback_t bat_event_callback;
static pmic_event_callback_t chgfault_event_callback;
static pmic_event_callback_t sc_event_callback;

enum chg_setting {
       SCP_EN,
       TRICKLE_CHG_EN,
       LOW_POWER_BOOT_ACK,
       BAT_TH_CHECK_DIS,
       BATTFET_CTL_EN,
       BATTFET_CTL,
       REV_MOD_EN,
       PLIM_DIS,
       CHG_LED_EN,
       CHGTMRRST,
       RESTART_CHG_STAT,
       AUTO_CHG_DIS,
       CYCLING_DIS,
       VI_PROGRAM_EN,
};

enum chg_state {
	CHG_POWER_OFF,
	CHG_RESTART,
	CHG_DISCHARGING,
	CHG_DISCHARGING_WITH_CHARGER,
	CHG_CHARGING,
};
struct charge_info ep7_charge_info;

static int usb_adapter_det = 4;
static int chg_value = 1;

/* Flag used to indicate if Charger workaround is active. */
int chg_wa_is_active;
/* Flag used to indicate if Charger workaround timer is on. */
int chg_wa_timer;
int disable_chg_timer;
struct workqueue_struct *chg_wq;
struct delayed_work chg_work;
static unsigned long expire;
static int state=CHG_RESTART;
static int suspend_flag;
//static int pmic_set_chg_current(unsigned short curr)
int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	return 0;
}
EXPORT_SYMBOL(pmic_set_chg_current);

static int pmic_set_scp(enum chg_setting type, unsigned short flag)
{
	unsigned int reg_value = 0;
	unsigned int mask =0;
	switch (type) {
	case SCP_EN:
		reg_value = BITFVAL(SCP_EN, flag);
		mask = BITFMASK(SCP_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}
	CHECK_ERROR(pmic_write_reg(REG_POWER_MISC, reg_value, mask));
	return 0;
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{

	unsigned int reg_value = 0;
	unsigned int mask = 0;

	switch (type) {	
	case TRICKLE_CHG_EN:
		reg_value = BITFVAL(TRICKLE_CHG_EN, flag);
		mask = BITFMASK(TRICKLE_CHG_EN);
		break;
	case LOW_POWER_BOOT_ACK:
		reg_value = BITFVAL(LOW_POWER_BOOT_ACK, flag);
		mask = BITFMASK(LOW_POWER_BOOT_ACK);
		break;
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case BATTFET_CTL_EN:
		reg_value = BITFVAL(BATTFET_CTL_EN, flag);
		mask = BITFMASK(BATTFET_CTL_EN);
		break;
	case BATTFET_CTL:
		reg_value = BITFVAL(BATTFET_CTL, flag);
		mask = BITFMASK(BATTFET_CTL);
		break;
	case REV_MOD_EN:
		reg_value = BITFVAL(REV_MOD_EN, flag);
		mask = BITFMASK(REV_MOD_EN);
		break;
	case PLIM_DIS:
		reg_value = BITFVAL(PLIM_DIS, flag);
		mask = BITFMASK(PLIM_DIS);
		break;
	case CHG_LED_EN:
		reg_value = BITFVAL(CHG_LED_EN, flag);
		mask = BITFMASK(CHG_LED_EN);
		break;
	case CHGTMRRST:
		reg_value = BITFVAL(CHGTMRRST, flag);
		mask = BITFMASK(CHGTMRRST);
		break;
	case RESTART_CHG_STAT:
		reg_value = BITFVAL(RESTART_CHG_STAT, flag);
		mask = BITFMASK(RESTART_CHG_STAT);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case CYCLING_DIS:
		reg_value = BITFVAL(CYCLING_DIS, flag);
		mask = BITFMASK(CYCLING_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));

	return 0;
}

static int pmic_get_batt_voltage(unsigned short *voltage)
{
	t_channel channel;
	unsigned short result[8];

	channel = BATTERY_VOLTAGE;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	*voltage = result[0];

	return 0;
}

static int pmic_get_batt_current(signed short *curr)
{
	t_channel channel;
	signed short result[8];
	bool valid_ch[8] = {1,0,1,0,0,1,0,1};
	int i;

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));

	//pr_notice("ADC  %03x %03x  %03x %03x  %03x %03x  %03x %03x\n",
	//	result[0], result[1], result[2], result[3],
	//	result[4], result[5], result[6], result[7]);

	*curr = 0;
	for(i=0;i<8;i++)
		if(valid_ch[i])
			*curr += (result[i]&0x200) ? (0xffc00|result[i]) : result[i];
	*curr /= 4;

	return 0;
}

static int coulomb_counter_calibration;
static unsigned int coulomb_counter_start_time_msecs;

static int pmic_start_coulomb_counter(void)
{
	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		ACC_COULOMB_PER_LSB * ACC_ONEC_VALUE, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_START_COUNTER, ACC_CONTROL_BIT_MASK));
	coulomb_counter_start_time_msecs = jiffies_to_msecs(jiffies);
	pr_debug("coulomb counter start time %u\n",
		coulomb_counter_start_time_msecs);
	return 0;
}

static int pmic_stop_coulomb_counter(void)
{
	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_STOP_COUNTER, ACC_CONTROL_BIT_MASK));
	return 0;
}

static int pmic_calibrate_coulomb_counter(void)
{
	int ret;
	unsigned int value;

	/* set scaler */
	CHECK_ERROR(pmic_write_reg(REG_ACC1,
		0x1, BITFMASK(ACC1_ONEC)));

	CHECK_ERROR(pmic_write_reg(
		REG_ACC0, ACC_CALIBRATION, ACC_CONTROL_BIT_MASK));
	msleep(ACC_CALIBRATION_DURATION_MSECS);

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	pr_debug("calibrate value = %x\n", value);
	coulomb_counter_calibration = (int)((s16)((u16) value));
	pr_debug("coulomb_counter_calibration = %d\n",
		coulomb_counter_calibration);

	return 0;
}

static int pmic_get_charger_coulomb(int *coulomb)
{
	int ret;
	unsigned int value;
	int calibration;
	unsigned int time_diff_msec;

	ret = pmic_read_reg(REG_ACC0, &value, BITFMASK(ACC_CCOUT));
	if (ret != 0)
		return -1;
	value = BITFEXT(value, ACC_CCOUT);
	//pr_debug("counter value = %x\n", value);
	*coulomb = ((s16)((u16)value)) * ACC_COULOMB_PER_LSB;

	if (abs(*coulomb) >= ACC_COULOMB_PER_LSB) {
			/* calibrate */
		time_diff_msec = jiffies_to_msecs(jiffies);
		time_diff_msec =
			(time_diff_msec > coulomb_counter_start_time_msecs) ?
			(time_diff_msec - coulomb_counter_start_time_msecs) :
			(0xffffffff - coulomb_counter_start_time_msecs
			+ time_diff_msec);
		calibration = coulomb_counter_calibration * (int)time_diff_msec
			/ (ACC_ONEC_VALUE * ACC_CALIBRATION_DURATION_MSECS);
		*coulomb -= calibration;
	}

	return 0;
}

static void init_charger_timer(void)
{
	pmic_set_chg_misc(CHGTMRRST, 1);
	expire = jiffies + ((BAT_CAP_MAH*3600UL*HZ)/CHG_CUR_MA);
}

static bool charger_timeout(void)
{
	return time_after(jiffies, expire);
}
 
static void reset_charger_timer(void)
{
	if(!charger_timeout())
		pmic_set_chg_misc(CHGTMRRST, 1);
}

static int pmic_restart_charging(void)
{
	//pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);
	pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);
	pmic_set_chg_misc(RESTART_CHG_STAT, 1);
	pmic_set_chg_misc(PLIM_DIS, 1);
	return 0;
}

struct mc13892_dev_info {
	struct device *dev;

	unsigned short voltage_raw;
	int voltage_uV;
	signed short current_raw;
	int current_uA;
	int battery_status;
	int full_counter;
	int charger_online;
	int usb_charger_online;
	int adapter_charger_online;
	int charger_voltage_uV;
	int accum_current_uAh;

	struct power_supply bat;
	struct power_supply charger;
	struct power_supply usb;
	struct power_supply adapter;

	struct workqueue_struct *monitor_wqueue;
	struct delayed_work monitor_work;
};

#define to_mc13892_dev_info(x) container_of((x), struct mc13892_dev_info, \
					      bat);

static enum power_supply_property mc13892_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_STATUS,
};

static enum power_supply_property mc13892_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

/*
static int pmic_get_chg_value(unsigned int *value)
{
	t_channel channel;
	unsigned short result[8], max1 = 0, min1 = 0, max2 = 0, min2 = 0, i;
	unsigned int average = 0, average1 = 0, average2 = 0;

	channel = CHARGE_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));


	for (i = 0; i < 8; i++) {
		if ((result[i] & 0x200) != 0) {
			result[i] = 0x400 - result[i];
			average2 += result[i];
			if ((max2 == 0) || (max2 < result[i]))
				max2 = result[i];
			if ((min2 == 0) || (min2 > result[i]))
				min2 = result[i];
		} else {
			average1 += result[i];
			if ((max1 == 0) || (max1 < result[i]))
				max1 = result[i];
			if ((min1 == 0) || (min1 > result[i]))
				min1 = result[i];
		}
	}

	if (max1 != 0) {
		average1 -= max1;
		if (max2 != 0)
			average2 -= max2;
		else
			average1 -= min1;
	} else
		average2 -= max2 + min2;

	if (average1 >= average2) {
		average = (average1 - average2) / 6;
		*value = average;
	} else {
		average = (average2 - average1) / 6;
		*value = ((~average) + 1) & 0x3FF;
	}

	return 0;
}*/

int get_battery_mA(void) /* get charging current float into battery */
 {
	signed short value;
	int bat_curr;
 
	pmic_get_batt_current(&value);
	bat_curr = (value*3000)/511;
	//pr_notice("%s %d\n", __func__, -bat_curr);
	return -bat_curr;
}
 
int get_battery_mV(void)
{
	unsigned short value = 0;
	pmic_get_batt_voltage(&value);
	//pr_notice("%s %d\n", __func__, value*4800/1023);
	return(value*4800/1023);
}


static int charger_detect(void)
{
	int ret;
	unsigned int value;
	int chrgse1b;
	ret = pmic_read_reg(REG_PU_MODE_S, &value, BITFMASK(BIT_CHG_SELECT));
	if (ret == 0)
	{
		chrgse1b = BITFEXT(value, BIT_CHG_SELECT);
		if(0 == chrgse1b){
			//TEST(KERN_DEBUG "### %s:Wall charging ###\n",__func__);
		}
		else if(1 == chrgse1b){
			//TEST(KERN_DEBUG "### %s:USB charging or abnormal wall charging###\n",__func__);
		}
		return chrgse1b;
	}
	TEST(" charger detect fail:%d\n", ret);
	return ret;
}

/*********************************
*	return		
*	0:USB charging 
*	2:USB 3rd adapter charging
*	3:Wall charging
*	4:Discharging
**********************************/
static int bsp_charger_detect(void)
{	
	int usb_online = 0,usb_status = 0; 
	unsigned int value;
	pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
	usb_online = BITFEXT(value, BIT_CHG_DETS);
	if (0 == usb_online)
	{
		TEST("### %s:usb offline ###\n", __func__);
		return USB_OFFLINE;//return 4
	}
	
	if(0 == charger_detect())
	{
		TEST("### %s:Wall charging ###\n", __func__);
		return  ADAPTER_THIRDPARTY_INSERT;//return 3
	}

	if(1 == charger_detect())	
	{
		usb_status = usb_adapter_state();
		if(0 == usb_status){
			TEST("### %s:USB charging ###\n", __func__);
			return  usb_status;//return 0
		}
		if(2 == usb_status){
			TEST("### %s:Wall charger ###\n", __func__);
			return  usb_status;//return 2
		}
		TEST(" Unknown device charging:%d \n", usb_status);
		return  usb_status;
	}
}
	

static void chg_thread(struct work_struct *work)
{
	unsigned int value = 0;
	int charger;
	int charge_current, voltage;
	static int charge_full_count=0;
	static int charge_restart_count=0;
	static int power_off_count=0;
	
	#ifdef DEBUG
	//struct timespec ts;
	//struct rtc_time tm;
	#endif
	
	pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
	charger = BITFEXT(value, BIT_CHG_DETS);
	switch(state)
	{
	  case CHG_RESTART:
		pmic_restart_charging();
		pmic_set_chg_current(0x0);
		TEST(KERN_DEBUG "%s:PMIC restart charging.\n",__FUNCTION__);  //for debug
		if(charger){
			init_charger_timer();
			TEST("**** usb_adapter_det=%d; bind_flag=%d  ****\n", usb_adapter_det, bind_flag);  //for debug
			if(2 == usb_adapter_det || 3 == usb_adapter_det)
			{
				if(1 == bind_flag)
				{
					pmic_set_chg_current(0x2);
					TEST("Warnning:Adapter charging but insert usb module, set charging current 240mA.\n");
				}
				else
				{
					pmic_set_chg_current(0x9);
					TEST("Debug:PMIC set Wall charging current 800mA.\n");
				}
			}
			else if(0 == usb_adapter_det)
			{
				if(0 == bind_flag)
				{
					pmic_set_chg_current(0x4); //400mA usb charging
					TEST("Debug:PMIC set USB charging current 400mA.\n");	
				}
				else if(1 == bind_flag)
				{
					pmic_set_chg_current(0x2); //240mA bind g_file_storage charging
					TEST("Debug:PMIC set USB File-backed Storage Gadget charging current 240mA.\n");	
				}
			}
			else{
					  #ifdef DEBUG
					    pr_info("Unkown charger type, set charge current to default 400mA\n");
					  #endif
				pmic_set_chg_current(0x4);
				TEST("Debug:PMIC set unknown device charging current 400mA.\n");	
			}	
			state = CHG_CHARGING;
			if(ep7_charge_info.battery_level_curr_status != BAT_CHARGEFULL)
			  ep7_charge_info.battery_level_curr_status = BAT_CHARGE_WITH_CHARGER;
		}
		else
		{
			pmic_set_chg_current(0);
			state = CHG_DISCHARGING;
			ep7_charge_info.battery_level_curr_status = BAT_DISCHARGE_WITHOUT_CHARGER;
		}
		  
			queue_delayed_work(chg_wq, &chg_work, HZ*1);
			break;

	case CHG_POWER_OFF:
		//pr_notice("Battery level < 3.5V!\n");
		//pr_notice("After power off, PMIC will charge up battery.\n");
		//pmic_set_chg_current(0x4); /* charge battery during power off */
		//orderly_poweroff(1);
		pm_power_off();
		break;
 
	case CHG_CHARGING:		
		reset_charger_timer();
		charge_current=get_battery_mA();
		voltage=get_battery_mV();
		TEST(KERN_DEBUG "current=%d, voltage=%d\n",charge_current, voltage);
		#if 0 
		  getnstimeofday(&ts);
	    rtc_time_to_tm(ts.tv_sec, &tm);
	    TEST(KERN_DEBUG "Now time is "
		  "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)", 
		  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
		#endif
		//if((get_battery_mV() > 4080) && (charge_current < 80)){		
		//if((get_battery_mV() > 4080) && (charger_timeout() || (charge_current < 80))){
			/*if(charger_timeout())
			{
			  printk("%s:charger timeout\n",__FUNCTION__);  //for debug
			  pmic_set_chg_current(0);
			  state = CHG_DISCHARGING_WITH_CHARGER;
			}*/
		if(1 == bind_flag)
		{		
			printk("current=%d, voltage=%d, flag=%d, detect=%d\n",charge_current, voltage, bind_flag, usb_adapter_det);
			if(charge_current > 0 && charge_current < 60 && voltage > 4180)
			{
				charge_full_count++;
				printk(KERN_DEBUG "UMS:charge_current < 60mA && voltage > 4180mV, count=%d\n", charge_full_count);
				if(charge_full_count>=10){
					charge_full_count=0;
					state = CHG_DISCHARGING_WITH_CHARGER;
					ep7_charge_info.battery_level_curr_status = BAT_CHARGEFULL;
					pmic_set_chg_current(0);
					hw_led_onoff(0);
					if(power_off_flag == 1)
						gpio_set_led_off();
				    	printk(KERN_DEBUG "charge_status=charge_full\n");
				}
			}
			else
			{
				charge_full_count = 0;
			}
		}
		else
		{
			printk("current=%d, voltage=%d, flag=%d, detect=%d\n",charge_current, voltage, bind_flag, usb_adapter_det);
			if(charge_current < 80 && charge_current > 0)
                        {
                                charge_full_count++;
                                printk(KERN_DEBUG "Charge:charge_current < 80mA, count=%d\n", charge_full_count);
                                if(charge_full_count>=10){
                                        charge_full_count=0;
                                        state = CHG_DISCHARGING_WITH_CHARGER;
                                        ep7_charge_info.battery_level_curr_status = BAT_CHARGEFULL;
                                        pmic_set_chg_current(0);
					hw_led_onoff(0);
					if(power_off_flag == 1)
						gpio_set_led_off();
                                        printk(KERN_DEBUG "charge_status=charge_full\n");
                                }
                        }
			else
			{
				charge_full_count = 0;
			}
		}
		
			
		if(!charger){
			charge_full_count=0;
			pmic_set_chg_current(0);
			state = CHG_DISCHARGING;
			ep7_charge_info.battery_level_curr_status = BAT_DISCHARGE_WITHOUT_CHARGER;
			printk(KERN_DEBUG "%s:discharge without charger\n",__FUNCTION__);
		}
		
		queue_delayed_work(chg_wq, &chg_work, HZ*2);
		break;

	case CHG_DISCHARGING:
		if(get_battery_mV() < 3500)
		{
			ep7_charge_info.battery_level_curr_status = BAT_CRITICALLOW;
			printk("--------------- critical low voltage ------------\n");
		  	power_off_count++;
			if(power_off_count >= 5){
			  power_off_count=0;
			  state = CHG_POWER_OFF;
			  printk(KERN_DEBUG "%s:power off\n",__FUNCTION__);}
		}
		else
		{
			power_off_count=0;
			ep7_charge_info.battery_level_curr_status = BAT_DISCHARGE_WITHOUT_CHARGER;	
		}

		if(charger){
			power_off_count = 0;
		  state = CHG_RESTART;
		  ep7_charge_info.battery_level_curr_status = BAT_CHARGE_WITH_CHARGER;
		}
		
		queue_delayed_work(chg_wq, &chg_work, HZ*5);
		break;

	case CHG_DISCHARGING_WITH_CHARGER:
		voltage = get_battery_mV();
		charge_current = get_battery_mA();
		printk(KERN_DEBUG "voltage=%d, current=%d\n",voltage, charge_current);
		if(power_off_flag == 1)
			gpio_set_led_off();

		#if 0 
		  getnstimeofday(&ts);
	    rtc_time_to_tm(ts.tv_sec, &tm);
	    printk(KERN_DEBUG "Now time is "
		  "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)", 
		  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		  tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
		#endif
			
		if(voltage < 4110)
		{
			charge_restart_count++;
			printk(KERN_DEBUG "battery voltage < 4110,charge_restart_count=%d\n",charge_restart_count);
			if(charge_restart_count>=10)
			{
				charge_restart_count=0;
				state = CHG_RESTART;
				printk(KERN_DEBUG "run restart charge routine\n");
			}	
		}
		else
			charge_restart_count=0;
	
		if(!charger)
		{
			charge_restart_count=0;
			state = CHG_DISCHARGING;
		}
		queue_delayed_work(chg_wq, &chg_work, HZ*2);
		break;
 	}
	//pr_notice("charger state %d %c\n", state, charger?'Y':'N');
}

static int mc13892_charger_update_status(struct mc13892_dev_info *di)
{
	int ret;
	unsigned int value;
	int online;
	//int old_adapter_charger_online, old_usb_charger_online;
	
	//old_adapter_charger_online = di->adapter_charger_online;
	//old_usb_charger_online = di->usb_charger_online;

	ret = pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
	if (ret == 0)
	{
		online = BITFEXT(value, BIT_CHG_DETS);
		if (online != di->charger_online) {
			di->charger_online = online;
			dev_info(di->charger.dev, "charger status: %s\n",
				online ? "online" : "offline");
					
			if (online == 0)
			{
				/*used for poweroff charging in kernel, when usb/adapter plugging out, enter real power off*/
				pmic_read_reg(REG_MEM_A, &value, 0xffffff);
				if(value & 0x1)
				{
					pmic_write_reg(REG_MEM_A, (0 << 0), (1 << 0));
					pm_power_off();
				}

				hw_set_ac(0);
				hw_led_onoff(1);
				//di->adapter_charger_online = 0;
				//di->usb_charger_online = 0;
				if(chg_value == 0)
					queue_delayed_work(chg_wq, &chg_work, HZ);
				chg_value = 1;
			}
			else if(online == 1)
			{
				 if((2 == usb_adapter_det) || (3 == usb_adapter_det))
				 	 hw_set_ac(1);
				 /*if((charger_detect() == 0))
				 {	
				   hw_set_ac(1);
				   di->adapter_charger_online = 1;}
				 else if((charger_detect() == 1))
					   di->usb_charger_online = 1;*/
			}
			  
		 	 power_supply_changed(&di->charger);
			
			/*if(di->adapter_charger_online != old_adapter_charger_online)
				power_supply_changed(&di->adapter);
			if(di->usb_charger_online != old_usb_charger_online)
      	power_supply_changed(&di->usb);*/
      
			cancel_delayed_work(&di->monitor_work);
			queue_delayed_work(di->monitor_wqueue,
				&di->monitor_work, HZ / 10);
			if (online)
			{
				pmic_start_coulomb_counter();
				chg_wa_timer = 1;
			} else {
				chg_wa_timer = 0;
				pmic_stop_coulomb_counter();
		}
	}
	}

	return ret;
}

static int mc13892_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di = container_of((psy), struct mc13892_dev_info, charger);
	switch (psp) {
	  case POWER_SUPPLY_PROP_ONLINE:
		  val->intval = di->charger_online;
		  return 0;
	  default:
		  break;
	}
	return -EINVAL;
}

static int mc13892_usb_charger_get_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val)
{
	//struct mc13892_dev_info *di = container_of((psy), struct mc13892_dev_info, usb);
	switch (psp) {		
    case POWER_SUPPLY_PROP_ONLINE:
			   //val->intval = di->usb_charger_online;
			   if(0 == usb_adapter_det)
			     val->intval = 1;
			   else 
			   	 val->intval = 0;
			  return 0;           
    default:
      break;}
  return -EINVAL;
}

static int mc13892_adapter_charger_get_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val)
{
  //struct mc13892_dev_info *di = container_of((psy), struct mc13892_dev_info, adapter);
  switch(psp){
    case POWER_SUPPLY_PROP_ONLINE:
		  //val->intval = di->adapter_charger_online;
		  if((2 == usb_adapter_det) || (3 == usb_adapter_det))
		    val->intval = 1;
			else 
			  val->intval = 0;		  	
      return 0;
    default:
      break;
  }
  return -EINVAL;
}
static int mc13892_battery_read_status(struct mc13892_dev_info *di)
{
	int retval;
	int coulomb;
	retval = pmic_get_batt_voltage(&(di->voltage_raw));
	if (retval == 0)
		di->voltage_uV = di->voltage_raw * BAT_VOLTAGE_UNIT_UV;

	retval = pmic_get_batt_current(&(di->current_raw));
	if (retval == 0)
			di->current_uA = di->current_raw * BAT_CURRENT_UNIT_UA;
	retval = pmic_get_charger_coulomb(&coulomb);
	if (retval == 0)
		di->accum_current_uAh = COULOMB_TO_UAH(coulomb);

	return retval;
}

static void mc13892_battery_update_status(struct mc13892_dev_info *di)
{
	unsigned int value;
	int retval;
	int old_battery_status = di->battery_status;

	if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN)
		di->full_counter = 0;

	if (di->charger_online) {
		retval = pmic_read_reg(REG_INT_SENSE0,
					&value, BITFMASK(BIT_CHG_CURRS));

		if (retval == 0) {
			value = BITFEXT(value, BIT_CHG_CURRS);
			if (value)
				di->battery_status =
					POWER_SUPPLY_STATUS_CHARGING;
			else
				di->battery_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		if (di->battery_status == POWER_SUPPLY_STATUS_NOT_CHARGING)
			di->full_counter++;
		else
			di->full_counter = 0;
	} else {
		di->battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
		di->full_counter = 0;
	}

	//dev_dbg(di->bat.dev, "bat status: %d\n",
	//	di->battery_status);

	if (old_battery_status != POWER_SUPPLY_STATUS_UNKNOWN &&
		di->battery_status != old_battery_status)
		power_supply_changed(&di->bat);
}

static void mc13892_battery_work(struct work_struct *work)
{
	struct mc13892_dev_info *di = container_of(work,
						     struct mc13892_dev_info,
						     monitor_work.work);
	const int interval = HZ * 2;

	//dev_dbg(di->dev, "%s\n", __func__);

	mc13892_battery_update_status(di);
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, interval);
}

static void short_circuit_event_callback(void *para)
{
	//struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	unsigned int value, reg_value, mask;
	pmic_read_reg(REG_MODE_0, &value, BITFMASK(VGEN1_ENABLE));
	reg_value = BITFVAL(VGEN1_ENABLE, 0);
	mask = BITFMASK(VGEN1_ENABLE);
	CHECK_ERROR(pmic_write_reg(REG_MODE_0, reg_value, mask));
	reg_value = BITFVAL(VGEN2_ENABLE, 0);
	mask = BITFMASK(VGEN2_ENABLE);
	CHECK_ERROR(pmic_write_reg(REG_MODE_0, reg_value, mask));
	pmic_read_reg(REG_MODE_0, &value, BITFMASK(VGEN1_ENABLE));
}

static void charger_online_event_callback(void *para)
{
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	pr_info("\n%s\n", __func__);
	usb_adapter_det = bsp_charger_detect();
	mc13892_charger_update_status(di);
}

int charger_fault = 0;
EXPORT_SYMBOL(charger_fault);
static void charger_fault_event_callback(void *para)
{
	#ifdef DEBUG
	struct mc13892_dev_info *di = (struct mc13892_dev_info *) para;
	#endif
	unsigned int value;
  pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_FAULT));
	charger_fault = BITFEXT(value, BIT_CHG_FAULT);
	
	#ifdef DEBUG
	printk(KERN_DEBUG "-----------------%s-----: 0x%x\n",__func__, charger_fault);
	switch(charger_fault){
		case 0x1:	
			  printk(KERN_DEBUG "Charger fault:=0x%x:charger path over voltage.\n", charger_fault);
		  break;
		case 0x2: 
			if(di->charger_online)
		    printk(KERN_DEBUG "Charger fault:=0x%x:battery dies/charger times out.\n", charger_fault);
		  else
		  	charger_fault = 0;	
		  break;
		case 0x3:	
			printk(KERN_DEBUG "Charger fault:=0x%x:battery out of temperature.\n", charger_fault);
		  break;
		default:
			;
		}	
	#endif
}

static int mc13892_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mc13892_dev_info *di = to_mc13892_dev_info(psy);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (di->battery_status == POWER_SUPPLY_STATUS_UNKNOWN) {
			mc13892_charger_update_status(di);
			mc13892_battery_update_status(di);
		}
		val->intval = di->battery_status;
		return 0;
	default:
		break;
	}

	mc13892_battery_read_status(di);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->accum_current_uAh;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 3800000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3300000;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static ssize_t chg_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (chg_value)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static ssize_t chg_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if ((strstr(buf, "1") != NULL) && (chg_value == 0)) {
			queue_delayed_work(chg_wq, &chg_work, HZ);
			state = CHG_RESTART;
			chg_value = 1;
	} else if (strstr(buf, "0") != NULL) {
			cancel_rearming_delayed_workqueue(chg_wq, &chg_work);
			pmic_set_chg_current(0);
			chg_value = 0;	
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, chg_enable_show, chg_enable_store);
/*
static ssize_t chg_wa_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (chg_wa_is_active & chg_wa_timer)
		return sprintf(buf, "Charger LED workaround timer is on\n");
	else
		return sprintf(buf, "Charger LED workaround timer is off\n");
}

static ssize_t chg_wa_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	if (strstr(buf, "1") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer)
				printk(KERN_INFO "Charger timer is already on\n");
			else {
				chg_wa_timer = 1;
				printk(KERN_INFO "Turned on the timer\n");
			}
		}
	} else if (strstr(buf, "0") != NULL) {
		if (chg_wa_is_active) {
			if (chg_wa_timer) {
				chg_wa_timer = 0;
				printk(KERN_INFO "Turned off charger timer\n");
			 } else {
				printk(KERN_INFO "The Charger workaround timer is off\n");
			}
		}
	}

	return size;
}

static DEVICE_ATTR(enable, 0644, chg_wa_enable_show, chg_wa_enable_store);*/

static int pmic_battery_remove(struct platform_device *pdev)
{
	struct mc13892_dev_info *di = platform_get_drvdata(pdev);

	cancel_rearming_delayed_workqueue(di->monitor_wqueue,
					  &di->monitor_work);
	cancel_rearming_delayed_workqueue(chg_wq,
					  &chg_work);
	destroy_workqueue(di->monitor_wqueue);
	destroy_workqueue(chg_wq);
	chg_wa_timer = 0;
	chg_wa_is_active = 0;
	disable_chg_timer = 0;
	power_supply_unregister(&di->bat);
	power_supply_unregister(&di->charger);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->adapter);
	
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_CHGDETI, bat_event_callback));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_CHGFAULTI, chgfault_event_callback));
	CHECK_ERROR(pmic_event_unsubscribe(EVENT_CHGFAULTI, sc_event_callback));
	
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_enable.attr);

	kfree(di);

	return 0;
}

static int pmic_battery_probe(struct platform_device *pdev)
{
	int retval = 0;
	struct mc13892_dev_info *di;
	//int value = 0;
	//int charger;
	pmic_version_t pmic_version;

	/* Only apply battery driver for MC13892 V2.0 due to ENGR108085 */
	pmic_version = pmic_get_version();
	if (pmic_version.revision < 20) {
		pr_debug("Battery driver is only applied for MC13892 V2.0\n");
		return -1;
	}
	if (machine_is_mx50_arm2()) {
		pr_debug("mc13892 charger is not used for this platform\n");
		return -1;
	}

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		printk(KERN_ERR "%s: malloc di failed\n",__func__);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);
	
	/*pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
  charger = BITFEXT(value, BIT_CHG_DETS);
	if(charger && !(charger_detect())){
	  hw_set_ac(1);}
	else
	  hw_set_ac(0);*/
	  
	usb_adapter_det = bsp_charger_detect();
	if(2 == usb_adapter_det || 3 == usb_adapter_det)
		hw_set_ac(1);
	else if(4 == usb_adapter_det)
		hw_set_ac(0);

	di->charger.name	= "mc13892_charger";
	di->charger.type = POWER_SUPPLY_TYPE_MAINS;
	di->charger.properties = mc13892_charger_props;
	di->charger.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->charger.get_property = mc13892_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->charger);
	if (retval) {
		dev_err(di->dev, "failed to register charger\n");
		goto charger_failed;
	}

	di->usb.name	= "mc13892_usb";
	di->usb.type = POWER_SUPPLY_TYPE_MAINS;
	di->usb.properties = mc13892_charger_props;
	di->usb.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->usb.get_property = mc13892_usb_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->usb);
	if (retval) {
		dev_err(di->dev, "failed to register usb charger\n");
		goto usb_charger_failed;
	}
	di->adapter.name	= "mc13892_adapter";
	di->adapter.type = POWER_SUPPLY_TYPE_MAINS;
	di->adapter.properties = mc13892_charger_props;
	di->adapter.num_properties = ARRAY_SIZE(mc13892_charger_props);
	di->adapter.get_property = mc13892_adapter_charger_get_property;
	retval = power_supply_register(&pdev->dev, &di->adapter);
	if (retval) {
		dev_err(di->dev, "failed to register adapter charger\n");
		goto adapter_charger_failed;
	}

	INIT_DELAYED_WORK(&chg_work, chg_thread);
	chg_wq = create_singlethread_workqueue("mxc_chg");
	if (!chg_wq) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(chg_wq, &chg_work, HZ);
	
	INIT_DELAYED_WORK(&di->monitor_work, mc13892_battery_work);
	di->monitor_wqueue = create_singlethread_workqueue(dev_name(&pdev->dev));
	if (!di->monitor_wqueue) {
		retval = -ESRCH;
		goto workqueue_failed;
	}
	queue_delayed_work(di->monitor_wqueue, &di->monitor_work, HZ * 2);

	di->dev	= &pdev->dev;
	di->bat.name	= "mc13892_bat";
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = mc13892_battery_props;
	di->bat.num_properties = ARRAY_SIZE(mc13892_battery_props);
	di->bat.get_property = mc13892_battery_get_property;
	di->bat.use_for_apm = 1;

	di->battery_status = POWER_SUPPLY_STATUS_UNKNOWN;

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		dev_err(di->dev, "failed to register battery\n");
		goto batt_failed;
	}
	  
	bat_event_callback.func = charger_online_event_callback;
	bat_event_callback.param = (void *) di;
	CHECK_ERROR(pmic_event_subscribe(EVENT_CHGDETI, bat_event_callback));
	
	chgfault_event_callback.func = charger_fault_event_callback;
	chgfault_event_callback.param = (void *) di;
	CHECK_ERROR(pmic_event_subscribe(EVENT_CHGFAULTI, chgfault_event_callback));

	sc_event_callback.func = short_circuit_event_callback;
	sc_event_callback.param = (void *) di;
	CHECK_ERROR(pmic_event_subscribe(EVENT_SCPI, sc_event_callback));

	retval = sysfs_create_file(&pdev->dev.kobj, &dev_attr_enable.attr);
	if (retval) {
		printk(KERN_ERR
		       "Battery: Unable to register sysdev entry for Battery");
		goto workqueue_failed;
	}
	pmic_set_scp(SCP_EN, 1);
	chg_wa_is_active = 1;
	chg_wa_timer = 0;
	disable_chg_timer = 0;
	ep7_charge_info.battery_level_curr_status=BAT_BLANK;
	pmic_stop_coulomb_counter();
	pmic_calibrate_coulomb_counter();
	pmic_write_reg(REG_MEM_A, (0 << 0), (1 << 0));
	goto success;

workqueue_failed:
	power_supply_unregister(&di->bat);
batt_failed:
	power_supply_unregister(&di->adapter);
adapter_charger_failed:
	power_supply_unregister(&di->usb);
usb_charger_failed:
	power_supply_unregister(&di->charger);
charger_failed:
	kfree(di);
success:
	dev_dbg(di->dev, "%s battery probed!\n", __func__);
  return retval;
}


int get_bat_charge_status(void)
{
	if(ep7_charge_info.battery_level_curr_status>=BAT_STATUS_NUM)
		printk(KERN_INFO "sorry for battery status out of limited level\n");
	
	return ep7_charge_info.battery_level_curr_status;

}
EXPORT_SYMBOL(get_bat_charge_status);

static int pmic_battery_suspend(struct platform_device *pdev,
                                pm_message_t state)
{
  unsigned int value,charger;
	pmic_read_reg(REG_INT_SENSE0, &value, BITFMASK(BIT_CHG_DETS));
  charger = BITFEXT(value, BIT_CHG_DETS);
	if(charger)
	return -1;	
	
  suspend_flag = 1;
	return 0;
};

static int pmic_battery_resume(struct platform_device *pdev)
{
	suspend_flag = 0;
	return 0;
};

static struct platform_driver pmic_battery_driver_ldm = {
	.driver = {
		   .name = "mc13892_battery",
		   .bus = &platform_bus_type,},
	.suspend = pmic_battery_suspend,
  .resume = pmic_battery_resume,
	.probe = pmic_battery_probe,
	.remove = pmic_battery_remove,
};

static int __init pmic_battery_init(void)
{
	pr_debug("PMIC Battery driver loading...\n");
	return platform_driver_register(&pmic_battery_driver_ldm);
}

static void __exit pmic_battery_exit(void)
{
	platform_driver_unregister(&pmic_battery_driver_ldm);
	pr_debug("PMIC Battery driver successfully unloaded\n");
}

module_init(pmic_battery_init);
module_exit(pmic_battery_exit);

MODULE_DESCRIPTION("pmic_battery driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
