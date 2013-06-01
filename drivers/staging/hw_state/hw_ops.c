
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/bcd.h>
#include <mach/hardware.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/mach/time.h>

#include <linux/device.h>
#include <linux/err.h> 
#include <linux/delay.h>
#include <linux/cdev.h>
#include <asm/system.h>

#include <linux/io.h>
#include <linux/sched.h>
#include <linux/hw_ops.h>

#include "hw_state.h"
//#include "hw_ioctl.h"

#define BOARD_EP7   7
#define BOARD_ARGS  BOARD_EP7

#define DISPLAY_6_600_800   0
#define DISPLAY_ARGS    DISPLAY_6_600_800

#define KEY_ARGS    4

#define AUDIO_SSM2602   1
#define AUDIO_ARGS  AUDIO_SSM2602

#define USB_INTERNAL    3   
#define USB_ARGS    USB_INTERNAL

#define CAP_TOUCH_ARGS  1

#define IOC_ARGS    1

#define WIFI_ARGS   1
#if 0
#define HW_CONFIG   ((unsigned long long)((BOARD_ARGS<<0)|(DISPLAY_ARGS<<4)|(KEY_ARGS<<8)|(AUDIO_ARGS<<12)|(USB_ARGS<<16)|(CAP_TOUCH_ARGS<<20)|\
                        (WIFI_ARGS<<24)|(IOC_ARGS<<28)))
#endif
#define PLATFORM_NAME	"ep7a"
#define	HW_CONFIG	672166861527056384

extern struct hardware_state_dev hw_state_dev;

int hw_set_ledctl(int (*set_led)(int))
{
	hw_state_dev.set_led = set_led;	
	return 0;
}

int hw_set_keylockctl(int (*set_key_lock)(int))
{
	hw_state_dev.set_key_lock = set_key_lock;
	return 0;
}

int hw_set_tslockctl(int (*set_ts_lock)(int))
{
	hw_state_dev.set_ts_lock = set_ts_lock;
	return 0;
}

int hw_led_onoff(int led_status)
{	
	printk("led_status = %d\n",led_status);
	if(hw_state_dev.set_led == NULL)
	return -1;
	else
	return hw_state_dev.set_led(led_status);

}
//mode=0 is led off mode,mode=1 is led on mode.default led user mode is on mode
int hw_led_set_usermode(int mode)
{
	if( (mode == 0) || ( mode==1 ) )
	{
	hw_state_dev.led_usermode = mode;
	if(mode == 1)
	hw_led_onoff(1);
	else
	hw_led_onoff(0);
  	}
	else
	{
	printk("led user mode set error,please check set value\n");
	return -1;
	}
	return 0;
}

int hw_led_get_usermode(void)
{
	return hw_state_dev.led_usermode;
}




int hw_key_lock(int key_lock_status)
{
	return hw_state_dev.set_key_lock(key_lock_status);
}

int hw_ts_lock(int ts_lock_status)
{
	return hw_state_dev.set_ts_lock(ts_lock_status);

}

int hw_set_led(int led_status)
{
//	hw.state_dev.set_led(led_status);
	hw_state_dev.led_status = led_status;
	return 0;
}

int hw_get_led(int *led_status)
{
	*led_status = hw_state_dev.led_status;
	return 0;
}


int hw_set_key(unsigned key_status)
{
	hw_state_dev.key_status = key_status;
	return 0;
}

int hw_get_key(unsigned *key_status)
{
	*key_status = hw_state_dev.key_status;
	return 0;

}

int hw_set_keylock(int key_lock_status)
{
	hw_state_dev.key_lock_status = key_lock_status;
	return 0;
}
int hw_get_keylock(int *key_lock_status)
{
	*key_lock_status = hw_state_dev.key_lock_status;
	return 0; 
}

int hw_set_tslock(int ts_lock_status)
{
	hw_state_dev.ts_lock_status = ts_lock_status;	
	return 0;
}
int hw_get_tslock(int *ts_lock_status)
{
	*ts_lock_status = hw_state_dev.ts_lock_status;
	return 0;
}

int hw_set_usb(int usb_status)
{
	hw_state_dev.usb_status = usb_status;
	return 0;
}

int hw_get_usb(int *usb_status)
{
	*usb_status = hw_state_dev.usb_status;
	return 0;
}

int hw_set_batt(int batt_cap)
{
	hw_state_dev.batt_cap = batt_cap;
	return 0;
}

int hw_get_batt(int *batt_cap)
{
	*batt_cap = hw_state_dev.batt_cap;
	return 0;
}

int hw_set_ac(int ac_status)
{
	hw_state_dev.ac_status = ac_status;
	return 0;
}

int hw_get_ac(int *ac_status)
{
	*ac_status = hw_state_dev.ac_status;
	return 0;
}

int hw_set_sd(int sd_status)
{
	hw_state_dev.sd_status = sd_status;
	return 0;
}

int hw_get_sd(int *sd_status)
{
	*sd_status = hw_state_dev.sd_status;
	return 0;
}

int hw_set_pwbtn(int pwbtn_status)
{
	hw_state_dev.pwbtn_status = pwbtn_status;
	return 0;
}

int hw_get_pwbtn(int *pwbtn_status)
{
	*pwbtn_status = hw_state_dev.pwbtn_status;
	return 0;
}

int hw_set_wakeup_from_rtc(int (*set_wakeup_from_rtc)(int))
{
	hw_state_dev.set_wakeup_from_rtc = set_wakeup_from_rtc;
	return 0;
}
int hw_set_alrwkup(int alr_wkup_status)
{
	!hw_state_dev.set_wakeup_from_rtc ? NULL: hw_state_dev.set_wakeup_from_rtc(alr_wkup_status);
	return 0;
}

int hw_get_alrwkup(int *alr_wkup_status)
{
	*alr_wkup_status = hw_state_dev.alr_wkup_status;
	return 0;
}

int hw_set_platform(char *platform_name)
{
	hw_state_dev.platform = "ep7a\0";

	return 0;
}

int hw_get_platform(char *platform_name)
{
	strcpy(platform_name, hw_state_dev.platform);

	return 0;
}

int hw_set_config(long long hwconfig_uboot)
{
	hw_state_dev.hwconfig_uboot = hwconfig_uboot;
	return 0;
}

int hw_get_config(long long *hwconfig_uboot)
{
	hw_state_dev.hwconfig_uboot = HW_CONFIG;
	*hwconfig_uboot = hw_state_dev.hwconfig_uboot;
	return 0;
}
int hw_set_hp(int hp_status)
{
	hw_state_dev.hp_status = hp_status;
	return 0;
}

int hw_get_hp(int *hp_status)
{
	*hp_status = hw_state_dev.hp_status;
	return 0;
}

int hw_set_wfpw(int wifi_status)
{
	hw_state_dev.wifi_status = wifi_status;
	return 0;
}

int hw_get_wfpw(int *wifi_status)
{
	*wifi_status = hw_state_dev.wifi_status;
	return 0;
}

//acquire the USB plus or minus
int hw_set_getusb_DpDn(int (*get_usb_DpDn)(void))
{
	hw_state_dev.get_usb_DpDn = get_usb_DpDn;
	return 0;
}


int hw_get_usb_DpDn(void)
{
        //printk("led_status = %d\n",led_status);
        if(hw_state_dev.get_usb_DpDn == NULL)
	return -1;
	else
	return hw_state_dev.get_usb_DpDn();

}


int hw_set_device_poweroff(int (*set_device_poweroff)(void))
{
	hw_state_dev.set_device_poweroff = set_device_poweroff;
	return 0;
}

int hw_poweroff(void)
{
	!hw_state_dev.set_device_poweroff? NULL : hw_state_dev.set_device_poweroff() ;
	return 0;
}

int hw_set_device_reboot(int (*set_device_reboot)(void))
{
	hw_state_dev.set_device_reboot = set_device_reboot;
	return 0;
}

int hw_reboot(void)
{
	!hw_state_dev.set_device_reboot ? NULL : hw_state_dev.set_device_reboot();
	return 0;
}

int hw_set_wdtctl(int (*set_wdt)(int))
{       
        hw_state_dev.set_wdt = set_wdt;
        return 0;
}


//if timeout = 0 it is off watchdog
//if timeout = 5-120 it is watchdog timeout value 5-120 second
int hw_set_wdt(int wdt_timeout)
{
        printk("wdt_timeout = %d\n",wdt_timeout);
        if(hw_state_dev.set_wdt == NULL)
        return -1;
        else
        return hw_state_dev.set_wdt(wdt_timeout);

}



int hw_get_low_batt(int *low_batt_status)
{
	int batt_cap;
//	int ret = 0;
	hw_get_batt(&batt_cap);
	if(batt_cap < 1){
		printk("low battery capacity, need charging\n");
		if(batt_cap >= 5){
			hw_state_dev.low_batt_status = 2;
			*low_batt_status = hw_state_dev.low_batt_status;
		}
		else if(batt_cap >= 3){
			hw_state_dev.low_batt_status = 1;
			*low_batt_status = hw_state_dev.low_batt_status;
		}
		else{
			hw_state_dev.low_batt_status = 0;
			*low_batt_status = hw_state_dev.low_batt_status;
			printk("critical low battery, auto power off...\n");
//			hw_poweroff();
		}
	}
	else{
		printk("battery capacity more than 15%%, not in the low capacity status\n");
		*low_batt_status = hw_state_dev.batt_cap;
	}
	return 0;
}

int hw_set_deepsuspend_level_ctrl(int (*set_suspend_level)(int suspend_level))
{
	hw_state_dev.set_suspend_level = set_suspend_level;
	return 0;
}

int hw_set_deepsuspend_level(int suspend_level)
{
	if(hw_state_dev.set_suspend_level == NULL)
		return -1;
	else
		return hw_state_dev.set_suspend_level(suspend_level);
}

int hw_set_deepsuspend(int suspend_level)
{
	hw_state_dev.suspend_level = suspend_level;
	return 0;
}

int hw_get_deepsuspend(int *suspend_level)
{
	*suspend_level = hw_state_dev.suspend_level;
	return 0;
}

EXPORT_SYMBOL(hw_set_ledctl);
EXPORT_SYMBOL(hw_set_tslockctl);
EXPORT_SYMBOL(hw_set_keylockctl);
EXPORT_SYMBOL(hw_get_led);
EXPORT_SYMBOL(hw_set_led);
EXPORT_SYMBOL(hw_set_key);
EXPORT_SYMBOL(hw_get_key);
EXPORT_SYMBOL(hw_set_usb);
EXPORT_SYMBOL(hw_get_usb);
EXPORT_SYMBOL(hw_set_batt);
EXPORT_SYMBOL(hw_get_batt);
EXPORT_SYMBOL(hw_set_ac);
EXPORT_SYMBOL(hw_get_ac);
EXPORT_SYMBOL(hw_set_sd);
EXPORT_SYMBOL(hw_get_sd);
EXPORT_SYMBOL(hw_set_pwbtn);
EXPORT_SYMBOL(hw_get_pwbtn);
EXPORT_SYMBOL(hw_set_wakeup_from_rtc);
EXPORT_SYMBOL(hw_set_alrwkup);
EXPORT_SYMBOL(hw_get_alrwkup);
EXPORT_SYMBOL(hw_set_hp);
EXPORT_SYMBOL(hw_get_hp);
EXPORT_SYMBOL(hw_set_wfpw);
EXPORT_SYMBOL(hw_get_wfpw);
EXPORT_SYMBOL(hw_set_getusb_DpDn);
EXPORT_SYMBOL(hw_get_usb_DpDn);
EXPORT_SYMBOL(hw_set_device_poweroff);
EXPORT_SYMBOL(hw_set_device_reboot);
EXPORT_SYMBOL(hw_poweroff);
EXPORT_SYMBOL(hw_reboot);
EXPORT_SYMBOL(hw_set_wdt);
EXPORT_SYMBOL(hw_set_wdtctl);
EXPORT_SYMBOL(hw_get_low_batt);
EXPORT_SYMBOL(hw_set_deepsuspend_level_ctrl);
EXPORT_SYMBOL(hw_set_deepsuspend_level);
EXPORT_SYMBOL(hw_set_deepsuspend);

