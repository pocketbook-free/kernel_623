#ifndef __HW_OPS_H__
#define __HW_OPS_H__


#define LED_ON		1
#define LED_OFF		0


int hw_set_ledctl(int (*set_led)(int));   //appy to led driver, you need implement set_led function in which control led hardware action and call hw_set_led.
int hw_set_keylockctl(int (*set_key_lock)(int));
int hw_set_tslockctl(int (*set_touch_lock)(int));

int hw_led_onoff(int led_status);
int hw_key_lock(int ts_tatus);
int hw_ts_lock(int ts_tatus);

int hw_set_led(int led_status);
int hw_get_led(int *led_status);

int hw_set_key(unsigned key_status);
int hw_get_key(unsigned *key_status);

int hw_set_usb(int usb_status);
int hw_get_usb(int *usb_status);

int hw_set_pwbtn(int pwbtn_status);
int hw_get_pwbtn(int *pwbtn_status);

int hw_set_sd(int sd_status);
int hw_get_sd(int *sd_status);

int hw_set_ac(int ac_status);
int hw_get_ac(int *ac_status);

int hw_set_keylock(int key_lock_status);
int hw_get_keylock(int *key_lock_status);

int hw_set_tslock(int ts_lock_status);
int hw_get_tslock(int *ts_lock_status);

int hw_set_batt(int batt_cap);
int hw_get_batt(int *batt_cap);

int hw_set_alrwkup(int alrwkup);
int hw_get_alrwkup(int *alrwkup);

int hw_set_platform(char platform[16]);
int hw_get_platform(char platform[16]);

int hw_set_config(long long hwconfig_uboot);
int hw_get_config(long long *hwconfig_uboot);

int hw_set_hp(int hp_status);
int hw_get_hp(int *hp_status);

int hw_set_wfpw(int wf_status);
int hw_get_wfpw(int *wf_status);

int hw_set_deepsuspend_level_ctrl(int (*set_suspend_level)(int));
int hw_get_usb_plus_minus(int (*get_usb_plus_minus)(void));
int hw_set_device_poweroff(int (*set_device_poweroff)(void));
int hw_set_device_reboot(int (*set_device_reboot)(void));
int hw_set_wakeup_from_rtc(int (*set_wakeup_from_rtc)(int));

int hw_set_deepsuspend_level(int suspend_level);
int hw_set_deepsuspend(int suspend_level);
int hw_get_deepsuspend(int *suspend_level);

int hw_poweroff(void);
int hw_set_getusb_DpDn(int (*get_usb_DpDn)(void));
int hw_get_usb_DpDn(void);
int hw_reboot(void);
	
int hw_led_set_usermode(int mode);
int hw_led_get_usermode(void);

int hw_set_wdtctl(int (*set_watchdog)(int));
int hw_set_wdt(int watchdog_status);
//int hw_set_wd_status(int watchdog_status);
//int hw_get_wd_status(int *watchdog_status);

int hw_get_low_batt(int *low_batt_status);
#endif
