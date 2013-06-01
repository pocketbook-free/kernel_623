#ifndef __HW_STATE_H__
#define __HW_STATE_H__



typedef struct hardware_state_dev
{
	int led_status;
	unsigned key_status;
	int usb_status;
	int pwbtn_status;
	int sd_status;
	int ac_status;
	int key_lock_status;
	int ts_lock_status;
	int batt_cap;
	int alr_wkup_status;
	int hp_status;
	int wifi_status;
	int led_usermode;
	unsigned long long hwconfig_uboot;
	char *platform;
	int low_batt_status;
	int suspend_level;
	
	int (*set_led)(int);
	int (*set_key_lock)(int);
	int (*set_ts_lock)(int);
	int (*get_usb_DpDn)(void);
	int (*set_device_poweroff)(void);
	int (*set_device_reboot)(void);
	int (*set_wakeup_from_rtc)(int);
	int (*set_wdt)(int);
	int (*set_suspend_level)(int);

} *phw_obj; 
#endif
