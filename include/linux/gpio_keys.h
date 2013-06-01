#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

struct gpio_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	int longpress_time;
	bool can_disable;
	const char key_status[20];
	int timer_interval;
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
//	void (*register_pwrkey) (pwrkey_callback);
//	int (*get_key_status) (int);
	unsigned int rep:1;		/* enable input subsystem auto repeat */
};
extern int report_pwrkey(int (*pwrkey_event0_report)(int));

#endif
