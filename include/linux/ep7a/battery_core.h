#ifndef _VENUS_BATTERY_CORE_H
#define _VENUS_BATTERY_CORE_H

#define MAX_LOW_VOL_COUNT 	5		// juget time is 5S 
#define LOW_VOL_THREAD_0 		350000		// low battery voltage is 3.5V
#define LOW_VOL_THREAD_1 		360000		// low battery voltage is 3.6V

//#define BATTERY_DEBUG

struct battery_platform_data {
	int (*battery_online)(void);
	int (*charger_online)(void);
	int (*charger_enable)(void);
	int (*battery_level)(void);
	int (*battery_voltage)(void);
	int (*bat_suspend)(void);
	int (*bat_resume)(void);
	int (*bat_remove)(void);
	
};

extern void battery_register(struct battery_platform_data *parent);
#endif

