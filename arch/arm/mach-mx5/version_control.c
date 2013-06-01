/* 
 * arch/arm/mach-mx50/version_control.c
 *
 * Copyright (C) 2012,SW-BSP1
 *
 * Board version control for distinguash wifi wc160&wc121
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/proc_fs.h>
#include <mach/gpio.h>

#if 0
//Please notice schematic about the version control pins
#define VC0_IO	(4*32 + 12)
#define VC1_IO	(4*32 + 13)
#define VC2_IO	(4*32 + 14)
#define VC3_IO	(4*32 + 15)
#endif

static int board_set_version(unsigned int version)
{
	//version can be get, but can't be set, version control by hardware R86 R87 R88 R89.
	return 0;
}

static int board_get_version(void)
{
#if 0
	unsigned int reg = 0;
	int i;
	for(i = 0; i < 8; i++)
	{
		reg = __raw_readl(IO_ADDRESS(GPIO5_BASE_ADDR + i * 4)); 	
		printk("Base=0x%x, reg=0x%x\n", i * 4, reg); 
	}
#endif
	return (((__raw_readl(IO_ADDRESS(GPIO5_BASE_ADDR + 0x08)) << 16) >> 28) & 0xF); //Read back GPIO5 bit[12, 15] status register's value.
}


static const struct version_proc {
	char *module_name;
	int (*get_version)(void);
	int (*set_version)(unsigned int);
} version_modules[] = {
	{ "board_version", 	board_get_version, 	board_set_version },
};

ssize_t version_proc_read(char *page, char **start, off_t off,
                      int count, int *eof, void *data)
{
	char *out = page;
	unsigned int version;
	const struct version_proc *proc = data;
	ssize_t len;
	
	version = proc->get_version();

	out += sprintf(out, "%d\n", version);

	len = out - page - off;
	if (len < count)
	{
		*eof = 1;
		if (len <= 0)
		{
			return 0;
		}
	}
	else
	{
		len = count;
	}
	*start = page + off;
	return len;
}

int __init version_init(void)
{
	int i;

	for (i = 0; i < (sizeof(version_modules) / sizeof(*version_modules)); i++)
	{
		struct proc_dir_entry *proc;
		mode_t mode;

		mode = 0;
		if (version_modules[i].get_version)
		{
			mode |= S_IRUGO;
		}

		proc = create_proc_entry(version_modules[i].module_name, mode, NULL);
		if (proc)
		{
			proc->data = (void *)(&version_modules[i]);
			proc->read_proc = version_proc_read;
		}
	}
	return 0;
}


late_initcall(version_init);

