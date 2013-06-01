/*
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License version 2 as
*  published by the Free Software Foundation.
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

//static char wf_version[37] = {0};
//int current_position = 0;
extern unsigned char *waveform_mem;
static int FPL_platform = 0;
static int Run_type = 0;
static char FPL_lot_number = 0;
static int Display_size = 0;
static int WF_type = 0;
static char WF_version = 0;
static char WF_subversion = 0;
static int AMEPD = 0;
const char *amepd_string="ED060XH2";
static int Timing_mode = 0;

static int get_wf_version(char *buf)
{
	char lot_number100,lot_number10,lot_number1;
	char wf_version10, wf_version1;
	char wf_subversion10, wf_subversion1;
	/*
	for(i = 0; i < 32; i++)
	{
		printk("%d: [%0x]=%x\n", i, waveform_mem + i, *(unsigned char*)(waveform_mem + i));
	}
	*/

//	printk("current_position:%d\n", current_position);
	static char wf_version[37] = {0};
	char *str_temp;
	int current_position = 0;
	int i;

	FPL_platform = *(waveform_mem + 13);
//	printk("FPL_platform:%x\n", FPL_platform);
	
	memset(wf_version, 0, sizeof(wf_version));
	switch(FPL_platform)
	{
		case 0x02:
		case 0x03:
			wf_version[current_position] = 'V';
			current_position++;
			wf_version[current_position] = '1';
			current_position++;
			wf_version[current_position] = '1';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			break;
		case 0x04:
			wf_version[current_position] = 'V';
			current_position++;
			wf_version[current_position] = '1';
			current_position++;
			wf_version[current_position] = '1';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			wf_version[current_position] = 'A';
			current_position++;
			break;
		case 0x06:
			wf_version[current_position] = 'V';
			current_position++;
			wf_version[current_position] = '2';
			current_position++;
			wf_version[current_position] = '2';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			break;
		case 0x07:
			wf_version[current_position] = 'V';
			current_position++;
			wf_version[current_position] = '2';
			current_position++;
			wf_version[current_position] = '5';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			break;
		default:
			break;
	}

	wf_version[current_position] = '_';
	current_position++;
//	printk("current_position:%d\n", current_position);

	Run_type = *(waveform_mem + 12);
//	printk("Run_type:%x\n", Run_type);
	switch(Run_type)
	{
		case 0x00:
			wf_version[current_position] = 'B';
			current_position++;
			break;
		case 0x01:
			wf_version[current_position] = 'T';
			current_position++;
			break;
		case 0x02:
			wf_version[current_position] = 'P';
			current_position++;
			break;
		case 0x03:
			wf_version[current_position] = 'Q';
			current_position++;
			break;
		case 0x04:
			wf_version[current_position] = 'A';
			current_position++;
			break;
		case 0x05:
			wf_version[current_position] = 'C';
			current_position++;
			break;
		case 0x06:
			wf_version[current_position] = 'D';
			current_position++;
			break;
		case 0x07:
			wf_version[current_position] = 'E';
			current_position++;
			break;
		case 0x08:
			wf_version[current_position] = 'F';
			current_position++;
			break;
		case 0x09:
			wf_version[current_position] = 'G';
			current_position++;
			break;
		case 0x0a:
			wf_version[current_position] = 'H';
			current_position++;
			break;
		case 0x0b:
			wf_version[current_position] = 'I';
			current_position++;
			break;
		case 0x0c:
			wf_version[current_position] = 'J';
			current_position++;
			break;
		case 0x0d:
			wf_version[current_position] = 'K';
			current_position++;
			break;
		case 0x0e:
			wf_version[current_position] = 'L';
			current_position++;
			break;
		case 0x0f:
			wf_version[current_position] = 'M';
			current_position++;
			break;
		case 0x10:
			wf_version[current_position] = 'N';
			current_position++;
			break;
		default:
			break;
	}

	//printk("current_position:%d\n", current_position);

	FPL_lot_number = *(waveform_mem + 14);
	//printk("FPL_lot_number:%d\n", FPL_lot_number);
	//char lot_number100,lot_number10,lot_number1;
	lot_number100 = FPL_lot_number / 100;
	lot_number10 = (FPL_lot_number - lot_number100*100) / 10;
	lot_number1 = FPL_lot_number % 10;
//	printk("lot_number100:%d,lot_number10:%d,lot_number1:%d\n", lot_number100, lot_number10, lot_number1);
	wf_version[current_position] = lot_number100 + 0x30;
	current_position++;
	wf_version[current_position] = lot_number10 + 0x30;
	current_position++;
	wf_version[current_position] = lot_number1 + 0x30;
	current_position++;

	wf_version[current_position] = '_';
	current_position++;
//	printk("current_position:%d\n", current_position);

	Display_size = *(waveform_mem + 20);
//	printk("Display_size:%x\n", Display_size);
	switch(Display_size)
	{
		case 0x32:
			wf_version[current_position] = '5';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			break;
		case 0x3C:
			wf_version[current_position] = '6';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			break;
		case 0x50:
			wf_version[current_position] = '8';
			current_position++;
			wf_version[current_position] = '0';
			current_position++;
			break;
		case 0x61:
			wf_version[current_position] = '9';
			current_position++;
			wf_version[current_position] = '7';
			current_position++;
			break;
		default:
			break;
	}

	wf_version[current_position] = '_';
	current_position++;
	WF_type = *(waveform_mem + 19);
	//printk("WF_type:%x\n", WF_type);
	if(0x1D == WF_type)
	{
		wf_version[current_position] = 'W';
		current_position++;
		wf_version[current_position] = 'N';
		current_position++;
	}
	else if(0x2D == WF_type)
	{
		wf_version[current_position] = 'W';
		current_position++;
		wf_version[current_position] = 'S';
		current_position++;
	}

	WF_version = *(waveform_mem + 17);
	//printk("WF_version:%x\n", WF_version);
	//char wf_version10, wf_version1;
	wf_version10 = WF_version / 16;
	wf_version1 = WF_version % 16;
	//printk("wf_version10:%c,wf_version1:%c\n", wf_version10, wf_version1);
	wf_version[current_position] = wf_version10 + 0x30;
	current_position++;
	wf_version[current_position] = wf_version1 + 0x30;
	current_position++;

	WF_subversion = *(waveform_mem + 18);
	//printk("WF_subversion :%x\n", WF_subversion);
	//char wf_subversion10, wf_subversion1;
	wf_subversion10 = WF_subversion / 16;
	wf_subversion1 = WF_subversion % 16;
//	printk("wf_subversion10:%d,wf_subversion1:%d\n", wf_subversion10, wf_subversion1);
	wf_version[current_position] = wf_subversion10 + 0x30;
	current_position++;
	wf_version[current_position] = wf_subversion1 + 0x30;
	current_position++;

	wf_version[current_position] = '_';
	current_position++;

	AMEPD = *(waveform_mem + 21);
	//printk("AMEPD:%x\n", AMEPD);
	if(0x31 == AMEPD || 0xCA == AMEPD)
	{
		wf_version[current_position] = 'E';
		current_position++;
		wf_version[current_position] = 'D';
		current_position++;
		wf_version[current_position] = '0';
		current_position++;
		wf_version[current_position] = '6';
		current_position++;
		wf_version[current_position] = '0';
		current_position++;
		wf_version[current_position] = 'S';
		current_position++;
		wf_version[current_position] = 'C';
		current_position++;
		wf_version[current_position] = 'M';
		current_position++;
		wf_version[current_position] = 'C';
		current_position++;
		wf_version[current_position] = '1';
		current_position++;
	}
	else
		if(AMEPD == 0x6d){/*for ep7a*/
			str_temp=amepd_string;
			for(i=0; i < 8; i++)
			{
				wf_version[current_position]=*str_temp;
				current_position++;
				str_temp++;
			}
		}	

	wf_version[current_position] = '_';
	current_position++;

	Timing_mode = *(waveform_mem + 24);
	//printk("Timing_mode:%x\n", Timing_mode);
	if(0x2D == WF_type)
		switch(Timing_mode)
		{
			case 0x03:
				wf_version[current_position] = 'B';
				current_position++;
				break;
			default:
				break;
		}
	else if(0x1D == WF_type)
		switch(Timing_mode)
		{
			case 0x01:
				wf_version[current_position] = 'A';
				current_position++;
				break;
			case 0x02:
				wf_version[current_position] = 'B';
				current_position++;
				break;
			case 0x03:
				wf_version[current_position] = 'C';
				current_position++;
				break;
			default:
				break;
		}

	wf_version[current_position] = 'T';
	current_position++;
	wf_version[current_position] = 'C';
	current_position++;
	wf_version[current_position] = '\n';
	//current_position++;
	//current_position++;
	//wf_version[current_position] = '.';
	//current_position++;
	//wf_version[current_position] = 'f';
	//current_position++;
	//wf_version[current_position] = 'w';
	//printk("the current_position:%d\n", current_position);

//	printk(KERN_ALERT "%s\n", wf_version);
	return sprintf(buf, wf_version);
//	memset(wf_version, 0, sizeof(wf_version));
//	return  0;
}

static int set_wf_version(int state)
{
	return 0;
}

struct wf_proc
{
	char *module_name;
	int (*wf_get) (char *);
	int (*wf_set) (unsigned int);
};

static const struct wf_proc wf_modules[] = 
{
	{"waveform_version", get_wf_version, set_wf_version}
};

size_t wf_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	ssize_t len;
	char *p = page;
	const struct wf_proc *proc = data;

	p += proc->wf_get(p);

	len = p -page;
//	printk("len:%d\n", len);

	*eof = 1;

	return len;
}

size_t wf_proc_write(struct file *file, const char __user *buf, unsigned long len, void *data)
{
	return 0;
}

static int proc_node_num = (sizeof(wf_modules) / sizeof(*wf_modules));
static struct proc_dir_entry *update_proc_root;
static struct proc_dir_entry *proc[1];


static int wf_proc_init(void)
{
	unsigned int i;

	update_proc_root = proc_mkdir("wf_version", NULL);
	if(!update_proc_root)
	{
		printk("create proc_root_dir failed\n");	
		return -1;
	}
      for(i = 0; i < proc_node_num; i++)
      {
		 mode_t mode;
             mode = 0;
     		 if(wf_modules[i].wf_set)
        	 {
           	      mode |= S_IRUGO;
         	 }
        	 if(wf_modules[i].wf_get)
        	 {
           	      mode |= S_IWUGO;
         	 }
	       proc[i] = create_proc_entry(wf_modules[i].module_name, mode, update_proc_root);
    		 if(proc[i])
     		 {
           	    proc[i]->data = (void *)(&wf_modules[i]);
           	    proc[i]->read_proc = wf_proc_read;
           	    proc[i]->write_proc = wf_proc_write;
       	 }
    	 }
     	return 0;
}

static int __init wf_init(void)
{
	wf_proc_init();
	return 0;
}

static int __exit wf_exit(void)
{
	return 0;
}

module_init(wf_init)
module_exit(wf_exit)

MODULE_LICENSE("GPL");
