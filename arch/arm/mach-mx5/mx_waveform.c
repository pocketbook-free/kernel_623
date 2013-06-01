#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <asm/io.h>
#include "crc32.h"
#include <asm/uaccess.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/gfp.h>
#include <linux/mx_waveform.h>

unsigned char *waveform_mem = NULL;

int __init mxc_waveform_init(void)
{
	unsigned long crc, computed_residue;
	int i; 
	unsigned long waveform_size;

	waveform_mem = ioremap(WAVEFORM_BASE, WAVEFORM_RESERVED_SIZE);

	if (!waveform_mem) {
		printk("Fail to ioremap waveform_mem reserved base.\n");
		return 0;
	}

	crc = *(unsigned long *)(waveform_mem + WAVEFORM_RESERVED_SIZE - sizeof(unsigned long));
	printk("The uboot calculated crc is:%x\n", crc);

/*	if (*(waveform_mem + 0x14) != 0x3c || *(waveform_mem + 0x0D) != 0x6) {
		waveform_mem = NULL;
		printk("Wrong waveform type!\n");
		return 0;
	}*/
	
	for (i = 0; i < 25; i++) {
		printk("%d: [%0x]=%x\n", i, waveform_mem + 4 * i, *(unsigned long *)(waveform_mem + 4 * i));
	}

//	for(i = 0; i < 10; i++) {
//		printk("%d: [%0x]=%x\n", i, waveform_mem + WAVEFORM_RESERVED_SIZE/2 - 4*i, \ 
//					*(unsigned long *)(waveform_mem + WAVEFORM_RESERVED_SIZE/2 - 4*i));
//	}

//	printk("^_^ ^_^ [%0x]=%0x\n", waveform_mem + WAVEFORM_SIZE -4, *(unsigned long *)(waveform_mem + WAVEFORM_SIZE -4));

	waveform_size = *((unsigned int *)waveform_mem + 1);
	printk("Target waveform size is %ld Bytes\n", waveform_size);

	//computed_residue = crc32(0L, (unsigned char *)waveform_mem, WAVEFORM_RESERVED_SIZE - sizeof(unsigned long)); 
	computed_residue = crc32(0L, (unsigned char *)waveform_mem, waveform_size); 
	printk("kernel computed_residue(crc32) = %x\n", computed_residue);

	if(crc != computed_residue) {
		printk("Check waveform crc32 fail.");
		return 0;
	} else {
		printk("Verify CRC32 OK!\n");
	}
	
	return 0;
}

EXPORT_SYMBOL(waveform_mem);
subsys_initcall(mxc_waveform_init);

