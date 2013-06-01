/*
 * Copyright 2008-2010 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * The code contained herein is licensed under the GNU General Public
 * License.  You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 *
 * Create static mapping between physical to virtual memory.
 */

#include <linux/mm.h>
#include <linux/init.h>
#include <linux/bootmem.h>
#include <asm/mach/map.h>
#include <mach/iomux-v3.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/iomux-v3.h>
#include <linux/mx_waveform.h>
#include "linux/share_region.h"


int usb_adapter_base=0;
int ccm_blk_base = 0;
/*!
 * This structure defines the MX5x memory map.
 */
static struct map_desc mx5_io_desc[] __initdata = {
	{
	 .virtual = AIPS1_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPS1_BASE_ADDR),
	 .length = AIPS1_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = SPBA0_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(SPBA0_BASE_ADDR),
	 .length = SPBA0_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = AIPS2_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPS2_BASE_ADDR),
	 .length = AIPS2_SIZE,
	 .type = MT_DEVICE},
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mx5_map_io(void)
{
	int i, tmp;
	void *virt_mem;
   	 unsigned int waveform_addr;
	
	mxc_iomux_v3_init(IO_ADDRESS(IOMUXC_BASE_ADDR));
	/* Fixup the mappings for MX53 */
	if (cpu_is_mx53() || cpu_is_mx50()) {
		for (i = 0; i < ARRAY_SIZE(mx5_io_desc); i++)
			mx5_io_desc[i].pfn -= __phys_to_pfn(0x20000000);
	}

	iotable_init(mx5_io_desc, ARRAY_SIZE(mx5_io_desc));
	mxc_arch_reset_init(IO_ADDRESS(WDOG1_BASE_ADDR));
	virt_mem = __alloc_bootmem(WAVEFORM_RESERVED_SIZE, PAGE_SIZE, WAVEFORM_BASE);
	if(virt_mem == NULL)
		printk("Failed to waveform reserve mem!\n");
	waveform_addr = virt_to_phys(virt_mem);
	printk("The waveform_addr is:%x\n", waveform_addr);


/*	if(reserve_bootmem(WAVEFORM_BASE, WAVEFORM_RESERVED_SIZE, BOOTMEM_DEFAULT)==0)
		printk("reserve OK(waveform)--------------------\n");*/

	tmp = reserve_bootmem(SHARE_REGION_BASE, SHARE_REGION_SIZE, BOOTMEM_DEFAULT);
	if (tmp == 0)
		printk("reserve OK!\n");
	else
		printk("reserve error!\n");

	usb_adapter_base = IO_ADDRESS(OTG_BASE_ADDR);
	ccm_blk_base = IO_ADDRESS(CCM_BASE_ADDR);
}

