#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include"adapter_usb.h"
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#define ADAPTER_INSERT 0x2
#define USB_INSERT 0x0

#undef PB_TEST
#ifdef PB_TEST
        #define TEST(x, args...)        printk(x, ##args)
#else
        #define TEST(x, args...)        NULL 
#endif

static int usb_insert_times = 1;
int usb_modules_status = 0;
EXPORT_SYMBOL(usb_modules_status);

#define	DRIVER_DESC	"TMSBG_BSP distinguishing usb adapter from usb"
#define	DRIVER_AUTHOR	"TMSBG_BSP02 DEMON"
#define	DRIVER_VERSION	"12 February 2012"


static struct clk *usb_phy1_clk;
static struct clk *usb_oh3_clk;
static struct clk *usb_ahb_clk;

u32 adapter_vir_base;


struct ep_queue_item {
	volatile unsigned int next_item_ptr;
	volatile unsigned int info;
	volatile unsigned int page0;
	volatile unsigned int page1;
	volatile unsigned int page2;
	volatile unsigned int page3;
	volatile unsigned int page4;
	unsigned int item_dma;
	unsigned int page_vir;
	unsigned int page_dma;
	struct ep_queue_item *next_item_vir;
	volatile unsigned int reserved[5];
};


typedef struct {
	int epnum;
	int dir;
	int max_pkt_size;
	struct usb_endpoint_instance *epi;
	struct ep_queue_item *ep_dtd[EP_TQ_ITEM_SIZE];
	int index; /* to index the free tx tqi */
	int done;  /* to index the complete rx tqi */
	struct ep_queue_item *tail; /* last item in the dtd chain */
	struct ep_queue_head *ep_qh;
} mxc_ep_t;



typedef struct {
	int    max_ep;
	int    ep0_dir;
	int    setaddr;
	struct ep_queue_head *ep_qh;
	mxc_ep_t *mxc_ep;
	u32    qh_dma;
} mxc_udc_ctrl;


static mxc_udc_ctrl mxc_udc;


static const char driver_name[] = "usb_adapter";
static const char driver_desc[] = DRIVER_DESC;

#undef DEBUG 
#ifdef DEBUG
#define DBG(x...) printk(x)
#else
#define DBG(x...) do {} while (0)
#endif

static void usb_phy_init(void)
{
	u32 temp;
	/* select 24M clk */
	temp = readl(USB_PHY1_CTRL);
	temp &= ~3;
	temp |= 1;
	writel(temp, USB_PHY1_CTRL);
	/* Config PHY interface */
	temp = readl(USB_PORTSC1);
	temp &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
	temp |= PORTSCX_PTW_16BIT;
	writel(temp, USB_PORTSC1);
	DBG("Config PHY  END\n");
}

static void usb_set_mode_device(void)
{
	u32 temp;

	/* Set controller to stop */
	temp = readl(USB_USBCMD);
	temp &= ~USB_CMD_RUN_STOP;
	writel(temp, USB_USBCMD);

	/* Do core reset */
	temp = readl(USB_USBCMD);
	temp |= USB_CMD_CTRL_RESET;
	writel(temp, USB_USBCMD);
	while (readl(USB_USBCMD) & USB_CMD_CTRL_RESET)
		;
	DBG("DOORE RESET END\n");

	DBG("init core to device mode\n");
	temp = readl(USB_USBMODE);
	temp &= ~USB_MODE_CTRL_MODE_MASK;	/* clear mode bits */
	temp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	temp |= USB_MODE_SETUP_LOCK_OFF;
	writel(temp, USB_USBMODE);
	DBG("init core to device mode end\n");
}


/*
 * Translate the virtual address of ram space to physical address
 * It is dependent on the implementation of mmu_init
 */
inline void *iomem_to_phys(unsigned long virt)
{
	if (virt >= 0xB0000000)
		return (void *)((virt - 0xB0000000) + 0x70000000);

	return (void *)virt;
}


/*
 * malloc an nocached memory
 * dmaaddr: phys address
 * size   : memory size
 * align  : alignment for this memroy
 * return : vir address(NULL when malloc failt)
*/
static void *malloc_dma_buffer(u32 *dmaaddr, int size, int align)
{
	int msize = (size + align  - 1);
	u32  vir_align;

	adapter_vir_base = (u32)kmalloc(msize,GFP_KERNEL); 
	//vir = ioremap_nocache(iomem_to_phys(vir), msize);
	memset((void *)adapter_vir_base, 0, msize);
	vir_align = (adapter_vir_base + align - 1) & (~(align - 1));
	//*dmaaddr = (u32)iomem_to_phys(vir_align);
	*dmaaddr = vir_align;
	DBG("vir addr %x, dma addr %x\n", vir_align, *dmaaddr);
	return (void *)vir_align;
}


static int mxc_init_usb_qh(void)
{
	int size;
	memset(&mxc_udc, 0, sizeof(mxc_udc));
	mxc_udc.max_ep = (readl(USB_DCCPARAMS) & DCCPARAMS_DEN_MASK) * 2;
	DBG("udc max ep = %d\n", mxc_udc.max_ep);
	size = mxc_udc.max_ep * sizeof(struct ep_queue_head);
	mxc_udc.ep_qh = malloc_dma_buffer(&mxc_udc.qh_dma,
					     size, USB_MEM_ALIGN_BYTE);
	if (!mxc_udc.ep_qh) {
		DBG("malloc ep qh dma buffer failure\n");
		return -1;
	}
	memset(mxc_udc.ep_qh, 0, size);
	DBG("mxc_udc.qh_dma & 0xfffff800 is 0x%x\n",mxc_udc.qh_dma & 0xfffff800);
	writel(mxc_udc.qh_dma & 0xfffff800, USB_ENDPOINTLISTADDR);

	return 0;
}

static void usb_init_eps(void)
{
	u32 temp;

	temp = readl(USB_ENDPTNAKEN);
	temp |= 0x10001;	/* clear mode bits */
	writel(temp, USB_ENDPTNAKEN);
	writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
	writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
	writel(0xffffffff, USB_ENDPTFLUSH);
	DBG("FLUSH END\n");
}

static int mxc_init_ep_struct(void)
{
	int i;

	DBG("init mxc ep\n");
	mxc_udc.mxc_ep = kmalloc(mxc_udc.max_ep * sizeof(mxc_ep_t),GFP_KERNEL);
	if (!mxc_udc.mxc_ep) {
		DBG("malloc ep struct failure\n");
		return -1;
	}
	memset((void *)mxc_udc.mxc_ep, 0, sizeof(mxc_ep_t) * mxc_udc.max_ep);
	for (i = 0; i < mxc_udc.max_ep / 2; i++) {
		mxc_ep_t *ep;
		ep  = mxc_udc.mxc_ep + i * 2;
		ep->epnum = i;
		ep->index = ep->done = 0;
		ep->dir = USB_RECV;  /* data from host to device */
		ep->ep_qh = &mxc_udc.ep_qh[i * 2];

		ep  = mxc_udc.mxc_ep + (i * 2 + 1);
		ep->epnum = i;
		ep->index = ep->done = 0;
		ep->dir = USB_SEND;  /* data to host from device */
		ep->ep_qh = &mxc_udc.ep_qh[(i * 2 + 1)];
	}
	return 0;
}


static void mxc_ep_qh_setup(u8 ep_num, u8 dir, u8 ep_type,
				 u32 max_pkt_len, u32 zlt, u8 mult)
{
	struct ep_queue_head *p_qh = mxc_udc.ep_qh + (2 * ep_num + dir);
	u32 tmp = 0;

	tmp = max_pkt_len << 16;
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		tmp |= (1 << 15);
		break;
	case USB_ENDPOINT_XFER_ISOC:
		tmp |= (mult << 30);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		break;
	default:
		DBG("error ep type is %d\n", ep_type);
		return;
	}
	if (zlt)
		tmp |= (1<<29);

	p_qh->config = tmp;
}


static void mxc_ep_setup(u8 ep_num, u8 dir, u8 ep_type)
{
	u32 epctrl = 0;
	epctrl = readl(USB_ENDPTCTRL(ep_num));
	if (dir) {
		if (ep_num)
			epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		epctrl |= EPCTRL_TX_ENABLE;
		epctrl |= ((u32)(ep_type) << EPCTRL_TX_EP_TYPE_SHIFT);
	} else {
		if (ep_num)
			epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		epctrl |= EPCTRL_RX_ENABLE;
		epctrl |= ((u32)(ep_type) << EPCTRL_RX_EP_TYPE_SHIFT);
	}
	writel(epctrl, USB_ENDPTCTRL(ep_num));
}

static int mxc_init_ep_dtd(u8 index)
{
	mxc_ep_t *ep;
	struct ep_queue_item *tqi;
	u32 dma;
	int i;

	if (index >= mxc_udc.max_ep)
		DBG("%s ep %d is not valid\n", __func__, index);

	ep = mxc_udc.mxc_ep + index;
	tqi = malloc_dma_buffer(&dma, EP_TQ_ITEM_SIZE *
			    EP_TQ_ITEM_SIZE * sizeof(struct ep_queue_item),
			    USB_MEM_ALIGN_BYTE);
	if (tqi == NULL) {
		DBG("%s malloc tq item failure\n", __func__);
		return -1;
	}
	for (i = 0; i < EP_TQ_ITEM_SIZE; i++) {
		ep->ep_dtd[i] = tqi + i;
		ep->ep_dtd[i]->item_dma =
			dma + i * sizeof(struct ep_queue_item);
	}
	return -1;
}



static void mxc_tqi_init_page(struct ep_queue_item *tqi)
{
	tqi->page0 = tqi->page_dma;
	tqi->page1 = tqi->page0 + 0x1000;
	tqi->page2 = tqi->page1 + 0x1000;
	tqi->page3 = tqi->page2 + 0x1000;
	tqi->page4 = tqi->page3 + 0x1000;
}
static int mxc_malloc_ep0_ptr(mxc_ep_t *ep)
{
	int i;
	struct ep_queue_item *tqi;
	int max_pkt_size = USB_MAX_CTRL_PAYLOAD;

	ep->max_pkt_size = max_pkt_size;
	for (i = 0; i < EP_TQ_ITEM_SIZE; i++) {
		tqi = ep->ep_dtd[i];
		tqi->page_vir = (u32)malloc_dma_buffer(&tqi->page_dma,
						    max_pkt_size,
						    USB_MEM_ALIGN_BYTE);
		if ((void *)tqi->page_vir == NULL) {
			DBG("malloc ep's dtd bufer failure, i=%d\n", i);
			return -1;
		}
		mxc_tqi_init_page(tqi);
	}
	return 0;
}


static void ep0_setup(void)
{
	mxc_ep_qh_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			    USB_MAX_CTRL_PAYLOAD, 0, 0);
	mxc_ep_qh_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			    USB_MAX_CTRL_PAYLOAD, 0, 0);
	mxc_ep_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	mxc_ep_setup(0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);
	mxc_init_ep_dtd(0 * 2 + USB_RECV);
	mxc_init_ep_dtd(0 * 2 + USB_SEND);
}

static  void mxc_usb_run(void)
{
	unsigned int temp = 0;
	/* Enable DR irq reg */
	temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
		| USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
		| USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;
	writel(temp, USB_USBINTR);
	
	/* Set controller to Run */
	temp = readl(USB_USBCMD);
	temp |= USB_CMD_RUN_STOP;
	writel(temp, USB_USBCMD);
	//DBG("Port Status(D+D-value) 	is 0x%x\n",(readl(USB_PORTSC1)>>10&0x3)) ;
}

static void mxc_udc_wait_cable_insert(void)
{
	u32 temp;
	int cable_connect = 1;
	int repeat = 4;
	do {
		udelay(50);
		temp = readl(USB_OTGSC);
		if (temp & (OTGSC_B_SESSION_VALID))
		{
			DBG("USB Mini cable Connected!\n");
			break;
		} else if (cable_connect == 1) 
		{
			DBG("wait usb cable into the connector!\n");
			cable_connect = 0;
		}
	} while (repeat--);
}

static void udc_connect(void)
{
	mxc_usb_run();	
	mxc_udc_wait_cable_insert();
}


/* Notes: configure USB clock*/
static void usbotg_init_ext(void)
{
	struct clk *usb_clk;

	/* the usb_ahb_clk will be enabled in usb_otg_init */
	usb_ahb_clk = clk_get(NULL, "usb_ahb_clk");

	usb_clk = clk_get(NULL, "usboh3_clk");
	clk_enable(usb_clk);
	usb_oh3_clk = usb_clk;

	usb_clk = clk_get(NULL, "usb_phy1_clk");
	clk_enable(usb_clk);
	usb_phy1_clk = usb_clk;
}



static void usbotg_clock_gate(bool on)
{
	pr_debug("%s: on is %d\n", __func__, on);
	if (on) {
		clk_enable(usb_ahb_clk);
		clk_enable(usb_oh3_clk);
		clk_enable(usb_phy1_clk);
	} else {
		clk_disable(usb_phy1_clk);
		clk_disable(usb_oh3_clk);
		clk_disable(usb_ahb_clk);
	}
}


static void mxc_usb_stop(void)
{
	unsigned int temp = 0;
	writel(temp, USB_USBINTR);
	/* Set controller to Stop */
	temp = readl(USB_USBCMD);
	temp &= ~USB_CMD_RUN_STOP;
	writel(temp, USB_USBCMD);
}

extern int usb_adapter_base;
extern int ccm_blk_base;

static void bsp_usb_dev_hand_reset(void)
{
	u32 temp;
	temp = readl(USB_DEVICEADDR);
	temp &= ~0xfe000000;
	writel(temp, USB_DEVICEADDR);
	writel(readl(USB_ENDPTSETUPSTAT), USB_ENDPTSETUPSTAT);
	writel(readl(USB_ENDPTCOMPLETE), USB_ENDPTCOMPLETE);
	while (readl(USB_ENDPTPRIME));
	writel(0xffffffff, USB_ENDPTFLUSH);
	DBG("reset-PORTSC=%x\n", readl(USB_PORTSC1));
	//usbd_device_event_irq(udc_device, DEVICE_RESET, 0);
}

int  bsp_mxc_irq_poll(void)
{
	unsigned irq_src = readl(USB_USBSTS) & readl(USB_USBINTR);
	if(0 != irq_src)
		DBG("irq_src = 0x%x\n",irq_src);	
	udelay(1000);
	writel(irq_src, USB_USBSTS);	
	if (irq_src & USB_STS_INT)
	{
		DBG("USB_INT\n");
		return 1;
	}
	if (irq_src & USB_STS_RESET)
	{
		DBG("USB_RESET\n");		
		return 2;
		//udelay(100);
		//bsp_usb_dev_hand_reset();
		
	}
	return 0;
}


void set_usboh3_clk(void)
{
	unsigned int reg;
	reg = readl(MXC_CCM_CSCMR1) & ~MXC_CCM_CSCMR1_USBOH3_CLK_SEL_MASK;
	reg |= 1 << MXC_CCM_CSCMR1_USBOH3_CLK_SEL_OFFSET;
	writel(reg, MXC_CCM_CSCMR1);
	reg = readl(MXC_CCM_CSCDR1);
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK;
	reg &= ~MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK;
	reg |= 4 << MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET;
	reg |= 1 << MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET;
	writel(reg, MXC_CCM_CSCDR1);
}

void set_usb_phy1_clk(void)
{
	unsigned int reg;

	reg = readl(MXC_CCM_CSCMR1);
	reg &= ~MXC_CCM_CSCMR1_USB_PHY_CLK_SEL;
	writel(reg, MXC_CCM_CSCMR1);
}

void enable_usboh3_clk(unsigned char enable)
{
	unsigned int reg;

	reg = readl(MXC_CCM_CCGR2);
	if (enable)
		reg |= 1 << MXC_CCM_CCGR2_CG14_OFFSET;
	else
		reg &= ~(1 << MXC_CCM_CCGR2_CG14_OFFSET);
	writel(reg, MXC_CCM_CCGR2);
}

void enable_usb_phy1_clk(unsigned char enable)
{
	unsigned int reg;

	reg = readl(MXC_CCM_CCGR4);
	if (enable)
		reg |= 1 << MXC_CCM_CCGR4_CG5_OFFSET;
	else
		reg &= ~(1 << MXC_CCM_CCGR4_CG5_OFFSET);
	writel(reg, MXC_CCM_CCGR4);
}



static void usb_udc_init(void)
{
	DBG("usb init start\n");
	
	usbotg_init_ext();
	usbotg_clock_gate(1);
	usb_phy_init();
	usb_set_mode_device();
	//mxc_init_usb_qh();
	//usb_init_eps();
	//mxc_init_ep_struct();
	//ep0_setup();
	//mxc_ep_qh_setup(0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			   // USB_MAX_CTRL_PAYLOAD, 0, 0);
	udc_connect();
}


int  bsp_usb_poll()
{
	int temp = 800,charg_statue = 0;
#if 0
	if(usb_insert_times)
	{
		usb_insert_times = 0;
		temp = 400;
	}
	else
	{
		temp = 350;
	}
#endif	
	while (temp--)
	{
		//DBG("temp=%d\n", temp);	
		if (bsp_mxc_irq_poll())
		{
			DBG("USB charging.........\n");
			charg_statue = 1;
			break;
		}
		
		if (0 == temp)
		{
			DBG("USB third-party Adapter.........\n");
			charg_statue = 0;
			break;
		}

	}
	return charg_statue;
}

int  usb_adapter_state(void)
{	
	//int port_stats = 0;
	int charg_statue = 0, err = 0;
	if (1 == usb_modules_status){
		TEST("### udc probed:usb charging ###\n");
		return 0;
	}
	
	usb_base = usb_adapter_base;	
	ccm_adapter__base = ccm_blk_base;
	disable_irq(18);
	usb_udc_init();
	charg_statue = bsp_usb_poll();
	//port_stats = readl(USB_PORTSC1)>>10&0x3;		
	mxc_usb_stop(); 		
	enable_irq(18);
	//usbotg_clock_gate(0);		
	//printk("port_stats =%d   charg_statue = %d\n",port_stats,charg_statue);
	//printk("  charg_statue = %d\n",charg_statue);
	if (0 == charg_statue)	
	{
		TEST("### USB 3rd adapter ###\n");
		return  ADAPTER_INSERT;
	}
	if (1 == charg_statue)
	{
		TEST("### Micro usb charging ###\n");
		return USB_INSERT;
	}

	TEST("### %s:unknown device:%d ###\n", __func__, charg_statue);
	return err;
}
