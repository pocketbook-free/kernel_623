#ifndef __TMSBG_UDC_H
#define __TMSBG_UDC_H


#define EP_TQ_ITEM_SIZE 4


#define USB_MEM_ALIGN_BYTE  4096

void __iomem *usb_base;
void __iomem *ccm_adapter__base;


#define USB_ID               (usb_base + 0x0000)
#define USB_HWGENERAL        (usb_base + 0x0004)
#define USB_HWHOST           (usb_base + 0x0008)
#define USB_HWDEVICE         (usb_base + 0x000C)
#define USB_HWTXBUF          (usb_base + 0x0010)
#define USB_HWRXBUF          (usb_base + 0x0014)
#define USB_SBUSCFG          (usb_base + 0x0090)

#define USB_CAPLENGTH        (usb_base + 0x0100) /* 8 bit */
#define USB_HCIVERSION       (usb_base + 0x0102) /* 16 bit */
#define USB_HCSPARAMS        (usb_base + 0x0104)
#define USB_HCCPARAMS        (usb_base + 0x0108)
#define USB_DCIVERSION       (usb_base + 0x0120) /* 16 bit */
#define USB_DCCPARAMS        (usb_base + 0x0124)
#define USB_USBCMD           (usb_base + 0x0140)
#define USB_USBSTS           (usb_base + 0x0144)
#define USB_USBINTR          (usb_base + 0x0148)
#define USB_FRINDEX          (usb_base + 0x014C)
#define USB_DEVICEADDR       (usb_base + 0x0154)
#define USB_ENDPOINTLISTADDR (usb_base + 0x0158)
#define USB_BURSTSIZE        (usb_base + 0x0160)
#define USB_TXFILLTUNING     (usb_base + 0x0164)
#define USB_ULPI_VIEWPORT    (usb_base + 0x0170)
#define USB_ENDPTNAK         (usb_base + 0x0178)
#define USB_ENDPTNAKEN       (usb_base + 0x017C)
#define USB_PORTSC1          (usb_base + 0x0184)
#define USB_OTGSC            (usb_base + 0x01A4)
#define USB_USBMODE          (usb_base + 0x01A8)
#define USB_ENDPTSETUPSTAT   (usb_base + 0x01AC)
#define USB_ENDPTPRIME       (usb_base + 0x01B0)
#define USB_ENDPTFLUSH       (usb_base + 0x01B4)
#define USB_ENDPTSTAT        (usb_base + 0x01B8)
#define USB_ENDPTCOMPLETE    (usb_base + 0x01BC)
#define USB_CTRL        (usb_base + 0x800)
#define USB_PHY1_CTRL        (usb_base + 0x80C)
#define USB_ENDPTCTRL(n)     (usb_base + 0x01C0 + (4 * (n)))



/* Register addresses of CCM*/
#define MXC_CCM_CCR			(ccm_adapter__base + 0x00)
#define MXC_CCM_CCDR		(ccm_adapter__base + 0x04) /* Reserved */
#define MXC_CCM_CSR			(ccm_adapter__base + 0x08)
#define MXC_CCM_CCSR			(ccm_adapter__base + 0x0C)
#define MXC_CCM_CACRR		(ccm_adapter__base + 0x10)
#define MXC_CCM_CBCDR		(ccm_adapter__base + 0x14)
#define MXC_CCM_CBCMR		(ccm_adapter__base + 0x18)
#define MXC_CCM_CSCMR1		(ccm_adapter__base + 0x1C)
#define MXC_CCM_CSCMR2		(ccm_adapter__base + 0x20) /* Reserved */
#define MXC_CCM_CSCDR1		(ccm_adapter__base + 0x24)
#define MXC_CCM_CS1CDR		(ccm_adapter__base + 0x28)
#define MXC_CCM_CS2CDR		(ccm_adapter__base + 0x2C)
#define MXC_CCM_CDCDR		(ccm_adapter__base + 0x30) /* Reserved */
#define MXC_CCM_CHSCDR		(ccm_adapter__base + 0x34) /* Reserved */
#define MXC_CCM_CSCDR2		(ccm_adapter__base + 0x38)
#define MXC_CCM_CSCDR3		(ccm_adapter__base + 0x3C) /* Reserved */
#define MXC_CCM_CSCDR4		(ccm_adapter__base + 0x40) /* Reserved */
#define MXC_CCM_CWDR		(ccm_adapter__base + 0x44) /* Reserved */
#define MXC_CCM_CDHIPR		(ccm_adapter__base + 0x48)
#define MXC_CCM_CDCR		(ccm_adapter__base + 0x4C)
#define MXC_CCM_CTOR		(ccm_adapter__base + 0x50)
#define MXC_CCM_CLPCR		(ccm_adapter__base + 0x54)
#define MXC_CCM_CISR			(ccm_adapter__base + 0x58)
#define MXC_CCM_CIMR		(ccm_adapter__base + 0x5C)
#define MXC_CCM_CCOSR		(ccm_adapter__base + 0x60)
#define MXC_CCM_CGPR		(ccm_adapter__base + 0x64) /* Reserved */
#define MXC_CCM_CCGR0		(ccm_adapter__base + 0x68)
#define MXC_CCM_CCGR1		(ccm_adapter__base + 0x6C)
#define MXC_CCM_CCGR2		(ccm_adapter__base + 0x70)
#define MXC_CCM_CCGR3		(ccm_adapter__base + 0x74)
#define MXC_CCM_CCGR4		(ccm_adapter__base + 0x78)
#define MXC_CCM_CCGR5		(ccm_adapter__base + 0x7C)
#define MXC_CCM_CCGR6		(ccm_adapter__base + 0x80)
#define MXC_CCM_CCGR7		(ccm_adapter__base + 0x84)
#define MXC_CCM_CMEOR		(ccm_adapter__base + 0x88)
#define MXC_CCM_CSR2			(ccm_adapter__base + 0x8C)
#define MXC_CCM_CLKSEQ_BYPASS	(ccm_adapter__base + 0x90)
#define MXC_CCM_CLK_SYS			(ccm_adapter__base + 0x94)
#define MXC_CCM_CLK_DDR		(ccm_adapter__base + 0x98)
#define MXC_CCM_GPMI			(ccm_adapter__base + 0xac)
#define MXC_CCM_BCH				(ccm_adapter__base + 0xb0)
	

/* Define the bits in register CSCMR1 */
#define MXC_CCM_CSCMR1_SSI_EXT2_CLK_SEL_OFFSET		(30)
#define MXC_CCM_CSCMR1_SSI_EXT2_CLK_SEL_MASK		(0x3 << 30)
#define MXC_CCM_CSCMR1_SSI_EXT1_CLK_SEL_OFFSET		(28)
#define MXC_CCM_CSCMR1_SSI_EXT1_CLK_SEL_MASK		(0x3 << 28)
#define MXC_CCM_CSCMR1_USB_PHY_CLK_SEL_OFFSET		(26)
#define MXC_CCM_CSCMR1_USB_PHY_CLK_SEL			(0x1 << 26)
#define MXC_CCM_CSCMR1_UART_CLK_SEL_OFFSET			(24)
#define MXC_CCM_CSCMR1_UART_CLK_SEL_MASK			(0x3 << 24)
#define MXC_CCM_CSCMR1_USBOH3_CLK_SEL_OFFSET		(22)
#define MXC_CCM_CSCMR1_USBOH3_CLK_SEL_MASK			(0x3 << 22)
#define MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_OFFSET	(20)
#define MXC_CCM_CSCMR1_ESDHC1_MSHC2_CLK_SEL_MASK		(0x3 << 20)
#define MXC_CCM_CSCMR1_ESDHC3_CLK_SEL_MX51		(0x1 << 19)
#define MXC_CCM_CSCMR1_ESDHC2_CLK_SEL			(0x1 << 19)
#define MXC_CCM_CSCMR1_ESDHC4_CLK_SEL			(0x1 << 18)
#define MX50_CCM_CSCMR1_ESDHC1_CLK_SEL_OFFSET	(21)
#define MX50_CCM_CSCMR1_ESDHC1_CLK_SEL_MASK	(0x3 << 21)
#define MX50_CCM_CSCMR1_ESDHC2_CLK_SEL			(0x1 << 20)
#define MX50_CCM_CSCMR1_ESDHC4_CLK_SEL			(0x1 << 19)
#define MX50_CCM_CSCMR1_ESDHC3_CLK_SEL_OFFSET	(16)
#define MX50_CCM_CSCMR1_ESDHC3_CLK_SEL_MASK		(0x7 << 16)
#define MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_OFFSET	(16)
#define MXC_CCM_CSCMR1_ESDHC3_MSHC2_CLK_SEL_MASK		(0x3 << 16)
#define MXC_CCM_CSCMR1_SSI1_CLK_SEL_OFFSET			(14)
#define MXC_CCM_CSCMR1_SSI1_CLK_SEL_MASK			(0x3 << 14)
#define MXC_CCM_CSCMR1_SSI2_CLK_SEL_OFFSET			(12)
#define MXC_CCM_CSCMR1_SSI2_CLK_SEL_MASK			(0x3 << 12)
#define MXC_CCM_CSCMR1_SSI3_CLK_SEL				(0x1 << 11)
#define MXC_CCM_CSCMR1_VPU_RCLK_SEL				(0x1 << 10)
#define MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_OFFSET		(8)
#define MXC_CCM_CSCMR1_SSI_APM_CLK_SEL_MASK		(0x3 << 8)
#define MXC_CCM_CSCMR1_TVE_CLK_SEL				(0x1 << 7)
#define MXC_CCM_CSCMR1_TVE_EXT_CLK_SEL			(0x1 << 6)
#define MXC_CCM_CSCMR1_CSPI_CLK_SEL_OFFSET			(4)
#define MXC_CCM_CSCMR1_CSPI_CLK_SEL_MASK			(0x3 << 4)
#define MXC_CCM_CSCMR1_SPDIF_CLK_SEL_OFFSET			(2)
#define MXC_CCM_CSCMR1_SPDIF_CLK_SEL_MASK			(0x3 << 2)
#define MXC_CCM_CSCMR1_SSI_EXT2_COM_CLK_SEL			(0x1 << 1)
#define MXC_CCM_CSCMR1_SSI_EXT1_COM_CLK_SEL			(0x1)

#define MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK		(0x3 << 6)


#define MXC_CCM_CCGR4_CG5_OFFSET			10

/* Define the bits in register CSCDR1 */
#define MXC_CCM_CSCDR1_ESDHC3_CLK_PRED_OFFSET		(22)
#define MXC_CCM_CSCDR1_ESDHC3_CLK_PRED_MASK		(0x7 << 22)
#define MXC_CCM_CSCDR1_ESDHC3_CLK_PODF_OFFSET		(19)
#define MXC_CCM_CSCDR1_ESDHC3_CLK_PODF_MASK		(0x7 << 19)
#define MXC_CCM_CSCDR1_ESDHC1_CLK_PRED_OFFSET		(16)
#define MXC_CCM_CSCDR1_ESDHC1_CLK_PRED_MASK		(0x7 << 16)
#define MXC_CCM_CSCDR1_PGC_CLK_PODF_OFFSET		(14)
#define MXC_CCM_CSCDR1_PGC_CLK_PODF_MASK		(0x3 << 14)
#define MXC_CCM_CSCDR1_ESDHC1_CLK_PODF_OFFSET		(11)
#define MXC_CCM_CSCDR1_ESDHC1_CLK_PODF_MASK		(0x7 << 11)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PRED_OFFSET		(8)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PRED_MASK		(0x7 << 8)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PODF_OFFSET		(6)
#define MXC_CCM_CSCDR1_USBOH3_CLK_PODF_MASK		(0x3 << 6)
#define MXC_CCM_CSCDR1_UART_CLK_PRED_OFFSET		(3)
#define MXC_CCM_CSCDR1_UART_CLK_PRED_MASK		(0x7 << 3)
#define MXC_CCM_CSCDR1_UART_CLK_PODF_OFFSET		(0)
#define MXC_CCM_CSCDR1_UART_CLK_PODF_MASK		(0x7)


#define MXC_CCM_CCGR2_CG15_OFFSET			30
#define MXC_CCM_CCGR2_CG14_OFFSET			28
#define MXC_CCM_CCGR2_CG13_OFFSET			26
#define MXC_CCM_CCGR2_CG12_OFFSET			24
#define MXC_CCM_CCGR2_CG11_OFFSET			22
#define MXC_CCM_CCGR2_CG10_OFFSET			20
#define MXC_CCM_CCGR2_CG9_OFFSET			18
#define MXC_CCM_CCGR2_CG8_OFFSET			16
#define MXC_CCM_CCGR2_CG7_OFFSET			14
#define MXC_CCM_CCGR2_CG6_OFFSET			12
#define MXC_CCM_CCGR2_CG5_OFFSET			10
#define MXC_CCM_CCGR2_CG4_OFFSET			8
#define MXC_CCM_CCGR2_CG3_OFFSET			6
#define MXC_CCM_CCGR2_CG2_OFFSET			4
#define MXC_CCM_CCGR2_CG1_OFFSET			2
#define MXC_CCM_CCGR2_CG0_OFFSET			0


/* USB STS Register Bit Masks */
#define  USB_STS_INT                          (0x00000001)
#define  USB_STS_ERR                          (0x00000002)
#define  USB_STS_PORT_CHANGE                  (0x00000004)
#define  USB_STS_FRM_LST_ROLL                 (0x00000008)
#define  USB_STS_SYS_ERR                      (0x00000010)
#define  USB_STS_IAA                          (0x00000020)
#define  USB_STS_RESET                        (0x00000040)
#define  USB_STS_SOF                          (0x00000080)
#define  USB_STS_SUSPEND                      (0x00000100)
#define  USB_STS_HC_HALTED                    (0x00001000)
#define  USB_STS_RCL                          (0x00002000)
#define  USB_STS_PERIODIC_SCHEDULE            (0x00004000)
#define  USB_STS_ASYNC_SCHEDULE               (0x00008000)


/* PORTSCX  Register Bit Masks */
#define  PORTSCX_CURRENT_CONNECT_STATUS       (0x00000001)
#define  PORTSCX_CONNECT_STATUS_CHANGE        (0x00000002)
#define  PORTSCX_PORT_ENABLE                  (0x00000004)
#define  PORTSCX_PORT_EN_DIS_CHANGE           (0x00000008)
#define  PORTSCX_OVER_CURRENT_ACT             (0x00000010)
#define  PORTSCX_OVER_CURRENT_CHG             (0x00000020)
#define  PORTSCX_PORT_FORCE_RESUME            (0x00000040)
#define  PORTSCX_PORT_SUSPEND                 (0x00000080)
#define  PORTSCX_PORT_RESET                   (0x00000100)
#define  PORTSCX_LINE_STATUS_BITS             (0x00000C00)
#define  PORTSCX_PORT_POWER                   (0x00001000)
#define  PORTSCX_PORT_INDICTOR_CTRL           (0x0000C000)
#define  PORTSCX_PORT_TEST_CTRL               (0x000F0000)
#define  PORTSCX_WAKE_ON_CONNECT_EN           (0x00100000)
#define  PORTSCX_WAKE_ON_CONNECT_DIS          (0x00200000)
#define  PORTSCX_WAKE_ON_OVER_CURRENT         (0x00400000)
#define  PORTSCX_PHY_LOW_POWER_SPD            (0x00800000)
#define  PORTSCX_PORT_FORCE_FULL_SPEED        (0x01000000)
#define  PORTSCX_PORT_SPEED_MASK              (0x0C000000)
#define  PORTSCX_PORT_WIDTH                   (0x10000000)
#define  PORTSCX_PHY_TYPE_SEL                 (0xC0000000)

struct urb;

struct usb_endpoint_instance;
struct usb_interface_instance;
struct usb_configuration_instance;
struct usb_device_instance;
struct usb_bus_instance;
extern int  usb_adapter_state(void);

/*
 * Device Events
 *
 * These are defined in the USB Spec (c.f USB Spec 2.0 Figure 9-1).
 *
 * There are additional events defined to handle some extra actions we need
 * to have handled.
 *
 */
typedef enum usb_device_event {

	DEVICE_UNKNOWN,		/* bi - unknown event */
	DEVICE_INIT,		/* bi  - initialize */
	DEVICE_CREATE,		/* bi  - */
	DEVICE_HUB_CONFIGURED,	/* bi  - bus has been plugged int */
	DEVICE_RESET,		/* bi  - hub has powered our port */

	DEVICE_ADDRESS_ASSIGNED,	/* ep0 - set address setup received */
	DEVICE_CONFIGURED,	/* ep0 - set configure setup received */
	DEVICE_SET_INTERFACE,	/* ep0 - set interface setup received */

	DEVICE_SET_FEATURE,	/* ep0 - set feature setup received */
	DEVICE_CLEAR_FEATURE,	/* ep0 - clear feature setup received */

	DEVICE_DE_CONFIGURED,	/* ep0 - set configure setup received for ?? */

	DEVICE_BUS_INACTIVE,	/* bi  - bus in inactive (no SOF packets) */
	DEVICE_BUS_ACTIVITY,	/* bi  - bus is active again */

	DEVICE_POWER_INTERRUPTION,	/* bi  - hub has depowered our port */
	DEVICE_HUB_RESET,	/* bi  - bus has been unplugged */
	DEVICE_DESTROY,		/* bi  - device instance should be destroyed */

	DEVICE_HOTPLUG,		/* bi  - a hotplug event has occured */

	DEVICE_FUNCTION_PRIVATE,	/* function - private */

} usb_device_event_t;





typedef struct urb_link {
	struct urb_link *next;
	struct urb_link *prev;
} urb_link;

/* USB Data structure - for passing data around.
 *
 * This is used for both sending and receiving data.
 *
 * The callback function is used to let the function driver know when
 * transmitted data has been sent.
 *
 * The callback function is set by the alloc_recv function when an urb is
 * allocated for receiving data for an endpoint and used to call the
 * function driver to inform it that data has arrived.
 */

/* in linux we'd malloc this, but in u-boot we prefer static data */
#define URB_BUF_SIZE 256

/* USB Requests
 *
 */

struct usb_device_request {
	u8 bmRequestType;
	u8 bRequest;
	u16 wValue;
	u16 wIndex;
	u16 wLength;
} __attribute__ ((packed));


/* USB Status
 *
 */
typedef enum urb_send_status {
	SEND_IN_PROGRESS,
	SEND_FINISHED_OK,
	SEND_FINISHED_ERROR,
	RECV_READY,
	RECV_OK,
	RECV_ERROR
} urb_send_status_t;

struct usb_configuration_descriptor {
	u8 bLength;
	u8 bDescriptorType;	/* 0x2 */
	u16 wTotalLength;
	u8 bNumInterfaces;
	u8 bConfigurationValue;
	u8 iConfiguration;
	u8 bmAttributes;
	u8 bMaxPower;
} __attribute__ ((packed));

typedef enum usb_device_status {
	USBD_OPENING,		/* we are currently opening */
	USBD_OK,		/* ok to use */
	USBD_SUSPENDED,		/* we are currently suspended */
	USBD_CLOSING,		/* we are currently closing */
} usb_device_status_t;



/*
 * Device State (c.f USB Spec 2.0 Figure 9-1)
 *
 * What state the usb device is in.
 *
 * Note the state does not change if the device is suspended, we simply set a
 * flag to show that it is suspended.
 *
 */
typedef enum usb_device_state {
	STATE_INIT,		/* just initialized */
	STATE_CREATED,		/* just created */
	STATE_ATTACHED,		/* we are attached */
	STATE_POWERED,		/* we have seen power indication (electrical bus signal) */
	STATE_DEFAULT,		/* we been reset */
	STATE_ADDRESSED,	/* we have been addressed (in default configuration) */
	STATE_CONFIGURED,	/* we have seen a set configuration device command */
	STATE_UNKNOWN,		/* destroyed */
} usb_device_state_t;


struct ep_queue_head {
	volatile unsigned int config;
	volatile unsigned int next_queue_item;
	volatile unsigned int info;
	volatile unsigned int page0;
	volatile unsigned int page1;
	volatile unsigned int page2;
	volatile unsigned int page3;
	volatile unsigned int page4;
	volatile unsigned int reserved_0;
	volatile unsigned char setup_data[8];
	volatile unsigned int reserved[4];
};



struct urb {

	struct usb_endpoint_instance *endpoint;
	struct usb_device_instance *device;

	struct usb_device_request device_request;	/* contents of received SETUP packet */

	struct urb_link link;	/* embedded struct for circular doubly linked list of urbs */

	u8* buffer;
	unsigned int buffer_length;
	unsigned int actual_length;

	urb_send_status_t status;
	int data;

	u16 buffer_data[URB_BUF_SIZE];	/* data received (OUT) or being sent (IN) */
};

/* Endpoint configuration
 *
 * Per endpoint configuration data. Used to track which function driver owns
 * an endpoint.
 *
 */
struct usb_endpoint_instance {
	int endpoint_address;	/* logical endpoint address */

	/* control */
	int status;		/* halted */
	int state;		/* available for use by bus interface driver */

	/* receive side */
	struct urb_link rcv;	/* received urbs */
	struct urb_link rdy;	/* empty urbs ready to receive */
	struct urb *rcv_urb;	/* active urb */
	int rcv_attributes;	/* copy of bmAttributes from endpoint descriptor */
	int rcv_packetSize;	/* maximum packet size from endpoint descriptor */
	int rcv_transferSize;	/* maximum transfer size from function driver */
	int rcv_queue;

	/* transmit side */
	struct urb_link tx;	/* urbs ready to transmit */
	struct urb_link done;	/* transmitted urbs */
	struct urb *tx_urb;	/* active urb */
	int tx_attributes;	/* copy of bmAttributes from endpoint descriptor */
	int tx_packetSize;	/* maximum packet size from endpoint descriptor */
	int tx_transferSize;	/* maximum transfer size from function driver */
	int tx_queue;

	int sent;		/* data already sent */
	int last;		/* data sent in last packet XXX do we need this */
};

struct usb_alternate_instance {
	struct usb_interface_descriptor *interface_descriptor;

	int endpoints;
	int *endpoint_transfersize_array;
	struct usb_endpoint_descriptor **endpoints_descriptor_array;
};

struct usb_interface_instance {
	int alternates;
	struct usb_alternate_instance *alternates_instance_array;
};

struct usb_configuration_instance {
	int interfaces;
	struct usb_configuration_descriptor *configuration_descriptor;
	struct usb_interface_instance *interface_instance_array;
};


/* USB Device Instance
 *
 * For each physical bus interface we create a logical device structure. This
 * tracks all of the required state to track the USB HOST's view of the device.
 *
 * Keep track of the device configuration for a real physical bus interface,
 * this includes the bus interface, multiple function drivers, the current
 * configuration and the current state.
 *
 * This will show:
 *	the specific bus interface driver
 *	the default endpoint 0 driver
 *	the configured function driver
 *	device state
 *	device status
 *	endpoint list
 */


struct usb_device_instance {

	/* generic */
	char *name;
	struct usb_device_descriptor *device_descriptor;	/* per device descriptor */

	void (*event) (struct usb_device_instance *device, usb_device_event_t event, int data);

	/* Do cdc device specific control requests */
	int (*cdc_recv_setup)(struct usb_device_request *request, struct urb *urb);

	/* bus interface */
	struct usb_bus_instance *bus;	/* which bus interface driver */

	/* configuration descriptors */
	int configurations;
	struct usb_configuration_instance *configuration_instance_array;

	/* device state */
	usb_device_state_t device_state;	/* current USB Device state */
	usb_device_state_t device_previous_state;	/* current USB Device state */

	u8 address;		/* current address (zero is default) */
	u8 configuration;	/* current show configuration (zero is default) */
	u8 interface;		/* current interface (zero is default) */
	u8 alternate;		/* alternate flag */

	usb_device_status_t status;	/* device status */

	int urbs_queued;	/* number of submitted urbs */

	/* Shouldn't need to make this atomic, all we need is a change indicator */
	unsigned long usbd_rxtx_timestamp;
	unsigned long usbd_last_rxtx_timestamp;

};

/* Bus Interface configuration structure
 *
 * This is allocated for each configured instance of a bus interface driver.
 *
 * The privdata pointer may be used by the bus interface driver to store private
 * per instance state information.
 */
struct usb_bus_instance {

	struct usb_device_instance *device;
	struct usb_endpoint_instance *endpoint_array;	/* array of available configured endpoints */

	int max_endpoints;	/* maximimum number of rx enpoints */
	unsigned char			maxpacketsize;

	unsigned int serial_number;
	char *serial_number_str;
	void *privdata;		/* private data for the bus interface */

};



/* Endpoints */
#define USB_ENDPOINT_NUMBER_MASK  0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK     0x80

#define USB_ENDPOINT_XFERTYPE_MASK 0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL  0
#define USB_ENDPOINT_XFER_ISOC     1
#define USB_ENDPOINT_XFER_BULK     2
#define USB_ENDPOINT_XFER_INT      3

/* ENDPOINTCTRLx  Register Bit Masks */
#define  EPCTRL_TX_ENABLE                     (0x00800000)
#define  EPCTRL_TX_DATA_TOGGLE_RST            (0x00400000)	/* Not EP0 */
#define  EPCTRL_TX_DATA_TOGGLE_INH            (0x00200000)	/* Not EP0 */
#define  EPCTRL_TX_TYPE                       (0x000C0000)
#define  EPCTRL_TX_DATA_SOURCE                (0x00020000)	/* Not EP0 */
#define  EPCTRL_TX_EP_STALL                   (0x00010000)
#define  EPCTRL_RX_ENABLE                     (0x00000080)
#define  EPCTRL_RX_DATA_TOGGLE_RST            (0x00000040)	/* Not EP0 */
#define  EPCTRL_RX_DATA_TOGGLE_INH            (0x00000020)	/* Not EP0 */
#define  EPCTRL_RX_TYPE                       (0x0000000C)
#define  EPCTRL_RX_DATA_SINK                  (0x00000002)	/* Not EP0 */
#define  EPCTRL_RX_EP_STALL                   (0x00000001)

#define USB_RECV 0
#define USB_SEND 1
#define USB_MAX_CTRL_PAYLOAD 64


/* bit 19-18 and 3-2 are endpoint type */
#define  EPCTRL_EP_TYPE_CONTROL               (0)
#define  EPCTRL_EP_TYPE_ISO                   (1)
#define  EPCTRL_EP_TYPE_BULK                  (2)
#define  EPCTRL_EP_TYPE_INTERRUPT             (3)
#define  EPCTRL_TX_EP_TYPE_SHIFT              (18)
#define  EPCTRL_RX_EP_TYPE_SHIFT              (2)



/* Device Controller Capability Parameter register */
#define DCCPARAMS_DC				0x00000080
#define DCCPARAMS_DEN_MASK			0x0000001f

/* bit 28 is parallel transceiver width for UTMI interface */
#define  PORTSCX_PTW                          (0x10000000)
#define  PORTSCX_PTW_8BIT                     (0x00000000)
#define  PORTSCX_PTW_16BIT                    (0x10000000)

/* USB MODE Register Bit Masks */
#define  USB_MODE_CTRL_MODE_IDLE              (0x00000000)
#define  USB_MODE_CTRL_MODE_DEVICE            (0x00000002)
#define  USB_MODE_CTRL_MODE_HOST              (0x00000003)
#define  USB_MODE_CTRL_MODE_MASK              0x00000003
#define  USB_MODE_CTRL_MODE_RSV               (0x00000001)
#define  USB_MODE_ES                          0x00000004 /* (big) Endian Sel */
#define  USB_MODE_SETUP_LOCK_OFF              (0x00000008)
#define  USB_MODE_STREAM_DISABLE              (0x00000010)
/* Endpoint Flush Register */
#define EPFLUSH_TX_OFFSET		      (0x00010000)
#define EPFLUSH_RX_OFFSET		      (0x00000000)

/* USB CMD  Register Bit Masks */
#define  USB_CMD_RUN_STOP                     (0x00000001)
#define  USB_CMD_CTRL_RESET                   (0x00000002)
#define  USB_CMD_PERIODIC_SCHEDULE_EN         (0x00000010)
#define  USB_CMD_ASYNC_SCHEDULE_EN            (0x00000020)
#define  USB_CMD_INT_AA_DOORBELL              (0x00000040)
#define  USB_CMD_ASP                          (0x00000300)
#define  USB_CMD_ASYNC_SCH_PARK_EN            (0x00000800)
#define  USB_CMD_SUTW                         (0x00002000)
#define  USB_CMD_ATDTW                        (0x00004000)
#define  USB_CMD_ITC                          (0x00FF0000)

/* USB INTR Register Bit Masks */
#define  USB_INTR_INT_EN                      (0x00000001)
#define  USB_INTR_ERR_INT_EN                  (0x00000002)
#define  USB_INTR_PTC_DETECT_EN               (0x00000004)
#define  USB_INTR_FRM_LST_ROLL_EN             (0x00000008)
#define  USB_INTR_SYS_ERR_EN                  (0x00000010)
#define  USB_INTR_ASYN_ADV_EN                 (0x00000020)
#define  USB_INTR_RESET_EN                    (0x00000040)
#define  USB_INTR_SOF_EN                      (0x00000080)
#define  USB_INTR_DEVICE_SUSPEND              (0x00000100)
/* OTGSC Register Bit Masks */
#define  OTGSC_ID_CHANGE_IRQ_STS		(1 << 16)
#define  OTGSC_B_SESSION_VALID_IRQ_EN           (1 << 27)
#define  OTGSC_B_SESSION_VALID_IRQ_STS          (1 << 19)
#define  OTGSC_B_SESSION_VALID                  (1 << 11)
#define  OTGSC_A_BUS_VALID			(1 << 9)

#endif				/* __TMSBG_UDC_H */



