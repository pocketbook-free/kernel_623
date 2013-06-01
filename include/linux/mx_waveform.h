#define	WAVEFORM_RESERVED_SIZE 0x300000
#define WAVEFORM_BASE	0x75a00000

int mxc_waveform_init(void);
struct jerry_waveform_t
{
	unsigned char *waveform;
	unsigned long *checksum;
};



