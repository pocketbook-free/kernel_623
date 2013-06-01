/*
 * imx-3stack-ssm2602.c  --  i.MX 3Stack Driver for Analog SSM2602 Codec
 *
 *Base on Freescale i.MX 3Stack driver (sound/soc/imx/imx-3stack-sgtl5000.c) 
 *Copyright (C) 2008-2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    21th Oct 2008   Initial version.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/fsl_devices.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/hw_ops.h>
#include <mach/dma.h>
#include <mach/clock.h>

#include "../codecs/ssm2602.h"
#include "imx-ssi.h"
#include "imx-pcm.h"

/* SSI BCLK and LRC master */
#define SSM2602_SSI_MASTER	1

struct imx_3stack_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
};

static struct imx_3stack_priv card_priv;

static int imx_3stack_audio_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	struct imx_3stack_priv *priv = &card_priv;
	unsigned int rate = params_rate(params);
	struct imx_ssi *ssi_mode = (struct imx_ssi *)cpu_dai->private_data;
	int ret = 0;

	unsigned int channels = params_channels(params);
	u32 dai_format;

	/* only need to do this once as capture and playback are sync */
	if (priv->hw)
		return 0;
	priv->hw = 1;

	snd_soc_dai_set_sysclk(codec_dai, SSM2602_SYSCLK, priv->sysclk, 0); //12MHz

#if SSM2602_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBM_CFM;
#else
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;
#endif

	ssi_mode->sync_mode = 1;
	if (channels == 1)
		ssi_mode->network_mode = 0;
	else
		ssi_mode->network_mode = 1;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 channels == 1 ? 0xfffffffe : 0xfffffffc,
				 2, 32);

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0, SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	struct imx_3stack_priv *priv = &card_priv;

	priv->hw = 0;
}

/*
 * imx_3stack SSM2602 audio DAI opserations.
 */
static struct snd_soc_ops imx_3stack_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_audio_hw_params,
};

static void imx_3stack_init_dam(int ssi_port, int dai_port)
{
	unsigned int ssi_ptcr = 0;
	unsigned int dai_ptcr = 0;
	unsigned int ssi_pdcr = 0;
	unsigned int dai_pdcr = 0;
	/* SGTL5000 uses SSI1 or SSI2 via AUDMUX port dai_port for audio */

	/* reset port ssi_port & dai_port */
	__raw_writel(0, DAM_PTCR(ssi_port));
	__raw_writel(0, DAM_PTCR(dai_port));
	__raw_writel(0, DAM_PDCR(ssi_port));
	__raw_writel(0, DAM_PDCR(dai_port));

	/* set to synchronous */
	ssi_ptcr |= AUDMUX_PTCR_SYN;
	dai_ptcr |= AUDMUX_PTCR_SYN;

#if SSM2602_SSI_MASTER
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TFSDIR;
	ssi_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, dai_port);

	/* set Tx Clock direction and source dai_port--> ssi_port output */
	ssi_ptcr |= AUDMUX_PTCR_TCLKDIR;
	ssi_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, dai_port);
#else
	/* set Rx sources ssi_port <--> dai_port */
	ssi_pdcr |= AUDMUX_PDCR_RXDSEL(dai_port);
	dai_pdcr |= AUDMUX_PDCR_RXDSEL(ssi_port);

	/* set Tx frame direction and source  ssi_port --> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TFSDIR;
	dai_ptcr |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, ssi_port);

	/* set Tx Clock direction and source ssi_port--> dai_port output */
	dai_ptcr |= AUDMUX_PTCR_TCLKDIR;
	dai_ptcr |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, ssi_port);
#endif

	__raw_writel(ssi_ptcr, DAM_PTCR(ssi_port));
	__raw_writel(dai_ptcr, DAM_PTCR(dai_port));
	__raw_writel(ssi_pdcr, DAM_PDCR(ssi_port));
	__raw_writel(dai_pdcr, DAM_PDCR(dai_port));
}

static void headphone_detect_handler(struct work_struct *work)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	int hp_status;
	char *envp[3];
	char *buf;
  printk(KERN_INFO "%s : %d\n", __func__,plat->hp_status());
	sysfs_notify(&pdev->dev.kobj, NULL, "headphone");
	hp_status = plat->hp_status();
  hw_set_hp(hp_status);
	/* setup a message for userspace headphone in */
	buf = kmalloc(32, GFP_ATOMIC);
	if (!buf)
		kfree(buf);

	envp[0] = "NAME=headphone";
	snprintf(buf, 32, "STATE=%d", hp_status);
	envp[1] = buf;
	envp[2] = NULL;
	kobject_uevent_env(&pdev->dev.kobj, KOBJ_CHANGE, envp);
	kfree(buf);

	if (hp_status)
		set_irq_type(plat->hp_irq, IRQ_TYPE_EDGE_RISING);
	else
		set_irq_type(plat->hp_irq, IRQ_TYPE_EDGE_FALLING);
	enable_irq(plat->hp_irq);
}

static DECLARE_DELAYED_WORK(hp_event, headphone_detect_handler);

static irqreturn_t imx_headphone_detect_handler(int irq, void *data)
{
	disable_irq_nosync(irq);
	schedule_delayed_work(&hp_event, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static ssize_t show_headphone(struct device_driver *dev, char *buf)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct platform_device *pdev = priv->pdev;
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	u16 hp_status;

	/* determine whether hp is plugged in */
	hp_status = plat->hp_status();
	return sprintf(buf, "%d\n", hp_status);
}

static DRIVER_ATTR(headphone, S_IRUGO | S_IWUSR, show_headphone, NULL);


/* imx_3stack digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_3stack_dai = {
	.name = "SSM2602",
	.stream_name = "SSM2602",
	.codec_dai = &ssm2602_dai,
	//.init = imx_3stack_ssm2602_init,
	.ops = &imx_3stack_ops,
};

static int imx_3stack_card_remove(struct platform_device *pdev)
{
	struct imx_3stack_priv *priv = &card_priv;
	struct mxc_audio_platform_data *plat;
	if (priv->pdev) {
		plat = priv->pdev->dev.platform_data;
		if (plat->finit)
			plat->finit();
	}

	return 0;
}

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "imx-3stack",
	.platform = &imx_soc_platform,
	.dai_link = &imx_3stack_dai,
	.num_links = 1,
	.remove = imx_3stack_card_remove,
};

static struct snd_soc_device imx_3stack_snd_devdata = {
	.card = &snd_soc_card_imx_3stack,
	.codec_dev = &soc_codec_dev_ssm2602,
};

static int __devinit imx_3stack_ssm2602_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &card_priv;
	struct snd_soc_dai *ssm2602_cpu_dai;
	struct ssm2602_setup_data *setup;

	int ret = 0;

	priv->pdev = pdev;

	//gpio_activate_audio_ports();
	imx_3stack_init_dam(plat->src_port, plat->ext_port);

	if (plat->src_port == 2)
		ssm2602_cpu_dai = imx_ssi_dai[2];
	else if (plat->src_port == 1)
		ssm2602_cpu_dai = imx_ssi_dai[0];
	else if (plat->src_port == 7)
		ssm2602_cpu_dai = imx_ssi_dai[4];


	imx_3stack_dai.cpu_dai = ssm2602_cpu_dai;

	ret = driver_create_file(pdev->dev.driver, &driver_attr_headphone);
	if (ret < 0) {
		pr_err("%s:failed to create driver_attr_headphone\n", __func__);
		goto sysfs_err;
	}

	ret = -EINVAL;
	if (plat->init && plat->init())
		goto err_plat_init;

	priv->sysclk = plat->sysclk;

  printk(KERN_INFO "%s : %d\n", __func__,plat->hp_status());
	if (plat->hp_status())
		ret = request_irq(plat->hp_irq,
				  imx_headphone_detect_handler,
				  IRQ_TYPE_EDGE_RISING, pdev->name, priv); 
	else
		ret = request_irq(plat->hp_irq,
				  imx_headphone_detect_handler,
				  IRQ_TYPE_EDGE_FALLING, pdev->name, priv);  
	if (ret < 0) {
		pr_err("%s: request irq failed\n", __func__);
		goto err_card_reg;
	}

	setup = kzalloc(sizeof(struct ssm2602_setup_data), GFP_KERNEL);
	if (!setup) {
		pr_err("%s: kzalloc ssm2602_setup_data failed\n", __func__);
		goto err_card_reg;
	}
	setup->clock_enable = plat->clock_enable;
	setup->i2c_bus = 0;
	setup->i2c_address = 0x1a;
	imx_3stack_snd_devdata.codec_data = setup;

        hw_set_hp(plat->hp_status());
        //plat->hp_enable(plat->hp_status());
	
	return 0;

err_card_reg:
	if (plat->finit)
		plat->finit();
err_plat_init:
	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);
sysfs_err:
	return ret;
}

static int imx_3stack_ssm2602_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;
	struct imx_3stack_priv *priv = &card_priv;

	free_irq(plat->hp_irq, priv);

	if (plat->finit)
		plat->finit();

	driver_remove_file(pdev->dev.driver, &driver_attr_headphone);

	return 0;
}

static struct platform_driver imx_3stack_ssm2602_audio_driver = {
	.probe = imx_3stack_ssm2602_probe,
	.remove = imx_3stack_ssm2602_remove,
	.driver = {
		   .name = "imx-3stack-ssm2602",
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_init(void)
{
	int ret;

	ret = platform_driver_register(&imx_3stack_ssm2602_audio_driver);
	if (ret)
		return -ENOMEM;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 1);
	if (!imx_3stack_snd_device)
		return -ENOMEM;

	platform_set_drvdata(imx_3stack_snd_device, &imx_3stack_snd_devdata);
	imx_3stack_snd_devdata.dev = &imx_3stack_snd_device->dev;
	ret = platform_device_add(imx_3stack_snd_device);

	if (ret)
		platform_device_put(imx_3stack_snd_device);

	return ret;
}

static void __exit imx_3stack_exit(void)
{
	platform_driver_unregister(&imx_3stack_ssm2602_audio_driver);
	platform_device_unregister(imx_3stack_snd_device);
	pr_debug("IMX machine driver successfully unloaded\n");
}

module_init(imx_3stack_init);
module_exit(imx_3stack_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("SSM2602 Driver for i.MX 3STACK");
MODULE_LICENSE("GPL");
