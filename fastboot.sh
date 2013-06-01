#!/bin/sh
sudo fastboot flash kernel arch/arm/boot/uImage

sleep 2

sudo fastboot reboot
