#!/bin/sh

set -x

# enter scripts working directory
cd "$(cd "$(dirname "$0")" && pwd)"

KERNEL_PRJ="623"

if [ "$USER" = "jenkins" ] ;
then
	CROSS_PREFIX="/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
else
	CROSS_PREFIX="ccache /opt/freescale/usr/local/gcc-4.4.4-glibc-2.11.1-multilib-1.0/arm-fsl-linux-gnueabi/bin/arm-fsl-linux-gnueabi-"
	BUILD_DIR=/tmp/${KERNEL_PRJ}_bin
	HEADERS_DIR=/tmp/${KERNEL_PRJ}_headers
	MODULES_DIR=/tmp/${KERNEL_PRJ}_modules

	install -d $BUILD_DIR
	install -d $HEADERS_DIR
	install -d $MODULES_DIR
fi

NCPUS=`cat /proc/cpuinfo | grep processor | wc -l`

make INSTALL_HDR_PATH=$HEADERS_DIR INSTALL_MOD_PATH=$MODULES_DIR CROSS_COMPILE="$CROSS_PREFIX" ARCH=arm O=$BUILD_DIR -j $NCPUS $1

