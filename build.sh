#!/bin/bash

home=`pwd`
export home

TOOLS=${home}/tools/arm-2014.05/bin
CROSS_COMPILE=$TOOLS/arm-none-linux-gnueabi-


function build_kernel()
{
	# clean kernel
	#make distclean
	#rm -f ${home}/release/uImage
	# build kernel
	echo "-------------------build kernel ------------------------------------- start\n"
	make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE uImage
	cp arch/arm/boot/uImage ${home}/release
	echo "-------------------build kernel ------------------------------------- stop\n"
}

function config_kernel()
{
	# config kernel
	echo "-------------------config kernel ------------------------------------- start\n"
	make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE s3c2410_defconfig
	echo "-------------------config kernel ------------------------------------- stop\n"
}

function build_boot()
{
	# clean boot
	#rm -f ${home}/release/u-boot.bin
	#make distclean
	echo "-------------------build boot ------------------------------------- start\n"
	make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE
	cp u-boot.bin ${home}/release
	echo "-------------------build boot ------------------------------------- stop\n"
}

function config_boot()
{
	# config boot
	echo "-------------------config boot ------------------------------------- start\n"
	make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE jz2440_defconfig
	echo "-------------------config boot ------------------------------------- stop\n"
}

function menucfg_boot()
{
	# config boot
	echo "-------------------menuconfig boot ------------------------------------- start\n"
	make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE menuconfig
	echo "-------------------menuconfig boot ------------------------------------- stop\n"
}

function show_usage()
{
	echo "
	kernel select:
	----------------linux_2.6
	----------------linux_3.10
	boot select:
	----------------u-boot-2017.01
	----------------u-boot-2016.11
"
	exit 0
}

cd ${home}
if [ $1 ]; then
	if [ $1 = kernel ]; then
		cd ./kernel/linux-4.20.9
		build_kernel
		exit 0
	elif [ $1 = kernelconfig ]; then
		cd ./kernel/linux-4.20.9
		config_kernel
		exit 0
	elif [ $1 = boot ]; then
		cd ./boot/u-boot-2016.11
		build_boot
		exit 0
	elif [ $1 = bootconfig ]; then
		cd ./boot/u-boot-2016.11
		config_boot
		exit 0
	elif [ $1 = bootmenucfg ]; then
		cd ./boot/u-boot-2016.11
		menucfg_boot
		exit 0
	fi
else 
	show_usage
fi
