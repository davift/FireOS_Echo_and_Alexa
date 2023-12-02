################################################################################
#
#  build_kernel_config.sh
#
#  Copyright (c) 2020-2022 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
################################################################################

KERNEL_SUBPATH="kernel/mediatek/4.9"
DEFCONFIG_NAME="mt8512_defconfig"
TARGET_ARCH="arm"
TOOLCHAIN_PREFIX="arm-linux-gnueabi-"
MAKE_DTBS=y

# Expected image files are seperated with ":"
KERNEL_IMAGES="arch/arm/boot/Image:arch/arm/boot/zImage"

################################################################################
# NOTE: You must fill in the following with the path to a copy of
#       arm-linux-gnueabi-4.9.4-2017.01 compiler.
################################################################################
CROSS_COMPILER_PATH=""
