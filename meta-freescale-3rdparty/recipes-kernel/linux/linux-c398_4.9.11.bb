# Copyright (C) 2016 WinSystems, Inc.
# Released under the MIT license (see COPYING.MIT for the terms)

SUMMARY = "Linux Kernel provided and supported by WinSystems, Inc."

#include recipes-kernel/linux/linux-fslc.inc
require recipes-kernel/linux/linux-imx.inc
require recipes-kernel/linux/linux-dtb.inc

DEPENDS += "lzop-native bc-native"

SRCBRANCH = "imx_4.9.11_1.0.0_ga"
LOCALVERSION = "-1.0.0"
SRCREV = "c27010d99a3d91703ea2d1a3f9630a9dedc3f86f"

SRC_URI += " \
           file://0001-imx6q-c398-Add-c398-device-tree-to-Makefile.patch \
           file://imx6q-c398.dts \
"

DEFAULT_PREFERENCE = "1"

do_compile_prepend() {
         cp -p ${WORKDIR}/imx6q-c398.dts ${S}/arch/arm/boot/dts
}

COMPATIBLE_MACHINE = "(imx|mx6|imx6qc398|mx6q)"
#COMPATIBLE_MACHINE = "(imx6qc398)"
