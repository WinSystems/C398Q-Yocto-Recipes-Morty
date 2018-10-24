# Copyright (C) 2016 WinSystems, Inc.
# Released under the MIT license (see COPYING.MIT for the terms)

SUMMARY = "Linux Kernel provided and supported by WinSystems, Inc."

require recipes-kernel/linux/linux-imx.inc

DEPENDS += "lzop-native bc-native"

SRCBRANCH = "4.9.11-1.0.0"
LOCALVERSION = "-1.0.0-ga+yocto"
SRCREV = "c5e4c2e3c42f0a1c8931d18850c53bf2745f6e96"

SRC_URI = "git://github.com/WinSystems/linux-imx6.git;branch=${SRCBRANCH} \
	    file://defconfig \
"

DEFAULT_PREFERENCE = "1"


COMPATIBLE_MACHINE = "(imx|mx6|imx6qc398|mx6q)"
