# Copyright (C) 2016 WinSystems, Inc.

require recipes-bsp/u-boot/u-boot.inc

inherit fsl-u-boot-localversion

DESCRIPTION = "Bootloader for WinSystems, Inc. imx6q-c398 platform"
LICENSE = "GPLv2+"
LIC_FILES_CHKSUM = "file://Licenses/gpl-2.0.txt;md5=b234ee4d69f5fce4486a80fdaf4a4263"

DEPENDS_mxs += "elftosb-native openssl-native"

PROVIDES += "u-boot"

SRCBRANCH ??= "master"

SRC_URI = "git://git.freescale.com/imx/uboot-imx.git;protocol=git;branch=${SRCBRANCH} \
	   file://0001-imx6qc398-Add-c398-to-Kconfig.patch \
           file://0002-imx6qc398-Fix-for-max7310-initialization.patch \
	   file://mx6qc398_defconfig \
           file://mx6qc398_spinor_defconfig \
           file://Kconfig \
           file://MAINTAINERS \
           file://Makefile \
           file://mx6q.cfg \
           file://mx6qc398.c \
           file://mx6qc398.h \
"

S = "${WORKDIR}/git"

do_compile_prepend() {
         cp -p ${WORKDIR}/mx6qc398_defconfig ${S}/configs
         cp -p ${WORKDIR}/mx6qc398_spinor_defconfig ${S}/configs
         cp -p ${WORKDIR}/mx6qc398.h ${S}/include/configs
         mkdir -p ${S}/board/freescale/mx6qc398
         cp -p ${WORKDIR}/Kconfig ${S}/board/freescale/mx6qc398
         cp -p ${WORKDIR}/MAINTAINERS ${S}/board/freescale/mx6qc398
         cp -p ${WORKDIR}/Makefile ${S}/board/freescale/mx6qc398
         cp -p ${WORKDIR}/mx6q.cfg ${S}/board/freescale/mx6qc398
         cp -p ${WORKDIR}/mx6qc398.c ${S}/board/freescale/mx6qc398
}

PACKAGE_ARCH = "${MACHINE_ARCH}"
