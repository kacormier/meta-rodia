DESCRIPTION = "RCATSONE: SIM driver"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

inherit module

S = "${WORKDIR}"

PR = "r0"

SRC_URI += "file://Makefile"
SRC_URI += "file://fpga.h"
SRC_URI += "file://circ_buffer.c"
SRC_URI += "file://circ_buffer.h"
SRC_URI += "file://simdrvmain.c"
SRC_URI += "file://simdrv.h"
SRC_URI += "file://fpgasup.c"
SRC_URI += "file://fpgasup.h"
SRC_URI += "file://ampsup.c"
SRC_URI += "file://ampsup.h"
SRC_URI += "file://rcats_msgs.h"
SRC_URI += "file://usbsup.c"
SRC_URI += "file://usbsup.h"

do_install_append() {
    # Copy simdrv.h to usr/include
    install -d ${D}/usr/include
    install -m 0755 ${S}/simdrv.h ${D}/usr/include/
}

