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
 
