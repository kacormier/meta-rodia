#DESCRIPTION = "RCATSONE: SIM server"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

# Header files
SRC_URI += "file://fpgaRegisterAccess.h"
SRC_URI += "file://fpga.h"
SRC_URI += "file://fpga_serial.h"
SRC_URI += "file://fpga_ph.h"
SRC_URI += "file://vikdrv.h"
SRC_URI += "file://cres.h"

# Stubs
SRC_URI += "file://stubs.h"

# Source files
SRC_URI += "file://fpga_ph.cpp"
SRC_URI += "file://fpgaRegisterAccess.c"

S = "${WORKDIR}"

PR = "r0"

do_compile() {
         ${CC} -c fpga_ph.cpp -o fpga_ph.o
         ${CC} -c fpgaRegisterAccess.c -o fpgaRegisterAccess.o
}

do_install() {
}