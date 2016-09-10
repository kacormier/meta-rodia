#DESCRIPTION = "RCATSONE: SIM server"

LICENSE = "CLOSED"
LIC_FILES_CHKSUM = ""

# Header files
SRC_URI += "file://fpgaRegisterAccess.h"
SRC_URI += "file://fpga.h"
SRC_URI += "file://fpga_serial.h"
SRC_URI += "file://fpga_ph.h"

# Stubs
SRC_URI += "file://stubs.h"

# Source files
SRC_URI += "file://fpga_ph.cpp"

S = "${WORKDIR}"

PR = "r0"

do_compile() {
         ${CC} -c fpga_ph.cpp -o fpga_ph.o
}

do_install() {
}