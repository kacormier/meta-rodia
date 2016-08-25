simdrv-objs := simdrvmain.o circ_buffer.o fpgasup.o ampsup.o
obj-m := simdrv.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) INSTALL_MOD_DIR=kernel/drivers/rcatsone -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
