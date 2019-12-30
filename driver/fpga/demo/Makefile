export KNLNEL_ROOT=/datadisk/lay/sources/fw_prague/kernel/linux-xlnx
export CROSS_COMPILE=arm-linux-
export ARCH=arm

obj-m := gic.o

gic-objs := \
	blink.o \
	cmd.o \
	irq.o

all:
	make -C $(KNLNEL_ROOT) M=$(PWD) modules

clean:
	make -C $(KNLNEL_ROOT) M=$(PWD) clean