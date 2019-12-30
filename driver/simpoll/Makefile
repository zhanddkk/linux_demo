
COMPILE_TIME = $(shell date +"%Y-%m-%d %H:%M:%S")
KVERS = $(shell uname -r)
KNLNEL_ROOT = /lib/modules/${KVERS}/build
PROJ_PATH = ${PWD}

obj-m := simpoll.o

simpoll-objs :=\
	main.o

CFLAGS_main.o := -DBUILD_TIME='"$(COMPILE_TIME)"'
	
all:
	make -C ${KNLNEL_ROOT} M=${PROJ_PATH} modules V=1
	
clean:
	make -C $(KNLNEL_ROOT) M=${PROJ_PATH} clean
	
.PHONY: all clean
