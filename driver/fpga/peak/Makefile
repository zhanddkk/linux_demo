
obj-m := peak.o

peak-objs :=\
	main.o

CFLAGS_main.o := -DBUILD_TIME='"$(COMPILE_TIME)"'
	
all:
	@${MAKE} -C ${KNLNEL_ROOT} M=${PROJ_PATH}/peak modules

install:
	@${MAKE} -C ${KNLNEL_ROOT} M=${PROJ_PATH}/peak modules_install

clean:
	@${MAKE} -C $(KNLNEL_ROOT) M=${PROJ_PATH}/peak clean
	
.PHONY: all clean install
