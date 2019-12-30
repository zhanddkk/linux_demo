
obj-m := jpeg.o

jpeg-objs :=\
	main.o

CFLAGS_main.o := -DBUILD_TIME='"$(COMPILE_TIME)"'
	
all:
	@${MAKE} -C ${KNLNEL_ROOT} M=${PROJ_PATH}/jpeg modules

install:
	@${MAKE} -C ${KNLNEL_ROOT} M=${PROJ_PATH}/jpeg modules_install

clean:
	@${MAKE} -C $(KNLNEL_ROOT) M=${PROJ_PATH}/jpeg clean
	
.PHONY: all clean install
