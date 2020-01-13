KNLNEL_ROOT ?= /datadisk/lay/sources/fw_prague/kernel/linux-xlnx
CROSS_COMPILE ?= arm-linux-
ARCH ?= arm
PROJ_PATH ?= $(shell pwd)
COMPILE_TIME = $(shell date +"%Y-%m-%d %H:%M:%S")
OUTPUT_DIR ?= ${PROJ_PATH}/output
INSTALL_MOD_DIR ?= csdental
INSTALL_MOD_PATH = ${OUTPUT_DIR}/fs

export KNLNEL_ROOT CROSS_COMPILE ARCH PROJ_PATH COMPILE_TIME INSTALL_MOD_DIR INSTALL_MOD_PATH

SUB_SRC= jpeg peak
SUB_INSTALL=$(addprefix install-, $(SUB_SRC))
SUB_CLEAN=$(addprefix clean-, $(SUB_SRC))

all: sync_file $(SUB_SRC)
	@echo "build all finished"

install: $(SUB_INSTALL)
	@echo "install finished"

clean: $(SUB_CLEAN)
	@echo "clean all finished"

sync_file:
	@if test -f ${PROJ_PATH}/sync.sh; then ${PROJ_PATH}/sync.sh ${KNLNEL_ROOT}/..; fi
	@echo "sync finished"
	
$(SUB_SRC):
	${MAKE}	-C $@ all

$(SUB_INSTALL):
	${MAKE} -C $(patsubst install-%,%,$@) install

$(SUB_CLEAN):
	${MAKE} -C $(patsubst clean-%,%,$@) clean

.PHONY: all install clean sync_file $(SUB_SRC) $(SUB_INSTALL) $(SUB_CLEAN)
