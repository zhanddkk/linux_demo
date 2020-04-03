KNLNEL_ROOT ?= /datadisk/user/lay/sources/prague/fw_prague/kernel/linux-xlnx
CROSS_COMPILE ?= arm-linux-
ARCH ?= arm
PROJ_PATH ?= $(shell pwd)
COMPILE_TIME = $(shell date +"%Y-%m-%d %H:%M:%S")
OUTPUT_DIR ?= ${PROJ_PATH}/output
INSTALL_MOD_DIR ?= csdental
INSTALL_MOD_PATH = ${OUTPUT_DIR}/fs
BUILD_PHONY += sync_file

sync_file:
	@if test -f ${PROJ_PATH}/sync.sh; then ${PROJ_PATH}/sync.sh ${KNLNEL_ROOT}/..; fi
	@echo "sync finished"
	
