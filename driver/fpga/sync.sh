#!/bin/bash
SUBS="include jpeg Makefile"
DIST_DIR=/home/lay/datadisk/sources/fw_prague/kernel/fpga

for i in ${SUBS}
do
	cp -afv ${PWD}/${i} ${DIST_DIR}/
done
