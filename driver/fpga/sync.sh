#!/bin/bash
SUBS="include jpeg peak Makefile"
DIST_DIR=$1/fpga

for i in ${SUBS}
do
	cp -afv ${PWD}/${i} ${DIST_DIR}/
done
