/*
 * fpga_jpeg.h
 *
 *  Created on: Dec 27, 2019
 *      Author: lay
 *
 *  Description:
 *              1. All operations in this module are not thread safety.
 *              2. You should wait current encode to be finished then start the next one.
 *
 */

#ifndef SRC_JPEG_FPGA_JPEG_H_
#define SRC_JPEG_FPGA_JPEG_H_

#include <linux/ioctl.h>

enum quantization_type {
	e_quantization_type_pl = 0,
	e_quantization_type_pc,
	e_quantization_type_sl,
	e_quantization_type_sc
};

enum je_result_status {
	e_je_result_status_succeed = 0,
	e_je_result_status_timeout,
	e_je_result_status_invalid_param,
	e_je_result_status_error
};

enum je_type {
	e_je_type_p = 0,
	e_je_type_s,
};

struct quantization {
	enum quantization_type type;
	unsigned long data[64];
};

struct je_raw_image_addr_rgb {
	unsigned long r;
	unsigned long g;
	unsigned long b;
};

union je_raw_image_addr {
	struct je_raw_image_addr_rgb shade;
	unsigned long pattern;
};

struct je_parameter {
	enum je_type type;
	union je_raw_image_addr raw_image_addr;
	unsigned long jpeg_addr;
};

struct je_result {
	enum je_result_status status;
	unsigned long size;
};

#define IOC_MAGIC					'c'

#define IOC_SET_JPEG_QUANTIZATION	_IOW(IOC_MAGIC, 0, struct quantization *)
#define IOC_DO_JPEG_ENCODE			_IOW(IOC_MAGIC, 1, struct je_parameter *)
#endif /* SRC_JPEG_FPGA_JPEG_H_ */
