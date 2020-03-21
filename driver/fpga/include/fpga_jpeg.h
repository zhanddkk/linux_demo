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

enum QUANTIZATION_TYPE
{
    N_QUANTIZATION_TYPE_PL = 0,
    N_QUANTIZATION_TYPE_PC,
    N_QUANTIZATION_TYPE_SL,
    N_QUANTIZATION_TYPE_SC
};

enum JPEG_ENCODE_RESULT
{
    N_JPEG_ENCODE_RESULT_SUCCEED = 0,
    N_JPEG_ENCODE_RESULT_TIMEOUT,
    N_JPEG_ENCODE_RESULT_INVALID_PARAM,
    N_JPEG_ENCODE_RESULT_ERROR
};

enum JPEG_ENCODE_TYPE
{
    N_JPEG_ENCODE_TYPE_P = 0,
    N_JPEG_ENCODE_TYPE_S,
};

struct QUANTIZATION
{
    enum QUANTIZATION_TYPE type;
    unsigned int data[64];
};

struct JPEG_ENCODE_RGB_RAW_IMG_ADDR
{
    unsigned int r;
    unsigned int g;
    unsigned int b;
    unsigned int o;
};

union JPEG_ENCODE_RAW_IMG_ADDR
{
    struct JPEG_ENCODE_RGB_RAW_IMG_ADDR shade;
    unsigned int pattern;
};

struct JPEG_ENCODE_PARAM
{
    enum JPEG_ENCODE_TYPE type;
    union JPEG_ENCODE_RAW_IMG_ADDR raw_image_addr;
    unsigned int jpeg_addr;
};

struct JPEG_ENCODE_RESULT_DATA
{
    enum JPEG_ENCODE_RESULT status;
    unsigned int size;
};

#define IOC_MAGIC					'c'

#define IOC_SET_JPEG_QUANTIZATION	_IOW(IOC_MAGIC, 0, struct QUANTIZATION *)
#define IOC_DO_JPEG_ENCODE			_IOW(IOC_MAGIC, 1, struct JPEG_ENCODE_PARAM *)
#endif /* SRC_JPEG_FPGA_JPEG_H_ */
