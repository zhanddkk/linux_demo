/*
 * fpga_peak.h
 *
 *  Created on: Jan 2, 2020
 *      Author: lay
 */

#ifndef SRC_INCLUDE_FPGA_PEAK_H_
#define SRC_INCLUDE_FPGA_PEAK_H_

#include <linux/ioctl.h>

enum PEAK_IMG_TYPE
{
    N_PEAK_IMG_TYPE_CL = 0,
    N_PEAK_IMG_TYPE_ML
};

enum PEAK_RESULT_TYPE
{
    N_PEAK_RESULT_TYPE_SUCCEED = 0,
    N_PEAK_RESULT_TYPE_ERROR
};

struct PEAK_DETECT_PARAM
{
    unsigned int in_addr;
    unsigned int out_addr;
    unsigned int histogram_param;
    enum PEAK_IMG_TYPE type;
};

struct PEAK_DETECT_RESULT
{
    enum PEAK_RESULT_TYPE r_type;
    unsigned int size;
};

union PEAK_BACK_RM_ADDR
{
    unsigned int cl[2];
    unsigned int ml[3];
};

struct PEAK_BACK_RM_PARAM
{
    union PEAK_BACK_RM_ADDR in_addr;
    union PEAK_BACK_RM_ADDR out_addr;
    enum PEAK_IMG_TYPE type;
};

struct PEAK_BACK_RM_RESULT
{
    enum PEAK_RESULT_TYPE r_type;
    unsigned int histogram_param;
};

#define IOC_MAGIC                   'c'

#define IOC_DO_PEAK_DETECT          _IOW(IOC_MAGIC, 0, struct QUANTIZATION *)
#define IOC_DO_PEAK_BACK_RM         _IOW(IOC_MAGIC, 1, struct JPEG_ENCODE_PARAM *)

#endif /* SRC_INCLUDE_FPGA_PEAK_H_ */
