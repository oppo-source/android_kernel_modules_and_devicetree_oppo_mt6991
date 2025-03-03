/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _VL53LX_HIST_PRIVATE_STRUCTS_H_
#define _VL53LX_HIST_PRIVATE_STRUCTS_H_

#include "vl53lx_types.h"
#include "vl53lx_hist_structs.h"

#define VL53LX_D_001         8

#ifdef __cplusplus
extern "C" {
#endif




struct VL53LX_hist_gen1_algo_private_data_t {

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_020;

	uint8_t  VL53LX_p_021;

	uint8_t  VL53LX_p_029;

	int32_t  VL53LX_p_016;


	int32_t   VL53LX_p_043[VL53LX_HISTOGRAM_BUFFER_SIZE];
	int32_t   VL53LX_p_068[VL53LX_HISTOGRAM_BUFFER_SIZE];

	uint8_t   VL53LX_p_040[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_018[VL53LX_HISTOGRAM_BUFFER_SIZE];
	uint16_t  VL53LX_p_014[VL53LX_HISTOGRAM_BUFFER_SIZE];
	uint16_t  VL53LX_p_008[VL53LX_HISTOGRAM_BUFFER_SIZE];

};




struct VL53LX_hist_gen2_algo_filtered_data_t {

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_020;

	uint8_t  VL53LX_p_021;

	uint16_t VL53LX_p_015;

	uint8_t  VL53LX_p_005;

	uint8_t  VL53LX_p_029;

	int32_t  VL53LX_p_028;

	int32_t  VL53LX_p_016;


	int32_t   VL53LX_p_007[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_032[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_001[VL53LX_HISTOGRAM_BUFFER_SIZE];


	int32_t   VL53LX_p_018[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_055[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_053[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_054[VL53LX_HISTOGRAM_BUFFER_SIZE];


};




struct VL53LX_hist_gen2_algo_detection_data_t {

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_020;

	uint8_t  VL53LX_p_021;

	int32_t  VL53LX_p_031;


	uint8_t   VL53LX_p_069[VL53LX_HISTOGRAM_BUFFER_SIZE];

	uint8_t   VL53LX_p_070[VL53LX_HISTOGRAM_BUFFER_SIZE];


	uint32_t  VL53LX_p_014[VL53LX_HISTOGRAM_BUFFER_SIZE];

	uint16_t  VL53LX_p_008[VL53LX_HISTOGRAM_BUFFER_SIZE];


	uint8_t   VL53LX_p_040[VL53LX_HISTOGRAM_BUFFER_SIZE];


};




struct VL53LX_hist_pulse_data_t {

	uint8_t  VL53LX_p_012;

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_023;

	uint8_t  VL53LX_p_024;

	uint8_t  VL53LX_p_013;


	uint8_t  VL53LX_p_025;

	uint8_t  VL53LX_p_051;


	int32_t  VL53LX_p_016;

	int32_t  VL53LX_p_017;

	int32_t  VL53LX_p_010;


	uint32_t VL53LX_p_026;

	uint32_t VL53LX_p_011;

	uint32_t VL53LX_p_027;


	uint16_t VL53LX_p_002;


};




struct VL53LX_hist_gen3_algo_private_data_t {

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_020;

	uint8_t  VL53LX_p_021;

	uint8_t  VL53LX_p_030;

	uint8_t  VL53LX_p_039;

	int32_t  VL53LX_p_028;

	int32_t  VL53LX_p_031;


	uint8_t  VL53LX_p_040[VL53LX_HISTOGRAM_BUFFER_SIZE];

	uint8_t  VL53LX_p_041[VL53LX_HISTOGRAM_BUFFER_SIZE];

	uint8_t  VL53LX_p_042[VL53LX_HISTOGRAM_BUFFER_SIZE];


	int32_t  VL53LX_p_052[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t  VL53LX_p_043[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t  VL53LX_p_018[VL53LX_HISTOGRAM_BUFFER_SIZE];


	uint8_t  VL53LX_p_044;

	uint8_t  VL53LX_p_045;

	uint8_t  VL53LX_p_046;


	struct VL53LX_hist_pulse_data_t  VL53LX_p_003[VL53LX_D_001];




	struct VL53LX_histogram_bin_data_t   VL53LX_p_006;

	struct VL53LX_histogram_bin_data_t   VL53LX_p_047;

	struct VL53LX_histogram_bin_data_t   VL53LX_p_048;

	struct VL53LX_histogram_bin_data_t   VL53LX_p_049;

	struct VL53LX_histogram_bin_data_t   VL53LX_p_050;




};




struct VL53LX_hist_gen4_algo_filtered_data_t {

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_020;

	uint8_t  VL53LX_p_021;


	int32_t   VL53LX_p_007[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_032[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_001[VL53LX_HISTOGRAM_BUFFER_SIZE];


	int32_t   VL53LX_p_053[VL53LX_HISTOGRAM_BUFFER_SIZE];

	int32_t   VL53LX_p_054[VL53LX_HISTOGRAM_BUFFER_SIZE];


	uint8_t  VL53LX_p_040[VL53LX_HISTOGRAM_BUFFER_SIZE];


};

#ifdef __cplusplus
}
#endif

#endif

