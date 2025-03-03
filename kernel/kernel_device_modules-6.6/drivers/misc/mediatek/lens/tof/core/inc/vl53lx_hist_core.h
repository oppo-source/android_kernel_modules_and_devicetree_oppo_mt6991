/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _VL53LX_HIST_CORE_H_
#define _VL53LX_HIST_CORE_H_

#include "vl53lx_types.h"
#include "vl53lx_ll_def.h"
#include "vl53lx_hist_private_structs.h"

#ifdef __cplusplus
extern "C"
{
#endif




void  VL53LX_f_022(
	uint8_t                         VL53LX_p_032,
	uint8_t                         filter_woi,
	struct VL53LX_histogram_bin_data_t    *pbins,
	int32_t                        *pa,
	int32_t                        *pb,
	int32_t                        *pc);




VL53LX_Error VL53LX_f_018(
	uint16_t                        vcsel_width,
	uint16_t                        fast_osc_frequency,
	uint32_t                        total_periods_elapsed,
	uint16_t                        VL53LX_p_004,
	struct VL53LX_range_data_t            *pdata,
	uint8_t histo_merge_nb);




void VL53LX_f_019(
	uint16_t             gain_factor,
	int16_t              range_offset_mm,
	struct VL53LX_range_data_t *pdata);




void  VL53LX_f_029(
	struct VL53LX_histogram_bin_data_t   *pdata,
	int32_t                        ambient_estimate_counts_per_bin);




void  VL53LX_f_005(
	struct VL53LX_histogram_bin_data_t   *pxtalk,
	struct VL53LX_histogram_bin_data_t   *pbins,
	struct VL53LX_histogram_bin_data_t   *pxtalk_realigned);



int8_t  VL53LX_f_030(
	struct VL53LX_histogram_bin_data_t   *pdata1,
	struct VL53LX_histogram_bin_data_t   *pdata2);



VL53LX_Error  VL53LX_f_031(
	struct VL53LX_histogram_bin_data_t   *pidata,
	struct VL53LX_histogram_bin_data_t   *podata);

#ifdef __cplusplus
}
#endif

#endif

