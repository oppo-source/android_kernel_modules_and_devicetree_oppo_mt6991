/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _VL53LX_HIST_ALGOS_GEN4_H_
#define _VL53LX_HIST_ALGOS_GEN4_H_

#include "vl53lx_types.h"
#include "vl53lx_ll_def.h"

#include "vl53lx_hist_private_structs.h"
#include "vl53lx_dmax_private_structs.h"


#ifdef __cplusplus
extern "C"
{
#endif




VL53LX_Error VL53LX_f_025(
	struct VL53LX_dmax_calibration_data_t         *pdmax_cal,
	struct VL53LX_hist_gen3_dmax_config_t         *pdmax_cfg,
	struct VL53LX_hist_post_process_config_t      *ppost_cfg,
	struct VL53LX_histogram_bin_data_t            *pbins,
	struct VL53LX_histogram_bin_data_t            *pxtalk,
	struct VL53LX_hist_gen3_algo_private_data_t   *palgo,
	struct VL53LX_hist_gen4_algo_filtered_data_t  *pfiltered,
	struct VL53LX_hist_gen3_dmax_private_data_t   *pdmax_algo,
	struct VL53LX_range_results_t                 *presults,
	uint8_t                                histo_merge_nb);




VL53LX_Error VL53LX_f_026(
	uint8_t                                pulse_no,
	struct VL53LX_histogram_bin_data_t           *ppulse,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo,
	struct VL53LX_hist_gen4_algo_filtered_data_t *pfiltered);




VL53LX_Error VL53LX_f_027(
	uint8_t                                pulse_no,
	uint16_t                               noise_threshold,
	struct VL53LX_hist_gen4_algo_filtered_data_t *pfiltered,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);




VL53LX_Error VL53LX_f_028(
	uint8_t   bin,
	int32_t   VL53LX_p_007,
	int32_t   VL53LX_p_032,
	int32_t   VL53LX_p_001,
	int32_t   ax,
	int32_t   bx,
	int32_t   cx,
	int32_t   VL53LX_p_028,
	uint8_t   VL53LX_p_030,
	uint32_t *pmedian_phase);


#ifdef __cplusplus
}
#endif

#endif

