/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _VL53LX_HIST_ALGOS_GEN3_H_
#define _VL53LX_HIST_ALGOS_GEN3_H_

#include "vl53lx_types.h"
#include "vl53lx_ll_def.h"

#include "vl53lx_hist_private_structs.h"
#include "vl53lx_dmax_private_structs.h"

#ifdef __cplusplus
extern "C"
{
#endif




void VL53LX_f_003(
	struct VL53LX_hist_gen3_algo_private_data_t   *palgo);






VL53LX_Error VL53LX_f_006(
	uint16_t                               ambient_threshold_events_scaler,
	int32_t                                ambient_threshold_sigma,
	int32_t                                min_ambient_threshold_events,
	uint8_t                            algo__crosstalk_compensation_enable,
	struct VL53LX_histogram_bin_data_t           *pbins,
	struct VL53LX_histogram_bin_data_t           *pxtalk,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);






VL53LX_Error VL53LX_f_007(
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);




VL53LX_Error VL53LX_f_008(
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);




VL53LX_Error VL53LX_f_009(
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);




VL53LX_Error VL53LX_f_016(
	VL53LX_HistTargetOrder                target_order,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);




VL53LX_Error VL53LX_f_010(
	uint8_t                                pulse_no,
	struct VL53LX_histogram_bin_data_t           *pbins,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);



VL53LX_Error VL53LX_f_015(
	uint8_t                                pulse_no,
	uint8_t                             clip_events,
	struct VL53LX_histogram_bin_data_t           *pbins,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo);




VL53LX_Error VL53LX_f_020(
	int16_t                            VL53LX_p_019,
	int16_t                            VL53LX_p_024,
	uint8_t                            VL53LX_p_030,
	uint8_t                            clip_events,
	struct VL53LX_histogram_bin_data_t       *pbins,
	uint32_t                          *pphase);




VL53LX_Error VL53LX_f_011(
	uint8_t                                pulse_no,
	struct VL53LX_histogram_bin_data_t           *pbins,
	struct VL53LX_hist_gen3_algo_private_data_t  *palgo,
	int32_t                                pad_value,
	struct VL53LX_histogram_bin_data_t           *ppulse);




VL53LX_Error VL53LX_f_014(
	uint8_t                       bin,
	uint8_t                       sigma_estimator__sigma_ref_mm,
	uint8_t                       VL53LX_p_030,
	uint8_t                       VL53LX_p_051,
	uint8_t                       crosstalk_compensation_enable,
	struct VL53LX_histogram_bin_data_t  *phist_data_ap,
	struct VL53LX_histogram_bin_data_t  *phist_data_zp,
	struct VL53LX_histogram_bin_data_t  *pxtalk_hist,
	uint16_t                     *psigma_est);




void VL53LX_f_017(
	uint8_t                      range_id,
	uint8_t                      valid_phase_low,
	uint8_t                      valid_phase_high,
	uint16_t                     sigma_thres,
	struct VL53LX_histogram_bin_data_t *pbins,
	struct VL53LX_hist_pulse_data_t    *ppulse,
	struct VL53LX_range_data_t         *pdata);


#ifdef __cplusplus
}
#endif

#endif

