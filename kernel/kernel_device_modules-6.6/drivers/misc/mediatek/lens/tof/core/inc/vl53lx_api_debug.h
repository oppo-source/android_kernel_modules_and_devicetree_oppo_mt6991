/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _VL53LX_API_DEBUG_H_
#define _VL53LX_API_DEBUG_H_

#include "vl53lx_platform.h"
#include "vl53lx_nvm_structs.h"

#ifdef __cplusplus
extern "C" {
#endif





VL53LX_Error VL53LX_decode_calibration_data_buffer(
	uint16_t                   buf_size,
	uint8_t                   *pbuffer,
	struct VL53LX_calibration_data_t *pdata);






VL53LX_Error VL53LX_get_nvm_debug_data(
	struct VL53LX_Dev_t                 *Dev,
	struct VL53LX_decoded_nvm_data_t *pdata);



VL53LX_Error VL53LX_get_histogram_debug_data(
	struct VL53LX_Dev_t                   *Dev,
	struct VL53LX_histogram_bin_data_t *pdata);






VL53LX_Error VL53LX_get_additional_data(
	struct VL53LX_Dev_t                *Dev,
	struct VL53LX_additional_data_t *pdata);






VL53LX_Error VL53LX_get_xtalk_debug_data(
	struct VL53LX_Dev_t                 *Dev,
	struct VL53LX_xtalk_debug_data_t *pdata);




VL53LX_Error VL53LX_get_offset_debug_data(
	struct VL53LX_Dev_t                 *Dev,
	struct VL53LX_offset_debug_data_t *pdata);

#ifdef VL53LX_LOG_ENABLE



void  VL53LX_signed_fixed_point_sprintf(
	int32_t    fp_value,
	uint8_t    frac_bits,
	uint16_t   buf_size,
	char      *pbuffer);




void VL53LX_print_static_nvm_managed(
	struct VL53LX_static_nvm_managed_t   *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);




void VL53LX_print_customer_nvm_managed(
	struct VL53LX_customer_nvm_managed_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);




void VL53LX_print_nvm_copy_data(
	struct VL53LX_nvm_copy_data_t        *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);




void VL53LX_print_histogram_bin_data(
	struct VL53LX_histogram_bin_data_t *pdata,
	char                        *pprefix,
	uint32_t                     trace_flags);




void VL53LX_print_xtalk_histogram_data(
	struct VL53LX_xtalk_histogram_data_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);




void VL53LX_print_xtalk_histogram_shape_data(
	struct VL53LX_xtalk_histogram_shape_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags);




void VL53LX_print_range_results(
	struct VL53LX_range_results_t *pdata,
	char                   *pprefix,
	uint32_t                trace_flags);



void VL53LX_print_range_data(
	struct VL53LX_range_data_t *pdata,
	char                *pprefix,
	uint32_t             trace_flags);




void VL53LX_print_offset_range_results(
	VL53LX_offset_range_results_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);




void VL53LX_print_offset_range_data(
	struct VL53LX_offset_range_data_t *pdata,
	char                       *pprefix,
	uint32_t                    trace_flags);




void VL53LX_print_cal_peak_rate_map(
	struct VL53LX_cal_peak_rate_map_t *pdata,
	char                       *pprefix,
	uint32_t                    trace_flags);




void VL53LX_print_additional_offset_cal_data(
	struct VL53LX_additional_offset_cal_data_t *pdata,
	char                                *pprefix,
	uint32_t                             trace_flags);



void VL53LX_print_additional_data(
	struct VL53LX_additional_data_t *pdata,
	char                     *pprefix,
	uint32_t                 trace_flags);




void VL53LX_print_gain_calibration_data(
	struct VL53LX_gain_calibration_data_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags);




void VL53LX_print_zone_calibration_data(
	struct VL53LX_zone_calibration_data_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags);




void VL53LX_print_zone_calibration_results(
	struct VL53LX_zone_calibration_results_t *pdata,
	char                              *pprefix,
	uint32_t                           trace_flags);




void VL53LX_print_xtalk_range_results(
	struct VL53LX_xtalk_range_results_t *pdata,
	char                         *pprefix,
	uint32_t                      trace_flags);




void VL53LX_print_xtalk_range_data(
	struct VL53LX_xtalk_range_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);




void VL53LX_print_xtalk_calibration_results(
	struct VL53LX_xtalk_calibration_results_t *pdata,
	char                               *pprefix,
	uint32_t                            trace_flags);




void VL53LX_print_xtalk_config(
	struct VL53LX_xtalk_config_t *pdata,
	char                  *pprefix,
	uint32_t               trace_flags);



void VL53LX_print_xtalk_extract_config(
	struct VL53LX_xtalkextract_config_t *pdata,
	char                         *pprefix,
	uint32_t                      trace_flags);



void VL53LX_print_zone_cal_config(
	struct VL53LX_zonecal_config_t *pdata,
	char                    *pprefix,
	uint32_t                 trace_flags);



void VL53LX_print_offset_cal_config(
	struct VL53LX_offsetcal_config_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);




void VL53LX_print_dmax_calibration_data(
	struct VL53LX_dmax_calibration_data_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags);




void VL53LX_print_calibration_data(
	struct VL53LX_calibration_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);




void VL53LX_print_xtalk_debug_data(
	struct VL53LX_xtalk_debug_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);



void VL53LX_print_offset_debug_data(
	struct VL53LX_offset_debug_data_t *pdata,
	char                       *pprefix,
	uint32_t                    trace_flags);




void VL53LX_print_optical_centre(
	struct VL53LX_optical_centre_t   *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);




void VL53LX_print_user_zone(
	struct VL53LX_user_zone_t   *pdata,
	char                 *pprefix,
	uint32_t              trace_flags);



void VL53LX_print_zone_config(
	struct VL53LX_zone_config_t *pdata,
	char                 *pprefix,
	uint32_t              trace_flags);




void VL53LX_print_spad_rate_data(
	struct VL53LX_spad_rate_data_t  *pspad_rates,
	char                     *pprefix,
	uint32_t                  trace_flags);




void VL53LX_print_spad_rate_map(
	struct VL53LX_spad_rate_data_t  *pspad_rates,
	char                     *pprefix,
	uint32_t                  trace_flags);


#endif

#ifdef __cplusplus
}
#endif

#endif

