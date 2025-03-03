// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include "vl53lx_platform_ipp.h"
#include "vl53lx_ll_def.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_hist_funcs.h"
#include "vl53lx_xtalk.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53LX_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53LX_TRACE_MODULE_CORE, status, ##__VA_ARGS__)


VL53LX_Error VL53LX_ipp_hist_process_data(
	struct VL53LX_Dev_t *Dev,
	struct VL53LX_dmax_calibration_data_t    *pdmax_cal,
	struct VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
	struct VL53LX_hist_post_process_config_t *ppost_cfg,
	struct VL53LX_histogram_bin_data_t       *pbins,
	struct VL53LX_xtalk_histogram_data_t     *pxtalk,
	uint8_t                           *pArea1,
	uint8_t                           *pArea2,
	uint8_t                           *phisto_merge_nb,
	struct VL53LX_range_results_t            *presults)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(Dev);

	status =
		VL53LX_hist_process_data(
			pdmax_cal,
			pdmax_cfg,
			ppost_cfg,
			pbins,
			pxtalk,
			pArea1,
			pArea2,
			presults,
			phisto_merge_nb);

	return status;
}

VL53LX_Error VL53LX_ipp_hist_ambient_dmax(
	struct VL53LX_Dev_t *Dev,
	uint16_t                           target_reflectance,
	struct VL53LX_dmax_calibration_data_t    *pdmax_cal,
	struct VL53LX_hist_gen3_dmax_config_t    *pdmax_cfg,
	struct VL53LX_histogram_bin_data_t       *pbins,
	int16_t                           *pambient_dmax_mm)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(Dev);

	status =
		VL53LX_hist_ambient_dmax(
			target_reflectance,
			pdmax_cal,
			pdmax_cfg,
			pbins,
			pambient_dmax_mm);

	return status;
}

VL53LX_Error VL53LX_ipp_xtalk_calibration_process_data(
	struct VL53LX_Dev_t *Dev,
	struct VL53LX_xtalk_range_results_t       *pxtalk_ranges,
	struct VL53LX_xtalk_histogram_data_t      *pxtalk_shape,
	struct VL53LX_xtalk_calibration_results_t *pxtalk_cal)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(Dev);

	status =
		VL53LX_xtalk_calibration_process_data(
			pxtalk_ranges,
			pxtalk_shape,
			pxtalk_cal);

	return status;
}



