/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef _VL53LX_CORE_H_
#define _VL53LX_CORE_H_

#include "vl53lx_platform.h"
#include "vl53lx_core_support.h"

#ifdef __cplusplus
extern "C" {
#endif




void VL53LX_init_version(
	struct VL53LX_Dev_t         *Dev);




void VL53LX_init_ll_driver_state(
	struct VL53LX_Dev_t         *Dev,
	VL53LX_DeviceState ll_state);




VL53LX_Error VL53LX_update_ll_driver_rd_state(
	struct VL53LX_Dev_t         *Dev);




VL53LX_Error VL53LX_check_ll_driver_rd_state(
	struct VL53LX_Dev_t         *Dev);




VL53LX_Error VL53LX_update_ll_driver_cfg_state(
	struct VL53LX_Dev_t         *Dev);




void VL53LX_copy_rtn_good_spads_to_buffer(
	struct VL53LX_nvm_copy_data_t  *pdata,
	uint8_t                 *pbuffer);




void VL53LX_init_system_results(
	struct VL53LX_system_results_t      *pdata);




void V53L1_init_zone_results_structure(
	uint8_t                 active_zones,
	struct VL53LX_zone_results_t  *pdata);




void V53L1_init_zone_dss_configs(
	struct VL53LX_Dev_t              *Dev);




void VL53LX_init_histogram_config_structure(
	uint8_t   even_bin0,
	uint8_t   even_bin1,
	uint8_t   even_bin2,
	uint8_t   even_bin3,
	uint8_t   even_bin4,
	uint8_t   even_bin5,
	uint8_t   odd_bin0,
	uint8_t   odd_bin1,
	uint8_t   odd_bin2,
	uint8_t   odd_bin3,
	uint8_t   odd_bin4,
	uint8_t   odd_bin5,
	struct VL53LX_histogram_config_t  *pdata);



void VL53LX_init_histogram_multizone_config_structure(
	uint8_t   even_bin0,
	uint8_t   even_bin1,
	uint8_t   even_bin2,
	uint8_t   even_bin3,
	uint8_t   even_bin4,
	uint8_t   even_bin5,
	uint8_t   odd_bin0,
	uint8_t   odd_bin1,
	uint8_t   odd_bin2,
	uint8_t   odd_bin3,
	uint8_t   odd_bin4,
	uint8_t   odd_bin5,
	struct VL53LX_histogram_config_t  *pdata);




void VL53LX_init_xtalk_bin_data_struct(
	uint32_t                        bin_value,
	uint16_t                        VL53LX_p_021,
	struct VL53LX_xtalk_histogram_shape_t *pdata);




void VL53LX_i2c_encode_uint16_t(
	uint16_t    ip_value,
	uint16_t    count,
	uint8_t    *pbuffer);




uint16_t VL53LX_i2c_decode_uint16_t(
	uint16_t    count,
	uint8_t    *pbuffer);




void VL53LX_i2c_encode_int16_t(
	int16_t     ip_value,
	uint16_t    count,
	uint8_t    *pbuffer);




int16_t VL53LX_i2c_decode_int16_t(
	uint16_t    count,
	uint8_t    *pbuffer);




void VL53LX_i2c_encode_uint32_t(
	uint32_t    ip_value,
	uint16_t    count,
	uint8_t    *pbuffer);




uint32_t VL53LX_i2c_decode_uint32_t(
	uint16_t    count,
	uint8_t    *pbuffer);




uint32_t VL53LX_i2c_decode_with_mask(
	uint16_t    count,
	uint8_t    *pbuffer,
	uint32_t    bit_mask,
	uint32_t    down_shift,
	uint32_t    offset);




void VL53LX_i2c_encode_int32_t(
	int32_t     ip_value,
	uint16_t    count,
	uint8_t    *pbuffer);




int32_t VL53LX_i2c_decode_int32_t(
	uint16_t    count,
	uint8_t    *pbuffer);




VL53LX_Error VL53LX_start_test(
	struct VL53LX_Dev_t     *Dev,
	uint8_t        test_mode__ctrl);




VL53LX_Error VL53LX_set_firmware_enable_register(
	struct VL53LX_Dev_t         *Dev,
	uint8_t            value);




VL53LX_Error VL53LX_enable_firmware(
	struct VL53LX_Dev_t         *Dev);




VL53LX_Error VL53LX_disable_firmware(
	struct VL53LX_Dev_t         *Dev);




VL53LX_Error VL53LX_set_powerforce_register(
	struct VL53LX_Dev_t         *Dev,
	uint8_t            value);





VL53LX_Error VL53LX_enable_powerforce(
	struct VL53LX_Dev_t         *Dev);



VL53LX_Error VL53LX_disable_powerforce(
	struct VL53LX_Dev_t         *Dev);





VL53LX_Error VL53LX_clear_interrupt(
	struct VL53LX_Dev_t         *Dev);





VL53LX_Error VL53LX_force_shadow_stream_count_to_zero(
	struct VL53LX_Dev_t         *Dev);




uint32_t VL53LX_calc_macro_period_us(
	uint16_t fast_osc_frequency,
	uint8_t  VL53LX_p_005);




uint16_t VL53LX_calc_range_ignore_threshold(
	uint32_t central_rate,
	int16_t  x_gradient,
	int16_t  y_gradient,
	uint8_t  rate_mult);




uint32_t VL53LX_calc_timeout_mclks(
	uint32_t  timeout_us,
	uint32_t  macro_period_us);



uint16_t VL53LX_calc_encoded_timeout(
	uint32_t  timeout_us,
	uint32_t  macro_period_us);




uint32_t VL53LX_calc_timeout_us(
	uint32_t  timeout_mclks,
	uint32_t  macro_period_us);



uint32_t VL53LX_calc_decoded_timeout_us(
	uint16_t  timeout_encoded,
	uint32_t  macro_period_us);




uint16_t VL53LX_encode_timeout(
	uint32_t timeout_mclks);




uint32_t VL53LX_decode_timeout(
	uint16_t encoded_timeout);




VL53LX_Error  VL53LX_calc_timeout_register_values(
	uint32_t                 phasecal_config_timeout_us,
	uint32_t                 mm_config_timeout_us,
	uint32_t                 range_config_timeout_us,
	uint16_t                 fast_osc_frequency,
	struct VL53LX_general_config_t *pgeneral,
	struct VL53LX_timing_config_t  *ptiming);




uint8_t VL53LX_encode_vcsel_period(
	uint8_t VL53LX_p_030);




uint32_t VL53LX_decode_unsigned_integer(
	uint8_t  *pbuffer,
	uint8_t   no_of_bytes);




void   VL53LX_encode_unsigned_integer(
	uint32_t  ip_value,
	uint8_t   no_of_bytes,
	uint8_t  *pbuffer);




VL53LX_Error VL53LX_hist_copy_and_scale_ambient_info(
	struct VL53LX_zone_hist_info_t        *pidata,
	struct VL53LX_histogram_bin_data_t    *podata);




void  VL53LX_hist_get_bin_sequence_config(
	struct VL53LX_Dev_t                     *Dev,
	struct VL53LX_histogram_bin_data_t   *pdata);




VL53LX_Error  VL53LX_hist_phase_consistency_check(
	struct VL53LX_Dev_t                   *Dev,
	struct VL53LX_zone_hist_info_t     *phist_prev,
	struct VL53LX_zone_objects_t       *prange_prev,
	struct VL53LX_range_results_t      *prange_curr);







VL53LX_Error  VL53LX_hist_events_consistency_check(
	uint8_t                      event_sigma,
	uint16_t                     min_effective_spad_count,
	struct VL53LX_zone_hist_info_t     *phist_prev,
	struct VL53LX_object_data_t        *prange_prev,
	struct VL53LX_range_data_t         *prange_curr,
	int32_t                     *pevents_tolerance,
	int32_t                     *pevents_delta,
	VL53LX_DeviceError          *prange_status);







VL53LX_Error  VL53LX_hist_merged_pulse_check(
	int16_t                      min_max_tolerance_mm,
	struct VL53LX_range_data_t         *pdata,
	VL53LX_DeviceError          *prange_status);






VL53LX_Error  VL53LX_hist_xmonitor_consistency_check(
	struct VL53LX_Dev_t                   *Dev,
	struct VL53LX_zone_hist_info_t     *phist_prev,
	struct VL53LX_zone_objects_t       *prange_prev,
	struct VL53LX_range_data_t         *prange_curr);






VL53LX_Error  VL53LX_hist_wrap_dmax(
	struct VL53LX_hist_post_process_config_t *phistpostprocess,
	struct VL53LX_histogram_bin_data_t       *pcurrent,
	int16_t                           *pwrap_dmax_mm);




void VL53LX_hist_combine_mm1_mm2_offsets(
	int16_t                              mm1_offset_mm,
	int16_t                              mm2_offset_mm,
	uint8_t                              encoded_mm_roi_centre,
	uint8_t                              encoded_mm_roi_size,
	uint8_t                              encoded_zone_centre,
	uint8_t                              encoded_zone_size,
	struct VL53LX_additional_offset_cal_data_t *pcal_data,
	uint8_t                             *pgood_spads,
	uint16_t                             aperture_attenuation,
	int16_t                             *prange_offset_mm);




VL53LX_Error VL53LX_hist_xtalk_extract_calc_window(
	int16_t                             target_distance_mm,
	uint16_t                            target_width_oversize,
	struct VL53LX_histogram_bin_data_t        *phist_bins,
	struct VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);




VL53LX_Error VL53LX_hist_xtalk_extract_calc_event_sums(
	struct VL53LX_histogram_bin_data_t        *phist_bins,
	struct VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);




VL53LX_Error VL53LX_hist_xtalk_extract_calc_rate_per_spad(
	struct VL53LX_hist_xtalk_extract_data_t   *pxtalk_data);



VL53LX_Error VL53LX_hist_xtalk_extract_calc_shape(
	struct VL53LX_hist_xtalk_extract_data_t  *pxtalk_data,
	struct VL53LX_xtalk_histogram_shape_t    *pxtalk_shape);



VL53LX_Error VL53LX_hist_xtalk_shape_model(
	uint16_t                         events_per_bin,
	uint16_t                         pulse_centre,
	uint16_t                         pulse_width,
	struct VL53LX_xtalk_histogram_shape_t  *pxtalk_shape);




uint16_t VL53LX_hist_xtalk_shape_model_interp(
	uint16_t      events_per_bin,
	uint32_t      phase_delta);




void VL53LX_spad_number_to_byte_bit_index(
	uint8_t  spad_number,
	uint8_t *pbyte_index,
	uint8_t *pbit_index,
	uint8_t *pbit_mask);




void VL53LX_encode_row_col(
	uint8_t  row,
	uint8_t  col,
	uint8_t *pspad_number);




void VL53LX_decode_zone_size(
	uint8_t   encoded_xy_size,
	uint8_t  *pwidth,
	uint8_t  *pheight);




void VL53LX_encode_zone_size(
	uint8_t  width,
	uint8_t  height,
	uint8_t *pencoded_xy_size);




void VL53LX_decode_zone_limits(
	uint8_t   encoded_xy_centre,
	uint8_t   encoded_xy_size,
	int16_t  *px_ll,
	int16_t  *py_ll,
	int16_t  *px_ur,
	int16_t  *py_ur);




uint8_t VL53LX_is_aperture_location(
	uint8_t   row,
	uint8_t   col);




void VL53LX_calc_max_effective_spads(
	uint8_t     encoded_zone_centre,
	uint8_t     encoded_zone_size,
	uint8_t    *pgood_spads,
	uint16_t    aperture_attenuation,
	uint16_t   *pmax_effective_spads);




void VL53LX_calc_mm_effective_spads(
	uint8_t     encoded_mm_roi_centre,
	uint8_t     encoded_mm_roi_size,
	uint8_t     encoded_zone_centre,
	uint8_t     encoded_zone_size,
	uint8_t    *pgood_spads,
	uint16_t    aperture_attenuation,
	uint16_t   *pmm_inner_effective_spads,
	uint16_t   *pmm_outer_effective_spads);




void VL53LX_hist_copy_results_to_sys_and_core(
	struct VL53LX_histogram_bin_data_t      *pbins,
	struct VL53LX_range_results_t           *phist,
	struct VL53LX_system_results_t          *psys,
	struct VL53LX_core_results_t            *pcore);




VL53LX_Error VL53LX_sum_histogram_data(
		struct VL53LX_histogram_bin_data_t *phist_input,
		struct VL53LX_histogram_bin_data_t *phist_output);




VL53LX_Error VL53LX_avg_histogram_data(
		uint8_t no_of_samples,
		struct VL53LX_histogram_bin_data_t *phist_sum,
		struct VL53LX_histogram_bin_data_t *phist_avg);




VL53LX_Error VL53LX_save_cfg_data(
	struct VL53LX_Dev_t  *Dev);




VL53LX_Error VL53LX_dynamic_zone_update(
	struct VL53LX_Dev_t  *Dev,
	struct VL53LX_range_results_t *presults);




VL53LX_Error VL53LX_update_internal_stream_counters(
	struct VL53LX_Dev_t  *Dev,
	uint8_t     external_stream_count,
	uint8_t     *pinternal_stream_count,
	uint8_t     *pinternal_stream_count_val
	);



VL53LX_Error VL53LX_multizone_hist_bins_update(
	struct VL53LX_Dev_t  *Dev);



VL53LX_Error VL53LX_set_histogram_multizone_initial_bin_config(
	struct VL53LX_zone_config_t           *pzone_cfg,
	struct VL53LX_histogram_config_t      *phist_cfg,
	struct VL53LX_histogram_config_t      *pmulti_hist
	);



uint8_t	VL53LX_encode_GPIO_interrupt_config(
	struct VL53LX_GPIO_interrupt_config_t	*pintconf);



struct VL53LX_GPIO_interrupt_config_t VL53LX_decode_GPIO_interrupt_config(
	uint8_t		system__interrupt_config);



VL53LX_Error VL53LX_set_GPIO_distance_threshold(
	struct VL53LX_Dev_t                      *Dev,
	uint16_t			threshold_high,
	uint16_t			threshold_low);



VL53LX_Error VL53LX_set_GPIO_rate_threshold(
	struct VL53LX_Dev_t                      *Dev,
	uint16_t			threshold_high,
	uint16_t			threshold_low);



VL53LX_Error VL53LX_set_GPIO_thresholds_from_struct(
	struct VL53LX_Dev_t                      *Dev,
	struct VL53LX_GPIO_interrupt_config_t *pintconf);





VL53LX_Error VL53LX_set_ref_spad_char_config(
	struct VL53LX_Dev_t    *Dev,
	uint8_t       vcsel_period_a,
	uint32_t      phasecal_timeout_us,
	uint16_t      total_rate_target_mcps,
	uint16_t      max_count_rate_rtn_limit_mcps,
	uint16_t      min_count_rate_rtn_limit_mcps,
	uint16_t      fast_osc_frequency);




VL53LX_Error VL53LX_set_ssc_config(
	struct VL53LX_Dev_t           *Dev,
	struct VL53LX_ssc_config_t *pssc_cfg,
	uint16_t             fast_osc_frequency);




VL53LX_Error VL53LX_get_spad_rate_data(
	struct VL53LX_Dev_t                *Dev,
	struct VL53LX_spad_rate_data_t  *pspad_rates);



uint32_t VL53LX_calc_crosstalk_plane_offset_with_margin(
		uint32_t     plane_offset_kcps,
		int16_t      margin_offset_kcps);



VL53LX_Error VL53LX_low_power_auto_data_init(
	struct VL53LX_Dev_t                     *Dev
	);



VL53LX_Error VL53LX_low_power_auto_data_stop_range(
	struct VL53LX_Dev_t                     *Dev
	);




VL53LX_Error VL53LX_dynamic_xtalk_correction_calc_required_samples(
	struct VL53LX_Dev_t                     *Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
	struct VL53LX_Dev_t *Dev,
	uint32_t xtalk_offset_out,
	struct VL53LX_smudge_corrector_config_t *pconfig,
	struct VL53LX_smudge_corrector_data_t *pout,
	uint8_t add_smudge,
	uint8_t soft_update
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_corrector(
	struct VL53LX_Dev_t                     *Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_data_init(
	struct VL53LX_Dev_t                     *Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_output_init(
	struct VL53LX_LLDriverResults_t *pres
	);



VL53LX_Error VL53LX_xtalk_cal_data_init(
	struct VL53LX_Dev_t                          *Dev
	);



VL53LX_Error VL53LX_config_low_power_auto_mode(
	struct VL53LX_general_config_t   *pgeneral,
	struct VL53LX_dynamic_config_t   *pdynamic,
	struct VL53LX_low_power_auto_data_t *plpadata
	);



VL53LX_Error VL53LX_low_power_auto_setup_manual_calibration(
	struct VL53LX_Dev_t        *Dev);



VL53LX_Error VL53LX_low_power_auto_update_DSS(
	struct VL53LX_Dev_t        *Dev);


VL53LX_Error VL53LX_compute_histo_merge_nb(
	struct VL53LX_Dev_t        *Dev, uint8_t *histo_merge_nb);

#ifdef __cplusplus
}
#endif

#endif

