/*
********************************************************************************
* Copyright (C) 2021 ams AG                                                    *
*                                                                              *
* This program is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU General Public License as published by the        *
* Free Software Foundation; version 2.                                         *
*                                                                              *
* This program is distributed in the hope that it will be useful, but          *
* WITHOUT ANY WARRANTY; without even the implied warranty of                   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General     *
* Public License for more details.                                             *
*                                                                              *
* You should have received a copy of the GNU General Public License along      *
* with this program; if not, write to the Free Software Foundation, Inc.,      *
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                 *
********************************************************************************
*/


/***** tof8801_app0.h *****/

#ifndef __TOF8801_APP0_H
#define __TOF8801_APP0_H

#include "include/tof8801.h"
#include <linux/time.h>
#include <linux/limits.h>
#include <linux/timer.h>

#define TOF8801_APP0_SET_INT_CMPRES_START OL_CMD_DATA4_OFFSET
#define TOF8801_APP0_CMD_START          OL_CMD_DATA9_OFFSET
#define TOF8801_APP0_IDLE_WAIT_TIMEOUT  200000
#define MEASURE_TIMEOUT_MSEC            3000
#define APP0_FAC_CALIB_MSEC_TIMEOUT     6000
#define TMF8801_MAX_DIST_MM             2625 // max dist is 2.5m + 5%
#define UNINITIALIZED_CLK_TRIM_VAL      (-999)
#define TMF8801_CR_WRAPAROUND           0x100000000L       // 64 bits
#define TMF8801_CR_WRAPAROUND_DIV2      0x80000000L        // 32 bits
// 3 measurements needed to calculate stable ratio
#define TMF8801_MINCOUNT                3
// expect 5 counts TOF per 1 count host (1/ratio)
#define TMF8801_EXPECTEDRATIO           5
#define TOF8801_APP0_OSC_TRIM_CMD       0x29
#define TOF8801_APP0_EXP_FREQ_RATIO_Q15  6972
#define TOF8801_APP0_MAX_FREQ_RATIO_Q15  7212 // lowest Frequency
#define TOF8801_APP0_MIN_FREQ_RATIO_Q15  6732 // highest Frequency
//#define TOF8801_APP0_FREQ_RATIO_HITH_Q15 6132 //25 KHz
//#define TOF8801_APP0_FREQ_RATIO_LOTH_Q15 6079
#define TOF8801_APP0_FREQ_RATIO_HITH_Q15 6991 //12.5 KHz
#define TOF8801_APP0_FREQ_RATIO_LOTH_Q15 6953
#define TOF8801_APP0_DEFAULT_CLK_CNT     100
#define TOF8801_APP0_CLK_CNT_CONSTANT    20000000
#define TOF8801_APP0_CLK_RATIO_AGGR_STEP 100
#define TOF8801_APP0_CLKRATIO_AGGR_LO   (TOF8801_APP0_EXP_FREQ_RATIO_Q15 - \
                                         TOF8801_APP0_CLK_RATIO_AGGR_STEP)
#define TOF8801_APP0_CLKRATIO_AGGR_HI   (TOF8801_APP0_EXP_FREQ_RATIO_Q15 + \
                                         TOF8801_APP0_CLK_RATIO_AGGR_STEP)
#define TOF8801_APP0_AGGR_ADJ_CHK(x)    (((x) > TOF8801_APP0_CLKRATIO_AGGR_HI) || \
                                         ((x) < TOF8801_APP0_CLKRATIO_AGGR_LO))
#define TOF8801_APP0_RATIO_TO_TRIM_CNT(ratio)               \
({                                                          \
    int32_t __cnt = 0;                                      \
    if (ratio <= TOF8801_APP0_FREQ_RATIO_LOTH_Q15) {        \
        __cnt = ratio - TOF8801_APP0_FREQ_RATIO_LOTH_Q15;   \
        __cnt /= 30; /*approx 50 q15 counts per trim step*/ \
        __cnt = min(-1,__cnt);                              \
    } else if (ratio >= TOF8801_APP0_FREQ_RATIO_HITH_Q15) { \
        __cnt = ratio - TOF8801_APP0_FREQ_RATIO_HITH_Q15;   \
        __cnt /= 30; /*approx 50 q15 counts per trim step*/ \
        __cnt = max(1,__cnt);                               \
    }                                                       \
    __cnt;                                                  \
})                                                          \


enum tof8801_app0_irq_flags {
  IRQ_RESULTS = 0x1,
  IRQ_DIAG    = 0x2,
  IRQ_ERROR   = 0x4,
};

#define TOF_HISTOGRAM_SUM           128

enum tof8801_app0_dataset_types {
  RESERVED_DATASET_1  = 0x1,
  RESERVED_DATASET_2  = 0x2,
  GEN_CFG_DATASET     = 0x3,
  AVAIL_SPADS_DATASET = 0x4,
};

/*****************************************************************************
 *
 *
 *  START app0 structures
 *
 *
 * ***************************************************************************
 */

struct tof8801_app0_anon_cmd {
  char buf[TOF8801_APP0_MAX_CMD_SIZE];
}PACKED ;

struct tof8801_app0_calib_cmd {
  char buf[TOF8801_APP0_CMD_NUM_PARAMS - 1];
  tof_core_data_t data_0;
  char cmd;
}PACKED ;

struct tof8801_app0_diag_mask_cmd {
  char buf[TOF8801_APP0_CMD_NUM_PARAMS - 4];
  char diag_0;
  char diag_1;
  char diag_2;
  char diag_3;
  char cmd;
}PACKED ;

union tof8801_app0_command {
  struct tof8801_app0_anon_cmd      anon_cmd;
  struct tof8801_app0_calib_cmd     calib_cmd;
  struct tof8801_app0_diag_mask_cmd diag_mask_cmd;
}PACKED ;

union tof8801_app0_capture_v1 {
  struct {
    char reserved_arr[TOF8801_APP0_MAX_CMD_SIZE - 6];
    char delay;          // 100s of usec
    char noise_thrshld;  // 0xFF for default
    char period;         // (ms) 0 for single capture
    char iterations[2];  // 0 for default, in 1000s of iterations
    char cmd;
  } PACKED;
  char buf[TOF8801_APP0_MAX_CMD_SIZE];
} PACKED;

union tof8801_app0_capture_v2 {
  struct {
    char reserved_arr[TOF8801_APP0_MAX_CMD_SIZE - 9];
    tof_core_data_t data;
    char alg;
    char gpio;
    char delay;          // 100s of usec
    char noise_thrshld;
    char period;         // (ms) 0 for single capture
    char iterations[2];  // 0 for default, in 1000s of iterations
    char cmd;
  } PACKED;
  char buf[TOF8801_APP0_MAX_CMD_SIZE];
} PACKED;

union tof8801_app0_capture {
  struct {
    char reserved_arr[TOF8801_APP0_MAX_CMD_SIZE - 6];
    char delay;          // 100s of usec
    char noise_thrshld;  // 0xFF for default
    char period;         // (ms) 0 for single capture
    char iterations[2];  // 0 for default, in 1000s of iterations
    char cmd;
  } PACKED;
  union tof8801_app0_capture_v1 v1;
  union tof8801_app0_capture_v2 v2;
  char buf[TOF8801_APP0_MAX_CMD_SIZE];
} PACKED;

// struct tof8801_app0_int_cmpres {
//   uint8_t  pers;
//   uint8_t  low_thres[2];
//   uint8_t  high_thres[2];
//   char cmd;
// };

struct tof8801_app0_version {
  char maj_rev;
  char min_rev;
  char patch_rev;
  char build_0_rev;
  char build_1_rev;
};

typedef struct
{
    uint32_t first_host;
    uint32_t last_host;
    uint32_t first_i2c;
    uint32_t last_i2c;
    uint32_t ratioQ15;
    uint32_t count;
    uint32_t trim_count;
} TMF8801_cr;

struct tof8801_alg_state_info {
  tof_core2_alg_state_data_t alg_data;
  char size;
} PACKED;

struct tof8801_factory_calib_data {
  tof_core2_factory_cal_data_t fac_data;
  char size;
} PACKED;

struct tof8801_configuration_data {
  tof_core2_algo_config_data_t cfg_data;
  char size;
} PACKED;

struct tof8801_app0_application {
  char app_id;
  int diag_state_mask;
  int measure_in_prog;
  int last_known_temp;
  unsigned int clk_iterations;
  int clk_trim_enable;
  uint16_t histogram[TOF_HISTOGRAM_SUM];
  tof_core_data_t                           cal_update;
  TMF8801_cr                                clk_cr;
  struct timespec64                         save_ts;
  struct tof8801_app0_version               version;
  union tof8801_app0_capture                cap_settings;
  union tof8801_app0_command                user_cmd;
  struct tof8801_app0_control_reg_frame     ctrl_frame;
  union tof8801_app0_error_frame            error_frame;
  union tof8801_app0_dataset                dataset;
  union tof8801_app0_algo_results_frame     algo_results_frame;
  union tof8801_app0_histogram_frame        hist_frame;
  // struct tof8801_app0_int_cmpres            int_cmpres; //If this is polling not user.
};

/*****************************************************************************
 *
 *
 *  END app0 structures
 *
 *
 * ***************************************************************************
 */

/*
 * ************************************************************************
 *
 *  Function Declarations
 *
 * ***********************************************************************
 */
extern void tof8801_app0_process_irq(void *chip, char int_stat);
extern void tof8801_app0_report_error(void *chip, char driver_err, char device_err);
extern int tof8801_app0_issue_cmd(void *chip);
extern void tof8801_app0_init_app(struct tof8801_app0_application *app0);
extern int tof8801_app0_switch_to_bootloader(void *chip);
extern int tof8801_app0_read_general_configuration(void *tof_chip);
extern void tof8801_app0_measure_timer_expiry_callback(struct timer_list *t);
extern int tof8801_app0_capture(void *tof_chip, int capture);
extern int tof8801_app0_wait_for_idle(void *chip, unsigned long usec_timeout);
extern void tof8801_app0_default_cap_settings(struct tof8801_app0_application *app0_app);
extern int tof8801_app0_is_v2(void *tof_chip);
extern int tof8801_app0_get_dataset(void *tof_chip, int dataset_type);
extern int tof8801_app0_get_version(void * tof_chip, char *str, unsigned int strlen);
extern int tof8801_app0_write_calibration(void *chip);
extern int tof8801_app0_measure_in_progress(void *chip);
extern int tof8801_app0_perform_factory_calibration(void *tof_chip);
extern int tof8801_app0_rw_osc_trim(void *chip, int *trim_parm, int write_op);
extern void tof8801_app0_set_clk_iterations(void *tof_chip,
                                            int capture_iterations);
// extern int tof8801_app0_write_int_cmpres(void *tof_chip);
// extern int tof8801_app0_read_int_cmpres(void *tof_chip);

#endif /* __TOF8801_APP0_H */
