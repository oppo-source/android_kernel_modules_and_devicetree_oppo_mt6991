#if !defined(_TRACE_LED_FAULT_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_LED_FAULT_H

#include <linux/tracepoint.h>

#undef TRACE_SYSTEM
#define TRACE_SYSTEM led

TRACE_EVENT(led_fault,

    TP_PROTO(long time_s,
            int app_id,
            const char* log_tag,
            const char* event_id,
            const char* log_type,
            int reg_reset_cnt,
            int funcpme_irq_count,
            int wtd_irq_count,
            int fw_ver,
            int effect_id,
            int group_id,
            const char* reg_info),

    TP_ARGS(time_s,
            app_id,
            log_tag,
            event_id,
            log_type,
            reg_reset_cnt,
            funcpme_irq_count,
            wtd_irq_count,
            fw_ver,
            effect_id,
            group_id,
            reg_info),

    TP_STRUCT__entry(
        __field(long, time_s)           /* Timestamp of the exception (seconds), e.g., 1612345678 */
        __field(int, app_id)            /* DCS App ID, e.g., 20120 */
        __string(log_tag, log_tag)      /* DCS Log Tag, e.g., "PSW_BSP_SENSOR" */
        __string(event_id, event_id)    /* DCS Event ID, e.g., "10002" (SENSOR_DEVICE_TYPE) */
        __string(log_type, log_type)    /* Specific error type, e.g., "led_wdt_err" */
        __field(int, reg_reset_cnt)     /* Register reset count, e.g., 1 */
        __field(int, funcpme_irq_count) /* Pattern end interrupt count, e.g., 5 */
        __field(int, wtd_irq_count)     /* Watchdog interrupt count, e.g., 0 */
        __field(int, fw_ver)            /* Firmware version, e.g., 0x1234 */
        __field(int, effect_id)         /* Effect ID, e.g., 0 (EFFECT_ID_OFF) */
        __field(int, group_id)          /* Group ID, e.g., 0 (GROUP_0) */
        __string(reg_info, reg_info)),  /* Key register info string, e.g., "02:01 04:03..." */

    TP_fast_assign(
        __entry->time_s = time_s;
        __entry->app_id = app_id;
        __assign_str(log_tag, log_tag);
        __assign_str(event_id, event_id);
        __assign_str(log_type, log_type);
        __entry->reg_reset_cnt = reg_reset_cnt;
        __entry->funcpme_irq_count = funcpme_irq_count;
        __entry->wtd_irq_count = wtd_irq_count;
        __entry->fw_ver = fw_ver;
        __entry->effect_id = effect_id;
        __entry->group_id = group_id;
        __assign_str(reg_info, reg_info);),

    TP_printk("time_s:%ld app_id:%d log_tag:%s event_id:%s log_type:%s "
          "reg_reset_cnt:%d funcpme_irq_count:%d wtd_irq_count:%d fw_ver:0x%x "
          "effect_id:%d group_id:%d reg_info:%s",
        __entry->time_s, __entry->app_id, __get_str(log_tag), __get_str(event_id), __get_str(log_type),
        __entry->reg_reset_cnt, __entry->funcpme_irq_count, __entry->wtd_irq_count, __entry->fw_ver,
        __entry->effect_id, __entry->group_id,
        __get_str(reg_info))
);

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH ../../../drivers/leds/aw22xxx
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace_led_fault

/* This part must be outside protection */
#include <trace/define_trace.h>

#endif /* _TRACE_LED_FAULT_H */
