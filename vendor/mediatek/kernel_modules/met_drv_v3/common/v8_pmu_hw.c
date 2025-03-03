// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/perf_event.h>
#include <linux/version.h>
#if KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE
#include <linux/perf/arm_pmuv3.h> /* Add for Kernel-mainline & Kernel-6.6 */
#endif
#include <asm/cpu.h>
#include <asm/cputype.h>
#include "met_kernel_symbol.h"
#include "cpu_pmu.h"
#include "interface.h"

/*******************************
 *      ARM v8 operations      *
 *******************************/
/*
 * Per-CPU PMCR: config reg
 */
#define ARMV8_PMCR_E		(1 << 0)	/* Enable all counters */
#define ARMV8_PMCR_P		(1 << 1)	/* Reset all counters */
#define ARMV8_PMCR_C		(1 << 2)	/* Cycle counter reset */
#define ARMV8_PMCR_D		(1 << 3)	/* CCNT counts every 64th cpu cycle */
#define ARMV8_PMCR_X		(1 << 4)	/* Export to ETM */
#define ARMV8_PMCR_DP		(1 << 5)	/* Disable CCNT if non-invasive debug */
#define	ARMV8_PMCR_N_SHIFT	11		/* Number of counters supported */
#define	ARMV8_PMCR_N_MASK	0x1f
#define	ARMV8_PMCR_MASK		0x3f		/* Mask for writable bits */

/*
 * PMOVSR: counters overflow flag status reg
 */
#define	ARMV8_OVSR_MASK		0xffffffff	/* Mask for writable bits */
#define	ARMV8_OVERFLOWED_MASK	ARMV8_OVSR_MASK

static inline void armv8_pmu_counter_select(unsigned int idx)
{
	asm volatile ("msr pmselr_el0, %x0"::"r" (idx));
	isb();
}

static inline void armv8_pmu_type_select(unsigned int idx, unsigned int type)
{
	armv8_pmu_counter_select(idx);
	asm volatile ("msr pmxevtyper_el0, %x0"::"r" (type));
}

static inline unsigned int armv8_pmu_read_count(unsigned int idx)
{
	unsigned int value;

	if (idx == 31) {
		asm volatile ("mrs %x0, pmccntr_el0":"=r" (value));
	} else {
		armv8_pmu_counter_select(idx);
		asm volatile ("mrs %x0, pmxevcntr_el0":"=r" (value));
	}
	return value;
}

static inline void armv8_pmu_enable_count(unsigned int idx)
{
	asm volatile ("msr pmcntenset_el0, %x0"::"r" (1 << idx));
}

static inline void armv8_pmu_disable_count(unsigned int idx)
{
	asm volatile ("msr pmcntenclr_el0, %x0"::"r" (1 << idx));
}

static inline void armv8_pmu_enable_intr(unsigned int idx)
{
	asm volatile ("msr pmintenset_el1, %x0"::"r" (1 << idx));
}

static inline void armv8_pmu_disable_intr(unsigned int idx)
{
	asm volatile ("msr pmintenclr_el1, %x0"::"r" (1 << idx));
	isb();
	asm volatile ("msr pmovsclr_el0, %x0"::"r" (1 << idx));
	isb();
}

static inline void armv8_pmu_write_counter(unsigned int idx,
					   unsigned int val, int is_cyc_cnt)
{
	if (is_cyc_cnt) {
		asm volatile ("msr pmccntr_el0, %x0"::"r" (val));
	} else {
		armv8_pmu_counter_select(idx);
		asm volatile ("msr pmxevcntr_el0, %x0"::"r" (val));
	}
}

static inline void armv8_pmu_disable_cyc_intr(void)
{
	armv8_pmu_disable_intr(31);
}

static inline unsigned int armv8_pmu_overflow(void)
{
	unsigned int val;

	asm volatile ("mrs %x0, pmovsclr_el0":"=r" (val));    /* read */
	val &= ARMV8_OVSR_MASK;
	asm volatile ("msr pmovsclr_el0, %x0"::"r" (val));    /* write to clear */

	return val;
}

static inline unsigned int armv8_pmu_control_read(void)
{
	unsigned int val;

	asm volatile ("mrs %x0, pmcr_el0":"=r" (val));
	return val;
}

static inline void armv8_pmu_control_write(u32 val)
{
	val &= ARMV8_PMCR_MASK;
	isb();
	asm volatile ("msr pmcr_el0, %x0"::"r" (val));
}

static void armv8_pmu_hw_reset_all(int generic_counters)
{
	int i;

	armv8_pmu_control_write(ARMV8_PMCR_C | ARMV8_PMCR_P);
	/* generic counter */
	for (i = 0; i < generic_counters; i++) {
		armv8_pmu_disable_intr(i);
		armv8_pmu_disable_count(i);
	}
	/* cycle counter */
	armv8_pmu_disable_intr(31);
	armv8_pmu_disable_count(31);
	armv8_pmu_overflow();	/* clear overflow */
}

/***********************************
 *      MET ARM v8 operations      *
 ***********************************/
static int armv8_pmu_hw_check_event(struct met_pmu *pmu, int idx, int event)
{
	int i;

	/* Check if event is duplicate */
	for (i = 0; i < idx; i++) {
		if (pmu[i].event == event)
			break;
	}
	if (i < idx) {
		/* pr_debug("++++++ found duplicate event 0x%02x i=%d\n", event, i); */
		return -1;
	}

	return 0;
}

static void armv8_pmu_hw_start(struct met_pmu *pmu, int count)
{
	int i;
	int generic = count - 1;

	armv8_pmu_hw_reset_all(generic);
	for (i = 0; i < generic; i++) {
		if (pmu[i].mode == MODE_POLLING) {
			armv8_pmu_type_select(i, pmu[i].event);
			armv8_pmu_enable_count(i);
		}
	}
	if (pmu[count - 1].mode == MODE_POLLING) {	/* cycle counter */
		armv8_pmu_enable_count(31);
	}
	armv8_pmu_control_write(ARMV8_PMCR_E);
}

static void armv8_pmu_hw_stop(int count)
{
	int generic = count - 1;

	armv8_pmu_hw_reset_all(generic);
}

static unsigned int armv8_pmu_hw_polling(struct met_pmu *pmu, int count, unsigned int *pmu_value)
{
	int i, cnt = 0;
	int generic = count - 1;

	for (i = 0; i < generic; i++) {
		if (pmu[i].mode == MODE_POLLING) {
			pmu_value[cnt] = armv8_pmu_read_count(i);
			cnt++;
		}
	}
	if (pmu[count - 1].mode == MODE_POLLING) {
		pmu_value[cnt] = armv8_pmu_read_count(31);
		cnt++;
	}
	armv8_pmu_control_write(ARMV8_PMCR_C | ARMV8_PMCR_P | ARMV8_PMCR_E);

	return cnt;
}

static unsigned long armv8_perf_event_get_evttype(struct perf_event *ev) {

	struct hw_perf_event *hwc;

	hwc = &ev->hw;
	return hwc->config_base & ARMV8_PMU_EVTYPE_EVENT;
}

static struct met_pmu	pmus[NR_CPUS][MXNR_PMU_EVENTS];

struct cpu_pmu_hw armv8_pmu = {
	.name = "armv8_pmu",
	.check_event = armv8_pmu_hw_check_event,
	.start = armv8_pmu_hw_start,
	.stop = armv8_pmu_hw_stop,
	.polling = armv8_pmu_hw_polling,
	.perf_event_get_evttype = armv8_perf_event_get_evttype,
	.pmu_read_clear_overflow_flag = armv8_pmu_overflow,
	.write_counter = armv8_pmu_write_counter,
	.disable_intr = armv8_pmu_disable_intr,
	.disable_cyc_intr = armv8_pmu_disable_cyc_intr,
};

static void set_pmu_event_count(void *info)
{
	unsigned int cpu;

	cpu = smp_processor_id();
#if (KERNEL_VERSION(6, 8, 0) > LINUX_VERSION_CODE)
	armv8_pmu.event_count[cpu] = ((read_sysreg(pmcr_el0) >> ARMV8_PMU_PMCR_N_SHIFT) & ARMV8_PMU_PMCR_N_MASK) + 1;
#else
	armv8_pmu.event_count[cpu] = FIELD_GET(ARMV8_PMU_PMCR_N, read_sysreg(pmcr_el0)) + 1;
#endif
}

void update_pmu_event_count(unsigned int cpu)
{
	smp_call_function_single(cpu, set_pmu_event_count, NULL, 1);
}

static void init_pmus(void)
{
	int	cpu;

	for_each_possible_cpu(cpu) {
		if (cpu<0 || cpu>=NR_CPUS)
			continue;

		update_pmu_event_count(cpu);
	}
}

struct cpu_pmu_hw *cpu_pmu_hw_init(void)
{
	int	cpu;

	init_pmus();
	for (cpu = 0; cpu < NR_CPUS; cpu++)
		armv8_pmu.pmu[cpu] = pmus[cpu];

	return &armv8_pmu;
}
