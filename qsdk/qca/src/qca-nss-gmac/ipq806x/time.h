/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */

#include <linux/types.h>
#include <linux/time_types.h>

#ifndef _STRUCT_TIMESPEC
#define _STRUCT_TIMESPEC
struct timespec {
	__kernel_old_time_t	tv_sec;		/* seconds */
	long			tv_nsec;	/* nanoseconds */
};
#endif

struct timeval {
	__kernel_old_time_t	tv_sec;		/* seconds */
	__kernel_suseconds_t	tv_usec;	/* microseconds */
};
typedef struct timeval timeval;

struct itimerspec {
	struct timespec it_interval;/* timer period */
	struct timespec it_value;	/* timer expiration */
};

struct itimerval {
	struct timeval it_interval;/* timer interval */
	struct timeval it_value;	/* current value */
};

struct tk_read_base {
	struct clocksource	*clock;
	u64			mask;
	u64			cycle_last;
	u32			mult;
	u32			shift;
	u64			xtime_nsec;
	ktime_t			base;
	u64			base_real;
};

struct timekeeper {
	struct tk_read_base	tkr_mono;
	struct tk_read_base	tkr_raw;
	u64			xtime_sec;
	unsigned long		ktime_sec;
	struct timespec64	wall_to_monotonic;
	ktime_t			offs_real;
	ktime_t			offs_boot;
	ktime_t			offs_tai;
	s32			tai_offset;
	unsigned int		clock_was_set_seq;
	u8			cs_was_changed_seq;
	ktime_t			next_leap_ktime;
	u64			raw_sec;
	struct timespec64	monotonic_to_boot;

	/* The following members are for timekeeping internal use */
	u64			cycle_interval;
	u64			xtime_interval;
	s64			xtime_remainder;
	u64			raw_interval;
	/* The ntp_tick_length() value currently being used.
	 * This cached copy ensures we consistently apply the tick
	 * length for an entire tick, as ntp_tick_length may change
	 * mid-tick, and we don't want to apply that new value to
	 * the tick in progress.
	 */
	u64			ntp_tick;
	/* Difference between accumulated time and NTP time in ntp
	 * shifted nano seconds. */
	s64			ntp_error;
	u32			ntp_error_shift;
	u32			ntp_err_mult;
	/* Flag used to avoid updating NTP twice with same second */
	u32			skip_second_overflow;
#ifdef CONFIG_DEBUG_TIMEKEEPING
	long			last_warning;
	/*
	 * These simple flag variables are managed
	 * without locks, which is racy, but they are
	 * ok since we don't really care about being
	 * super precise about how many events were
	 * seen, just that a problem was observed.
	 */
	int			underflow_seen;
	int			overflow_seen;
#endif
};

#ifdef CONFIG_CLOCKSOURCE_VALIDATE_LAST_CYCLE
static inline u64 clocksource_delta(u64 now, u64 last, u64 mask)
{
	u64 ret = (now - last) & mask;

	/*
	 * Prevent time going backwards by checking the MSB of mask in
	 * the result. If set, return 0.
	 */
	return ret & ~(mask >> 1) ? 0 : ret;
}
#else
static inline u64 clocksource_delta(u64 now, u64 last, u64 mask)
{
	return (now - last) & mask;
}
#endif

struct clocksource {
	u64 (*read)(struct clocksource *cs);
	u64 mask;
	u32 mult;
	u32 shift;
	u64 max_idle_ns;
	u32 maxadj;
#ifdef CONFIG_ARCH_CLOCKSOURCE_DATA
	struct arch_clocksource_data archdata;
#endif
	u64 max_cycles;
	const char *name;
	struct list_head list;
	int rating;
	int (*enable)(struct clocksource *cs);
	void (*disable)(struct clocksource *cs);
	unsigned long flags;
	void (*suspend)(struct clocksource *cs);
	void (*resume)(struct clocksource *cs);
	void (*mark_unstable)(struct clocksource *cs);
	void (*tick_stable)(struct clocksource *cs);

	/* private: */
#ifdef CONFIG_CLOCKSOURCE_WATCHDOG
	/* Watchdog related data, used by the framework */
	struct list_head wd_list;
	u64 cs_last;
	u64 wd_last;
#endif
	struct module *owner;
};

extern void do_gettimeofday(struct timeval *tv);
extern void getnstimeofday64(struct timespec64 *ts);
extern int __getnstimeofday64(struct timespec64 *ts);
extern int of_get_phy_mode_b(struct device_node *np);
extern const void *of_get_mac_address_b(struct device_node *np);
static const void *of_get_mac_address_mtd(struct device_node *np);
static void *of_get_mac_addr(struct device_node *np, const char *name);
static inline u64 timekeeping_get_ns(struct tk_read_base *tkr);
static inline u64 timekeeping_get_delta(struct tk_read_base *tkr);
static inline u64 tk_clock_read(struct tk_read_base *tkr);
static inline u64 timekeeping_delta_to_ns(struct tk_read_base *tkr, u64 delta);
