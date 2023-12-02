/* SPDX-License-Identifier: GPL-2.0 */
/*
 *  thermal_core.h
 *
 *  Copyright (C) 2012  Intel Corp
 *  Author: Durgadoss R <durgadoss.r@intel.com>
 */

#ifndef __THERMAL_CORE_H__
#define __THERMAL_CORE_H__

#include <linux/device.h>
#include <linux/thermal.h>
#include <linux/alarmtimer.h>

/* Initial state of a cooling device during binding */
#define THERMAL_NO_TARGET -1UL

/* 10 seconds default power off delay */
#define THERMAL_DEFAULT_POWEROFF_DELAY_MS (10000)

/* Init section thermal table */
extern struct thermal_governor *__governor_thermal_table[];
extern struct thermal_governor *__governor_thermal_table_end[];

#define THERMAL_TABLE_ENTRY(table, name)			\
	static typeof(name) *__thermal_table_entry_##name	\
	__used __section(__##table##_thermal_table) = &name

#define THERMAL_GOVERNOR_DECLARE(name)	THERMAL_TABLE_ENTRY(governor, name)

#define for_each_governor_table(__governor)		\
	for (__governor = __governor_thermal_table;	\
	     __governor < __governor_thermal_table_end;	\
	     __governor++)

#define to_thermal_zone(_dev) \
	container_of(_dev, struct thermal_zone_device, device)

#define to_cooling_device(_dev)	\
	container_of(_dev, struct thermal_cooling_device, device)

int thermal_register_governor(struct thermal_governor *);
void thermal_unregister_governor(struct thermal_governor *);
void thermal_zone_device_rebind_exception(struct thermal_zone_device *,
					  const char *, size_t);
void thermal_zone_device_unbind_exception(struct thermal_zone_device *,
					  const char *, size_t);
int thermal_zone_device_set_policy(struct thermal_zone_device *, char *);
int thermal_build_list_of_policies(char *buf);

/* sysfs I/F */
int thermal_zone_create_device_groups(struct thermal_zone_device *, int);
void thermal_zone_destroy_device_groups(struct thermal_zone_device *);
void thermal_cooling_device_setup_sysfs(struct thermal_cooling_device *);
void thermal_cooling_device_destroy_sysfs(struct thermal_cooling_device *cdev);
/* used only at binding time */
ssize_t trip_point_show(struct device *, struct device_attribute *, char *);
ssize_t weight_show(struct device *, struct device_attribute *, char *);
ssize_t weight_store(struct device *, struct device_attribute *, const char *,
		     size_t);

#ifdef CONFIG_THERMAL_STATISTICS
void thermal_cooling_device_stats_update(struct thermal_cooling_device *cdev,
					 unsigned long new_state);
#else
static inline void
thermal_cooling_device_stats_update(struct thermal_cooling_device *cdev,
				    unsigned long new_state) {}
#endif /* CONFIG_THERMAL_STATISTICS */

/* device tree support */
#ifdef CONFIG_THERMAL_OF
int of_parse_thermal_zones(void);
void of_thermal_destroy_zones(void);
int of_thermal_get_ntrips(struct thermal_zone_device *);
bool of_thermal_is_trip_valid(struct thermal_zone_device *, int);
const struct thermal_trip *
of_thermal_get_trip_points(struct thermal_zone_device *);
#else
static inline int of_parse_thermal_zones(void) { return 0; }
static inline void of_thermal_destroy_zones(void) { }
static inline int of_thermal_get_ntrips(struct thermal_zone_device *tz)
{
	return 0;
}
static inline bool of_thermal_is_trip_valid(struct thermal_zone_device *tz,
					    int trip)
{
	return false;
}
static inline const struct thermal_trip *
of_thermal_get_trip_points(struct thermal_zone_device *tz)
{
	return NULL;
}
#endif

struct thermal_alarm_timer {
	struct alarm timer;
	spinlock_t spinlock;
	struct list_head poll_tz_list;
	int delay_ms;
	bool timer_pending;
	struct list_head node;
};

struct thermal_alarm_poll_tz {
	struct thermal_zone_device *tz;
	struct list_head node;
};

#endif /* __THERMAL_CORE_H__ */
