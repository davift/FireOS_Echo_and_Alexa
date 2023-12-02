/*
 *  Copyright (C) 2010 Samsung Electronics
 *  MyungJoo Ham <myungjoo.ham@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <linux/delay.h>
#include <dm.h>
#include <ntc_thermistor.h>

#define NTC_CONNECTED_POSITIVE	0
#define NTC_CONNECTED_GROUND	1

struct ntc_compensation {
	int		temp_c;
	unsigned int	ohm;
};
static const struct ntc_compensation comp_table[] = {
	{ .temp_c	= -40, .ohm	= 4397119 },
	{ .temp_c	= -35, .ohm	= 3088599 },
	{ .temp_c	= -30, .ohm	= 2197225 },
	{ .temp_c	= -25, .ohm	= 1581881 },
	{ .temp_c	= -20, .ohm	= 1151037 },
	{ .temp_c	= -15, .ohm	= 846579 },
	{ .temp_c	= -10, .ohm	= 628988 },
	{ .temp_c	= -5, .ohm	= 471632 },
	{ .temp_c	= 0, .ohm	= 357012 },
	{ .temp_c	= 5, .ohm	= 272500 },
	{ .temp_c	= 10, .ohm	= 209710 },
	{ .temp_c	= 15, .ohm	= 162651 },
	{ .temp_c	= 20, .ohm	= 127080 },
	{ .temp_c	= 25, .ohm	= 100000 },
	{ .temp_c	= 30, .ohm	= 79222 },
	{ .temp_c	= 35, .ohm	= 63167 },
	{ .temp_c	= 40, .ohm	= 50677 },
	{ .temp_c	= 45, .ohm	= 40904 },
	{ .temp_c	= 50, .ohm	= 33195 },
	{ .temp_c	= 55, .ohm	= 27091 },
	{ .temp_c	= 60, .ohm	= 22224 },
	{ .temp_c	= 65, .ohm	= 18323 },
	{ .temp_c	= 70, .ohm	= 15184 },
	{ .temp_c	= 75, .ohm	= 12635 },
	{ .temp_c	= 80, .ohm	= 10566 },
	{ .temp_c	= 85, .ohm	= 8873 },
	{ .temp_c	= 90, .ohm	= 7481 },
	{ .temp_c	= 95, .ohm	= 6337 },
	{ .temp_c	= 100, .ohm	= 5384 },
	{ .temp_c	= 105, .ohm	= 4594 },
	{ .temp_c	= 110, .ohm	= 3934 },
	{ .temp_c	= 115, .ohm	= 3380 },
	{ .temp_c	= 120, .ohm	= 2916 },
	{ .temp_c	= 125, .ohm	= 2522 },
};

static void lookup_comp(unsigned int ohm,
			int *i_low, int *i_high)
{
	int start, end, mid;
	int n_comp = sizeof(comp_table) / sizeof(comp_table[0]);

	/*
	 * Handle special cases: Resistance is higher than or equal to
	 * resistance in first table entry, or resistance is lower or equal
	 * to resistance in last table entry.
	 * In these cases, return i_low == i_high, either pointing to the
	 * beginning or to the end of the table depending on the condition.
	 */
	if (ohm >= comp_table[0].ohm) {
		*i_low = 0;
		*i_high = 0;
		return;
	}
	if (ohm <= comp_table[n_comp - 1].ohm) {
		*i_low = n_comp - 1;
		*i_high = n_comp - 1;
		return;
	}

	/* Do a binary search on compensation table */
	start = 0;
	end = n_comp - 1;
	while (start < end) {
		mid = start + (end - start) / 2;
		/*
		 * start <= mid < end
		 * data->comp[start].ohm > ohm >= data->comp[end].ohm
		 *
		 * We could check for "ohm == data->comp[mid].ohm" here, but
		 * that is a quite unlikely condition, and we would have to
		 * check again after updating start. Check it at the end instead
		 * for simplicity.
		 */
		if (ohm >= comp_table[mid].ohm) {
			end = mid;
		} else {
			start = mid + 1;
			/*
			 * ohm >= data->comp[start].ohm might be true here,
			 * since we set start to mid + 1. In that case, we are
			 * done. We could keep going, but the condition is quite
			 * likely to occur, so it is worth checking for it.
			 */
			if (ohm >= comp_table[start].ohm)
				end = start;
		}
		/*
		 * start <= end
		 * data->comp[start].ohm >= ohm >= data->comp[end].ohm
		 */
	}
	/*
	 * start == end
	 * ohm >= data->comp[end].ohm
	 */
	*i_low = end;
	if (ohm == comp_table[end].ohm)
		*i_high = end;
	else
		*i_high = end - 1;
}

static int get_ohm_of_thermistor(unsigned int uv)
{
	u32 puv = 1800000;
	u64 n, puo, pdo;
	puo = 100000;
	pdo = 0;
	int connect = NTC_CONNECTED_GROUND;

	if (uv == 0) {
		return (connect == NTC_CONNECTED_POSITIVE) ?
			INT_MAX : 0;
	}

	if (uv >= puv) {
		return (connect == NTC_CONNECTED_POSITIVE) ?
			0 : INT_MAX;
	}

	if (connect == NTC_CONNECTED_POSITIVE && puo == 0)
		n = (pdo * (puv - uv)) / uv;
	else if (connect == NTC_CONNECTED_GROUND && pdo == 0)
		n = (puo * uv)/(puv - uv);
	else if (connect == NTC_CONNECTED_POSITIVE)
		n = (pdo * puo * (puv - uv))/(puo * uv - pdo * (puv - uv));
	else
		n = (pdo * puo * uv)/(pdo * (puv - uv) - puo * uv);

	if (n > INT_MAX)
		n = INT_MAX;
	return n;
}

static int get_temp_mc(unsigned int ohm)
{
	int low, high;
	int temp;

	lookup_comp(ohm, &low, &high);
	if (low == high) {
		/* Unable to use linear approximation */
		temp = comp_table[low].temp_c * 1000;
	} else {
		temp = comp_table[low].temp_c * 1000 +
			((comp_table[high].temp_c - comp_table[low].temp_c) *
			 1000 * ((int)ohm - (int)comp_table[low].ohm)) /
			((int)comp_table[high].ohm - (int)comp_table[low].ohm);
	}
	return temp;
}

static int ntc_thermistor_get_ohm(unsigned int voltage)
{
	return get_ohm_of_thermistor(voltage);
}

int ntc_read_temp(int *temp, unsigned int voltage)
{
	int ohm;

	/* Convert voltage to uV */
	ohm = ntc_thermistor_get_ohm(voltage*1000);
	if (ohm < 0)
		return ohm;

	*temp = get_temp_mc(ohm);

	return 0;
}