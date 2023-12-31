/*
 * ntc_thermistor.c - NTC Thermistors
 *
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/math64.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/platform_data/ntc_thermistor.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/thermal.h>

struct ntc_compensation {
	int		temp_c;
	unsigned int	ohm;
};

/* Order matters, ntc_match references the entries by index */
static const struct platform_device_id ntc_thermistor_id[] = {
	{ "ncp15wb473", TYPE_NCPXXWB473 },
	{ "ncp18wb473", TYPE_NCPXXWB473 },
	{ "ncp21wb473", TYPE_NCPXXWB473 },
	{ "ncp03wb473", TYPE_NCPXXWB473 },
	{ "ncp15wl333", TYPE_NCPXXWL333 },
	{ "b57330v2103", TYPE_B57330V2103},
	{ "ncp03wf104", TYPE_NCPXXWF104 },
	{ "ncp15xh103", TYPE_NCPXXXH103 },
	{ "ntcg103jf103ft1", TYPE_NTCG103JF103 },
	{ },
};

/*
 * A compensation table should be sorted by the values of .ohm
 * in descending order.
 * The following compensation tables are from the specification of Murata NTC
 * Thermistors Datasheet
 */
static const struct ntc_compensation ncpXXwb473[] = {
	{ .temp_c	= -40, .ohm	= 1747920 },
	{ .temp_c	= -35, .ohm	= 1245428 },
	{ .temp_c	= -30, .ohm	= 898485 },
	{ .temp_c	= -25, .ohm	= 655802 },
	{ .temp_c	= -20, .ohm	= 483954 },
	{ .temp_c	= -15, .ohm	= 360850 },
	{ .temp_c	= -10, .ohm	= 271697 },
	{ .temp_c	= -5, .ohm	= 206463 },
	{ .temp_c	= 0, .ohm	= 158214 },
	{ .temp_c	= 5, .ohm	= 122259 },
	{ .temp_c	= 10, .ohm	= 95227 },
	{ .temp_c	= 15, .ohm	= 74730 },
	{ .temp_c	= 20, .ohm	= 59065 },
	{ .temp_c	= 25, .ohm	= 47000 },
	{ .temp_c	= 30, .ohm	= 37643 },
	{ .temp_c	= 35, .ohm	= 30334 },
	{ .temp_c	= 40, .ohm	= 24591 },
	{ .temp_c	= 45, .ohm	= 20048 },
	{ .temp_c	= 50, .ohm	= 16433 },
	{ .temp_c	= 55, .ohm	= 13539 },
	{ .temp_c	= 60, .ohm	= 11209 },
	{ .temp_c	= 65, .ohm	= 9328 },
	{ .temp_c	= 70, .ohm	= 7798 },
	{ .temp_c	= 75, .ohm	= 6544 },
	{ .temp_c	= 80, .ohm	= 5518 },
	{ .temp_c	= 85, .ohm	= 4674 },
	{ .temp_c	= 90, .ohm	= 3972 },
	{ .temp_c	= 95, .ohm	= 3388 },
	{ .temp_c	= 100, .ohm	= 2902 },
	{ .temp_c	= 105, .ohm	= 2494 },
	{ .temp_c	= 110, .ohm	= 2150 },
	{ .temp_c	= 115, .ohm	= 1860 },
	{ .temp_c	= 120, .ohm	= 1615 },
	{ .temp_c	= 125, .ohm	= 1406 },
};
static const struct ntc_compensation ncpXXwl333[] = {
	{ .temp_c	= -40, .ohm	= 1610154 },
	{ .temp_c	= -35, .ohm	= 1130850 },
	{ .temp_c	= -30, .ohm	= 802609 },
	{ .temp_c	= -25, .ohm	= 575385 },
	{ .temp_c	= -20, .ohm	= 416464 },
	{ .temp_c	= -15, .ohm	= 304219 },
	{ .temp_c	= -10, .ohm	= 224193 },
	{ .temp_c	= -5, .ohm	= 166623 },
	{ .temp_c	= 0, .ohm	= 124850 },
	{ .temp_c	= 5, .ohm	= 94287 },
	{ .temp_c	= 10, .ohm	= 71747 },
	{ .temp_c	= 15, .ohm	= 54996 },
	{ .temp_c	= 20, .ohm	= 42455 },
	{ .temp_c	= 25, .ohm	= 33000 },
	{ .temp_c	= 30, .ohm	= 25822 },
	{ .temp_c	= 35, .ohm	= 20335 },
	{ .temp_c	= 40, .ohm	= 16115 },
	{ .temp_c	= 45, .ohm	= 12849 },
	{ .temp_c	= 50, .ohm	= 10306 },
	{ .temp_c	= 55, .ohm	= 8314 },
	{ .temp_c	= 60, .ohm	= 6746 },
	{ .temp_c	= 65, .ohm	= 5503 },
	{ .temp_c	= 70, .ohm	= 4513 },
	{ .temp_c	= 75, .ohm	= 3721 },
	{ .temp_c	= 80, .ohm	= 3084 },
	{ .temp_c	= 85, .ohm	= 2569 },
	{ .temp_c	= 90, .ohm	= 2151 },
	{ .temp_c	= 95, .ohm	= 1809 },
	{ .temp_c	= 100, .ohm	= 1529 },
	{ .temp_c	= 105, .ohm	= 1299 },
	{ .temp_c	= 110, .ohm	= 1108 },
	{ .temp_c	= 115, .ohm	= 949 },
	{ .temp_c	= 120, .ohm	= 817 },
	{ .temp_c	= 125, .ohm	= 707 },
};

static const struct ntc_compensation ncpXXwf104[] = {
	{ .temp_c = -40, .ohm   = 4397119 },
	{ .temp_c = -39, .ohm   = 4092873 },
	{ .temp_c = -38, .ohm   = 3811717 },
	{ .temp_c = -37, .ohm   = 3551748 },
	{ .temp_c = -36, .ohm   = 3311235 },
	{ .temp_c = -35, .ohm   = 3088598 },
	{ .temp_c = -34, .ohm   = 2882395 },
	{ .temp_c = -33, .ohm   = 2691309 },
	{ .temp_c = -32, .ohm   = 2514137 },
	{ .temp_c = -31, .ohm   = 2349777 },
	{ .temp_c = -30, .ohm   = 2197225 },
	{ .temp_c = -29, .ohm   = 2055557 },
	{ .temp_c = -28, .ohm   = 1923931 },
	{ .temp_c = -27, .ohm   = 1801573 },
	{ .temp_c = -26, .ohm   = 1687773 },
	{ .temp_c = -25, .ohm   = 1581880 },
	{ .temp_c = -24, .ohm   = 1483099 },
	{ .temp_c = -23, .ohm   = 1391113 },
	{ .temp_c = -22, .ohm   = 1305412 },
	{ .temp_c = -21, .ohm   = 1225530 },
	{ .temp_c = -20, .ohm   = 1151036 },
	{ .temp_c = -19, .ohm   = 1081535 },
	{ .temp_c = -18, .ohm   = 1016661 },
	{ .temp_c = -17, .ohm   = 956079 },
	{ .temp_c = -16, .ohm   = 899480 },
	{ .temp_c = -15, .ohm   = 846578 },
	{ .temp_c = -14, .ohm   = 797111 },
	{ .temp_c = -13, .ohm   = 750834 },
	{ .temp_c = -12, .ohm   = 707523 },
	{ .temp_c = -11, .ohm   = 666972 },
	{ .temp_c = -10, .ohm   = 628988 },
	{ .temp_c = -9, .ohm    = 593342 },
	{ .temp_c = -8, .ohm    = 559930 },
	{ .temp_c = -7, .ohm    = 528601 },
	{ .temp_c = -6, .ohm    = 499212 },
	{ .temp_c = -5, .ohm    = 471632 },
	{ .temp_c = -4, .ohm    = 445771 },
	{ .temp_c = -3, .ohm    = 421479 },
	{ .temp_c = -2, .ohm    = 398652 },
	{ .temp_c = -1, .ohm    = 377192 },
	{ .temp_c = 0, .ohm     = 357011 },
	{ .temp_c = 1, .ohm     = 338005 },
	{ .temp_c = 2, .ohm     = 320121 },
	{ .temp_c = 3, .ohm     = 303286 },
	{ .temp_c = 4, .ohm     = 287433 },
	{ .temp_c = 5, .ohm     = 272499 },
	{ .temp_c = 6, .ohm     = 258426 },
	{ .temp_c = 7, .ohm     = 245159 },
	{ .temp_c = 8, .ohm     = 232649 },
	{ .temp_c = 9, .ohm     = 220847 },
	{ .temp_c = 10, .ohm    = 209709 },
	{ .temp_c = 11, .ohm    = 199196 },
	{ .temp_c = 12, .ohm    = 189268 },
	{ .temp_c = 13, .ohm    = 179889 },
	{ .temp_c = 14, .ohm    = 171027 },
	{ .temp_c = 15, .ohm    = 162650 },
	{ .temp_c = 16, .ohm    = 154726 },
	{ .temp_c = 17, .ohm    = 147232 },
	{ .temp_c = 18, .ohm    = 140142 },
	{ .temp_c = 19, .ohm    = 133432 },
	{ .temp_c = 20, .ohm    = 127080 },
	{ .temp_c = 21, .ohm    = 121065 },
	{ .temp_c = 22, .ohm    = 115368 },
	{ .temp_c = 23, .ohm    = 109969 },
	{ .temp_c = 24, .ohm    = 104852 },
	{ .temp_c = 25, .ohm    = 100000 },
	{ .temp_c = 26, .ohm    = 95398 },
	{ .temp_c = 27, .ohm    = 91032 },
	{ .temp_c = 28, .ohm    = 86889 },
	{ .temp_c = 29, .ohm    = 82956 },
	{ .temp_c = 30, .ohm    = 79221 },
	{ .temp_c = 31, .ohm    = 75675 },
	{ .temp_c = 32, .ohm    = 72306 },
	{ .temp_c = 33, .ohm    = 69104 },
	{ .temp_c = 34, .ohm    = 66060 },
	{ .temp_c = 35, .ohm    = 63167 },
	{ .temp_c = 36, .ohm    = 60415 },
	{ .temp_c = 37, .ohm    = 57796 },
	{ .temp_c = 38, .ohm    = 55305 },
	{ .temp_c = 39, .ohm    = 52934 },
	{ .temp_c = 40, .ohm    = 50676 },
	{ .temp_c = 41, .ohm    = 48528 },
	{ .temp_c = 42, .ohm    = 46482 },
	{ .temp_c = 43, .ohm    = 44532 },
	{ .temp_c = 44, .ohm    = 42674 },
	{ .temp_c = 45, .ohm    = 40903 },
	{ .temp_c = 46, .ohm    = 39213 },
	{ .temp_c = 47, .ohm    = 37601 },
	{ .temp_c = 48, .ohm    = 36062 },
	{ .temp_c = 49, .ohm    = 34595 },
	{ .temp_c = 50, .ohm    = 33194 },
	{ .temp_c = 51, .ohm    = 31859 },
	{ .temp_c = 52, .ohm    = 30583 },
	{ .temp_c = 53, .ohm    = 29366 },
	{ .temp_c = 54, .ohm    = 28202 },
	{ .temp_c = 55, .ohm    = 27090 },
	{ .temp_c = 56, .ohm    = 26028 },
	{ .temp_c = 57, .ohm    = 25012 },
	{ .temp_c = 58, .ohm    = 24041 },
	{ .temp_c = 59, .ohm    = 23112 },
	{ .temp_c = 60, .ohm    = 22224 },
	{ .temp_c = 61, .ohm    = 21374 },
	{ .temp_c = 62, .ohm    = 20560 },
	{ .temp_c = 63, .ohm    = 19782 },
	{ .temp_c = 64, .ohm    = 19036 },
	{ .temp_c = 65, .ohm    = 18322 },
	{ .temp_c = 66, .ohm    = 17640 },
	{ .temp_c = 67, .ohm    = 16986 },
	{ .temp_c = 68, .ohm    = 16360 },
	{ .temp_c = 69, .ohm    = 15759 },
	{ .temp_c = 70, .ohm    = 15184 },
	{ .temp_c = 71, .ohm    = 14631 },
	{ .temp_c = 72, .ohm    = 14100 },
	{ .temp_c = 73, .ohm    = 13591 },
	{ .temp_c = 74, .ohm    = 13103 },
	{ .temp_c = 75, .ohm    = 12635 },
	{ .temp_c = 76, .ohm    = 12187 },
	{ .temp_c = 77, .ohm    = 11756 },
	{ .temp_c = 78, .ohm    = 11343 },
	{ .temp_c = 79, .ohm    = 10946 },
	{ .temp_c = 80, .ohm    = 10565 },
	{ .temp_c = 81, .ohm    = 10199 },
	{ .temp_c = 82, .ohm    = 9847 },
	{ .temp_c = 83, .ohm    = 9509 },
	{ .temp_c = 84, .ohm    = 9184 },
	{ .temp_c = 85, .ohm    = 8872 },
	{ .temp_c = 86, .ohm    = 8572 },
	{ .temp_c = 87, .ohm    = 8283 },
	{ .temp_c = 88, .ohm    = 8005 },
	{ .temp_c = 89, .ohm    = 7738 },
	{ .temp_c = 90, .ohm    = 7481 },
	{ .temp_c = 91, .ohm    = 7234 },
	{ .temp_c = 92, .ohm    = 6997 },
	{ .temp_c = 93, .ohm    = 6768 },
	{ .temp_c = 94, .ohm    = 6548 },
	{ .temp_c = 95, .ohm    = 6336 },
	{ .temp_c = 96, .ohm    = 6131 },
	{ .temp_c = 97, .ohm    = 5934 },
	{ .temp_c = 98, .ohm    = 5743 },
	{ .temp_c = 99, .ohm    = 5560 },
	{ .temp_c = 100, .ohm   = 5383 },
	{ .temp_c = 101, .ohm   = 5214 },
	{ .temp_c = 102, .ohm   = 5050 },
	{ .temp_c = 103, .ohm   = 4893 },
	{ .temp_c = 104, .ohm   = 4740 },
	{ .temp_c = 105, .ohm   = 4594 },
	{ .temp_c = 106, .ohm   = 4452 },
	{ .temp_c = 107, .ohm   = 4316 },
	{ .temp_c = 108, .ohm   = 4184 },
	{ .temp_c = 109, .ohm   = 4057 },
	{ .temp_c = 110, .ohm   = 3934 },
	{ .temp_c = 111, .ohm   = 3815 },
	{ .temp_c = 112, .ohm   = 3701 },
	{ .temp_c = 113, .ohm   = 3590 },
	{ .temp_c = 114, .ohm   = 3483 },
	{ .temp_c = 115, .ohm   = 3380 },
	{ .temp_c = 116, .ohm   = 3281 },
	{ .temp_c = 117, .ohm   = 3185 },
	{ .temp_c = 118, .ohm   = 3092 },
	{ .temp_c = 119, .ohm   = 3003 },
	{ .temp_c = 120, .ohm   = 2916 },
	{ .temp_c = 121, .ohm   = 2832 },
	{ .temp_c = 122, .ohm   = 2750 },
	{ .temp_c = 123, .ohm   = 2672 },
	{ .temp_c = 124, .ohm   = 2595 },
	{ .temp_c = 125, .ohm   = 2522 },
};

static const struct ntc_compensation ncpXXxh103[] = {
	{ .temp_c	= -40, .ohm	= 247565 },
	{ .temp_c	= -35, .ohm	= 181742 },
	{ .temp_c	= -30, .ohm	= 135128 },
	{ .temp_c	= -25, .ohm	= 101678 },
	{ .temp_c	= -20, .ohm	= 77373 },
	{ .temp_c	= -15, .ohm	= 59504 },
	{ .temp_c	= -10, .ohm	= 46222 },
	{ .temp_c	= -5, .ohm	= 36244 },
	{ .temp_c	= 0, .ohm	= 28674 },
	{ .temp_c	= 5, .ohm	= 22878 },
	{ .temp_c	= 10, .ohm	= 18399 },
	{ .temp_c	= 15, .ohm	= 14910 },
	{ .temp_c	= 20, .ohm	= 12169 },
	{ .temp_c	= 25, .ohm	= 10000 },
	{ .temp_c	= 30, .ohm	= 8271 },
	{ .temp_c	= 35, .ohm	= 6883 },
	{ .temp_c	= 40, .ohm	= 5762 },
	{ .temp_c	= 45, .ohm	= 4851 },
	{ .temp_c	= 50, .ohm	= 4105 },
	{ .temp_c	= 55, .ohm	= 3492 },
	{ .temp_c	= 60, .ohm	= 2985 },
	{ .temp_c	= 65, .ohm	= 2563 },
	{ .temp_c	= 70, .ohm	= 2211 },
	{ .temp_c	= 75, .ohm	= 1915 },
	{ .temp_c	= 80, .ohm	= 1666 },
	{ .temp_c	= 85, .ohm	= 1454 },
	{ .temp_c	= 90, .ohm	= 1275 },
	{ .temp_c	= 95, .ohm	= 1121 },
	{ .temp_c	= 100, .ohm	= 990 },
	{ .temp_c	= 105, .ohm	= 876 },
	{ .temp_c	= 110, .ohm	= 779 },
	{ .temp_c	= 115, .ohm	= 694 },
	{ .temp_c	= 120, .ohm	= 620 },
	{ .temp_c	= 125, .ohm	= 556 },
};

/*
 * The following compensation table is from the specification of EPCOS NTC
 * Thermistors Datasheet
 */
static const struct ntc_compensation b57330v2103[] = {
	{ .temp_c	= -40, .ohm	= 190030 },
	{ .temp_c	= -35, .ohm	= 145360 },
	{ .temp_c	= -30, .ohm	= 112060 },
	{ .temp_c	= -25, .ohm	= 87041 },
	{ .temp_c	= -20, .ohm	= 68104 },
	{ .temp_c	= -15, .ohm	= 53665 },
	{ .temp_c	= -10, .ohm	= 42576 },
	{ .temp_c	= -5, .ohm	= 34001 },
	{ .temp_c	= 0, .ohm	= 27326 },
	{ .temp_c	= 5, .ohm	= 22096 },
	{ .temp_c	= 10, .ohm	= 17973 },
	{ .temp_c	= 15, .ohm	= 14703 },
	{ .temp_c	= 20, .ohm	= 12090 },
	{ .temp_c	= 25, .ohm	= 10000 },
	{ .temp_c	= 30, .ohm	= 8311 },
	{ .temp_c	= 35, .ohm	= 6941 },
	{ .temp_c	= 40, .ohm	= 5825 },
	{ .temp_c	= 45, .ohm	= 4911 },
	{ .temp_c	= 50, .ohm	= 4158 },
	{ .temp_c	= 55, .ohm	= 3536 },
	{ .temp_c	= 60, .ohm	= 3019 },
	{ .temp_c	= 65, .ohm	= 2588 },
	{ .temp_c	= 70, .ohm	= 2227 },
	{ .temp_c	= 75, .ohm	= 1924 },
	{ .temp_c	= 80, .ohm	= 1668 },
	{ .temp_c	= 85, .ohm	= 1451 },
	{ .temp_c	= 90, .ohm	= 1266 },
	{ .temp_c	= 95, .ohm	= 1108 },
	{ .temp_c	= 100, .ohm	= 973 },
	{ .temp_c	= 105, .ohm	= 857 },
	{ .temp_c	= 110, .ohm	= 757 },
	{ .temp_c	= 115, .ohm	= 671 },
	{ .temp_c	= 120, .ohm	= 596 },
	{ .temp_c	= 125, .ohm	= 531 },
};

static const struct ntc_compensation ntcg103jf103ft1[] = {
	{ .temp_c = -40, .ohm   = 188500 },
	{ .temp_c = -39, .ohm   = 178600 },
	{ .temp_c = -38, .ohm   = 169200 },
	{ .temp_c = -37, .ohm   = 160400 },
	{ .temp_c = -36, .ohm   = 152100 },
	{ .temp_c = -35, .ohm   = 144300 },
	{ .temp_c = -34, .ohm   = 136900 },
	{ .temp_c = -33, .ohm   = 130000 },
	{ .temp_c = -32, .ohm   = 123400 },
	{ .temp_c = -31, .ohm   = 117200 },
	{ .temp_c = -30, .ohm   = 111300 },
	{ .temp_c = -29, .ohm   = 105800 },
	{ .temp_c = -28, .ohm   = 100600 },
	{ .temp_c = -27, .ohm   = 95640 },
	{ .temp_c = -26, .ohm   = 90970 },
	{ .temp_c = -25, .ohm   = 86560 },
	{ .temp_c = -24, .ohm   = 82380 },
	{ .temp_c = -23, .ohm   = 78430 },
	{ .temp_c = -22, .ohm   = 74690 },
	{ .temp_c = -21, .ohm   = 71140 },
	{ .temp_c = -20, .ohm   = 67790 },
	{ .temp_c = -19, .ohm   = 64610 },
	{ .temp_c = -18, .ohm   = 61600 },
	{ .temp_c = -17, .ohm   = 58740 },
	{ .temp_c = -16, .ohm   = 56030 },
	{ .temp_c = -15, .ohm   = 53460 },
	{ .temp_c = -14, .ohm   = 51030 },
	{ .temp_c = -13, .ohm   = 48710 },
	{ .temp_c = -12, .ohm   = 46520 },
	{ .temp_c = -11, .ohm   = 44430 },
	{ .temp_c = -10, .ohm   = 42450 },
	{ .temp_c = -9, .ohm    = 40570 },
	{ .temp_c = -8, .ohm    = 38780 },
	{ .temp_c = -7, .ohm    = 37080 },
	{ .temp_c = -6, .ohm    = 35460 },
	{ .temp_c = -5, .ohm    = 33930 },
	{ .temp_c = -4, .ohm    = 32460 },
	{ .temp_c = -3, .ohm    = 31070 },
	{ .temp_c = -2, .ohm    = 29750 },
	{ .temp_c = -1, .ohm    = 28490 },
	{ .temp_c = 0, .ohm     = 27280 },
	{ .temp_c = 1, .ohm     = 26140 },
	{ .temp_c = 2, .ohm     = 25050 },
	{ .temp_c = 3, .ohm     = 24010 },
	{ .temp_c = 4, .ohm     = 23020 },
	{ .temp_c = 5, .ohm     = 22070 },
	{ .temp_c = 6, .ohm     = 21170 },
	{ .temp_c = 7, .ohm     = 20310 },
	{ .temp_c = 8, .ohm     = 19490 },
	{ .temp_c = 9, .ohm     = 18710 },
	{ .temp_c = 10, .ohm    = 17960 },
	{ .temp_c = 11, .ohm    = 17250 },
	{ .temp_c = 12, .ohm    = 16570 },
	{ .temp_c = 13, .ohm    = 15910 },
	{ .temp_c = 14, .ohm    = 15290 },
	{ .temp_c = 15, .ohm    = 14700 },
	{ .temp_c = 16, .ohm    = 14130 },
	{ .temp_c = 17, .ohm    = 13590 },
	{ .temp_c = 18, .ohm    = 13070 },
	{ .temp_c = 19, .ohm    = 12570 },
	{ .temp_c = 20, .ohm    = 12090 },
	{ .temp_c = 21, .ohm    = 11640 },
	{ .temp_c = 22, .ohm    = 11200 },
	{ .temp_c = 23, .ohm    = 10780 },
	{ .temp_c = 24, .ohm    = 10380 },
	{ .temp_c = 25, .ohm    = 10000 },
	{ .temp_c = 26, .ohm    = 9633 },
	{ .temp_c = 27, .ohm    = 9282 },
	{ .temp_c = 28, .ohm    = 8945 },
	{ .temp_c = 29, .ohm    = 8622 },
	{ .temp_c = 30, .ohm    = 8312 },
	{ .temp_c = 31, .ohm    = 8015 },
	{ .temp_c = 32, .ohm    = 7730 },
	{ .temp_c = 33, .ohm    = 7456 },
	{ .temp_c = 34, .ohm    = 7194 },
	{ .temp_c = 35, .ohm    = 6942 },
	{ .temp_c = 36, .ohm    = 6700 },
	{ .temp_c = 37, .ohm    = 6468 },
	{ .temp_c = 38, .ohm    = 6245 },
	{ .temp_c = 39, .ohm    = 6031 },
	{ .temp_c = 40, .ohm    = 5826 },
	{ .temp_c = 41, .ohm    = 5628 },
	{ .temp_c = 42, .ohm    = 5438 },
	{ .temp_c = 43, .ohm    = 5255 },
	{ .temp_c = 44, .ohm    = 5080 },
	{ .temp_c = 45, .ohm    = 4911 },
	{ .temp_c = 46, .ohm    = 4749 },
	{ .temp_c = 47, .ohm    = 4592 },
	{ .temp_c = 48, .ohm    = 4442 },
	{ .temp_c = 49, .ohm    = 4297 },
	{ .temp_c = 50, .ohm    = 4158 },
	{ .temp_c = 51, .ohm    = 4024 },
	{ .temp_c = 52, .ohm    = 3895 },
	{ .temp_c = 53, .ohm    = 3771 },
	{ .temp_c = 54, .ohm    = 3651 },
	{ .temp_c = 55, .ohm    = 3536 },
	{ .temp_c = 56, .ohm    = 3425 },
	{ .temp_c = 57, .ohm    = 3318 },
	{ .temp_c = 58, .ohm    = 3215 },
	{ .temp_c = 59, .ohm    = 3115 },
	{ .temp_c = 60, .ohm    = 3019 },
	{ .temp_c = 61, .ohm    = 2927 },
	{ .temp_c = 62, .ohm    = 2837 },
	{ .temp_c = 63, .ohm    = 2751 },
	{ .temp_c = 64, .ohm    = 2668 },
	{ .temp_c = 65, .ohm    = 2588 },
	{ .temp_c = 66, .ohm    = 2511 },
	{ .temp_c = 67, .ohm    = 2436 },
	{ .temp_c = 68, .ohm    = 2364 },
	{ .temp_c = 69, .ohm    = 2295 },
	{ .temp_c = 70, .ohm    = 2227 },
	{ .temp_c = 71, .ohm    = 2163 },
	{ .temp_c = 72, .ohm    = 2100 },
	{ .temp_c = 73, .ohm    = 2039 },
	{ .temp_c = 74, .ohm    = 1981 },
	{ .temp_c = 75, .ohm    = 1924 },
	{ .temp_c = 76, .ohm    = 1869 },
	{ .temp_c = 77, .ohm    = 1817 },
	{ .temp_c = 78, .ohm    = 1765 },
	{ .temp_c = 79, .ohm    = 1716 },
	{ .temp_c = 80, .ohm    = 1668 },
	{ .temp_c = 81, .ohm    = 1622 },
	{ .temp_c = 82, .ohm    = 1577 },
	{ .temp_c = 83, .ohm    = 1534 },
	{ .temp_c = 84, .ohm    = 1492 },
	{ .temp_c = 85, .ohm    = 1451 },
	{ .temp_c = 86, .ohm    = 1412 },
	{ .temp_c = 87, .ohm    = 1374 },
	{ .temp_c = 88, .ohm    = 1337 },
	{ .temp_c = 89, .ohm    = 1302 },
	{ .temp_c = 90, .ohm    = 1267 },
	{ .temp_c = 91, .ohm    = 1234 },
	{ .temp_c = 92, .ohm    = 1201 },
	{ .temp_c = 93, .ohm    = 1170 },
	{ .temp_c = 94, .ohm    = 1139 },
	{ .temp_c = 95, .ohm    = 1110 },
	{ .temp_c = 96, .ohm    = 1081 },
	{ .temp_c = 97, .ohm    = 1054 },
	{ .temp_c = 98, .ohm    = 1027 },
	{ .temp_c = 99, .ohm    = 1001 },
	{ .temp_c = 100, .ohm   = 975 },
	{ .temp_c = 101, .ohm   = 951 },
	{ .temp_c = 102, .ohm   = 927 },
	{ .temp_c = 103, .ohm   = 904 },
	{ .temp_c = 104, .ohm   = 881 },
	{ .temp_c = 105, .ohm   = 860 },
	{ .temp_c = 106, .ohm   = 838 },
	{ .temp_c = 107, .ohm   = 818 },
	{ .temp_c = 108, .ohm   = 798 },
	{ .temp_c = 109, .ohm   = 779 },
	{ .temp_c = 110, .ohm   = 760 },
	{ .temp_c = 111, .ohm   = 742 },
	{ .temp_c = 112, .ohm   = 724 },
	{ .temp_c = 113, .ohm   = 707 },
	{ .temp_c = 114, .ohm   = 690 },
	{ .temp_c = 115, .ohm   = 674 },
	{ .temp_c = 116, .ohm   = 658 },
	{ .temp_c = 117, .ohm   = 643 },
	{ .temp_c = 118, .ohm   = 628 },
	{ .temp_c = 119, .ohm   = 613 },
	{ .temp_c = 120, .ohm   = 599 },
	{ .temp_c = 121, .ohm   = 585 },
	{ .temp_c = 122, .ohm   = 572 },
	{ .temp_c = 123, .ohm   = 559 },
	{ .temp_c = 124, .ohm   = 546 },
	{ .temp_c = 125, .ohm   = 534 },
};

struct ntc_data {
	struct ntc_thermistor_platform_data *pdata;
	const struct ntc_compensation *comp;
	int n_comp;
};

#if defined(CONFIG_OF) && IS_ENABLED(CONFIG_IIO)
static int ntc_adc_iio_read(struct ntc_thermistor_platform_data *pdata)
{
	struct iio_channel *channel = pdata->chan;
	int raw, uv, ret;

	ret = iio_read_channel_raw(channel, &raw);
	if (ret < 0) {
		pr_err("read channel() error: %d\n", ret);
		return ret;
	}

	ret = iio_convert_raw_to_processed(channel, raw, &uv, 1000);
	if (ret < 0) {
		/* Assume 12 bit ADC with vref at pullup_uv */
		uv = (pdata->pullup_uv * (s64)raw) >> 12;
	}

	return uv;
}

static const struct of_device_id ntc_match[] = {
	{ .compatible = "murata,ncp15wb473",
		.data = &ntc_thermistor_id[0] },
	{ .compatible = "murata,ncp18wb473",
		.data = &ntc_thermistor_id[1] },
	{ .compatible = "murata,ncp21wb473",
		.data = &ntc_thermistor_id[2] },
	{ .compatible = "murata,ncp03wb473",
		.data = &ntc_thermistor_id[3] },
	{ .compatible = "murata,ncp15wl333",
		.data = &ntc_thermistor_id[4] },
	{ .compatible = "epcos,b57330v2103",
		.data = &ntc_thermistor_id[5]},
	{ .compatible = "murata,ncp03wf104",
		.data = &ntc_thermistor_id[6] },
	{ .compatible = "murata,ncp15xh103",
		.data = &ntc_thermistor_id[7] },
	{ .compatible = "tdk,ntcg103jf103ft1",
		.data = &ntc_thermistor_id[8] },

	/* Usage of vendor name "ntc" is deprecated */
	{ .compatible = "ntc,ncp15wb473",
		.data = &ntc_thermistor_id[0] },
	{ .compatible = "ntc,ncp18wb473",
		.data = &ntc_thermistor_id[1] },
	{ .compatible = "ntc,ncp21wb473",
		.data = &ntc_thermistor_id[2] },
	{ .compatible = "ntc,ncp03wb473",
		.data = &ntc_thermistor_id[3] },
	{ .compatible = "ntc,ncp15wl333",
		.data = &ntc_thermistor_id[4] },
	{ },
};
MODULE_DEVICE_TABLE(of, ntc_match);

static struct ntc_thermistor_platform_data *
ntc_thermistor_parse_dt(struct device *dev)
{
	struct iio_channel *chan;
	enum iio_chan_type type;
	struct device_node *np = dev->of_node;
	struct ntc_thermistor_platform_data *pdata;
	int ret;

	if (!np)
		return NULL;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	chan = devm_iio_channel_get(dev, NULL);
	if (IS_ERR(chan))
		return ERR_CAST(chan);

	ret = iio_get_channel_type(chan, &type);
	if (ret < 0)
		return ERR_PTR(ret);

	if (type != IIO_VOLTAGE)
		return ERR_PTR(-EINVAL);

	if (of_property_read_u32(np, "pullup-uv", &pdata->pullup_uv))
		return ERR_PTR(-ENODEV);
	if (of_property_read_u32(np, "pullup-ohm", &pdata->pullup_ohm))
		return ERR_PTR(-ENODEV);
	if (of_property_read_u32(np, "pulldown-ohm", &pdata->pulldown_ohm))
		return ERR_PTR(-ENODEV);

	if (of_find_property(np, "connected-positive", NULL))
		pdata->connect = NTC_CONNECTED_POSITIVE;
	else /* status change should be possible if not always on. */
		pdata->connect = NTC_CONNECTED_GROUND;

	pdata->chan = chan;
	pdata->read_uv = ntc_adc_iio_read;

	return pdata;
}
#else
static struct ntc_thermistor_platform_data *
ntc_thermistor_parse_dt(struct device *dev)
{
	return NULL;
}

#define ntc_match	NULL

#endif

static inline u64 div64_u64_safe(u64 dividend, u64 divisor)
{
	if (divisor == 0 && dividend == 0)
		return 0;
	if (divisor == 0)
		return UINT_MAX;
	return div64_u64(dividend, divisor);
}

static int get_ohm_of_thermistor(struct ntc_data *data, unsigned int uv)
{
	struct ntc_thermistor_platform_data *pdata = data->pdata;
	u32 puv = pdata->pullup_uv;
	u64 n, puo, pdo;
	puo = pdata->pullup_ohm;
	pdo = pdata->pulldown_ohm;

	if (uv == 0)
		return (pdata->connect == NTC_CONNECTED_POSITIVE) ?
			INT_MAX : 0;
	if (uv >= puv)
		return (pdata->connect == NTC_CONNECTED_POSITIVE) ?
			0 : INT_MAX;

	if (pdata->connect == NTC_CONNECTED_POSITIVE && puo == 0)
		n = div_u64(pdo * (puv - uv), uv);
	else if (pdata->connect == NTC_CONNECTED_GROUND && pdo == 0)
		n = div_u64(puo * uv, puv - uv);
	else if (pdata->connect == NTC_CONNECTED_POSITIVE)
		n = div64_u64_safe(pdo * puo * (puv - uv),
				puo * uv - pdo * (puv - uv));
	else
		n = div64_u64_safe(pdo * puo * uv, pdo * (puv - uv) - puo * uv);

	if (n > INT_MAX)
		n = INT_MAX;
	return n;
}

static void lookup_comp(struct ntc_data *data, unsigned int ohm,
			int *i_low, int *i_high)
{
	int start, end, mid;

	/*
	 * Handle special cases: Resistance is higher than or equal to
	 * resistance in first table entry, or resistance is lower or equal
	 * to resistance in last table entry.
	 * In these cases, return i_low == i_high, either pointing to the
	 * beginning or to the end of the table depending on the condition.
	 */
	if (ohm >= data->comp[0].ohm) {
		*i_low = 0;
		*i_high = 0;
		return;
	}
	if (ohm <= data->comp[data->n_comp - 1].ohm) {
		*i_low = data->n_comp - 1;
		*i_high = data->n_comp - 1;
		return;
	}

	/* Do a binary search on compensation table */
	start = 0;
	end = data->n_comp;
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
		if (ohm >= data->comp[mid].ohm) {
			end = mid;
		} else {
			start = mid + 1;
			/*
			 * ohm >= data->comp[start].ohm might be true here,
			 * since we set start to mid + 1. In that case, we are
			 * done. We could keep going, but the condition is quite
			 * likely to occur, so it is worth checking for it.
			 */
			if (ohm >= data->comp[start].ohm)
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
	if (ohm == data->comp[end].ohm)
		*i_high = end;
	else
		*i_high = end - 1;
}

static int get_temp_mc(struct ntc_data *data, unsigned int ohm)
{
	int low, high;
	int temp;

	lookup_comp(data, ohm, &low, &high);
	if (low == high) {
		/* Unable to use linear approximation */
		temp = data->comp[low].temp_c * 1000;
	} else {
		temp = data->comp[low].temp_c * 1000 +
			((data->comp[high].temp_c - data->comp[low].temp_c) *
			 1000 * ((int)ohm - (int)data->comp[low].ohm)) /
			((int)data->comp[high].ohm - (int)data->comp[low].ohm);
	}
	return temp;
}

static int ntc_thermistor_get_ohm(struct ntc_data *data)
{
	int read_uv;

	if (data->pdata->read_ohm)
		return data->pdata->read_ohm();

	if (data->pdata->read_uv) {
		read_uv = data->pdata->read_uv(data->pdata);
		if (read_uv < 0)
			return read_uv;
		return get_ohm_of_thermistor(data, read_uv);
	}
	return -EINVAL;
}

static int ntc_read_temp(void *data, int *temp)
{
	int ohm;

	ohm = ntc_thermistor_get_ohm(data);
	if (ohm < 0)
		return ohm;

	*temp = get_temp_mc(data, ohm);

	return 0;
}

static ssize_t ntc_show_type(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "4\n");
}

static ssize_t ntc_show_temp(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ntc_data *data = dev_get_drvdata(dev);
	int ohm;

	ohm = ntc_thermistor_get_ohm(data);
	if (ohm < 0)
		return ohm;

	return sprintf(buf, "%d\n", get_temp_mc(data, ohm));
}

static SENSOR_DEVICE_ATTR(temp1_type, S_IRUGO, ntc_show_type, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, ntc_show_temp, NULL, 0);

static struct attribute *ntc_attrs[] = {
	&sensor_dev_attr_temp1_type.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ntc);

static const struct thermal_zone_of_device_ops ntc_of_thermal_ops = {
	.get_temp = ntc_read_temp,
};

static int ntc_thermistor_probe(struct platform_device *pdev)
{
	struct thermal_zone_device *tz;
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id =
			of_match_device(of_match_ptr(ntc_match), dev);
	const struct platform_device_id *pdev_id;
	struct ntc_thermistor_platform_data *pdata;
	struct device *hwmon_dev;
	struct ntc_data *data;

	pdata = ntc_thermistor_parse_dt(dev);
	if (IS_ERR(pdata))
		return PTR_ERR(pdata);
	else if (pdata == NULL)
		pdata = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "No platform init data supplied.\n");
		return -ENODEV;
	}

	/* Either one of the two is required. */
	if (!pdata->read_uv && !pdata->read_ohm) {
		dev_err(dev,
			"Both read_uv and read_ohm missing. Need either one of the two.\n");
		return -EINVAL;
	}

	if (pdata->read_uv && pdata->read_ohm) {
		dev_warn(dev,
			 "Only one of read_uv and read_ohm is needed; ignoring read_uv.\n");
		pdata->read_uv = NULL;
	}

	if (pdata->read_uv && (pdata->pullup_uv == 0 ||
				(pdata->pullup_ohm == 0 && pdata->connect ==
				 NTC_CONNECTED_GROUND) ||
				(pdata->pulldown_ohm == 0 && pdata->connect ==
				 NTC_CONNECTED_POSITIVE) ||
				(pdata->connect != NTC_CONNECTED_POSITIVE &&
				 pdata->connect != NTC_CONNECTED_GROUND))) {
		dev_err(dev, "Required data to use read_uv not supplied.\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(struct ntc_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	pdev_id = of_id ? of_id->data : platform_get_device_id(pdev);

	data->pdata = pdata;

	switch (pdev_id->driver_data) {
	case TYPE_NCPXXWB473:
		data->comp = ncpXXwb473;
		data->n_comp = ARRAY_SIZE(ncpXXwb473);
		break;
	case TYPE_NCPXXWL333:
		data->comp = ncpXXwl333;
		data->n_comp = ARRAY_SIZE(ncpXXwl333);
		break;
	case TYPE_B57330V2103:
		data->comp = b57330v2103;
		data->n_comp = ARRAY_SIZE(b57330v2103);
		break;
	case TYPE_NCPXXWF104:
		data->comp = ncpXXwf104;
		data->n_comp = ARRAY_SIZE(ncpXXwf104);
		break;
	case TYPE_NCPXXXH103:
		data->comp = ncpXXxh103;
		data->n_comp = ARRAY_SIZE(ncpXXxh103);
		break;
	case TYPE_NTCG103JF103:
		data->comp = ntcg103jf103ft1;
		data->n_comp = ARRAY_SIZE(ntcg103jf103ft1);
		break;
	default:
		dev_err(dev, "Unknown device type: %lu(%s)\n",
				pdev_id->driver_data, pdev_id->name);
		return -EINVAL;
	}

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, pdev_id->name,
							   data, ntc_groups);
	if (IS_ERR(hwmon_dev)) {
		dev_err(dev, "unable to register as hwmon device.\n");
		return PTR_ERR(hwmon_dev);
	}

	dev_info(dev, "Thermistor type: %s successfully probed.\n",
		 pdev_id->name);

	tz = devm_thermal_zone_of_sensor_register(dev, 0, data,
						  &ntc_of_thermal_ops);
	if (IS_ERR(tz))
		dev_dbg(dev, "Failed to register to thermal fw.\n");

	return 0;
}

static struct platform_driver ntc_thermistor_driver = {
	.driver = {
		.name = "ntc-thermistor",
		.of_match_table = of_match_ptr(ntc_match),
	},
	.probe = ntc_thermistor_probe,
	.id_table = ntc_thermistor_id,
};

module_platform_driver(ntc_thermistor_driver);

MODULE_DESCRIPTION("NTC Thermistor Driver");
MODULE_AUTHOR("MyungJoo Ham <myungjoo.ham@samsung.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ntc-thermistor");
