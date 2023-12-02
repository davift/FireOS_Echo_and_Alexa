/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2014 Broadcom Corporation.
 * Copyright 2015 Free Electrons.
 */

#ifndef _FB_NAND_H_
#define _FB_NAND_H_

#include <jffs2/load_kernel.h>

/**
 * fastboot_nand_get_part_info() - Lookup NAND partion by name
 *
 * @part_name: Named device to lookup
 * @part_info: Pointer to returned part_info pointer
 * @response: Pointer to fastboot response buffer
 */
int fastboot_nand_get_part_info(char *part_name, struct part_info **part_info,
				char *response);

/**
 * fastboot_nand_flash_write() - Write image to NAND for fastboot
 *
 * @cmd: Named device to write image to
 * @download_buffer: Pointer to image data
 * @download_bytes: Size of image data
 * @response: Pointer to fastboot response buffer
 */
void fastboot_nand_flash_write(const char *cmd, void *download_buffer,
			       u32 download_bytes, char *response);

/**
 * fastboot_nand_flash_erase() - Erase NAND for fastboot
 *
 * @cmd: Named device to erase
 * @response: Pointer to fastboot response buffer
 */
void fastboot_nand_erase(const char *cmd, char *response);

/*
 * Enable NAND tests when building with userdebug
*/

int fb_nand_lookup(const char *partname,
		   struct mtd_info **mtd,
		   struct part_info **part,
		   char *response);

#ifndef VARIANT_USER_BUILD
/*
 * Code for runtime worn bad block test will be built in
 * if enable this config
*/
#define MTK_NAND_WORN_BAD_TEST 1

/* Test to create BBT */
#define MTK_NAND_CREATE_BBT_TEST 1

/* Read BBT test */
#define MTK_NAND_READ_BBT_TEST 1

/* bit err test */
#define MTK_NAND_BIT_ERRS 1
#else
#define MTK_NAND_WORN_BAD_TEST 0
#define MTK_NAND_CREATE_BBT_TEST 0
#define MTK_NAND_READ_BBT_TEST 0
#define MTK_NAND_BIT_ERRS 0
#endif

#endif
