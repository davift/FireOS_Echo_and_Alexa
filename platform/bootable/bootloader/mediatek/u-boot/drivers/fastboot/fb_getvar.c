// SPDX-License-Identifier: BSD-2-Clause
/*
 * Copyright (C) 2016 The Android Open Source Project
 */

#include <common.h>
#include <fastboot.h>
#include <fastboot-internal.h>
#include <fb_mmc.h>
#include <fb_nand.h>
#include <fs.h>
#include <version.h>

#include <mmc.h>
#include <amz_idme_features.h>
#include <amz_onetime_unlock.h>
#include "secure_boot.h"

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
#include <linux/mtd/mtd.h>
#endif

static void getvar_cpu_rev(char *var_parameter, char *response);
static void getvar_production(char *var_parameter, char *response);
static void getvar_revision(char *var_parameter, char *response);
static void getvar_secure(char *var_parameter, char *response);
static void getvar_slot_count(char *var_parameter, char *response);
static void getvar_version(char *var_parameter, char *response);
static void getvar_bootloader_version(char *var_parameter, char *response);
static void getvar_downloadsize(char *var_parameter, char *response);
static void getvar_serialno(char *var_parameter, char *response);
static void getvar_version_baseband(char *var_parameter, char *response);
static void getvar_antirback_lk_version(char *var_parameter, char *response);
static void getvar_antirback_tee_version(char *var_parameter, char *response);
static void getvar_antirback_uboot_version(char *var_parameter, char *response);
static void getvar_product(char *var_parameter, char *response);
static void getvar_current_slot(char *var_parameter, char *response);
static void getvar_slot_suffixes(char *var_parameter, char *response);
static void getvar_has_slot(char *var_parameter, char *response);
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
static void getvar_emmc_mid(char *var_parameter, char *response);
static void getvar_emmc_cid(char *var_parameter, char *response);
static void getvar_emmc_size(char *var_parameter, char *response);
static void getvar_partition_type(char *part_name, char *response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_partition_size(char *part_name, char *response);
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
static void getvar_nand_page_size(char *var_parameter, char *response);
static void getvar_nand_erase_size(char *var_parameter, char *response);
#endif
static void getvar_unlock_code(char *var_parameter, char *response);
static void getvar_unlock_status(char *var_parameter, char *response);
static void getvar_otu_code(char *var_parameter, char *response);
static void getvar_otu_status(char *var_parameter, char *response);

static const struct {
	const char *variable;
	void (*dispatch)(char *var_parameter, char *response);
} getvar_dispatch[] = {
	{
		.variable = "cpurev",
		.dispatch = getvar_cpu_rev
	}, {
		.variable = "device_sn",
		.dispatch = getvar_serialno
	}, {
		.variable = "fastboot_locked",
		.dispatch = getvar_production
	}, {
		.variable = "production",
		.dispatch = getvar_production
	}, {
		.variable = "revision",
		.dispatch = getvar_revision
	}, {
		.variable = "secure",
		.dispatch = getvar_secure
	}, {
		.variable = "slot-count",
		.dispatch = getvar_slot_count
	}, {
		.variable = "version",
		.dispatch = getvar_version
	}, {
		.variable = "bootloader-version",
		.dispatch = getvar_bootloader_version
	}, {
		.variable = "version-bootloader",
		.dispatch = getvar_bootloader_version
	}, {
		.variable = "downloadsize",
		.dispatch = getvar_downloadsize
	}, {
		.variable = "max-download-size",
		.dispatch = getvar_downloadsize
	}, {
		.variable = "serialno",
		.dispatch = getvar_serialno
	}, {
		.variable = "version-baseband",
		.dispatch = getvar_version_baseband
	}, {
		.variable = "antirback_lk_version",
		.dispatch = getvar_antirback_lk_version
	}, {
		.variable = "antirback_tee_version",
		.dispatch = getvar_antirback_tee_version
	}, {
		.variable = "antirback_uboot_version",
		.dispatch = getvar_antirback_uboot_version
	}, {
		.variable = "product",
		.dispatch = getvar_product
	}, {
		.variable = "current-slot",
		.dispatch = getvar_current_slot
	}, {
		.variable = "slot-suffixes",
		.dispatch = getvar_slot_suffixes
	}, {
		.variable = "has_slot",
		.dispatch = getvar_has_slot
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	}, {
		.variable = "emmc_mid",
		.dispatch = getvar_emmc_mid
	}, {
		.variable = "emmc_cid",
		.dispatch = getvar_emmc_cid
	}, {
		.variable = "emmc_size",
		.dispatch = getvar_emmc_size
	}, {
		.variable = "partition-type",
		.dispatch = getvar_partition_type
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
	}, {
		.variable = "partition-size",
		.dispatch = getvar_partition_size
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
	}, {
		.variable = "nand_page_size",
		.dispatch = getvar_nand_page_size
	}, {
		.variable = "nand_erase_size",
		.dispatch = getvar_nand_erase_size
#endif
	}, {
		.variable = "unlock_code",
		.dispatch = getvar_unlock_code
	}, {
		.variable = "unlock_status",
		.dispatch = getvar_unlock_status
	}, {
		.variable = "otu_code",
		.dispatch = getvar_otu_code
	}, {
		.variable = "otu_status",
		.dispatch = getvar_otu_status
	}
};

static void getvar_cpu_rev(char *var_parameter, char *response)
{
	fastboot_okay(CONFIG_SYS_CPU, response);
}

static void getvar_production(char *var_parameter, char *response)
{
	if (get_locked_production_state())
		fastboot_okay("yes", response);
	else
		fastboot_okay("no", response);
}

static void getvar_revision(char *var_parameter, char *response)
{
	const char *tmp = env_get("loadcfg");

	if (tmp)
		fastboot_okay(tmp, response);
	else
		fastboot_fail("Value not set", response);
}

static void getvar_secure(char *var_parameter, char *response)
{
	if (is_dm_verity_enabled())
		fastboot_okay("yes", response);
	else
		fastboot_okay("no", response);
}

static void getvar_slot_count(char *var_parameter, char *response)
{
	fastboot_okay("2", response);
}

static void getvar_version(char *var_parameter, char *response)
{
	fastboot_okay(FASTBOOT_VERSION, response);
}

static void getvar_bootloader_version(char *var_parameter, char *response)
{
	fastboot_okay(U_BOOT_VERSION, response);
}

static void getvar_downloadsize(char *var_parameter, char *response)
{
	fastboot_response("OKAY", response, "0x%08x", fastboot_buf_size);
}

static void getvar_serialno(char *var_parameter, char *response)
{
	const char *tmp = env_get("serial#");

	if (tmp)
		fastboot_okay(tmp, response);
	else
		fastboot_fail("Value not set", response);
}

static void getvar_unlock_code(char *var_parameter, char *response)
{
	unsigned char unlock_code[UNLOCK_CODE_LEN];
	unsigned int len = sizeof(unlock_code);

	if (!amzn_get_unlock_code(unlock_code, &len))
		fastboot_okay(unlock_code, response);
	else
		fastboot_fail("Error in reading unlock_code", response);
}

static void getvar_unlock_status(char *var_parameter, char *response)
{
	if (amzn_target_is_unlocked()) {
		fastboot_okay("true", response);
	} else {
		fastboot_okay("false", response);
	}
}

static void getvar_otu_code(char *var_parameter, char *response)
{
	unsigned char otu_unlock_code[ONETIME_UNLOCK_CODE_LEN + 1];
	unsigned int len = sizeof(otu_unlock_code);

	if (!amzn_get_one_tu_code(otu_unlock_code, &len))
		fastboot_okay(otu_unlock_code, response);
	else
		fastboot_fail("Error in reading otu_code", response);
}

static void getvar_otu_status(char *var_parameter, char *response)
{
	if (amzn_target_is_onetime_unlocked()) {
		fastboot_okay("true", response);
	} else {
		fastboot_okay("false", response);
	}
}

static void getvar_version_baseband(char *var_parameter, char *response)
{
	fastboot_okay("N/A", response);
}

static void getvar_antirback_lk_version(char *var_parameter, char *response)
{
	if (is_production_device()) {
		char v[16];
		int rc = 0;
		rc = snprintf(v, sizeof(v), "%d", get_antirback_lk_version_from_boot_args());
		if (rc < 0) {
			fastboot_fail("Unable to parse anti-rollback version.", response);
			return;
		}
		fastboot_okay(v, response);
	} else {
		fastboot_fail("N/A, not production device", response);
	}
}

static void getvar_antirback_tee_version(char *var_parameter, char *response)
{
	if (is_production_device()) {
		char v[16];
		int rc = 0;
		rc = snprintf(v, sizeof(v), "%d", get_antirback_tee_version_from_boot_args());
		if (rc < 0) {
			fastboot_fail("Unable to parse anti-rollback version.", response);
			return;
		}
		fastboot_okay(v, response);
	} else {
		fastboot_fail("N/A, not production device", response);
	}
}

static void getvar_antirback_uboot_version(char *var_parameter, char *response)
{
	if (is_production_device()) {
		char v[16];
		int rc = 0;
		rc = snprintf(v, sizeof(v), "%d", get_antirback_uboot_version_from_boot_args());
		if (rc < 0) {
			fastboot_fail("Unable to parse anti-rollback version.", response);
			return;
		}
		fastboot_okay(v, response);
	} else {
		fastboot_fail("N/A, not production device", response);
	}
}

static void getvar_product(char *var_parameter, char *response)
{
	const char *prod = env_get("product");

	if (prod)
		fastboot_okay(prod, response);
	else
		fastboot_fail("product is not set", response);
}

static void getvar_current_slot(char *var_parameter, char *response)
{

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	int active_slot = ufbl_get_active_slot();

	if (active_slot == 0)
		fastboot_okay("_a", response);
	else if (active_slot == 1)
		fastboot_okay("_b", response);
	else
		fastboot_fail("Active slot out of range", response);
#else
    /* UFBL is not enabled for NAND devices, for now always return _a */
	fastboot_okay("_a", response);
#endif
}

static void getvar_slot_suffixes(char *var_parameter, char *response)
{
	fastboot_okay("_a,_b", response);
}

static void getvar_has_slot(char *part_name, char *response)
{
	if (part_name && (!strcmp(part_name, "boot") ||
			  !strcmp(part_name, "system")))
		fastboot_okay("yes", response);
	else
		fastboot_okay("no", response);
}

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
static void getvar_emmc_mid(char *var_parameter, char *response)
{
	struct mmc *mmc;
	mmc = find_mmc_device(CONFIG_FASTBOOT_FLASH_MMC_DEV);

	if (!mmc) {
		fastboot_fail("mmc block device not found", response);
	} else if (mmc_init(mmc)) {
		fastboot_fail("mmc block device did not initailize()", response);
	} else {
		fastboot_response("OKAY", response, "%x", mmc->cid[0] >> 24);
	}
}

static void getvar_emmc_cid(char *var_parameter, char *response)
{
	struct mmc *mmc;
	char cid_code[33];
	mmc = find_mmc_device(CONFIG_FASTBOOT_FLASH_MMC_DEV);

	if (!mmc) {
		fastboot_fail("mmc block device not found", response);
	} else if (mmc_init(mmc)) {
		fastboot_fail("mmc block device did not initailize()", response);
	} else {
		int rc = 0;
		rc = snprintf(cid_code, sizeof(cid_code), "%x%x%x%x", mmc->cid[0],
						mmc->cid[1], mmc->cid[2], mmc->cid[3]);
		if (rc < 0) {
			fastboot_fail("Unable to parse eMMC CID", response);
			return;
		}
		fastboot_response("OKAY", response, "%s", cid_code);
	}
}

static void getvar_emmc_size(char *var_parameter, char *response)
{
	struct mmc *mmc;
	mmc = find_mmc_device(CONFIG_FASTBOOT_FLASH_MMC_DEV);

	if (!mmc) {
		fastboot_fail("mmc block device not found", response);
	} else if (mmc_init(mmc)) {
		fastboot_fail("mmc block device failed mmc_init()", response);
	} else {
		fastboot_response("OKAY", response, "0x%016zx", mmc->capacity);
	}
}

static void getvar_partition_type(char *part_name, char *response)
{
	int r;
	struct blk_desc *dev_desc;
	disk_partition_t part_info;

	r = fastboot_mmc_get_part_info(part_name, &dev_desc, &part_info,
				       response);
	if (r >= 0) {
		r = fs_set_blk_dev_with_part(dev_desc, r);
		if (r < 0)
			fastboot_fail("failed to set partition", response);
		else
			fastboot_okay(fs_get_type_name(), response);
	}
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH)
static void getvar_partition_size(char *part_name, char *response)
{
	int r = -1;
	size_t size = 0;

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_MMC)
	struct blk_desc *dev_desc;
	disk_partition_t part_info;

	r = fastboot_mmc_get_part_info(part_name, &dev_desc, &part_info,
				       response);
	if (r >= 0)
		size = part_info.size * part_info.blksz;
#endif
#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
	struct part_info *part_info;

	r = fastboot_nand_get_part_info(part_name, &part_info, response);
	if (r >= 0)
		size = part_info->size;
#endif
	if (r >= 0)
		fastboot_response("OKAY", response, "0x%016zx", size);
	else
		fastboot_fail("unknown partition", response);
}
#endif

#if CONFIG_IS_ENABLED(FASTBOOT_FLASH_NAND)
static void getvar_nand_page_size(char *var_parameter, char *response)
{
	struct mtd_info *mtd = get_mtd_device(NULL, 0);

	if (!mtd) {
		fastboot_fail("failed to get mtd device", response);
		return;
	}

	fastboot_response("OKAY", response, "0x%08zx", mtd->writesize);
}

static void getvar_nand_erase_size(char *var_parameter, char *response)
{
	struct mtd_info *mtd = get_mtd_device(NULL, 0);

	if (!mtd) {
		fastboot_fail("failed to get mtd device", response);
		return;
	}

	fastboot_response("OKAY", response, "0x%08zx", mtd->erasesize);
}
#endif

/**
 * fastboot_getvar() - Writes variable indicated by cmd_parameter to response.
 *
 * @cmd_parameter: Pointer to command parameter
 * @response: Pointer to fastboot response buffer
 *
 * Look up cmd_parameter first as an environment variable of the form
 * fastboot.<cmd_parameter>, if that exists return use its value to set
 * response.
 *
 * Otherwise lookup the name of variable and execute the appropriate
 * function to return the requested value.
 */
void fastboot_getvar(char *cmd_parameter, char *response)
{
	if (!cmd_parameter) {
		fastboot_fail("missing var", response);
	} else {
#define FASTBOOT_ENV_PREFIX	"fastboot."
		int i;
		char *var_parameter = cmd_parameter;
		char envstr[FASTBOOT_RESPONSE_LEN];
		const char *s;
		int rc = 0;

		rc = snprintf(envstr, sizeof(envstr) - 1,
			 	FASTBOOT_ENV_PREFIX "%s", cmd_parameter);

		if (rc < 0) {
			fastboot_fail("Unable to parse parameter", response);
			return;
		}
		s = env_get(envstr);
		if (s) {
			fastboot_response("OKAY", response, "%s", s);
			return;
		}

		strsep(&var_parameter, ":");
		for (i = 0; i < ARRAY_SIZE(getvar_dispatch); ++i) {
			if (!strcmp(getvar_dispatch[i].variable,
				    cmd_parameter)) {
				getvar_dispatch[i].dispatch(var_parameter,
							    response);
				return;
			}
		}
		pr_warn("WARNING: unknown variable: %s\n", cmd_parameter);
		fastboot_fail("Variable not implemented", response);
	}
}
