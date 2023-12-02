// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2014 Broadcom Corporation.
 * Copyright 2015 Free Electrons.
 */

#include <config.h>
#include <common.h>

#include <fastboot.h>
#include <image-sparse.h>

#include <linux/mtd/mtd.h>
#include <jffs2/jffs2.h>
#include <nand.h>
#include <ubi_uboot.h>

struct fb_nand_sparse {
	struct mtd_info		*mtd;
	struct part_info	*part;
};

__weak int board_fastboot_erase_partition_setup(char *name)
{
	return 0;
}

__weak int board_fastboot_write_partition_setup(char *name)
{
	return 0;
}

#if defined(CONFIG_MTK_NAND_FB_CUS) && defined(CONFIG_CMD_UBI)
struct boot_device_info {
	const char *name_1;
	const char *name_2;
	unsigned int start_pgs;
	unsigned int bootpart_pgs;
} boot_copies[] = {
	{"mlk1", "brhgptlk_0", 0, 64},
	{"mlk2", "brhgptlk_1", 256, 64},
	{"mlk3", "brhgptlk_2", 512, 64},
	{"mlk4", "brhgptlk_3", 768, 64}
};
struct fb_ubi_sparse {
	struct ubi_volume       *vol;
	lbaint_t                total_blks;
	lbaint_t                blk_sz;
};

static int fb_nand_is_boot_part(char *name)
{
	int i, boot_part_num = 0;

	boot_part_num = sizeof(boot_copies) / sizeof(struct boot_device_info);
	for (i = 0; i < boot_part_num; i++) {
		if (!strncmp(name, boot_copies[i].name_1, strlen(name)) ||
		    !strncmp(name, boot_copies[i].name_2, strlen(name)))
			return i;
	}

	return -1;
}

static int fb_nand_boot_addr(char *name, int *boot_pages, int *img_size)
{
	int i;

	i = fb_nand_is_boot_part(name);
	if (i >= 0) {
		*boot_pages = boot_copies[i].start_pgs;
		*img_size = boot_copies[i].bootpart_pgs;
	}

	return i;
}

static lbaint_t fb_ubi_sparse_write(struct sparse_storage *info,
				    lbaint_t blk, lbaint_t blkcnt,
				    const void *buffer)
{
	struct fb_ubi_sparse *sparse = info->priv;
	int ret;
	size_t total_size = sparse->total_blks * sparse->blk_sz;

	if (blk == info->start) {
		ret = ubi_volume_begin_write(sparse->vol->name, buffer,
					     (size_t)blkcnt * info->blksz,
					     total_size);
	} else {
		ret = ubi_volume_continue_write(sparse->vol->name, buffer,
						(size_t)blkcnt * info->blksz);
	}

	if (ret != 0) {
		printf("Failed to write sparse chunk\n");
		/*since the return type lbaint_t is unsigned
		 *it should not return minus value when fail
		 *just return 0
		 *the function which call fb_nand_sparse_write will know
		 *write fail*/
		return 0;
	}

	return blkcnt;
}

static lbaint_t fb_ubi_sparse_reserve(struct sparse_storage *info,
				      lbaint_t blk, lbaint_t blkcnt)
{
	return blkcnt;
}

static void fastboot_ubi_write(const char *cmd, void *download_buffer,
			       u32 download_bytes, char *response)
{
	struct ubi_volume *vol = NULL;
	u32 leb_size, rsvd_bytes;
	int ret;

	vol = ubi_find_volume(cmd);
	if (!vol) {
		printf("Haven't found the volume.\n");
		fastboot_fail("invalid NAND device", response);
	} else {
		if (is_sparse_image(download_buffer)) {
			struct fb_ubi_sparse sparse_priv;
			struct sparse_storage sparse;
			sparse_header_t *s_header;

			leb_size = vol->ubi->leb_size;
			rsvd_bytes = vol->reserved_pebs *
				     (leb_size - vol->data_pad);

			sparse_priv.vol = vol;
			s_header = (sparse_header_t *)download_buffer;
			sparse_priv.total_blks = s_header->total_blks;
			sparse_priv.blk_sz = s_header->blk_sz;

			sparse.blksz = vol->ubi->min_io_size;
			sparse.start = 0;
			sparse.size = rsvd_bytes / sparse.blksz;
			sparse.write = fb_ubi_sparse_write;
			sparse.reserve = fb_ubi_sparse_reserve;
			sparse.mssg = fastboot_fail;

			printf("Flashing sparse image at offset " LBAFU "\n",
			       sparse.start);

			sparse.priv = &sparse_priv;
			ret = write_sparse_image(&sparse, cmd, download_buffer,
						 response);
			if (!ret)
				fastboot_okay(NULL, response);
		} else {
			ret = ubi_volume_write(cmd, download_buffer,
					       download_bytes);
			if (ret < 0)
				fastboot_fail("", response);

			printf("wrote %u bytes\n", download_bytes);
			fastboot_okay(NULL, response);
		}
	}
}
#endif

int fb_nand_lookup(const char *partname,
		   struct mtd_info **mtd,
		   struct part_info **part,
		   char *response)
{
	struct mtd_device *dev;
	int ret;
	u8 pnum;

	ret = mtdparts_init();
	if (ret) {
		pr_err("Cannot initialize MTD partitions\n");
		fastboot_fail("cannot init mtdparts", response);
		return ret;
	}

	ret = find_dev_and_part(partname, &dev, &pnum, part);
	if (ret) {
		pr_err("cannot find partition: '%s'\n", partname);
		fastboot_fail("cannot find partition", response);
		return ret;
	}

	if (dev->id->type != MTD_DEV_TYPE_NAND) {
		pr_err("partition '%s' is not stored on a NAND device\n",
		      partname);
		fastboot_fail("not a NAND device", response);
		return -EINVAL;
	}

	*mtd = get_nand_dev_by_index(dev->id->num);

	return 0;
}

static int _fb_nand_erase(struct mtd_info *mtd, struct part_info *part,
			  bool force)
{
	struct erase_info ei;
	int ret;
	u64 offset;

	printf("Erasing blocks 0x%llx to 0x%llx\n",
	       part->offset, part->offset + part->size);

	for (offset = part->offset; offset < (part->offset + part->size);
	     offset += mtd->erasesize) {
		memset(&ei, 0, sizeof(struct erase_info));
		ei.mtd = mtd;
		ei.addr = offset;
		ei.len = mtd->erasesize;

		if (force)
			ei.scrub = 1;

		ret = mtd_block_isbad(mtd, ei.addr);
		if (ret > 0 && !force)
			continue;

		ret = mtd_erase(mtd, &ei);
		if (ret == -EIO && !force)
			mtd_block_markbad(mtd, offset);
	}

	printf("........ erased 0x%llx bytes from '%s'\n",
	       part->size, part->name);

	return 0;
}

static int _fb_block_find_next_good(struct mtd_info *mtd,
				    struct part_info *part, u32 page)
{
	u32 page_per_block = mtd->erasesize / mtd->writesize;
	u32 count = part->size / mtd->writesize;
	u32 start_page = part->offset / mtd->writesize;
	u32 i, block;
	int ret = 0;

	if ((page - start_page) >= count)
		return -EINVAL;

	i = page - start_page + page_per_block;
	i = (i / page_per_block) * page_per_block;

	for (; i < count; i += page_per_block) {
		if (!mtd_block_isbad(mtd, (i * mtd->writesize) + part->offset))
			break;
	}

	if (i < count)
		ret = i + start_page;
	else
		ret = -EINVAL;

	return ret;
}

static int _fb_move_worn_bad_blk(struct mtd_info *mtd, struct part_info *part,
				 u32 err_page)
{
	u32 dst_page, src_page;
	u32 pages_per_block;
	u32 erase_offset;
	void *buf;
	int ret = 0;
	size_t written, read_;
	struct erase_info ei;

	buf = kmalloc(mtd->writesize, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pages_per_block = mtd->erasesize / mtd->writesize;

findgood:
	src_page = (err_page / pages_per_block) * pages_per_block;

	/* Find next good block */
	ret = _fb_block_find_next_good(mtd, part, err_page);
	if (ret < 0) {
		pr_err("out of part range!\n");
		goto freebuf;
	}
	dst_page = ret;

	/* Erase good block */
	erase_offset = dst_page * mtd->writesize;
	memset(&ei, 0, sizeof(struct erase_info));
	ei.mtd = mtd;
	ei.addr = erase_offset;
	ei.len = mtd->erasesize;
	ret = mtd_erase(mtd, &ei);
	if (ret == -EIO) {
		pr_err("Erase fail at page %d\n", dst_page);
		mtd_block_markbad(mtd, erase_offset);
		goto findgood;
	}

	/* Move programmed data to new good block */
	while ((src_page % pages_per_block) < (err_page % pages_per_block)) {
		ret = mtd_read(mtd, src_page * mtd->writesize,
			       mtd->writesize, &read_, buf);
		if (ret < 0) {
			/*
			 * Programmed data read error and data lost
			 * Skip it and go to read next page
			 */
			pr_info("page %d data lost!\n", src_page);
			src_page++;
			dst_page++;
			continue;
		}

		ret = mtd_write(mtd, dst_page * mtd->writesize,
				mtd->writesize, &written, buf);
		if (ret == -EIO) {
			/* Program failed, mark bad and go to find good block */
			mtd_block_markbad(mtd, dst_page * mtd->writesize);
			goto findgood;
		} else if (written != (mtd->writesize)) {
			/* return directly if other errors happened */
			pr_err("write error %u\n", written);
			ret = -EFAULT;
			goto freebuf;
		}

		src_page++;
		dst_page++;
	}

	/* Mark worn block as bad */
	mtd_block_markbad(mtd, err_page * mtd->writesize);

freebuf:
	kfree(buf);

	return ret < 0 ? ret : dst_page;
}

static int _fb_write_skip_bad(struct mtd_info *mtd, struct part_info *part,
			      void *buffer, u32 offset,
			      size_t length, size_t *written, int flags)
{
	uint32_t write_count;
	ssize_t block_offset;
	size_t retlen;
	int ret = 0, phy_block;
	uint8_t *verbuf;

	if ((offset % mtd->writesize) != 0)
		return -EINVAL;

	if (written != NULL)
		*written = 0;

	while (length) {
		if (offset >= (part->offset + part->size)) {
			pr_err("offset 0x%08x out of part range %llx!\n",
			       offset, part->offset + part->size);
			return -EINVAL;
		}

		block_offset = offset & (mtd->erasesize - 1);
		if (mtd_block_isbad(mtd, offset & ~(mtd->erasesize - 1))) {
			printf("Skip bad block 0x%08x\n",
				offset & ~(mtd->erasesize - 1));
			offset += mtd->erasesize - block_offset;
			if (written != NULL)
				*written += mtd->erasesize - block_offset;
			continue;
		}
REWRITE:

		write_count = mtd->erasesize - (offset % mtd->erasesize);
		write_count = (write_count < length ? write_count : length);
		ret = mtd_write(mtd, offset, write_count, &retlen, buffer);

		if (ret == -EIO) {
			phy_block = offset / mtd->writesize;
			/* move worn bad block */
			pr_err("Write fail at blk %d\n", phy_block);
			ret = _fb_move_worn_bad_blk(mtd, part, phy_block);
			if (ret < 0) {
				pr_err("Fail to recover worn bad, err:%d!\n",
				       ret);
				return ret;
			}

			pr_err("Recover worn bad successfully!\n");
			if (written != NULL)
				*written += ret * mtd->writesize - offset;
			offset = ret * mtd->writesize;

			goto REWRITE;
		} else if (retlen != write_count) {
			pr_err("Write error %u\n", retlen);
			return -EFAULT;
		}

		if (flags & WITH_WR_VERIFY) {
			verbuf = memalign(ARCH_DMA_MINALIGN, write_count);
			retlen = 0;
			ret = mtd_read(mtd, offset,
				       write_count, &retlen, verbuf);
			if (ret < 0 || retlen != write_count) {
				pr_err("fb verify read error ret = %d retlen = %u\n",
				       ret, retlen);
				free(verbuf);
				return -EFAULT;

			}

			ret = memcmp(buffer, verbuf, write_count);
			free(verbuf);

			if (ret) {
				pr_err("fb verify error ret = %d\n", ret);
				free(verbuf);
				return -EFAULT;
			}
		}

		if (written != NULL)
			*written += write_count;
		length -= write_count;
		offset += write_count;
		buffer += write_count;
	}

	return ret;
}

static int _fb_nand_write(struct mtd_info *mtd, struct part_info *part,
			  void *buffer, u32 offset,
			  size_t length, size_t *written)
{
	int flags = WITH_WR_VERIFY;

#ifdef CONFIG_FASTBOOT_FLASH_NAND_TRIMFFS
	flags |= WITH_DROP_FFS;
#endif

	return _fb_write_skip_bad(mtd, part, buffer, offset, length, written,
				  flags);

}

static lbaint_t fb_nand_sparse_write(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt, const void *buffer)
{
	struct fb_nand_sparse *sparse = info->priv;
	size_t written;
	int ret;

	ret = _fb_nand_write(sparse->mtd, sparse->part, (void *)buffer,
			     blk * info->blksz,
			     blkcnt * info->blksz, &written);

	if (ret < 0) {
		printf("Failed to write sparse chunk\n");
		/*since the return type lbaint_t is unsigned
		 *it should not return minus value when fail
		 *just return 0
		 *the function which call fb_nand_sparse_write will know
		 *write fail*/
		return 0;
	}

/* TODO - verify that the value "written" includes the "bad-blocks" ... */

	/*
	 * the return value must be 'blkcnt' ("good-blocks") plus the
	 * number of "bad-blocks" encountered within this space...
	 */
	return written / info->blksz;
}

static int check_skip_len(struct mtd_info *mtd, loff_t offset, size_t length,
			  size_t *used)
{
	size_t len_excl_bad = 0;
	int ret = 0;

	while (len_excl_bad < length) {
		size_t block_len, block_off;
		loff_t block_start;

		if (offset >= mtd->size)
			return -1;

		block_start = offset & ~(loff_t)(mtd->erasesize - 1);
		block_off = offset & (mtd->erasesize - 1);
		block_len = mtd->erasesize - block_off;

		if (!mtd_block_isbad(mtd, block_start))
			len_excl_bad += block_len;
		else
			ret = 1;

		offset += block_len;
		*used += block_len;
	}

	/* If the length is not a multiple of block_len, adjust. */
	if (len_excl_bad > length)
		*used -= (len_excl_bad - length);

	return ret;
}

static lbaint_t fb_nand_sparse_reserve(struct sparse_storage *info,
		lbaint_t blk, lbaint_t blkcnt)
{
	struct fb_nand_sparse *sparse = info->priv;
	loff_t offset;
	size_t length;
	size_t used = 0;
	offset = blk * info->blksz;
	length = blkcnt * info->blksz;

	check_skip_len(sparse->mtd, offset, length, &used);

	return used  / info->blksz;
}

/**
 * fastboot_nand_get_part_info() - Lookup NAND partion by name
 *
 * @part_name: Named device to lookup
 * @part_info: Pointer to returned part_info pointer
 * @response: Pointer to fastboot response buffer
 */
int fastboot_nand_get_part_info(char *part_name, struct part_info **part_info,
				char *response)
{
	struct mtd_info *mtd = NULL;

	return fb_nand_lookup(part_name, &mtd, part_info, response);
}

/**
 * fastboot_nand_flash_write() - Write image to NAND for fastboot
 *
 * @cmd: Named device to write image to
 * @download_buffer: Pointer to image data
 * @download_bytes: Size of image data
 * @response: Pointer to fastboot response buffer
 */
void fastboot_nand_flash_write(const char *cmd, void *download_buffer,
			       u32 download_bytes, char *response)
{
	struct part_info part_mtd;
	struct part_info *part;
	struct mtd_info *mtd = NULL;
	int boot_pages, img_size, ret;
	const char *NAND_PARTITION_NAME = "nand0";

	ret = fb_nand_lookup(cmd, &mtd, &part, response);
	if (ret) {
#ifdef CONFIG_MTK_NAND_FB_CUS
		/* if the nand is erased or empty, then we need to check whether
		 * it is a boot partition
		 */
		ret = fb_nand_boot_addr(cmd, &boot_pages, &img_size);
		if (ret >= 0) {
			/* cmd matches boot partition name, use nand partition name
			 * "nand0" to replace cmd here.
			 */
			ret = fb_nand_lookup(NAND_PARTITION_NAME, &mtd, &part, response);
			if (ret) {
				pr_err("invalid NAND device\n");
				fastboot_fail("invalid NAND device", response);
				return;
			}
#ifdef CONFIG_TARGET_MT8519
			if (mtd->writesize == 2048)
				img_size *= 2;
#endif
			part->offset = boot_pages * mtd->writesize;
			part->size =  img_size * mtd->writesize;
		} else {
			printf("\nlooking for %s as ubi volume\n", cmd);
			/* Try to write to the ubi volume */
			fastboot_ubi_write(cmd, download_buffer, download_bytes, response);
			printf("\nwrote %u bytes on ubi volume\n", download_bytes);
			return;
		}
#else
		pr_err("invalid NAND device\n");
		fastboot_fail("invalid NAND device", response);
		return;
#endif
	}

	ret = board_fastboot_write_partition_setup(part->name);
	if (ret)
		return;

	if (is_sparse_image(download_buffer)) {
		struct fb_nand_sparse sparse_priv;
		struct sparse_storage sparse;

		sparse_priv.mtd = mtd;
		sparse_priv.part = part;

		sparse.blksz = mtd->writesize;
		sparse.start = part->offset / sparse.blksz;
		sparse.size = part->size / sparse.blksz;
		sparse.write = fb_nand_sparse_write;
		sparse.reserve = fb_nand_sparse_reserve;
		sparse.mssg = fastboot_fail;

		printf("Flashing sparse image at offset " LBAFU "\n",
		       sparse.start);

		sparse.priv = &sparse_priv;
		ret = write_sparse_image(&sparse, cmd, download_buffer,
					 response);
		if (!ret)
			fastboot_okay(NULL, response);
	} else {
		printf("Flashing raw image at offset 0x%llx\n",
		       part->offset);

		/* Must erase partition before flashing for raw partition */
		ret = _fb_nand_erase(mtd, part, false);
		if (ret) {
			pr_err("failed erasing from device %s\n", mtd->name);
			fastboot_fail("failed erasing from device", response);
			return;
		}

		ret = _fb_nand_write(mtd, part, download_buffer, part->offset,
				     download_bytes, NULL);
		/* if bootloader is upgraded, please refresh partition table */
		if ((strncmp("BOOTLOADER!", (char *)download_buffer, 11) == 0)
			|| (strncmp("NANDCFG!", (char *)download_buffer, 8) == 0))
			board_mtdparts_init();

		printf("........ wrote %u bytes to '%s'\n",
		       download_bytes, part->name);
	}

	if (ret) {
		fastboot_fail("error writing the image", response);
		return;
	}

	fastboot_okay(NULL, response);
}

/**
 * fastboot_nand_flash_erase() - Erase NAND for fastboot
 *
 * @cmd: Named device to erase
 * @response: Pointer to fastboot response buffer
 */
void fastboot_nand_erase(const char *cmd, char *response)
{
	struct part_info part_mtd;
	struct part_info *part;
	struct mtd_info *mtd = NULL;
	int boot_pages, img_size, ret;

	const char *NAND_PARTITION_NAME = "nand0";
	const char *NAND_FORCE_FORMAT = "force_format";
	bool force = false;

	if (!strncmp(cmd, NAND_PARTITION_NAME, strlen(NAND_PARTITION_NAME)) ||
	    !strncmp(cmd, NAND_FORCE_FORMAT, strlen(NAND_FORCE_FORMAT))) {
		mtd = get_mtd_device(NULL, 0);
		if (mtd) {
			part_mtd.offset = 0;
			part_mtd.size = mtd->size;
			part_mtd.name = NAND_PARTITION_NAME;

			part = &part_mtd;
		} else {
			pr_err("invalid NAND device\n");
			fastboot_fail("invalid NAND device", response);
			return;
		}

		if (!strncmp(cmd, "force_format", 12))
			force = true;
	} else if (fb_nand_is_boot_part(cmd) >= 0) {
#ifdef CONFIG_MTK_NAND_FB_CUS
		mtd = get_mtd_device(NULL, 0);
		if (mtd) {
			ret = fb_nand_boot_addr(cmd, &boot_pages, &img_size);
			if (ret >= 0) {
#ifdef CONFIG_TARGET_MT8519
				if (mtd->writesize == 2048)
					img_size *= 2;
#endif
				part_mtd.offset = boot_pages * mtd->writesize;
				part_mtd.size = img_size * mtd->writesize;
			} else {
				pr_err("invalid boot pages\n");
				fastboot_fail("invalid boot pages", response);
				return;
			}
			part_mtd.name = NAND_PARTITION_NAME;

			part = &part_mtd;
		} else {
			pr_err("invalid NAND device\n");
			fastboot_fail("invalid NAND device", response);
			return;
		}
#else
		pr_err("invalid NAND device\n");
		fastboot_fail("invalid NAND device", response);
		return;
#endif
	} else {
		ret = fb_nand_lookup(cmd, &mtd, &part, response);
		if (ret) {
			pr_err("invalid NAND device\n");
			fastboot_fail("invalid NAND device", response);
			return;
		}

		ret = board_fastboot_erase_partition_setup(part->name);
		if (ret)
			return;
	}

	ret = _fb_nand_erase(mtd, part, force);
	if (ret) {
		pr_err("failed erasing from device %s\n", mtd->name);
		fastboot_fail("failed erasing from device", response);
		return;
	}

#ifdef CONFIG_NAND
	if (force)
		mtk_uboot_part(mtd, NULL);
#endif

	fastboot_okay(NULL, response);
}
