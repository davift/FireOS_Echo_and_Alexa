/*
 * Copyright 2019-2023 Amazon Technologies, Inc. All Rights Reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/hugetlb.h>
#include <linux/huge_mm.h>
#include <linux/mount.h>
#include <linux/seq_file.h>
#include <linux/highmem.h>
#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/pagemap.h>
#include <linux/mempolicy.h>
#include <linux/rmap.h>
#include <linux/swap.h>
#include <linux/kallsyms.h>
#include <linux/swapops.h>
#include <linux/mmu_notifier.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/elf.h>
#include <asm/uaccess.h>
#include <asm/tlbflush.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	#define KERNEL_VERSION_3_18
#else
	#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
		#define KERNEL_VERSION_4_4
	#elif LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
		#define KERNEL_VERSION_4_9
	#elif  LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
		#define KERNEL_VERSION_5_4
	#else
		#define KERNEL_VERSION_6_1
	#endif
#endif

#ifndef KERNEL_VERSION_6_1
#include <linux/vmacache.h>
#endif

#if defined(KERNEL_VERSION_5_4) || defined(KERNEL_VERSION_6_1)
#include <linux/pagewalk.h>
#endif



#ifdef KERNEL_VERSION_6_1
#include <linux/swapops.h>
#include <linux/sched.h>
#include <linux/mm_types.h>
#include <linux/mm.h>
#endif

#define MAX_PROC_SIZE 32

static unsigned long pss_kb = 0;
static int pid = -1;
static int in_write;

static DEFINE_SPINLOCK(perfinfo_lock);

#ifdef CONFIG_PROC_PAGE_MONITOR
/*
 * Proportional Set Size(PSS): my share of RSS.
 *
 * PSS of a process is the count of pages it has in memory, where each
 * page is divided by the number of processes sharing it.  So if a
 * process has 1000 pages all to itself, and 1000 shared with one other
 * process, its PSS will be 1500.
 *
 * To keep (accumulated) division errors low, we adopt a 64bit
 * fixed-point pss counter to minimize division errors. So (pss >>
 * PSS_SHIFT) would be the real byte count.
 *
 * A shift of 12 before division means (assuming 4K page size):
 * 	- 1M 3-user-pages add up to 8KB errors;
 * 	- supports mapcount up to 2^24, or 16M;
 * 	- supports PSS up to 2^52 bytes, or 4PB.
 */
#define PSS_SHIFT 12

#ifdef KERNEL_VERSION_3_18
int (*kernel_walk_page_range) (unsigned long addr, unsigned long end,
							struct mm_walk *walk);
struct page * (*kernel_vm_normal_page) (struct vm_area_struct *vma,
						unsigned long addr, pte_t pte);
int (*kernel_swp_swapcount) (swp_entry_t entry);
#endif

#if defined(KERNEL_VERSION_4_9) || defined(KERNEL_VERSION_4_4)
int (*kernel_walk_page_vma) (struct vm_area_struct *vma, struct mm_walk *walk);
struct page * (*kernel_vm_normal_page) (struct vm_area_struct *vma,
						unsigned long addr, pte_t pte);
int (*kernel_swp_swapcount) (swp_entry_t entry);
#endif

#if defined(KERNEL_VERSION_5_4) || defined(KERNEL_VERSION_6_1)
int (*kernel_walk_page_vma) (struct vm_area_struct *vma,const struct mm_walk_ops *ops, void *private) = walk_page_vma;
struct page * (*kernel_vm_normal_page) (struct vm_area_struct *vma,
						unsigned long addr, pte_t pte) = vm_normal_page;
int (*kernel_swp_swapcount) (swp_entry_t entry) = swp_swapcount;
#endif

struct mem_size_stats {
#ifdef KERNEL_VERSION_3_18
	struct vm_area_struct *vma;
#endif
	u64 pss;
	u64 swap_pss;
};

static void smaps_account(struct mem_size_stats *mss, struct page *page,
		unsigned long size, bool young, bool dirty)
{
	int mapcount;

	mapcount = page_mapcount(page);
	if (mapcount >= 2) {
		u64 pss_delta;
		pss_delta = (u64)size << PSS_SHIFT;
		do_div(pss_delta, mapcount);
		mss->pss += pss_delta;
	} else {
		mss->pss += (u64)size << PSS_SHIFT;
	}
}

#ifndef KERNEL_VERSION_3_18
static void smaps_pte_entry(pte_t *pte, unsigned long addr,
		struct mm_walk *walk)
#else
static void smaps_pte_entry(pte_t ptent, unsigned long addr,
	        unsigned long ptent_size, struct mm_walk *walk)
#endif
{
	struct mem_size_stats *mss = walk->private;
#ifndef KERNEL_VERSION_3_18
	struct vm_area_struct *vma = walk->vma;
#else
	struct vm_area_struct *vma = mss->vma;
	pte_t *pte = &ptent;
#endif
	struct page *page = NULL;

	if (pte_present(*pte)) {
		page = kernel_vm_normal_page(vma, addr, *pte);
	} else if (is_swap_pte(*pte)) {
		swp_entry_t swpent = pte_to_swp_entry(*pte);

		if (!non_swap_entry(swpent)) {
			int mapcount;
			u64 pss_delta = (u64)PAGE_SIZE << PSS_SHIFT;

			mapcount = kernel_swp_swapcount(swpent);
			if (mapcount >= 2)
				do_div(pss_delta, mapcount);
			mss->swap_pss += pss_delta;
		} else if (is_migration_entry(swpent)) {
			#ifdef KERNEL_VERSION_6_1
				page = pfn_swap_entry_to_page(swpent);
			#else
				page = migration_entry_to_page(swpent);
			#endif
		}

	}
	if (!page)
		return;
	smaps_account(mss, page, PAGE_SIZE, pte_young(*pte), pte_dirty(*pte));
}

#if !defined(KERNEL_VERSION_3_18)
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
static void smaps_pmd_entry(pmd_t *pmd, unsigned long addr,
		struct mm_walk *walk)
{
	struct mem_size_stats *mss = walk->private;
	struct vm_area_struct *vma = walk->vma;
	bool locked = !!(vma->vm_flags & VM_LOCKED);
	struct page *page;

	/* FOLL_DUMP will return -EFAULT on huge zero page */
	page = follow_trans_huge_pmd(vma, addr, pmd, FOLL_DUMP);
	if (IS_ERR_OR_NULL(page))
		return;
	smaps_account(mss, page, HPAGE_PMD_SIZE, pmd_young(*pmd), pmd_dirty(*pmd));
}
#else
static void smaps_pmd_entry(pmd_t *pmd, unsigned long addr, struct mm_walk *walk)
{
}
#endif
#endif

static int smaps_pte_range(pmd_t *pmd, unsigned long addr, unsigned long end,
							struct mm_walk *walk)
{
#ifndef KERNEL_VERSION_3_18
	struct vm_area_struct *vma = walk->vma;
#else
	struct mem_size_stats *mss = walk->private;
	struct vm_area_struct *vma = mss->vma;
#endif
	pte_t *pte;
	spinlock_t *ptl;

#if !defined(KERNEL_VERSION_4_9) && !defined(KERNEL_VERSION_5_4) && !defined(KERNEL_VERSION_6_1)
	if (pmd_trans_huge_lock(pmd, vma, &ptl) == 1) {
#else
	ptl = pmd_trans_huge_lock(pmd, vma);
	if (ptl) {
#endif
#ifndef KERNEL_VERSION_3_18
		smaps_pmd_entry(pmd, addr, walk);
#else
		smaps_pte_entry(*(pte_t *)pmd, addr, HPAGE_PMD_SIZE, walk);
#endif
		spin_unlock(ptl);
		return 0;
	}

	if (pmd_trans_unstable(pmd))
		return 0;
	/*
	 * The mmap_sem held all the way back in print_mem() is what
	 * keeps khugepaged out of here and from collapsing things
	 * in here.
	 */
	pte = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
	for (; addr != end; pte++, addr += PAGE_SIZE)
#ifndef KERNEL_VERSION_3_18
		smaps_pte_entry(pte, addr, walk);
#else
		smaps_pte_entry(*pte, addr, PAGE_SIZE, walk);
#endif
	pte_unmap_unlock(pte - 1, ptl);
	cond_resched();
	return 0;
}

static int show_smap(void *v, void *pss_kb_ptr)
{
	struct vm_area_struct *vma = v;
	struct mem_size_stats mss;

#if !defined(KERNEL_VERSION_5_4) && !defined(KERNEL_VERSION_6_1)
	struct mm_walk smaps_walk = {
		.pmd_entry = smaps_pte_range,
#ifdef CONFIG_HUGETLB_PAGE
		.hugetlb_entry = smaps_hugetlb_range,
#endif
		.mm = vma->vm_mm,
		.private = &mss,
	};
#else
	struct mm_walk_ops smaps_walk_ops = {
		.pmd_entry = smaps_pte_range,
#ifdef CONFIG_HUGETLB_PAGE
                .hugetlb_entry = smaps_hugetlb_range,
#endif

	};
#endif
	int ret = 0;
	memset(&mss, 0, sizeof mss);
#ifndef KERNEL_VERSION_3_18
#if defined(KERNEL_VERSION_5_4) || defined(KERNEL_VERSION_6_1)
	kernel_walk_page_vma(vma, &smaps_walk_ops, &mss);
#else
	/* mmap_sem is held in print_mem */
	kernel_walk_page_vma(vma, &smaps_walk);
#endif
#else
	mss.vma = vma;
	if (vma->vm_mm && !is_vm_hugetlb_page(vma))
		kernel_walk_page_range(vma->vm_start, vma->vm_end, &smaps_walk);
#endif
	*(unsigned long *)pss_kb_ptr  += ((unsigned long)(mss.pss >> (10 + PSS_SHIFT)) + (unsigned long)(mss.swap_pss >> (10 + PSS_SHIFT)));
	return ret;
}

#ifdef KERNEL_VERSION_6_1
static void print_mem(struct task_struct *task)
{
	struct mm_struct *mm = task->mm;
	struct vm_area_struct *vma;
	VMA_ITERATOR(vmi, mm, 0);

	if (mm) {
		pss_kb = 0;
		down_read(&mm->mmap_lock);
		for_each_vma(vmi, vma) {
			show_smap(vma, &pss_kb);
		}
		up_read(&mm->mmap_lock);
	}

}
#else
static void print_mem(struct task_struct *task)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;

	mm = task->mm;
	if (mm) {
		pss_kb = 0;
		down_read(&mm->mmap_sem);
		for (vma = mm->mmap ; vma ; vma = vma->vm_next) {
			show_smap(vma, &pss_kb);
		}
		up_read(&mm->mmap_sem);
	}
}

#endif

static int calculate_pss(void)
{
	struct task_struct *task;

	/* Validate the pid before going through the task list */
	if (pid <= 0 || pid >= PID_MAX_LIMIT) {
		pid = -1;
		in_write = 0;
		return -EINVAL;
	}

	for_each_process(task) {
		if (task == NULL || task->pid != pid)
			continue;
		print_mem(task);
		break;
	}

	pid = -1;
	in_write = 0;

	return 0;
}

static struct kobject *perfinfo_kobj;

static ssize_t total_pss_kb_show(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf)
{
	calculate_pss();
        return sprintf(buf, "%lu\n", pss_kb);
}

static ssize_t total_pss_kb_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int tpid;

	if (count > MAX_PROC_SIZE)
		return -EINVAL;

	if (!spin_trylock(&perfinfo_lock)) {
		pr_info("perfinfo_lock failed\n");
		return -EIO;
	}

	if (in_write) {
		spin_unlock(&perfinfo_lock);
		pr_info("pid data not read\n");
		return -EIO;
	}

	if (sscanf(buf, "%d", &tpid) != 1) {
		spin_unlock(&perfinfo_lock);
		return -EINVAL;
	}

	/* Validate the input pid */
	if (tpid <= 0 || tpid >= PID_MAX_LIMIT) {
		spin_unlock(&perfinfo_lock);
		return -EINVAL;
	}

	pid = tpid;
	in_write = 1;
	spin_unlock(&perfinfo_lock);

	return count;
}


static struct kobj_attribute total_pss_kb_attr =__ATTR(total_pss_kb, 0660, total_pss_kb_show,
                                                   total_pss_kb_store);


static int __init perfinfo_init(void)
{
	int error = 0;
#if !defined(KERNEL_VERSION_5_4) && !defined(KERNEL_VERSION_6_1)
#ifndef KERNEL_VERSION_3_18
	kernel_walk_page_vma = (void*)kallsyms_lookup_name("walk_page_vma");
	if (kernel_walk_page_vma == NULL)
		return -EINVAL;
#else
	kernel_walk_page_range = (void*)kallsyms_lookup_name("walk_page_range");
	if (kernel_walk_page_range == NULL)
		return -EINVAL;
#endif
	kernel_vm_normal_page = (void*)kallsyms_lookup_name("vm_normal_page");
	if (kernel_vm_normal_page == NULL)
		return -EINVAL;

	kernel_swp_swapcount = (void*)kallsyms_lookup_name("swp_swapcount");
	if (kernel_swp_swapcount == NULL)
		return -EINVAL;
#endif
        perfinfo_kobj = kobject_create_and_add("perfinfo",
                                                 kernel_kobj);
        if(!perfinfo_kobj)
                return -ENOMEM;

        error = sysfs_create_file(perfinfo_kobj, &total_pss_kb_attr.attr);
        if (error) {
                pr_err("failed to create the total_pss_kb file in /sys/kernel/perfinfo \n");
        }
        return error;
}

static void __exit perfinfo_exit(void)
{
	pr_info("Module exiting\n");
	kobject_put(perfinfo_kobj);
	return;
}

module_init(perfinfo_init);
module_exit(perfinfo_exit);

MODULE_AUTHOR ("Rohan Bhutkar, rbhutkar@amazon.com");
MODULE_DESCRIPTION ("Print PSS information for a process in kB");
MODULE_LICENSE("GPL v2");

#endif /* CONFIG_PROC_PAGE_MONITOR */
