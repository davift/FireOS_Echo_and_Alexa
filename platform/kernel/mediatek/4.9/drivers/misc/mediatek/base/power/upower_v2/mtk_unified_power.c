/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/ktime.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <mt-plat/mtk_chip.h>
#include <linux/delay.h>
#include <linux/math64.h>

/* local include */
#include "mtk_upower.h"

#if defined(CONFIG_MACH_MT8518)
#ifdef UPOWER_NUM_LARGER
#include "mtk_svs.h"
#include "mtk_static_power.h"
#endif
#include <mt8518-cpufreq.h>
#endif
#if defined(CONFIG_MACH_MT8512)
#include "mtk_static_power.h"
#include <mt8512-cpufreq.h>
#endif


#if UPOWER_ENABLE_TINYSYS_SSPM
#include <sspm_reservedmem_define.h>
#endif

#ifdef UPOWER_USE_QOS_IPI
#if UPOWER_ENABLE_TINYSYS_SSPM
//#include <mtk_spm_vcore_dvfs_ipi.h>
#include <mtk_vcorefs_governor.h>
#include <helio-dvfsrc-ipi.h>
#endif
#endif

#if UPOWER_ENABLE
unsigned char upower_enable = 1;
#else
unsigned char upower_enable;
#endif
#ifdef UPOWER_NUM_LARGER
int iter;
#endif

bool is_upower_org_tbl_ready;

#if defined(CONFIG_MACH_MT8512)
#define NR_SVS_DET    1
#endif

/* for log print */
#define LOG_BUF_LEN 1024
#define UPOWER_UT
struct mtk_upower_buf {
	char buf[LOG_BUF_LEN];
	char *p_idx;
};

#define reset_upower_buf(upower) ((upower).p_idx = (upower).buf)
#define get_upower_buf(upower)   ((upower).buf)
#define upower_buf_append(upower, fmt, args...) \
	((upower).p_idx += snprintf((upower).p_idx, \
	LOG_BUF_LEN - strlen((upower).buf), fmt, ##args))

/* reference to target upower tbl, ex: big upower tbl */
struct upower_tbl *upower_tbl_ref;
/* Used for rcu lock, points to all the target tables list*/
struct upower_tbl_info *new_p_tbl_infos;
/* reference to raw upower tbl lists */
struct upower_tbl_info *upower_tbl_infos;
/* sspm reserved mem info for sspm upower */
phys_addr_t upower_data_phy_addr, upower_data_virt_addr;
unsigned long long upower_data_size;
unsigned int opp_row_num;
#if 0
static void print_tbl(void)
{
	int i, j;
/* --------------------print static orig table -------------------------*/
#if 0
	struct upower_tbl *tbl;

	for (i = 0; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;
		/* table size must be 512 bytes */
		upower_debug("Bank %d , tbl size %ld\n",
			i, sizeof(struct upower_tbl));
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			upower_debug(" cap, volt, dyn, lkg: %llu, %u, %u, {%u, %u, %u, %u, %u}\n",
				tbl->row[j].cap, tbl->row[j].volt,
				tbl->row[j].dyn_pwr,
				tbl->row[j].lkg_pwr[0],
				tbl->row[j].lkg_pwr[1],
				tbl->row[j].lkg_pwr[2],
				tbl->row[j].lkg_pwr[3],
				tbl->row[j].lkg_pwr[4]);
		}

		upower_debug(" lkg_idx, num_row: %d, %d\n",
				tbl->lkg_idx, tbl->row_num);
		upower_debug("-----------------------------------------------------------------\n");
	}
#else
/* --------------------print sram table -------------------------*/
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		/* table size must be 512 bytes */
		upower_debug("---Bank %d , tbl size %ld---\n",
			i, sizeof(struct upower_tbl));
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			upower_debug(" cap = %llu, volt = %u, dyn = %u, lkg = {%u, %u, %u, %u, %u}\n",
				upower_tbl_ref[i].row[j].cap,
				upower_tbl_ref[i].row[j].volt,
				upower_tbl_ref[i].row[j].dyn_pwr,
				upower_tbl_ref[i].row[j].lkg_pwr[0],
				upower_tbl_ref[i].row[j].lkg_pwr[1],
				upower_tbl_ref[i].row[j].lkg_pwr[2],
				upower_tbl_ref[i].row[j].lkg_pwr[3],
				upower_tbl_ref[i].row[j].lkg_pwr[4]);
		}
		upower_debug(" lkg_idx, num_row: %d, %d\n",
			upower_tbl_ref[i].lkg_idx,
			upower_tbl_ref[i].row_num);
		upower_debug("-------------------------------------------------\n");
	}
#endif
}
#endif

#ifdef UPOWER_UT
void upower_ut(void)
{
	struct upower_tbl_info **addr_ptr_tbl_info;
	struct upower_tbl_info *ptr_tbl_info;
	struct upower_tbl *ptr_tbl;
	int i, j;

	upower_debug("----upower_get_tbl()----\n");
	/* get addr of ptr which points to upower_tbl_infos[] */
	addr_ptr_tbl_info = upower_get_tbl();
	/* get ptr which points to upower_tbl_infos[] */
	ptr_tbl_info = *addr_ptr_tbl_info;
	upower_debug("get upower tbl location = %p\n",
			ptr_tbl_info[0].p_upower_tbl);
#if 0
	upower_debug("ptr_tbl_info --> %p --> tbl %p (p_upower_tbl_infos --> %p)\n",
				ptr_tbl_info, ptr_tbl_info[0].p_upower_tbl,
				p_upower_tbl_infos);
#endif

	/* print all the tables that record in upower_tbl_infos[]*/
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		upower_debug("bank %d\n", i);
		ptr_tbl = ptr_tbl_info[i].p_upower_tbl;
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			upower_debug(" cap = %llu, volt = %u, dyn = %u, lkg = {%u, %u, %u, %u, %u, %u}\n",
					ptr_tbl->row[j].cap,
					ptr_tbl->row[j].volt,
					ptr_tbl->row[j].dyn_pwr,
					ptr_tbl->row[j].lkg_pwr[0],
					ptr_tbl->row[j].lkg_pwr[1],
					ptr_tbl->row[j].lkg_pwr[2],
					ptr_tbl->row[j].lkg_pwr[3],
					ptr_tbl->row[j].lkg_pwr[4],
					ptr_tbl->row[j].lkg_pwr[5]);
		}
		upower_debug(" lkg_idx, num_row, nr_idle_states: %d, %d ,%d\n",
					ptr_tbl->lkg_idx, ptr_tbl->row_num,
					ptr_tbl->nr_idle_states);

		for (i = 0; i < NR_UPOWER_DEGREE; i++) {
			upower_debug("(%d)C c0 = %lu, c1 = %lu\n",
					degree_set[i],
					ptr_tbl->idle_states[i][0].power,
					ptr_tbl->idle_states[i][1].power);

		}
	}

	upower_debug("----upower_get_power()----\n");
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		upower_debug("bank %d\n", i);
		upower_debug("[dyn] %u, %u, %u, %u, %u, %u\n",
					upower_get_power(i, 0, UPOWER_DYN),
					upower_get_power(i, 1, UPOWER_DYN),
					upower_get_power(i, 2, UPOWER_DYN),
					upower_get_power(i, 3, UPOWER_DYN),
					upower_get_power(i, 4, UPOWER_DYN),
					upower_get_power(i, 5, UPOWER_DYN));
		upower_debug("[lkg] %u, %u, %u, %u, %u, %u\n",
					upower_get_power(i, 0, UPOWER_LKG),
					upower_get_power(i, 1, UPOWER_LKG),
					upower_get_power(i, 2, UPOWER_LKG),
					upower_get_power(i, 3, UPOWER_LKG),
					upower_get_power(i, 4, UPOWER_LKG),
					upower_get_power(i, 5, UPOWER_LKG));
	}
}
#endif
static void upower_update_dyn_pwr(void)
{
	unsigned long long refPower, newVolt, refVolt, newPower;
	unsigned long long temp1, temp2;
	int i, j;
	struct upower_tbl *tbl;

	for (i = 0; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			refPower = (unsigned long long)tbl->row[j].dyn_pwr;
			refVolt = (unsigned long long)tbl->row[j].volt;
			newVolt = (unsigned long long)
					upower_tbl_ref[i].row[j].volt;
			temp1 = (refPower * newVolt * newVolt);
			temp2 = (refVolt * refVolt);
#if defined(__LP64__) || defined(_LP64)
			newPower = temp1 / temp2;
#else
			newPower = div64_u64(temp1, temp2);
#endif
			upower_tbl_ref[i].row[j].dyn_pwr = newPower;
		}
	}
}

static void upower_get_p_state_lkg(unsigned int bank,
				unsigned int spower_bank_id)
{
	int j, k;
	int degree;
	unsigned int volt;
	unsigned int temp;

	/* get p-state lkg */
	for (j = 0; j < UPOWER_OPP_NUM; j++) {
		volt = (unsigned int)upower_tbl_ref[bank].row[j].volt;
		for (k = 0; k < NR_UPOWER_DEGREE; k++) {
			degree = degree_set[k];
			/* get leakage and transfer mw to uw */
			temp = mt_spower_get_leakage(
					spower_bank_id,
					(volt/100),
					degree);

			upower_tbl_ref[bank].row[j].lkg_pwr[k] =
				(temp * 1000) / 4;
		}
	}
}

static void upower_get_c_state_lkg(unsigned int bank,
		unsigned int spower_bank_id, struct upower_tbl *tbl)
{
	int j, k;
	int degree;
	unsigned int volt;
	unsigned int temp;

	/* get c-state lkg */
	upower_tbl_ref[bank].nr_idle_states = NR_UPOWER_CSTATES;
	volt = UPOWER_C1_VOLT;
	for (j = 0; j < NR_UPOWER_DEGREE; j++) {
		for (k = 0; k < NR_UPOWER_CSTATES; k++) {
			/* if c1 state, query lkg from lkg driver */
			if (k == UPOWER_C1_IDX) {
				degree = degree_set[j];
				/* get leakage and transfer mw to uw */
				temp = mt_spower_get_leakage(
						spower_bank_id,
						(volt/100),
						degree);

				upower_tbl_ref[bank].idle_states[j][k].power =
					(unsigned long)((temp * 1000) / 4);
			} else {
				upower_tbl_ref[bank].idle_states[j][k].power =
						tbl->idle_states[j][k].power;
			}
		}
	}
}

static void upower_update_lkg_pwr(void)
{
	int i;
	struct upower_tbl *tbl;
	int j, k;
	int spower_bank_id;

	for (i = 0; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;

#ifdef EARLY_PORTING_SPOWER
		/* get p-state lkg */
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			for (k = 0; k < NR_UPOWER_DEGREE; k++)
				upower_tbl_ref[i].row[j].lkg_pwr[k] =
							tbl->row[j].lkg_pwr[k];
		}

		/* get c-state lkg */
		for (j = 0; j < NR_UPOWER_DEGREE; j++) {
			for (k = 0; k < NR_UPOWER_CSTATES; k++)
				upower_tbl_ref[i].idle_states[j][k].power =
						tbl->idle_states[j][k].power;
		}
#else
		spower_bank_id = upower_bank_to_spower_bank(i);

		/* wrong bank or LL L CLUSTER set default lkg_pwr*/
		if (spower_bank_id == -1) {

			for (j = 0; j < UPOWER_OPP_NUM; j++) {
				for (k = 0; k < NR_UPOWER_DEGREE; k++)
					upower_tbl_ref[i].row[j].lkg_pwr[k] =
							tbl->row[j].lkg_pwr[k];
			}
			continue;
		}

		upower_get_p_state_lkg(i, spower_bank_id);

		upower_get_c_state_lkg(i, spower_bank_id, tbl);
#endif
	}
}

static void upower_init_cap(void)
{
	int i, j;
	struct upower_tbl *tbl;

	for (i = 0; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;
		for (j = 0; j < UPOWER_OPP_NUM; j++)
			upower_tbl_ref[i].row[j].cap = tbl->row[j].cap;
	}
}

static void upower_init_rownum(void)
{
	int i;
	struct upower_tbl *tbl;

	for (i = 0; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;
		upower_tbl_ref[i].row_num = tbl->row_num;
	}
}

static unsigned int svs_is_enabled(void)
{
#ifndef EARLY_PORTING_SVS
	return mt_svs_is_enabled();
#else
	return 0;
#endif
}

static void upower_wait_for_svs_volt_done(void)
{
#ifndef SVS_NOT_SET_VOLT
	unsigned char svs_volt_not_ready = 0;
	int i;

	udelay(100);
	while (1) {
		svs_volt_not_ready = 0;
		for (i = 0; i < NR_UPOWER_BANK; i++) {
			if (upower_tbl_ref[i].row[UPOWER_OPP_NUM - 1].volt == 0)
				svs_volt_not_ready = 1;
		}
		if (!svs_volt_not_ready)
			break;
		/* if svs volt not ready, wait 100us */
		upower_debug("wait for svs update\n");
		udelay(100);
	}
#endif
}

#ifdef UPOWER_NUM_LARGER
static void upower_wait_for_SVS_volt_done_upn_larger(void)
{
	unsigned char svs_volt_not_ready = 0;
	int i;

	/* ensure upower bank num does not larger than svs det num */
	iter =
	(int)NR_SVS_DET < (int)NR_UPOWER_BANK ? NR_SVS_DET:NR_UPOWER_BANK;
	udelay(100);
	while (1) {
		svs_volt_not_ready = 0;
		for (i = 0; i < iter; i++) {
			upower_debug("tbl_ref = %d iter %d\n",
				upower_tbl_ref[i].row[UPOWER_OPP_NUM - 1].volt,
				iter);
			if (upower_tbl_ref[i].row[UPOWER_OPP_NUM - 1].volt == 0)
				svs_volt_not_ready = 1;
		}
		if (!svs_volt_not_ready)
			break;
		/* if svs volt not ready, wait 100us */
		udelay(100);
	}
}
#endif

static void upower_init_lkgidx(void)
{
	int i;

	for (i = 0; i < NR_UPOWER_BANK; i++)
		upower_tbl_ref[i].lkg_idx = DEFAULT_LKG_IDX;
}

static void upower_init_volt(void)
{
	int i, j;
	struct upower_tbl *tbl;

	for (i = 0; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;
		for (j = 0; j < UPOWER_OPP_NUM; j++)
			upower_tbl_ref[i].row[j].volt = tbl->row[j].volt;
	}
}
#ifdef UPOWER_NUM_LARGER
static void confirm_volt(void)
{
	int i, j;
	struct upower_tbl *tbl;

	for (i = iter; i < NR_UPOWER_BANK; i++) {
		tbl = upower_tbl_infos[i].p_upower_tbl;
		for (j = 0; j < UPOWER_OPP_NUM; j++)
			upower_tbl_ref[i].row[j].volt = tbl->row[j].volt;
	}

}
#endif

static int upower_update_tbl_ref(void)
{
	int i;
	int ret = 0;

	/* To disable upower, do not update upower ptr*/
	if (!upower_enable) {
		upower_error("upower is disabled\n");
		return 0;
	}

#ifdef UPOWER_PROFILE_API_TIME
	upower_get_start_time_us(UPDATE_TBL_PTR);
#endif

	new_p_tbl_infos = kzalloc(sizeof(*new_p_tbl_infos)
				* NR_UPOWER_BANK, GFP_KERNEL);
	if (!new_p_tbl_infos) {
		upower_error("Out of mem to create new_p_tbl_infos\n");
		return -ENOMEM;
	}

	/* upower_tbl_ref is the ptr points to table in sram */
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		new_p_tbl_infos[i].p_upower_tbl = &upower_tbl_ref[i];
		new_p_tbl_infos[i].name = upower_tbl_infos[i].name;
		/* upower_debug("new_p_tbl_infos[%d].name = %s\n",
		 *  i, new_p_tbl_infos[i].name);
		 */
	}

#ifdef UPOWER_RCU_LOCK
	rcu_assign_pointer(p_upower_tbl_infos, new_p_tbl_infos);
	/* synchronize_rcu();*/
#else
	p_upower_tbl_infos = new_p_tbl_infos;
#endif

#ifdef UPOWER_PROFILE_API_TIME
	upower_get_diff_time_us(UPDATE_TBL_PTR);
	print_diff_results(UPDATE_TBL_PTR);
#endif

	return ret;
}

#ifdef UPOWER_USE_QOS_IPI
#if UPOWER_ENABLE_TINYSYS_SSPM
void upower_send_data_ipi(phys_addr_t phy_addr, unsigned long long size)
{
	struct qos_ipi_data qos_d;

	qos_d.cmd = QOS_IPI_UPOWER_DATA_TRANSFER;
	qos_d.u.upower_data.arg[0] = phy_addr;
	qos_d.u.upower_data.arg[1] = size;
	qos_ipi_to_sspm_command(&qos_d, 3);
}

void upower_dump_data_ipi(void)
{
	struct qos_ipi_data qos_d;

	qos_d.cmd = QOS_IPI_UPOWER_DUMP_TABLE;
	qos_ipi_to_sspm_command(&qos_d, 1);
}
#endif
#endif

void upower_get_tbl_ref(void)
{
#if UPOWER_ENABLE_TINYSYS_SSPM
	int i;
	unsigned char *ptr;
#endif

#ifdef UPOWER_NOT_READY
	return 0;
#endif
	/* get raw upower table and target upower table location */
	get_original_table();

#if UPOWER_ENABLE_TINYSYS_SSPM
	/* get sspm reserved mem */
	upower_data_phy_addr = sspm_reserve_mem_get_phys(UPD_MEM_ID);
	upower_data_virt_addr = sspm_reserve_mem_get_virt(UPD_MEM_ID);
	upower_data_size = sspm_reserve_mem_get_size(UPD_MEM_ID);

	upower_debug("phy_addr = 0x%llx, virt_addr=0x%llx\n",
				(unsigned long long)upower_data_phy_addr,
				(unsigned long long)upower_data_virt_addr);

	/* clear */
	ptr = (unsigned char *)(uintptr_t)upower_data_virt_addr;
	for (i = 0; i < upower_data_size; i++)
		ptr[i] = 0x0;

	upower_tbl_ref = (struct upower_tbl *)(uintptr_t)upower_data_virt_addr;

#ifdef UPOWER_USE_QOS_IPI
	upower_send_data_ipi(upower_data_phy_addr, upower_data_size);
#else
	/* send sspm reserved mem into sspm through svs's ipi (need fix) */
	mt_svs_send_upower_table_ref(upower_data_phy_addr, upower_data_size);
#endif
#endif
	/* upower_tbl_ref is assigned in get_original_table() if no sspm */
	upower_debug("upower tbl orig location([0](%p)= %p\n",
			upower_tbl_infos, upower_tbl_infos[0].p_upower_tbl);
	upower_debug("upower tbl new location([0](%p)\n", upower_tbl_ref);
	is_upower_org_tbl_ready = true;
}

#ifdef UPOWER_PROFILE_API_TIME
static void profile_api(void)
{
	int i, j;

	upower_debug("----profile upower_get_power()----\n");
	/* do 56*2 times */
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			upower_get_power(i, j, UPOWER_DYN);
			upower_get_power(i, j, UPOWER_LKG);
		}
	}
	upower_debug("----profile upower_update_tbl_ref()----\n");
	for (i = 0; i < 10; i++)
		upower_update_tbl_ref();
}
#endif

static int upower_debug_proc_show(struct seq_file *m, void *v)
{

	struct upower_tbl_info **addr_ptr_tbl_info;
	struct upower_tbl_info *ptr_tbl_info;
	struct upower_tbl *ptr_tbl;
	int i, j;

	/* get addr of ptr which points to upower_tbl_infos[] */
	addr_ptr_tbl_info = upower_get_tbl();
	/* get ptr which points to upower_tbl_infos[] */
	ptr_tbl_info = *addr_ptr_tbl_info;

	seq_printf(m,
	"ptr_tbl_info --> %p --> tbl %p (p_upower_tbl_infos --> %p)\n",
	ptr_tbl_info, ptr_tbl_info[0].p_upower_tbl, p_upower_tbl_infos);

	/* print all the tables that record in upower_tbl_infos[]*/
	for (i = 0; i < NR_UPOWER_BANK; i++) {
		seq_printf(m, "%s\n", upower_tbl_infos[i].name);
		ptr_tbl = ptr_tbl_info[i].p_upower_tbl;
		for (j = 0; j < UPOWER_OPP_NUM; j++) {
			seq_printf(m, " cap = %llu, volt = %u, dyn = %u,",
					ptr_tbl->row[j].cap,
					ptr_tbl->row[j].volt,
					ptr_tbl->row[j].dyn_pwr);
			seq_printf(m, " lkg = {%u, %u, %u, %u, %u, %u}\n",
					ptr_tbl->row[j].lkg_pwr[0],
					ptr_tbl->row[j].lkg_pwr[1],
					ptr_tbl->row[j].lkg_pwr[2],
					ptr_tbl->row[j].lkg_pwr[3],
					ptr_tbl->row[j].lkg_pwr[4],
					ptr_tbl->row[j].lkg_pwr[5]);
		}
		seq_printf(m, " lkg_idx, num_row: %d, %d\n\n",
					ptr_tbl->lkg_idx, ptr_tbl->row_num);
	}

#ifdef UPOWER_USE_QOS_IPI
#if UPOWER_ENABLE_TINYSYS_SSPM
	upower_dump_data_ipi();
#endif
#endif
	return 0;
}

#define PROC_FOPS_RW(name)					\
	static int name ## _proc_open(struct inode *inode,	\
		struct file *file)				\
	{							\
		return single_open(file, name ## _proc_show,	\
			PDE_DATA(inode));			\
	}							\
	static const struct file_operations name ## _proc_fops = {	\
		.owner = THIS_MODULE,				\
		.open  = name ## _proc_open,			\
		.read  = seq_read,				\
		.llseek = seq_lseek,				\
		.release = single_release,			\
		.write = name ## _proc_write,			\
	}

#define PROC_FOPS_RO(name)					\
	static int name ## _proc_open(struct inode *inode,	\
		struct file *file)				\
	{							\
		return single_open(file, name ## _proc_show,	\
			PDE_DATA(inode));			\
	}							\
	static const struct file_operations name ## _proc_fops = {	\
		.owner = THIS_MODULE,				\
		.open = name ## _proc_open,			\
		.read = seq_read,				\
		.llseek = seq_lseek,				\
		.release = single_release,			\
	}

#define PROC_ENTRY(name)	{__stringify(name), &name ## _proc_fops}
/* create fops */
PROC_FOPS_RO(upower_debug);

static int create_procfs(void)
{
	struct proc_dir_entry *upower_dir = NULL;
	int i = 0;

	struct pentry {
		const char *name;
		const struct file_operations *fops;
	};

	struct pentry upower_entries[] = {
		/* {__stringify(name), &name ## _proc_fops} */
		PROC_ENTRY(upower_debug),
	};

	/* To disable upower, do not create procfs*/
	if (!upower_enable)
		return 0;

	/* create proc/upower node */
	upower_dir = proc_mkdir("upower", NULL);
	if (!upower_dir) {
		upower_error("[%s] mkdir /proc/upower failed\n", __func__);
		return -1;
	}

	for (i = 0; i < ARRAY_SIZE(upower_entries); i++) {
		if (!proc_create(upower_entries[i].name,
			0644,
			upower_dir,
			upower_entries[i].fops)) {
		upower_error("[%s]: create /proc/upower/%s failed\n", __func__,
		upower_entries[i].name);
		return -3;
		}
	}
	return 0;
}

static int __init upower_init(void)
{
	if (!is_upower_org_tbl_ready) {
		upower_error("upower original table is not ready\n");
		return -1;
	}
#ifdef UPOWER_UT
	upower_debug("--------- (UT)before tbl ready--------------\n");
	upower_ut();
#endif

	/* init rownum to UPOWER_OPP_NUM*/
	upower_init_rownum();

	upower_init_cap();

	/* apply orig volt and lkgidx, if svs is not enabled*/
	if (!svs_is_enabled()) {
		upower_debug("svs is not enabled\n");
		upower_init_lkgidx();
		upower_init_volt();
	} else {
#ifdef UPOWER_NUM_LARGER
		upower_wait_for_SVS_volt_done_upn_larger();
		confirm_volt();
#endif
		upower_wait_for_svs_volt_done();
	}
	upower_update_dyn_pwr();
	upower_update_lkg_pwr();
	upower_update_tbl_ref();

#ifdef UPOWER_UT
	upower_debug("--------- (UT)tbl ready--------------\n");
	upower_ut();
#endif

#ifdef UPOWER_PROFILE_API_TIME
	profile_api();
#endif

	create_procfs();

	/* print_tbl(); */
	return 0;
}
#ifdef __KERNEL__
//subsys_initcall(upower_get_tbl_ref);
late_initcall(upower_init);
#endif
MODULE_DESCRIPTION("MediaTek Unified Power Driver v1.0");
MODULE_LICENSE("GPL");
