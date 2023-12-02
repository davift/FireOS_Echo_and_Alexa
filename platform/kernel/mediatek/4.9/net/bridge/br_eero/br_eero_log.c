/*
 * Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/debugfs.h>

#include "br_eero.h"

#define EERO_BR_LOG_BUFF_MASK (br_eero_log_buff_len - 1)

static const int br_eero_log_buff_len = EERO_BR_LOG_BUF_LEN;

static char *br_eero_log_char_addr(struct br_eero_file_log *file_log,
				   size_t idx)
{
	return &file_log->log_buff[idx & EERO_BR_LOG_BUFF_MASK];
}

static void br_eero_emit_log_char(struct br_eero_file_log *file_log, char c)
{
	char *char_addr;

	char_addr = br_eero_log_char_addr(file_log, file_log->log_end);
	*char_addr = c;
	file_log->log_end++;

	if (file_log->log_end - file_log->log_start > br_eero_log_buff_len)
		file_log->log_start = file_log->log_end - br_eero_log_buff_len;
}

__printf(2, 3) static int br_eero_fdebug_log(struct br_eero_file_log *file_log,
					     const char *fmt, ...)
{
	va_list args;
	static char debug_log_buf[256];
	char *p;

	if (!file_log)
		return 0;

	spin_lock_bh(&file_log->lock);
	va_start(args, fmt);
	vscnprintf(debug_log_buf, sizeof(debug_log_buf), fmt, args);
	va_end(args);

	for (p = debug_log_buf; *p != 0; p++)
		br_eero_emit_log_char(file_log, *p);

	spin_unlock_bh(&file_log->lock);

	wake_up(&file_log->queue_wait);

	return 0;
}

void br_eero_debug_log(struct br_eero_br *eero_br, const char *fmt, ...)
{
	va_list args;
	char tmp_log_buf[256];
	unsigned long rem_nsec;
	u64 ts = local_clock();

	if (!eero_br)
		return;

	rem_nsec = do_div(ts, 1000000000);

	va_start(args, fmt);
	vscnprintf(tmp_log_buf, sizeof(tmp_log_buf), fmt, args);
	br_eero_fdebug_log(eero_br->file_log, "[%5lu.%06lu] %s",
			   (unsigned long)ts, rem_nsec / 1000, tmp_log_buf);
	va_end(args);
}

static int br_eero_log_open(struct inode *inode, struct file *file)
{
	nonseekable_open(inode, file);
	file->private_data = inode->i_private;
	return 0;
}

static bool br_eero_log_empty(struct br_eero_file_log *file_log)
{
	return !(file_log->log_start - file_log->log_end);
}

static ssize_t br_eero_log_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct br_eero_br *eero_br = file->private_data;
	struct br_eero_file_log *file_log = eero_br->file_log;
	int err, i = 0;
	char *char_addr;
	char c;

	if ((file->f_flags & O_NONBLOCK) && br_eero_log_empty(file_log))
		return 0;

	if (!buf)
		return -EINVAL;

	if (count == 0)
		return 0;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	err = wait_event_interruptible(file_log->queue_wait,
				       (!br_eero_log_empty(file_log)));

	if (err)
		return err;

	spin_lock_bh(&file_log->lock);

	while ((!err) && (i < count) &&
	       (file_log->log_start != file_log->log_end)) {
		char_addr =
			br_eero_log_char_addr(file_log, file_log->log_start);
		c = *char_addr;

		file_log->log_start++;

		spin_unlock_bh(&file_log->lock);

		err = __put_user(c, buf);

		spin_lock_bh(&file_log->lock);

		buf++;
		i++;
	}

	spin_unlock_bh(&file_log->lock);

	if (!err)
		return i;

	return err;
}

static int br_eero_log_release(struct inode *inode, struct file *file)
{
	return 0;
}

static unsigned int br_eero_log_poll(struct file *file, poll_table *wait)
{
	struct br_eero_br *eero_br = file->private_data;
	struct br_eero_file_log *file_log = eero_br->file_log;

	poll_wait(file, &file_log->queue_wait, wait);

	if (!br_eero_log_empty(file_log))
		return POLLIN | POLLRDNORM;

	return 0;
}

struct br_eero_debuginfo br_eero_debuginfo_eero_br_log = {
	.attr = { .name = "eero_br_logs",
		.mode = S_IRUGO, },
	.fops = { .owner = THIS_MODULE,
		.open           = br_eero_log_open,
		.release        = br_eero_log_release,
		.read           = br_eero_log_read,
		.poll           = br_eero_log_poll,
		.llseek         = no_llseek,
	}
};

int br_eero_debug_log_setup(struct dentry *dir, struct br_eero_br *eero_br)
{
	struct dentry *file;

	if (!dir)
		goto err;

	if (!eero_br->file_log) {
		eero_br->file_log =
			kzalloc(sizeof(*eero_br->file_log), GFP_ATOMIC);
		spin_lock_init(&eero_br->file_log->lock);
		init_waitqueue_head(&eero_br->file_log->queue_wait);
	}

	if (!eero_br->file_log)
		goto err;

	file = debugfs_create_file(br_eero_debuginfo_eero_br_log.attr.name,
				   br_eero_debuginfo_eero_br_log.attr.mode, dir,
				   eero_br,
				   &br_eero_debuginfo_eero_br_log.fops);
	if (!file)
		goto err;

	return 0;

err:
	kfree(eero_br->file_log);
	return -ENOMEM;
}

void br_eero_debug_log_cleanup(struct br_eero_br *eero_br)
{
	kfree(eero_br->file_log);
	eero_br->file_log = NULL;
}
