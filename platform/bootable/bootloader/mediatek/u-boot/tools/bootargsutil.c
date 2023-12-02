// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2014
 * DENX Software Engineering
 * Heiko Schocher <hs@denx.de>
 *
 * fit_info: print the offset and the len of a property from
 *	     node in a fit file.
 *
 * Based on:
 * (C) Copyright 2008 Semihalf
 *
 * (C) Copyright 2000-2004
 * DENX Software Engineering
 * Wolfgang Denk, wd@denx.de
 *
 * Updated-by: Prafulla Wadaskar <prafulla@marvell.com>
 *		FIT image specific code abstracted from mkimage.c
 *		some functions added to address abstraction
 *
 * All rights reserved.
 */

#include "mkimage.h"
#include "fit_common.h"
#include <image.h>
#include <u-boot/crc.h>

void usage(char *cmdname)
{
	fprintf(stderr, "Usage: %s -f fit file [-a  value]\n"
			 "          -f ==> set fit file which is used'\n"
			 "          -a ==> append value\n",
		cmdname);
	exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
	int ffd = -1;
	struct stat fsbuf;
	void *fit_blob = NULL;
	int len = 0;
	int  nodeoffset = 0;	/* node offset from libfdt */
	const void *nodep = NULL;	/* property node pointer */
	char *fdtfile = NULL;
	char *nodename = NULL;
	char *propertyname = NULL;
        char * appendstr = NULL;
	int size_inc = 0;
	char cmdname[256];
	int c = 0;
	fsbuf.st_size = 0;
	strncpy(cmdname, *argv, sizeof(cmdname) - 1);
	cmdname[sizeof(cmdname) - 1] = '\0';
	while ((c = getopt(argc, argv, "f:a:")) != -1)
		switch (c) {
		case 'f':
			fdtfile = optarg;
			break;
               case 'a':
			appendstr = optarg;
			size_inc = strlen(appendstr)*2;
			break;
		default:
			usage(cmdname);
			break;
		}

	nodename = "/chosen";
	propertyname = "bootargs";

	printf("appendstr %s\n",appendstr);

	if (!fdtfile) {
		fprintf(stderr, "%s: Missing fdt file\n", *argv);
		usage(*argv);
	}
	if (!nodename) {
		fprintf(stderr, "%s: Missing node name\n", *argv);
		usage(*argv);
	}
	if (!propertyname) {
		fprintf(stderr, "%s: Missing property name\n", *argv);
		usage(*argv);
	}
	ffd = mmap_fdt(cmdname, fdtfile, size_inc, &fit_blob, &fsbuf, false);

	if (ffd < 0) {
		printf("Could not open %s\n", fdtfile);
		exit(EXIT_FAILURE);
	}

	nodeoffset = fdt_path_offset(fit_blob, nodename);
	if (nodeoffset < 0) {
		printf("%s not found.", nodename);
		(void) munmap((void *)fit_blob, fsbuf.st_size);
		close(ffd);
		exit(EXIT_FAILURE);
	}
	nodep = fdt_getprop(fit_blob, nodeoffset, propertyname, &len);
	if ((len == 0) || (nodep == NULL)) {
		printf("len == 0 or nodep NULL %s\n", propertyname);
		(void) munmap((void *)fit_blob, fsbuf.st_size);
		close(ffd);
		exit(EXIT_FAILURE);
	}

	printf("NAME: %s\n", fit_get_name(fit_blob, nodeoffset, NULL));
	printf("LEN: %d\n", len);
	printf("OFF: %d\n", (int)(nodep - fit_blob));
	if(*((char *)nodep + len - 1) == 0)
		printf("STR CONTENT: %s\n", nodep);

	if(appendstr) {
		int len1 = strlen(nodep);
		int len2 = strlen(appendstr);
		int newlen = len1 + 1 +  len2 + 1;
		char  *newvalue = malloc(newlen);
		if (!newvalue) {
			printf("malloc newvalue fail\n");
			close(ffd);
			exit(EXIT_FAILURE);
		}
		int ret = 0;
		memset(newvalue ,0, newlen);

		memcpy(newvalue, nodep, len1);
		newvalue[len1] = '  ';
		memcpy(newvalue + len1 + 1, appendstr, len2);
		newvalue[len1 + 1 + len2] = 0;

		ret = fdt_setprop(fit_blob, nodeoffset, propertyname, newvalue, newlen);
		printf("ret: %d APPENDED STR CONTENT: %s\n", ret , newvalue);
		free(newvalue);
	}

	(void) munmap((void *)fit_blob, fsbuf.st_size);
	close(ffd);
	exit(EXIT_SUCCESS);
}
