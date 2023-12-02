#
# Copyright (c) 2022 Amazon.com, Inc. or its affiliates.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version
# 2 of the License, or (at your option) any later version.
#
# Makefile for the eero bridge obj which will be part of bridge
#
obj-$(CONFIG_EERO_BR) += eero_br.o

eero_br-y += br_eero.o br_eero_segment.o br_eero_debugfs.o br_eero_sysctl.o \
            br_eero_segment_id.o br_eero_roam.o br_eero_sysfs.o\
            br_eero_netlink.o

eero_br-${CONFIG_EERO_BR_LOG_FILE} += br_eero_log.o
