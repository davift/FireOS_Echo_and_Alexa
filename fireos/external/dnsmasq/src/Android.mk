LOCAL_PATH := $(call my-dir)

#########################

include $(CLEAR_VARS)
LOCAL_SRC_FILES :=  bpf.c cache.c dbus.c dhcp.c dnsmasq.c forward.c helper.c lease.c log.c \
                    netlink.c network.c option.c rfc1035.c rfc2131.c tftp.c util.c

LOCAL_MODULE := dnsmasq

LOCAL_C_INCLUDES := external/dnsmasq/src

LOCAL_CFLAGS := -O2 -g -W -Wall -D__ANDROID__ -DIPV6 -DNO_TFTP -DNO_SCRIPT -D_BSD_SOURCE
LOCAL_SYSTEM_SHARED_LIBRARIES := libc
LOCAL_SHARED_LIBRARIES := libcutils liblog

## fosmod_wifi_bug_fix begin
ifeq ($(TARGET_BUILD_VARIANT),user)
  LOCAL_CFLAGS += -DDHCP_HIDE_HOSTNAME
endif
## fosmod_wifi_bug_fix end

include $(BUILD_EXECUTABLE)
