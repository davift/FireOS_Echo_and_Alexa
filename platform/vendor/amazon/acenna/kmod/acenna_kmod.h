/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/**
 * @file acenna_kmod.h
 * @brief Kernel Module header for ACE NNA
 */
#ifndef ACENNAMOD_H_
#define ACENNAMOD_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define DRIVER_VERSION     "1.8.5"
#define DRIVER_NAME        "acenna"
#define DEVICE_NAME_FORMAT "acenna%d"
#define DEVICE_MAX_NUM      256

/**
 * sync_mode(synchronous mode) value
 */
#define SYNC_MODE_INVALID (0x00)
#define SYNC_MODE_NONCACHED (0x01)
#define SYNC_MODE_WRITECOMBINE (0x02)
#define SYNC_MODE_DMACOHERENT (0x03)
#define SYNC_MODE_MASK (0x03)
#define SYNC_MODE_MIN (0x01)
#define SYNC_MODE_MAX (0x03)
#define SYNC_ALWAYS (0x04)

#define METRICS_BUFFER_SIZE     (4096)
#define METRICS_CLASS_ID        ("NNA")

/**
 * @brief Struct used to make IOCTL calls for Memory allocation and
 * de-allocation requests
 * @ingroup ACENNA_KERNEL_IOCTL_MSG
 */
typedef struct stMemAreaRequest
{
    /**
     * Identifier of Client making the request. Valid values are between 0 and
     * %NNA_MAX_CLIENTS%
     */
    int clientid;
    /**
     * memid for request. Valid values are between 0 and
     * %NNA_MAX_ALLOCATED_MEMAREAS_PER_CLIENT%
     */
    int memid;
    /**
     * Size of allocation requested. Maximum allowed NNA_MAX_MEMAREA_SIZE
     */
    int size;
    /**
     * physical address of requested buffers. Only valid if 0 is returned.
     *
     */
    uint64_t phy_addr;
    /**
     * Cache synchronization mode override.
     * 0 - Let flags used in open() decide (default)
     * 1 - Noncached
     * 2 - Write Combined
     * 3 - DMA Coherent
     *
     */
    int sync_mode;
} stMemAreaRequest;

typedef struct stMemAreaSync
{
    /**
     * Identifier of Client making the request. Valid values are between 0 and
     * %NNA_MAX_CLIENTS%
     */
    int clientid;
    /**
     * memid for request. Valid values are between 0 and
     * %NNA_MAX_ALLOCATED_MEMAREAS_PER_CLIENT%
     */
    int memid;
    /**
     * Size of area to by synced
     */
    int size;
    /**
     * Offset of area to be synced from buffer start
     *
     */
    int offset;
    /**
     * Direction of DMA data flow
     * 0 - Data moves bidirectional (default)
     * 1 - Data moves from main memory to the device
     * 2 - Data moves from the device to main memory
     * 3 - None (Debug only)
     *
     */
    int data_direction;
} stMemAreaSync;

typedef struct NnaMetricsRequest
{
    char buffer[METRICS_BUFFER_SIZE];
    int reserved;
} NnaMetricsRequest_t;

/**
 * @brief Struct used to make IOCTL calls for ACE NNA register access requests
 * @ingroup ACENNA_KERNEL_IOCTL_MSG
 */
typedef struct regAccessRequest_s
{
    int32_t offset;     /** Register offset to read/write */
    uint32_t value;     /** For write requests, value to write */
} regAccessRequest_t;

/**
 * @brief Struct used to make IOCTL call for timestamps from kernel
 * @ingroup ACENNA_KERNEL_IOCTL_MSG
 */
typedef struct timeval_exch
{
    __aligned_u64 tv_sec;   /** Seconds */
    __aligned_u64 tv_usec;  /** Micro seconds */
} timeval_exch;

typedef enum
{
    NNA_POWER_OFF = 0,
    NNA_POWER_ON = 1
} NnaPowerState_t;

/**
 *  IOCTL Magic for Memory allocation requests
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_ALLOC         _IOW('a', 1, stMemAreaRequest)
/**
 *  IOCTL Magic for Memory de-allocation requests
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_DEALLOC       _IOW('a', 2, stMemAreaRequest)
/**
 *  IOCTL Magic to clear interrupt flags in kernel.
 * \deprecated Register writes are sniffed to implicitly clear interrupt flags.
 * No explicit userspace command needed.
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_TRIGGER       _IO('a', 3)
/**
 *  IOCTL Magic to request last interrupt timestamp
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_HW_INT_TIME   _IOR('a', 4, timeval_exch)
/**
 *  IOCTL Magic for Register Write requests
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_REG_WRITE     _IOW('a', 5, regAccessRequest_t)
/**
 *  IOCTL Magic for Register Read requests
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_REG_READ      _IOR('a', 6, regAccessRequest_t)
/**
 *  IOCTL Magic to clear POLL Event flag
 * @ingroup ACENNA_KERNEL_IOCTL_MAGIC
 */
#define NNA_IOCTL_CLEAR_POLL    _IO('a', 7)

#define NNA_IOCTL_SYNC_FOR_DEVICE _IOW('a', 8, stMemAreaSync)

#define NNA_IOCTL_SYNC_FOR_CPU _IOW('a', 9, stMemAreaSync)

#define NNA_IOCTL_METRICS_REQUEST _IOW('a', 10, NnaMetricsRequest_t)

#define NNA_IOCTL_SET_POWER_STATE _IOW('a', 11, NnaPowerState_t)

#define NNA_IOCTL_GET_POWER_STATE _IOR('a', 12, NnaPowerState_t)

#define NNA_IOCTL_GET_DEFAULT_POWER_STATE _IOR('a', 13, NnaPowerState_t)

/**
 *  Max number of clients that can be connected to NNA driver
 */
#define NNA_MAX_CLIENTS 16

/**
 *  Max number of CMA areas that a client can ask from NNA driver
 */
#define NNA_MAX_ALLOCATED_MEMAREAS_PER_CLIENT 128

/**
 *  NNA Max mem area size
 */
#define NNA_MAX_MEMAREA_SIZE (128*1024*1024)

#endif //   ACENNAMOD_H_
