/******************************************************************************
 * Copyright 2020 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <firmament.h>

#include "diskio.h"
#include "drv_sdio.h"
#include "sdio.h"

#define SECTOR_SIZE 512

#if (SDIO_POLLING_IRQ_PROTECT) && (SDIO_TRANSFER_MODE == SD_POLLING_MODE)
#define SDIO_XFER_IRQ_PROTECT 1
#else
#define SDIO_XFER_IRQ_PROTECT 0
#endif

static struct rt_mutex _sd_lock;
static rt_bool_t _sd_initialized = RT_FALSE;
static rt_bool_t _sd_lock_initialized = RT_FALSE;

static rt_err_t _sd_lock_take(void)
{
    if (_sd_lock_initialized != RT_TRUE) {
        return RT_EOK;
    }

    return rt_mutex_take(&_sd_lock, RT_WAITING_FOREVER);
}

static void _sd_lock_release(void)
{
    if (_sd_lock_initialized == RT_TRUE) {
        rt_mutex_release(&_sd_lock);
    }
}

static rt_err_t _sd_ensure_initialized(void)
{
    if (_sd_initialized == RT_TRUE) {
        return RT_EOK;
    }

    if (_sd_lock_take() != RT_EOK) {
        return RT_ERROR;
    }

    if (_sd_initialized == RT_TRUE) {
        _sd_lock_release();
        return RT_EOK;
    }

    if (SD_Init() != SD_OK) {
        _sd_lock_release();
        return RT_ERROR;
    }

    _sd_initialized = RT_TRUE;
    _sd_lock_release();
    return RT_EOK;
}

rt_err_t _sd_dev_init(rt_device_t dev)
{
    (void)dev;
    return _sd_ensure_initialized();
}

rt_err_t _sd_dev_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

rt_err_t _sd_dev_close(rt_device_t dev)
{
    return RT_EOK;
}

rt_size_t _sd_dev_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    uint32_t sector = pos;
    rt_size_t r_count = 0;
    uint8_t* p = (uint8_t*)buffer;

    (void)dev;

    if (buffer == RT_NULL || size == 0 || _sd_ensure_initialized() != RT_EOK) {
        return 0;
    }

    if (_sd_lock_take() != RT_EOK) {
        return 0;
    }

    while (r_count < size) {
        rt_size_t chunk = size - r_count;
        if (chunk > SDIO_MAX_SECTORS_PER_XFER) {
            chunk = SDIO_MAX_SECTORS_PER_XFER;
        }

#if SDIO_XFER_IRQ_PROTECT
        rt_base_t level = rt_hw_interrupt_disable();
        uint8_t ret = SD_ReadDisk(p, sector, (uint8_t)chunk);
        rt_hw_interrupt_enable(level);
#else
        uint8_t ret = SD_ReadDisk(p, sector, (uint8_t)chunk);
#endif

        if (ret != SD_OK) {
            rt_kprintf("sd read fail: sector=%lu chunk=%u\n", (unsigned long)sector, (unsigned)chunk);
            break;
        }

        p += chunk * SECTOR_SIZE;
        sector += chunk;
        r_count += chunk;
    }

    _sd_lock_release();

    return r_count;
}

rt_size_t _sd_dev_write(rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    uint32_t sector = pos;
    rt_size_t w_count = 0;
    const uint8_t* p = (const uint8_t*)buffer;

    (void)dev;

    if (buffer == RT_NULL || size == 0 || _sd_ensure_initialized() != RT_EOK) {
        return 0;
    }

    if (_sd_lock_take() != RT_EOK) {
        return 0;
    }

    while (w_count < size) {
        rt_size_t chunk = size - w_count;
        if (chunk > SDIO_MAX_SECTORS_PER_XFER) {
            chunk = SDIO_MAX_SECTORS_PER_XFER;
        }

#if SDIO_XFER_IRQ_PROTECT
        rt_base_t level = rt_hw_interrupt_disable();
        uint8_t ret = SD_WriteDisk((uint8_t*)p, sector, (uint8_t)chunk);
        rt_hw_interrupt_enable(level);
#else
        uint8_t ret = SD_WriteDisk((uint8_t*)p, sector, (uint8_t)chunk);
#endif

        if (ret != SD_OK) {
            rt_kprintf("sd write fail: sector=%lu chunk=%u\n", (unsigned long)sector, (unsigned)chunk);
            break;
        }

        p += chunk * SECTOR_SIZE;
        sector += chunk;
        w_count += chunk;
    }

    _sd_lock_release();

    return w_count;
}

rt_err_t _sd_dev_control(rt_device_t dev, int cmd, void* args)
{
    (void)dev;

    switch (cmd) {
    case RT_DEVICE_CTRL_BLK_GETGEOME: {
        struct rt_device_blk_geometry geometry;
        SD_CardInfo SDCardInfo;

        if (args == RT_NULL) {
            return RT_EINVAL;
        }

        if (_sd_ensure_initialized() != RT_EOK) {
            return RT_ERROR;
        }

        if (_sd_lock_take() != RT_EOK) {
            return RT_ERROR;
        }

        geometry.bytes_per_sector = SECTOR_SIZE;
        geometry.block_size = SECTOR_SIZE;

        if (SD_GetCardInfo(&SDCardInfo) == SD_OK) {
            if (SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD) {
                geometry.sector_count = (SDCardInfo.SD_csd.DeviceSize + 1) * 1024;
            } else {
                geometry.sector_count = SDCardInfo.CardCapacity / SECTOR_SIZE;
            }

            *(struct rt_device_blk_geometry*)args = geometry;
        } else {
            _sd_lock_release();
            return RT_ERROR;
        }

        _sd_lock_release();
    } break;

    case RT_DEVICE_CTRL_BLK_SYNC: {
        uint32_t timeout = SDIO_SYNC_TIMEOUT;

        if (_sd_ensure_initialized() != RT_EOK) {
            return RT_ERROR;
        }

        if (_sd_lock_take() != RT_EOK) {
            return RT_ERROR;
        }

        while (SD_GetState() == SD_CARD_PROGRAMMING) {
            if (timeout-- == 0) {
                _sd_lock_release();
                return RT_ETIMEOUT;
            }
        }

        _sd_lock_release();
    } break;

    case RT_DEVICE_CTRL_BLK_ERASE: {
        return RT_ERROR; /* need to support */
    } break;

    default: {
        rt_kprintf("unknown sd control cmd:%d\n", cmd);
        return RT_EINVAL;
    } break;
    }

    return RT_EOK;
}

rt_err_t drv_sdio_init(const char* device_name)
{
    if (device_name == RT_NULL) {
        return RT_EINVAL;
    }
    static struct rt_device _sd_dev;

    if (_sd_lock_initialized != RT_TRUE) {
        RT_TRY(rt_mutex_init(&_sd_lock, "sdlock", RT_IPC_FLAG_PRIO));
        _sd_lock_initialized = RT_TRUE;
    }

    rt_memset(&_sd_dev, 0, sizeof(_sd_dev));

    _sd_dev.type = RT_Device_Class_Block;
    _sd_dev.init = _sd_dev_init;
    _sd_dev.open = _sd_dev_open;
    _sd_dev.close = _sd_dev_close;
    _sd_dev.read = _sd_dev_read;
    _sd_dev.write = _sd_dev_write;
    _sd_dev.control = _sd_dev_control;

    /* register to device manager */
    return rt_device_register(&_sd_dev, device_name, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE);
}
