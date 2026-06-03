/******************************************************************************
 * Copyright 2022 The Firmament Authors. All Rights Reserved.
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

#include "FMS.h"
#include "driver/rgb_led/aw2023.h"
#include "led.h"
#include "module/workqueue/workqueue_manager.h"

// The red and blue leds are swapped on this board.

static rt_device_t rgb_led_dev;

void vehicle_status_change_cb(uint8_t status)
{
    switch (status) {
    case VehicleStatus_Disarm:
        rgb_led_set_color(AW2023_LED_RED);
        break;
    case VehicleStatus_Standby:
        rgb_led_set_color(AW2023_LED_GREEN);
        break;
    case VehicleStatus_Arm:
        rgb_led_set_color(AW2023_LED_GREEN);
        break;
    default:
        rgb_led_set_color(AW2023_LED_BLUE);
        break;
    }
}

void vehicle_state_change_cb(uint8_t mode)
{
    if (mode == VehicleState_None) {
        /* unknown mode */
        rgb_led_set_color(AW2023_LED_BLUE);
    }
}

fmt_err_t rgb_led_set_color(uint32_t color)
{
    if (rgb_led_dev == NULL) {
        return FMT_EEMPTY;
    }

    if (rt_device_control(rgb_led_dev, AW2023_CMD_SET_COLOR, (void*)color) != RT_EOK) {
        return FMT_ERROR;
    }

    return FMT_EOK;
}

fmt_err_t rgb_led_set_bright(uint32_t bright)
{
    if (rgb_led_dev == NULL) {
        return FMT_EEMPTY;
    }

    if (rt_device_control(rgb_led_dev, AW2023_CMD_SET_BRIGHT, (void*)bright) != RT_EOK) {
        return FMT_ERROR;
    }

    return FMT_EOK;
}

fmt_err_t rgb_led_set_mode(uint8_t mode)
{
    if (rgb_led_dev == NULL) {
        return FMT_EEMPTY;
    }

    if (mode == RGB_LED_MANUAL_MODE) {
        if (rt_device_control(rgb_led_dev, AW2023_CMD_SET_MANUAL_MODE, NULL) != RT_EOK) {
            return FMT_ERROR;
        }
    } else if (mode == RGB_LED_PATERN_MODE) {
        if (rt_device_control(rgb_led_dev, AW2023_CMD_SET_PATERN_MODE, NULL) != RT_EOK) {
            return FMT_ERROR;
        }
    } else {
        return FMT_EINVAL;
    }

    return FMT_EOK;
}

fmt_err_t led_control_init(void)
{
    rgb_led_dev = rt_device_find("aw2023");

    RT_CHECK(rt_device_open(rgb_led_dev, RT_DEVICE_OFLAG_RDWR));
    /* set rgb led initial color */
    FMT_TRY(rgb_led_set_mode(RGB_LED_PATERN_MODE));
    FMT_TRY(rgb_led_set_color(AW2023_LED_RED));
    FMT_TRY(rgb_led_set_bright(0xFF));

    // No need to create a task to run the LEDs, 
    // because the driver chip will automatically run the pattern.

    return FMT_EOK;
}
