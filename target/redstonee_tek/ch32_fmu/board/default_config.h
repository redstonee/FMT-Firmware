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

/* This is the default toml config for the target, which is loaded when there is
 * no sysconfig.toml finded. Please do not format this file. */

// clang-format off
__attribute__((section(".slow_data")))
static char default_conf[] = STRING(
target = "RedStoneeTek-CH32_FMU"\n
[console]\n
	[[console.devices]]\n
	type = "serial"\n
	name = "serial0"\n
	baudrate = 115200\n
	auto-switch = true\n
	[[console.devices]]\n
	type = "mavlink"\n
	name = "mav_console"\n
	auto-switch = true\n
[pilot-cmd]\n
    stick-channel = [4,3,1,2]\n
    [pilot-cmd.device]\n
    type = "rc"\n
    name = "rc"\n
    protocol = "sbus"\n
    channel-num = 8\n
    sample-time = 0.05\n
    range = [1000,2000]\n
    [[pilot-cmd.mode]]\n
    mode = 5                    # Position mode\n
    channel = 5\n
    range = [1000,1300]\n
    [[pilot-cmd.mode]]\n
    mode = 4                    # Altitude mode\n
    channel = 5\n
    range = [1400,1600]\n
    [[pilot-cmd.mode]]\n
    mode = 3                    # Stabilize mode\n
    channel = 5\n
    range = [1700,2000]\n
    [[pilot-cmd.command]]\n
    type = 1                    # 1:event | 2:status\n
    cmd = 1002                  # FMS_Cmd_Disarm\n
    channel = 6\n
    range = [1800,2000]\n
[actuator]\n
    [[actuator.devices]]\n
    protocol = "pwm"\n
    name = "main_out"\n
    freq = 400\n
    [[actuator.mappings]]\n
    from = "control_out"\n
    to = "main_out"\n
    chan-map = [[1,2,3,4],[1,2,3,4]]\n
[mavproxy]\n
    [[mavproxy.devices]]\n
    chan = 0\n
    type = "serial"\n
    name = "serial5"\n
    baudrate = 57600\n
    [[mavproxy.devices]]\n
	chan = 0\n
	type = "usb"\n
	name = "usbd0"\n
    auto-switch = true\n
);
// clang-format on
