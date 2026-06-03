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

#ifndef FMT_BSP_H__
#define FMT_BSP_H__

#include <firmament.h>

#ifdef __cplusplus
extern "C" {
#endif

// Board Information
#define TARGET_NAME           "RedStoneeTek-CH32_FMU"

// Internal SRAM memory size[Kbytes]
#define SYSTEM_TOTAL_MEM_SIZE (512 * 1024)

// Internal Free SRAM memory used by kernel (e.g, rt_malloc)
// Put this in the `RAM`
extern int _heap_start;
extern int _heap_end;
#define SYSTEM_FREE_MEM_BEGIN (&_heap_start)
#define SYSTEM_FREE_MEM_END  (&_heap_end)

void rt_hw_board_init(void);
void bsp_early_initialize(void);
void bsp_initialize(void);
void bsp_post_initialize(void);

#ifdef __cplusplus
}
#endif

#endif
