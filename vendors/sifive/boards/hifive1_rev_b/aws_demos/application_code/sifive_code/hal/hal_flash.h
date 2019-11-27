/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_HAL_FLASH_
#define H_HAL_FLASH_

#ifdef __cplusplus
extern "C" {
#endif

#include <inttypes.h>

struct hal_flash {
    uint32_t hf_base_addr;
    uint32_t hf_size;
    uint32_t hf_sector_size;
    int hf_sector_cnt;
    int hf_align;       /* Alignment requirement. 1 if unrestricted. */
    uint8_t hf_erased_val;
};

int
fe310_flash_read(uint32_t address, void *dst, uint32_t num_bytes);

int __attribute((section(".data.fe310_flash_transmit")))
fe310_flash_transmit(uint8_t out_byte);

int __attribute((section(".data.fe310_flash_fifo_put")))
fe310_flash_fifo_put(uint8_t out_byte);

int __attribute((section(".data.fe310_flash_fifo_write")))
fe310_flash_fifo_write(const uint8_t *ptr, int count);

int __attribute((section(".data.fe310_flash_wait_till_ready")))
fe310_flash_wait_till_ready(void);

int __attribute((section(".data.fe310_flash_write_enable")))
fe310_flash_write_enable(void);

int  __attribute((section(".data.fe310_flash_write_page"))) __attribute((noinline))
fe310_flash_write_page(uint32_t address, const void *src, uint32_t num_bytes);

int fe310_flash_write(uint32_t address, const void *src, uint32_t num_bytes);

int __attribute((section(".data.fe310_flash_erase_sector"))) __attribute((noinline))
fe310_flash_erase_sector(uint32_t sector_address);

int
fe310_flash_sector_info(int idx, uint32_t *address, uint32_t *sz);

int
fe310_flash_init();

#ifdef __cplusplus
}
#endif

#endif /* H_HAL_FLASH_ */

