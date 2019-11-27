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

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_spi.h"
#include "hal_flash.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

#define SPI0_CTRL_ADDR	 0x10014000
#define SPI0_REG(offset) (*(volatile uint32_t *)(SPI0_CTRL_ADDR + offset))

#define FE310_FLASH_SECTOR_SZ    4096


#define FLASH_CMD_READ_STATUS_REGISTER 0x05
#define FLASH_CMD_WRITE_ENABLE 0x06
#define FLASH_CMD_PAGE_PROGRAM 0x02
#define FLASH_CMD_SECTOR_ERASE 0x20

#define FLASH_STATUS_BUSY 0x01
#define FLASH_STATUS_WEN  0x02

const struct hal_flash fe310_flash_dev = {
    .hf_base_addr = 0x20000000,
    .hf_size = 512 * 1024,  	 /* XXX read from factory info? */
	.hf_sector_size = FE310_FLASH_SECTOR_SZ,
    .hf_sector_cnt = 128,       /* XXX read from factory info? */
    .hf_align = 1,
    .hf_erased_val = 0xff,
};

int
fe310_flash_read(uint32_t address, void *dst, uint32_t num_bytes)
{
    memcpy(dst, (void *)(address + fe310_flash_dev.hf_base_addr), num_bytes);
    return 0;
}

int __attribute((section(".data.fe310_flash_transmit")))
fe310_flash_transmit(uint8_t out_byte)
{
    int in_byte;

    /* Empty RX FIFO */
    while ((int)SPI0_REG(SPI_REG_RXFIFO) >= 0) {
    }
    SPI0_REG(SPI_REG_TXFIFO) = out_byte;
    do {
         in_byte = (int)SPI0_REG(SPI_REG_RXFIFO);
    } while (in_byte < 0);

    return in_byte;
}

int __attribute((section(".data.fe310_flash_fifo_put")))
fe310_flash_fifo_put(uint8_t out_byte)
{
    int went_out = 0;

    /* Empty RX FIFO */
    for (;;) {
        if ((int)SPI0_REG(SPI_REG_RXFIFO) >= 0) {
            went_out++;
        }
        if ((int)SPI0_REG(SPI_REG_TXFIFO) >= 0) {
            SPI0_REG(SPI_REG_TXFIFO) = out_byte;
            break;
        }
    }

    return went_out;
}

int __attribute((section(".data.fe310_flash_fifo_write")))
fe310_flash_fifo_write(const uint8_t *ptr, int count)
{
    int went_out = 0;

    while (count > 0) {
        if ((int)SPI0_REG(SPI_REG_RXFIFO) >= 0) {
            went_out++;
        }
        if ((int)SPI0_REG(SPI_REG_TXFIFO) >= 0) {
            SPI0_REG(SPI_REG_TXFIFO) = *ptr++;
            count--;
        }
    }

    return went_out;
}

int __attribute((section(".data.fe310_flash_wait_till_ready")))
fe310_flash_wait_till_ready(void)
{
    int status;
    do {
        SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
        fe310_flash_transmit(FLASH_CMD_READ_STATUS_REGISTER);
        /* Wait for ready */
        status = fe310_flash_transmit(0xFF);
        SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
    } while (status & FLASH_STATUS_BUSY);

    return 0;
}

int __attribute((section(".data.fe310_flash_write_enable")))
fe310_flash_write_enable(void)
{
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    fe310_flash_transmit(FLASH_CMD_WRITE_ENABLE);
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;
    return 0;
}

int  __attribute((section(".data.fe310_flash_write_page"))) __attribute((noinline))
fe310_flash_write_page(uint32_t address, const void *src, uint32_t num_bytes)
{
    /* Number of bytes that left controller FIFO */
    int went_out = 0;
    portENTER_CRITICAL();

    /* Disable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 0;
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    SPI0_REG(SPI_REG_FMT) &= ~SPI_FMT_DIR(SPI_DIR_TX);

    fe310_flash_wait_till_ready();
    fe310_flash_write_enable();

    /* Page program */
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;

    /* Writes bytes without waiting for input FIFO */
    went_out += fe310_flash_fifo_put(FLASH_CMD_PAGE_PROGRAM);
    went_out += fe310_flash_fifo_put(address >> 16);
    went_out += fe310_flash_fifo_put(address >> 8);
    went_out += fe310_flash_fifo_put(address);
    went_out += fe310_flash_fifo_write(src, num_bytes);

    /* Wait till input FIFO if filled, all bytes were transmitted */
    while (went_out < num_bytes + 4) {
        if ((int)SPI0_REG(SPI_REG_RXFIFO) >= 0) {
            went_out++;
        }
    }
    /* CS deactivated */
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;

    /* Wait for flash to become ready, before switching to auto mode */
    fe310_flash_wait_till_ready();

    /* Enable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 1;

    /* Now interrupts can be handled with code in flash */
    portEXIT_CRITICAL();
    return 0;
}

int
fe310_flash_write(uint32_t address, const void *src, uint32_t num_bytes)
{
    const int page_size = 256;
    uint32_t page_end;
    uint32_t chunk;
    const bool src_in_flash = (fe310_flash_dev.hf_base_addr <= (uint32_t)src &&
       fe310_flash_dev.hf_base_addr + fe310_flash_dev.hf_size > (uint32_t)src);

    while (num_bytes > 0) {
        page_end = (address + page_size) & ~(page_size - 1);
        if (address + num_bytes < page_end) {
            page_end = address + num_bytes;
        }
        chunk = page_end - address;
        /* If src is from flash, move small chunk to RAM first */
        if (src_in_flash) {
            uint8_t ram_buf[16];
            if (chunk > 16) {
                chunk = 16;
            }
            memcpy(ram_buf, src, chunk);
            if (fe310_flash_write_page(address, ram_buf, chunk) < 0) {
                return -1;
            }
        } else {
            if (fe310_flash_write_page(address, src, chunk) < 0) {
                return -1;
            }
        }
        address += chunk;
        num_bytes -= chunk;
        src += chunk;
    }
    return 0;
}

int __attribute((section(".data.fe310_flash_erase_sector"))) __attribute((noinline))
fe310_flash_erase_sector(uint32_t sector_address)
{
    portENTER_CRITICAL();

    /* Disable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 0;
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    SPI0_REG(SPI_REG_FMT) &= ~SPI_FMT_DIR(SPI_DIR_TX);

    fe310_flash_wait_till_ready();
    fe310_flash_write_enable();

    /* Erase sector */
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_HOLD;
    fe310_flash_transmit(FLASH_CMD_SECTOR_ERASE);
    fe310_flash_transmit(sector_address >> 16);
    fe310_flash_transmit(sector_address >> 8);
    fe310_flash_transmit(sector_address);
    SPI0_REG(SPI_REG_CSMODE) = SPI_CSMODE_AUTO;

    /* Wait for ready */
    fe310_flash_wait_till_ready();

    /* Enable auto mode */
    SPI0_REG(SPI_REG_FCTRL) = 1;

    portEXIT_CRITICAL();

    return 0;
}

int
fe310_flash_sector_info(int idx, uint32_t *address, uint32_t *sz)
{
	configASSERT(idx < fe310_flash_dev.hf_sector_cnt);
    *address = fe310_flash_dev.hf_base_addr + idx * FE310_FLASH_SECTOR_SZ;
    *sz = FE310_FLASH_SECTOR_SZ;
    return 0;
}

int
fe310_flash_init()
{
    return 0;
}
