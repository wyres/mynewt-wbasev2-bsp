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
#ifndef H_BSP_H
#define H_BSP_H

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <mcu/mcu.h>
#include "stm32l151xc.h"
#include "bsp/bsp_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Define special stackos sections */
#define sec_data_core   __attribute__((section(".data.core")))
#define sec_bss_core    __attribute__((section(".bss.core")))
#define sec_bss_nz_core __attribute__((section(".bss.core.nz")))

/* More convenient section placement macros. */
#define bssnz_t         sec_bss_nz_core

extern uint8_t _ram_start;

bool hal_bsp_nvmLock();
bool hal_bsp_nvmUnlock();
// Read and write PROM. It is caller's responsibility to nvmUnlock() before access and nvmLock() afterwards
uint8_t hal_bsp_nvmRead8(uint16_t off);
uint16_t hal_bsp_nvmRead16(uint16_t off);
bool hal_bsp_nvmRead(uint16_t off, uint8_t len, uint8_t* buf);
bool hal_bsp_nvmWrite8(uint16_t off, uint8_t v);
bool hal_bsp_nvmWrite16(uint16_t off, uint16_t v);
bool hal_bsp_nvmWrite(uint16_t off, uint8_t len, uint8_t* buf);
uint16_t hal_bsp_nvmSize();

// Get hw version (1=revB, 2=revC, 3=revD)
int BSP_getHwVer();
// set hw version (eg from config at boot time rather than the build hardcoded version)
void BSP_setHwVer(int v);
void BSP_antSwInit(int txPin, int rxPin);
void BSP_antSwDeInit(int txPin, int rxPin);
void BSP_antSwTx(int txPin, int rxPin);
void BSP_antSwRx(int txPin, int rxPin);

bool hal_bsp_adc_init();
bool hal_bsp_adc_define(int pin, int chan);
int hal_bsp_adc_readmV(int chan);
void hal_bsp_adc_release(int pin, int chan);
void hal_bsp_adc_deinit();

int hal_bsp_init_i2c();
int hal_bsp_deinit_i2c();

#ifdef __cplusplus
}
#endif

#endif  /* H_BSP_H */
