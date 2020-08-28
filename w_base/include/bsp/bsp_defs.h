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
#ifndef H_BSP_DEFS_H
#define H_BSP_DEFS_H

#include "stm32l1xx_hal_dma.h"
#include "stm32l1xx_hal_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

// Set os idle stack size here as we have callups from the os_tick_idle()
#ifdef OS_IDLE_STACK_SIZE
#undef OS_IDLE_STACK_SIZE
#endif
#define OS_IDLE_STACK_SIZE (128)

// The board specific definitions in here for BSP supplied code.


/* UART */
// mynewt device name to access uarts. Note name is mynewt, STM32 doc pin labelling has UART1/2/3
#define UART0_DEV "uart0"
#define UART1_DEV "uart1"
#define UART2_DEV "uart2"
#define UARTDBG_DEV "uartbb0"       // named by uart bitbang code en dur

#define I2C0_DEV "i2c0"         // mynewt bus device name for the I2C1 bus (STM32 numbering)
#define ALTI_NODE "alt0"        // mynewt bus node name for the altimeter
#define ACC_NODE "acc0"          // mynewt bus node name for the accelero

/* Our altimetre is a LPS22HB */
#define ALTI_LPS22HB 1
/* and the accelero is a LIS2DE12 */
#define ACC_LIS2DE12 1

// Mappings for generic code modules are in the syscfg

/* GPIO  type pins */
#define LED_1                               MCU_GPIO_PORTA(0)       // also ADC input 0
#define LED_2                               MCU_GPIO_PORTA(15)      // No ADC
#define LED_BLINK_PIN   LED_1

#define LIGHT_SENSOR                        MCU_GPIO_PORTA(3)       // ADC channel 3
#define LIGHT_SENSOR_ADCCHAN (ADC_CHANNEL_3)
#define EXT_IO                              MCU_GPIO_PORTA(8)       // No ADC

#define EXT_BUTTON                          MCU_GPIO_PORTB(3)       // No ADC
#define BUTTON                              (EXT_BUTTON)
#define SPEAKER                             MCU_GPIO_PORTA(1)       // also ADC input 1
#define EXT_UART_PWR                        MCU_GPIO_PORTA(11)      // No ADC
#define EXT_I2C_PWR                         MCU_GPIO_PORTA(12)      // No ADC
#define SENSOR_PWR                          MCU_GPIO_PORTB(6)       // No ADC

#define BATTERY_GPIO    (128)                   // needs a gpio pin number, but of a value that is beyond 'real' ones
#define BATTERY_ADCCHAN (ADC_CHANNEL_VREFINT)

// ALtimetre I2C chan/addr
#define ALTIMETER_I2C_CHAN 0
#define ALTIMETER_I2C_ADDR 0x5D
// accelerometer I2C chan/addr
#define ACCELERO_I2C_CHAN  0
// Default I2C addres for ST lis2de12 is 0x18, our board pulls SA0 (which is the LSB) high -> 0x19
#define ACCELERO_I2C_ADDR  0x19

// Note : STM32 internal periph gpio mappings set in syscfg
// I2C0 is first I2C on STM32 (called I2C1 in pinout doc)
#define I2C_0_FREQUENCY (100000)

// SPI0 is mynewt numbering, but SPI1 is STM32 name for first SPI. 
#define SPI_0_IRQ_PRIO (2)
#define SPI_0_BAUDRATE (3000)

#define SPI_1_IRQ_PRIO (2)
#define SPI_1_BAUDRATE (3000)
// Note I2C and SPI pin allocation is in syscfg.yml

// debug uart does bitbang on a gpio (optional)
#define BSP_UART_DBG_TX                     MYNEWT_VAL(UART_DBG_TX)
#define BSP_UART_DBG_RX                     MYNEWT_VAL(UART_DBG_RX)

// I2S2 Pins
#define MICROPHONE_I2S2_SD_PIN              MCU_GPIO_PORTB(15)
#define MICROPHONE_I2S2_CLK_PIN             MCU_GPIO_PORTB(13)

// CN5 : note this can be SPI2 bus (but not if using microphone, as PB13/PB15 are used for I2S) or GPIOs (again, PB13/PB15 not if using microphone)
#define CN5_PIN1                            MCU_GPIO_PORTB(15)
#define CN5_PIN2                            MCU_GPIO_PORTB(14)
#define CN5_PIN3                            MCU_GPIO_PORTB(13)
#define CN5_PIN4                            MCU_GPIO_PORTB(12)

#define SX1272_PIN_DIO_0						MCU_GPIO_PORTB(1)
#define SX1272_PIN_DIO_1						MCU_GPIO_PORTB(10)
#define SX1272_PIN_DIO_2						MCU_GPIO_PORTB(11)
#define SX1272_PIN_DIO_3						MCU_GPIO_PORTB(7)
#define SX1272_PIN_DIO_4						MCU_GPIO_PORTB(5)
#define SX1272_PIN_DIO_5						MCU_GPIO_PORTB(4)
#define SX1272_PIN_NSS							MCU_GPIO_PORTB(0)
#define SX1272_PIN_RESET						MCU_GPIO_PORTA(2)

#define ANTENNA_SWITCH_TX					MCU_GPIO_PORTA(4)
#define ANTENNA_SWITCH_RX					MCU_GPIO_PORTC(13)

#define SWD_CLK								MCU_GPIO_PORTA(14)
#define SWD_DIO								MCU_GPIO_PORTA(13)

#define HSE_IN		 						MCU_GPIO_PORTH(0)
#define HSE_OUT								MCU_GPIO_PORTH(1)

#define LSE_IN		 						MCU_GPIO_PORTC(14)
#define LSE_OUT								MCU_GPIO_PORTC(15)

#ifdef __cplusplus
}
#endif

#endif  /* H_BSP_DEFS_H */
