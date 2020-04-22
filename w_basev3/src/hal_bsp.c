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
#include <assert.h>

#include "os/mynewt.h"

#include "bsp/bsp.h"
#include "sysflash/sysflash.h"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

#define RAM_SIZE    (32 * 1024)         // TODO this should come from bsp.yml
//#define PROM_BASE sysflash_map_dflt[EEPROM_AREA].fa_off
//#define PROM_SIZE sysflash_map_dflt[EEPROM_AREA].fa_size
#define PROM_BASE (0x08080000)      // MYNEWT_VAL(bsp.eeprom_map.EEPROM_AREA.offset)      ?? how to get from bsp.yml
#define PROM_SIZE   (8*1024)            // TODO this should also come from bsp.yml as .size

#if MYNEWT_VAL(UART_0) || MYNEWT_VAL(UART_DBG)
#include <uart/uart.h>
#endif
#if MYNEWT_VAL(UART_0)
#include <uart_hal/uart_hal.h>
#endif
#if MYNEWT_VAL(UART_DBG)
#include <uart_bitbang/uart_bitbang.h>
#endif

#include <hal/hal_bsp.h>
#include <hal/hal_gpio.h>
#include <hal/hal_flash_int.h>
#include <hal/hal_timer.h>

#if MYNEWT_VAL(ADC) 
//#include <adc/adc.h>
#include "stm32l1xx_hal_adc.h"
#include "stm32l1xx_hal_rcc.h"
#include "stm32l1xx_hal.h"

#endif

#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
#include <hal/hal_spi.h>
#endif
#if MYNEWT_VAL(I2C_0)
#if MYNEWT_VAL(USE_BUS_I2C)
#include "bus/drivers/i2c_common.h"
#include "bus/drivers/i2c_hal.h"
#else
#include "hal/hal_i2c.h"
#endif  /* USE_BUS_I2C */
#endif      /* I2C_0 */

#include <stm32l151xc.h>
#include <stm32l1xx_hal_rcc.h>
#include <stm32l1xx_hal_pwr.h>
#include <stm32l1xx_hal_flash.h>
#include <stm32l1xx_hal_gpio.h>
#include <stm32l1xx_hal_gpio_ex.h>
#include <mcu/stm32l1_bsp.h>
#include "mcu/stm32l1xx_mynewt_hal.h"
#include "mcu/stm32_hal.h"
#if MYNEWT_VAL(I2S)
#include "stm32l1xx_hal_i2s.h"
#endif

#include "bsp/bsp.h"

// Should be in header file??
extern void hal_mcu_halt();  
extern int hal_gpio_init_stm(int pin, GPIO_InitTypeDef *cfg);

/*
*   Array with all idle/default gpio mode 
*   for lowest power consumption in initial
*   mode
*/
typedef struct
{
	int pin;
	int idle_type;  /*as defined in stm32l1xx_hal_gpio.h */
}w_base_v2_pins_t;

static w_base_v2_pins_t W_BASE_V2_PINS_IDLE[] =
{    
/*
    { .pin = SX1262_PIN_DIO_0, 								.idle_type = GPIO_NOPULL	},
    { .pin = SX122_DIO_1, 									.idle_type = GPIO_NOPULL	},
    { .pin = SX1272_DIO_2, 									.idle_type = GPIO_NOPULL	},
    { .pin = SX1272_DIO_3, 									.idle_type = GPIO_NOPULL	},
    { .pin = SX1272_DIO_4, 									.idle_type = GPIO_NOPULL	},
    { .pin = SX1272_DIO_5, 									.idle_type = GPIO_NOPULL	},
    { .pin = SX1262_PIN_RESET, 								.idle_type = GPIO_PULLUP	},
*/
    { .pin = LED_1, 										.idle_type = GPIO_PULLDOWN	},
    { .pin = LED_2, 										.idle_type = GPIO_PULLDOWN	},
 
    { .pin = EXT_UART_PWR, 									.idle_type = GPIO_PULLUP	},
 
    { .pin = EXT_I2C_PWR, 									.idle_type = GPIO_PULLUP	},

    { .pin = LIGHT_SENSOR, 									.idle_type = GPIO_PULLDOWN	},
    { .pin = SPEAKER, 										.idle_type = GPIO_PULLUP	},
    /*WARNING : SENSOR_PWR still drains current even in INPUT.PULL_DOWN mode. This pin  */
    /*          must be in OUTPUT zero                                                  */
    /*{ .pin = SENSOR_PWR, 									.idle_type = GPIO_PULLDOWN 	},*/

    { .pin = EXT_IO, 										.idle_type = GPIO_PULLDOWN	},
    { .pin = EXT_BUTTON, 									.idle_type = GPIO_PULLDOWN	},
    { .pin = MYNEWT_VAL(SPI_1_PIN_MISO),					.idle_type = GPIO_PULLDOWN	},
    { .pin = MYNEWT_VAL(SPI_1_PIN_SS),		    			.idle_type = GPIO_PULLDOWN	},

#if 0//MYNEWT_VAL(BUILD_RELEASE)
    { .pin = SWD_CLK, 										.idle_type = GPIO_PULLDOWN 	},
    { .pin = SWD_DIO, 										.idle_type = GPIO_PULLDOWN 	},
#endif
 
    { .pin = HSE_IN, 										.idle_type = GPIO_NOPULL 	},
    { .pin = HSE_OUT, 										.idle_type = GPIO_NOPULL 	},
 
    /*TODO : test it */
    /*{ .pin = LSE_IN, 										.idle_type = GPIO_PULLDOWN 	},      */
    /*{ .pin = LSE_OUT, 										.idle_type = GPIO_PULLDOWN 	},  */
};



void bsp_deinit_all_ios()
{
    int i, pin, type;
    
    GPIO_InitTypeDef highz_cfg = {
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL
    };

    /* I2S */
    bsp_deinit_i2s();

    /* I2C */
    hal_bsp_deinit_i2c();

    /* SPI */
    hal_bsp_deinit_spi();

    /*UART */
    hal_bsp_uart_deinit();
    
    for(i=0; i<NELEMS(W_BASE_V2_PINS_IDLE); i++){

        pin = W_BASE_V2_PINS_IDLE[i].pin;
        type = W_BASE_V2_PINS_IDLE[i].idle_type;

        if(pin > 0){
            //deinit will set it as configured
            hal_gpio_deinit(pin);
            if(type==GPIO_NOPULL){
                /*note for HIGH_Z mode :                                */
                /*analog input setup is recommmended for lowest power   */
                /*consumption but actually not allowed by hal_gpio.c   */
                highz_cfg.Pin = pin;
                highz_cfg.Alternate = pin;
                hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
            }else{
                hal_gpio_init_in(pin, type);
            }
        }
    }
}

/* Uart0 is UART1 in STM32 doc hence names of HAL defns */
#if MYNEWT_VAL(UART_0)
static struct uart_dev hal_uart0;

static const struct stm32_uart_cfg os_bsp_uart0_cfg = {
        .suc_uart = USART1,      
        .suc_rcc_reg = &RCC->APB2ENR,
        .suc_rcc_dev = RCC_APB2ENR_USART1EN,
        .suc_pin_tx = MYNEWT_VAL(UART_0_PIN_TX),
        .suc_pin_rx = MYNEWT_VAL(UART_0_PIN_RX),
        .suc_pin_rts = MYNEWT_VAL(UART_0_PIN_RTS),
        .suc_pin_cts = MYNEWT_VAL(UART_0_PIN_CTS),
        .suc_pin_af = GPIO_AF7_USART1,
        .suc_irqn = USART1_IRQn
};
#endif
#if MYNEWT_VAL(UART_1)
static struct uart_dev hal_uart1;

static const struct stm32_uart_cfg os_bsp_uart1_cfg = {
        .suc_uart = USART2,      
        .suc_rcc_reg = &RCC->APB1ENR,
        .suc_rcc_dev = RCC_APB1ENR_USART2EN,
        .suc_pin_tx = MYNEWT_VAL(UART_1_PIN_TX),
        .suc_pin_rx = MYNEWT_VAL(UART_1_PIN_RX),
        .suc_pin_rts = MYNEWT_VAL(UART_1_PIN_RTS),
        .suc_pin_cts = MYNEWT_VAL(UART_1_PIN_CTS),
        .suc_pin_af = GPIO_AF7_USART2,
        .suc_irqn = USART2_IRQn
};
#endif
#if MYNEWT_VAL(UART_2)
static struct uart_dev hal_uart2;

static const struct stm32_uart_cfg os_bsp_uart2_cfg = {
        .suc_uart = USART3,      
        .suc_rcc_reg = &RCC->APB1ENR,
        .suc_rcc_dev = RCC_APB1ENR_USART3EN,
        .suc_pin_tx = MYNEWT_VAL(UART_2_PIN_TX),
        .suc_pin_rx = MYNEWT_VAL(UART_2_PIN_RX),
        .suc_pin_rts = MYNEWT_VAL(UART_2_PIN_RTS),
        .suc_pin_cts = MYNEWT_VAL(UART_2_PIN_CTS),
        .suc_pin_af = GPIO_AF7_USART3,
        .suc_irqn = USART3_IRQn
};
#endif

#if MYNEWT_VAL(ADC) 
/*static struct acd_dev hal_adc_dev;
static const struct stm32_adc_dev_cfg adc_cfg = {
    .c_refmv=0,
    .c_res=12,
    .c_configured=true,
};*/
#endif

#if MYNEWT_VAL(I2C_0)
#if MYNEWT_VAL(USE_BUS_I2C)
static struct bus_i2c_dev bus_i2c0;
static struct bus_i2c_dev_cfg cfg_i2c0 = {
    .i2c_num = 0,
    .pin_sda = I2C_0_SDA,
    .pin_scl = I2C_0_SCL,
};
static struct bus_i2c_node node_alti;
static struct bus_i2c_node_cfg cfg_alti = {
    .node_cfg.bus_name=I2C0_DEV,
    .node_cfg.lock_timeout_ms=0,
    .addr = ALTIMETER_I2C_ADDR,
    .freq = 100,
    .quirks = 0,
};
static struct bus_i2c_node node_acc;
static struct bus_i2c_node_cfg cfg_acc = {
    .node_cfg.bus_name=I2C0_DEV,
    .node_cfg.lock_timeout_ms=0,
    .addr = ACCELERO_I2C_ADDR,
    .freq = 100,
    .quirks = 0,
};
#else   /* USE_BUS_I2C */
// Note: I2C0 is I2C1 in STM32 doc hence names of defns from HAL
struct stm32_hal_i2c_cfg os_bsp_i2c0_cfg = {
    .hic_i2c = I2C1,
    .hic_rcc_reg = &RCC->APB1ENR,
    .hic_rcc_dev = RCC_APB1ENR_I2C1EN,
    .hic_pin_sda = MYNEWT_VAL(I2C_0_PIN_SDA),   // I2C_0_SDA,         // ok
    .hic_pin_scl = MYNEWT_VAL(I2C_0_PIN_SCL),   //I2C_0_SCL,         // ok
    .hic_pin_af = GPIO_AF4_I2C1,
    .hic_10bit = 0,
    .hic_speed = I2C_0_FREQUENCY                     // 100kHz 
};

#endif /* USE_BUS_I2C */

#endif

// SPI0 (mynewt) refers to SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_0_MASTER)
struct stm32_hal_spi_cfg os_bsp_spi0_cfg = {
    .sck_pin = MYNEWT_VAL(SPI_0_PIN_SCK),
    .mosi_pin = MYNEWT_VAL(SPI_0_PIN_MOSI),
    .miso_pin = MYNEWT_VAL(SPI_0_PIN_MISO),
    .ss_pin = MYNEWT_VAL(SPI_0_PIN_SS),
    .irq_prio = SPI_0_IRQ_PRIO,
};
#endif
#if MYNEWT_VAL(SPI_1_SLAVE) || MYNEWT_VAL(SPI_1_MASTER)
struct stm32_hal_spi_cfg os_bsp_spi1_cfg = {
    .sck_pin = MYNEWT_VAL(SPI_1_PIN_SCK),
    .mosi_pin = MYNEWT_VAL(SPI_1_PIN_MOSI),
    .miso_pin = MYNEWT_VAL(SPI_1_PIN_MISO),
    .ss_pin = MYNEWT_VAL(SPI_1_PIN_SS),
    .irq_prio = SPI_1_IRQ_PRIO,
};
#endif

#if MYNEWT_VAL(I2S)
#define I2S_AUDIOFREQ_150K               ((uint32_t)150000)
I2S_HandleTypeDef I2S_InitStructure = {
    .Instance = SPI2,
	.Init.Standard = I2S_STANDARD_PHILIPS,
	.Init.DataFormat = I2S_DATAFORMAT_16B,
	.Init.AudioFreq = I2S_AUDIOFREQ_150K,
	.Init.CPOL = I2S_CPOL_LOW,
	.Init.Mode = I2S_MODE_MASTER_RX,
};
#endif //I2S
static const struct hal_bsp_mem_dump dump_cfg[] = {
    [0] = {
        .hbmd_start = &_ram_start,
        .hbmd_size = RAM_SIZE
    },
};

extern const struct hal_flash stm32_flash_dev;
const struct hal_flash *
hal_bsp_flash_dev(uint8_t id)
{
    /*
     * Internal flash mapped to id 0.
     */
    if (id != 0) {
        return NULL;
    }
    return &stm32_flash_dev;
}

const struct hal_bsp_mem_dump *
hal_bsp_core_dump(int *area_cnt)
{
    *area_cnt = sizeof(dump_cfg) / sizeof(dump_cfg[0]);
    return dump_cfg;
}


void
hal_bsp_init(void)
{
    int rc;

    (void)rc;

    /*GPIOs & interfaces*/
    bsp_deinit_all_ios();
    

    /* NOTE : we do NOT use the stm32 unified peripheral initialisation code (calling stm32_periph_create()) as
     * that code does not (yet) have the neccessary to do deinit()/init() when doing sleep modes
     */
#if MYNEWT_VAL(UART_0)
// BW    hal_bsp_uart_init();
    rc = os_dev_create((struct os_dev *) &hal_uart0, UART0_DEV,
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart0_cfg);
    assert(rc == 0);    
#endif
#if MYNEWT_VAL(UART_1)
    rc = os_dev_create((struct os_dev *) &hal_uart1, UART1_DEV,
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart1_cfg);
    assert(rc == 0);    
#endif
#if MYNEWT_VAL(UART_2)
    rc = os_dev_create((struct os_dev *) &hal_uart2, UART2_DEV,
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&os_bsp_uart2_cfg);
    assert(rc == 0);    
#endif

#if MYNEWT_VAL(TIMER_0)
    hal_timer_init(0, MYNEWT_VAL(TIMER_0_TIM));
#endif

#if MYNEWT_VAL(TIMER_1)    
    hal_timer_init(1, MYNEWT_VAL(TIMER_1_TIM));
#endif

#if MYNEWT_VAL(TIMER_2)
    hal_timer_init(2, MYNEWT_VAL(TIMER_2_TIM));
#endif

#if (MYNEWT_VAL(OS_CPUTIME_TIMER_NUM) >= 0)
    rc = os_cputime_init(MYNEWT_VAL(OS_CPUTIME_FREQ));
    assert(rc == 0);
#endif

#if MYNEWT_VAL(ADC)
/*  device for adc not yet possible as only STM32F4 driver  
    rc = os_dev_create((struct os_dev *) &hal_adc, ADC_DEV,
      OS_DEV_INIT_PRIMARY, 0, adc_hal_init, (void *)&adc_cfg);
    assert(rc == 0);
    */
#endif

    rc = hal_bsp_init_spi();
    assert(rc ==0);

    rc = hal_bsp_init_i2c();
    assert(rc ==0);
    // only init i2c here if using bus driver - else the i2c is init/deinit() on each usage
#if MYNEWT_VAL(USE_BUS_I2C)
    // Create I2C0 bus driver
    rc = bus_i2c_hal_dev_create(I2C0_DEV, &bus_i2c0, &cfg_i2c0);
    assert(rc == 0);
    // And the I2C nodes on the W_BASE card
    // Altimeter
    rc = bus_i2c_node_create(ALTI_NODE, &node_alti,
                    &cfg_alti, &(cfg_alti.node_cfg));
    assert(rc == 0);
    // Accelero
    rc = bus_i2c_node_create(ACC_NODE, &node_acc,
                    &cfg_acc, &(cfg_acc.node_cfg));
    assert(rc == 0);
#endif /* USE_BUS_I2C */

#if MYNEWT_VAL(I2S)
    rc = bsp_init_i2s();
    assert(rc == 0);
#endif //I2S

    /* power up radio here */
    hal_bsp_radio_on();

}

/**
 * Returns the configured priority for the given interrupt. If no priority
 * configured, return the priority passed in
 *
 * @param irq_num
 * @param pri
 *
 * @return uint32_t
 */
uint32_t
hal_bsp_get_nvic_priority(int irq_num, uint32_t pri)
{
    /* Add any interrupt priorities configured by the bsp here */
    return pri;
}

int hal_bsp_init_spi(void) {

    int rc=0;

// note : SPI0 is SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, &os_bsp_spi0_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_SLAVE)
    rc = hal_spi_init(0, &os_bsp_spi0_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_MASTER)
    // MyNewt numbers devices from 0
    rc = hal_spi_init(1, &os_bsp_spi1_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_SLAVE)
    rc = hal_spi_init(1, &os_bsp_spi1_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif
    return rc;

/*ANOTHER DEINIT METHOD */
/*
#if MYNEWT_VAL(SPI_0_MASTER) || MYNEWT_VAL(SPI_0_SLAVE)
    hal_spi_enable(0);
#endif

#if MYNEWT_VAL(SPI_1_MASTER) || MYNEWT_VAL(SPI_1_SLAVE)
    hal_spi_enable(1);
#endif
*/

}

int hal_bsp_deinit_spi(void){

// note : SPI0 is SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_0_MASTER)
    
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    __HAL_RCC_SPI1_CLK_DISABLE( );

    hal_gpio_deinit(os_bsp_spi0_cfg.ss_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.ss_pin, HAL_GPIO_PULL_UP);

    hal_gpio_deinit(os_bsp_spi0_cfg.sck_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.sck_pin, HAL_GPIO_PULL_DOWN);
    
    hal_gpio_deinit(os_bsp_spi0_cfg.miso_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.miso_pin, HAL_GPIO_PULL_DOWN);

    hal_gpio_deinit(os_bsp_spi0_cfg.mosi_pin);
    hal_gpio_init_in(os_bsp_spi0_cfg.mosi_pin, HAL_GPIO_PULL_DOWN);
#endif

#if MYNEWT_VAL(SPI_1_SLAVE) || MYNEWT_VAL(SPI_1_MASTER)
    
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    __HAL_RCC_SPI2_CLK_DISABLE( );

    hal_gpio_deinit(os_bsp_spi1_cfg.ss_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.ss_pin, HAL_GPIO_PULL_UP);

    hal_gpio_deinit(os_bsp_spi1_cfg.sck_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.sck_pin, HAL_GPIO_PULL_DOWN);
    
    hal_gpio_deinit(os_bsp_spi1_cfg.miso_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.miso_pin, HAL_GPIO_PULL_DOWN);

    hal_gpio_deinit(os_bsp_spi1_cfg.mosi_pin);
    hal_gpio_init_in(os_bsp_spi1_cfg.mosi_pin, HAL_GPIO_PULL_DOWN);

#endif
    return 0;
}

#if MYNEWT_VAL(USE_BUS_I2C)
// need these as STM32 MCU HAL code does NOT define them, and the I2C mynewt driver code requires them
int hal_i2c_disable(uint8_t n) {
    return 0;
}
int hal_i2c_enable(uint8_t n) {
    return 0;
}
int hal_i2c_init_hw(uint8_t i2c_num, const struct hal_i2c_hw_settings *cfg) {
    return 0;
}
int hal_i2c_config(uint8_t i2c_num, const struct hal_i2c_settings *cfg) {
    
    return 0;
}
#endif /* USE_BUS_I2C */
// initialise I2C
int hal_bsp_init_i2c() {
    int rc = 0;
#if MYNEWT_VAL(I2C_0)
    rc = hal_i2c_init(0, &os_bsp_i2c0_cfg);
#endif
#if MYNEWT_VAL(I2C_1)
    rc = hal_i2c_init(1, &os_bsp_i2c1_cfg);
#endif
#if MYNEWT_VAL(I2C_2)
    rc = hal_i2c_init(2, &os_bsp_i2c2_cfg);
#endif
    return rc;
}
// deinit for power saving
int hal_bsp_deinit_i2c() {
    
#if MYNEWT_VAL(I2C_0)

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE( );

#if 0
    /*deinit is not done properly because of this : */
    hal_gpio_deinit(os_bsp_i2c0_cfg.hic_pin_sda);
    hal_gpio_init_in(os_bsp_i2c0_cfg.hic_pin_sda, HAL_GPIO_PULL_UP);
    
    hal_gpio_deinit(os_bsp_i2c0_cfg.hic_pin_scl);
   	hal_gpio_init_in(os_bsp_i2c0_cfg.hic_pin_scl, HAL_GPIO_PULL_UP);    
#endif

#endif
    return 0;
}

#if MYNEWT_VAL(I2S)

int bsp_init_i2s()
{
    int rc = -1;
    // Enable SPI2 APB1 clocks
	__HAL_RCC_SPI2_CLK_ENABLE();

	// Enable GPIOB clock
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// I2S peripheral configuration
	__HAL_I2S_RESET_HANDLE_STATE(&I2S_InitStructure);

    hal_gpio_init_af(MICROPHONE_I2S2_SD_PIN, GPIO_AF5_SPI2, GPIO_PULLUP, GPIO_MODE_AF_PP);
    hal_gpio_init_af(MICROPHONE_I2S2_CLK_PIN, GPIO_AF5_SPI2, GPIO_PULLUP, GPIO_MODE_AF_PP);

	rc = HAL_I2S_Init(&I2S_InitStructure);

	// SPI1 IRQ Channel configuration
	HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(SPI2_IRQn);

    // Enable the I2S
    __HAL_I2S_ENABLE(&I2S_InitStructure);

    return rc;
}

int bsp_deinit_i2s()
{
    int rc = -1;

    rc = HAL_I2S_DeInit(&I2S_InitStructure);

    HAL_NVIC_DisableIRQ(SPI2_IRQn);

    // Disable SPI2 APB1 clocks
	__HAL_RCC_SPI2_CLK_DISABLE();

	// Disable GPIOB clock
	__HAL_RCC_GPIOB_CLK_DISABLE();

    // Disable the I2S
    __HAL_I2S_DISABLE(&I2S_InitStructure);

    hal_gpio_deinit(MICROPHONE_I2S2_SD_PIN);
    hal_gpio_init_in(MICROPHONE_I2S2_SD_PIN, HAL_GPIO_PULL_DOWN);
    hal_gpio_deinit(MICROPHONE_I2S2_CLK_PIN);
    hal_gpio_init_in(MICROPHONE_I2S2_CLK_PIN, HAL_GPIO_PULL_DOWN);

    return rc;
}


#else
int bsp_init_i2s()
{
    return -1;
}

int bsp_deinit_i2s()
{
    return 0;
}
#endif //I2S




// NVM access - we have a EEPROM on this MCU which is handy
uint16_t hal_bsp_nvmSize() {
    return PROM_SIZE;
}
bool hal_bsp_nvmLock() {
    return (HAL_FLASHEx_DATAEEPROM_Lock()==HAL_OK);
}
bool hal_bsp_nvmUnlock() {
    return (HAL_FLASHEx_DATAEEPROM_Unlock()==HAL_OK);
}
uint8_t hal_bsp_nvmRead8(uint16_t off) {
    return *((volatile uint8_t*)(PROM_BASE+off));
}
uint16_t hal_bsp_nvmRead16(uint16_t off) {
    return *((volatile uint16_t*)(PROM_BASE+off));
}
bool hal_bsp_nvmRead(uint16_t off, uint8_t len, uint8_t* buf) {
    for(int i=0;i<len;i++) {
        *(buf+i) = hal_bsp_nvmRead8(off+i);
    }
    return true;
}

bool hal_bsp_nvmWrite8(uint16_t off, uint8_t v) {
    HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_BYTE, ((uint32_t)PROM_BASE)+off);
    return (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTBYTE, ((uint32_t)PROM_BASE)+off, v)==HAL_OK);
}
bool hal_bsp_nvmWrite16(uint16_t off, uint16_t v) {
    HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEERASEDATA_HALFWORD, ((uint32_t)PROM_BASE)+off);
    return (HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_FASTHALFWORD, ((uint32_t)PROM_BASE)+off, v)==HAL_OK);
}
bool hal_bsp_nvmWrite(uint16_t off, uint8_t len, uint8_t* buf) {
    bool ret = true;
    for(int i=0;i<len;i++) {
        ret &= hal_bsp_nvmWrite8(off+i, *(buf+i));
    }
    return ret;
}

// hwrev value is in build, but can be overridden by app code (eg from a config value at boot time)
static int _hwrev = MYNEWT_VAL(BSP_HW_REV);
int BSP_getHwVer() {
    return _hwrev;
}
void BSP_setHwVer(int v) {
    _hwrev = v;
}
// BSP specific functions to set the radio antenna switch correctly
// For W_BASEV3 card, the antenna switch is controlled by the SX1262 so these functions have no effect
void BSP_antSwInit(int txPin, int rxPin) {
}
void BSP_antSwDeInit(int txPin, int rxPin) {
}
void BSP_antSwTx(int txPin, int rxPin) {
}
void BSP_antSwRx(int txPin, int rxPin) {
}

#if MYNEWT_VAL(ADC) 
// Initialise an adc for basic gpio like use
static struct {
    ADC_HandleTypeDef adcHandle;
    bool active;
} _adc1 = {
    .adcHandle= {        
        .Instance = ADC1,
    },
    .active=false,
};

bool hal_bsp_adc_init() {
    // allow mulitple calls to init without issues
    if (!_adc1.active) {
        // Configure ADC
        ADC_HandleTypeDef* adch = &_adc1.adcHandle;

#if MYNEWT_VAL(STM32_CLOCK_HSE)
        // Enable HSI - already done in clock setup
        __HAL_RCC_HSI_ENABLE( );

        // Wait till HSI is ready
        while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET );
#endif
        __HAL_RCC_ADC1_CLK_ENABLE( );
        // First ensure its is in known state (seems odd but if you don't do this then the init fails)
        HAL_ADC_DeInit( adch );
        // setup init data
        adch->Init.Resolution            = ADC_RESOLUTION_12B;
        adch->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        adch->Init.ContinuousConvMode    = DISABLE;
        adch->Init.DiscontinuousConvMode = DISABLE;
        adch->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        adch->Init.ExternalTrigConv      = ADC_SOFTWARE_START;        
        adch->Init.DMAContinuousRequests = DISABLE;
        adch->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        adch->Init.NbrOfConversion       = 1;
        adch->Init.LowPowerAutoWait      = DISABLE;
        adch->Init.LowPowerAutoPowerOff  = DISABLE;
        int rc = HAL_ADC_Init( adch );
        if (rc!=HAL_OK) {
            return false;   // thats a fail
        }
        
        // Enable ADC1
        __HAL_ADC_ENABLE( adch );

        _adc1.active = true;
    }
    return true;
}

bool hal_bsp_adc_define(int pin, int chan) {
    ADC_HandleTypeDef* adch = &_adc1.adcHandle;
    ADC_ChannelConfTypeDef adcConf = { 0 };
    adcConf.Channel = chan;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;

    int rc = HAL_ADC_ConfigChannel( adch, &adcConf );

    // If pin is 'real', define it as analog in 
    if (pin>=0 && pin <128) {
        /* 
        GPIO_InitTypeDef gpio_td = {
            .Pin = pin,
            .Mode = GPIO_MODE_ANALOG,
            .Pull = GPIO_NOPULL,
            .Alternate = pin
        };
        hal_gpio_init_stm(gpio_td.Pin, &gpio_td);
        */
        // easiest way is to deconfig it as GPIO as does the same without using stm specific hal fn
        hal_gpio_deinit(pin);
    }
    return (rc==HAL_OK);
}



int hal_bsp_adc_read(int channel) {
    ADC_HandleTypeDef* adch = &_adc1.adcHandle;
    ADC_ChannelConfTypeDef adcConf = { 0 };
    int adcData = 0;

    adcConf.Channel = channel;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ADC_SAMPLETIME_192CYCLES;
    HAL_ADC_ConfigChannel( adch, &adcConf );
    // Start ADC Software Conversion
    HAL_ADC_Start(adch);

    HAL_ADC_PollForConversion(adch, HAL_MAX_DELAY);

    adcData = HAL_ADC_GetValue(adch);
    
    return (uint16_t)adcData;
}

void hal_bsp_adc_release(int pin, int chan) {
    // noop? or define as ANALOG for lowest power?
    if (pin>=0 && pin <128) {
        // but this is already its state... 
        hal_gpio_deinit(pin);
    }

}
void hal_bsp_adc_deinit() {
    if (_adc1.active) {
        ADC_HandleTypeDef* adch = &_adc1.adcHandle;
        // Stop ADC
        __HAL_ADC_DISABLE( adch);
        // Stop its clock
        __HAL_RCC_ADC1_CLK_DISABLE( );

#if MYNEWT_VAL(STM32_CLOCK_HSE)
        // Disable HSI - no, required for system clock
        __HAL_RCC_HSI_DISABLE( );
#endif

        HAL_ADC_DeInit( adch );
        _adc1.active = false;
    }
}

#else   /* !ADC */

bool hal_bsp_adc_init() {
    return false;       // no ADC here
}

bool hal_bsp_adc_define(int pin, int chan) {
    return false;
}

int hal_bsp_adc_read(int channel) {
    return 0;
}

void hal_bsp_adc_release(int pin, int chan) {
}
void hal_bsp_adc_deinit() {
}
#endif  /* ADC */


/** PWM management functions */
bool hal_bsp_pwm_init() {
    // nothing required globally
    return true;
}
bool hal_bsp_pwm_define(int pin, int tim) {
    // Check that this pin has the AF being the output of this timer
    // static table for STM32L151CC, crossed with possible GPIO pins free on this board
    // TODO
    return true;
}

bool hal_bsp_pwm_start(int pin, int tim, int freq, int dc) {
#if 0
//    GPIO_InitTypeDef GPIO_InitCfg;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef  TIM_OCInitStruct;

    switch(pin) {
        case MCU_GPIO_PORTA(1): {
        /* --------------------------- System Clocks Configuration ---------------------*/
        /* TIM2 clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
        /* GPIOA clock enable */
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        /*--------------------------------- GPIO Configuration -------------------------*/
        // enable pin as AF function for this timer
/*        GPIO_InitCfg.Pin =  1<<(pin%16);
        GPIO_InitCfg.Mode = GPIO_MODE_OUTPUT_PP;                	
        GPIO_InitCfg.Pull = GPIO_NOPULL;              	//who knows think its 'push-pull mode'
        GPIO_InitCfg.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitCfg.Alternate = GPIO_AF1_TIM2;
        hal_gpio_init_stm(pin, &GPIO_InitCfg);
        */
        hal_gpio_init_af(pin, GPIO_AF1_TIM2, HAL_GPIO_PULL_NONE, 0);

        int period = (1000000 / (freq)) * (HSE_VALUE/1000000); 
        int dcTrigger = (period * dc) / 100;

        TIM_TimeBaseInitStruct.TIM_Prescaler = 1;
        TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Down;         //Timer counts up from 0
        TIM_TimeBaseInitStruct.TIM_Period = period-1;
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);   

        /* PWM1 Mode configuration: Channel2 
        PA1 is associated with TIM2_CH2
        The Channel number is in the function call: 
        TIM_OC2Init = channel 2  */
        TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;                   //pin output mode configured as PWM (repetitive)
        TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OC2Init(TIM2, &TIM_OCInitStruct);
        
        TIM_SetCompare2(TIM2,dcTrigger); // duty cicle [0:1000]

            /* TIM2 enable counter */
        TIM_Cmd(TIM2, ENABLE);
        break;
    }
    default: {
        assert(0);      // no other pin supporting PWM out
    }
#endif
    return true;
}
void hal_bsp_pwm_stop(int pin, int tim) {
#if 0
    	/* TIM2 disable counter */
	TIM_Cmd(TIM2, DISABLE);
    /* deconfig pin */
    hal_gpio_deinit(p->pin);
#endif
}
void hal_bsp_pwm_deinit() {
        // nothing required globally
}

#if MYNEWT_VAL(I2S)
int hal_bsp_i2s_read(uint16_t *data)
{
    HAL_StatusTypeDef rc = HAL_ERROR;

    while( __HAL_I2S_GET_FLAG( &I2S_InitStructure, I2S_FLAG_TXE ) == RESET );
    rc = HAL_I2S_Transmit(&I2S_InitStructure, data, 1, 0xFF);
    if (rc != HAL_OK)
    {
        return -1;
    }

    while( __HAL_I2S_GET_FLAG( &I2S_InitStructure, I2S_FLAG_RXNE ) == RESET );
    rc = HAL_I2S_Receive(&I2S_InitStructure, data, 1, 0xFF);
    if (rc != HAL_OK)
    {
        return -1;
    }
    return 0;
}
#else
    int hal_bsp_i2s_read(uint16_t *data)
    {
        return -1;
    }
#endif //I2S

// Power up radio
void hal_bsp_radio_on(void) {
    assert(RADIO_PWR_ENABLE>=0);        // wbasev3 has radio power control
    hal_gpio_deinit(RADIO_PWR_ENABLE);
    hal_gpio_init_out(RADIO_PWR_ENABLE, 1);
    hal_gpio_write(RADIO_PWR_ENABLE, 1);
}
void hal_bsp_radio_off(void) {
    assert(RADIO_PWR_ENABLE>=0);        // wbasev3 has radio power control
    hal_gpio_write(RADIO_PWR_ENABLE, 0);
    hal_gpio_deinit(RADIO_PWR_ENABLE);
    hal_gpio_init_in(RADIO_PWR_ENABLE, HAL_GPIO_PULL_DOWN);
}

void hal_bsp_uart_init(void)
{
#if MYNEWT_VAL(UART_0)
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_CLK_ENABLE( );

    if (os_bsp_uart0_cfg.suc_pin_tx>=0) {
        hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_tx);
        hal_gpio_init_af(os_bsp_uart0_cfg.suc_pin_tx, os_bsp_uart0_cfg.suc_pin_af, 0, 0);
    }
    
    if (os_bsp_uart0_cfg.suc_pin_rx>=0) {
        hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_rx);
        hal_gpio_init_af(os_bsp_uart0_cfg.suc_pin_rx, os_bsp_uart0_cfg.suc_pin_af, 0, 0);
    }
#endif
#if MYNEWT_VAL(UART_1)
    __HAL_RCC_USART2_RELEASE_RESET( );
    __HAL_RCC_USART2_RELEASE_RESET( );
    __HAL_RCC_USART2_CLK_ENABLE( );

    if (os_bsp_uart1_cfg.suc_pin_tx>=0) {
        hal_gpio_deinit(os_bsp_uart1_cfg.suc_pin_tx);
        hal_gpio_init_af(os_bsp_uart1_cfg.suc_pin_tx, os_bsp_uart1_cfg.suc_pin_af, 0, 0);
    }
    if (os_bsp_uart1_cfg.suc_pin_rx>=0) {
       hal_gpio_deinit(os_bsp_uart1_cfg.suc_pin_rx);
        hal_gpio_init_af(os_bsp_uart1_cfg.suc_pin_rx, os_bsp_uart1_cfg.suc_pin_af, 0, 0);
    }
#endif
#if MYNEWT_VAL(UART_2)
    __HAL_RCC_USART3_RELEASE_RESET( );
    __HAL_RCC_USART3_RELEASE_RESET( );
    __HAL_RCC_USART3_CLK_ENABLE( );

    if (os_bsp_uart2_cfg.suc_pin_tx>=0) {
        hal_gpio_deinit(os_bsp_uart2_cfg.suc_pin_tx);
        hal_gpio_init_af(os_bsp_uart2_cfg.suc_pin_tx, os_bsp_uart2_cfg.suc_pin_af, 0, 0);
    }
    
    if (os_bsp_uart2_cfg.suc_pin_rx>=0) {
        hal_gpio_deinit(os_bsp_uart2_cfg.suc_pin_rx);
        hal_gpio_init_af(os_bsp_uart2_cfg.suc_pin_rx, os_bsp_uart2_cfg.suc_pin_af, 0, 0);
    }
#endif
}

static GPIO_InitTypeDef highz_cfg = {
        .Mode = GPIO_MODE_ANALOG,
        .Pull = GPIO_NOPULL
};

void hal_bsp_uart_deinit(void)
{
#if MYNEWT_VAL(UART_0)
    // Uart0 is UART1 in STM32 doc hence names of HAL defns
    __HAL_RCC_USART1_FORCE_RESET( );
    __HAL_RCC_USART1_RELEASE_RESET( );
    __HAL_RCC_USART1_CLK_DISABLE( );

    if (os_bsp_uart0_cfg.suc_pin_tx>=0) {
        hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_tx);
        highz_cfg.Pin = os_bsp_uart0_cfg.suc_pin_tx;
        highz_cfg.Alternate = os_bsp_uart0_cfg.suc_pin_tx;
        hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    }    
    if (os_bsp_uart0_cfg.suc_pin_rx>=0) {
        hal_gpio_deinit(os_bsp_uart0_cfg.suc_pin_rx);
        highz_cfg.Pin = os_bsp_uart0_cfg.suc_pin_rx;
        highz_cfg.Alternate = os_bsp_uart0_cfg.suc_pin_rx;
        hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    }
#endif //MYNEWT_VAL(UART_0)
#if MYNEWT_VAL(UART_1)
    // Uart1 is UART2 in STM32 doc hence names of HAL defns
    __HAL_RCC_USART2_FORCE_RESET( );
    __HAL_RCC_USART2_RELEASE_RESET( );
    __HAL_RCC_USART2_CLK_DISABLE( );

    if (os_bsp_uart1_cfg.suc_pin_tx>=0) {
        hal_gpio_deinit(os_bsp_uart1_cfg.suc_pin_tx);
        highz_cfg.Pin = os_bsp_uart1_cfg.suc_pin_tx;
        highz_cfg.Alternate = os_bsp_uart1_cfg.suc_pin_tx;
        hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    }    
    if (os_bsp_uart1_cfg.suc_pin_rx>=0) {
        hal_gpio_deinit(os_bsp_uart1_cfg.suc_pin_rx);
        highz_cfg.Pin = os_bsp_uart1_cfg.suc_pin_rx;
        highz_cfg.Alternate = os_bsp_uart1_cfg.suc_pin_rx;
        hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    }
#endif //MYNEWT_VAL(UART_1)
#if MYNEWT_VAL(UART_2)
    // Uart2 is UART3 in STM32 doc hence names of HAL defns
    __HAL_RCC_USART3_FORCE_RESET( );
    __HAL_RCC_USART3_RELEASE_RESET( );
    __HAL_RCC_USART3_CLK_DISABLE( );

    if (os_bsp_uart2_cfg.suc_pin_tx>=0) {
        hal_gpio_deinit(os_bsp_uart2_cfg.suc_pin_tx);
        highz_cfg.Pin = os_bsp_uart2_cfg.suc_pin_tx;
        highz_cfg.Alternate = os_bsp_uart2_cfg.suc_pin_tx;
        hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    }    
    if (os_bsp_uart2_cfg.suc_pin_rx>=0) {
        hal_gpio_deinit(os_bsp_uart2_cfg.suc_pin_rx);
        highz_cfg.Pin = os_bsp_uart2_cfg.suc_pin_rx;
        highz_cfg.Alternate = os_bsp_uart2_cfg.suc_pin_rx;
        hal_gpio_init_stm(highz_cfg.Pin, &highz_cfg);
    }
#endif //MYNEWT_VAL(UART_2)
}

/** enter a MCU stop mode, with all periphs off or lowest possible power, and never return */
void hal_bsp_halt() {
      
    //tell lowpowermgr to deinit stuff
    hal_bsp_power_handler_sleep_enter(HAL_BSP_POWER_DEEP_SLEEP);

    // ask MCU HAL to stop it
    hal_mcu_halt();
}

#if MYNEWT_VAL(BSP_POWER_SETUP)

static LP_HOOK_t _hook_get_mode_cb=NULL;
static LP_HOOK_t _hook_exit_cb=NULL;
static LP_HOOK_t _hook_enter_cb=NULL;

/* hook idle enter/exit phases. 
 * Note the get_mode call is made with irq disabled in OS critical section - so don't hang about
 * enter/exit are called outside of the critical region
 * */
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit)
{
    // Should only have 1 hook of sleeping in the code, so assert if called twice
    assert(_hook_enter_cb==NULL);
    _hook_get_mode_cb = getMode;
    _hook_enter_cb = enter;
    _hook_exit_cb = exit;
}

/* the 2 following functions are called from hal_os_tick.c iff BSP_POWER_SETUP is set */
/* get the required power sleep mode that the application wants to be in */
int hal_bsp_power_handler_get_mode(os_time_t ticks)
{
    // ask to BSP for the appropriate power mode
    return (_hook_get_mode_cb!=NULL)?(*_hook_get_mode_cb)():HAL_BSP_POWER_WFI;
}

/* enter sleep - called before entering critical region */
void hal_bsp_power_handler_sleep_enter(int nextMode)
{
    if (_hook_enter_cb!=NULL) {
        (*_hook_enter_cb)();
    }

    /* Now BSP deinit                      */
    /* shared bus interfaces               */
    switch(nextMode) {
        
        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP:
        case HAL_BSP_POWER_SLEEP:
          
            /* I2S */
            bsp_deinit_i2s();

            /* I2C */
            hal_bsp_deinit_i2c();

            /* SPI */
            hal_bsp_deinit_spi();

            /*UART */
            hal_bsp_uart_deinit();

            /* Radio */
            hal_bsp_radio_off();

            break;
        case HAL_BSP_POWER_WFI: 

        case HAL_BSP_POWER_ON:
        default:             
           break;
    }

}

/* exit sleep - called after exiting critical region */
void hal_bsp_power_handler_sleep_exit(int lastMode)
{
    /* and tell hook */
    if (_hook_exit_cb!=NULL) {
        (*_hook_exit_cb)();
    }

    /* Now BSP must reinit                 */
    /* shared bus interfaces               */
    switch(lastMode) {
        
        case HAL_BSP_POWER_OFF:
        case HAL_BSP_POWER_DEEP_SLEEP:
        case HAL_BSP_POWER_SLEEP:
     
            /* I2S */
            bsp_init_i2s();

            /* I2C */         
            hal_bsp_init_i2c();

            /* SPI */
            hal_bsp_init_spi();

            /*UART */
            hal_bsp_uart_init();

            /* Radio */
            hal_bsp_radio_on();

            break;
        case HAL_BSP_POWER_WFI: 

        case HAL_BSP_POWER_ON:
        default:             
           break;
    }
}
#else 
void hal_bsp_power_hooks(LP_HOOK_t getMode, LP_HOOK_t enter, LP_HOOK_t exit) {
    // noop
    (void)getMode;
    (void)enter;
    (void)exit;
}
int hal_bsp_power_handler_get_mode(os_time_t ticks) {
    return HAL_BSP_POWER_WFI;
}
void hal_bsp_power_handler_sleep_enter(int nextMode){

}
void hal_bsp_power_handler_sleep_exit(int lastMode){
    
}

#endif
