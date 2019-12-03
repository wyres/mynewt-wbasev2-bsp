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
#include <stm32l1xx_hal_gpio_ex.h>
#include <mcu/stm32l1_bsp.h>
#include "mcu/stm32l1xx_mynewt_hal.h"
#include "mcu/stm32_hal.h"
#if MYNEWT_VAL(RTC)
#include "hal/hal_rtc.h"
#endif

#include "bsp/bsp.h"

// Uart0 is UART1 in STM32 doc hence names of HAL defns
#if MYNEWT_VAL(UART_0)
static struct uart_dev hal_uart0;

static const struct stm32_uart_cfg uart_cfg[UART_CNT] = {
    [0] = {
        .suc_uart = USART1,      
        .suc_rcc_reg = &RCC->APB2ENR,
        .suc_rcc_dev = RCC_APB2ENR_USART1EN,
        .suc_pin_tx = BSP_UART_0_TX, //ok
        .suc_pin_rx = BSP_UART_0_RX, //ok
        .suc_pin_rts = -1,
        .suc_pin_cts = -1,
        .suc_pin_af = GPIO_AF7_USART1,
        .suc_irqn = USART1_IRQn
    }
};
#endif

// UartDbg is bitbang on a gpio
#if MYNEWT_VAL(UART_DBG)
static struct uart_dev hal_uartdbg;
static const struct uart_bitbang_conf uartdbg_cfg = {
    .ubc_rxpin = BSP_UART_DBG_RX,
    .ubc_txpin = BSP_UART_DBG_TX,
    .ubc_cputimer_freq = MYNEWT_VAL(OS_CPUTIME_FREQ),
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
static struct stm32_hal_i2c_cfg i2c0_cfg = {
    .hic_i2c = I2C1,
    .hic_rcc_reg = &RCC->APB1ENR,
    .hic_rcc_dev = RCC_APB1ENR_I2C1EN,
    .hic_pin_sda = I2C_0_SDA,         // ok
    .hic_pin_scl = I2C_0_SCL,         // ok
    .hic_pin_af = GPIO_AF4_I2C1,
    .hic_10bit = 0,
    .hic_speed = I2C_0_FREQUENCY                     // 100kHz 
};
#endif /* USE_BUS_I2C */

#endif

// SPI0 (mynewt) refers to SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_SLAVE) || MYNEWT_VAL(SPI_0_MASTER)
struct stm32_hal_spi_cfg spi0_cfg = {
    .ss_pin   = SPI_0_MASTER_PIN_NSS, 
    .sck_pin  = SPI_0_MASTER_PIN_SCK,
    .miso_pin = SPI_0_MASTER_PIN_MISO,
    .mosi_pin = SPI_0_MASTER_PIN_MOSI, 
    .irq_prio = SPI_0_IRQ_PRIO,
};
#endif
#if MYNEWT_VAL(SPI_1_SLAVE) || MYNEWT_VAL(SPI_1_MASTER)
struct stm32_hal_spi_cfg spi1_cfg = {
    .ss_pin   = SPI_1_MASTER_PIN_NSS,      
    .sck_pin  = SPI_1_MASTER_PIN_SCK,     
    .miso_pin = SPI_1_MASTER_PIN_MISO, 
    .mosi_pin = SPI_1_MASTER_PIN_MOSI, 
    .irq_prio = SPI_1_IRQ_PRIO,
};
#endif

#if MYNEWT_VAL(RTC)
struct stm32_hal_rtc_cfg rtc_cfg = {
    .hrc_hour_fmt = 24,
    .hrc_a_prediv = 31,
    .hrc_s_prediv = 1023
};
#endif


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

static void
clock_config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };

    /* Enable HSI Oscillator and Activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        assert(0);
    }

    /* Set Voltage scale1 as MCU will run at 32MHz */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) ;

    /*
     * Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     * clocks dividers
     */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | \
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        assert(0);
    }
    

#if MYNEWT_VAL(I2C_0) || MYNEWT_VAL(RTC) || MYNEWT_VAL(RNG)
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

#if MYNEWT_VAL(RTC)
     PeriphClkInit.PeriphClockSelection |= RCC_PERIPHCLK_RTC;
     PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
#endif

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        assert(0);
    }
#endif
}

void
hal_bsp_init(void)
{
    int rc;

    (void)rc;

    clock_config();

#if MYNEWT_VAL(UART_0)
    rc = os_dev_create((struct os_dev *) &hal_uart0, UART0_DEV,
      OS_DEV_INIT_PRIMARY, 0, uart_hal_init, (void *)&uart_cfg[0]);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(UART_DBG)
    assert(BSP_UART_DBG_TX!=-1);        // mst define at least tx pin
    rc = os_dev_create((struct os_dev *) &hal_uartdbg, UARTDBG_DEV,
      OS_DEV_INIT_PRIMARY, 0, uart_bitbang_init, (void *)&uartdbg_cfg);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(TIMER_0)
    hal_timer_init(0, TIM2);
#endif

#if MYNEWT_VAL(TIMER_1)
    hal_timer_init(1, TIM3);
#endif

#if MYNEWT_VAL(TIMER_2)
    hal_timer_init(2, TIM4);
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

// note : SPI0 is SPI1 in STM32 doc
#if MYNEWT_VAL(SPI_0_MASTER)
    rc = hal_spi_init(0, &spi0_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_0_SLAVE)
    rc = hal_spi_init(0, &spi0_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_MASTER)
    // MyNewt numbers devices from 0
    rc = hal_spi_init(1, &spi1_cfg, HAL_SPI_TYPE_MASTER);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(SPI_1_SLAVE)
    rc = hal_spi_init(1, &spi1_cfg, HAL_SPI_TYPE_SLAVE);
    assert(rc == 0);
#endif

#if MYNEWT_VAL(RTC)
    rc = hal_rtc_init(&rtc_cfg);
    assert(rc == 0);
#endif

// Note I2C0 is I2C1 in STM32 doc
#if MYNEWT_VAL(I2C_0)
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
#endif
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


#if MYNEWT_VAL(I2C_0) || MYNEWT_VAL(I2C_1) || MYNEWT_VAL(I2C_2)
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
    rc = hal_i2c_init(0, &i2c0_cfg);
#endif
#if MYNEWT_VAL(I2C_1)
    rc = hal_i2c_init(1, &i2c1_cfg);
#endif
#if MYNEWT_VAL(I2C_2)
    rc = hal_i2c_init(2, &i2c2_cfg);
#endif
    return rc;
}
// deinit for power saving
int hal_bsp_deinit_i2c() {
    int rc = 0;
#if MYNEWT_VAL(I2C_0)
    // no hal fn to deinit currently... TODO
//    rc = hal_i2c_deinit(0, &i2c0_cfg);
#endif
#if MYNEWT_VAL(I2C_1)
    rc = hal_i2c_deinit(1, &i2c1_cfg);
#endif
#if MYNEWT_VAL(I2C_2)
    rc = hal_i2c_deinit(2, &i2c2_cfg);
#endif
    return rc;
}
#endif


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
// For W_BASE card, 2 pins are ALWAYS required (revB or revC or later)
void BSP_antSwInit(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    hal_gpio_init_out(txPin, 0);
    hal_gpio_init_out(rxPin, 0);
}
void BSP_antSwDeInit(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    hal_gpio_init_in(txPin, HAL_GPIO_PULL_NONE);
    hal_gpio_init_in(rxPin, HAL_GPIO_PULL_NONE);
}
void BSP_antSwTx(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    if (BSP_getHwVer()<2) {     // proto or revB
        hal_gpio_write(txPin, 1);
        hal_gpio_write(rxPin, 0);
    } else {
        hal_gpio_write(txPin, 0);
        hal_gpio_write(rxPin, 1);
    }
}
void BSP_antSwRx(int txPin, int rxPin) {
    assert(txPin!=-1);
    assert(rxPin!=-1);
    if (BSP_getHwVer()<2) {     // proto or revB
        hal_gpio_write(txPin, 0);
        hal_gpio_write(rxPin, 1);
    } else {
        hal_gpio_write(txPin, 1);
        hal_gpio_write(rxPin, 1);

    }
}

/**
 * Move the system into the specified power state
 *
 * @param state The power state to move the system into, this is one of
 *                 * HAL_BSP_POWER_ON: Full system on
 *                 * HAL_BSP_POWER_WFI: Processor off, wait for interrupt.
 *                 * HAL_BSP_POWER_SLEEP: Put the system to sleep
 *                 * HAL_BSP_POWER_DEEP_SLEEP: Put the system into deep sleep.
 *                 * HAL_BSP_POWER_OFF: Turn off the system.
 *                 * HAL_BSP_POWER_PERUSER: From this value on, allow user
 *                   defined power states.
 *
 * @return 0 on success, non-zero if system cannot move into this power state.
 */
//int hal_bsp_power_state(int state);

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
        // Enable HSI - already done in clock setup
        /* 
        __HAL_RCC_HSI_ENABLE( );

        // Wait till HSI is ready
        while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
        {
        }
        */
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

#define ADC_MAX_VALUE                               4095    // 12 bits max value
// Should read the factory calibrated vref from the eerom at 0x1FF8 00F8/9
#define ADC_VREF_BANDGAP                            1224    // vRef in mV for ADC

int hal_bsp_adc_readmV(int channel) {
    ADC_HandleTypeDef* adch = &_adc1.adcHandle;
    int adcData = 0;


    // Start ADC Software Conversion
    HAL_ADC_Start(adch);

    HAL_ADC_PollForConversion(adch, HAL_MAX_DELAY);

    adcData = HAL_ADC_GetValue(adch);

    int ref_voltage = ( uint32_t )ADC_VREF_BANDGAP * ( uint32_t )ADC_MAX_VALUE;
    // We don't use the VREF from calibValues here.
    // calculate the Voltage in millivolt
    if (adcData>0) {
        adcData = ref_voltage / ( uint32_t )adcData;
    }
//    adcData = (adcData*ADC_VREF_BANDGAP) / ADC_MAX_VALUE;
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

        // Disable HSI - no, required for system clock
//        __HAL_RCC_HSI_DISABLE( );

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

int hal_bsp_adc_readmV(int channel) {
    return 0;
}

void hal_bsp_adc_release(int pin, int chan) {
}
void hal_bsp_adc_deinit() {
}
#endif  /* ADC */