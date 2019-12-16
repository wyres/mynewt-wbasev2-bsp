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
#include "stm32l1xx_hal_rtc.h"
#endif
#if MYNEWT_VAL(BSP_POWER_SETUP)
#include "wyres-generic/lowpowermgr.h"
#endif

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


// MCU Wake Up Time
#define MIN_ALARM_DELAY                             3 // in ticks

// sub-second number of bits
#define N_PREDIV_S                                  10

// Synchronous prediv
#define PREDIV_S                                    ( ( 1 << N_PREDIV_S ) - 1 )

// Asynchronous prediv
#define PREDIV_A                                    ( 1 << ( 15 - N_PREDIV_S ) ) - 1

// Sub-second mask definition
#define ALARM_SUBSECOND_MASK                        ( N_PREDIV_S << RTC_ALRMASSR_MASKSS_Pos )

// RTC Time base in us
#define USEC_NUMBER                                 1000000
#define MSEC_NUMBER                                 ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                               3
#define CONV_NUMER                                  ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                                  ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )


/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                                ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                             ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                            ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                          ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                            ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                               ( ( uint32_t )   24U )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( ( uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( ( uint32_t )0x445550 )

/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )

/*!
 * RTC timer context 
 */
typedef struct
{
    uint32_t        Time;         // Reference time
    RTC_TimeTypeDef CalendarTime; // Reference time in calendar format
    RTC_DateTypeDef CalendarDate; // Reference date in calendar format
}RtcTimerContext_t;


/*!
 * \brief RTC Handle
 */
static RTC_HandleTypeDef RtcHandle = 
{
    .Instance = NULL,
    .Init = 
    { 
        .HourFormat = 0,
        .AsynchPrediv = 0,
        .SynchPrediv = 0,
        .OutPut = 0,
        .OutPutPolarity = 0,
        .OutPutType = 0
    },
    .Lock = HAL_UNLOCKED,
    .State = HAL_RTC_STATE_RESET
};

static RTC_DateTypeDef date;
static RTC_TimeTypeDef time;
/*!
 * \brief RTC Alarm
 */
//static RTC_AlarmTypeDef RtcAlarm;

/**
  * @brief  This function handles  WAKE UP TIMER  interrupt request.
  * @retval None
  */
void RTC_WKUP_IRQHandler(void)
{
  HAL_RTCEx_WakeUpTimerIRQHandler(&RtcHandle);
}

/**
  * @brief  Alarm A callback.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
    hal_gpio_toggle(EXT_IO);
}

/**
  * @brief  Wake Up Timer callback.
  * @param  hrtc: pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    hal_gpio_toggle(EXT_IO);
}

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

#ifdef HIGH_SPEED_EXTERNAL_OSCILLATOR_CLOCK

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* Enable HSI Oscillator and Activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = (RCC_OSCILLATORTYPE_HSE |
                                        RCC_OSCILLATORTYPE_LSE);
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
#else

    /* Enable HSI Oscillator and Activate PLL with HSI as source */
    RCC_OscInitStruct.OscillatorType = (RCC_OSCILLATORTYPE_HSI|
                                        RCC_OSCILLATORTYPE_LSE);
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
#endif


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

    /* Configure the source of time base considering current system clocks settings*/
    HAL_InitTick (TICK_INT_PRIORITY);

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
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_RTC_ENABLE( );

    RtcHandle.Instance            = RTC;
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv   = PREDIV_A;  // RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = PREDIV_S;  // RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    rc = HAL_RTC_Init( &RtcHandle );

    assert(rc == 0);

    date.Year                     = 0;
    date.Month                    = RTC_MONTH_JANUARY;
    date.Date                     = 1;
    date.WeekDay                  = RTC_WEEKDAY_MONDAY;
    HAL_RTC_SetDate( &RtcHandle, &date, RTC_FORMAT_BIN );

    /*at 0:0:0*/
    time.Hours                    = 0;
    time.Minutes                  = 0;
    time.Seconds                  = 0;
    time.SubSeconds               = 0;
    time.TimeFormat               = 0;
    time.StoreOperation           = RTC_STOREOPERATION_RESET;
    time.DayLightSaving           = RTC_DAYLIGHTSAVING_NONE;
    HAL_RTC_SetTime( &RtcHandle, &time, RTC_FORMAT_BIN );

    // Init alarm.
    //HAL_RTC_DeactivateAlarm( &RtcHandle, RTC_ALARM_A );


    // Enable Direct Read of the calendar registers (not through Shadow registers)
    //HAL_RTCEx_EnableBypassShadow( &RtcHandle );

    //HAL_NVIC_SetPriority( RTC_Alarm_IRQn, 1, 0 );
    //HAL_NVIC_EnableIRQ( RTC_Alarm_IRQn );

    // Init alarm.
    //HAL_RTC_DeactivateAlarm( &RtcHandle, RTC_ALARM_A );


    //setup alarm
/*
    RtcAlarm.AlarmTime.Hours            = 0;
    RtcAlarm.AlarmTime.Minutes          = 0;
    RtcAlarm.AlarmTime.Seconds          = 0;
    RtcAlarm.AlarmTime.SubSeconds       = 0;
    RtcAlarm.AlarmTime.TimeFormat       = RTC_HOURFORMAT12_AM;
    RtcAlarm.AlarmTime.DayLightSaving   = RTC_DAYLIGHTSAVING_NONE;
    RtcAlarm.AlarmTime.StoreOperation   = RTC_STOREOPERATION_RESET;
    RtcAlarm.AlarmMask                  = RTC_ALARMMASK_SECONDS;
    RtcAlarm.AlarmSubSecondMask         = RTC_ALARMSUBSECONDMASK_ALL;
    RtcAlarm.AlarmDateWeekDaySel        = RTC_ALARMDATEWEEKDAYSEL_DATE;
    RtcAlarm.AlarmDateWeekDay           = 1;
    RtcAlarm.Alarm                      = RTC_ALARM_A;

    HAL_RTC_SetAlarm_IT(&RtcHandle, &RtcAlarm, FORMAT_BIN);
*/
    /* Enable and set RTC_WKUP_IRQ */
 
    //HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 6, 0);
    //HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    //HAL_NVIC_DisableIRQ(RTC_WKUP_IRQn);

    hal_gpio_deinit(EXT_IO);
    hal_gpio_init_out(EXT_IO, 0);
    hal_gpio_write(EXT_IO, 0);

    NVIC_SetPriority(RTC_WKUP_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    NVIC_SetVector(RTC_WKUP_IRQn, (uint32_t)RTC_WKUP_IRQHandler);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    rc = HAL_RTCEx_SetWakeUpTimer_IT(&RtcHandle, 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

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

int BSP_getHwVer() {
    return MYNEWT_VAL(BSP_HW_REV);      //2;       // RevC
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



#if MYNEWT_VAL(BSP_POWER_SETUP)

// TODO this should be in the OS?
static LP_HOOK_t _hook_exit_cb=NULL;
static LP_HOOK_t _hook_enter_cb=NULL;

//Register a backup timer during tickless sleep
void hal_bsp_power_backup_timer(int os_ticks_per_sec)
{
    //MCU_init_alarm(os_ticks_per_sec, reload_val, prio);
    // TODO : could be replaced by a call to 
    //  hal_timer_init(...)
    //  hal_rtc_init(...) ????
}

//hook idle enter/exit phases. Note the hooks are call with irq disabled in OS critical section - so don't hang about
void hal_bsp_power_hooks(LP_HOOK_t enter, LP_HOOK_t exit)
{
    // Should only have 1 hook of sleeping in the code, so assert if called twice
    assert(_hook_enter_cb==NULL);
    _hook_enter_cb = enter;
    _hook_exit_cb = exit;

}


int hal_bsp_power_handler_enter(os_time_t ticks)
{


    // ask to BSP for the appropriate power mode
    return (_hook_enter_cb!=NULL)?(*_hook_enter_cb)():HAL_BSP_POWER_WFI;
}


int hal_bsp_power_handler_exit(void)
{
    //  Upon waking, sync the OS time.
    // TODO    os_power_sync_time();
    // and tell hook
    if (_hook_exit_cb!=NULL) {
        (*_hook_exit_cb)();
        return 0;
    }

    return -1;
}

#endif



//BRIAN'S 1st DRAFT

#if 0


// TODO : all the code below is not yet operational, essentially coz the linker directives are not in place.
// ToBeTested
// in MCU specific code
static void MCU_init_alarm(uint32_t os_ticks_per_sec, uint32_t reload_val, int prio) {
#if 0
    //  Power on the RTC before using.
    rcc_enable_rtc_clock();
    rtc_interrupt_disable(RTC_SEC);
    rtc_interrupt_disable(RTC_ALR);
    rtc_interrupt_disable(RTC_OW);
    //  rtc_auto_awake() will not reset the RTC when you press the RST button.
    //  It will also continue to count while the MCU is held in reset. If
    //  you want it to reset, use rtc_awake_from_off()
    rtc_awake_from_off(clock_source);  //  This will enable RTC.
    rtc_set_prescale_val(prescale);

    //  Set the RTC time only at power on. Don't set it when waking from standby.
    rtc_set_counter_val(0);              //  Start counting millisecond ticks from 0
    rtc_set_alarm_time((uint32_t) -1);   //  Reset alarm to -1 or 0xffffffff so we don't trigger now
    exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);  //  Enable alarm wakeup via the interrupt
    exti_enable_request(EXTI17);

    NVIC_SetVector(RTC_IRQn,       (uint32_t) rtc_isr);        //  Set the Interrupt Service Routine for RTC
    NVIC_SetVector(RTC_Alarm_IRQn, (uint32_t) rtc_alarm_isr);  //  Set the Interrupt Service Routine for RTC Alarm
    
    nvic_enable_irq(NVIC_RTC_IRQ);        //  Enable RTC tick interrupt processing
    nvic_enable_irq(NVIC_RTC_ALARM_IRQ);  //  Enable RTC alarm wakeup interrupt processing
    __disable_irq();                      //  Disable interrupts while we make changes
    rtc_clear_flag(RTC_SEC);
    rtc_clear_flag(RTC_ALR);
    rtc_clear_flag(RTC_OW);
    rtc_interrupt_enable(RTC_ALR);        //  Allow RTC to generate alarm interrupts
    //  rtc_interrupt_enable(RTC_SEC);    //  Not used: Allow RTC to generate tick interrupts
    __enable_irq();                       //  Enable interrupts
#endif
}
static void MCU_set_alarm(uint32_t millisec) {
#if 0
    //  Set alarm for millisec milliseconds from now.
    volatile uint32_t now = rtc_get_counter_val();

    //  Not documented, but you must disable write protection else the alarm time will not be set and rtc_exit_config_mode() will hang.
    //  TODO: Disable only if write protection is enabled.
    pwr_disable_backup_domain_write_protect();
    rtc_set_alarm_time(now + millisec);
#endif
}

static void MCU_sleep(uint32_t timeMS, int hal_power_state) {
    if (timeMS < 10) { 
        //  no point in sleeping for <10 milliseconds
        // busy wait
        os_time_delay(os_time_ms_to_ticks32(timeMS));
        return;
    }

        //  Stop the system timer.  TODO: Start the timer after sleeping.
    NVIC_DisableIRQ(TIM2_IRQn);

    //  Set the RTC alarm to wake up in `timeMS` milliseconds from now.
    MCU_set_alarm(timeMS);

    // go into relevant power saving mode, to be woken by the alarm
    hal_bsp_power_state(hal_power_state);
}

//  This intercepts all calls to os_tick_init()
void __wrap_os_tick_init(uint32_t os_ticks_per_sec, int prio)
{
    uint32_t reload_val;
    reload_val = ((uint64_t)SystemCoreClock / os_ticks_per_sec) - 1;
    //  Init the power management.
    MCU_init_alarm(os_ticks_per_sec, reload_val, prio);
}

//  This is what os_tick_idle() should be like ie MCU/BSP independant
void __wrap_os_tick_idle(os_time_t ticks) {
    OS_ASSERT_CRITICAL();
    //  Sleep for the number of ticks.
    uint32_t timeMS = os_time_ticks_to_ms32(ticks);
    
    // The hook tells us sleep type we want, or WFI if no hook
    int hal_power_state = _hook_enter_cb!=NULL?(*_hook_enter_cb)():HAL_BSP_POWER_WFI;
    // Do the sleeping - this should BLOCK and return in timeMS 
    MCU_sleep(timeMS, hal_power_state);

    //  Upon waking, sync the OS time.
// TODO    os_power_sync_time();
    // and tell hook
    if (_hook_exit_cb!=NULL) {
        (*_hook_exit_cb)();
    }
}
#endif
