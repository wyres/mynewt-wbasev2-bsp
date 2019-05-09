/**
 * Kerlink (support@kerlink.com)
 */

#include <syscfg/syscfg.h>
#include <os/os_dev.h>
#include <hal/hal_bsp.h>
#include <mcu/mcu.h>
#include "bsp/bsp.h"
#include <assert.h>

#include "bsp/bsp_pwr_mngt.h"

#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_hal_gpio_ex.h"

/* save content of GPIOx registers in group  */
void
save_gpio_register(struct gpio_register *group){

    group->moder = READ_REG(group->gpio_group->MODER);
    group->otyper = READ_REG(group->gpio_group->OTYPER);
    group->ospeedr = READ_REG(group->gpio_group->OSPEEDR);
    group->pupdr = READ_REG(group->gpio_group->PUPDR);
    group->idr = READ_REG(group->gpio_group->IDR);
    group->odr = READ_REG(group->gpio_group->ODR);
    group->bsrr = READ_REG(group->gpio_group->BSRR);
    group->lckr = READ_REG(group->gpio_group->LCKR);
    group->afrl = READ_REG(group->gpio_group->AFR[0]);
    group->afrh = READ_REG(group->gpio_group->AFR[1]);
    group->brr = READ_REG(group->gpio_group->BRR);
}

/* write content of group in GPIOx registers    */
int
restore_gpio_register(struct gpio_register *group){

    if (group == NULL){
        return -1;
    }

    WRITE_REG(group->gpio_group->MODER, group->moder);
    WRITE_REG(group->gpio_group->OTYPER, group->otyper);
    WRITE_REG(group->gpio_group->OSPEEDR, group->ospeedr);
    WRITE_REG(group->gpio_group->PUPDR, group->pupdr);
    WRITE_REG(group->gpio_group->IDR, group->idr);
    WRITE_REG(group->gpio_group->ODR, group->odr);
    WRITE_REG(group->gpio_group->BSRR, group->bsrr);
    WRITE_REG(group->gpio_group->LCKR, group->lckr);
    WRITE_REG(group->gpio_group->AFR[0], group->afrl);
    WRITE_REG(group->gpio_group->AFR[1], group->afrh);
    WRITE_REG(group->gpio_group->BRR, group->brr);

    return 0;
}

int
restore_one_gpio(struct gpio_register *group, uint16_t gpio){

    struct gpio_register group_temp;

    uint16_t port = gpio >> 4;

    if (group == NULL){
        return -1;
    }

    if (port != GPIO_GET_INDEX(group->gpio_group)) {
        return -1;
    }

    group_temp.gpio_group = group->gpio_group;

    /* retrieve the pin info    */
    gpio = gpio & 0x0F;
    gpio = 0x1 << gpio;


    /* Only MODER and PUPDR registers need to be restored   */
    LL_GPIO_SetPinMode(group->gpio_group, gpio, 0);
    LL_GPIO_SetPinPull(group->gpio_group, gpio, 0);

    group_temp.moder = READ_REG(group->gpio_group->MODER) | group->moder;
    group_temp.pupdr = READ_REG(group->gpio_group->PUPDR) | group->pupdr;

    WRITE_REG(group->gpio_group->MODER, group_temp.moder);
    WRITE_REG(group->gpio_group->PUPDR, group_temp.pupdr);

    return 0;
}

/* save content of the clock enable register in rcc_reg    */
void
save_clock_enable_reg_value(struct rcc_register *rcc_reg){

    rcc_reg->ahbenr = READ_REG(RCC->AHBENR);
    rcc_reg->apb1enr = READ_REG(RCC->APB1ENR);
    rcc_reg->apb2enr = READ_REG(RCC->APB2ENR);
}

/* restore content of rcc_reg of in RCC registers    */
void
restore_clock_enable_reg_value(struct rcc_register *rcc_reg){

    WRITE_REG(RCC->AHBENR, rcc_reg->ahbenr);
    WRITE_REG(RCC->APB1ENR, rcc_reg->apb1enr);
    WRITE_REG(RCC->APB2ENR, rcc_reg->apb2enr);
}

/* Let only the LPTIM and PWR clocks enabled */
void
clock_enable_mngt(void){

    /* Buses clock in normal mode   */
    WRITE_REG(RCC->AHBENR, 0);
    WRITE_REG(RCC->APB1ENR, 0x00000000);
    WRITE_REG(RCC->APB2ENR, 0);
}
