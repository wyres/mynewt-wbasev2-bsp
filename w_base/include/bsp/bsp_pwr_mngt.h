/**
 * Kerlink (support@kerlink.com)
 */

#ifndef BSP_PWR_MNGT_H
#define BSP_PWR_MNGT_H

/* Groups of GPIO available for the STM32L0 */

struct gpio_register {
    GPIO_TypeDef * gpio_group;
    uint32_t moder;
    uint32_t otyper;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t idr;
    uint32_t odr;
    uint32_t bsrr;
    uint32_t lckr;
    uint32_t afrl;
    uint32_t afrh;
    uint32_t brr;
};

/*  RCC management  */
struct rcc_register {
    uint32_t iopenr;
    uint32_t ahbenr;
    uint32_t apb1enr;
    uint32_t apb2enr;
};


void save_gpio_register(struct gpio_register *group);
int restore_gpio_register(struct gpio_register *group);
int restore_one_gpio(struct gpio_register *group, uint16_t gpio);

void save_clock_enable_reg_value(struct rcc_register *rcc_reg);
void restore_clock_enable_reg_value(struct rcc_register *rcc_reg);
void clock_enable_mngt(void);
#endif
