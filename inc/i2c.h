#ifndef I2C_H
#define I2C_H

#include <stdint.h>
#include "stm32f401.h"
#include "gpio.h"

typedef void (*i2c_callback_t)(void);
typedef void (*i2c_error_callback_t)(uint32_t sr1);

void i2c_init(I2C_TypeDef *i2c, uint32_t pclk, uint32_t speed);

void i2c_master_transmit_it(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint16_t size,
                            i2c_callback_t cb, i2c_error_callback_t err_cb);

void i2c_master_receive_it(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint16_t size,
                           i2c_callback_t cb, i2c_error_callback_t err_cb);
                           
void i2c_master_write_read_it(I2C_TypeDef *i2c, uint8_t addr,
                              uint8_t *wdata, uint16_t wsize,
                              uint8_t *rdata, uint16_t rsize,
                              i2c_callback_t cb, i2c_error_callback_t err_cb);

void I2C1_EV_IRQHandler(void);
void I2C1_ER_IRQHandler(void);
void I2C2_EV_IRQHandler(void);
void I2C2_ER_IRQHandler(void);

#endif
