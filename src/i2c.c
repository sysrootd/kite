#include "i2c.h"

typedef enum { I2C_IDLE, I2C_TX, I2C_RX, I2C_TXRX } I2C_Mode;

typedef struct {
    I2C_TypeDef *inst;
    uint8_t *tx_buf;
    uint16_t tx_len;
    uint8_t *rx_buf;
    uint16_t rx_len;
    uint8_t addr;
    I2C_Mode mode;
    i2c_callback_t cb_done;
    i2c_error_callback_t cb_err;
} I2C_Context;

static I2C_Context i2c_ctx[2]; // [0] -> I2C1, [1] -> I2C2

static inline I2C_Context* get_ctx(I2C_TypeDef *i2c) {
    return (i2c == I2C1) ? &i2c_ctx[0] : &i2c_ctx[1];
}

void i2c_init(I2C_TypeDef *i2c, uint32_t pclk, uint32_t speed) {
    if (i2c == I2C1) {
        RCC->APB1ENR |= (1U << 21); // I2C1 clock
        gpio_init(GPIOB, 6, AF, OD, FAST, PU, 4);
        gpio_init(GPIOB, 7, AF, OD, FAST, PU, 4);
        NVIC->ISER[0] |= (1U << 31); // I2C1_EV IRQ
        NVIC->ISER[1] |= (1U << 0);  // I2C1_ER IRQ
    } else if (i2c == I2C2) {
        RCC->APB1ENR |= (1U << 22); // I2C2 clock
        gpio_init(GPIOB, 10, AF, OD, FAST, PU, 4);
        gpio_init(GPIOB, 11, AF, OD, FAST, PU, 4);
        NVIC->ISER[1] |= (1U << 1);  // I2C2_EV IRQ
        NVIC->ISER[1] |= (1U << 2);  // I2C2_ER IRQ
    }

    i2c->CR1 |= (1U << 15);
    i2c->CR1 &= ~(1U << 15);

    i2c->CR2 = (pclk / 1000000U);
    i2c->CCR = (pclk / (2 * speed));
    i2c->TRISE = (pclk / 1000000U) + 1;

    i2c->CR1 |= (1U << 10) | (1U << 0); // ACK + PE
    i2c->CR2 |= (1U << 9) | (1U << 10) | (1U << 8); // ITBUFEN, ITEVTEN, ITERREN
}

void i2c_master_transmit_it(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint16_t size,
                            i2c_callback_t cb, i2c_error_callback_t err_cb) {
    I2C_Context *ctx = get_ctx(i2c);
    *ctx = (I2C_Context){ i2c, data, size, 0, 0, (addr << 1), I2C_TX, cb, err_cb };
    i2c->CR1 |= (1U << 8); // START
}

void i2c_master_receive_it(I2C_TypeDef *i2c, uint8_t addr, uint8_t *data, uint16_t size,
                           i2c_callback_t cb, i2c_error_callback_t err_cb) {
    I2C_Context *ctx = get_ctx(i2c);
    *ctx = (I2C_Context){ i2c, 0, 0, data, size, (addr << 1) | 1, I2C_RX, cb, err_cb };
    i2c->CR1 |= (1U << 8); // START
}

void i2c_master_write_read_it(I2C_TypeDef *i2c, uint8_t addr,
                              uint8_t *wdata, uint16_t wsize,
                              uint8_t *rdata, uint16_t rsize,
                              i2c_callback_t cb, i2c_error_callback_t err_cb) {
    I2C_Context *ctx = get_ctx(i2c);
    *ctx = (I2C_Context){ i2c, wdata, wsize, rdata, rsize, (addr << 1), I2C_TXRX, cb, err_cb };
    i2c->CR1 |= (1U << 8); // START
}

static void i2c_ev_handler(I2C_Context *ctx) {
    I2C_TypeDef *i2c = ctx->inst;
    uint32_t sr1 = i2c->SR1;

    if (sr1 & (1U << 0)) { // SB
        i2c->DR = ctx->addr;
    } else if (sr1 & (1U << 1)) { // ADDR
        (void)i2c->SR2;
    } else if (sr1 & (1U << 7)) { // TXE
        if (ctx->tx_len > 0) {
            i2c->DR = *ctx->tx_buf++;
            ctx->tx_len--;
        } else {
            if (ctx->mode == I2C_TXRX) {
                ctx->addr |= 1; // switch to read
                ctx->mode = I2C_RX;
                i2c->CR1 |= (1U << 8); // repeated START
            } else {
                i2c->CR1 |= (1U << 9); // STOP
                if (ctx->cb_done) ctx->cb_done();
            }
        }
    } else if (sr1 & (1U << 6)) { // RXNE
        if (ctx->rx_len > 0) {
            *ctx->rx_buf++ = i2c->DR;
            ctx->rx_len--;
        }
        if (ctx->rx_len == 0) {
            i2c->CR1 &= ~(1U << 10); // ACK off
            i2c->CR1 |= (1U << 9);   // STOP
            if (ctx->cb_done) ctx->cb_done();
        }
    }
}

static void i2c_er_handler(I2C_Context *ctx) {
    uint32_t err = ctx->inst->SR1;
    ctx->inst->SR1 = 0; // clear
    if (ctx->cb_err) ctx->cb_err(err);
}

/* IRQ Handlers */
void I2C1_EV_IRQHandler(void) { i2c_ev_handler(&i2c_ctx[0]); }
void I2C1_ER_IRQHandler(void) { i2c_er_handler(&i2c_ctx[0]); }
void I2C2_EV_IRQHandler(void) { i2c_ev_handler(&i2c_ctx[1]); }
void I2C2_ER_IRQHandler(void) { i2c_er_handler(&i2c_ctx[1]); }
