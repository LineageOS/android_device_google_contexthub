#include <errno.h>
#include <stdint.h>

#include <gpio.h>
#include <i2c.h>
#include <seos.h>
#include <util.h>

#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>
#include <plat/inc/pwr.h>

#define I2C_VERBOSE_DEBUG 0

#if I2C_VERBOSE_DEBUG
#define i2c_log_debug(x) osLog(LOG_DEBUG, x)
#else
#define i2c_log_debug(x) do {} while(0)
#endif

#define I2C_CR1_PE          (1 << 0)
#define I2C_CR1_SMBUS       (1 << 1)
#define I2C_CR1_SMBTYPE     (1 << 3)
#define I2C_CR1_ENARP       (1 << 4)
#define I2C_CR1_ENPEC       (1 << 5)
#define I2C_CR1_ENGC        (1 << 6)
#define I2C_CR1_NOSTRETCH   (1 << 7)
#define I2C_CR1_START       (1 << 8)
#define I2C_CR1_STOP        (1 << 9)
#define I2C_CR1_ACK         (1 << 10)
#define I2C_CR1_POS         (1 << 11)
#define I2C_CR1_PEC         (1 << 12)
#define I2C_CR1_ALERT       (1 << 13)
#define I2C_CR1_SWRST       (1 << 15)

#define I2C_CR2_FREQ(x)     ((x) & I2C_CR2_FREQ_MASK)
#define I2C_CR2_FREQ_MASK   0x3F
#define I2C_CR2_ITERREN     (1 << 8)
#define I2C_CR2_ITEVTEN     (1 << 9)
#define I2C_CR2_ITBUFEN     (1 << 10)
#define I2C_CR2_DMAEN       (1 << 11)
#define I2C_CR2_LAST        (1 << 12)

#define I2C_OAR1_ADD7(x)    (((x) & I2C_OAR1_ADD7_MASK) << 1)
#define I2C_OAR1_ADD7_MASK  0x7F
#define I2C_OAR1_ADD10(x)   ((x) & I2C_OAR1_ADD10_MASK)
#define I2C_OAR1_ADD10_MASK 0x3FF
#define I2C_OAR1_ADDMODE    (1 << 15)

#define I2C_SR1_SB          (1 << 0)
#define I2C_SR1_ADDR        (1 << 1)
#define I2C_SR1_BTF         (1 << 2)
#define I2C_SR1_ADD10       (1 << 3)
#define I2C_SR1_STOPF       (1 << 4)
#define I2C_SR1_RXNE        (1 << 6)
#define I2C_SR1_TXE         (1 << 7)
#define I2C_SR1_BERR        (1 << 8)
#define I2C_SR1_ARLO        (1 << 9)
#define I2C_SR1_AF          (1 << 10)
#define I2C_SR1_OVR         (1 << 11)
#define I2C_SR1_PECERR      (1 << 12)
#define I2C_SR1_TIMEOUT     (1 << 14)
#define I2C_SR1_SMBALERT    (1 << 15)

#define I2C_SR2_MSL         (1 << 0)
#define I2C_SR2_BUSY        (1 << 1)
#define I2C_SR2_TRA         (1 << 2)
#define I2C_SR2_GENCALL     (1 << 4)
#define I2C_SR2_SMBDEFAULT  (1 << 5)
#define I2C_SR2_SMBHOST     (1 << 6)
#define I2C_SR2_DUALF       (1 << 7)

#define I2C_CCR(x)          ((x) & I2C_CCR_MASK)
#define I2C_CCR_MASK        0xFFF
#define I2C_CCR_DUTY_16_9   (1 << 14)
#define I2C_CCR_FM          (1 << 15)

#define I2C_TRISE(x)        ((x) & I2C_TRISE_MASK)
#define I2C_TRISE_MASK      0x3F

struct stm_i2c_regs {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
};

struct stm_i2c_state {
    struct {
        union {
            uint8_t *buf;
            const uint8_t *cbuf;
        };
        size_t size;
        size_t offset;

        i2c_callback_t callback;
        void *cookie;
    } rx, tx;

    enum {
        STM_I2C_DISABLED,
        STM_I2C_SLAVE,
        STM_I2C_MASTER,
    } mode;

    enum {
        STM_I2C_SLAVE_IDLE,
        STM_I2C_SLAVE_RX_ARMED,
        STM_I2C_SLAVE_RX,
        STM_I2C_SLAVE_TX_ARMED,
        STM_I2C_SLAVE_TX,
    } slave_state;

    enum {
        STM_I2C_MASTER_IDLE,
        STM_I2C_MASTER_START,
        STM_I2C_MASTER_TX_ADDR,
        STM_I2C_MASTER_TX_DATA,
        STM_I2C_MASTER_RX_ADDR,
        STM_I2C_MASTER_RX_DATA,
    } master_state;
};

struct stm_i2c_cfg {
    struct stm_i2c_regs *regs;

    uint32_t clock;

    gpio_number_t gpio_scl;
    gpio_number_t gpio_sda;

    IRQn_Type irq_ev;
    IRQn_Type irq_er;
};

struct stm_i2c_device {
    const struct stm_i2c_cfg cfg;
    struct stm_i2c_state state;

    i2c_addr_t addr;

    struct gpio scl;
    struct gpio sda;
};

static inline void stm_i2c_ack_enable(struct stm_i2c_device *pdev)
{
    pdev->cfg.regs->CR1 |= I2C_CR1_ACK;
}

static inline void stm_i2c_ack_disable(struct stm_i2c_device *pdev)
{
    pdev->cfg.regs->CR1 &= ~I2C_CR1_ACK;
}

static inline void stm_i2c_stop_enable(struct stm_i2c_device *pdev)
{
    pdev->cfg.regs->CR1 |= I2C_CR1_STOP;
}

static inline void stm_i2c_irq_enable(struct stm_i2c_device *pdev,
        uint32_t mask)
{
    pdev->cfg.regs->CR2 |= mask;
}

static inline void stm_i2c_irq_disable(struct stm_i2c_device *pdev,
        uint32_t mask)
{
    pdev->cfg.regs->CR2 &= ~mask;
}

static inline void stm_i2c_enable(struct stm_i2c_device *pdev)
{
    pdev->cfg.regs->CR1 |= I2C_CR1_PE;
}

static inline void stm_i2c_disable(struct stm_i2c_device *pdev)
{
    pdev->cfg.regs->CR1 &= ~I2C_CR1_PE;
}

static inline void stm_i2c_set_speed(struct stm_i2c_device *pdev,
        const i2c_speed_t speed)
{
    struct stm_i2c_regs *regs = pdev->cfg.regs;
    int ccr, ccr_1, ccr_2;
    int apb1_clk;

    apb1_clk = pwrGetBusSpeed(PERIPH_BUS_APB1);

    regs->CR2 = (regs->CR2 & ~I2C_CR2_FREQ_MASK) |
                 I2C_CR2_FREQ(apb1_clk / 1000000);

    if (speed <= 100000) {
        ccr = apb1_clk / (speed * 2);
        if (ccr < 4)
            ccr = 4;
        regs->CCR = I2C_CCR(ccr);

        regs->TRISE = I2C_TRISE((apb1_clk / 1000000) + 1);
    } else if (speed <= 400000) {
        ccr_1 = apb1_clk / (speed * 3);
        if (ccr_1 == 0 || apb1_clk / (ccr_1 * 3) > speed)
            ccr_1 ++;
        ccr_2 = apb1_clk / (speed * 25);
        if (ccr_2 == 0 || apb1_clk / (ccr_2 * 25) > speed)
            ccr_2 ++;

        if ((apb1_clk / (ccr_1 * 3)) > (apb1_clk / (ccr_2 * 25)))
            regs->CCR = I2C_CCR_FM | I2C_CCR(ccr_1);
        else
            regs->CCR = I2C_CCR_FM | I2C_CCR_DUTY_16_9 | I2C_CCR(ccr_2);

        regs->TRISE = I2C_TRISE(((3*apb1_clk)/10000000) + 1);
    }
}

static inline void stm_i2c_slave_idle(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;

    state->slave_state = STM_I2C_SLAVE_RX_ARMED;
    stm_i2c_ack_enable(pdev);
    stm_i2c_irq_disable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
}

static inline void stm_i2c_rx_done(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    size_t rx_offset = state->rx.offset;

    stm_i2c_ack_disable(pdev);
    state->rx.offset = 0;
    state->rx.callback(state->rx.cookie, 0, rx_offset);
}

static inline void stm_i2c_tx_done(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    size_t tx_offset = state->tx.offset;

    stm_i2c_slave_idle(pdev);
    state->tx.callback(state->tx.cookie, tx_offset, 0);
}

static inline void stm_i2c_rxtx_done(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    size_t tx_offset = state->tx.offset;
    size_t rx_offset = state->rx.offset;

    state->tx.offset = 0;
    state->rx.offset = 0;
    state->tx.callback(state->tx.cookie, tx_offset, rx_offset);
}

static void stm_i2c_tx_next_byte(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (state->tx.offset < state->tx.size) {
        regs->DR = state->tx.cbuf[state->tx.offset];
        state->tx.offset++;
    } else {
        /* TODO: error on overflow */
    }
}

static void stm_i2c_slave_addr(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    i2c_log_debug("addr");

    if (state->slave_state == STM_I2C_SLAVE_RX_ARMED) {
        state->slave_state = STM_I2C_SLAVE_RX;
        stm_i2c_irq_enable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    }
    /* clear ADDR by doing a dummy reads from SR1 (already read) then SR2 */
    (void)regs->SR2;
}

static void stm_i2c_slave_stopf(struct stm_i2c_device *pdev)
{
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    i2c_log_debug("stopf");

    (void)regs->SR1;
    stm_i2c_enable(pdev);
    /* clear STOPF by doing a dummy read from SR1 and strobing the PE bit */

    stm_i2c_rx_done(pdev);
    stm_i2c_slave_idle(pdev);
}

static inline void stm_i2c_slave_rxne(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;
    uint8_t data = regs->DR;

    i2c_log_debug("rxne");

    if (state->rx.offset < state->rx.size) {
        state->rx.buf[state->rx.offset] = data;
        state->rx.offset++;
    } else {
        stm_i2c_ack_disable(pdev);
        /* TODO: error on overflow */
    }
}

static void stm_i2c_slave_txe(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;

    i2c_log_debug("txe");

    if (state->slave_state == STM_I2C_SLAVE_RX) {
        state->slave_state = STM_I2C_SLAVE_TX_ARMED;
        stm_i2c_irq_disable(pdev, I2C_CR2_ITBUFEN);
        stm_i2c_rx_done(pdev);
        /* stm_i2c_tx_next_byte() will happen when the task provides a
           TX buffer; the I2C controller will stretch the clock until then */
    } else {
        stm_i2c_tx_next_byte(pdev);
    }
}

static void stm_i2c_slave_af(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    i2c_log_debug("af");

    if (state->slave_state == STM_I2C_SLAVE_TX) {
        /* TODO: confirm we're actually at the end of transmission
           (ack failures at the end of transmission are expected) */
        stm_i2c_tx_done(pdev);
    }
    regs->SR1 &= ~I2C_SR1_AF;
}

static void stm_i2c_master_start(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (state->master_state == STM_I2C_MASTER_START) {
        if (state->tx.size > 0) {
            state->master_state = STM_I2C_MASTER_TX_ADDR;
            regs->DR = pdev->addr << 1;
        } else {
            state->master_state = STM_I2C_MASTER_RX_ADDR;
            stm_i2c_ack_enable(pdev);
            regs->DR = (pdev->addr << 1) | 0x01;
        }
    }
}

static void stm_i2c_master_addr(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (state->master_state == STM_I2C_MASTER_TX_ADDR) {
        regs->SR2; // Clear ADDR
        regs->DR = state->tx.cbuf[0];
        state->tx.offset ++;
        state->master_state = STM_I2C_MASTER_TX_DATA;
    } else if (state->master_state == STM_I2C_MASTER_RX_ADDR) {
        if (state->rx.size == 1) // Generate NACK here for 1 byte transfers
            stm_i2c_ack_disable(pdev);
        regs->SR2; // Clear ADDR
        if (state->rx.size == 1) // Generate STOP here for 1 byte transfers
            stm_i2c_stop_enable(pdev);
        state->master_state = STM_I2C_MASTER_RX_DATA;
    }
}

static void stm_i2c_master_rxtx(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (state->master_state == STM_I2C_MASTER_TX_DATA) {
        if (state->tx.offset == state->tx.size) {
            stm_i2c_stop_enable(pdev);
            state->tx.size = 0;
            if (state->rx.size > 0) {
                state->master_state = STM_I2C_MASTER_START;
                regs->CR1 |= I2C_CR1_START;
            } else {
                state->master_state = STM_I2C_MASTER_IDLE;
                stm_i2c_rxtx_done(pdev);
            }
        } else {
            regs->DR = state->tx.cbuf[state->tx.offset];
            state->tx.offset ++;
        }
    } else if (state->master_state == STM_I2C_MASTER_RX_DATA) {
        state->rx.buf[state->rx.offset] = regs->DR;
        state->rx.offset ++;
        // Need to generate NACK + STOP on 2nd to last read
        if (state->rx.offset + 1 == state->rx.size) {
            regs->CR1 = (regs->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
        } else if (state->rx.offset == state->rx.size) {
            state->master_state = STM_I2C_MASTER_IDLE;
            stm_i2c_rxtx_done(pdev);
        }
    }
}

static void stm_i2c_master_af(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if ((state->master_state == STM_I2C_MASTER_TX_ADDR ||
         state->master_state == STM_I2C_MASTER_TX_DATA ||
         state->master_state == STM_I2C_MASTER_RX_ADDR ||
         state->master_state == STM_I2C_MASTER_RX_DATA)) {
        regs->SR1 &= ~I2C_SR1_AF;
        stm_i2c_stop_enable(pdev);
        state->master_state = STM_I2C_MASTER_IDLE;
        stm_i2c_rxtx_done(pdev);
    }
}

static void stm_i2c_ev_isr(struct stm_i2c_device *pdev)
{
    struct stm_i2c_regs *regs = pdev->cfg.regs;
    uint16_t sr1 = regs->SR1;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        if (sr1 & I2C_SR1_ADDR) {
            stm_i2c_slave_addr(pdev);
        } else if (sr1 & I2C_SR1_STOPF) {
            stm_i2c_slave_stopf(pdev);
        } else if (sr1 & I2C_SR1_RXNE) {
            stm_i2c_slave_rxne(pdev);
        } else if (sr1 & I2C_SR1_TXE) {
            stm_i2c_slave_txe(pdev);
        } else if (sr1 & I2C_SR1_BTF) {
            if (regs->SR2 & I2C_SR2_TRA)
                stm_i2c_slave_txe(pdev);
           else
                stm_i2c_slave_rxne(pdev);
        }
        /* TODO: other flags */
    } else if (pdev->state.mode == STM_I2C_MASTER) {
        if (sr1 & I2C_SR1_SB)
            stm_i2c_master_start(pdev);
        else if (sr1 & I2C_SR1_ADDR)
            stm_i2c_master_addr(pdev);
        else if (sr1 & (I2C_SR1_TXE | I2C_SR1_RXNE | I2C_SR1_BTF))
            stm_i2c_master_rxtx(pdev);
    }
}

static void stm_i2c_er_isr(struct stm_i2c_device *pdev)
{
    struct stm_i2c_regs *regs = pdev->cfg.regs;
    uint16_t sr1 = regs->SR1;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        if (sr1 & I2C_SR1_AF)
            stm_i2c_slave_af(pdev);
        /* TODO: other flags */
    } else if (pdev->state.mode == STM_I2C_MASTER) {
        if (sr1 & I2C_SR1_AF)
            stm_i2c_master_af(pdev);
    }
}

#define DECLARE_IRQ_HANDLERS(_n)                \
    extern void I2C##_n##_EV_IRQHandler();      \
    extern void I2C##_n##_ER_IRQHandler();      \
                                                \
    extern void I2C##_n##_EV_IRQHandler()       \
    {                                           \
        stm_i2c_ev_isr(&stm_i2c_devs[_n - 1]);  \
    }                                           \
                                                \
    extern void I2C##_n##_ER_IRQHandler()       \
    {                                           \
        stm_i2c_er_isr(&stm_i2c_devs[_n - 1]);  \
    }

static struct stm_i2c_device stm_i2c_devs[] = {
    [0] = {
        .cfg = {
            .regs = (struct stm_i2c_regs *)I2C1_BASE,

            .clock = PERIPH_APB1_I2C1,

            .gpio_scl = GPIO_PB(8),
            .gpio_sda = GPIO_PB(9),

            .irq_ev = I2C1_EV_IRQn,
            .irq_er = I2C1_ER_IRQn,
        },
    },
};
DECLARE_IRQ_HANDLERS(1)

static inline void stm_i2c_gpio_init(struct gpio *gpio, gpio_number_t num)
{
    gpio_request(gpio, num);
    gpio_configure(gpio, GPIO_MODE_ALTERNATE, GPIO_PULL_NONE);
    gpio_configure_output(gpio, GPIO_OUT_OPEN_DRAIN);
    gpio_assign_func(gpio, GPIO_A2_AFR_I2C);
}

int OS_I2C_master_request(uint8_t bus_id, i2c_speed_t speed)
{
    if (bus_id >= ARRAY_SIZE(stm_i2c_devs))
        return -EINVAL;

    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;

    if (pdev->state.mode == STM_I2C_DISABLED) {
        pdev->state.mode = STM_I2C_MASTER;
        stm_i2c_gpio_init(&pdev->scl, cfg->gpio_scl);
        stm_i2c_gpio_init(&pdev->sda, cfg->gpio_sda);

        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, true);

        stm_i2c_disable(pdev);

        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, true);
        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, false);

        stm_i2c_irq_enable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN |I2C_CR2_ITERREN);
        stm_i2c_set_speed(pdev, speed);
        pdev->state.master_state = STM_I2C_MASTER_IDLE;

        NVIC_EnableIRQ(cfg->irq_er);
        NVIC_EnableIRQ(cfg->irq_ev);

        stm_i2c_enable(pdev);
        return 0;
    } else {
        return -EBUSY;
    }
}

int OS_I2C_master_release(uint8_t bus_id)
{
    if (bus_id >= ARRAY_SIZE(stm_i2c_devs))
        return -EINVAL;

    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;

    if (pdev->state.mode == STM_I2C_MASTER && pdev->state.master_state == STM_I2C_MASTER_IDLE) {
        pdev->state.mode = STM_I2C_DISABLED;
        stm_i2c_irq_disable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
        stm_i2c_disable(pdev);
        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, false);
        return 0;
    } else {
        return -EBUSY;
    }
}

int OS_I2C_master_rxtx(uint8_t bus_id, uint8_t addr,
        const void *tx_buf, size_t tx_size,
        void *rx_buf, size_t rx_size, i2c_callback_t callback, void *cookie)
{
    if (bus_id >= ARRAY_SIZE(stm_i2c_devs))
        return -EINVAL;
    else if (addr & 0x80)
        return -ENXIO;

    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    struct stm_i2c_state *state = &pdev->state;
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (pdev->state.mode == STM_I2C_MASTER) {
        pdev->addr = addr;
        state->tx.cbuf = tx_buf;
        state->tx.offset = 0;
        state->tx.size = tx_size;
        state->tx.callback = callback;
        state->tx.cookie = cookie;
        state->rx.buf = rx_buf;
        state->rx.offset = 0;
        state->rx.size = rx_size;
        state->rx.callback = NULL;
        state->rx.cookie = NULL;
        state->master_state = STM_I2C_MASTER_START;
        regs->CR1 |= I2C_CR1_START;

        return 0;
    } else {
        return -EBUSY;
    }
}

int OS_I2C_slave_request(uint8_t bus_id, i2c_addr_t addr)
{
    if (bus_id >= ARRAY_SIZE(stm_i2c_devs))
        return -EINVAL;

    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;

    if (pdev->state.mode == STM_I2C_DISABLED) {
        pdev->state.mode = STM_I2C_SLAVE;

        pdev->addr = addr;

        stm_i2c_gpio_init(&pdev->scl, cfg->gpio_scl);
        stm_i2c_gpio_init(&pdev->sda, cfg->gpio_sda);

        return 0;
    } else {
        return -EBUSY;
    }
}

int OS_I2C_slave_release(uint8_t bus_id)
{
    if (bus_id >= ARRAY_SIZE(stm_i2c_devs))
        return -EINVAL;

    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];

    if (pdev->state.mode == STM_I2C_SLAVE) {
        pdev->state.mode = STM_I2C_DISABLED;
        return 0;
    } else {
        return -EBUSY;
    }
}

void OS_I2C_slave_enable_rx(uint8_t bus_id, void *rx_buf, size_t size,
        i2c_callback_t callback, void *cookie)
{
    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;
    struct stm_i2c_state *state = &pdev->state;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        state->rx.buf = rx_buf;
        state->rx.offset = 0;
        state->rx.size = size;
        state->rx.callback = callback;
        state->rx.cookie = cookie;
        state->slave_state = STM_I2C_SLAVE_RX_ARMED;

        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, true);
        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, true);
        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, false);

        NVIC_EnableIRQ(cfg->irq_er);
        NVIC_EnableIRQ(cfg->irq_ev);

        stm_i2c_enable(pdev);
        cfg->regs->OAR1 = I2C_OAR1_ADD7(pdev->addr);
        stm_i2c_irq_enable(pdev, I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
        stm_i2c_ack_enable(pdev);
    }
}

int OS_I2C_slave_send(uint8_t bus_id, const void *buf, size_t size,
        i2c_callback_t callback, void *cookie)
{
    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    struct stm_i2c_state *state = &pdev->state;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        if (state->slave_state != STM_I2C_SLAVE_TX_ARMED)
            return -EBUSY;

        state->tx.cbuf = buf;
        state->tx.offset = 0;
        state->tx.size = size;
        state->tx.callback = callback;
        state->tx.cookie = cookie;
        state->slave_state = STM_I2C_SLAVE_TX;

        stm_i2c_tx_next_byte(pdev);
        stm_i2c_irq_enable(pdev, I2C_CR2_ITBUFEN);

        return 0;
    } else {
        return -EBUSY;
    }
}

void OS_I2C_slave_disable(uint8_t bus_id)
{
    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        stm_i2c_irq_disable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
        stm_i2c_ack_disable(pdev);
        stm_i2c_disable(pdev);
        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, false);
    }
}
