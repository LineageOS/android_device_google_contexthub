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

#define I2C_CR2_FREQ(x)     (x)
#define I2C_CR2_FREQ_MASK   0x3F
#define I2C_CR2_ITERREN     (1 << 8)
#define I2C_CR2_ITEVTEN     (1 << 9)
#define I2C_CR2_ITBUFEN     (1 << 10)
#define I2C_CR2_DMAEN       (1 << 11)
#define I2C_CR2_LAST        (1 << 12)

#define I2C_OAR1_ADD(x)     (x << 1)
#define I2C_OAR1_ADD_MASK   0x3FF
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
        STM_I2C_IDLE,
        STM_I2C_SLAVE_RX_ARMED,
        STM_I2C_SLAVE_RX,
        STM_I2C_SLAVE_TX_ARMED,
        STM_I2C_SLAVE_TX,
    } state;
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

static inline void stm_i2c_slave_idle(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;

    state->state = STM_I2C_SLAVE_RX_ARMED;
    stm_i2c_ack_enable(pdev);
    stm_i2c_irq_disable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
}

static inline void stm_i2c_rx_done(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;

    stm_i2c_ack_disable(pdev);
    state->rx.offset = 0;
    state->rx.callback(state->rx.cookie, 0);
}

static inline void stm_i2c_tx_done(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;

    stm_i2c_slave_idle(pdev);
    state->tx.callback(state->tx.cookie, 0);
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

    if (state->state == STM_I2C_SLAVE_RX_ARMED) {
        state->state = STM_I2C_SLAVE_RX;
        stm_i2c_irq_enable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    }
    (void)regs->SR1;
    (void)regs->SR2;
    /* clear ADDR by doing a dummy reads from SR1 then SR2 */
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
        regs->CR1 &= ~I2C_CR1_ACK;
        /* TODO: error on overflow */
    }
}

static void stm_i2c_slave_txe(struct stm_i2c_device *pdev)
{
    struct stm_i2c_state *state = &pdev->state;

    i2c_log_debug("txe");

    if (state->state == STM_I2C_SLAVE_RX) {
        state->state = STM_I2C_SLAVE_TX_ARMED;
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

    if (state->state == STM_I2C_SLAVE_TX) {
        /* TODO: confirm we're actually at the end of transmission
           (ack failures at the end of transmission are expected) */
        stm_i2c_tx_done(pdev);
    }
    regs->SR1 &= ~I2C_SR1_AF;
}

static void stm_i2c_ev_isr(struct stm_i2c_device *pdev)
{
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (regs->SR1 & I2C_SR1_ADDR)
        stm_i2c_slave_addr(pdev);
    else if (regs->SR1 & I2C_SR1_STOPF)
        stm_i2c_slave_stopf(pdev);
    else if ((regs->SR1 & I2C_SR1_RXNE) ||
            ((regs->SR1 & I2C_SR1_BTF) && !(regs->SR2 & I2C_SR2_TRA)))
        stm_i2c_slave_rxne(pdev);
    else if ((regs->SR1 & I2C_SR1_TXE) ||
            ((regs->SR1 & I2C_SR1_BTF) && (regs->SR2 & I2C_SR2_TRA)))
        stm_i2c_slave_txe(pdev);
    /* TODO: other flags */
}

static void stm_i2c_er_isr(struct stm_i2c_device *pdev)
{
    struct stm_i2c_regs *regs = pdev->cfg.regs;

    if (regs->SR1 & I2C_SR1_AF)
        stm_i2c_slave_af(pdev);
    /* TODO: other flags */
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

int OS_I2C_slave_request(uint8_t bus_id, i2c_speed_t speed, i2c_addr_t addr)
{
    if (bus_id >= ARRAY_SIZE(stm_i2c_devs))
        return -EINVAL;

    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;

    pdev->addr = addr;
    stm_i2c_gpio_init(&pdev->scl, cfg->gpio_scl);
    stm_i2c_gpio_init(&pdev->sda, cfg->gpio_sda);

    return 0;
}

void OS_I2C_slave_enable_rx(uint8_t bus_id, void *rx_buf, size_t size,
        i2c_callback_t callback, void *cookie)
{
    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;
    struct stm_i2c_state *state = &pdev->state;

    state->rx.buf = rx_buf;
    state->rx.offset = 0;
    state->rx.size = size;
    state->rx.callback = callback;
    state->rx.cookie = cookie;
    state->state = STM_I2C_SLAVE_RX_ARMED;

    pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, true);
    pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, true);
    pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, false);

    NVIC_EnableIRQ(cfg->irq_er);
    NVIC_EnableIRQ(cfg->irq_ev);

    stm_i2c_enable(pdev);
    cfg->regs->OAR1 = I2C_OAR1_ADD(pdev->addr);
    stm_i2c_irq_enable(pdev, I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
    stm_i2c_ack_enable(pdev);
}

int OS_I2C_slave_send(uint8_t bus_id, const void *buf, size_t size,
        i2c_callback_t callback, void *cookie)
{
    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    struct stm_i2c_state *state = &pdev->state;

    if (state->state != STM_I2C_SLAVE_TX_ARMED)
        return -EBUSY;

    state->tx.cbuf = buf;
    state->tx.offset = 0;
    state->tx.size = size;
    state->tx.callback = callback;
    state->tx.cookie = cookie;
    state->state = STM_I2C_SLAVE_TX;

    stm_i2c_tx_next_byte(pdev);
    stm_i2c_irq_enable(pdev, I2C_CR2_ITBUFEN);

    return 0;
}

void OS_I2C_slave_disable(uint8_t bus_id)
{
    struct stm_i2c_device *pdev = &stm_i2c_devs[bus_id];
    const struct stm_i2c_cfg *cfg = &pdev->cfg;

    stm_i2c_irq_disable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
    stm_i2c_ack_disable(pdev);
    stm_i2c_disable(pdev);
    pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, false);
}

/*

#define I2C_BS_ADDR_WAS_NAKED     -1
#define I2C_BS_ARBITRATION_FAILED -2

void (*I2cMasterCbk)(void* userData, int32_t txBytesSent, int32_t rxBytesSent); //I2C_BS_* is also possible to get for "*BytesSent"
bool i2cTrans(uint32_t bus, uint8_t addr, const void *txPtr, uint32_t txLen, void *rxPtr, uint32_t rxLen, I2cMasterCbk cbk, void *userData);


*/
