#include <errno.h>
#include <stdint.h>

#include <gpio.h>
#include <i2c.h>
#include <seos.h>
#include <util.h>
#include <atomicBitset.h>
#include <atomic.h>

#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>
#include <plat/inc/pwr.h>

#include <cpu/inc/barrier.h>

#define I2C_VERBOSE_DEBUG       0
#define I2C_MAX_QUEUE_DEPTH     5

#if I2C_VERBOSE_DEBUG
#define i2c_log_debug(x) osLog(LOG_DEBUG, x "\n")
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

struct StmI2c {
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

enum StmI2cSpiMasterState
{
    STM_I2C_MASTER_IDLE,
    STM_I2C_MASTER_START,
    STM_I2C_MASTER_TX_ADDR,
    STM_I2C_MASTER_TX_DATA,
    STM_I2C_MASTER_RX_ADDR,
    STM_I2C_MASTER_RX_DATA,
};

struct I2cStmState {
    struct {
        union {
            uint8_t *buf;
            const uint8_t *cbuf;
            uint8_t byte;
        };
        size_t size;
        size_t offset;
        bool preamble;

        I2cCallbackF callback;
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
    } slaveState;

    // StmI2cSpiMasterState
    uint8_t masterState;
};

struct StmI2cCfg {
    struct StmI2c *regs;

    uint32_t clock;

    GpioNum gpioScl;
    uint8_t gpioSclAf;
    GpioNum gpioSda;
    uint8_t gpioSdaAf;
    enum GpioPullMode gpioPull;

    IRQn_Type irqEv;
    IRQn_Type irqEr;
};

struct StmI2cDev {
    const struct StmI2cCfg *cfg;
    struct I2cStmState state;

    uint32_t next;
    uint32_t last;

    I2cAddr addr;

    struct Gpio scl;
    struct Gpio sda;
};

static const struct StmI2cCfg mStmI2cCfgs[] = {
    [0] = {
        .regs = (struct StmI2c *)I2C1_BASE,

        .clock = PERIPH_APB1_I2C1,

        .gpioScl = GPIO_PB(8),
        .gpioSclAf = GPIO_AF_I2C1,
        .gpioSda = GPIO_PB(9),
        .gpioSdaAf = GPIO_AF_I2C1,
        .gpioPull = GPIO_PULL_NONE,

        .irqEv = I2C1_EV_IRQn,
        .irqEr = I2C1_ER_IRQn,
    },
    [1] = {
        .regs = (struct StmI2c *)I2C2_BASE,

        .clock = PERIPH_APB1_I2C2,

        .irqEv = I2C2_EV_IRQn,
        .irqEr = I2C2_ER_IRQn,
    },
    [2] = {
        .regs = (struct StmI2c *)I2C3_BASE,

        .clock = PERIPH_APB1_I2C3,

        .gpioScl = GPIO_PA(8),
        .gpioSclAf = GPIO_AF_I2C3_A,
        .gpioSda = GPIO_PB(4),
        .gpioSdaAf = GPIO_AF_I2C3_B,
        .gpioPull = GPIO_PULL_NONE,

        .irqEv = I2C3_EV_IRQn,
        .irqEr = I2C3_ER_IRQn,
    },
};

static struct StmI2cDev mStmI2cDevs[ARRAY_SIZE(mStmI2cCfgs)];

struct StmI2cXfer
{
    uint32_t        id;
    const void     *txBuf;
    size_t          txSize;
    void           *rxBuf;
    size_t          rxSize;
    I2cCallbackF    callback;
    void           *cookie;
    I2cBus          busId;
    I2cAddr         addr;
};

ATOMIC_BITSET_DECL(mXfersValid, I2C_MAX_QUEUE_DEPTH, static);
static struct StmI2cXfer mXfers[I2C_MAX_QUEUE_DEPTH] = { };

static inline struct StmI2cXfer *stmI2cGetXfer(void)
{
    int32_t idx = atomicBitsetFindClearAndSet(mXfersValid);

    if (idx < 0)
        return NULL;
    else
        return mXfers + idx;
}

static inline void stmI2cPutXfer(struct StmI2cXfer *xfer)
{
    if (xfer)
        atomicBitsetClearBit(mXfersValid, xfer - mXfers);
}

static inline void i2cStmAckEnable(struct StmI2cDev *pdev)
{
    pdev->cfg->regs->CR1 |= I2C_CR1_ACK;
}

static inline void i2cStmAckDisable(struct StmI2cDev *pdev)
{
    pdev->cfg->regs->CR1 &= ~I2C_CR1_ACK;
}

static inline void i2cStmStopEnable(struct StmI2cDev *pdev)
{
    pdev->cfg->regs->CR1 |= I2C_CR1_STOP;
}

static inline void i2cStmIrqEnable(struct StmI2cDev *pdev,
        uint32_t mask)
{
    pdev->cfg->regs->CR2 |= mask;
}

static inline void i2cStmIrqDisable(struct StmI2cDev *pdev,
        uint32_t mask)
{
    pdev->cfg->regs->CR2 &= ~mask;
}

static inline void stmI2cEnable(struct StmI2cDev *pdev)
{
    pdev->cfg->regs->CR1 |= I2C_CR1_PE;
}

static inline void stmI2cDisable(struct StmI2cDev *pdev)
{
    pdev->cfg->regs->CR1 &= ~I2C_CR1_PE;
}

static inline void stmI2cSpeedSet(struct StmI2cDev *pdev,
        const I2cSpeed speed)
{
    struct StmI2c *regs = pdev->cfg->regs;
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

static inline void stmI2cSlaveIdle(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;

    state->slaveState = STM_I2C_SLAVE_RX_ARMED;
    i2cStmAckEnable(pdev);
    i2cStmIrqDisable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
}

static inline void i2cStmRxDone(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    size_t rxOffst = state->rx.offset;

    state->rx.offset = 0;
    state->rx.callback(state->rx.cookie, 0, rxOffst, 0);
}

static inline void i2cStmTxDone(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    size_t txOffst = state->tx.offset;

    stmI2cSlaveIdle(pdev);
    state->tx.callback(state->tx.cookie, txOffst, 0, 0);
}

static inline void i2cStmTxRxDone(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;
    size_t txOffst = state->tx.offset;
    size_t rxOffst = state->rx.offset;
    uint32_t id;
    int i;
    struct StmI2cXfer *xfer;

    state->tx.offset = 0;
    state->rx.offset = 0;
    state->tx.callback(state->tx.cookie, txOffst, rxOffst, 0);

    do {
        id = atomicAdd(&pdev->next, 1);
    } while (!id);

    for (i=0; i<I2C_MAX_QUEUE_DEPTH; i++) {
        xfer = &mXfers[i];

        if (xfer->busId == (pdev - mStmI2cDevs) &&
                atomicCmpXchg32bits(&xfer->id, id, 0)) {
            pdev->addr = xfer->addr;
            state->tx.cbuf = xfer->txBuf;
            state->tx.offset = 0;
            state->tx.size = xfer->txSize;
            state->tx.callback = xfer->callback;
            state->tx.cookie = xfer->cookie;
            state->rx.buf = xfer->rxBuf;
            state->rx.offset = 0;
            state->rx.size = xfer->rxSize;
            state->rx.callback = NULL;
            state->rx.cookie = NULL;
            atomicWriteByte(&state->masterState, STM_I2C_MASTER_START);
            stmI2cPutXfer(xfer);
            regs->CR1 |= I2C_CR1_START;
            return;
        }
    }

    atomicWriteByte(&state->masterState, STM_I2C_MASTER_IDLE);
}

static void i2cStmTxNextByte(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;

    if (state->tx.preamble) {
        regs->DR = state->tx.byte;
        state->tx.offset++;
    } else if (state->tx.offset < state->tx.size) {
        regs->DR = state->tx.cbuf[state->tx.offset];
        state->tx.offset++;
    } else {
        state->slaveState = STM_I2C_SLAVE_TX_ARMED;
        i2cStmIrqDisable(pdev, I2C_CR2_ITBUFEN);
        state->tx.callback(state->tx.cookie, state->tx.offset, 0, 0);
    }
}

static void i2cStmSlaveAddrMatched(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;

    i2c_log_debug("addr");

    if (state->slaveState == STM_I2C_SLAVE_RX_ARMED) {
        state->slaveState = STM_I2C_SLAVE_RX;
        i2cStmIrqEnable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    } else if (state->slaveState == STM_I2C_SLAVE_TX) {
        i2cStmIrqEnable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    }
    /* clear ADDR by doing a dummy reads from SR1 (already read) then SR2 */
    (void)regs->SR2;
}

static void i2cStmSlaveStopRxed(struct StmI2cDev *pdev)
{
    struct StmI2c *regs = pdev->cfg->regs;

    i2c_log_debug("stopf");

    (void)regs->SR1;
    stmI2cEnable(pdev);
    /* clear STOPF by doing a dummy read from SR1 and strobing the PE bit */

    stmI2cSlaveIdle(pdev);
    i2cStmRxDone(pdev);
}

static inline void i2cStmSlaveRxBufNotEmpty(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;
    uint8_t data = regs->DR;

    i2c_log_debug("rxne");

    if (state->rx.offset < state->rx.size) {
        state->rx.buf[state->rx.offset] = data;
        state->rx.offset++;
    } else {
        i2cStmAckDisable(pdev);
        /* TODO: error on overflow */
    }
}

static void i2cStmSlaveTxBufEmpty(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;

    i2c_log_debug("txe");

    if (state->slaveState == STM_I2C_SLAVE_RX) {
        state->slaveState = STM_I2C_SLAVE_TX_ARMED;
        i2cStmIrqDisable(pdev, I2C_CR2_ITBUFEN);
        i2cStmAckDisable(pdev);
        i2cStmRxDone(pdev);
        /* i2cStmTxNextByte() will happen when the task provides a
           TX buffer; the I2C controller will stretch the clock until then */
    } else {
        i2cStmTxNextByte(pdev);
    }
}

static void i2cStmSlaveNakRxed(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;

    i2c_log_debug("af");

    if (state->slaveState == STM_I2C_SLAVE_TX) {
        state->tx.offset--;
        /* NACKs seem to be preceded by a spurious TXNE, so adjust the offset to
           compensate (the corresponding byte written to DR was never actually
           transmitted) */
        i2cStmTxDone(pdev);
    }
    regs->SR1 &= ~I2C_SR1_AF;
}

static void i2cStmMasterSentStart(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;

    if (atomicReadByte(&state->masterState) == STM_I2C_MASTER_START) {
        if (state->tx.size > 0) {
            atomicWriteByte(&state->masterState, STM_I2C_MASTER_TX_ADDR);
            regs->DR = pdev->addr << 1;
        } else {
            atomicWriteByte(&state->masterState, STM_I2C_MASTER_RX_ADDR);
            i2cStmAckEnable(pdev);
            regs->DR = (pdev->addr << 1) | 0x01;
        }
    }
}

static void i2cStmMasterSentAddr(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;
    uint8_t masterState = atomicReadByte(&state->masterState);

    if (masterState == STM_I2C_MASTER_TX_ADDR) {
        regs->SR2; // Clear ADDR
        regs->DR = state->tx.cbuf[0];
        state->tx.offset ++;
        atomicWriteByte(&state->masterState, STM_I2C_MASTER_TX_DATA);
    } else if (masterState == STM_I2C_MASTER_RX_ADDR) {
        if (state->rx.size == 1) // Generate NACK here for 1 byte transfers
            i2cStmAckDisable(pdev);
        regs->SR2; // Clear ADDR
        if (state->rx.size == 1) // Generate STOP here for 1 byte transfers
            i2cStmStopEnable(pdev);
        atomicWriteByte(&state->masterState, STM_I2C_MASTER_RX_DATA);
    }
}

static void i2cStmMasterTxRx(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;
    uint8_t masterState = atomicReadByte(&state->masterState);

    if (masterState == STM_I2C_MASTER_TX_DATA) {
        if (state->tx.offset == state->tx.size) {
            i2cStmStopEnable(pdev);
            state->tx.size = 0;
            if (state->rx.size > 0) {
                atomicWriteByte(&state->masterState, STM_I2C_MASTER_START);
                regs->CR1 |= I2C_CR1_START;
            } else {
                i2cStmTxRxDone(pdev);
            }
        } else {
            regs->DR = state->tx.cbuf[state->tx.offset];
            state->tx.offset ++;
        }
    } else if (masterState == STM_I2C_MASTER_RX_DATA) {
        state->rx.buf[state->rx.offset] = regs->DR;
        state->rx.offset ++;
        // Need to generate NACK + STOP on 2nd to last read
        if (state->rx.offset + 1 == state->rx.size) {
            regs->CR1 = (regs->CR1 & ~I2C_CR1_ACK) | I2C_CR1_STOP;
        } else if (state->rx.offset == state->rx.size) {
            i2cStmTxRxDone(pdev);
        }
    }
}

static void i2cStmMasterNakRxed(struct StmI2cDev *pdev)
{
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;
    uint8_t masterState = atomicReadByte(&state->masterState);

    if (masterState == STM_I2C_MASTER_TX_ADDR ||
            masterState == STM_I2C_MASTER_TX_DATA ||
            masterState == STM_I2C_MASTER_RX_ADDR ||
            masterState == STM_I2C_MASTER_RX_DATA) {
        regs->SR1 &= ~I2C_SR1_AF;
        i2cStmStopEnable(pdev);
        i2cStmTxRxDone(pdev);
    }
}

static void stmI2cIsrEvent(struct StmI2cDev *pdev)
{
    struct StmI2c *regs = pdev->cfg->regs;
    uint16_t sr1 = regs->SR1;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        if (sr1 & I2C_SR1_ADDR) {
            i2cStmSlaveAddrMatched(pdev);
        } else if (sr1 & I2C_SR1_RXNE) {
            i2cStmSlaveRxBufNotEmpty(pdev);
        } else if (sr1 & I2C_SR1_TXE) {
            i2cStmSlaveTxBufEmpty(pdev);
        } else if (sr1 & I2C_SR1_BTF) {
            if (regs->SR2 & I2C_SR2_TRA)
                i2cStmSlaveTxBufEmpty(pdev);
           else
                i2cStmSlaveRxBufNotEmpty(pdev);
        } else if (sr1 & I2C_SR1_STOPF) {
            i2cStmSlaveStopRxed(pdev);
        }
        /* TODO: other flags */
    } else if (pdev->state.mode == STM_I2C_MASTER) {
        if (sr1 & I2C_SR1_SB)
            i2cStmMasterSentStart(pdev);
        else if (sr1 & I2C_SR1_ADDR)
            i2cStmMasterSentAddr(pdev);
        else if (sr1 & (I2C_SR1_TXE | I2C_SR1_RXNE | I2C_SR1_BTF))
            i2cStmMasterTxRx(pdev);
    }
}

static void stmI2cIsrError(struct StmI2cDev *pdev)
{
    struct StmI2c *regs = pdev->cfg->regs;
    uint16_t sr1 = regs->SR1;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        if (sr1 & I2C_SR1_AF)
            i2cStmSlaveNakRxed(pdev);
        /* TODO: other flags */
    } else if (pdev->state.mode == STM_I2C_MASTER) {
        if (sr1 & I2C_SR1_AF)
            i2cStmMasterNakRxed(pdev);
    }
}

#define DECLARE_IRQ_HANDLERS(_n)                \
    extern void I2C##_n##_EV_IRQHandler();      \
    extern void I2C##_n##_ER_IRQHandler();      \
                                                \
    extern void I2C##_n##_EV_IRQHandler()       \
    {                                           \
        stmI2cIsrEvent(&mStmI2cDevs[_n - 1]);  \
    }                                           \
                                                \
    extern void I2C##_n##_ER_IRQHandler()       \
    {                                           \
        stmI2cIsrError(&mStmI2cDevs[_n - 1]);  \
    }

DECLARE_IRQ_HANDLERS(1);
DECLARE_IRQ_HANDLERS(3);

static inline void stmI2cGpioInit(struct Gpio *gpio, GpioNum num,
        enum GpioPullMode pull, enum GpioAltFunc func)
{
    gpioRequest(gpio, num);
    gpioConfigAlt(gpio, pull, GPIO_OUT_OPEN_DRAIN, func);
}

int i2cMasterRequest(I2cBus busId, I2cSpeed speed)
{
    if (busId >= ARRAY_SIZE(mStmI2cDevs))
        return -EINVAL;

    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    struct I2cStmState *state = &pdev->state;
    const struct StmI2cCfg *cfg = &mStmI2cCfgs[busId];

    if (state->mode == STM_I2C_DISABLED) {
        state->mode = STM_I2C_MASTER;

        pdev->cfg = cfg;
        pdev->next = 2;
        pdev->last = 1;
        atomicBitsetInit(mXfersValid, I2C_MAX_QUEUE_DEPTH);

        stmI2cGpioInit(&pdev->scl, cfg->gpioScl, cfg->gpioPull, cfg->gpioSclAf);
        stmI2cGpioInit(&pdev->sda, cfg->gpioSda, cfg->gpioPull, cfg->gpioSdaAf);

        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, true);

        stmI2cDisable(pdev);

        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, true);
        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, false);

        i2cStmIrqEnable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);
        stmI2cSpeedSet(pdev, speed);
        atomicWriteByte(&state->masterState, STM_I2C_MASTER_IDLE);

        NVIC_EnableIRQ(cfg->irqEr);
        NVIC_EnableIRQ(cfg->irqEv);

        stmI2cEnable(pdev);
        return 0;
    } else {
        return -EBUSY;
    }
}

int i2cMasterRelease(I2cBus busId)
{
    if (busId >= ARRAY_SIZE(mStmI2cDevs))
        return -EINVAL;

    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    struct I2cStmState *state = &pdev->state;
    const struct StmI2cCfg *cfg = pdev->cfg;

    if (state->mode == STM_I2C_MASTER) {
        if (atomicCmpXchgByte((uint8_t *)&state->masterState,
                STM_I2C_MASTER_IDLE, STM_I2C_DISABLED)) {
            i2cStmIrqDisable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
            stmI2cDisable(pdev);
            pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, false);
            return 0;
        } else {
            return -EBUSY;
        }
    } else {
        return -EINVAL;
    }
}


int i2cMasterTxRx(I2cBus busId, I2cAddr addr,
        const void *txBuf, size_t txSize, void *rxBuf, size_t rxSize,
        I2cCallbackF callback, void *cookie)
{
    uint32_t id;

    if (busId >= ARRAY_SIZE(mStmI2cDevs))
        return -EINVAL;
    else if (addr & 0x80)
        return -ENXIO;

    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    struct I2cStmState *state = &pdev->state;
    struct StmI2c *regs = pdev->cfg->regs;

    if (state->mode != STM_I2C_MASTER)
        return -EINVAL;

    struct StmI2cXfer *xfer = stmI2cGetXfer();

    if (xfer) {
        xfer->busId = busId;
        xfer->addr = addr;
        xfer->txBuf = txBuf;
        xfer->txSize = txSize;
        xfer->rxBuf = rxBuf;
        xfer->rxSize = rxSize;
        xfer->callback = callback;
        xfer->cookie = cookie;

        do {
            id = atomicAdd(&pdev->last, 1);
        } while (!id);

        // after this point the transfer can be picked up by the transfer
        // complete interrupt
        atomicWrite32bits(&xfer->id, id);

        // only initiate transfer here if we are in IDLE. Otherwise the transfer
        // completion interrupt will start the next transfer (not necessarily
        // this one)
        if (atomicCmpXchgByte((uint8_t *)&state->masterState,
                STM_I2C_MASTER_IDLE, STM_I2C_MASTER_START)) {
            // it is possible for this transfer to already be complete by the
            // time we get here. if so, transfer->id will have been set to 0.
            if (atomicCmpXchg32bits(&xfer->id, id, 0)) {
                pdev->addr = xfer->addr;
                state->tx.cbuf = xfer->txBuf;
                state->tx.offset = 0;
                state->tx.size = xfer->txSize;
                state->tx.callback = xfer->callback;
                state->tx.cookie = xfer->cookie;
                state->rx.buf = xfer->rxBuf;
                state->rx.offset = 0;
                state->rx.size = xfer->rxSize;
                state->rx.callback = NULL;
                state->rx.cookie = NULL;
                stmI2cPutXfer(xfer);
                regs->CR1 |= I2C_CR1_START;
            }
        }
        return 0;
    } else {
        return -EBUSY;
    }
}

int i2cSlaveRequest(I2cBus busId, I2cAddr addr)
{
    if (busId >= ARRAY_SIZE(mStmI2cDevs))
        return -EINVAL;

    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    const struct StmI2cCfg *cfg = &mStmI2cCfgs[busId];

    if (pdev->state.mode == STM_I2C_DISABLED) {
        pdev->state.mode = STM_I2C_SLAVE;

        pdev->addr = addr;
        pdev->cfg = cfg;

        stmI2cGpioInit(&pdev->scl, cfg->gpioScl, cfg->gpioPull, cfg->gpioSclAf);
        stmI2cGpioInit(&pdev->sda, cfg->gpioSda, cfg->gpioPull, cfg->gpioSdaAf);

        return 0;
    } else {
        return -EBUSY;
    }
}

int i2cSlaveRelease(I2cBus busId)
{
    if (busId >= ARRAY_SIZE(mStmI2cDevs))
        return -EINVAL;

    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    const struct StmI2cCfg *cfg = pdev->cfg;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        pdev->state.mode = STM_I2C_DISABLED;
        i2cStmIrqDisable(pdev, I2C_CR2_ITBUFEN | I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
        i2cStmAckDisable(pdev);
        stmI2cDisable(pdev);
        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, false);
        return 0;
    } else {
        return -EBUSY;
    }
}

void i2cSlaveEnableRx(I2cBus busId, void *rxBuf, size_t rxSize,
        I2cCallbackF callback, void *cookie)
{
    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    const struct StmI2cCfg *cfg = pdev->cfg;
    struct I2cStmState *state = &pdev->state;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        state->rx.buf = rxBuf;
        state->rx.offset = 0;
        state->rx.size = rxSize;
        state->rx.callback = callback;
        state->rx.cookie = cookie;
        state->slaveState = STM_I2C_SLAVE_RX_ARMED;

        pwrUnitClock(PERIPH_BUS_APB1, cfg->clock, true);
        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, true);
        pwrUnitReset(PERIPH_BUS_APB1, cfg->clock, false);

        NVIC_EnableIRQ(cfg->irqEr);
        NVIC_EnableIRQ(cfg->irqEv);

        stmI2cEnable(pdev);
        cfg->regs->OAR1 = I2C_OAR1_ADD7(pdev->addr);
        i2cStmIrqEnable(pdev, I2C_CR2_ITERREN | I2C_CR2_ITEVTEN);
        i2cStmAckEnable(pdev);
    }
}

static int i2cSlaveTx(I2cBus busId, const void *txBuf, uint8_t byte,
        size_t txSize, I2cCallbackF callback, void *cookie)
{
    struct StmI2cDev *pdev = &mStmI2cDevs[busId];
    struct I2cStmState *state = &pdev->state;

    if (pdev->state.mode == STM_I2C_SLAVE) {
        if (state->slaveState == STM_I2C_SLAVE_RX)
            return -EBUSY;

        if (txBuf) {
            state->tx.cbuf = txBuf;
            state->tx.preamble = false;
        } else {
            state->tx.byte = byte;
            state->tx.preamble = true;
        }
        state->tx.offset = 0;
        state->tx.size = txSize;
        state->tx.callback = callback;
        state->tx.cookie = cookie;

        if (state->slaveState == STM_I2C_SLAVE_TX_ARMED) {
            state->slaveState = STM_I2C_SLAVE_TX;
            i2cStmTxNextByte(pdev);
            i2cStmIrqEnable(pdev, I2C_CR2_ITBUFEN);
        } else {
            state->slaveState = STM_I2C_SLAVE_TX;
        }

        return 0;
    } else {
        return -EBUSY;
    }
}

int i2cSlaveTxPreamble(I2cBus busId, uint8_t byte, I2cCallbackF callback,
        void *cookie)
{
    return i2cSlaveTx(busId, NULL, byte, 0, callback, cookie);
}

int i2cSlaveTxPacket(I2cBus busId, const void *txBuf, size_t txSize,
        I2cCallbackF callback, void *cookie)
{
    return i2cSlaveTx(busId, txBuf, 0, txSize, callback, cookie);
}
