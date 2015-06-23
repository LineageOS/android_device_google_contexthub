#include <errno.h>
#include <string.h>

#include <gpio.h>
#include <spi.h>
#include <spi_priv.h>
#include <util.h>
#include <atomicBitset.h>
#include <atomic.h>

#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>
#include <plat/inc/pwr.h>
#include <plat/inc/exti.h>
#include <plat/inc/syscfg.h>

#define SPI_CR1_CPHA                (1 << 0)
#define SPI_CR1_CPOL                (1 << 1)
#define SPI_CR1_MSTR                (1 << 2)

#define SPI_CR1_BR(x)               ((LOG2_CEIL(x) - 1) << 3)
#define SPI_CR1_BR_MIN              2
#define SPI_CR1_BR_MAX              256
#define SPI_CR1_BR_MASK             (0x7 << 3)

#define SPI_CR1_SPE                 (1 << 6)
#define SPI_CR1_LSBFIRST            (1 << 7)
#define SPI_CR1_SSI                 (1 << 8)
#define SPI_CR1_SSM                 (1 << 9)
#define SPI_CR1_RXONLY              (1 << 10)
#define SPI_CR1_DFF                 (1 << 11)
#define SPI_CR1_BIDIOE              (1 << 14)
#define SPI_CR1_BIDIMODE            (1 << 15)

#define SPI_CR2_TXEIE               (1 << 7)
#define SPI_CR2_RXNEIE              (1 << 6)
#define SPI_CR2_ERRIE               (1 << 5)
#define SPI_CR2_INT_MASK            (SPI_CR2_TXEIE | SPI_CR2_RXNEIE | SPI_CR2_ERRIE)

#define SPI_CR2_SSOE                (1 << 2)

#define SPI_SR_RXNE                 (1 << 0)
#define SPI_SR_TXE                  (1 << 1)
#define SPI_SR_BSY                  (1 << 7)

struct StmSpi {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
};

struct StmSpiState {
    uint8_t bitsPerWord;
    uint8_t xferEnable;
    uint16_t words;

    void *rxBuf;
    const void *txBuf;
    uint16_t rxIdx;
    uint16_t txIdx;

    uint16_t txWord;

    struct ChainedIsr isrNss;
};

struct StmSpiCfg {
    struct StmSpi *regs;

    uint32_t clockBus;
    uint32_t clockUnit;

    GpioNum gpioMiso;
    GpioNum gpioMosi;
    GpioNum gpioSclk;
    GpioNum gpioNss;
    enum GpioAltFunc gpioFunc;

    IRQn_Type irq;
    IRQn_Type irqNss;
};

struct StmSpiDev {
    struct SpiDevice *base;
    const struct StmSpiCfg *cfg;
    struct StmSpiState state;

    struct Gpio miso;
    struct Gpio mosi;
    struct Gpio sck;
    struct Gpio nss;
};

static inline void stmSpiGpioInit(struct Gpio *gpio,
        GpioNum number, enum GpioAltFunc func)
{
    gpioRequest(gpio, number);
    gpioConfigAlt(gpio, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, func);
}

static inline void stmSpiSckPullMode(struct StmSpiDev *pdev,
        enum GpioPullMode sckPull)
{
    gpioConfigAlt(&pdev->sck, GPIO_SPEED_LOW, sckPull, GPIO_OUT_PUSH_PULL, pdev->cfg->gpioFunc);
}

static inline int stmSpiEnable(struct StmSpiDev *pdev,
        const struct SpiMode *mode, bool master)
{
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    if (mode->bitsPerWord != 8 &&
            mode->bitsPerWord != 16)
        return -EINVAL;

    unsigned int div;
    if (master) {
        if (!mode->speed)
            return -EINVAL;

        uint32_t pclk = pwrGetBusSpeed(PERIPH_BUS_AHB1);
        div = pclk / mode->speed;
        if (div > SPI_CR1_BR_MAX)
            return -EINVAL;
        else if (div < SPI_CR1_BR_MIN)
            div = SPI_CR1_BR_MIN;
    }

    atomicWriteByte(&state->xferEnable, false);

    pwrUnitClock(pdev->cfg->clockBus, pdev->cfg->clockUnit, true);

    if (master) {
        regs->CR1 &= ~SPI_CR1_BR_MASK;
        regs->CR1 |= SPI_CR1_BR(div);
    }

    if (mode->cpol == SPI_CPOL_IDLE_LO)
        regs->CR1 &= ~SPI_CR1_CPOL;
    else
        regs->CR1 |= SPI_CR1_CPOL;

    if (mode->cpha == SPI_CPHA_LEADING_EDGE)
        regs->CR1 &= ~SPI_CR1_CPHA;
    else
        regs->CR1 |= SPI_CR1_CPHA;

    if (mode->bitsPerWord == 8)
        regs->CR1 &= ~SPI_CR1_DFF;
    else
        regs->CR1 |= SPI_CR1_DFF;

    if (mode->format == SPI_FORMAT_MSB_FIRST)
        regs->CR1 &= ~SPI_CR1_LSBFIRST;
    else
        regs->CR1 |= SPI_CR1_LSBFIRST;

    if (master)
        regs->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR;
    else
        regs->CR1 &= ~(SPI_CR1_SSM | SPI_CR1_MSTR);

    return 0;
}

static int stmSpiMasterStartSync(struct SpiDevice *dev, spi_cs_t cs,
        const struct SpiMode *mode)
{
    struct StmSpiDev *pdev = dev->pdata;

    int err = stmSpiEnable(pdev, mode, true);
    if (err < 0)
        return err;

    stmSpiSckPullMode(pdev, mode->cpol ? GPIO_PULL_UP : GPIO_PULL_DOWN);

    gpioRequest(&pdev->nss, cs);
    gpioConfigOutput(&pdev->nss, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 0);

    return 0;
}

static int stmSpiSlaveStartSync(struct SpiDevice *dev,
        const struct SpiMode *mode)
{
    struct StmSpiDev *pdev = dev->pdata;
    stmSpiGpioInit(&pdev->nss, pdev->cfg->gpioNss, pdev->cfg->gpioFunc);
    return stmSpiEnable(pdev, mode, false);
}

static inline bool stmSpiIsMaster(struct StmSpiDev *pdev)
{
    struct StmSpi *regs = pdev->cfg->regs;
    return !!(regs->CR1 & SPI_CR1_MSTR);
}

static inline int stmSpiEnableTransfer(struct SpiDevice *dev, void *rxBuf,
        const void *txBuf, size_t size, const struct SpiMode *mode)
{
    struct StmSpiDev *pdev = dev->pdata;
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    if (atomicReadByte(&state->xferEnable) == true)
        return -EBUSY;

    state->bitsPerWord = mode->bitsPerWord;
    state->rxBuf = rxBuf;
    state->rxIdx = 0;
    state->txBuf = txBuf;
    state->txIdx = 0;
    state->txWord = mode->txWord;

    if (mode->bitsPerWord == 8)
        state->words = size;
    else
        state->words = size / 2;

    if (state->words > 0)
        atomicWriteByte(&state->xferEnable, true);

    regs->CR2 |= SPI_CR2_ERRIE | SPI_CR2_RXNEIE | SPI_CR2_TXEIE;

    regs->CR1 |= SPI_CR1_SPE;

    return 0;
}

static int stmSpiRxTx(struct SpiDevice *dev, void *rxBuf, const void *txBuf,
        size_t size, const struct SpiMode *mode)
{
    return stmSpiEnableTransfer(dev, rxBuf, txBuf, size, mode);
}

static int stmSpiSlaveIdle(struct SpiDevice *dev, const struct SpiMode *mode)
{
    return stmSpiEnableTransfer(dev, NULL, NULL, 0, mode);
}

static inline void stmSpiDisable(struct SpiDevice *dev, bool master)
{
    struct StmSpiDev *pdev = dev->pdata;
    struct StmSpi *regs = pdev->cfg->regs;

    while (regs->SR & SPI_SR_BSY)
        ;

    if (master) {
        gpioSet(&pdev->nss, 1);
        stmSpiSckPullMode(pdev, GPIO_PULL_NONE);
    }

    regs->CR1 &= ~SPI_CR1_SPE;
    pwrUnitClock(pdev->cfg->clockBus, pdev->cfg->clockUnit, false);
}

static int stmSpiMasterStopSync(struct SpiDevice *dev)
{
    stmSpiDisable(dev, true);
    return 0;
}

static int stmSpiSlaveStopSync(struct SpiDevice *dev)
{
    stmSpiDisable(dev, false);
    return 0;
}

static bool stmSpiExtiIsr(struct ChainedIsr *isr)
{
    struct StmSpiState *state = container_of(isr, struct StmSpiState, isrNss);
    struct StmSpiDev *pdev = container_of(state, struct StmSpiDev, state);

    if (!extiIsPendingGpio(&pdev->nss))
        return false;

    spiSlaveCsInactive(pdev->base);
    extiClearPendingGpio(&pdev->nss);
    return true;
}

static void stmSpiSlaveSetCsInterrupt(struct SpiDevice *dev, bool enabled)
{
    struct StmSpiDev *pdev = dev->pdata;
    struct ChainedIsr *isr = &pdev->state.isrNss;

    if (enabled) {
        isr->func = stmSpiExtiIsr;

        syscfgSetExtiPort(&pdev->nss);
        extiEnableIntGpio(&pdev->nss, EXTI_TRIGGER_RISING);
        extiChainIsr(pdev->cfg->irqNss, isr);
    } else {
        extiUnchainIsr(pdev->cfg->irqNss, isr);
        extiDisableIntGpio(&pdev->nss);
    }
}

static bool stmSpiSlaveCsIsActive(struct SpiDevice *dev)
{
    struct StmSpiDev *pdev = dev->pdata;
    return gpioGet(&pdev->nss) == 0;
}

static void stmSpiDone(struct StmSpiDev *pdev)
{
    if (stmSpiIsMaster(pdev))
        spiMasterRxTxDone(pdev->base, 0);
    else
        spiSlaveRxTxDone(pdev->base, 0);
}

static inline void stmSpiTxe(struct StmSpiDev *pdev)
{
    struct StmSpiState *state = &pdev->state;
    struct StmSpi *regs = pdev->cfg->regs;

    if (atomicReadByte(&state->xferEnable) == true) {
        if (!state->txBuf) {
            regs->DR = state->txWord;
        } else if (state->bitsPerWord == 8) {
            const uint8_t *txBuf8 = state->txBuf;
            regs->DR = txBuf8[state->txIdx];
        } else {
            const uint16_t *txBuf16 = state->txBuf;
            regs->DR = txBuf16[state->txIdx];
        }

        state->txIdx++;

        if ((!state->txBuf || state->txIdx >= state->words) &&
            (!state->rxBuf || state->rxIdx >= state->words)) {
            atomicWriteByte(&state->xferEnable, false);
            stmSpiDone(pdev);
        }
    } else {
        regs->DR = state->txWord;
    }
}

static void stmSpiRxne(struct StmSpiDev *pdev)
{
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    if (atomicReadByte(&state->xferEnable) == true) {
        if (!state->rxBuf) {
            (void)regs->DR;
        } else if (state->bitsPerWord == 8) {
            uint8_t *rxBuf8 = state->rxBuf;
            rxBuf8[state->rxIdx] = regs->DR;
        } else {
            uint16_t *rxBuf16 = state->rxBuf;
            rxBuf16[state->rxIdx] = regs->DR;
        }

        state->rxIdx++;

        if ((!state->txBuf || state->txIdx >= state->words) &&
            (!state->rxBuf || state->rxIdx >= state->words)) {
            atomicWriteByte(&state->xferEnable, false);
            stmSpiDone(pdev);
        }
    } else {
        (void)regs->DR;
    }
}

static void stmSpiIsr(struct StmSpiDev *pdev)
{
    struct StmSpi *regs = pdev->cfg->regs;

    if (regs->SR & SPI_SR_RXNE) {
        stmSpiRxne(pdev);
    }

    if (regs->SR & SPI_SR_TXE) {
        stmSpiTxe(pdev);
    }

    /* TODO: error conditions */
}

static int stmSpiRelease(struct SpiDevice *dev)
{
    struct StmSpiDev *pdev = dev->pdata;

    NVIC_DisableIRQ(pdev->cfg->irq);

    pdev->base = NULL;
    return 0;
}

#define DECLARE_IRQ_HANDLER(_n)             \
    void SPI##_n##_IRQHandler();            \
    void SPI##_n##_IRQHandler()             \
    {                                       \
        stmSpiIsr(&mStmSpiDevs[_n - 1]); \
    }

const struct SpiDevice_ops mStmSpiOps = {
    .masterStartSync = stmSpiMasterStartSync,
    .masterRxTx = stmSpiRxTx,
    .masterStopSync = stmSpiMasterStopSync,

    .slaveStartSync = stmSpiSlaveStartSync,
    .slaveIdle = stmSpiSlaveIdle,
    .slaveRxTx = stmSpiRxTx,
    .slaveStopSync = stmSpiSlaveStopSync,

    .slaveSetCsInterrupt = stmSpiSlaveSetCsInterrupt,
    .slaveCsIsActive = stmSpiSlaveCsIsActive,

    .release = stmSpiRelease,
};

static const struct StmSpiCfg mStmSpiCfgs[] = {
    [0] = {
        .regs = (struct StmSpi *)SPI1_BASE,

        .clockBus = PERIPH_BUS_APB2,
        .clockUnit = PERIPH_APB2_SPI1,

        .gpioMiso = GPIO_PA(6),
        .gpioMosi = GPIO_PA(7),
        .gpioSclk = GPIO_PA(5),
        .gpioNss = GPIO_PA(4),
        .gpioFunc = GPIO_AF_SPI1,

        .irq = SPI1_IRQn,
        .irqNss = EXTI4_IRQn,
    },
    [1] = {
        .regs = (struct StmSpi *)SPI2_BASE,

        .clockBus = PERIPH_BUS_APB1,
        .clockUnit = PERIPH_APB1_SPI2,

        .gpioMiso = GPIO_PB(14),
        .gpioMosi = GPIO_PB(15),
        .gpioSclk = GPIO_PB(13),
        .gpioNss = GPIO_PB(12),
        .gpioFunc = GPIO_AF_SPI2_A,

        .irq = SPI2_IRQn,
        .irqNss = EXTI15_10_IRQn,
    },
};

static struct StmSpiDev mStmSpiDevs[ARRAY_SIZE(mStmSpiCfgs)];
DECLARE_IRQ_HANDLER(1)
DECLARE_IRQ_HANDLER(2)

static void stmSpiInit(struct StmSpiDev *pdev, const struct StmSpiCfg *cfg,
        struct SpiDevice *dev)
{
    stmSpiGpioInit(&pdev->miso, cfg->gpioMiso, cfg->gpioFunc);
    stmSpiGpioInit(&pdev->mosi, cfg->gpioMosi, cfg->gpioFunc);
    stmSpiGpioInit(&pdev->sck, cfg->gpioSclk, cfg->gpioFunc);

    NVIC_EnableIRQ(cfg->irq);

    pdev->base = dev;
    pdev->cfg = cfg;
}

int spiRequest(struct SpiDevice *dev, uint8_t busId)
{
    if (busId >= ARRAY_SIZE(mStmSpiDevs))
        return -ENODEV;

    struct StmSpiDev *pdev = &mStmSpiDevs[busId];
    const struct StmSpiCfg *cfg = &mStmSpiCfgs[busId];
    if (!pdev->base)
        stmSpiInit(pdev, cfg, dev);

    memset(&pdev->state, 0, sizeof(pdev->state));
    dev->ops = &mStmSpiOps;
    dev->pdata = pdev;
    return 0;
}
