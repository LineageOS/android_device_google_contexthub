#include <errno.h>
#include <string.h>

#include <gpio.h>
#include <spi.h>
#include <spi_priv.h>
#include <util.h>
#include <atomicBitset.h>
#include <atomic.h>
#include <platform.h>

#include <plat/inc/cmsis.h>
#include <plat/inc/dma.h>
#include <plat/inc/gpio.h>
#include <plat/inc/pwr.h>
#include <plat/inc/exti.h>
#include <plat/inc/syscfg.h>
#include <plat/inc/spi.h>
#include <plat/inc/plat.h>

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
#define SPI_CR2_TXDMAEN             (1 << 1)
#define SPI_CR2_RXDMAEN             (1 << 0)
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

    uint16_t txWord;

    bool rxDone;
    bool txDone;

    struct ChainedIsr isrNss;

    bool nssChange;
};

struct StmSpiCfg {
    struct StmSpi *regs;

    uint32_t clockBus;
    uint32_t clockUnit;

    IRQn_Type irq;

    uint8_t dmaBus;
};

struct StmSpiDev {
    struct SpiDevice *base;
    const struct StmSpiCfg *cfg;
    const struct StmSpiBoardCfg *board;
    struct StmSpiState state;

    struct Gpio miso;
    struct Gpio mosi;
    struct Gpio sck;
    struct Gpio nss;
};

static inline void stmSpiGpioInit(struct Gpio *gpio,
        GpioNum number, enum GpioSpeed speed, enum GpioAltFunc func)
{
    gpioRequest(gpio, number);
    gpioConfigAlt(gpio, speed, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, func);
}

static inline void stmSpiDataPullMode(struct StmSpiDev *pdev,
        enum GpioSpeed dataSpeed, enum GpioPullMode dataPull)
{
    gpioConfigAlt(&pdev->miso, dataSpeed, dataPull, GPIO_OUT_PUSH_PULL, pdev->board->gpioFunc);
    gpioConfigAlt(&pdev->mosi, dataSpeed, dataPull, GPIO_OUT_PUSH_PULL, pdev->board->gpioFunc);
}

static inline void stmSpiSckPullMode(struct StmSpiDev *pdev,
        enum GpioSpeed sckSpeed, enum GpioPullMode sckPull)
{
    gpioConfigAlt(&pdev->sck, sckSpeed, sckPull, GPIO_OUT_PUSH_PULL, pdev->board->gpioFunc);
}

static inline void stmSpiStartDma(struct StmSpiDev *pdev,
        const struct StmSpiDmaCfg *dmaCfg, const void *buf, uint8_t bitsPerWord,
        bool minc, size_t size, DmaCallbackF callback, bool rx)
{
    struct StmSpi *regs = pdev->cfg->regs;
    struct dmaMode mode;

    memset(&mode, 0, sizeof(mode));

    if (bitsPerWord == 8) {
        mode.psize = DMA_SIZE_8_BITS;
        mode.msize = DMA_SIZE_8_BITS;
    } else {
        mode.psize = DMA_SIZE_16_BITS;
        mode.msize = DMA_SIZE_16_BITS;
    }
    mode.priority = DMA_PRIORITY_HIGH;
    mode.direction = rx ? DMA_DIRECTION_PERIPH_TO_MEM :
            DMA_DIRECTION_MEM_TO_PERIPH;
    mode.periphAddr = (uintptr_t)&regs->DR;
    mode.minc = minc;
    mode.channel = dmaCfg->channel;

    dmaStart(pdev->cfg->dmaBus, dmaCfg->stream, buf, size, &mode, callback,
            pdev);
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

    state->txWord = mode->txWord;
    state->bitsPerWord = mode->bitsPerWord;

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

    stmSpiDataPullMode(pdev, pdev->board->gpioSpeed, pdev->board->gpioPull);
    stmSpiSckPullMode(pdev, pdev->board->gpioSpeed, mode->cpol ? GPIO_PULL_UP : GPIO_PULL_DOWN);

    gpioRequest(&pdev->nss, cs);
    gpioConfigOutput(&pdev->nss, pdev->board->gpioSpeed, pdev->board->gpioPull, GPIO_OUT_PUSH_PULL, 1);

    return 0;
}

static int stmSpiSlaveStartSync(struct SpiDevice *dev,
        const struct SpiMode *mode)
{
    struct StmSpiDev *pdev = dev->pdata;

    stmSpiDataPullMode(pdev, pdev->board->gpioSpeed, GPIO_PULL_NONE);
    stmSpiSckPullMode(pdev, pdev->board->gpioSpeed, GPIO_PULL_NONE);

    stmSpiGpioInit(&pdev->nss, pdev->board->gpioNss, pdev->board->gpioSpeed, pdev->board->gpioFunc);
    return stmSpiEnable(pdev, mode, false);
}

static inline bool stmSpiIsMaster(struct StmSpiDev *pdev)
{
    struct StmSpi *regs = pdev->cfg->regs;
    return !!(regs->CR1 & SPI_CR1_MSTR);
}

static void stmSpiDone(struct StmSpiDev *pdev, int err)
{
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    platReleaseDevInSleepMode(Stm32sleepSpiXfer);

    while (regs->SR & SPI_SR_BSY)
        ;

    if (stmSpiIsMaster(pdev)) {
        if (state->nssChange)
            gpioSet(&pdev->nss, 1);
        spiMasterRxTxDone(pdev->base, err);
    } else {
        regs->CR2 = SPI_CR2_TXEIE;
        spiSlaveRxTxDone(pdev->base, err);
    }
}

static void stmSpiRxDone(void *cookie, uint16_t bytesLeft, int err)
{
    struct StmSpiDev *pdev = cookie;
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    regs->CR2 &= ~SPI_CR2_RXDMAEN;
    state->rxDone = true;

    if (state->txDone) {
        atomicWriteByte(&state->xferEnable, false);
        stmSpiDone(pdev, err);
    }
}

static void stmSpiTxDone(void *cookie, uint16_t bytesLeft, int err)
{
    struct StmSpiDev *pdev = cookie;
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    regs->CR2 &= ~SPI_CR2_TXDMAEN;
    state->txDone = true;

    if (state->rxDone) {
        atomicWriteByte(&state->xferEnable, false);
        stmSpiDone(pdev, err);
    }
}

static int stmSpiRxTx(struct SpiDevice *dev, void *rxBuf, const void *txBuf,
        size_t size, const struct SpiMode *mode)
{
    struct StmSpiDev *pdev = dev->pdata;
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;
    bool txMinc = true;
    uint32_t cr2 = SPI_CR2_TXDMAEN;

    if (atomicXchgByte(&state->xferEnable, true) == true)
        return -EBUSY;

    if (stmSpiIsMaster(pdev))
        gpioSet(&pdev->nss, 0);

    state->rxDone = !rxBuf;
    state->txDone = false;
    state->nssChange = mode->nssChange;

    if (rxBuf) {
        stmSpiStartDma(pdev, &pdev->board->dmaRx, rxBuf, mode->bitsPerWord,
                true, size, stmSpiRxDone, true);
        cr2 |= SPI_CR2_RXDMAEN;
    }

    if (!txBuf) {
        txBuf = &state->txWord;
        txMinc = false;
    }
    stmSpiStartDma(pdev, &pdev->board->dmaTx, txBuf, mode->bitsPerWord, txMinc,
            size, stmSpiTxDone, false);

    /* Ensure the TXE and RXNE bits are cleared; otherwise the DMA controller
     * may "receive" the byte sitting in the SPI controller's FIFO right now,
     * or drop/corrupt the first TX byte.  Timing is crucial here, so do it
     * right before enabling DMA.
     */
    if (!stmSpiIsMaster(pdev)) {
        regs->CR2 &= ~SPI_CR2_TXEIE;

        if (regs->SR & SPI_SR_RXNE)
            (void)regs->DR;

        if (regs->SR & SPI_SR_TXE)
            regs->DR = mode->txWord;
    }

    regs->CR2 = cr2;
    regs->CR1 |= SPI_CR1_SPE;

    platRequestDevInSleepMode(Stm32sleepSpiXfer, 12);

    return 0;
}

static int stmSpiSlaveIdle(struct SpiDevice *dev, const struct SpiMode *mode)
{
    struct StmSpiDev *pdev = dev->pdata;
    struct StmSpi *regs = pdev->cfg->regs;
    struct StmSpiState *state = &pdev->state;

    if (atomicXchgByte(&state->xferEnable, true) == true)
        return -EBUSY;

    regs->CR2 = SPI_CR2_TXEIE;
    regs->CR1 |= SPI_CR1_SPE;

    atomicXchgByte(&state->xferEnable, false);
    return 0;
}

static inline void stmSpiDisable(struct SpiDevice *dev, bool master)
{
    struct StmSpiDev *pdev = dev->pdata;
    struct StmSpi *regs = pdev->cfg->regs;

    while (regs->SR & SPI_SR_BSY)
        ;

    if (master) {
        gpioSet(&pdev->nss, 1);
        stmSpiSckPullMode(pdev, pdev->board->gpioSpeed, pdev->board->gpioPull);
    }

    regs->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN | SPI_CR2_TXEIE);
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
        extiChainIsr(pdev->board->irqNss, isr);
    } else {
        extiUnchainIsr(pdev->board->irqNss, isr);
        extiDisableIntGpio(&pdev->nss);
    }
}

static bool stmSpiSlaveCsIsActive(struct SpiDevice *dev)
{
    struct StmSpiDev *pdev = dev->pdata;
    return gpioGet(&pdev->nss) == 0;
}

static inline void stmSpiTxe(struct StmSpiDev *pdev)
{
    struct StmSpi *regs = pdev->cfg->regs;

    /**
     * n.b.: if nothing handles the TXE interrupt in slave mode, the SPI
     * controller will just keep reading the existing value from DR anytime it
     * needs data
     */
    regs->DR = pdev->state.txWord;
    regs->CR2 &= ~SPI_CR2_TXEIE;
}

static void stmSpiIsr(struct StmSpiDev *pdev)
{
    struct StmSpi *regs = pdev->cfg->regs;

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

        .irq = SPI1_IRQn,

        .dmaBus = SPI1_DMA_BUS,
    },
    [1] = {
        .regs = (struct StmSpi *)SPI2_BASE,

        .clockBus = PERIPH_BUS_APB1,
        .clockUnit = PERIPH_APB1_SPI2,

        .irq = SPI2_IRQn,

        .dmaBus = SPI2_DMA_BUS,
    },
};

static struct StmSpiDev mStmSpiDevs[ARRAY_SIZE(mStmSpiCfgs)];
DECLARE_IRQ_HANDLER(1)
DECLARE_IRQ_HANDLER(2)

static void stmSpiInit(struct StmSpiDev *pdev, const struct StmSpiCfg *cfg,
        const struct StmSpiBoardCfg *board, struct SpiDevice *dev)
{
    stmSpiGpioInit(&pdev->miso, board->gpioMiso, board->gpioSpeed, board->gpioFunc);
    stmSpiGpioInit(&pdev->mosi, board->gpioMosi, board->gpioSpeed, board->gpioFunc);
    stmSpiGpioInit(&pdev->sck, board->gpioSclk, board->gpioSpeed, board->gpioFunc);

    NVIC_EnableIRQ(cfg->irq);

    pdev->base = dev;
    pdev->cfg = cfg;
    pdev->board = board;
}

int spiRequest(struct SpiDevice *dev, uint8_t busId)
{
    if (busId >= ARRAY_SIZE(mStmSpiDevs))
        return -ENODEV;

    const struct StmSpiBoardCfg *board = boardStmSpiCfg(busId);
    if (!board)
        return -ENODEV;

    struct StmSpiDev *pdev = &mStmSpiDevs[busId];
    const struct StmSpiCfg *cfg = &mStmSpiCfgs[busId];
    if (!pdev->base)
        stmSpiInit(pdev, cfg, board, dev);

    memset(&pdev->state, 0, sizeof(pdev->state));
    dev->ops = &mStmSpiOps;
    dev->pdata = pdev;
    return 0;
}
