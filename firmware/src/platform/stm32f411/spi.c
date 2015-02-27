#include <errno.h>
#include <string.h>

#include <gpio.h>
#include <spi.h>
#include <spi_priv.h>
#include <util.h>

#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>
#include <plat/inc/pwr.h>

#define SPI_CR1_CPHA                (1 << 0)
#define SPI_CR1_CPOL                (1 << 1)
#define SPI_CR1_MSTR                (1 << 2)

#define SPI_CR1_BR(x)               ((LOG2_CEIL(x) - 1) << 3)
#define SPI_CR1_BR_MIN              2
#define SPI_CR1_BR_MAX              256
#define SPI_CR1_BR_MASK             (0x7 << 3)

#define SPI_CR1_SPE                 (1 << 6)
#define SPI_CR1_LSBFIRST            (1 << 7)
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

struct stm_spi_regs {
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

struct stm_spi_state {
    uint8_t bits_per_word;
    uint16_t words;

    void *rx_buf;
    uint16_t rx_idx;

    const void *tx_buf;
    uint16_t tx_idx;
};

struct stm_spi_cfg {
    struct stm_spi_regs *regs;

    uint32_t clock_bus;
    uint32_t clock_unit;

    gpio_number_t gpio_miso;
    gpio_number_t gpio_mosi;
    gpio_number_t gpio_sck;
    gpio_number_t gpio_nss;
    uint8_t gpio_func;

    IRQn_Type irq;
};

struct stm_spi_dev {
    struct spi_device *base;
    const struct stm_spi_cfg cfg;
    struct stm_spi_state state;

    struct gpio miso;
    struct gpio mosi;
    struct gpio sck;
    struct gpio nss;
};

static int stm_spi_master_start_sync(struct spi_device *dev, spi_cs_t cs,
        const struct spi_mode *mode)
{
    struct stm_spi_dev *pdev = dev->pdata;
    struct stm_spi_regs *regs = pdev->cfg.regs;

    if (!mode->speed)
        return -EINVAL;

    if (mode->bits_per_word != 8 &&
            mode->bits_per_word != 16)
        return -EINVAL;

    uint32_t pclk = pwrGetBusSpeed(PERIPH_BUS_AHB1);
    unsigned int div = pclk / mode->speed;
    if (div > SPI_CR1_BR_MAX)
        return -EINVAL;
    else if (div < SPI_CR1_BR_MIN)
        div = SPI_CR1_BR_MIN;

    pwrUnitClock(pdev->cfg.clock_bus, pdev->cfg.clock_unit, true);

    regs->CR1 &= ~SPI_CR1_BR_MASK;
    regs->CR1 |= SPI_CR1_BR(div);

    if (mode->cpol == SPI_CPOL_IDLE_LO)
        regs->CR1 &= ~SPI_CR1_CPOL;
    else
        regs->CR1 |= SPI_CR1_CPOL;

    if (mode->cpha == SPI_CPHA_LEADING_EDGE)
        regs->CR1 &= ~SPI_CR1_CPHA;
    else
        regs->CR1 |= SPI_CR1_CPHA;

    if (mode->bits_per_word == 8)
        regs->CR1 &= ~SPI_CR1_DFF;
    else
        regs->CR1 |= SPI_CR1_DFF;

    if (mode->format == SPI_FORMAT_MSB_FIRST)
        regs->CR1 &= ~SPI_CR1_LSBFIRST;
    else
        regs->CR1 |= SPI_CR1_LSBFIRST;

    regs->CR2 |= SPI_CR2_SSOE;
    regs->CR1 |= SPI_CR1_MSTR;

    return 0;
}

static void stm_spi_tx_next_byte(struct stm_spi_dev *pdev)
{
    struct stm_spi_regs *regs = pdev->cfg.regs;
    struct stm_spi_state *state = &pdev->state;

    if (state->bits_per_word == 8) {
        const uint8_t *tx_buf8 = state->tx_buf;
        regs->DR = tx_buf8[state->tx_idx];
    } else {
        const uint16_t *tx_buf16 = state->tx_buf;
        regs->DR = tx_buf16[state->tx_idx];
    }
}

static int stm_spi_master_rxtx(struct spi_device *dev, void *rx_buf,
        const void *tx_buf, size_t size, const struct spi_mode *mode)
{
    struct stm_spi_dev *pdev = dev->pdata;
    struct stm_spi_regs *regs = pdev->cfg.regs;
    struct stm_spi_state *state = &pdev->state;

    state->bits_per_word = mode->bits_per_word;
    state->rx_buf = rx_buf;
    state->rx_idx = 0;
    state->tx_buf = tx_buf;
    state->tx_idx = 0;

    if (mode->bits_per_word == 8)
        state->words = size;
    else
        state->words = size / 2;

    regs->CR2 &= ~SPI_CR2_INT_MASK;
    regs->CR2 |= SPI_CR2_ERRIE;

    if (rx_buf)
        regs->CR2 |= SPI_CR2_RXNEIE;

    if (tx_buf) {
        regs->CR1 &= ~SPI_CR1_RXONLY;
        regs->CR2 |= SPI_CR2_TXEIE;
        stm_spi_tx_next_byte(pdev);
    } else {
        regs->CR1 |= SPI_CR1_RXONLY;
    }

    regs->CR1 |= SPI_CR1_SPE;
    return 0;
}

static int stm_spi_master_stop_sync(struct spi_device *dev)
{
    struct stm_spi_dev *pdev = dev->pdata;

    pwrUnitClock(pdev->cfg.clock_bus, pdev->cfg.clock_unit, false);
    return 0;
}

static void stm_spi_done(struct stm_spi_dev *pdev)
{
    struct stm_spi_regs *regs = pdev->cfg.regs;
    struct stm_spi_state *state = &pdev->state;

    if (state->tx_buf)
        while (regs->SR & SPI_SR_BSY)
            ;

    regs->CR1 &= ~SPI_CR1_SPE;
    spi_master_rxtx_done(pdev->base, 0);
}

static void stm_spi_rx_done(struct stm_spi_dev *pdev)
{
    struct stm_spi_regs *regs = pdev->cfg.regs;

    regs->CR2 &= ~SPI_CR2_RXNEIE;
    stm_spi_done(pdev);
}

static void stm_spi_tx_done(struct stm_spi_dev *pdev)
{
    struct stm_spi_regs *regs = pdev->cfg.regs;
    struct stm_spi_state *state = &pdev->state;

    regs->CR2 &= ~SPI_CR2_TXEIE;
    if (!state->rx_buf)
        stm_spi_done(pdev);
}

static void stm_spi_txe(struct stm_spi_dev *pdev)
{
    struct stm_spi_state *state = &pdev->state;

    state->tx_idx++;
    if (state->tx_idx == state->words)
        stm_spi_tx_done(pdev);
    else
        stm_spi_tx_next_byte(pdev);
}

static void stm_spi_rxne(struct stm_spi_dev *pdev)
{
    struct stm_spi_regs *regs = pdev->cfg.regs;
    struct stm_spi_state *state = &pdev->state;

    if (state->bits_per_word == 8) {
        uint8_t *rx_buf8 = state->rx_buf;
        rx_buf8[state->rx_idx] = regs->DR;
    } else {
        uint16_t *rx_buf16 = state->rx_buf;
        rx_buf16[state->rx_idx] = regs->DR;
    }

    state->rx_idx++;
    if (state->rx_idx == state->words)
        stm_spi_rx_done(pdev);
}

static void stm_spi_isr(struct stm_spi_dev *pdev)
{
    struct stm_spi_regs *regs = pdev->cfg.regs;

    if (regs->SR & SPI_SR_RXNE) {
        stm_spi_rxne(pdev);
    } else if (regs->SR & SPI_SR_TXE) {
        stm_spi_txe(pdev);
    } else {
        /* TODO */
    }
}

#define DECLARE_IRQ_HANDLER(_n)             \
    void SPI##_n##_IRQHandler();            \
    void SPI##_n##_IRQHandler()             \
    {                                       \
        stm_spi_isr(&stm_spi_devs[_n - 1]); \
    }

const struct spi_device_ops stm_spi_ops = {
    .master_start_sync = stm_spi_master_start_sync,
    .master_rxtx = stm_spi_master_rxtx,
    .master_stop_sync = stm_spi_master_stop_sync,
};

static struct stm_spi_dev stm_spi_devs[] = {
    [0] = {
        .cfg = {
            .regs = (struct stm_spi_regs *)SPI1_BASE,

            .clock_bus = PERIPH_BUS_APB2,
            .clock_unit = PERIPH_APB2_SPI1,

            .gpio_miso = GPIO_PA(6),
            .gpio_mosi = GPIO_PA(7),
            .gpio_sck = GPIO_PA(5),
            .gpio_nss = GPIO_PA(4),
            .gpio_func = GPIO_A2_AFR_SPI123,

            .irq = SPI1_IRQn,
        },
    },
};
DECLARE_IRQ_HANDLER(1)

static inline void stm_spi_gpio_init(struct gpio *gpio,
        gpio_number_t number, uint8_t func)
{
    gpio_request(gpio, number);
    gpio_configure(gpio, GPIO_MODE_ALTERNATE, GPIO_PULL_NONE);
    gpio_assign_func(gpio, func);
}

static void stm_spi_init(struct stm_spi_dev *pdev, struct spi_device *dev)
{
    const struct stm_spi_cfg *cfg = &pdev->cfg;

    stm_spi_gpio_init(&pdev->miso, cfg->gpio_miso, cfg->gpio_func);
    stm_spi_gpio_init(&pdev->mosi, cfg->gpio_mosi, cfg->gpio_func);
    stm_spi_gpio_init(&pdev->sck, cfg->gpio_sck, cfg->gpio_func);
    stm_spi_gpio_init(&pdev->nss, cfg->gpio_nss, cfg->gpio_func);

    NVIC_EnableIRQ(cfg->irq);

    pdev->base = dev;
}

int spi_request(struct spi_device *dev, uint8_t bus_id)
{
    if (bus_id >= ARRAY_SIZE(stm_spi_devs))
        return -ENODEV;

    struct stm_spi_dev *pdev = &stm_spi_devs[bus_id];
    if (!pdev->base)
        stm_spi_init(&stm_spi_devs[bus_id], dev);

    memset(&pdev->state, 0, sizeof(pdev->state));
    dev->ops = &stm_spi_ops;
    dev->pdata = pdev;
    return 0;
}
