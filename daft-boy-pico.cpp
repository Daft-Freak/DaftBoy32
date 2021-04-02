#include <cstdio>

#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "pico/stdlib.h"

#include "st7789.hpp"
#define DISPLAY_ST7789

#include "DMGAPU.h"
#include "DMGDisplay.h"
#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

#ifdef DISPLAY_ST7789
pimoroni::ST7789 screen(240, 240, nullptr);
uint32_t dma_channel;
using ST7789Reg = pimoroni::ST7789::reg;
#endif

enum class Button
{
    UP = 2,
    DOWN = 3,
    LEFT = 4,
    RIGHT = 5,

    A = 12,
    B = 13,
    X = 14,
    Y = 15,
};

DMGCPU cpu;

void getROMBank(uint8_t bank, uint8_t *ptr)
{
    printf("getROMBank!\n");
}

void updateCartRAM(uint8_t *cartRam, unsigned int size)
{
    printf("updateCartRAM!\n");
}

void onVBlank()
{
#ifdef DISPLAY_ST7789
    //
    using reg = pimoroni::ST7789::reg;
    int cs = 17;
    int dc = 16;

    // more theft
    dma_channel_wait_for_finish_blocking(dma_channel);

    uint8_t r = reg::RAMWR;

    gpio_put(cs, 0);

    gpio_put(dc, 0); // command mode
    spi_write_blocking(spi0, &r, 1);

    gpio_put(dc, 1); // data mode

    dma_channel_set_read_addr(dma_channel, cpu.getDisplay().getData(), true);
#endif
}

#ifdef DISPLAY_ST7789
void clearScreen()
{
    int cs = 17;
    int dc = 16;

    gpio_put(cs, 0);

    gpio_put(dc, 0); // command mode
    uint8_t r = ST7789Reg::RAMWR;
    spi_write_blocking(spi0, &r, 1);

    gpio_put(dc, 1); // data mode
    for(int i = 0; i < 240 * 240; i++)
    {
        uint16_t v = 0;
        spi_write_blocking(spi0, (const uint8_t *)&v, 2);
    }

    gpio_put(cs, 1);
}
#endif

void initButton(Button b)
{
    int gpio = static_cast<int>(b);
    gpio_set_function(gpio, GPIO_FUNC_SIO);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

bool getButton(Button b)
{
    return !gpio_get(static_cast<int>(b));
}

int main()
{
    stdio_init_all();

    initButton(Button::UP);
    initButton(Button::DOWN);
    initButton(Button::LEFT);
    initButton(Button::RIGHT);
    initButton(Button::A);
    initButton(Button::B);
    initButton(Button::X);
    initButton(Button::Y);

#ifdef DISPLAY_ST7789
    screen.init();
    clearScreen();

    // window
    screen.command(ST7789Reg::CASET, 4, "\x00\x28\x00\xc7"); // 40 - 199
    screen.command(ST7789Reg::RASET, 4, "\x00\x30\x00\xbf"); // 48 - 191

    // stolen from comments in st7789.cpp
    // setup spi for 16-bit transfers
    spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // initialise dma channel for transmitting pixel data to screen
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_16);
    channel_config_set_dreq(&config, spi_get_index(spi0) ? DREQ_SPI1_TX : DREQ_SPI0_TX);
    dma_channel_configure(dma_channel, &config, &spi_get_hw(spi0)->dr, cpu.getDisplay().getData(), 160 * 144, false);
    // end theft

#endif

    //setup
    cpu.getMem().setROMBankCallback(getROMBank);
    cpu.getMem().setCartRamUpdateCallback(updateCartRAM);

    cpu.getDisplay().setVBlankCallback(onVBlank);

    //cpu.getMem().setCartROM(tetrisROM);
    //cpu.getMem().setCartROM(superMarioLandROM);

    cpu.reset();

    auto lastUpdate = get_absolute_time();

    while (1)
    {
        auto now = get_absolute_time();

        if(absolute_time_diff_us(lastUpdate, now) >= 10000)
        {
            uint8_t inputs = (getButton(Button::RIGHT) ? (1 << 0) : 0) // right
                           | (getButton(Button::LEFT) ? (1 << 1) : 0) // left
                           | (getButton(Button::UP) ? (1 << 2) : 0)
                           | (getButton(Button::DOWN) ? (1 << 3) : 0)
                           | (getButton(Button::X) ? (1 << 4) : 0) // A
                           | (getButton(Button::Y) ? (1 << 5) : 0) // B
                           | (getButton(Button::A) ? (1 << 6) : 0) // select
                           | (getButton(Button::B) ? (1 << 7) : 0); // start

            cpu.setInputs(inputs);

            // no audio
            auto &apu = cpu.getAPU();
            while(apu.getNumSamples())
                apu.getSample();

            auto start = get_absolute_time();
            cpu.run(10);
            auto end = get_absolute_time();
            auto emuElapsed = absolute_time_diff_us(start, end);

            lastUpdate = delayed_by_ms(lastUpdate, 10);

            if(absolute_time_diff_us(lastUpdate, now) > 100000)
            {
                // running behind
                lastUpdate = delayed_by_ms(lastUpdate, 10);
            }
        }
    }

    return 0;
}
