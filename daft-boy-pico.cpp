#include <cstdio>

#include "hardware/clocks.h"
#include "hardware/dma.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"

#ifdef PICO_VGA_BOARD
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#else
#include "st7789.hpp"
#define DISPLAY_ST7789
#endif

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

#ifdef PICO_VGA_BOARD
    while(scanvideo_scanline_number(scanvideo_get_next_scanline_id()) < 80) {}
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

#ifdef PICO_VGA_BOARD

static void fillScanlineBuffer(struct scanvideo_scanline_buffer *buffer)
{
    static uint32_t postamble[] = {
        0x0000u | (COMPOSABLE_EOL_ALIGN << 16)
    };

    int w = 160;
    auto scanline = scanvideo_scanline_number(buffer->scanline_id);

    // top / bottom lines
    if(scanline < 8 || scanline >= 152)
    {
        buffer->data[0] = 2;
        buffer->data[1] = host_safe_hw_ptr(buffer->data + 8);

        buffer->data[2] = 0;
        buffer->data[3] = 0;

        buffer->data[8] = COMPOSABLE_COLOR_RUN;
        buffer->data[9] = (COMPOSABLE_EOL_ALIGN << 16) | (213 - 3);

        buffer->data_used = 4;
        return;
    }

    buffer->data[0] = 3;
    buffer->data[1] = host_safe_hw_ptr(buffer->data + 8);

    buffer->data[2] = w / 2;
    const uint16_t *pixels = cpu.getDisplay().getData() + (scanline - 8) * w;
    buffer->data[3] = host_safe_hw_ptr(pixels);

    buffer->data[4] = count_of(postamble);
    buffer->data[5] = host_safe_hw_ptr(postamble);

    buffer->data[6] = 0;
    buffer->data[7] = 0;
    buffer->data_used = 8;

    // fill first 27 pixels
    buffer->data[8] = COMPOSABLE_COLOR_RUN;
    buffer->data[9] = (COMPOSABLE_RAW_RUN << 16) | (27 - 3 - 1);
    buffer->data[10] = ((w + 2 - 3) << 16u);

    // hmm, do I need to fill the last few pixels too?
}
#endif

void core1Main()
{
#ifdef PICO_VGA_BOARD
    scanvideo_setup(&vga_mode_213x160_60);
    scanvideo_timing_enable(true);
#endif

    while(true)
    {
#ifdef PICO_VGA_BOARD
        struct scanvideo_scanline_buffer *buffer = scanvideo_begin_scanline_generation(true);
        while (buffer)
        {
            fillScanlineBuffer(buffer);
            scanvideo_end_scanline_generation(buffer);
            buffer = scanvideo_begin_scanline_generation(false);
        }
#endif
    }
}

void initButton(Button b)
{
#ifndef PICO_VGA_BOARD
    int gpio = static_cast<int>(b);
    gpio_set_function(gpio, GPIO_FUNC_SIO);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
#endif
}

bool getButton(Button b)
{
#ifdef PICO_VGA_BOARD
    return false;
#else
    return !gpio_get(static_cast<int>(b));
#endif
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

#ifdef PICO_VGA_BOARD
    // currently only used for vga
    multicore_launch_core1(core1Main);
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

#ifndef PICO_VGA_BOARD
        if(absolute_time_diff_us(lastUpdate, now) >= 10000)
#endif
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
