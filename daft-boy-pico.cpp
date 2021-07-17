#include <cstdio>

#include "hardware/clocks.h"
#include "hardware/dma.h"

#ifndef PICO_AUDIO_I2S_DATA_PIN
// fallback to my wierd setup
#define PICO_AUDIO_I2S_DATA_PIN 6
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE 20
#endif

#include "pico/audio_i2s.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#ifdef PICO_VGA_BOARD
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#else
#include "pico/st7789.hpp"
#define DISPLAY_ST7789
#endif

#include "DMGAPU.h"
#include "DMGDisplay.h"
#include "DMGCPU.h"
#include "DMGMemory.h"
#include "DMGRegs.h"

#ifdef DISPLAY_ST7789
pimoroni::ST7789 screen(240, 240, nullptr);
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
    screen.update();
#endif

#ifdef PICO_VGA_BOARD
    while(scanvideo_scanline_number(scanvideo_get_next_scanline_id()) < 80) {}
#endif
}

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

static audio_buffer_pool *audio_pool = nullptr;

void initAudio() {
    static audio_format_t audio_format = {
      .sample_freq = 32768,
      .format = AUDIO_BUFFER_FORMAT_PCM_S16,
      .channel_count = 1
    };

    static struct audio_buffer_format producer_format = {
      .format = &audio_format,
      .sample_stride = 2
    };

    struct audio_buffer_pool *producer_pool = audio_new_producer_pool(&producer_format, 4, 256);
    const struct audio_format *output_format;

    struct audio_i2s_config config = {
      .data_pin = PICO_AUDIO_I2S_DATA_PIN,
      .clock_pin_base = PICO_AUDIO_I2S_CLOCK_PIN_BASE,
      .dma_channel = 1,
      .pio_sm = 1,
    };

    output_format = audio_i2s_setup(&audio_format, &config);
    if (!output_format) {
      panic("PicoAudio: Unable to open audio device.\n");
    }

    bool ok = audio_i2s_connect(producer_pool);
    assert(ok);
    audio_i2s_set_enabled(true);
    audio_pool = producer_pool;
}

void core1Main()
{
#ifdef PICO_VGA_BOARD
    scanvideo_setup(&vga_mode_213x160_60);
    scanvideo_timing_enable(true);
#endif

    initAudio();

    auto &apu = cpu.getAPU();

    while(true)
    {
        // audio
        if(apu.getNumSamples() >= 256)
        {
            struct audio_buffer *buffer = audio_pool ? take_audio_buffer(audio_pool, false) : nullptr;
            if(buffer)
            {
                auto samples = (int16_t *) buffer->buffer->bytes;
                for(uint32_t i = 0; i < buffer->max_sample_count; i++)
                    *samples++ = apu.getSample();

                buffer->sample_count = buffer->max_sample_count;
                give_audio_buffer(audio_pool, buffer);
            }
        }
#ifdef PICO_VGA_BOARD
        // video
        struct scanvideo_scanline_buffer *buffer = scanvideo_begin_scanline_generation(true);
        while(buffer)
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
    screen.clear();

    // window
    screen.set_window(40, 48, 160, 144);
    screen.frame_buffer = const_cast<uint16_t *>(cpu.getDisplay().getData());
#endif

    // vga/audio
    multicore_launch_core1(core1Main);

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

#if 0
            // no audio
            auto &apu = cpu.getAPU();
            while(apu.getNumSamples())
                apu.getSample();
#endif

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
