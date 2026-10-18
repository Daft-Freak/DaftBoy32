#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/psram.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/stdlib.h"


#include "fatfs/ff.h"

#include "config.h"

//#include "BIOS.h"
#include "Display.h"
// drivers
#include "i2c.h"

#include "AGBCPU.h"


#define PSRAM_LOCATION (XIP_BASE + _u(0x1000000))

static AGBCPU cpu;

static FATFS fs;

static uint16_t framebuffer[240 * 160];

// sync and speed measurement
static volatile bool waitSync = false;
static uint32_t emuTime = 0;
static uint32_t emuFrames = 0;

static repeating_timer_t audioTimer;

static const I2CDriver *i2cDrivers[]
{
};

void __not_in_flash_func(display_draw_line)(void *, int line, uint16_t *buf)
{
    // FIXME: presto-specific v padding
    // also could be more efficient here
    if(line < 40 || line >= 200)
    {
        for(int x = 0; x < 240; x++)
            *buf++ = 0;

        if(line == 200)
        {
            waitSync = false;
            __sev();
        }
        
        return;
    }

    auto in = framebuffer + (line - 40) * 240;
    for(int x = 0; x < 240; x++)
        *buf++ = *in++;
}

static void core1Main()
{
    while(true)
    {
        auto start = get_absolute_time();
        cpu.runFrame();
        emuTime += absolute_time_diff_us(start, get_absolute_time());

        emuFrames++;

        // wait for next frame
        waitSync = true;
        while(waitSync) __wfe();
    }
}


static bool audioTimerCallback(struct repeating_timer *t)
{
    auto apu = reinterpret_cast<AGBAPU *>(t->user_data);

    auto sample = apu->getSample();

#ifdef PWM_AUDIO_SLICE
    uint16_t pwm_sample = (int(sample) + 0x8000) >> (16 - PWM_AUDIO_BITS);
    pwm_set_chan_level(PWM_AUDIO_SLICE, PWM_AUDIO_CHAN, pwm_sample);
#endif

    return true;
}

static void initHardware()
{
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    sleep_ms(10);
    set_sys_clock_khz(378000, false);

    stdio_init_all();

    // init i2c_default if requested by board config
#ifdef DEFAULT_I2C_CLOCK
    i2c_init(i2c_default, DEFAULT_I2C_CLOCK);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // init i2c drivers
    for(auto &driver : i2cDrivers)
        driver->init();
#endif

    init_display();
    set_display_size(240, 240);

    // we just messed with the clocks so reconfigure psram
    psram_configure_params(PICO_DEFAULT_PSRAM_MAX_FREQ, PICO_DEFAULT_PSRAM_MAX_SELECT, PICO_DEFAULT_PSRAM_MIN_DESELECT);
    psram_reinitialize();
    size_t psramSize = psram_get_size();

    printf("Detected %i bytes PSRAM\n", psramSize);

    // init storage/filesystem
    auto res = f_mount(&fs, "", 1);

    if(res != FR_OK)
    {
        // TODO: display errors
        printf("Failed to mount filesystem! (error %i)\n", res);

        if(res == FR_NOT_READY)
            printf("Check that the SD card is inserted correctly.\n");

        while(true);
    }

    //init_audio();
#ifdef PWM_AUDIO_SLICE
    pwm_config c = pwm_get_default_config();
    pwm_config_set_wrap(&c, (1 << PWM_AUDIO_BITS) - 1);
    //pwm_config_set_clkdiv_int(&c, 2);
    pwm_init(PWM_AUDIO_SLICE, &c, true);
    pwm_set_chan_level(PWM_AUDIO_SLICE, PWM_AUDIO_CHAN, 1 << (PWM_AUDIO_BITS - 1));

    gpio_set_function(PWM_AUDIO_PIN, GPIO_FUNC_PWM);
#endif
    add_repeating_timer_us(((1000000 * 2) / 32768 + 1) / 2, audioTimerCallback, &cpu.getAPU(), &audioTimer);
}

static void initEmulator()
{
#ifndef NO_BIOS
    extern char _binary_bios_gba_start[];
    cpu.getMem().setBIOSROM(reinterpret_cast<const uint8_t *>(_binary_bios_gba_start));
#endif

    cpu.getDisplay().setFramebuffer(framebuffer);

    // load ROM
    // TODO: detect if ROM is too big for PSRAM
    auto psramPtr = reinterpret_cast<uint8_t *>(PSRAM_LOCATION);
    FIL f;
    if(f_open(&f, "AMKP.gba", FA_READ | FA_OPEN_EXISTING) == FR_OK)
    {
        printf("Loading ROM...\n");

        UINT bytesRead = 0;
        f_read(&f, psramPtr, psram_detect_size(), &bytesRead);

        printf("Read %u bytes\n", bytesRead);
        cpu.getMem().setCartROM(psramPtr, bytesRead);
    
        f_close(&f);
    }

    // reset CPU (and everything else)
    cpu.reset();

    multicore_launch_core1(core1Main);
}

int main()
{
    initHardware();

    // emulator init
    initEmulator();

    while(true)
    {
        update_display();

        auto frames = emuFrames;
        auto time = emuTime;

        if(frames >= 60)
        {
            emuFrames = 0;
            emuTime = 0;

            auto realTime = frames * 1000000 / 60;

            int percent = realTime * 100 / time;

            printf("%lu frames in %luus (%i%%)\n", frames, time, percent);
        }
    }

    return 0;
}