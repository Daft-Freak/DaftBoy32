#include <cstdlib>
#include <ctime>
#include <forward_list>
#include <string>
#include <string_view>

#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/psram.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
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
        return;
    }

    auto in = framebuffer + (line - 40) * 240;
    for(int x = 0; x < 240; x++)
        *buf++ = *in++;
}

static void initHardware()
{
    set_sys_clock_khz(252000, false);

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
}

static void initEmulator()
{
    extern char _binary_bios_gba_start[];
    cpu.getMem().setBIOSROM(reinterpret_cast<const uint8_t *>(_binary_bios_gba_start));

    cpu.getDisplay().setFramebuffer(framebuffer);

    cpu.reset();
}

int main()
{
    initHardware();

    // emulator init
    initEmulator();

    while(true)
    {
        cpu.runFrame();

        // don't have audio yet
        auto &apu = cpu.getAPU();
        while(apu.hasSample())
            apu.getSample();

        update_display();
    }

    return 0;
}