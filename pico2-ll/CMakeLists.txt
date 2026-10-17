enable_language(ASM)

pico_sdk_init()

#embed BIOS
#generate_bios(PACEPicoBIOS elf32-littlearm armv6s-m)

add_executable(DaftBoyAdvanceP2
    Main.cpp

    Filesystem.cpp
    Storage.cpp

    fatfs/ff.c
    fatfs/ffunicode.c
)


# workaround missing PSRAM defines
if(PICO_BOARD STREQUAL "solderparty_rp2350_stamp_xl")
    target_compile_definitions(DaftBoyAdvanceP2 PRIVATE
        PICO_PSRAM_CS_PIN=8
        PICO_AUTO_DETECT_PSRAM_SIZE=1
    )
endif()

# driver selection based on boards
set(AUDIO_DRIVER none)

# standalone boards
if(PICO_BOARD STREQUAL "pimoroni_presto")
    set(DISPLAY_DRIVER "dpi")
endif()

# select HSTX anyway, but complain about it
if(NOT DISPLAY_DRIVER)
    set(DISPLAY_DRIVER "hstx")

    message("No DISPLAY_DRIVER set, assuming HSTX")
endif()

target_sources(DaftBoyAdvanceP2 PRIVATE
#    driver/Audio_${AUDIO_DRIVER}.cpp
    driver/Display_${DISPLAY_DRIVER}.cpp
)

# faster flash access
target_compile_definitions(DaftBoyAdvanceP2 PRIVATE PICO_EMBED_XIP_SETUP=1)
# for config.h
target_include_directories(DaftBoyAdvanceP2 PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/driver)

# PIO headers
#pico_generate_pio_header(DaftBoyAdvanceP2 ${CMAKE_CURRENT_LIST_DIR}/driver/dbi-8bit.pio)
#pico_generate_pio_header(DaftBoyAdvanceP2 ${CMAKE_CURRENT_LIST_DIR}/driver/dbi-spi.pio)
pico_generate_pio_header(DaftBoyAdvanceP2 ${CMAKE_CURRENT_LIST_DIR}/driver/dpi.pio)
#pico_generate_pio_header(DaftBoyAdvanceP2 ${CMAKE_CURRENT_LIST_DIR}/driver/i2s.pio)
pico_generate_pio_header(DaftBoyAdvanceP2 ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

#custom linker script
pico_add_linker_script_override_path(DaftBoyAdvanceP2 ${CMAKE_CURRENT_LIST_DIR})

# usual setup
target_link_libraries(DaftBoyAdvanceP2
    DaftBoyAdvanceCore

    hardware_dma
    hardware_i2c
    hardware_pio
    hardware_psram
    hardware_pwm
    hardware_spi

    pico_multicore
    pico_stdlib
)


pico_enable_stdio_uart(DaftBoyAdvanceP2 1)
pico_add_extra_outputs(DaftBoyAdvanceP2)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/DaftBoyAdvanceP2.uf2
	DESTINATION bin
)
