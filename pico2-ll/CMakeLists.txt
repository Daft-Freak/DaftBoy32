enable_language(ASM)

pico_sdk_init()

#embed BIOS
function(generate_bios TARGET_NAME OBJCOPY_TARGET OBJCOPY_ARCH)
    #embed BIOS
    set(BIOS_FILE bios.gba)
    set(BIOS_PATH ${CMAKE_CURRENT_LIST_DIR}/../)
    set(BIOS_OUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

    if(NOT EXISTS ${BIOS_PATH}${BIOS_FILE})
        # TODO: fall back to bios emulation
        message(FATAL_ERROR "Could not find ${BIOS_FILE}")
    endif()

    add_custom_command(
        OUTPUT bios.o
        WORKING_DIRECTORY ${BIOS_PATH}
        COMMAND ${CMAKE_OBJCOPY} -I binary -O ${OBJCOPY_TARGET} -B ${OBJCOPY_ARCH} --rename-section .data=.rodata,alloc,load,readonly,data,contents ${BIOS_FILE} ${CMAKE_CURRENT_BINARY_DIR}/bios.o
        DEPENDS ${BIOS_PATH}${BIOS_FILE}
    )

    add_library(${TARGET_NAME} bios.o)
    target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})
    set_target_properties(${TARGET_NAME} PROPERTIES LINKER_LANGUAGE C)
endfunction()

if(NO_BIOS)
    add_library(DaftBoyBIOS INTERFACE)
    target_compile_definitions(DaftBoyBIOS INTERFACE NO_BIOS)
else()
    generate_bios(DaftBoyBIOS elf32-littlearm armv6s-m)
endif()

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
    DaftBoyBIOS

    hardware_dma
    hardware_i2c
    hardware_interp
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


# gonna go fast
pico_define_boot_stage2(flash_clkdiv3 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(flash_clkdiv3 PRIVATE PICO_FLASH_SPI_CLKDIV=3 PICO_FLASH_SPI_RXDELAY=3)

pico_set_boot_stage2(DaftBoyAdvanceP2 flash_clkdiv3)