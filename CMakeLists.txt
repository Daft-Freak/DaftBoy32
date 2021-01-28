cmake_minimum_required(VERSION 3.9.0)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

if(PICO_SDK_PATH)

    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    # Pull in Pico SDK (must be before project)
    include(pico_sdk_import.cmake)
    include(../pimoroni-pico/drivers/st7789/st7789.cmake)

    project(daft-boy-pico C CXX)

    # Initialise the Pico SDK
    pico_sdk_init()

    # Add executable. Default name is the project name, version 0.1

    set(SOURCES
        daft-boy-pico.cpp

        DMGAPU.cpp
        DMGCPU.cpp
        DMGDisplay.cpp
        DMGMemory.cpp
    )

    add_executable(daft-boy-pico ${SOURCES})
    set_source_files_properties(${SOURCES} PROPERTIES COMPILE_FLAGS -flto)

    pico_set_program_name(daft-boy-pico "daft-boy-pico")
    pico_set_program_version(daft-boy-pico "0.1")

    pico_enable_stdio_uart(daft-boy-pico 1)
    pico_enable_stdio_usb(daft-boy-pico 0)

    # Add the standard library to the build
    target_link_libraries(daft-boy-pico pico_stdlib st7789)

    target_compile_definitions(daft-boy-pico PRIVATE -DNO_GBC)

    #shouldn't really need this
    target_include_directories(daft-boy-pico PRIVATE ../pimoroni-pico/drivers/st7789/)

    pico_add_extra_outputs(daft-boy-pico)
else()

project(gameblit)
find_package (32BLIT CONFIG REQUIRED PATHS ../32blit-sdk)

#add_definitions("-DPROFILER")

blit_executable(gameblit
    control-icons.cpp
    file-browser.cpp
    menu.cpp

    gameblit.cpp

    DMGAPU.cpp
    DMGCPU.cpp
    DMGDisplay.cpp
    DMGMemory.cpp
)
blit_assets_yaml(gameblit assets.yml)
blit_metadata(gameblit metadata.yml)

##
target_compile_definitions(gameblit PRIVATE -DNO_GBC)
##

#set_target_properties(gameblit PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)

# setup release packages
set(PROJECT_DISTRIBS LICENSE README.md)
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)
endif()