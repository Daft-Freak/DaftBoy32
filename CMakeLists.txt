cmake_minimum_required(VERSION 3.9.0)

include(${32BLIT_DIR}/32blit-pico/sdk_import.cmake OPTIONAL)

project(gameblit)

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

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

if(32BLIT_PICO)
    target_compile_definitions(gameblit PRIVATE -DNO_GBC -DNO_AUDIO)
else()
    set_target_properties(gameblit PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

# setup release packages
set(PROJECT_DISTRIBS LICENSE README.md)
install (FILES ${PROJECT_DISTRIBS} DESTINATION .)
set (CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set (CPACK_GENERATOR "ZIP" "TGZ")
include (CPack)