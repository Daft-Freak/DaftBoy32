cmake_minimum_required(VERSION 3.9.0)
project(gameblit)
set(32BLIT_PATH "../" CACHE PATH "Path to 32blit.cmake")

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

include(${32BLIT_PATH}/32blit.cmake)

#add_definitions("-DPROFILER")

blit_executable(gameblit
    control-icons.cpp
    DMGAPU.cpp
    DMGCPU.cpp
    DMGDisplay.cpp
    DMGMemory.cpp
    file-browser.cpp
    gameblit.cpp
    menu.cpp
)
blit_assets_yaml(gameblit assets.yml)
blit_metadata(gameblit metadata.yml)

set_target_properties(gameblit PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)