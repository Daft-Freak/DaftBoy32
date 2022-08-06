add_library(DaftBoyCore INTERFACE)

target_sources(DaftBoyCore INTERFACE
    DMGAPU.cpp
    DMGCPU.cpp
    DMGDisplay.cpp
    DMGMemory.cpp
    DMGRecompiler.cpp

    compiler/X86Builder.cpp # TODO: arch detect
    compiler/DMGRecompilerX86.cpp
)

target_include_directories(DaftBoyCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# DBA
add_library(DaftBoyAdvanceCore INTERFACE)

target_sources(DaftBoyAdvanceCore INTERFACE
    AGBAPU.cpp
    AGBCPU.cpp
    AGBDisplay.cpp
    AGBMemory.cpp
)

target_include_directories(DaftBoyAdvanceCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})