add_library(DaftBoyCore INTERFACE)

target_sources(DaftBoyCore INTERFACE
    DMGAPU.cpp
    DMGCPU.cpp
    DMGDisplay.cpp
    DMGMemory.cpp
)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    target_sources(DaftBoyCore INTERFACE
        DMGRecompiler.cpp
        compiler/X86Builder.cpp
        compiler/DMGRecompilerX86.cpp
    )
    target_compile_definitions(DaftBoyCore INTERFACE RECOMPILER_X86)
endif()

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