add_library(DaftBoyCore INTERFACE)

target_sources(DaftBoyCore INTERFACE
    DMGAPU.cpp
    DMGCPU.cpp
    DMGDisplay.cpp
    DMGMemory.cpp
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

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    target_sources(DaftBoyAdvanceCore INTERFACE
        compiler/X86Builder.cpp
    )
    target_compile_definitions(DaftBoyAdvanceCore INTERFACE RECOMPILER_X86)
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^arm")
    target_sources(DaftBoyAdvanceCore INTERFACE
        compiler/AGBRecompiler.cpp
        compiler/ThumbBuilder.cpp
        compiler/AGBRecompilerThumb.cpp
    )
    target_compile_definitions(DaftBoyAdvanceCore INTERFACE RECOMPILER_THUMB)
endif()

target_include_directories(DaftBoyAdvanceCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})