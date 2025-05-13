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

target_include_directories(DaftBoyAdvanceCore INTERFACE ${CMAKE_CURRENT_LIST_DIR})

# recompiler
add_library(DaftBoyRecompiler INTERFACE)

# common sources
target_sources(DaftBoyRecompiler INTERFACE
    compiler/RecompilerGeneric.cpp
    # builders might be here if they didn't define enums with the same name
)

# detect target arch
if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64")
    target_sources(DaftBoyRecompiler INTERFACE compiler/X86Builder.cpp compiler/X86Target.cpp)
    target_compile_definitions(DaftBoyRecompiler INTERFACE RECOMPILER_X86)
    set(HAVE_RECOMPILER TRUE)
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "^arm" AND NOT ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64") # 32 bit arm
    target_sources(DaftBoyRecompiler INTERFACE compiler/ThumbBuilder.cpp compiler/ThumbTarget.cpp)
    target_compile_definitions(DaftBoyRecompiler INTERFACE RECOMPILER_THUMB)
    set(HAVE_RECOMPILER TRUE)
endif()

if(HAVE_RECOMPILER)
    # enable DMG recompiler
    target_sources(DaftBoyCore INTERFACE
        compiler/DMGRecompiler.cpp
    )
    target_link_libraries(DaftBoyCore INTERFACE DaftBoyRecompiler)

    #enable AGB recompiler
    target_sources(DaftBoyAdvanceCore INTERFACE
        compiler/AGBRecompiler.cpp
    )
    target_link_libraries(DaftBoyAdvanceCore INTERFACE DaftBoyRecompiler)
endif()