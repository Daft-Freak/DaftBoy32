# minimal SDL shell

add_executable(DaftBoySDL Main.cpp)

find_package(SDL3 REQUIRED CONFIG REQUIRED COMPONENTS SDL3)

target_link_libraries(DaftBoySDL DaftBoyCore DaftBoyAdvanceCore SDL3::SDL3)

if(EMSCRIPTEN)
    # Emscripten-specific magic

    set(EMSCRIPTEN_SHELL ${CMAKE_CURRENT_LIST_DIR}/emscripten-shell.html)

    set_target_properties(DaftBoySDL PROPERTIES
        SUFFIX ".html"
        LINK_FLAGS "-s ENVIRONMENT=web -s ASYNCIFY=1 --shell-file ${EMSCRIPTEN_SHELL}"
        LINK_DEPENDS ${EMSCRIPTEN_SHELL}
    )
endif()
