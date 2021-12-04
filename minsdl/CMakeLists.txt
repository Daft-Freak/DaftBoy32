# minimal SDL shell

add_executable(DaftBoySDL Main.cpp)

if(EMSCRIPTEN)
    # Emscripten-specific magic
    add_library(SDL2::SDL2 INTERFACE IMPORTED)
    set_target_properties(SDL2::SDL2 PROPERTIES
        INTERFACE_COMPILE_OPTIONS "SHELL:-s USE_SDL=2"
        INTERFACE_LINK_LIBRARIES "-s USE_SDL=2"
    )

    set(EMSCRIPTEN_SHELL ${CMAKE_CURRENT_LIST_DIR}/emscripten-shell.html)

    set_target_properties(DaftBoySDL PROPERTIES
        SUFFIX ".html"
        LINK_FLAGS "-s ENVIRONMENT=web -s ASYNCIFY=1 --shell-file ${EMSCRIPTEN_SHELL}"
        LINK_DEPENDS ${EMSCRIPTEN_SHELL}
    )
else()
    find_package(SDL2 REQUIRED)
endif()

target_link_libraries(DaftBoySDL DaftBoyCore DaftBoyAdvanceCore SDL2::SDL2)