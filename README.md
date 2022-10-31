# DaftBoy32

## Things this is:
 - A Game Boy (+Color) emulator for 32blit, PicoSystem and other devices supported by the 32blit SDK
 - Also includes a WIP GBA core, this does not run on the 32blit.
 - Written by me

## Things this is NOT:
 - Complete (missing the link port, a few cartrige types and some GBC bits)
 - The fastest
 - [The most accurate](https://cupboard.daftgames.net/GBEmulatorShootout/)
 - A good example (?)


Originally built in about a week with the goal of "run Tetris". Also, first time writing an emulator. May explode on anything I haven't tested.

## Building

### 32blit

Uses the 32blit SDK, for a 32blit device build:
```
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=path/to/32blit-sdk/32blit.toolchain ..
make
```
See [the docs](https://github.com/32blit/32blit-sdk/tree/master/docs#readme) for more info (and building for other platforms).

### SDL

More minimal SDL-based shell, useful for testing:

```
mkdir build
cd build
cmake -DBUILD_32BLIT=OFF -DBUILD_SDL=1 ..
make
```

(Currently the only way to use the GBA core)

There's also a really minimal test runner (`-DBUILD_TESTS=1`), but you _probably_ don't want that.
