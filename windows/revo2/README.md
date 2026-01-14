# Revo 2 Examples

## Prerequisites

- **MinGW/MSYS2**: The build requires `g++` compiler. Make sure you're running `make` from an MSYS2/MinGW terminal where `g++` is available in PATH.

### Installing/Reinstalling MinGW-w64

#### Option 1: Using MSYS2 (Recommended)

If you're using MSYS2/MINGW64 terminal:

```bash
# Update package database
pacman -Syu

# Install or reinstall MinGW-w64 GCC toolchain
pacman -S mingw-w64-x86_64-gcc

# Or if you want to reinstall (remove and reinstall):
pacman -R mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-gcc

# Verify installation
g++ --version
```

#### Option 2: Standalone MinGW-w64

Download and install from:
- https://www.mingw-w64.org/downloads/
- Or use the installer: https://sourceforge.net/projects/mingw-w64/

## Project Structure

The project expects the following directory layout (relative to project root):

```
dist/
├── include/
│   ├── stark-sdk.h
│   ├── zlgcan/
│   │   ├── canframe.h
│   │   ├── config.h
│   │   ├── typedef.h
│   │   └── zlgcan.h
│   └── zqwl-can/
│       ├── canframe.h
│       ├── config.h
│       ├── typedef.h
│       ├── zcan.h
│       └── zlgcan.h
└── shared/
    └── win/
        ├── bc_stark_sdk.dll
        ├── bc_stark_sdk.dll.lib
        ├── kerneldlls/          # ZLG CAN/CANFD kernel DLLs, configs & resources
        │   └── ...              # See actual folder for full contents
        ├── zlgcan.dll           # ZLG CAN/CANFD DLL
        ├── zlgcan.lib
        └── zqwlcan.dll          # ZQWL CAN/CANFD DLL
```

**Note**: The Makefile automatically locates these directories. Ensure the `dist` folder is at the project root level (two levels up from `windows/revo2/`).

## Building

```shell
make clean # Clean old build artifacts
make       # Build default revo2_canfd (ZLG Windows API) example
make run   # Run default revo2_canfd example

make TARGET=revo2_canfd_zqwl      # Build revo2_canfd_zqwl (ZQWL CAN API) example
make run TARGET=revo2_canfd_zqwl  # Run ZQWL CAN example

```

## Runtime DLLs (Windows)

At runtime, Windows needs to find (either in `PATH` or next to the `.exe`):

- `dist/shared/win/bc_stark_sdk.dll`
- `dist/shared/win/zlgcan.dll` (for ZLG CAN examples)
- `dist/shared/win/zqwlcan.dll` (for ZQWL CAN examples)

Most systems already have the required Microsoft VC runtime DLLs installed.
If not, you can either install the official VC Redistributable, or drop the
DLLs mentioned in `windows/dll/README.md` next to the executable.

The `make run` targets automatically prefix `PATH` with the correct
`dist/shared/win` directories so these CAN DLLs can be found when launching
the `.exe`.

## Troubleshooting

If you get an error like "系统找不到指定的文件" (file not found) when running `make`:

1. **Check if g++ is available**: Run `g++ --version` in your terminal
2. **Use MSYS2/MinGW terminal**: The Makefile uses `cygpath` which requires MSYS2/MinGW environment
3. **Verify PATH**: Ensure MinGW bin directory is in your PATH
