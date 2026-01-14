# Revo 1 Examples

## Prerequisites

- **MinGW/MSYS2**: The build requires `g++` compiler. Make sure you're running `make` from an MSYS2/MinGW terminal where `g++` is available in PATH.

### Installing/Reinstalling MinGW-w64

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

## Building

```shell
make clean            # Clean old build artifacts
make                  # Build all Revo1 examples

make run_revo1_ctrl   # Run revo1_modbus_ctrl example (Modbus over serial)
make run_revo1_can    # Run revo1_can_ctrl example (CAN, uses ZLG CAN API)
```

## ZQWL CAN vs ZLG CAN (hardware families)

This repo supports **two different CAN hardware/SDK families**:

- **ZLG CAN** (default for Windows examples)
  - Headers under: `dist/include/zlgcan/`
  - DLLs under: `dist/shared/win/zlgcan.dll` + `dist/shared/win/kerneldlls/`
  - Windows Revo1 CAN example: `revo1_can.cpp` (build via `make run_revo1_can`)

- **ZQWL CAN** (separate SDK)
  - Headers under: `dist/include/zqwl-can/`
  - DLLs under: `dist/shared/win/zqwlcan.dll` + `dist/shared/win/kerneldlls/`
  - Windows Revo2 ZQWL example: see `windows/revo2/revo2_canfd_zqwl.cpp` and its README

Choose the hardware/SDK that matches your actual CAN adapter. **Do not mix** ZLG and ZQWL headers/DLLs in the same build.

## Runtime DLLs (Windows)

At runtime, Windows needs to find (either in `PATH` or next to the `.exe`):

- `dist/shared/win/bc_stark_sdk.dll`
- `dist/shared/win/zlgcan.dll` (for ZLG CAN examples like `revo1_can`)
- `dist/shared/win/zqwlcan.dll` (only for ZQWL CAN examples)

Most systems already have the required Microsoft VC runtime DLLs installed.
If not, you can either install the official VC Redistributable, or drop the
DLLs mentioned in `windows/dll/README.md` next to the executable.

The Makefiles’ run targets (e.g. `make run_revo1_can`) automatically prefix
`PATH` with the correct `dist/shared/win` directories so these CAN DLLs can
be found when launching the `.exe`.
