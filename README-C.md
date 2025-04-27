# Build C Example

The `macos` and `linux` directories contain example programs demonstrating how to use `libbc_stark_sdk` with the C/C++ programming language.

Currently, the provided examples include:

- `stark_example.cpp` – A `Modbus RTU` example
- `stark_multi_example.cpp` – A `Modbus RTU` example for multi-device communication on a BUS

The provided Makefile is designed to build all examples, using gcc for compilation and pkg-config to determine the necessary include paths and linker settings for libserialport. This serves as a minimal reference on how to structure a Makefile for compiling a program with `libbc_stark_sdk`.

To build the examples, ensure that `make`, `gcc`, `pkg-config`, and `libbc_stark_sdk` are correctly installed. Then, simply run:

```shell
cd linux && make && make run
cd mac && make && make run
```

If the build process fails, you may need to adjust the Makefile or configure your environment variables accordingly.

Alternatively, you can compile these examples using any other compiler, IDE, or build system. Just ensure that the `stark-sdk.h` header is accessible for compilation and the `libbc_stark_sdk` library is available for linking and execution.
