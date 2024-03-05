# Build C example

The macos, linux directory contains example programs showing how to use libstark for Language C.

The examples currently included are:

list_ports.c - displays a list of ports on the system.
rs485.c - get info & control Stark Device by RS-485 bus.

The Makefile in this directory will attempt to build all the examples,
using 'gcc' to compile them and 'pkg-config' to discover the include
paths and linker settings needed to build with libserialport. It provides
a minimal example of how to write a Makefile to build a program using
libstark.

If you have make, gcc, pkg-config and libserialport installed correctly
then running 'make' should build the example programs in this directory.
If this doesn't work, you may need to modify the Makefile or set necessary
paths in your environment to suit your system.

You can also build these examples using any other compiler, IDE or build
system. You just need the libserialport.h & stark-sdk.h header available to compile them,
and the libserialport & libstark library available to link and run them.
