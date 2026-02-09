"""
Common initialization functions for Python examples

Provides unified device initialization similar to C's stark_common.h.
Supports: Auto-detect, Modbus, ZQWL CAN/CANFD, ZLG CAN, SocketCAN

Usage:
    from common_init import parse_args_and_init, cleanup_context, print_init_usage

    ctx = await parse_args_and_init(sys.argv)
    if ctx is None:
        return
    # ... use ctx ...
    await cleanup_context(ctx)
"""

import sys
import os
import platform
import argparse
from dataclasses import dataclass
from typing import Optional, List, Any

# Setup path
_current_dir = os.path.dirname(os.path.abspath(__file__))
if _current_dir not in sys.path:
    sys.path.insert(0, _current_dir)

from common_imports import sdk, logger, get_hw_type_name, check_sdk


@dataclass
class DeviceContext:
    """Device context for unified access"""
    ctx: Any                    # Device handler
    slave_id: int               # Slave ID
    hw_type: Any                # StarkHardwareType enum
    protocol_type: Any          # StarkProtocolType enum
    port_name: str              # Port name or interface
    baudrate: int = 0           # Baudrate (for Modbus/CAN)
    serial_number: str = ""     # Device serial number
    firmware_version: str = ""  # Firmware version
    # Internal state
    _zlg_controller: Any = None # ZLG controller (if using ZLG)
    _is_socketcan: bool = False # SocketCAN mode (SDK built-in)
    _socketcan_controller: Any = None  # External SocketCAN controller (if using external adapter)


async def init_modbus(port: str, baudrate: int, slave_id: int) -> Optional[DeviceContext]:
    """Initialize device via Modbus (RS485)"""
    check_sdk()
    try:
        ctx = await sdk.init_modbus(port, baudrate)
        
        # Get device info
        info = await ctx.get_device_info(slave_id)
        
        return DeviceContext(
            ctx=ctx,
            slave_id=slave_id,
            hw_type=info.hardware_type,
            protocol_type=sdk.StarkProtocolType.Modbus,
            port_name=port,
            baudrate=baudrate,
            serial_number=info.serial_number or "",
            firmware_version=info.firmware_version or "",
        )
    except Exception as e:
        logger.error(f"Modbus init failed: {e}")
        return None


async def init_zqwl(port: str, baudrate: int, slave_id: int, is_canfd: bool = False, 
                   data_baudrate: int = 5000000) -> Optional[DeviceContext]:
    """Initialize device via ZQWL CAN/CANFD
    
    Args:
        port: Serial port path
        baudrate: Arbitration baudrate (1000000 for both CAN 2.0 and CANFD)
        slave_id: Slave ID
        is_canfd: True for CANFD, False for CAN 2.0
        data_baudrate: Data baudrate for CANFD (default 5000000, ignored for CAN 2.0)
    """
    check_sdk()
    try:
        if is_canfd:
            ctx = await sdk.init_zqwl(port, baudrate, data_baudrate)
            protocol = sdk.StarkProtocolType.CanFd
        else:
            ctx = await sdk.init_zqwl(port, baudrate, 0)  # 0 = CAN 2.0
            protocol = sdk.StarkProtocolType.Can
        
        # Get device info
        info = await ctx.get_device_info(slave_id)
        
        return DeviceContext(
            ctx=ctx,
            slave_id=slave_id,
            hw_type=info.hardware_type,
            protocol_type=protocol,
            port_name=port,
            baudrate=baudrate,
            serial_number=info.serial_number or "",
            firmware_version=info.firmware_version or "",
        )
    except Exception as e:
        logger.error(f"ZQWL {'CANFD' if is_canfd else 'CAN'} init failed: {e}")
        return None


# Legacy aliases for backward compatibility
async def init_zqwl_can(port: str, baudrate: int, slave_id: int) -> Optional[DeviceContext]:
    """Initialize device via ZQWL CAN 2.0 (legacy, use init_zqwl instead)"""
    return await init_zqwl(port, baudrate, slave_id, is_canfd=False)


async def init_zqwl_canfd(port: str, arb_baudrate: int, data_baudrate: int, slave_id: int) -> Optional[DeviceContext]:
    """Initialize device via ZQWL CANFD (legacy, use init_zqwl instead)"""
    return await init_zqwl(port, arb_baudrate, slave_id, is_canfd=True, data_baudrate=data_baudrate)


async def init_zlg(slave_id: int, is_canfd: bool = False) -> Optional[DeviceContext]:
    """Initialize device via ZLG USB-CAN/CANFD (Linux only)
    
    Args:
        slave_id: Slave ID
        is_canfd: True for CANFD, False for CAN 2.0
    """
    if platform.system() != "Linux":
        logger.error("ZLG CAN is only supported on Linux")
        return None
    
    check_sdk()
    try:
        # Import ZLG adapter based on mode
        if is_canfd:
            sys.path.insert(0, os.path.join(_current_dir, "revo2_canfd"))
            from zlg_canfd import Revo2CanfdController as Controller
        else:
            sys.path.insert(0, os.path.join(_current_dir, "revo2_can"))
            from zlg_can import Revo2CanController as Controller
        
        controller = Controller(master_id=1, slave_id=slave_id)
        if not await controller.initialize():
            logger.error("Failed to initialize ZLG adapter")
            return None
        
        # Get device info
        info = await controller.get_device_info()
        if not info:
            logger.error("Failed to get device info via ZLG")
            return None
        
        return DeviceContext(
            ctx=controller.client,
            slave_id=slave_id,
            hw_type=info.hardware_type,
            protocol_type=sdk.StarkProtocolType.CanFd if is_canfd else sdk.StarkProtocolType.Can,
            port_name="ZLG USB-CAN" + ("FD" if is_canfd else ""),
            serial_number=info.serial_number or "",
            firmware_version=info.firmware_version or "",
            _zlg_controller=controller,
        )
    except ImportError as e:
        logger.error(f"Failed to import ZLG adapter: {e}")
        return None
    except Exception as e:
        logger.error(f"ZLG {'CANFD' if is_canfd else 'CAN'} init failed: {e}")
        return None


# Legacy alias
async def init_zlg_can(slave_id: int, is_canfd: bool = False) -> Optional[DeviceContext]:
    """Initialize device via ZLG (legacy, use init_zlg instead)"""
    return await init_zlg(slave_id, is_canfd)


async def init_protobuf(port: str, slave_id: int = 10) -> Optional[DeviceContext]:
    """Initialize device via Protobuf protocol (serial)
    
    Protobuf protocol uses fixed baudrate 115200 and default slave_id 10.
    Position range is 0-100 (not 0-1000 like Modbus/CAN).
    
    Args:
        port: Serial port path
        slave_id: Slave ID (default 10)
    """
    check_sdk()
    try:
        ctx = await sdk.protobuf_open(port, slave_id)
        
        # Try to get device info (SN, FW version)
        serial_number = ""
        firmware_version = ""
        try:
            info = await ctx.get_device_info(slave_id)
            serial_number = info.serial_number or ""
            firmware_version = info.firmware_version or ""
        except Exception:
            pass  # Device info is optional for Protobuf
        
        return DeviceContext(
            ctx=ctx,
            slave_id=slave_id,
            hw_type=sdk.StarkHardwareType.Revo1Protobuf,
            protocol_type=sdk.StarkProtocolType.Protobuf,
            port_name=port,
            baudrate=115200,
            serial_number=serial_number,
            firmware_version=firmware_version,
        )
    except Exception as e:
        logger.error(f"Protobuf init failed: {e}")
        return None


async def init_socketcan(iface: str, slave_id: int, is_canfd: bool = False) -> Optional[DeviceContext]:
    """Initialize device via SocketCAN - SDK built-in (Linux only)
    
    Uses SDK's built-in SocketCAN support (recommended). No external adapter needed.
    Similar to C++ examples' -b/-B options.
    
    Args:
        iface: CAN interface name (e.g., "can0")
        slave_id: Slave ID
        is_canfd: True for CANFD, False for CAN 2.0
    """
    if platform.system() != "Linux":
        logger.error("SocketCAN is only supported on Linux")
        return None
    
    check_sdk()
    try:
        # Use SDK's built-in SocketCAN initialization
        if is_canfd:
            # Check if SDK has init_socketcan_canfd (v1.1.0+)
            if hasattr(sdk, 'init_socketcan_canfd'):
                sdk.init_socketcan_canfd(iface)
            else:
                logger.error("SDK does not support SocketCAN CANFD (requires v1.1.0+)")
                return None
            protocol = sdk.StarkProtocolType.CanFd
        else:
            # Check if SDK has init_socketcan_can (v1.1.0+)
            if hasattr(sdk, 'init_socketcan_can'):
                sdk.init_socketcan_can(iface)
            else:
                logger.error("SDK does not support SocketCAN CAN (requires v1.1.0+)")
                return None
            protocol = sdk.StarkProtocolType.Can
        
        # Create device handler
        ctx = sdk.init_device_handler(protocol, master_id=1)
        
        # Get device info
        info = await ctx.get_device_info(slave_id)
        
        return DeviceContext(
            ctx=ctx,
            slave_id=slave_id,
            hw_type=info.hardware_type,
            protocol_type=protocol,
            port_name=iface,
            serial_number=info.serial_number or "",
            firmware_version=info.firmware_version or "",
            _is_socketcan=True,
        )
    except Exception as e:
        logger.error(f"SocketCAN {'CANFD' if is_canfd else 'CAN'} init failed: {e}")
        return None


async def init_socketcan_external(iface: str, slave_id: int, is_canfd: bool = False) -> Optional[DeviceContext]:
    """Initialize device via SocketCAN - external adapter (Linux only)
    
    Uses external socketcan_can.py adapter from revo2_can/.
    Similar to C++ examples' -s/-S options.
    
    Args:
        iface: CAN interface name (e.g., "can0")
        slave_id: Slave ID
        is_canfd: True for CANFD, False for CAN 2.0
    """
    if platform.system() != "Linux":
        logger.error("SocketCAN is only supported on Linux")
        return None
    
    check_sdk()
    try:
        # Set environment variable for interface
        os.environ["STARK_SOCKETCAN_IFACE"] = iface
        
        # Import SocketCAN adapter
        sys.path.insert(0, os.path.join(_current_dir, "revo2_can"))
        from socketcan_can import Revo2CanController
        
        controller = Revo2CanController(master_id=1, slave_id=slave_id)
        if not await controller.initialize():
            logger.error("Failed to initialize SocketCAN adapter")
            return None
        
        # Get device info
        info = await controller.get_device_info()
        if not info:
            logger.error("Failed to get device info via SocketCAN")
            return None
        
        return DeviceContext(
            ctx=controller.client,
            slave_id=slave_id,
            hw_type=info.hardware_type,
            protocol_type=sdk.StarkProtocolType.CanFd if is_canfd else sdk.StarkProtocolType.Can,
            port_name=iface,
            serial_number=info.serial_number or "",
            firmware_version=info.firmware_version or "",
            _is_socketcan=True,
            _socketcan_controller=controller,  # Keep reference for cleanup
        )
    except ImportError as e:
        logger.error(f"Failed to import SocketCAN adapter: {e}")
        return None
    except Exception as e:
        logger.error(f"SocketCAN external {'CANFD' if is_canfd else 'CAN'} init failed: {e}")
        return None


async def auto_detect_and_init(select_device: bool = True) -> Optional[DeviceContext]:
    """Auto-detect device and initialize
    
    Args:
        select_device: If True and multiple devices found, prompt user to select
    """
    check_sdk()
    try:
        devices = await sdk.auto_detect(scan_all=True)
        
        if not devices:
            logger.error("No devices found")
            return None
        
        # Display found devices
        logger.info(f"Found {len(devices)} device(s):")
        for i, dev in enumerate(devices):
            hw_name = get_hw_type_name(dev.hardware_type) if dev.hardware_type else "Unknown"
            print(f"\n[{i + 1}] {hw_name}")
            print(f"    Protocol: {dev.protocol_type}")
            print(f"    Port: {dev.port_name}")
            print(f"    Slave ID: 0x{dev.slave_id:02X} ({dev.slave_id})")
            if dev.serial_number:
                print(f"    Serial: {dev.serial_number}")
        
        # Select device
        device = devices[0]
        if len(devices) > 1 and select_device:
            try:
                choice = int(input(f"\nSelect device [1-{len(devices)}]: "))
                if 1 <= choice <= len(devices):
                    device = devices[choice - 1]
                else:
                    logger.error("Invalid selection")
                    return None
            except (ValueError, EOFError):
                logger.error("Invalid input")
                return None
        
        # Initialize from detected device
        ctx = await sdk.init_from_detected(device)
        
        return DeviceContext(
            ctx=ctx,
            slave_id=device.slave_id,
            hw_type=device.hardware_type,
            protocol_type=device.protocol_type,
            port_name=device.port_name,
            serial_number=device.serial_number or "",
            firmware_version=device.firmware_version or "",
        )
    except Exception as e:
        logger.error(f"Auto-detect failed: {e}")
        return None


async def cleanup_context(ctx: DeviceContext):
    """Cleanup device context"""
    if ctx is None:
        return
    
    try:
        if ctx._zlg_controller:
            ctx._zlg_controller.cleanup()
        elif ctx._socketcan_controller:
            # External SocketCAN adapter cleanup
            ctx._socketcan_controller.cleanup()
        elif ctx.protocol_type == sdk.StarkProtocolType.Can or ctx.protocol_type == sdk.StarkProtocolType.CanFd:
            # Use unified close_device_handler which handles both ZQWL and SocketCAN (SDK built-in)
            if hasattr(sdk, 'close_device_handler'):
                await sdk.close_device_handler(ctx.ctx)
            else:
                # Fallback for older SDK versions
                sdk.close_zqwl()
        elif ctx.protocol_type == sdk.StarkProtocolType.Modbus:
            await sdk.modbus_close(ctx.ctx)
        elif ctx.protocol_type == sdk.StarkProtocolType.Protobuf:
            # Protobuf uses serial port, close via context
            await ctx.ctx.close()
        logger.info("Device connection closed")
    except Exception as e:
        logger.error(f"Cleanup error: {e}")


def print_init_usage(prog_name: str = "program"):
    """Print initialization usage help"""
    print(f"\nInitialization options:")
    print(f"  {prog_name}                                    # Auto-detect (recommended)")
    print(f"  {prog_name} -m <port> <baudrate> <slave_id>    # Modbus (RS485)")
    print(f"  {prog_name} -p <port> [slave_id]               # Protobuf (serial, 115200 baud)")
    print(f"  {prog_name} -c <port> <baudrate> <slave_id>    # CAN 2.0 (ZQWL)")
    print(f"  {prog_name} -f <port> <arb> <data> <slave_id>  # CANFD (ZQWL)")
    print(f"  {prog_name} -b <iface> <slave_id>              # SocketCAN CAN 2.0 - SDK built-in (Linux)")
    print(f"  {prog_name} -B <iface> <slave_id>              # SocketCAN CANFD - SDK built-in (Linux)")
    print(f"  {prog_name} -s <iface> <slave_id>              # SocketCAN CAN 2.0 - external adapter (Linux)")
    print(f"  {prog_name} -S <iface> <slave_id>              # SocketCAN CANFD - external adapter (Linux)")
    print(f"  {prog_name} -z <slave_id>                      # ZLG CAN 2.0 (Linux)")
    print(f"  {prog_name} -Z <slave_id>                      # ZLG CANFD (Linux)")
    print(f"\nExamples:")
    print(f"  {prog_name} -m /dev/ttyUSB0 460800 127")
    print(f"  {prog_name} -p /dev/ttyUSB0          # Protobuf with default slave_id 10")
    print(f"  {prog_name} -p /dev/ttyUSB0 10       # Protobuf with custom slave_id")
    print(f"  {prog_name} -c /dev/ttyUSB0 1000000 1")
    print(f"  {prog_name} -f /dev/ttyUSB0 1000000 5000000 127")
    print(f"  {prog_name} -b can0 1       # SDK built-in SocketCAN (recommended)")
    print(f"  {prog_name} -s can0 1       # External adapter SocketCAN")
    print(f"  {prog_name} -z 2")


def create_init_parser(prog_name: Optional[str] = None) -> argparse.ArgumentParser:
    """Create argument parser with initialization options"""
    parser = argparse.ArgumentParser(
        prog=prog_name,
        add_help=False,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    
    # Initialization mode (mutually exclusive)
    init_group = parser.add_mutually_exclusive_group()
    init_group.add_argument('-m', '--modbus', nargs=3, metavar=('PORT', 'BAUD', 'SLAVE'),
                           help='Modbus: port baudrate slave_id')
    init_group.add_argument('-p', '--protobuf', nargs='+', metavar=('PORT', 'SLAVE'),
                           help='Protobuf: port [slave_id] (default slave_id=10, baudrate=115200)')
    init_group.add_argument('-c', '--can', nargs=3, metavar=('PORT', 'BAUD', 'SLAVE'),
                           help='ZQWL CAN 2.0: port baudrate slave_id')
    init_group.add_argument('-f', '--canfd', nargs=4, metavar=('PORT', 'ARB', 'DATA', 'SLAVE'),
                           help='ZQWL CANFD: port arb_baud data_baud slave_id')
    init_group.add_argument('-b', '--socketcan-builtin', nargs=2, metavar=('IFACE', 'SLAVE'),
                           help='SocketCAN CAN 2.0 - SDK built-in (recommended): interface slave_id')
    init_group.add_argument('-B', '--socketcan-fd-builtin', nargs=2, metavar=('IFACE', 'SLAVE'),
                           help='SocketCAN CANFD - SDK built-in (recommended): interface slave_id')
    init_group.add_argument('-s', '--socketcan', nargs=2, metavar=('IFACE', 'SLAVE'),
                           help='SocketCAN CAN 2.0 - external adapter: interface slave_id')
    init_group.add_argument('-S', '--socketcan-fd', nargs=2, metavar=('IFACE', 'SLAVE'),
                           help='SocketCAN CANFD - external adapter: interface slave_id')
    init_group.add_argument('-z', '--zlg', nargs=1, metavar='SLAVE',
                           help='ZLG CAN 2.0: slave_id')
    init_group.add_argument('-Z', '--zlg-fd', nargs=1, metavar='SLAVE',
                           help='ZLG CANFD: slave_id')
    
    return parser


async def parse_args_and_init(argv: List[str], extra_parser: Optional[argparse.ArgumentParser] = None) -> tuple:
    """Parse command line arguments and initialize device
    
    Args:
        argv: Command line arguments (sys.argv)
        extra_parser: Optional parser with additional arguments
        
    Returns:
        (DeviceContext, parsed_args, remaining_args) or (None, None, None) on error
    """
    prog_name = os.path.basename(argv[0]) if argv else "program"
    
    # Create init parser
    init_parser = create_init_parser(prog_name)
    
    # Parse known args (init options)
    init_args, remaining = init_parser.parse_known_args(argv[1:])
    
    # Parse extra args if provided
    extra_args = None
    if extra_parser:
        extra_args, remaining = extra_parser.parse_known_args(remaining)
    
    # Initialize based on mode
    ctx = None
    
    if init_args.modbus:
        port, baud, slave = init_args.modbus
        ctx = await init_modbus(port, int(baud), int(slave, 0))
        
    elif init_args.protobuf:
        # Protobuf: port [slave_id]
        port = init_args.protobuf[0]
        slave = int(init_args.protobuf[1], 0) if len(init_args.protobuf) > 1 else 10
        ctx = await init_protobuf(port, slave)
        
    elif init_args.can:
        port, baud, slave = init_args.can
        ctx = await init_zqwl(port, int(baud), int(slave, 0), is_canfd=False)
        
    elif init_args.canfd:
        port, arb, data, slave = init_args.canfd
        ctx = await init_zqwl(port, int(arb), int(slave, 0), is_canfd=True, data_baudrate=int(data))
        
    elif init_args.socketcan_builtin:
        # SDK built-in SocketCAN CAN 2.0 (-b)
        iface, slave = init_args.socketcan_builtin
        ctx = await init_socketcan(iface, int(slave, 0), is_canfd=False)
        
    elif init_args.socketcan_fd_builtin:
        # SDK built-in SocketCAN CANFD (-B)
        iface, slave = init_args.socketcan_fd_builtin
        ctx = await init_socketcan(iface, int(slave, 0), is_canfd=True)
        
    elif init_args.socketcan:
        # External adapter SocketCAN CAN 2.0 (-s)
        iface, slave = init_args.socketcan
        ctx = await init_socketcan_external(iface, int(slave, 0), is_canfd=False)
        
    elif init_args.socketcan_fd:
        # External adapter SocketCAN CANFD (-S)
        iface, slave = init_args.socketcan_fd
        ctx = await init_socketcan_external(iface, int(slave, 0), is_canfd=True)
        
    elif init_args.zlg:
        slave = init_args.zlg[0]
        ctx = await init_zlg(int(slave, 0), is_canfd=False)
        
    elif init_args.zlg_fd:
        slave = init_args.zlg_fd[0]
        ctx = await init_zlg(int(slave, 0), is_canfd=True)
        
    else:
        # Auto-detect
        ctx = await auto_detect_and_init()
    
    if ctx is None:
        return None, None, None
    
    # Print device info
    print(f"\n[Init] {get_hw_type_name(ctx.hw_type)}")
    print(f"  Protocol: {ctx.protocol_type}")
    print(f"  Port: {ctx.port_name}")
    print(f"  Slave ID: 0x{ctx.slave_id:02X} ({ctx.slave_id})")
    if ctx.serial_number:
        print(f"  Serial: {ctx.serial_number}")
    if ctx.firmware_version:
        print(f"  Firmware: {ctx.firmware_version}")
    print()
    
    return ctx, extra_args, remaining


# Export all public functions
__all__ = [
    'DeviceContext',
    'init_modbus',
    'init_protobuf',
    'init_zqwl',
    'init_zqwl_can',      # Legacy alias
    'init_zqwl_canfd',    # Legacy alias
    'init_zlg',
    'init_zlg_can',       # Legacy alias
    'init_socketcan',
    'init_socketcan_external',
    'auto_detect_and_init',
    'cleanup_context',
    'print_init_usage',
    'create_init_parser',
    'parse_args_and_init',
]
