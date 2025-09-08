# Ubuntu 示例

## Requirement

- Ubuntu 20.04 LTS or later

### Download or update stark lib dependencies

```shell
# First download libs, in project root directory
rm VERSION
./download-lib.sh

# 下载完成后目录包含如下
ros2_stark_ws/src/ros2_stark_controller/lib
└── libbc_stark_sdk.so

ros2_stark_ws/src/ros2_stark_controller/include
└── ros2_stark_controller
    ├── stark-sdk.h
    └── stark_node.hpp
```

## Revo1 RS-485/CAN 通信示例

详细说明请参考：[Revo1 README](revo1/README.md)

## Revo2 RS-485/CAN/CANFD 通信示例

详细说明请参考：[Revo2 CAN README](revo2/README.md)

## Revo2 EtherCAT 通信示例

详细说明请参考：[Revo2 EtherCAT README](revo2_ec/README.md)
