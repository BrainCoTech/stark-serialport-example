# Revo 2 示例程序

```shell
make clean # 清理旧的编译文件

make # 编译默认模式（Modbus）
make run_revo2_ctrl # 运行 revo2_modbus_ctrl 示例
make run_revo2_ctrl_multi # 运行 revo2_modbus_ctrl_multi 示例
make run_revo2_dfu # 运行 revo2_dfu 示例

make MODE=can # 编译 CAN/CANFD 接口模式
make run_revo2_can_ctrl # 运行 revo2_can_ctrl 示例
make run_revo2_can_dfu # 运行 revo2_can_dfu 示例
make run_revo2_canfd # 运行 revo2_canfd_ctrl 示例
make run_revo2_canfd_dfu # 运行 revo2_canfd_dfu 示例

make MODE=ethercat # 编译 EtherCAT 接口模式
make run_revo2_ethercat
```
