# Revo 1 示例程序

```shell
make clean # 清理旧的编译文件

make # 编译默认模式（Modbus）
make run_revo1_ctrl # 运行 revo1_modbus_ctrl 示例
make run_revo1_ctrl_multi # 运行 revo1_modbus_ctrl_multi 示例
make run_revo1_touch # 运行 revo1_modbus_touch 示例
make run_revo1_dfu # 运行 revo1_dfu 示例

make MODE=can # 编译 CAN/CANFD 接口模式
make run_revo1_can # 运行 revo1_can_ctrl 示例
```
