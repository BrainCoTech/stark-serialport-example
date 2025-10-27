# Revo 2 - EtherCAT 版本 示例程序

```shell
ip link show # 检查网络接口
ethercat version # 检查 EtherCAT 版本
sudo systemctl enable ethercat # 启用 EtherCAT 主站
sudo systemctl restart ethercat # 重启 EtherCAT 主站
sudo systemctl status ethercat # 检查 EtherCAT 主站是否运行

# /usr/local/etc/ethercat.conf
journalctl -u ethercat.service -b # 查看 EtherCAT 运行日志
# 查看内核日志，排查 驱动层 / 网卡绑定 是否正常
dmesg | grep EtherCAT
dmesg | grep ec_

sudo ethercat slaves  # 检查是否有 EtherCAT 设备连接
sudo ethercat sdos # 检查 EtherCAT 设备的 SDO 信息
sudo ethercat pdos # 检查 EtherCAT 设备的 PDO 信息

# 读取固件版本
ethercat upload -t string -p 0 0x8000 0x11 # FW version
ethercat upload -t string -p 0 0x8000 0x12 # SN
ethercat upload -t string -p 0 0x8000 0x13 # Wrist FW version
ethercat upload -t string -p 0 0x8000 0x14 # Wrist SN

ethercat upload -t uint8 -p 0 0x8010 0x06 # Touch Sensor Vendor ID
ethercat upload -t string -p 0 0x8010 0x0D # Thumb Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x0E # Index Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x0F # Middle Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x10 # Ring Touch Sensor FW
ethercat upload -t string -p 0 0x8010 0x11 # Pinky Touch Sensor FW 

# 在OP模式下，读取关节信息
# echo "=== 读取所有关节数据 ==="
# echo "位置数据 (6x UINT16):"
ethercat upload -t raw -p 0 0x6000 0x01
ethercat upload -t raw -p 0 0x6000 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "关节位置%d: %d\n", NR, $1}'

# 在OP模式下，读取触觉信息
ethercat upload -t raw -p 0 0x6010 0x01
ethercat upload -t raw -p 0 0x6010 0x01 | xxd -r -p | od -An -t u2 --endian=little -w2 | awk '{printf "法向力%d: %d\n", NR, $1}'

make clean # 清理旧的编译文件
make # 编译
make run_revo2_sdo # 运行 revo2_sdo 示例
make run_revo2_pdo # 运行 revo2_pdo 示例
make run_revo2_touch_sdo # 运行 revo2_sdo 示例 - 触觉版
make run_revo2_touch_pdo # 运行 revo2_pdo 示例 - 触觉版
```
