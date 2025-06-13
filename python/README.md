# Python

## Requirement

- Python 3.9~3.12
- Mac 10.15 or later
- Windows 10 build 10.0.15063 or later
- Ubuntu 20.04 LTS or later

## Usage

```shell
cd python

# 安装依赖
pip3 install -r requirements.txt

cd revo2
# 控制设备-单个灵巧手
python3 revo2_ctrl.py
# 控制设备-一个总线上控制多个手
python3 revo2_ctrl_multi.py
# 动作序列, 支持6个内置手势，24个自定义手势
python3 revo2_action_seq.py
# 更新配置，修改slave_id, 波特率，Turbo模式等
python3 revo2_cfg.py
# 升级设备
python3 dfu.py

cd revo2_canfd
# CANFD-智嵌物联
python3 zqwl_canfd.py
```
