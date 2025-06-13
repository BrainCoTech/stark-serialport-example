# Python

## Requirement

- Python 3.8~3.12
- Mac 10.15 or later
- Windows 10 build 10.0.15063 or later
- Ubuntu 20.04 LTS or later

## Usage

```shell
cd python

# 安装依赖
pip3 install -r requirements.txt

cd revo1

# 读取设备信息
python3 revo1_get.py
# 控制设备-单个灵巧手
python3 revo1_ctrl.py
# 控制设备-一个总线上控制多个手
python3 revo1_ctrl_multi.py
# 动作序列, 支持6个内置手势，6个自定义手势
python3 revo1_action_seq.py
# 更新配置，修改slave_id, 波特率，Turbo模式等
python3 revo1_cfg.py
# 触觉版
python3 revo1_touch.py
# 升级设备
python3 dfu.py
```
