# Revo2 Python SDK

## Usage

```shell
cd python

# 安装依赖
(py310) ➜  python git:(main) ✗ pip3 install -r requirements.txt

(py310) ➜  python git:(main) ✗ cd revo2
# 控制/读取信息-单只手
(py310) ➜  revo2 git:(main) ✗ python revo2_ctrl.py
# 控制/读取信息-单只手（电容式触觉手）
(py310) ➜  revo2 git:(main) ✗ python revo2_touch.py
# 控制/读取信息-单只手（压感式触觉手）
(py310) ➜  revo2 git:(main) ✗ python revo2_touch_pressure.py
# 控制/读取信息-多只手
(py310) ➜  revo2 git:(main) ✗ python revo2_ctrl_multi.py
# 控制双手
(py310) ➜  revo2 git:(main) ✗ python revo2_ctrl_dual.py
# 动作序列
(py310) ➜  revo2 git:(main) ✗ python revo2_action_seq.py
# 更新配置，修改设备ID, 波特率，Turbo模式等
(py310) ➜  revo2 git:(main) ✗ python revo2_cfg.py
# 固件OTA
(py310) ➜  revo2 git:(main) ✗ python revo2_dfu.py
```
