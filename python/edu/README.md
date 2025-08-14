# 臂环控制机械手示例

## EMG数据训练&推理

1. 通过臂环采集EMG数据样本进行训练手势分类（参考提供的上位机，每个手势采集3次，每次5秒）
2. 根据收到的EMG数据推理当前手势（如采集3秒数据进行推理）
3. 根据手势控制控制机械手

```shell
# Conda Python 3.8 ~ 3.12

# conda activate py310
(py310) ➜ which python
# /path/to/miniconda/base/envs/py310/bin/python
(py310) ➜ python -V
# Python 3.10.16

(py310) ➜ pip install -r requirements.txt --index-url https://pypi.org/simple/ # 安装依赖

# 无法通过以上方式安装的话，可尝试手动下载之后，再通过pip安装.whl文件
# https://pypi.org/project/bc-edu-sdk/
# (py310) ➜ pip install --force-reinstall '/path/to/bc_edu_sdk.whl'

(py310) ➜ python glove_example.py # 连接遥操作手套，接收6通道手套数据(flex_data)
(py310) ➜ python armband_example.py # 连接遥操作臂环，接收8通道EMG数据(afe_data)
```
