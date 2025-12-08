# Example of Robotic Arm Control

## EMG data training & inference

1. Collect EMG data samples for gesture classification by using the arm ring and record 3 samples per gesture for 5 seconds each.
2. Infer the current gesture based on the received EMG data (for example, record 3 seconds of data for inference).
3. Control the robotic arm based on the gesture.

```shell
# Conda Python 3.8 ~ 3.12

# conda activate py310
(py310) ➜ which python
# /path/to/miniconda/base/envs/py310/bin/python
(py310) ➜ python -V
# Python 3.10.16

(py310) ➜ pip install -r requirements.txt --index-url https://pypi.org/simple/ # Install dependencies

# If the above method fails, you can try to manually download and then install the .whl file through pip
# https://pypi.org/project/bc-edu-sdk/
# (py310) ➜ pip install --force-reinstall '/path/to/bc_edu_sdk.whl'

(py310) ➜ python glove_example.py # Connect to remote control glove, receive 6-channel glove data(flex_data)
(py310) ➜ python armband_example.py # Connect to remote control arm ring, receive 8-channel EMG data(afe_data)
```
