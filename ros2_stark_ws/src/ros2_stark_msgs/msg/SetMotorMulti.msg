uint8 slave_id # 从机ID
uint8 mode     # 1~6, 对应以下6种控制模式
# 1. 位置
# 2. 速度
# 3. 电流，目前仅支持二代手
# 4. PWM，目前仅支持二代手
# 5. 位置+期望时间，目前仅支持二代手
# 6. 位置+期望速度，目前仅支持二代手

uint16[6]  positions            # 位置
int16[6]   speeds               # 速度
int16[6]   currents             # 电流
int16[6]   pwms                 # PWM
uint16[6]  durations            # 期望时间, 单位ms
