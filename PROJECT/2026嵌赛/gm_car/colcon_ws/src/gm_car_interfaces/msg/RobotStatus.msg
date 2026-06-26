# RobotStatus.msg
builtin_interfaces/Time stamp #时间
float32 battery_voltage    # 电池电压 (V)
float32 linear_velocity    # 线速度 (m/s)
float32 angular_velocity   # 角速度 (rad/s)
float32 x_position        # X位置 (m)
float32 y_position        # Y位置 (m)
float32 angle             # 角度 (rad)
bool is_valid             # 数据是否有效