# Robot's general state
string state
string status_msg	
float32 battery_voltage
float32 battery
# 0 -> Motor 1, 1 -> Motor 2
int32[2] temp
# 0 -> Motor 1, 1 -> Motor 2
int32[2] current_consumption
# 0 -> Channel A, 1 -> Channel B
int32[2] channel_ref
string motor_control_mode
string motor_input_mode
# 0 -> Linear speed, 1 -> angular speed
float32[2] desired_speed
float32[2] real_speed
# 0 -> LEFT motor, 1 -> RIGHT motor
float32[2] rpm_motors
# Controller control loop frequency 
float32 controller_hz
# Odometry Type (TRACKS / WHEELS)
string odometry_type
# Internal odom
float32[3] internal_odometry
