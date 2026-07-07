# B2Status.msg
# Aggregated high-level B2 status derived from sport and low state topics.

std_msgs/Header header
uint8 mode
uint8 gait_type
uint32 error_code
float32 body_height
float32[3] position
float32[3] velocity
float32 yaw_speed
uint8 battery_soc
float32 battery_voltage
float32 battery_current
bool low_state_available
