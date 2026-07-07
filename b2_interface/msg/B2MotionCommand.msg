# B2MotionCommand.msg
# High-level motion command for the Unitree B2 sport API.
#
# Supported command values:
#   "move"
#   "stop_move"
#   "balance_stand"
#   "recovery_stand"
#   "stand_down"
#   "damp"

string command
float32 linear_x
float32 linear_y
float32 angular_z
