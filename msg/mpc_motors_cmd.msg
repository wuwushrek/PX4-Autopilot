# Few static parameters for the message
uint8 MPC_ON = 1
uint8 MPC_OFF = 0
uint8 MPC_TEST = 2
uint8 MPC_RESET = 3

# time since system start (microseconds)
uint64 timestamp


# Desired motor commands
float32[6] motor_val_des

# Desired angular rate and trust command
# T, wx, wy, wz -> T is between the right range and the same for wx, wy, wz
# Maybe we can go a level above to command the quaternion
float32[4] thrust_and_angrate_des

# Specify the mode of control of the MPC
uint8 mpc_on

# Constant parameter weighting if we track the angrate_des or motor output directly
uint8 weight_motors # Value between 0 and 100
