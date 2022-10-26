# MPC state and control messages

uint64 timestamp # time in microseconds since system start
uint64 timestamp_sample # time in microseconds since system start of the last sample used to generate this message

# Position in local NED frame
float32 x # X Position in NED frame in meters
float32 y # Y Position in NED frame in meters
float32 z # Z Position in NED frame in meters

# Velocity in body NED frame
float32 vx # X velocity in NED frame in meter / s
float32 vy # Y velocity in NED frame in meter / s
float32 vz # Z velocity in NED frame in meter / s

# Quaternion components -. world tp body frame rotation
float32 qw # Quaternion component 0, w (1 in null-rotation)
float32 qx # Quaternion component 1, x (0 in null-rotation)
float32 qy # Quaternion component 2, y (0 in null-rotation)
float32 qz # Quaternion component 3, z (0 in null-rotation)

# Angular rates in body frame
float32 wx # Roll angular speed in radian / s
float32 wy # Pitch angular speed in radian / s
float32 wz # Yaw angular speed in radian / s

# Motor outputs
float32 m1 # Motor 1 output (0..1)
float32 m2 # Motor 2 output (0..1)
float32 m3 # Motor 3 output (0..1)
float32 m4 # Motor 4 output (0..1)
