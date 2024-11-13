Header header

# status information
int16 status
float32 temperature

# orientation information
float32 roll # rad
float32 pitch # rad
float32 yaw # rad
float32[9] orientation_covariance

# inertial information
geometry_msgs/Vector3 velocity_enu # m/s
geometry_msgs/Vector3 velocity_xyz # m/s
geometry_msgs/Vector3 angular_velocity # rad/s
geometry_msgs/Vector3 linear_acceleration # m/s^2

# position
float64 latitude
float64 longitude
float64 altitude # m

# sensor calibration
float32 alignment_angle  
float32 scale 

