# World frame (absolute) waypoint request to pilot.
# A new waypoint request should contain a different goal.id
# (incremented) from previous requests. This same message should then
# be re-sent at 5-10 Hz.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance

float64[3] magnetic_field


