Header header
# Measured velocity [m/s]
geometry_msgs/Vector3 velocity_body
geometry_msgs/Vector3 velocity_enu
geometry_msgs/Vector3 velocity_water
geometry_msgs/Vector3 velocity_water_enu
float64[4] BmVel_body 
float64[4] BmAmp_body 
float64[4] BmRange
float64[4] BmSNR
float64[4] BmVel_water
float64[4] BmAmp_water

float64 altitude 
float64 h # heading
float64 p # pitch
float64 r # roll

float64[9] velocity_covariance

