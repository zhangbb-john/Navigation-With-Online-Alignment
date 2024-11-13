# World frame (absolute) waypoint request to pilot.
# A new waypoint request should contain a different goal.id
# (incremented) from previous requests. This same message should then
# be re-sent at 5-10 Hz.

Header header

geometry_msgs/Vector3 x_axis
geometry_msgs/Vector3 y_axis
geometry_msgs/Vector3 z_axis
geometry_msgs/Vector3 rpy


