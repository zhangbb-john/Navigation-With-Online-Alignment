# Contains values common to all waypoint request and status messages.
# Waypoint goals with higher priority trump lower priority goals. For
# goals of the same priority, the pilot should follow the latest
# received.

# Name of the requesting node, used with id to identify the request.
Header header

float32 PWM_PropA
float32 PWM_PropB
float32 PWM_PropC
float32 PWM_PropD
float32 PWM_PropE
float32 PWM_PropF

float32 Power_PropA
float32 Power_PropB
float32 Power_PropC
float32 Power_PropD
float32 Power_PropE
float32 Power_PropF

float32 Fx
float32 Fy
float32 Fz
float32 Mx
float32 My
float32 Mz