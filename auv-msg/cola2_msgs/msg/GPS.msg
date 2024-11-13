# Extracted from UUV Simulator:
#   https://github.com/uuvsimulator/uuv_simulator

# This is a message to hold data from a DVL sensor (Doppler Velocity Log).
#
# Distances are in [m], velocities in [m/s]
#
# If the covariance is known, it should be filled.
# If it is unknown, it should be set to all zeros.
# If a measurement was invalid, its covariance should be set to -1 so it can be
# disregarded.
#
# DVLBeams are optional. If they are set they contain individual ranges and 1D
# doppler velocity estimates orthogonal to the ray.

Header header

int32 time

float64 latitude
float64 longitude
float64 altitude

int16 status
int16 satellites_used
int32[] satellite_visible_snr
float64 pdop
float64 hdop

float64 speed
float64 direction

