# Incorporate 'int32 date' and 'float64 mean_snr'
Header header
# date and time
int32 time
int32 date # ddmmyy ('$GNRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh')

# position
float64 latitude
float64 longitude
float64 altitude

# velocity 
float64 speed
float64 direction

# errror index
int16 status
int16 satellites_used
int32[] satellite_visible_snr
float64 mean_snr # mean of the satellite_visible_snr above
float64 pdop
float64 hdop


