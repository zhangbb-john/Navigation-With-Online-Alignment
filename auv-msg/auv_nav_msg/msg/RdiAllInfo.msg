Header header
# Measured velocity [m/s]
int32 correlation_mag_min #7
int32 amp_min #8
int32 err_vel_max #11 12
float32[4] bm_bt_range #17-24
float32[4] bt_vel #25-32
int32[4] bm_bt_corr #33-36
int32[4] bm_amp #37-40
int32[4] bm_bt_good #41-44# bt is short for bottom track
int32[3] ref_layer #45-50
float32[4] wt_vel #51-58
int32[4] bm_wt_corr #59-62
int32[4] bm_wt_echo_intensity #63-66
int32[4] bm_wt_good #67-70
int32 bt_max_depth
int32[4] bm_rssi #73-76
int32 gain #77
float32[4] bm_bt_range_msb #78-81

float64[3] attitude_rph#roll pitch heading # variable leader data format 19-24
float32 depth # variable leader data format 17-18
float32 sound_vel # variable leader data format 15-16
float32 temperature # variable leader data format 27-28
