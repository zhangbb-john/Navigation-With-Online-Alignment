//noise mode
#define GAUSSIAN 0
#define T_DISTR 1
#define NOISE_MODE T_DISTR
//algorithm mode
#define UKF 0
#define ROBUST_UKF 1
#define FILTER ROBUST_UKF

#define DR 0
#define DOA 1
#define DOA_BEACON_KNOWN 2
#define PROPOSED 3
#define METHOD DOA
//running mode: simulation or experiment 
#define SIMULATION 0
#define FIELD 1
#define EXPERIMENT FIELD
#define SENSOR_MSG_IMU 0
#define AHRS 1
#define NORMAL 0
#define INFERENCE 1
#define SITUATION INFERENCE