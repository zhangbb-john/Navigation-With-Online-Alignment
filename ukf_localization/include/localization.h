#pragma once
#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#ifndef ENU_ACC
// #define ENU_ACC
#endif

//#include <au_core/math_util.h>
#include <utils.h>
#include <types.h>
#include <ukf.h>
#include <iostream>
#include <memory>
#include <vector>
#include <initial_usbl.h>
/*
 * localization filter based on UKF
 */
class Localization
{
public:
    /*
   * initializes the filter
   * @param alpha: spread of sigma points (1 < alpha < 1e-4)
   * @param beta: prior knowledge of distribution (Gaussian = 2)
   * @param kappa: secondary scaling parameter
   */
    Localization(double alpha, double beta, double kappa);

    ~Localization() = default;

    /*
   * resets filter state
   * call after setting initial covariance or process noise
   */
    void reset();

    /*
   * calls predict and update for a sensor measurement
   * if filter is not initialized, initializes with IMU data
   */
    void processMeasurement(Measurement measurement);

    /*
   * runs the predict step only
   * @param: deltaT: prediction time in seconds
   */
    void predict(double deltaT);

    /*
   * set initial covariance of filter
   * @param P: initial covariance
   */
    void setInitialCovariance(
        const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &P);

    /*
   * set process noise covariance
   * @param Q: noise covariance
   */
    void setProcessNoise(const Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &Q);

    void setParam(bool INITIALIZATION_NLS, double threshold);
    /*
   * set state of filter
   * @param x: state
   */
    void setState(const Eigen::Matrix<double, STATE_SIZE, 1> &x);

    /*
   * get current filter estimate
   * @return: filter state estimate
   */
    Eigen::Matrix<double, STATE_SIZE, 1> &getState();

    /*
   * get filter covariance
   * @return covariance
   */
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> &getCovariance();

    /*
   * get initialization state
   * @return: true if initialized
   */
    bool isInitialized();

    /*
   * get time for last measurement
   * @param: time in seconds
   */
    double getLastMeasurementTime();

    void setLastMeasurementTime(double lastime);

protected:
    /*
   * state transition function (process model)
   * 3d kinematics model for the robot
   */
    Eigen::Matrix<double, STATE_SIZE, 1> stateTransitionFunction(
        const Eigen::Matrix<double, STATE_SIZE, 1> &x, double deltaT, bool output);

    /*
   * custom difference and mean function to handle angular values
   */
    void setupProcessModel();

    /*
   * setup measurement models for imu, depth and dvl
   */
    void setupMeasurementModels();

    // unscented kalman filter
    Ukf<STATE_SIZE> filter_;

    // filter initialized flag
    bool isInitialized_;
    bool isUSBLInitialized_;
    bool  INITIALIZATION_NLS_;

    Eigen::Matrix<double, STATE_SIZE, 1> x;
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P;

    // timestamp of most recent measurement
    double lastMeasurementTime_;

    // state transition matrix
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> transitionMatrix_;
    // initial covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> initialP_;

    // measurement models
    MeasurementModel<DEPTH_SIZE, STATE_SIZE> depthModel_;
    MeasurementModel<DVL_SIZE, STATE_SIZE> dvlModel_;
    MeasurementModel<IMU_SIZE, STATE_SIZE> imuModel_;
    MeasurementModel<ANGLE_SIZE, STATE_SIZE> angleModel_;
    MeasurementModel<RECVIM_SIZE, STATE_SIZE> recvimModel_;
    MeasurementModel<USBL_INITIAL_SIZE, STATE_SIZE> usblInitialModel_;
    std::vector<AngleMeasurement> angle_measurements;
    std::vector<RecvimMeasurement> recvim_measurements;
    InitialUSBL initial_usbl;
};
#endif