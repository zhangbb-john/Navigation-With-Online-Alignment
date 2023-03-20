#ifndef INITIAL_USBL
#define INITIAL_USBL

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <chrono>
#include <sstream>
using namespace std;
using namespace Eigen;
struct AngleMeasurement
{
    Vector2d angle;
    Vector2d sigma;
    Vector3d position;
    Quaterniond q;
};
struct RecvimMeasurement
{
    Vector2d recvim;
    Vector2d sigma;
    Vector3d position;
    Quaterniond q;
    Vector3d velocity;
};

struct ANGLE_COST
{
    ANGLE_COST(const Vector2d &angle, const Vector2d &sigma, const Vector3d &position, const Quaterniond &q) : _angle(angle), _sigma(sigma), _position(position), _q(q) {}

    template <typename T>
    bool operator()(
        const T *const beacon_pos, const T *const usbl_rpy,
        T *residual) const
    {
        T Base2Beacon[3];
        Base2Beacon[0] = beacon_pos[0] - T(_position[0]);
        Base2Beacon[1] = beacon_pos[1] - T(_position[1]);
        Base2Beacon[2] = beacon_pos[2] - T(_position[2]);
        T Base2BeaconInBase[3];

        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_base2w[4] = {T(_q.w()), T(_q.x()), T(_q.y()), T(_q.z())};

        T R[9];
        const double kPi = 3.14159265358979323846;
        const T rad2deg(180.0 / kPi);
        T euler_angle[3] = {usbl_rpy[0] * rad2deg, usbl_rpy[1] * rad2deg, usbl_rpy[2] * rad2deg};

        T q_USBL2base[4];
        ceres::EulerAnglesToRotationMatrix(euler_angle, 3, R);
        ceres::RotationMatrixToQuaternion(R, q_USBL2base);

        //cout << "Base2Beacon: " << Base2Beacon << endl;
        ceres::QuaternionRotatePoint(q_base2w, Base2Beacon, Base2BeaconInBase);
        T Base2BeaconInUSBL[3];
        //cout << "_q.matrix:" << endl << _q.matrix() << endl;

        //cout << "Base2BeaconInBase:" << Base2BeaconInBase[0] << ", " << Base2BeaconInBase[1] << ", " << Base2BeaconInBase[2] << endl;
        ceres::QuaternionRotatePoint(q_USBL2base, Base2BeaconInBase, Base2BeaconInUSBL);
        //cout << "Base2BeaconInUSBL:" << Base2BeaconInUSBL[0] << ", " << Base2BeaconInUSBL[1] <<", " << Base2BeaconInUSBL[2] << endl;
        

        residual[0] = ceres::atan(T(_angle[0]) - ceres::atan2(Base2BeaconInUSBL[1], Base2BeaconInUSBL[0])) / T(_sigma[0]);

        residual[1] = ceres::atan(T(_angle[1]) - ceres::atan2(Base2BeaconInUSBL[2], ceres::sqrt(Base2BeaconInUSBL[0] * Base2BeaconInUSBL[0] + Base2BeaconInUSBL[1] * Base2BeaconInUSBL[1]))) / T(_sigma[1]);
        //cout << "residual:" << residual << endl;

        return true;
    }
    const Vector2d _angle;
    const Vector2d _sigma;
    const Vector3d _position;
    const Quaterniond _q;
};
struct RECVIM_COST
{
    RECVIM_COST(const Vector2d &recvim, const Vector2d &sigma, const Vector3d &position, const Quaterniond &q, const Vector3d &velocity) : _recvim(recvim), _sigma(sigma), _position(position), _q(q), _velocity(velocity) {}

    template <typename T>
    bool operator()(
        const T *const beacon_pos,
        T *residual) const
    {
        T Base2Beacon[3];
        Base2Beacon[0] = beacon_pos[0] - T(_position[0]);
        Base2Beacon[1] = beacon_pos[1] - T(_position[1]);
        Base2Beacon[2] = beacon_pos[2] - T(_position[2]);
        T Base2BeaconInBase[3];
        //cout << "position is :" << _position[0] << ", " << _position[1] << ", " << _position[2] << endl;
        // Convert quaternion from Eigen convention (x, y, z, w)
        // to Ceres convention (w, x, y, z)
        T q_base2w[4] = {T(_q.w()), T(_q.x()), T(_q.y()), T(_q.z())};
        ceres::QuaternionRotatePoint(q_base2w, Base2Beacon, Base2BeaconInBase);
        //cout << "Base2BeaconInBase template: " << Base2BeaconInBase[0] << ", " << Base2BeaconInBase[1] << ", " << Base2BeaconInBase[2] << endl;
        //cout << "beaconbeacon in world: " << beacon_pos[0] - _position[0] << ", " << beacon_pos[1] - _position[1] << ", "<< beacon_pos[2] - _position[2] << endl;

        //cout << "_q.matrix:" << endl << _q.matrix() << endl;
        Vector3d eigenbase2beacon;
        stringstream stream;
        string str;
        //cout <<"beacon_pos[0] is " << beacon_pos[0] << endl;
        //cout << "_position[0] is " << _position[0] << endl;
        stream << beacon_pos[0] - _position[0];
        stream >> str;
        //cout << str << endl;
        //cout << str.substr(1, str.length() - 1) << " str end" << std::endl;

        // cout << "stod start" << std::endl;
        // double x = stod(str.substr(1, str.length() - 2));
        //cout<< "x = " << x << endl;

        // cout <<"beacon_pos[1] is " << beacon_pos[1] << endl;
        // cout << "_position[1] is " << _position[1] << endl;
        // stream << beacon_pos[1] - _position[1];
        // stream >> str;
        // cout << "string start: " << str << endl;
        // double y = stod(str.substr(1, str.length() - 2));
        // eigenbase2beacon(1) = y;

        // stream << beacon_pos[2] - _position[2];
        // stream >> str;
        // cout << str << endl;
        // double z = stod(str.substr(1, str.length() - 2));
        // cout << "start beacon <<" << endl;
        // eigenbase2beacon << x, y, z;
        // eigenbase2beacon << 20 - _position[0] << 20 - _position[1] << 6 - _position[2];
        // Vector3d eigenbase2beaconinbase = _q.matrix() * eigenbase2beacon;
        // cout << "eigenbase2beaconinbase:" << eigenbase2beaconinbase << endl;
        T norm = ceres::sqrt(Base2BeaconInBase[0] * Base2BeaconInBase[0] + Base2BeaconInBase[1] * Base2BeaconInBase[1] + Base2BeaconInBase[2] * Base2BeaconInBase[2]);

        Base2BeaconInBase[0] = Base2BeaconInBase[0] / norm;
        Base2BeaconInBase[1] = Base2BeaconInBase[1] / norm;
        Base2BeaconInBase[2] = Base2BeaconInBase[2] / norm;
        T speed =  Base2BeaconInBase[0] * _velocity[0] + Base2BeaconInBase[1] * _velocity[1] + Base2BeaconInBase[2] * _velocity[2];

        residual[0] = (T(_recvim[0]) - speed) / 3.0e-2;
        //cout << "speed error: " << T(_recvim[0]) - speed << endl;

        residual[1] = (T(_recvim[1]) - beacon_pos[2]) / 0.6;
        
        return true;
    }
    const Vector2d _recvim;
    const Vector2d _sigma;
    const Vector3d _position;
    const Quaterniond _q;
    const Vector3d _velocity;
};

class InitialUSBL
{
public:
    InitialUSBL();
    ~InitialUSBL() = default;

    bool Initialization(vector<AngleMeasurement> &angle_measurements, vector<RecvimMeasurement> &recvim_measurements, Vector3d &beacon_pos, Vector3d &usbl_rpy);
    void set_threshold(double threshold);
    double get_threshold();
protected:
    double threshold_; 
};


#endif