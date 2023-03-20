#include <data_gen/param.h>
void Param::Param(const ros::NodeHandle &private_nh))
    :private_nh_(private_nh)
{
    loadParams();
}
void Param::loadParams()
{
    private_nh_.param("type", type, 10);
    private_nh_.param("imu_timestep", imu_timestep, 0.05);
    private_nh_.param("dvl_timestep", dvl_timestep, 0.25);
    private_nh_.param("angle_timestep", angle_timestep, 0.25);
    private_nh_.param("recvim_timestep", recvim_timestep, 0.25);
    private_nh_.param("t_start", t_start, 0);
    private_nh_.param("t_end", t_end, 300);
    private_nh_.param("circle_num", circle_num, 1);
    private_nh_.param("x_mag", x_mag, 20);
    private_nh_.param("y_mag", y_mag, 15);
    private_nh_.param("z_mag", z_mag, 5);
    private_nh_.param("angular_mag", angular_mag, 0.2);
    private_nh_.param("augular_speed", augular_speed, 0.05);
    private_nh_.param("gyro_bias_sigma", gyro_bias_sigma, 1.0e-5);
    private_nh_.param("acc_bias_sigma", acc_bias_sigma, 1.0e-5);
    private_nh_.param("gyro_noise_sigma", gyro_noise_sigma, 1.0e-2);
    private_nh_.param("acc_noise_sigma", acc_noise_sigma, 1.0e-2);
    private_nh_.param("theta", theta, 0.05);
    loadMatrixFromParams(beacon_pos, "beacon_pos");
}

void Param::loadMatrixFromParams(Eigen::MatrixXd &mat,
                                 const std::string &key)
{
    size_t size = mat.rows();
    mat.setZero();
    XmlRpc::XmlRpcValue param;

    try
    {
        private_nh_.getParam(key, param);
        for (size_t i = 0; i < size; i++)
        {
            for (size_t j = 0; j < size; j++)
            {
                // needed if all points don't have decimal points
                std::ostringstream os;
                os << param[size * i + j];
                std::istringstream is(os.str());
                is >> mat(i, j);
            }
        }
    }
    catch (...)
    {
        ROS_ERROR("Error loading %s param", "initial_estimate_covariance");
    }
}