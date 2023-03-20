#include "initial_usbl.h"

InitialUSBL::InitialUSBL()
{
    std::cout << "class :"<< threshold_ << std::endl;
}
bool InitialUSBL::Initialization(vector<AngleMeasurement> &angle_measurements, vector<RecvimMeasurement> &recvim_measurements, Vector3d &beacon_pos, Vector3d &usbl_rpy)
{
    // std::cout << "in Initialization threshold is " << threshold_ << std::endl;

    if ((angle_measurements.size() + recvim_measurements.size() < 80))// || (recvim_measurements.size() < 2) )
    {
        cout << "now angle_measurements size is " << angle_measurements.size() << endl << "recvim_measurements size is " << recvim_measurements.size() << endl;
        return false;
    }
    cout << "now angle_measurements size is " << angle_measurements.size() << endl << "recvim_measurements size is " << recvim_measurements.size() << endl;

    double beacon_pos_param[3] = {0, 0, 0}; // abc参数的估计值
    double usbl_rpy_param[3] = {0, 0, 0};
    ceres::Problem problem;
    for (size_t i = 0; i < angle_measurements.size(); i++)
    {
        Vector2d angle = angle_measurements[i].angle;
        Vector2d sigma = angle_measurements[i].sigma;
        Vector3d position = angle_measurements[i].position;
        Quaterniond q = angle_measurements[i].q;
        problem.AddResidualBlock( // 向问题中添加误差项
                                  // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<ANGLE_COST, 2, 3, 3>(
                new ANGLE_COST(angle, sigma, position, q)),
            nullptr,          // 核函数，这里不使用，为空
            beacon_pos_param, // 待估计参数
            usbl_rpy_param);
    }
    for (size_t i = 0; i < recvim_measurements.size(); i ++)
    {
        Vector2d recvim = recvim_measurements[i].recvim;
        Vector2d sigma = recvim_measurements[i].sigma;
        Vector3d position = recvim_measurements[i].position;
        Quaterniond q = recvim_measurements[i].q;
        Vector3d velocity = recvim_measurements[i].velocity;
        problem.AddResidualBlock( // 向问题中添加误差项
                                  // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<RECVIM_COST, 2, 3>(
                new RECVIM_COST(recvim, sigma, position, q, velocity)),
            nullptr,          // 核函数，这里不使用，为空
            beacon_pos_param // 待估计参数
            );        
    }
    // 配置求解器
    ceres::Solver::Options options;               // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR; // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;  // 输出到cout

    ceres::Solver::Summary summary; // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary); // 开始优化

	if (summary.termination_type == ceres::CONVERGENCE && summary.final_cost < summary.initial_cost * threshold_)
	{
        cout << "Initialization converge" << endl;
        if (abs(usbl_rpy_param[0]) + abs(usbl_rpy_param[1]) + abs(usbl_rpy_param[2]) > 1)
        {
            cout << "abs(usbl_rpy_param[0]) + abs(usbl_rpy_param[1]) + abs(usbl_rpy_param[2]) > 1" << endl;
            cout << summary.BriefReport() << endl;
            beacon_pos << beacon_pos_param[0], beacon_pos_param[1], beacon_pos_param[2];
            cout << "estimated beacon_pos = " << beacon_pos << endl;

            usbl_rpy << usbl_rpy_param[0], usbl_rpy_param[1], usbl_rpy_param[2];
            cout << "estimated usbl_rpy is " << usbl_rpy << endl;
            return false;
        }
	}
	else if (summary.termination_type != ceres::CONVERGENCE)
    {
        cout << "summary.termination_type = ceres::Divergence" << endl;
        cout << summary.BriefReport() << endl;
        beacon_pos << beacon_pos_param[0], beacon_pos_param[1], beacon_pos_param[2];
        cout << "estimated beacon_pos = " << beacon_pos << endl;

        usbl_rpy << usbl_rpy_param[0], usbl_rpy_param[1], usbl_rpy_param[2];
        cout << "estimated usbl_rpy is " << usbl_rpy << endl;
        return false;
    }
    else if (summary.final_cost > summary.initial_cost * threshold_)
    {
        cout << "summary.final_cost > summary.initial_cost * " << threshold_ << endl;
        cout << summary.BriefReport() << endl;
        beacon_pos << beacon_pos_param[0], beacon_pos_param[1], beacon_pos_param[2];
        cout << "estimated beacon_pos = " << beacon_pos << endl;

        usbl_rpy << usbl_rpy_param[0], usbl_rpy_param[1], usbl_rpy_param[2];
        cout << "estimated usbl_rpy is " << usbl_rpy << endl;
        return false;
    }
    else 
	{
        cout << "Initialization diverge " << endl;
        cout << summary.BriefReport() << endl;
        beacon_pos << beacon_pos_param[0], beacon_pos_param[1], beacon_pos_param[2];
        cout << "estimated beacon_pos = " << beacon_pos << endl;

        usbl_rpy << usbl_rpy_param[0], usbl_rpy_param[1], usbl_rpy_param[2];
        cout << "estimated usbl_rpy is " << usbl_rpy << endl;
        return false;
	}
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.push_back(std::make_pair(beacon_pos_param, beacon_pos_param));
    covariance_blocks.push_back(std::make_pair(usbl_rpy_param, usbl_rpy_param));
    covariance_blocks.push_back(std::make_pair(beacon_pos_param, usbl_rpy_param));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    double cov_bb[3 * 3];
    double cov_bu[3 * 3];
    double cov_uu[3 * 3];
    covariance.GetCovarianceBlock(beacon_pos_param, beacon_pos_param, cov_bb);
    covariance.GetCovarianceBlock(usbl_rpy_param, usbl_rpy_param, cov_uu);
    covariance.GetCovarianceBlock(beacon_pos_param, usbl_rpy_param, cov_bu);
    cout << "cov_bb is " << cov_bb[0] << ", " << cov_bb[4] << ", " << cov_bb[8] << endl;
    cout << "cov_uu is " << cov_uu[0] << ", " << cov_uu[4] << ", " << cov_uu[8] << endl;
    
    
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl; 
    // 输出结果
    cout << summary.BriefReport() << endl;
    beacon_pos << beacon_pos_param[0], beacon_pos_param[1], beacon_pos_param[2];
    cout << "estimated beacon_pos = " << beacon_pos << endl;

    usbl_rpy << usbl_rpy_param[0], usbl_rpy_param[1], usbl_rpy_param[2];
    cout << "estimated usbl_rpy is " << usbl_rpy << endl;
    return true;
}

void InitialUSBL::set_threshold(double threshold)
{
    threshold_ = threshold;
    std::cout << "set_threshold, Set threshold is " << threshold_ << std::endl;
}
double InitialUSBL::get_threshold()
{
    return threshold_;
}
