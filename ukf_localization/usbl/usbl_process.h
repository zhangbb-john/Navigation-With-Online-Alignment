//
// Created by 陈鑫龙 on 2021/10/19.
//

#ifndef USBL_DATA_PROCESS_H
#define USBL_DATA_PROCESS_H
#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <vector>
#include "auv_nav_msg/USBLLONG.h"
#include "auv_nav_msg/USBLANGLES.h"
// #include "auv_nav_msg/MissionControlData.h"
#include "auv_nav_msg/RECVIM.h"
#include <std_srvs/Trigger.h>

// ros::Publisher usbllong_pub, usblangles_pub, stop_mission_pub, recvim_pub;

ros::Publisher usbllong_pub, usblangles_pub, recvim_pub;
ros::ServiceClient disable_all_and_set_idle_client;
auv_nav_msg::RECVIM recvim;
bool WAIT_TIME = false;

typedef struct USBLLONG_PACKAGE
{
    float current_time = 0;
    float measurement_time = 0;
    int remote_address = 0;

    float x = 0;
    float y = 0;
    float z = 0;

    float e = 0;
    float n = 0;
    float u = 0;

    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    int propagation_time = 0;
    int rssi = 0;
    int integrity = 0;

    float accuracy;

    void init_time(float current_time_, float measurement_time_)
    {
        current_time = current_time_;
        measurement_time = measurement_time_;
    }

    void init_env(float e_, float n_, float u_)
    {
        e = e_;
        n = n_;
        u = u_;
    }

    void init_rpw(float roll_, float pitch_, float yaw_)
    {
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
    }

    friend std::ostream &operator<<(std::ostream &os, const USBLLONG_PACKAGE &usbllong)
    {
        os << "USBLLONG,";

        // os << std::setprecision(6) << std::setiosflags(std::ios::fixed) << usbllong.measurement_time << ",";

        os << std::setprecision(2) << std::setiosflags(std::ios::fixed) << usbllong.e << ",";
        os << std::setprecision(2) << std::setiosflags(std::ios::fixed) << usbllong.n << ",";
        os << std::setprecision(2) << std::setiosflags(std::ios::fixed) << usbllong.u << ",";

        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usbllong.roll << ",";
        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usbllong.pitch << ",";
        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usbllong.yaw;

        return os;
    }

} USBLLONG;

typedef struct USBLANGLES_PACKAGE
{
    float current_time = 0;
    float measurement_time = 0;

    int remote_address = 0;
    double lbearing = 0;
    double lelevation = 0;

    double bearing = 0;
    double elevation = 0;

    float roll = 0;
    float pitch = 0;
    float yaw = 0;

    int rssi = 0;
    int integrity = 0;

    float accuracy;

    void init_time(float current_time_, float measurement_time_)
    {
        current_time = current_time_;
        measurement_time = measurement_time_;
    }

    void init_be(double bearing_, double elevation_)
    {
        bearing = bearing_;
        elevation = elevation_;
    }

    void init_rpw(float roll_, float pitch_, float yaw_)
    {
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;
    }

    friend std::ostream &operator<<(std::ostream &os, const USBLANGLES_PACKAGE &usblangles)
    {
        os << "USBLANGLES,";

        // os << std::setprecision(6) << std::setiosflags(std::ios::fixed) << usblangles.measurement_time << ",";

        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usblangles.bearing << ",";
        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usblangles.elevation << ",";

        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usblangles.roll << ",";
        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usblangles.pitch << ",";
        os << std::setprecision(4) << std::setiosflags(std::ios::fixed) << usblangles.yaw;

        return os;
    }

} USBLANGLES;

// 字符串分割
std::vector<std::string> split(const std::string &s, const char ch)
{
    int start = 0;
    int len = 0;
    std::vector<std::string> ret;
    for (int i = 0; i < s.length(); i++)
    {
        if (s[i] == ch)
        {
            ret.push_back(s.substr(start, len));
            start = i + 1;
            len = 0;
        }
        else
        {
            len++;
        }
    }
    if (start < s.length())
    {
        ret.push_back(s.substr(start, len));
    }
    return std::move(ret);
}

// p被匹配的串，t模式串
// 找到返回下标，否则-1
// 朴素算法，时间复杂度O(n*m)
int index_first(const std::string &p, const std::string &t)
{
    if (p.length() < t.length())
    {
        return -1;
    }
    size_t end = p.length() - t.length();
    for (size_t i = 0; i < end; ++i)
    {
        bool flag = true;
        for (size_t j = 0; j < t.length(); ++j)
        {
            if (p[i + j] != t[j])
            {
                flag = false;
                break;
            }
        }
        if (flag)
        {
            return i;
        }
    }
    return -1;
}

#include <sstream>
auv_nav_msg::USBLLONG usbllong_msg(const std::string &s)
{
    auto split_ret = split(s, ',');
    if (split_ret.size() != 17)
    {
        std::cout << "error" << std::endl;
    }
    auv_nav_msg::USBLLONG usbllong;
    usbllong.header.stamp = ros::Time::now();
    usbllong.current_time = stof(split_ret[1]);
    usbllong.measurement_time = stof(split_ret[2]);
    usbllong.pos_xyz.x = stof(split_ret[4]);
    usbllong.pos_xyz.y = stof(split_ret[5]);
    usbllong.pos_xyz.z = stof(split_ret[6]);
    usbllong.pos_enu.x = stof(split_ret[7]);
    usbllong.pos_enu.y = stof(split_ret[8]);
    usbllong.pos_enu.z = stof(split_ret[9]);
    usbllong.r = stof(split_ret[10]);
    usbllong.p = stof(split_ret[11]);
    usbllong.y = stof(split_ret[12]);
    usbllong.propagation_time = stoi(split_ret[13]);
    usbllong.rssi = stoi(split_ret[14]);
    usbllong.integrity = stoi(split_ret[15]);
    usbllong.accuracy = stof(split_ret[16]);
    return usbllong;
}
std::string usbllong_process(const std::string &s)
{
    auto split_ret = split(s, ',');
    if (split_ret.size() != 17)
    {
        return "";
    }
    USBLLONG usbllong;
    usbllong.init_time(stof(split_ret[1]), stof(split_ret[2]));
    usbllong.init_env(stof(split_ret[7]), stof(split_ret[8]), stof(split_ret[9]));
    usbllong.init_rpw(stof(split_ret[10]), stof(split_ret[11]), stof(split_ret[12]));

    usbllong_pub.publish(usbllong_msg(s));

    std::stringstream ss;
    ss << usbllong;

    return ss.str();
}

auv_nav_msg::USBLANGLES usblangles_msg(const std::string &s)
{
    auto split_ret = split(s, ',');
    if (split_ret.size() != 14)
    {
        std::cout << "error" << std::endl;
    }
    auv_nav_msg::USBLANGLES usblangles;
    usblangles.header.stamp = ros::Time::now();
    usblangles.current_time = stof(split_ret[1]);
    usblangles.measurement_time = stof(split_ret[2]);
    usblangles.lbearing = stof(split_ret[4]);
    usblangles.lelevation = stof(split_ret[5]);
    usblangles.bearing = stof(split_ret[6]);
    usblangles.elevation = stof(split_ret[7]);
    usblangles.r = stof(split_ret[8]);
    usblangles.p = stof(split_ret[9]);
    usblangles.y = stof(split_ret[10]);
    usblangles.rssi = stoi(split_ret[11]);
    usblangles.integrity = stoi(split_ret[12]);
    usblangles.accuracy = stof(split_ret[13]);

    return usblangles;
}
std::string usblangles_process(const std::string &s)
{
    auto split_ret = split(s, ',');
    if (split_ret.size() != 14)
    {
        return "";
    }
    USBLANGLES usblangles;
    usblangles.init_time(stof(split_ret[1]), stof(split_ret[2]));
    usblangles.init_be(stod(split_ret[6]), stod(split_ret[7]));
    usblangles.init_rpw(stof(split_ret[8]), stof(split_ret[9]), stof(split_ret[10]));

    usblangles_pub.publish(usblangles_msg(s));

    std::stringstream ss;
    ss << usblangles;
    return ss.str();
}

std::string recvim_process(const std::string &s)
{
    auto split_ret = split(s, ',');
    if (split_ret.size() != 10)
    {
        std::cout << "error" << std::endl;
    }

    recvim.header.stamp = ros::Time::now();
    recvim.duration = stoi(split_ret[5]);
    recvim.rssi = stoi(split_ret[6]);
    recvim.integrity = stoi(split_ret[7]);
    recvim.velocity = stof(split_ret[8]);
    recvim.data = split_ret[9];
    recvim.propagation_time = 0;
    WAIT_TIME = true;
    recvim_pub.publish(recvim);
    // return "AT?T";
    return " ";
}
// std::string time_process(const std::string &s)
// {
//     if (WAIT_TIME)
//     {
//         auto split_ret = split(s, ':');
//         if (split_ret.size() != 3)
//         {
//             std::cout << "error" << std::endl;
//         }
//         recvim.propagation_time = stoi(split_ret[2]);
//         recvim_pub.publish(recvim);
//         WAIT_TIME = false;
//     }
//     return "";
// }
std::string usbl_process(const std::string &s)
{
    if (index_first(s, "USBLLONG") != -1)
    {
        return usbllong_process(s);
    }
    else if (index_first(s, "USBLANGLES") != -1)
    {
        return usblangles_process(s);
    }
    else if (index_first(s, "stop") != -1)
    {

        std_srvs::Trigger trigger;

        bool success = disable_all_and_set_idle_client.call(trigger);
        if (!success)
        {
            return "StopFailed";
        }
        else if (!trigger.response.success)
        {
            return "StopResponseFailed";
        }
        return "StopSucceeded";
        // auv_nav_msg::MissionControlData stopMsg;
        // stopMsg.header.stamp = ros::Time::now();
        // stopMsg.mission_cancel = true;
        // stop_mission_pub.publish(stopMsg);
        // return "StopPublished";
    }
    else if (index_first(s, "RECVIM") != -1)
    {
        return recvim_process(s);
    }
    // else if (index_first(s, "AT?T") != -1)
    // {
    //     return time_process(s);
    // }
    return "";
}
#endif //USBL_DATA_PROCESS_H
