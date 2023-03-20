#include <utils.h>

double startSec ;
int count_no_sensor;
//files 
std::ofstream timefile; 
std::ofstream measurefile;
std::ofstream logfile;
std::ofstream measure_log;
std::ofstream odomfile;
std::ofstream velocity;
std::ofstream statefile;
std::ofstream dvlfile;
std::ofstream rotfile;

double NormalizeAngle(double angle) {
    double a = fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) 
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}
void File_init(std::string dir)
{
    std::cout<<"dir is "<<dir<<std::endl;
    std::string log_dir = dir+"/log";
    std::string date = "_20210622";
    // record time when no sensor received.
    // std::string timefile_str = log_dir+"/time.txt";
    // timefile.open(timefile_str.c_str(),std::ios::out);
    
    //record type of received measurement.
    // std::string measurefile_str = log_dir+"/measure.txt";
    // measurefile.open(measurefile_str.c_str(),std::ios::out);

    // record state vector before and after measure
    std::string logfile_str = log_dir+"/log"+date+".txt";
    logfile.open(logfile_str.c_str(),std::ios::out);
    // imuCallback and so on also record received type
    // std::string measure_log_str = log_dir+"/measure_log.txt";
    // measure_log.open(measure_log_str.c_str(),std::ios::out);
    
    // x y z +v
    std::string odomfile_str = log_dir + "/odom.txt";
    odomfile.open(odomfile_str.c_str(),std::ios::out);

    std::string velocity_str = log_dir + "/velocity.txt";
    velocity.open(velocity_str.c_str(), std::ios::out);

    std::string state_str = log_dir + "/state.txt";
    statefile.open(state_str.c_str(), std::ios::out);
    
    std::string dvl_str = log_dir + "/dvl.txt";
    dvlfile.open(dvl_str.c_str(), std::ios::out);

    std::string rot_str = log_dir + "/rot.txt";
    rotfile.open(rot_str.c_str(), std::ios::out);

    std::string time_str = log_dir + "/time.txt";
    // timefile.open(time_str.c_str(), std::ios::out);

}

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
