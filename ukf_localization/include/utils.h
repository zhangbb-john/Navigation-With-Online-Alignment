#ifndef UTILS_H
#define UTILS_H
#include <iostream> 
#include <fstream>
#include <math.h>
#include <iomanip>  // std::setprecision()
#include <vector>
double NormalizeAngle(double angle);
void File_init(std::string dir);
int index_first(const std::string &p, const std::string &t);
std::vector<std::string> split(const std::string &s, const char ch);
extern double startSec;
extern int count_no_sensor;
//files
extern std::ofstream timefile;
extern std::ofstream measurefile;
extern std::ofstream logfile;
extern std::ofstream measure_log;
extern std::ofstream odomfile;
extern std::ofstream velocity;
extern std::ofstream statefile;
extern std::ofstream dvlfile;
extern std::ofstream rotfile;
extern std::ofstream timefile;

#endif