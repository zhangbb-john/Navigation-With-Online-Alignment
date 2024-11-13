//
// Created by 陈鑫龙 on 2021/10/19.
//

#include <sys/select.h>
#include <nav_msgs/Odometry.h>

#include "usbl/common.h"

#include "usbl/usbl_process.h"
#include <fstream>
#include <unistd.h>

#define MAXLINE 1024

using namespace std;
double pn = 0, pe = 0, pu = 0, qw = 0, qx, qy, qz;
void odomCallback(const nav_msgs::Odometry odom){
    pn = odom.pose.pose.position.x;
    pe = odom.pose.pose.position.y;
    pu = odom.pose.pose.position.z;
    qw = odom.pose.pose.orientation.w;
    qx = odom.pose.pose.orientation.x;
    qy = odom.pose.pose.orientation.y;
    qz = odom.pose.pose.orientation.z;
}

int main (int argc, char **argv) {
    // if (argc != 3) {
    //     error(1, 0, "usage: usbl_client <IPaddress> <port>");
    // }
    //ros 
    ros::init(argc, argv, "usbl_tcp");
    ros::NodeHandle nh;
    ros::Subscriber obom_subscribe = nh.subscribe(
        "/Locater/Odom", 100, odomCallback);
    std::cout << "usbl tcp starts ++++";

    double start_sec = 0, send_time = -1 , last_time = 0;

    start_sec = ros::Time::now().toSec();
    usbllong_pub = nh.advertise<auv_nav_msg::USBLLONG>("/Sensor/USBLLONG",1, true);
    usblangles_pub = nh.advertise<auv_nav_msg::USBLANGLES>("/Sensor/USBLANGLES",1, true);
    // stop_mission_pub  = nh.advertise<auv_nav_msg::MissionControlData>("/Mission/ControlData",1, true);
    recvim_pub = nh.advertise<auv_nav_msg::RECVIM>("/Sensor/RECVIM",1,true);
    disable_all_and_set_idle_client = nh.serviceClient<std_srvs::Trigger>("/tmr/captain/disable_all_and_set_idle");

    int socket_fd = tcp_client("192.168.0.129", 9200);
    printf ("tcp init!\n");
    cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
    char recv_line[MAXLINE], send_line[MAXLINE];
    int n;
    ofstream usbl_read;
    string  dir;
    ros::param::get("dir",dir);
    std::string file_usbl( dir + "/src/ukf_localization/usbl/beacon/usbl.txt");
    usbl_read.open(file_usbl.c_str(),std::ios::out);

    // 声明监听集合
    fd_set readmask;
    fd_set allreads;

    // 清空allreads集合
    FD_ZERO(&allreads);
    // 将标准输入（fd = 0）加入 allreads集合
    FD_SET(0, &allreads);
    // 将usbl和auv主控的TCP连接的fd加入allreads集合
    FD_SET(socket_fd, &allreads);
    std::cout << 20220111 << std::endl;
    string source_level = "+++AT!L" + to_string(0) + "\n";
    send_data(socket_fd, source_level.c_str(), source_level.size());
    sleep(1);
    string save = "+++AT&W\n";
    send_data(socket_fd, save.c_str(), save.size());  
    sleep(1);
    string show = "+++AT&V\n" ;
    send_data(socket_fd, show.c_str(), show.size());  
   
    
    double last_spin = ros::Time::now().toSec();

    for (;;) {
        // select会改变readmask，每次都需要重新copy
        readmask = allreads;
        // 只检测 读 事件
        int rc = select(socket_fd + 1, &readmask, NULL, NULL, NULL);

        if (rc <= 0) {
            error(1, errno, "select failed");
        }

        if (FD_ISSET(socket_fd, &readmask)) {
            n = read_data(socket_fd, recv_line, MAXLINE - 1);
            if (n < 0) {
                error(1, errno, "read error");
            } else if (n == 0) {
                error(1, 0, "server terminated \n");
            }

            // data process
            // get location from read data
            // and send it to usbl
            recv_line[n] = '\0';

            // 显示在屏幕
            fputs("recv msg: ", stdout);
            fputs(recv_line, stdout);
            fputs("\n", stdout);
            usbl_read << recv_line << endl;

            string message = usbl_process(recv_line);
            if (message.length() == 0) {
                std :: ostringstream str; 
                str << std::fixed << std::setprecision(2) << "ODOM," << pn << "," << pe << "," << pu << "," << setprecision(4) << qx << "," << qy << "," << qz << "," << qw; 
                message = str.str();
                
                std::cout << message << std::endl;
                string sendim = "+++AT*SENDIM," + to_string(message.length()) + ",2,ack," + message + "\n";

                send_time = ros::Time::now().toSec();
                if ( send_time - last_time >= 1 ){
                    cout << "time :"<< send_time - start_sec << ";send msg: " << sendim << endl;

                    send_data(socket_fd, sendim.c_str(), sendim.size());
                    last_time = send_time ; 
                }                
                continue;
            }
            string sendim = "+++AT*SENDIM," + to_string(message.length()) + ",2,ack," + message + "\n";

            send_time = ros::Time::now().toSec();
            if ( send_time - last_time >= 0.02 ){
                cout << "time :"<< send_time - start_sec << ";send msg: " << sendim << endl;

                send_data(socket_fd, sendim.c_str(), sendim.size());
                last_time = send_time ; 
            }
            // send_data(socket_fd, sendim.c_str(), sendim.size());
//            fputs(sendim.c_str(), socket_fd);
        }

        if (FD_ISSET(STDIN_FILENO, &readmask)) {
            if (fgets(send_line, MAXLINE - 1, stdin) != NULL) {
                int i = strlen(send_line);
                if (send_line[i - 1] == '\n') {
                    send_line[i] = 0;
                }

                printf("now sending %s\n", send_line);
                ssize_t rt = write(socket_fd, send_line, strlen(send_line));
                if (rt < 0) {
                    error(1, errno, "write failed ");
                }
                printf("send bytes: %zu \n", rt);
            }
        }
        if(ros::Time::now().toSec() - last_spin >= 1){
            ros::spinOnce();
            last_spin = ros::Time::now().toSec();
        }

    }
    exit(0);
}