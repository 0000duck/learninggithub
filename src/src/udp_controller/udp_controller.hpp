//
// Created by aidous on 18-12-3.
//

#ifndef UDP_SEND_UDP_HPP
#define UDP_SEND_UDP_HPP

#endif //UDP_SEND_UDP_HPP

#include <ros/ros.h>
#include <iostream>
#include <sys/types.h>
//#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <queue>
#include <chrono>
#include <string>
#include <vector>
#include"std_msgs/Float32.h" 
//#include "control_msgs/GetECUReport.h"
//#include "control_msgs/SteerCmd.h"
//#include "speed_ctrl_msgs/speed_ctrl.h"
//#include "sensor_driver_msgs/GpswithHeading.h"
//#include "transform_tools/transform_interface.hpp"

namespace carnet{
    struct CarState{
        double expvel;
        double expsteering;
    };

    class udp_controller{
    public:
        udp_controller(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                 std::string node_name);

        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Timer loop_timer_;
        std::string node_name_{"udp_sender_node"};

        ros::Publisher  expveldata_pub_;
        ros::Publisher  expdirdata_pub_;
        ros::Publisher  expsteerdata_pub_;

        std_msgs::Float32 cmd_vel_;
        std_msgs::Float32 cmd_str_;
	    std_msgs::Float32 cmd_dir_;

         bool cmddataupdate;


        //// UDP Settings
        /* local address and port */
        int CarNetSocket;
        sockaddr_in addr_local;
        int local_port;

        /* remote address and port */
        sockaddr_in addr_remote;
        socklen_t addr_remote_len; //// do not forget to init the 'addr_remote_len'
        std::string remote_ip;
        int remote_port;


        CarState carstate;

        /* init、receive and send functions */
        char RecvData[150];
        char temp_RecvData[150];

        void initialize();

        void timerCb();

        bool initSocket(void);
        //void sendmsgs();
        void UpdateMsg();
        void recvmsgs(char ch);
    };
}//namespace carnet
