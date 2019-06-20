#include "udp_controller.hpp"

namespace carnet{
    udp_controller::udp_controller(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                       std::string node_name)
            :nh_(nh),
             nh_private_(nh_private),
             node_name_(node_name){
        initialize();
    }



    bool udp_controller::initSocket() {

        this->addr_remote_len = sizeof(this->addr_local);

        //// create a socket
        this->CarNetSocket = socket(AF_INET, SOCK_DGRAM, 0);
        if(this->CarNetSocket < 0)
        {
            perror("create CarNetSocket failed!\n");
            return false;
        }
        else
        {
            std::cout<<"create CarNetSocket succeed!"<<std::endl;
        }
        //// set the local address
        memset((char*)&addr_local, 0, sizeof(addr_local));
        this->addr_local.sin_addr.s_addr = htonl(INADDR_ANY);
        this->addr_local.sin_family = AF_INET;
        this->addr_local.sin_port = htons(local_port);

        //// bind the socket with local address
        if(bind(CarNetSocket, (sockaddr*)&addr_local, sizeof(sockaddr)) < 0)
        {
            perror("bind the CarNetSocket failed!");
            return false;
        }
        else
        {
            std::cout<<"bind the CarNetSocket succeed!"<<std::endl;
            std::cout<<"Local Port : "<< this->local_port<<std::endl;
        }
        //// set the remote address
        //memset(&addr_remote,0,sizeof(addr_remote));
        this->addr_remote.sin_addr.s_addr = inet_addr(remote_ip.c_str());
        this->addr_remote.sin_family = AF_INET;
        this->addr_remote.sin_port = htons(remote_port);

        std::cout<<"Remote IP  : "<< this->remote_ip.c_str()<<std::endl;
        std::cout<<"Remote Port: "<< this->remote_port<<std::endl;

        return true;
    }

    void udp_controller::UpdateMsg(){
        //// Receive Other Car State
        unsigned char recvBuf[4096];  //// receive buffer
        int recvlen;                  //// bytes received
        recvlen = recvfrom(CarNetSocket, recvBuf, 4096, MSG_DONTWAIT, (struct sockaddr *)&addr_remote, &addr_remote_len);
        if(recvlen > 0 && this->addr_remote.sin_port == htons(this->remote_port))
        {
            std::cout<<" RECV DATA NOW!!! "<<std::endl;
            cmddataupdate = true;
            for (int i = 0; i < recvlen; i++)
            {
                recvmsgs(recvBuf[i]);
            }

        }
    }

    void udp_controller::recvmsgs(char ch){
        static char static_cHeadFlag = 0;
        static char static_cTailFlag = 0;
        static char static_cOverFlag = 0;

        static int static_cRXDataNum = 0;

        static int static_cRXAfterStarDataNum = 0;

        static char static_cRXDataChecksum = 0; //校验位
        static char static_cRXDataCsc[1];

        char cTemp = ch;

        if('$' == cTemp)
        {
            static_cHeadFlag = 1;//找到了包头‘$’
            static_cTailFlag = 0;
            static_cOverFlag = 0;
            static_cRXDataChecksum = cTemp;
            static_cRXDataNum = 0;
            memset(RecvData,0,150);
        }
        if(1 == static_cHeadFlag)
        {
            if('*' == cTemp)
                static_cTailFlag = 1;
            else  //没找到包尾，存数 求校验
            {
                this->RecvData[static_cRXDataNum++] = cTemp;
                static_cRXAfterStarDataNum = 0;
            }
            if(static_cTailFlag == 1)
            {
                static_cRXDataCsc[static_cRXAfterStarDataNum++] = cTemp;
                if(1 == static_cRXAfterStarDataNum)
                {
                    if(/*(static_cRXDataCsc[1] == cTempRXDataCsc1) && (static_cRXDataCsc[2] == cTempRXDataCsc2)*/1)
                    {
                        char RecvDataHead[10];
                        char *pStr = NULL;
                        //char *next_token = NULL;
                        char seps[] = ",\t\n";
                        double dTemp = 0.0;

                        int i = 0;
                        int j = 0;
                        int DataCommaNum = 0;
                        char tempData[15];

                        memset(RecvDataHead,0,10);
                        memset(tempData,0,15);
                        memcpy(this->temp_RecvData,this->RecvData,150);
                        pStr = strtok(this->RecvData,seps);
                        if(pStr)
                        {
                            strcpy(RecvDataHead,pStr);
                            if(0 == strcmp(RecvDataHead,"$"))
                            {
                                while( i++ <= static_cRXDataNum)
                                {
                                    if(i > 150)
                                        i = 0;
                                    if(',' == this->temp_RecvData[i])
                                    {
                                        j = 0;
                                        DataCommaNum++;
                                        switch(DataCommaNum)
                                        {
                                            case 1:
                                                dTemp = atof(tempData);
                                                memset(tempData,0,15);
                                                break;
                                            case 2:
                                                dTemp = atof(tempData);
                                                carstate.expvel  = dTemp;
                                                memset(tempData,0,15);
                                                break;
                                            case 3:
                                                dTemp = atof(tempData);
                                                carstate.expsteering = dTemp;
                                                memset(tempData,0,15);
                                                break;
                                            default:
                                                break;
                                        }
                                    }
                                    else
                                    {
                                        tempData[j++] = this->temp_RecvData[i];
                                    }
                                }
                            }
                            else
                            {
                                static_cOverFlag = 1;
                            }
                        }
                        else // 数据解析完毕
                        {
                            static_cOverFlag = 1;
                        }
                    }
                }
                else if(static_cRXAfterStarDataNum > 1)
                {
                    static_cOverFlag = 1;
                }
            }
            if(static_cRXDataNum >= 150)
            {
                static_cOverFlag = 1;
            }
        }
        if(1 == static_cOverFlag)
        {
            static_cHeadFlag   = 1;
            static_cOverFlag   = 0;
            static_cTailFlag   = 0;
            static_cRXDataChecksum = 0;
            static_cRXDataNum = 0;
            static_cRXAfterStarDataNum = 0;
            memset(this->RecvData,0,150);
        }

    }

    void udp_controller::initialize(){
        local_port = 8001;
        remote_ip = "127.0.0.1"; //"127.0.0.1";自机通讯地址//
        remote_port = 8000;

        //test
        carstate.expvel = 0.0;
        carstate.expsteering = 0.0;

        initSocket();

        auto &pnh = nh_;

        expsteerdata_pub_ = pnh.advertise<std_msgs::Float32>("cmd_str",10);

        expdirdata_pub_ = pnh.advertise<std_msgs::Float32>("cmd_dir",10);

        expveldata_pub_ = pnh.advertise<std_msgs::Float32>("cmd_vel",10);

        this->cmd_vel_.data = 0;
        this->cmd_str_.data = 0;
        this->cmd_dir_.data = 0;

        loop_timer_ = pnh.createTimer(ros::Duration(0.05), boost::bind(&udp_controller::timerCb, this));
    }

    void udp_controller::timerCb() {
        ROS_INFO_ONCE("udp car start");
        UpdateMsg();
        std::cout<<"carstate.expvel: "<<carstate.expvel<<std::endl;
        std::cout<<"carstate.expsteering: "<<carstate.expsteering<<std::endl;


        cmd_str_.data = double(fabs(carstate.expsteering));//角度

        if(carstate.expsteering>0)//方向
        {
            cmd_dir_.data = 1;
        }
        else  if(carstate.expsteering<0)
        {
            cmd_dir_.data = -1;
        }
        else cmd_dir_.data = 0;

        cmd_vel_.data =0.9*double(carstate.expvel);//速度
        
         expsteerdata_pub_.publish(cmd_str_);
         expdirdata_pub_.publish(cmd_dir_);
         expveldata_pub_.publish(cmd_vel_);
        
    }
}//namespace carnet

int main(int argc, char** argv) {
    std::string node_name = "udp_controller";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    carnet::udp_controller sender(nh, nh_private, node_name);
    ROS_INFO("Initialized sender node.");
    ros::spin();
}
