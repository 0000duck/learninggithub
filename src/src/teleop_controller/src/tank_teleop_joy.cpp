
#include <ros/ros.h>
#include"std_msgs/Float32.h" 
#include<string>
#include<vector>
#include <sensor_msgs/Joy.h> 



// create the TeleopTurtle class and define the joyCallback function that will take a joy msg 
class TeleopTank
{ 
public: 
    TeleopTank(); 

    
private: 
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy); 
    void Data_send();
    
    ros::NodeHandle nh_; 
    
    ros::Timer loop_timer_;
    int dir_,str_,vel_,rev_; 
    
    // used to define which axes of the joystick will control our Tank 
    double dir_scale_, str_scale_,vel_scale_; 
    //float cmd_dir_,cmd_str_,cmd_vel_;
    ros::Publisher dir_pub_; 
    ros::Publisher str_pub_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_; 

    std_msgs::Float32 cmd_vel_;
    std_msgs::Float32 cmd_str_;
    std_msgs::Float32 cmd_dir_;
    std_msgs::Float32 cmd_vel;
    std_msgs::Float32 cmd_str;
	std_msgs::Float32 cmd_dir;
    int receive=0;


}; 
    
    
TeleopTank::TeleopTank(): 
 str_(0),
 vel_(5),
 rev_(1) 

{ 

    //auto &pnh = nh_;
    //  initialize some parameters 
    nh_.param("axis_str", str_, str_); 
    nh_.param("axis_vel", vel_, vel_); 
    nh_.param("button_rev",rev_, rev_);
    nh_.param("scale_str", str_scale_, str_scale_); 
    nh_.param("scale_vel", vel_scale_, vel_scale_); 


    // create a publisher that will advertise on the command_velocity topic of the turtle 
    this->dir_pub_ = this->nh_.advertise<std_msgs::Float32>("cmd_dir", 10);//发布“cmd_dir“为话题的消息，第二个参数表示消息发布队列的大小
	this->str_pub_ = this->nh_.advertise<std_msgs::Float32>("cmd_str", 10);
	this->vel_pub_ = this->nh_.advertise<std_msgs::Float32>("cmd_vel", 10);
    this->cmd_vel_.data = 0;
    this->cmd_str_.data = 0;
    this->cmd_dir_.data = 0;
    this->cmd_vel.data = 0;
    this->cmd_str.data = 0;
    this->cmd_dir.data = 0;    
     // subscribe to the joystick topic for the input to drive the Tank 

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTank::joyCallback, this); 

    loop_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&TeleopTank::Data_send, this));
}


void TeleopTank::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
{ 

    // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the Tank 
     cmd_str.data = (str_scale_*joy->axes[str_]); //角度
     receive = joy->buttons[rev_];
     cmd_vel.data = (vel_scale_*joy->axes[vel_]);

}

void TeleopTank::Data_send()
{
       cmd_str_.data = abs(50*cmd_str.data); //角度
if(cmd_str.data>0)
    {
        cmd_dir_.data = 1;
    }
else if(cmd_str.data<0)
    {
        cmd_dir_.data = -1;
    }
else
    {
        cmd_dir_.data = 0;
    }

    if(receive)
    {
        cmd_vel_.data = 6*((cmd_vel.data)-2);//倒车
    }
    else
    {
        cmd_vel_.data = -0.9*16*((cmd_vel.data)-2);//cmd_vel_.data = 0.9*16*((cmd_vel.data)+2);
    }
    

    dir_pub_.publish(cmd_dir_);
    str_pub_.publish(cmd_str_);
    vel_pub_.publish(cmd_vel_);  
}


int main(int argc, char** argv) 
{ 
    // initialize our ROS node, create a teleop_tank, and spin our node until Ctrl-C is pressed 
    ros::init(argc, argv, "teleop_tank"); 
    TeleopTank teleop_tank;
    // teleop_tank.init();
    // teleop_tank.run();

    ros::spin(); 
}
