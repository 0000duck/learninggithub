先启动ROS
$ roscore

1. 利用gmapping功能包进行SLAM
（1）启动仿真环境和gmapping节点
   $ roslaunch mycar_gazebo laser_nav_gazebo.launch 
   $ roslaunch mycar_navigation gmapping_demo.launch 
 (2) 用键盘控制小车移动
   $ roslaunch mrobot_teleop mrobot_teleop.launch
    
    i     前进
    u,o   左/右转
    k     停
    j,l   原地左/右转
    <     后退
    m,>   后左/右转

2. 利用hector功能包进行SLAM
 （1）启动仿真环境和hector节点
    $ roslaunch mycar_gazebo laser_nav_gazebo.launch
    $ roslaunch mycar_navigation hector_demo.launch
  (2) 用键盘控制小车移动
    $ roslaunch mrobot_teleop mrobot_teleop.launch

3. 自主探索导航功能演示
  (1) 启动仿真环境和相关功能包
    $ roslaunch mycar_gazebo laser_nav_gazebo.launch
    $ roslaunch mycar_navigation exploring_slam_demo
  (2) 运行自主探索Python脚本
    $ rosrun mycar_navigation exploring_slam.py


