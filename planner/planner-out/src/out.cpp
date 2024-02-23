
#include<ros/ros.h>

#include<std_msgs/String.h>

#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
//接收到消息后进入回调函数

void traj_planning_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg);

struct current_position_def
{
  double x;
  double y;
  double z;
  double orb_x;
  double orb_y;  
  double orb_z;
  double x_fu;
  double y_fu;
};

current_position_def current_position, planning_vel,planning_position,planning_acc,controller_force_,controller_orientation_, planning_yaw;

//回调函数是订阅节点接收消息的基础机制，当有消息到达时会自动以消息指针作为参数，再调用回调函数，完成对消息内容的处理，如上是一个简单的回调函数，用来接收publisher发布的string消息，并将其打印出来

int main(int argc,char **argv)

{
//初始化节点
  string traj_save = "traj_save.txt";//当前位姿
  ofstream f_t_s;
  f_t_s.open(traj_save.c_str()); // delete previos data from last run.
  f_t_s.close();


ros::init(argc,argv,"listener");

ros::NodeHandle n;

ros::Subscriber sub=n.subscribe("/drone_0_planning/pos_cmd",1000,traj_planning_callback);

//订阅节点首先需要声明自己订阅的消息话题，该消息会在ros master中注册，master会关注系统中是否存在发布该话题的节点，如果存在则会帮助两个节点建立连接，完成数据的传输，NodeHandle::subscriber()用来创建一个Subscriber，第一个参数即为消息话题，第二个是消息队列的大小，第三个参数是接收到话题消息后的回调函数.

ros::spin();//节点进入循环状态，有消息到达时调用回调函数完成处理

return 0;

}



//保存订阅的期望planning位置话题信息以及速度信息
void traj_planning_callback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
  planning_position.x = msg -> position.x;
  planning_position.y = msg -> position.y;
  planning_position.z = msg -> position.z;
  cout<<"保存期望planning postion"<<endl;
  ros::Time currentTime2 = ros::Time::now(); 
  string traj_planning = "traj_planning.txt";//当前位姿
  ofstream f_t_p;
  f_t_p.open(traj_planning.c_str(),ios::app);
  f_t_p << fixed;
  f_t_p << currentTime2 << setprecision(4) <<"    "<< planning_position.x  <<"    "<< planning_position.y <<"    "<< planning_position.z <<endl;
  f_t_p.close();


  planning_vel.x = msg -> velocity.x;
  planning_vel.y = msg -> velocity.y;
  planning_vel.z = msg -> velocity.z;
  cout<<"保存期望planning velocity"<<endl;
  ros::Time currentTime3 = ros::Time::now(); 
  string velocity_planning = "velocity_planning.txt";//当前位姿
  ofstream f_v_p;
  f_v_p.open(velocity_planning.c_str(),ios::app);
  f_v_p << fixed;
  f_v_p << currentTime3 << setprecision(4) <<"    "<< planning_vel.x  <<"    "<< planning_vel.y <<"    "<< planning_vel.z <<endl;
  f_v_p.close();

  planning_acc.x = msg -> acceleration.x;
  planning_acc.y = msg -> acceleration.y;
  planning_acc.z = msg -> acceleration.z;
  cout<<"保存期望planning  acceleration"<<endl;
  ros::Time currentTime4 = ros::Time::now(); 
  string acc_planning = "acceleration_planning.txt";//当前位姿
  ofstream f_a_p;
  f_a_p.open(acc_planning.c_str(),ios::app);
  f_a_p << fixed;
  f_a_p << currentTime4 << setprecision(4) <<"    "<< planning_acc.x  <<"    "<< planning_acc.y <<"    "<< planning_acc.z <<endl;
  f_a_p.close();

 planning_yaw.x= msg -> yaw;
 cout<<"保存期望planning  yaw"<<endl;
  ros::Time currentTime5 = ros::Time::now(); 
  string yaw_planning = "yaw_planning.txt";//当前位姿
  ofstream f_y_p;
  f_y_p.open(yaw_planning.c_str(),ios::app);
  f_y_p << fixed;
  f_y_p << currentTime5 << setprecision(4) <<"    "<< planning_yaw.x  <<endl;
  f_y_p.close();
}
