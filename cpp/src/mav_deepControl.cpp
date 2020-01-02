/*****************************************************************************
-----------PRATYUSH VARSHNEY-----------------
------------pratyushvarshney@cse.iitk.ac.in-------------
*****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "include/mav_nonlinear_mpc/params.hpp"
// #include <params.hpp>
using namespace Eigen;
// using namespace Eigen;
#define NO_OF_STATES 6

geometry_msgs::PoseStamped pose_;
mav_msgs::RollPitchYawrateThrust mpc_;
nav_msgs::Odometry odom_;

void pose_cb(const geometry_msgs::PoseStamped msg)
{
    pose_ = msg;
    return;
}

void mpc_cb(const mav_msgs::RollPitchYawrateThrust msg)
{
    mpc_ = msg;
    return;
}

void odom_cb(const nav_msgs::Odometry msg)
{
    odom_ = msg;
    return;
}



static void toEulerAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
  yaw = atan2(siny_cosp, cosy_cosp);
}

float max(float a, float b){
  if(a < b)
    return b;
  else
    return a;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "weight_mpc");
  ros::NodeHandle nh;

  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("command/pose",1000,pose_cb);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("ground_truth/odometry",1000,odom_cb);
  ros::Subscriber mpc_sub = nh.subscribe<mav_msgs::RollPitchYawrateThrust>("rpyth",1000,mpc_cb);
  
  ros::Publisher  rpyth_pub = nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1000);
  
  ros::Rate loop_rate(100);
  weights np;

  double x,y,z, vx,vy,vz, rs,ps,ys;
  mav_msgs::RollPitchYawrateThrust rpyth;



  while (nh.ok()) 
  {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    toEulerAngle(odom_.pose.pose.orientation, roll, pitch, yaw);

    x = odom_.pose.pose.position.x;
    y = odom_.pose.pose.position.y;
    z = odom_.pose.pose.position.z;


    vx = odom_.twist.twist.linear.x;
    vy = odom_.twist.twist.linear.y;
    vz = odom_.twist.twist.linear.z;

    VectorXf target(NO_OF_STATES);
    target(0) = pose_.pose.position.x;
    target(1) = pose_.pose.position.y;
    target(2) = 0.0;

    target(3) = 0.0;
    target(4) = 0.0;
    target(5) = 0.0;

    VectorXf state_(NO_OF_STATES);
    
    state_(0) = x;
    state_(1) = y;

    state_(2) = vx;
    state_(3) = vy;

    state_(4) = roll;
    state_(5) = pitch;




    Matrix<float,1,NO_OF_STATES> input_;

    for(int i = 0; i < NO_OF_STATES; i++)
      input_(0,i) = state_(i) -target(i);

    
    for(int i = 0; i < NO_OF_STATES; i++){ 
      input_(0,i) = (input_(0,i) - np.mean(i,0)) / np.std(i,0);
    }

    

    VectorXf controls_(2);

    // LAYER 1
    Matrix<float,1,64> op1_w = (input_ * np.layer_1_weight.transpose());
    Matrix<float,1,64> op1 = op1_w + np.bias_1.transpose();

    for(int i = 0; i < 64; i++)
      op1(0,i) = max(0, op1(0,i));


    //LAYER 2

    Matrix<float,1,64> op2_w = (op1 * np.layer_2_weight.transpose());
    Matrix<float,1,64> op2 = op2_w + np.bias_2.transpose();

    for(int i = 0; i < 64; i++)
      op2(0,i) = max(0, op2(0,i));

    


    // LAYER 3
    Matrix<float,1,2> op3_w = (op2 * np.layer_3_weight.transpose());
    Matrix<float,1,2> op3_b = op3_w + np.bias_3.transpose();
    ArrayWrapper<Matrix<float,1,2> > op3(op3_b); 
    op3.tanh();

    rpyth.header.stamp = ros::Time::now();
    rpyth.roll  = op3(0,0);
    rpyth.pitch = op3(0,1);
    rpyth.yaw_rate = mpc_.yaw_rate;
    rpyth.thrust.z = mpc_.thrust.z;

    rpyth_pub.publish(rpyth);
    ros::spinOnce(); 

    loop_rate.sleep();
  }
  return 0;
}