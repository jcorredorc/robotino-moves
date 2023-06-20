#pragma once

#include "angles/angles.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"

class RobotinoDrive
{
private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;
  ros::Publisher odom_vis_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sub_;

  // Variables
  geometry_msgs::Pose robotino_pose_;

  // Function prototypes

  void updateCommandVelocity(double vel_x, double vel_y, double angular);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg_odom);
  void bumperMsgCallBack(const std_msgs::Bool::ConstPtr& bumper);
  void odomVisMarker();
  double getYaw(const geometry_msgs::Pose& pose_msg);
  void rotateAngleIncrement(double angular_speed, double relative_angle, bool clockwise);

public:
  RobotinoDrive(ros::NodeHandle& nodeHandle);
  void rotateRelativeAngle(double angular_speed, double relative_angle, bool clockwise);
  ~RobotinoDrive();
};
