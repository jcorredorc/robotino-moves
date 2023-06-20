#include "robotino_moves/robotino_drive.hpp"

RobotinoDrive::RobotinoDrive(ros::NodeHandle& nodeHandle)
{
  cmd_vel_pub_ = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  odom_vis_pub_ = nodeHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
  odom_sub_ = nodeHandle.subscribe("/odom", 10, &RobotinoDrive::odomMsgCallBack, this);
  bumper_sub_ = nodeHandle.subscribe("/bumper", 1, &RobotinoDrive::bumperMsgCallBack, this);
  // RobotinoDrive::rotateAngleIncrement(1, 1.5707, false);
  rotateRelativeAngle(0.5, 1.5707, true);
}

void RobotinoDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr& msg_odom)
{
  ROS_INFO_ONCE("header frame:  %s", msg_odom->header.frame_id.c_str());
  ROS_INFO_ONCE("child frame: %s", msg_odom->child_frame_id.c_str());
  ROS_INFO_ONCE("x: %f", msg_odom->pose.pose.position.x);
  ROS_INFO_ONCE("y: %f", msg_odom->pose.pose.position.y);
  ROS_INFO_ONCE("Twist_z %f", msg_odom->twist.twist.angular.z);
  ROS_INFO_ONCE("theta_z: %f", msg_odom->pose.pose.orientation.z);
  robotino_pose_ = msg_odom->pose.pose;
  odomVisMarker();
  ROS_INFO_ONCE("-------------------");
  // ROS_INFO("yaw: [%f]rad | [%f]deg", getYaw(robotino_pose_), angles::to_degrees(getYaw(robotino_pose_)));
  // ROS_INFO("yaw: [%f] | norm: [%f] | norm+: [%f]", angles::to_degrees(getYaw(robotino_pose_)),
  //  angles::to_degrees(angles::normalize_angle(getYaw(robotino_pose_))),
  //  angles::to_degrees((angles::normalize_angle_positive(getYaw(robotino_pose_)))));
}

void RobotinoDrive::bumperMsgCallBack(const std_msgs::Bool::ConstPtr& bumper)
{
  ROS_INFO_ONCE("Bumper: %d", bumper->data);
  if (bumper->data)
  {
    ROS_INFO("Bumper activated ...");
    ros::shutdown();
  }
}

void RobotinoDrive::odomVisMarker()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = robotino_pose_;
  // marker.pose.position.x = g_robotino_pose.position.x;
  // marker.pose.position.z = g_robotino_pose.position.z;
  // marker.pose.orientation.x = g_robotino_pose.orientation.x;
  // marker.pose.orientation.y = g_robotino_pose.orientation.y;
  // marker.pose.orientation.z = g_robotino_pose.orientation.z;
  // marker.pose.orientation.w = g_robotino_pose.orientation.w;
  /* marker.pose.position.y = 0;
  marker.pose.position.x = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0; */
  marker.scale.x = 0.7;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color.a = 0.8;  // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  odom_vis_pub_.publish(marker);
}

double RobotinoDrive::getYaw(const geometry_msgs::Pose& pose_msg)
{
  tf2::Quaternion q(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  // ROS_INFO_ONCE("yaw: [%f]rad ; [%f]deg; acos(w)=[%f]deg", g_yaw, radians2degrees(g_yaw),
  // radians2degrees(2 * acos(g_robotino_pose.orientation.w)));
  ROS_WARN_COND(isnan(yaw), "Warning: yaw isnan!");
  return yaw;
}

void RobotinoDrive::rotateRelativeAngle(double angular_vel, double relative_angle, bool clockwise = false)
{
  ros::Rate loop_rate(1000);
  double error = 0.001;
  updateCommandVelocity(0, 0, angular_vel);
  do
  {
    // ROS_INFO_ONCE("ok rotateRelativeAngle ..");
    if (!isnan(getYaw(robotino_pose_)))
      error = relative_angle - getYaw(robotino_pose_);
    if (error >= 0.001)
      updateCommandVelocity(0, 0, angular_vel);
    else
      updateCommandVelocity(0, 0, 0);
    ROS_INFO("relative_angle: [%f]deg | odom_angle: [%f]deg | error: [%f]deg ", angles::to_degrees(relative_angle),
             angles::to_degrees(getYaw(robotino_pose_)), angles::to_degrees(error));
    ros::spinOnce();
    loop_rate.sleep();
  } while (ros::ok());
  // ros::shutdown();
}

void RobotinoDrive::updateCommandVelocity(double vel_x, double vel_y, double angular)
{
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = vel_x;
  vel_msg.linear.y = vel_y;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = angular;
  cmd_vel_pub_.publish(vel_msg);
}

/*
Este metodo no tiene buen desempeño.
void RobotinoDrive::rotateAngleIncrement(double angular_vel, double relative_angle, bool clockwise = false)
{
  geometry_msgs::Twist vel_msg;
  // set a random linear velocity in the x-axis
  vel_msg.linear.x = 0;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  // set a random angular velocity in the y-axis
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;

  if (clockwise)
    vel_msg.angular.z = -abs(angular_vel);
  else
    vel_msg.angular.z = abs(angular_vel);

  double current_angle = 0.0;
  double t0 = ros::Time::now().toSec();
  ros::Rate loop_rate(1);
  do
  {
    cmd_vel_pub_.publish(vel_msg);
    // .publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    current_angle = angular_vel * (t1 - t0);
    // current_angle = 0.90 * (t1 - t0);
    // current_angle = getYaw(robotino_pose_);
    ROS_INFO("odom_angle: [%f]deg | current_angle: [%f]deg | relative_angle: [%f]deg",
             angles::to_degrees(getYaw(robotino_pose_)), angles::to_degrees(current_angle),
             angles::to_degrees(relative_angle));
    // ROS_INFO("t0= [%f] - t1 = [%f] := [%f]", t0, t1, t1 - t0);
    ros::spinOnce();
    loop_rate.sleep();
  } while (current_angle < relative_angle);

  vel_msg.angular.z = 0;
  cmd_vel_pub_.publish(vel_msg);
}
 */

RobotinoDrive::~RobotinoDrive()
{
}