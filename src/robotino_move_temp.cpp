
#include <stdio.h>
#include <string.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"

enum mode_operation
{
  mode_move,
  mode_rotate,
  mode_rel_orientation,
  mode_abs_orientation,
  mode_go_to_goal,
  mode_none
};

ros::Publisher g_velocity_publisher;
ros::Subscriber g_pose_subscriber;
ros::Subscriber g_bumper_subscriber;
ros::Publisher odom_vis_publ;
geometry_msgs::Pose g_robotino_pose;
// int g_bumper = 0;
std::vector<double> g_pose_goal;
std::string g_mode_operation;
double g_linear_vel;
double g_angular_vel;
double g_yaw;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate1(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
double radians2degrees(double angle_in_radians);
void setDesiredOrientation(double desired_angle_radians);
void poseCallback(const nav_msgs::Odometry::ConstPtr& msg_odom);
void bumperCallback(const std_msgs::Bool::ConstPtr& bumper);
void odom_vis_marker_func();
mode_operation str2int(std::string& mode);

void rotate(double angular_vel, double angle_increment, bool clockwise)
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
  {
    vel_msg.angular.z = -abs(angular_vel);
    angle_increment = -abs(angle_increment);
  }
  else
  {
    vel_msg.angular.z = abs(angular_vel);
    angle_increment = abs(angle_increment);
  }

  double angle_init = g_yaw;
  // double t0 = ros::Time::now().toSec();
  ros::Rate loop_rate(1000);
  do
  {
    g_velocity_publisher.publish(vel_msg);
    // double t1 = ros::Time::now().toSec();
    // current_angle = angular_speed * (t1 - t0);
    // current_angle = 0.90 * (t1 - t0);
    // cur<<<<<<<<<<<<<rent_angle = g_yaw;
    ROS_INFO("angle_int: [%f]rad | current_angle_odom: [%f]rad |\n angular_speed: [%f]rad/sec | relative_angle: "
             "[%f]rad",
             angle_init, g_yaw, angular_vel, angle_increment);
    // ROS_INFO("t0= [%f] - t1 = [%f] := [%f]", t0, t1, t1 - t0);
    ros::spinOnce();
    loop_rate.sleep();
  } while (g_yaw <= angle_init + angle_increment);

  vel_msg.angular.z = 0;
  g_velocity_publisher.publish(vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotino_moves");
  ros::NodeHandle nh_;
  g_velocity_publisher = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  g_pose_subscriber = nh_.subscribe("/odom", 10, poseCallback);
  g_bumper_subscriber = nh_.subscribe("/bumper", 1, bumperCallback);
  // publisher Marker!!
  odom_vis_publ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

  if (!(nh_.getParam("/robotino/pose_goal", g_pose_goal) & nh_.getParam("/robotino/mode_operation", g_mode_operation) &
        nh_.getParam("/robotino/linear_vel", g_linear_vel) & nh_.getParam("/robotino/angular_vel", g_angular_vel)))
  {
    ROS_ERROR("Could not read parameters..");
    ros::requestShutdown();
  }
  else
  {
    ROS_INFO("ok load parameters");

    // ROS_INFO("g_pose_goal: %d, mode: %s, linear_vel: %d, angular_vel: %d  ", g_pose_goal[1],
    // g_mode_operation,g_mode_operation, g_linear_vel, g_angular_vel);
    ROS_INFO("g_pose_goal: %f, %f, %f ", g_pose_goal[0], g_pose_goal[1], g_pose_goal[2]);
    // ROS_INFO_STREAM("mode Op:" << g_mode_operation);
    // ROS_INFO(" OJO con strings: [%s]", g_mode_operation.c_str());
    ROS_INFO("lineal_vel: %f", g_linear_vel);
    ROS_INFO("angular_vel: [%f]", g_angular_vel);
  }

  /** test code here **/

  if (ros::ok())
  {
    switch (str2int(g_mode_operation))
    {
      case mode_move:
        ROS_INFO("Move Mode: vel_x: [%f]; dist:[%f]", g_linear_vel, g_pose_goal[0]);
        move(g_linear_vel, g_pose_goal[0], true);
        break;
      case mode_rotate:
        ROS_INFO("Rotate Mode: angular_vel: [%f], angle: [%f] deg", g_linear_vel, g_pose_goal[2]);
        rotate(g_angular_vel, degrees2radians(g_pose_goal[2]), false);
        break;
      case mode_rel_orientation:
        ROS_INFO("RelOrientation Mode");
        break;
      case mode_abs_orientation:
        ROS_INFO("AbsOrientation Mode");
        setDesiredOrientation(degrees2radians(g_pose_goal[2]));
        break;
      case mode_go_to_goal:
        ROS_INFO("GotoGoal");
        break;
      default:
        ROS_INFO("Invalid Mode");
        break;
    }
  }
  else
    ros::shutdown();

  // ros::spin();
  return 0;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward = true)
{
  // straight Line
  geometry_msgs::Twist vel_msg;
  // bool isForward = false;
  // double speed = 0.1;
  // double distance = 1;

  if (isForward)
    vel_msg.linear.x = abs(speed);
  else
    vel_msg.linear.x = -abs(speed);
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;
  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = 0;
  g_velocity_publisher.publish(vel_msg);
  double t0 = ros::Time::now().toSec();
  double current_distance = 0.0;
  ros::Rate loop_rate(30);
  do
  {
    g_velocity_publisher.publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    current_distance = speed * (t1 - t0);
    ROS_INFO("current_distance: [%f], target_distance: [%f]", current_distance, distance);
    ros::spinOnce();
    loop_rate.sleep();
  } while (current_distance < distance);
  vel_msg.linear.x = 0;
  g_velocity_publisher.publish(vel_msg);
}

void rotate1(double angular_speed, double relative_angle, bool clockwise = false)
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
    vel_msg.angular.z = -abs(angular_speed);
  else
    vel_msg.angular.z = abs(angular_speed);

  double current_angle = 0.0;
  double t0 = ros::Time::now().toSec();
  ros::Rate loop_rate(1000);
  do
  {
    g_velocity_publisher.publish(vel_msg);
    double t1 = ros::Time::now().toSec();
    // current_angle = angular_speed * (t1 - t0);
    // current_angle = 0.90 * (t1 - t0);
    current_angle = g_yaw;
    ROS_INFO("odom_angle: [%f]rad | current_angle: [%f]rad | angular_speed: [%f]rad/sec | relative_angle: [%f]rad",
             g_yaw, current_angle, angular_speed, relative_angle);
    // ROS_INFO("t0= [%f] - t1 = [%f] := [%f]", t0, t1, t1 - t0);
    ros::spinOnce();
    loop_rate.sleep();
  } while (current_angle < relative_angle);

  vel_msg.angular.z = 0;
  g_velocity_publisher.publish(vel_msg);
}

void setDesiredOrientation(double desired_angle_radians)
{
  double relative_angle_radians = desired_angle_radians - g_yaw;
  bool clockwise = ((relative_angle_radians < 0) ? true : false);
  // cout<< desired_angle_radians <<","<<relative_angle_radians<<","<<clockwise<<endl;
  rotate(g_angular_vel, abs(relative_angle_radians), clockwise);
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg_odom)
{
  ROS_INFO_ONCE("%s", msg_odom->header.frame_id.c_str());
  ROS_INFO_ONCE("Twist_z %f", msg_odom->twist.twist.angular.z);
  g_robotino_pose.position.x = msg_odom->pose.pose.position.x;
  ROS_INFO_ONCE("x: %f", msg_odom->pose.pose.position.x);
  g_robotino_pose.position.y = msg_odom->pose.pose.position.y;
  ROS_INFO_ONCE("y: %f", msg_odom->pose.pose.position.y);
  g_robotino_pose.position.z = msg_odom->pose.pose.position.z;
  g_robotino_pose.orientation.z = msg_odom->pose.pose.orientation.z;
  // ROS_INFO_ONCE("theta_z: %f", msg_odom->pose.pose.orientation.z);
  g_robotino_pose.orientation.x = msg_odom->pose.pose.orientation.x;
  g_robotino_pose.orientation.y = msg_odom->pose.pose.orientation.y;
  g_robotino_pose.orientation.w = msg_odom->pose.pose.orientation.w;

  ROS_INFO_ONCE("-------------------");
  tf2::Quaternion q(msg_odom->pose.pose.orientation.x, msg_odom->pose.pose.orientation.y,
                    msg_odom->pose.pose.orientation.z, msg_odom->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, g_yaw);
  // ROS_INFO("roll: [%f]", roll);
  // ROS_INFO("pitch: [%f]", pitch);
  ROS_INFO_ONCE("yaw: [%f]rad ; [%f]deg; acos(w)=[%f]deg", g_yaw, radians2degrees(g_yaw),
                radians2degrees(2 * acos(g_robotino_pose.orientation.w)));
  ROS_INFO_ONCE("-------------------");
  odom_vis_marker_func();
}

void bumperCallback(const std_msgs::Bool::ConstPtr& bumper)
{
  ROS_INFO("Bumper: %d", bumper->data);
  if (bumper->data)
  {
    // g_bumper += 1;
    move(0, 0, true);
    rotate(0, 0, false);
    ros::shutdown();
  }
}

mode_operation str2int(std::string& mode)
{
  if (mode == "rotate")
    return mode_rotate;
  if (mode == "move")
    return mode_move;
  if (mode == "RelOrientation")
    return mode_rel_orientation;
  if (mode == "AbsOrientation")
    return mode_abs_orientation;
  if (mode == "GotoGoal")
    return mode_go_to_goal;
  else
    return mode_none;
}

double degrees2radians(double angle_in_degrees)
{
  return angle_in_degrees * PI / 180.0;
}

double radians2degrees(double angle_in_radians)
{
  return angle_in_radians * 180.0 / PI;
}

void odom_vis_marker_func()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.y = g_robotino_pose.position.y;
  marker.pose.position.x = g_robotino_pose.position.x;
  marker.pose.position.z = g_robotino_pose.position.z;
  marker.pose.orientation.x = g_robotino_pose.orientation.x;
  marker.pose.orientation.y = g_robotino_pose.orientation.y;
  marker.pose.orientation.z = g_robotino_pose.orientation.z;
  marker.pose.orientation.w = g_robotino_pose.orientation.w;
  marker.scale.x = 1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 0.8;  // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  // only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  odom_vis_publ.publish(marker);
}