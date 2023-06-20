#include "robotino_moves/robotino_drive.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotino_moves_node");
  ros::NodeHandle nodeHandle("~");
  RobotinoDrive Robotino(nodeHandle);
  // Robotino.rotateRelativeAngle(0.5, 1.5707, false);
  ros::spin();
  return 0;
}