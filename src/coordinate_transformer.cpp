#include "ros/ros.h"
#include "dobot_coordinate_transformer/CameraCoordinate.h"

bool coordinate_transform(dobot_coordinate_transformer::CameraCoordinate::Request  &req,
                          dobot_coordinate_transformer::CameraCoordinate::Response &res)
{
  res.dobot_x = req.camera_x;
  res.dobot_y = req.camera_y;
  res.dobot_z = req.camera_z;
  res.dobot_t = req.camera_t;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dobot_coordinate_transformer");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("coordinate_transform", coordinate_transform);
  ROS_INFO("Ready to dobot coordinate transform.");
  ros::spin();

  return 0;
}
