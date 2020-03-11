#include "ros/ros.h"
//#include "dobot_coordinate_transformer/CameraCoordinate.h"
#include "dobot_coordinate_transformer/CameraCoordinate.h"

/*
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
*/

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
