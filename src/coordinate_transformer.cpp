#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

std::string camera_frame = "camera_rgb_optical_frame";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dobot_coordinate_transformer");
  ros::NodeHandle n;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.2, 0.0, 0.35) );
  tf::Quaternion q;
  q.setRPY(M_PI, 0, -M_PI/2);
  transform.setRotation(q);

  ROS_INFO("Ready to dobot coordinate transform.");
  ros::Rate r(100);
  while(1)
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", camera_frame));
    r.sleep();
  }

  return 0;
}
