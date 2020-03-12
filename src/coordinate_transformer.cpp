#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <stdlib.h>

std::string camera_frame_ = "camera_rgb_optical_frame";
std::string robot_frame_ = "base_link";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dobot_coordinate_transformer");
  ros::NodeHandle n;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.2, 0.0, 0.35));
  tf::Quaternion q;
  q.setRPY(M_PI, 0, -M_PI/2);
  transform.setRotation(q);

  ros::Publisher dobot_coordinate_points_pub = n.advertise<sensor_msgs::PointCloud2>("dobot_coordinate_points", 1000);
  pcl::PointCloud<pcl::PointXYZ> dobot_coordinate_points;

  for (int i=0;i<3;++i)
  {
    for (int j=0;j<3;++j)
    {
      for (int k=0;k<3;++k)
      {
        dobot_coordinate_points.push_back(pcl::PointXYZ(0.2 + 0.02 * i, -0.1 + 0.1 * j, -0.02 + 0.02 * k));
      }
    }
  }

  sensor_msgs::PointCloud2 dobot_coordinate_points_ros;
  pcl::toROSMsg(dobot_coordinate_points, dobot_coordinate_points_ros);
  dobot_coordinate_points_ros.header.frame_id = robot_frame_;


  ros::Publisher dobot_range_points_pub = n.advertise<sensor_msgs::PointCloud2>("dobot_range_points", 10000);
  pcl::PointCloud<pcl::PointXYZ> random_points;
  for (int i=0;i<1000000;i++)
  {
    random_points.push_back(pcl::PointXYZ((double)rand()/RAND_MAX - 0.5, (double)rand()/RAND_MAX - 0.5, (double)rand()/RAND_MAX - 0.5));
  }

  pcl::PointCloud<pcl::PointXYZ> dobot_range_points;
  for (int i=0;i<random_points.size();++i)
  {
    double x = random_points[i].x;
    double y = random_points[i].y;
    double d = std::sqrt(x*x+y*y);
    if (x > 0)
    {
      if (d > 0.15 && d < 0.25)
      {
        dobot_range_points.push_back(random_points[i]);
      }
    }
  }

  std::cout << rand() << std::endl;
  std::cout << RAND_MAX << std::endl;
  std::cout << (double)rand()/RAND_MAX << std::endl;

  sensor_msgs::PointCloud2 dobot_range_points_ros;
  pcl::toROSMsg(dobot_range_points, dobot_range_points_ros);
  dobot_range_points_ros.header.frame_id = robot_frame_;


  ROS_INFO("Ready to dobot coordinate transform.");
  ros::Rate r(100);
  while(1)
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), robot_frame_, camera_frame_));
    dobot_coordinate_points_pub.publish(dobot_coordinate_points_ros);
    dobot_range_points_pub.publish(dobot_range_points_ros);
    r.sleep();
  }
  return 0;
}
