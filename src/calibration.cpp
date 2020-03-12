#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot_msgs/SetCmdTimeout.h"
#include "dobot_msgs/SetQueuedCmdClear.h"
#include "dobot_msgs/SetQueuedCmdStartExec.h"
#include "dobot_msgs/SetQueuedCmdForceStopExec.h"
#include "dobot_msgs/GetDeviceVersion.h"

#include "dobot_msgs/SetEndEffectorParams.h"
#include "dobot_msgs/SetPTPJointParams.h"
#include "dobot_msgs/SetPTPCoordinateParams.h"
#include "dobot_msgs/SetPTPJumpParams.h"
#include "dobot_msgs/SetPTPCommonParams.h"
#include "dobot_msgs/SetPTPCmd.h"

#include "dobot_msgs/SetCPCmd.h"
#include "dobot_msgs/SetCPParams.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#define B_MAX 100
#define B_MIN 0
#define G_MAX 100
#define G_MIN 0
#define R_MAX 255
#define R_MIN 100

#define AREA_MAX 10000
#define AREA_MIN 100

const int image_width_ = 640;
const int image_height_ = 480;

std::string camera_frame_ = "";
std::string image_topic_ = "";
std::string camera_points_topic_ = "";
std::string point_cloud_topic_ = "";

using namespace sensor_msgs;
using namespace message_filters;

class RedPlateDetector
{
public:
  RedPlateDetector():sync2(image_sub,point_cloud_sub, 10)
  {
    image_sub.subscribe(n, image_topic_, 1);
    point_cloud_sub.subscribe(n, camera_points_topic_, 1);
    sync2.registerCallback(boost::bind(&RedPlateDetector::callback,this, _1, _2));
    detection_ = false;
    enable_ = false;
  }

  void detection(pcl::PointXYZ& p)
  {
    enable_ = true;
    detection_ = false;
    while(!detection_)
    {
      ros::spinOnce();
    }
    p = detection_point_;
    detection_ = false;
    enable_ = false;
  }

  void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& point_cloud)
  {
    if (enable_)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      cv::Mat binImg;

      cv::Scalar s_min = cv::Scalar(B_MIN, G_MIN, R_MIN);
      cv::Scalar s_max = cv::Scalar(B_MAX, G_MAX, R_MAX);
      inRange(cv_ptr->image, s_min, s_max, binImg);

      imshow("input image", cv_ptr->image);
      imshow("bin image", binImg);
      cv::waitKey(0);

      cv::Mat stats;
      cv::Mat centroids;
      cv::Mat labelImg;
      int nLab = cv::connectedComponentsWithStats(binImg, labelImg, stats, centroids);

      int max_area;
      int detection_label;
      for (int i = 1; i < nLab; ++i) {
            int *param = stats.ptr<int>(i);
            int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

            if ( area > max_area)
            {
              detection_label = i;
              max_area = area;
            }
      }

      cv::Point2d detection_2d_pos;
      double *param = centroids.ptr<double>(detection_label);
      int x = static_cast<int>(param[0]);
      int y = static_cast<int>(param[1]);
      detection_2d_pos = cv::Point2d(x,y);

      sensor_msgs::PointCloud2 point_cloud_transformed;
      pcl_ros::transformPointCloud("base_link", transform, *point_cloud, point_cloud_transformed);
      
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(point_cloud_transformed, pcl_cloud);
      x = detection_2d_pos.x;
      y = detection_2d_pos.y;
      detection_point_ = pcl_cloud[image_width_ * y + x]; 
      detection_ = true;
    }
  }

private:
  ros::NodeHandle n;
  message_filters::Subscriber<sensor_msgs::Image> image_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub;
  TimeSynchronizer<Image, PointCloud2> sync2;
  pcl::PointXYZ detection_point_;
  bool detection_;
  bool enable_;
  tf::StampedTransform transform;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    ros::ServiceClient client;
    ros::Publisher point_cloud_pub;
    point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("detection_points", 50);

    // SetCmdTimeout
    client = n.serviceClient<dobot_msgs::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot_msgs::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
  }

    // Clear the command queue
    client = n.serviceClient<dobot_msgs::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot_msgs::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot_msgs::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot_msgs::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot_msgs::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot_msgs::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot_msgs::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot_msgs::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot_msgs::SetPTPJointParams srv; 
        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot_msgs::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot_msgs::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot_msgs::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);

    client = n.serviceClient<dobot_msgs::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot_msgs::SetPTPCmd srv;

    srv.request.ptpMode = 1;
    srv.request.x = 0;
    srv.request.y = -200;
    srv.request.z = 0;
    srv.request.r = 0;
    client.call(srv);

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

    RedPlateDetector rpd;
    pcl::PointCloud<pcl::PointXYZ> detection_points; 

    for(int i=0;i<dobot_coordinate_points.size();++i)
    { 

      int x = dobot_coordinate_points[i].x * 1000;
      int y = dobot_coordinate_points[i].y * 1000;
      int z = dobot_coordinate_points[i].z * 1000;

      std::cout << "move:" << x << " " << y << " " << z << std::endl;

      srv.request.ptpMode = 1;
      srv.request.x = x;
      srv.request.y = y;
      srv.request.z = z;
      srv.request.r = 0;
      client.call(srv);

      pcl::PointXYZ p;
      rpd.detection(p);
      detection_points.push_back(p);
    }

    sensor_msgs::PointCloud2 detection_point_ros;
    pcl::toROSMsg(detection_points, detection_point_ros);
    detection_point_ros.header.frame_id = "base_link";
    point_cloud_pub.publish(detection_point_ros); 

    return 0;
}

