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

using namespace sensor_msgs;
using namespace message_filters;

int detection_ = false;
pcl::PointCloud<pcl::PointXYZ> detection_points_; 

void callback(const ImageConstPtr& image, const PointCloud2ConstPtr& point_cloud)
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
  std::vector<int> detection_labels;
  int nLab = cv::connectedComponentsWithStats(binImg, labelImg, stats, centroids);

 for (int i = 1; i < nLab; ++i) {
      int *param = stats.ptr<int>(i);
      int area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

      if ( area > AREA_MIN && area < AREA_MAX)
      {
        detection_labels.push_back(i);
      }
  }

  std::vector<cv::Point2d> detection_2d_poses;
  for (int i=0;i<detection_labels.size();++i)
  {
    double *param = centroids.ptr<double>(i);
    int x = static_cast<int>(param[0]);
    int y = static_cast<int>(param[1]);
    detection_2d_poses.push_back(cv::Point2d(x,y));
  }

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg(*point_cloud, pcl_cloud);
  for (int i=0;i<detection_2d_poses.size();++i)
  {
    int x = detection_2d_poses[i].x;
    int y = detection_2d_poses[i].y;
    detection_points_.push_back(pcl_cloud[image_width_ * y + x]); 
  }

  std::cout << "detection points size:" << detection_points_.size() << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle n;

    ros::ServiceClient client;

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
    std::cout << "move home pos: " << client.call(srv) << std::endl;

    //detection
    message_filters::Subscriber<Image> image_sub(n, "image_raw", 1);
    message_filters::Subscriber<PointCloud2> point_cloud_sub(n, "depth/ponits", 1);
    TimeSynchronizer<Image, PointCloud2> sync(image_sub, point_cloud_sub, 10);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Rate r(10);
    while(!detection_)
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}

