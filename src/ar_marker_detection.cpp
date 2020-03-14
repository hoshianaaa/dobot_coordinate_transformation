#include "ros/ros.h"
#include "std_msgs/String.h"

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

#include <dynamic_reconfigure/server.h>
#include <dobot_coordinate_transformer/configConfig.h>

int B_MAX = 100;
int B_MIN = 0;
int G_MAX = 100;
int G_MIN = 0;
int R_MAX = 255;
int R_MIN = 100;

#define AREA_MAX 10000
#define AREA_MIN 100

const int image_width_ = 640;
const int image_height_ = 480;

std::string camera_frame_ = "camera_rgb_optical_frame";
std::string image_topic_ = "/camera/rgb/image_raw";
std::string camera_cloud_topic_ = "/camera/depth/points";
std::string detect_cloud_topic_ = "detect_point";
std::string robot_frame_ = "base_link";

class RedPlateDetector
{
public:
  RedPlateDetector()
  {
    cloud_sub_ = n.subscribe(camera_cloud_topic_, 1, &RedPlateDetector::cloudCallback, this);
    image_sub_ = n.subscribe(image_topic_, 1, &RedPlateDetector::imageCallback, this);
    cloud_pub_ = n.advertise<sensor_msgs::PointCloud2>(detect_cloud_topic_, 1000);
    point_cloud_.clear();
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    std::cout << "cloud callback" << std::endl;
    sensor_msgs::PointCloud2 msg_transformed;
    pcl_ros::transformPointCloud(robot_frame_, transform, *msg, msg_transformed);
    pcl::fromROSMsg(*msg, point_cloud_);
    std::cout << "cloud callback end" << std::endl;
    return;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& image)
  {
    if(point_cloud_.size() == 0)return;
    std::cout << "image call back" << std::endl;
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
    cv::waitKey(10);

    cv::Mat stats;
    cv::Mat centroids;
    cv::Mat labelImg;
    int nLab = cv::connectedComponentsWithStats(binImg, labelImg, stats, centroids);

    if (nLab < 2){return;}
    std::cout << "nlab:" << nLab << std::endl;

    int max_area = 0;
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

    std::cout << "detection label:" << detection_label << std::endl;

    cv::Point2d detection_2d_pos;
    double *param = centroids.ptr<double>(detection_label);
    int x = static_cast<int>(param[0]);
    int y = static_cast<int>(param[1]);
    detection_2d_pos = cv::Point2d(x,y);
    std::cout << "image call back end" << std::endl;
    return;
  }

private:
  ros::NodeHandle n;
  ros::Subscriber cloud_sub_;
  ros::Subscriber image_sub_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;
  tf::StampedTransform transform;
  ros::Publisher cloud_pub_;
};

void callback(dobot_coordinate_transformer::configConfig &config, uint32_t level) {

  std::cout << "config callback" << std::endl;
  B_MAX = config.b_max;
  B_MIN = config.b_min;
  G_MAX = config.g_max;
  G_MIN = config.g_min;
  R_MAX = config.r_max;
  R_MIN = config.r_min;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_detection");
    ros::NodeHandle n;

    dynamic_reconfigure::Server<dobot_coordinate_transformer::configConfig> server;
    dynamic_reconfigure::Server<dobot_coordinate_transformer::configConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    RedPlateDetector rpd;

    while(1)
    {
      ros::spinOnce();
    }

    return 0;
}

