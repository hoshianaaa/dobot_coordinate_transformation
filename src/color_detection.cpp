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

//Hue:色相 Saturation:彩度 Value:明度
int H_MAX = 100;
int H_MIN = 0;
int S_MAX = 100;
int S_MIN = 0;
int V_MAX = 255;
int V_MIN = 100;


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

    cv::Mat hsvImg;
    cvtColor(cv_ptr->image, hsvImg, CV_BGR2HSV, 3);

    cv::Mat binImg;

    cv::Scalar s_min = cv::Scalar(H_MIN, S_MIN, V_MIN);
    cv::Scalar s_max = cv::Scalar(H_MAX, S_MAX, V_MAX);
    inRange(hsvImg, s_min, s_max, binImg);

    cv::Mat erodeImg;
    cv::Mat element(10,10,CV_8U, cv::Scalar(1));
    cv::erode(binImg, erodeImg, element, cv::Point(-1,-1),1);

    imshow("input image", cv_ptr->image);
    imshow("bin image", binImg);
    imshow("erode image", erodeImg);
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
  H_MAX = config.h_max;
  H_MIN = config.h_min;
  S_MAX = config.s_max;
  S_MIN = config.s_min;
  V_MAX = config.v_max;
  V_MIN = config.v_min;
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

