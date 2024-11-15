#ifndef CV_PROCESSING__DETECTOR_NODE_HPP_
#define CV_PROCESSING__DETECTOR_NODE_HPP_

// ROS
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "cv_processing/detector.hpp"
#include "cv_processing/number_classifier.hpp"
#include "cv_processing/pnp_solver.hpp"
#include "autoaim_interfaces/msg/armors.hpp"
#include "autoaim_interfaces/msg/gimbal.hpp"

namespace rm_auto_aim
{

class ArmorDetectorNode : public rclcpp::Node
{
public:
  ArmorDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

  std::unique_ptr<Detector> initDetector();
  std::vector<Armor> detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg);

  // Armor Detector
  std::unique_ptr<Detector> detector_;

  // Detected armors publisher
  autoaim_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<autoaim_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker armor_marker_;
  visualization_msgs::msg::Marker text_marker_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<PnPSolver> pnp_solver_;

  // Image subscrpition
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;

  // 云台状态接收节点
  rclcpp::Subscription<autoaim_interfaces::msg::Gimbal>::SharedPtr gimbal_sub_;

  // 云台状态接收回调函数
  void gimbalallback(const autoaim_interfaces::msg::Gimbal::SharedPtr msg);

  // 目标颜色(=默认)
  bool target_color = BLUE;

  //test
  cv::Point2f point[5];

  // 全向相机标定参数1.0
  // std::array<double, 9> CCD_camera_k = { 3470.646316488916 ,0 ,1528.024699529231 ,0 ,3469.213214249634 ,1075.992012812182 ,0 ,0 ,1}; // 相机内参矩阵
  // std::vector<double> CCD_camera_d = { -0.0193 , -0.2938 ,0 ,0 ,3.5211}; // 相机畸变矩阵

  // 全向相机标定参数2.0
  // std::array<double, 9> CCD_camera_k = { 3456.469763885999 ,0 ,1533.126313328456 ,0 ,3457.906133479295 ,1103.545526256766 ,0 ,0 ,1}; // 相机内参矩阵
  // std::vector<double> CCD_camera_d = { -0.0193 , -0.2938 ,0 ,0 ,3.5211}; // 相机畸变矩阵

  // 哨兵相机标定参数
  std::array<double, 9> CCD_camera_k = { 2413.924765731947 ,0 ,733.7501879050053 ,0 ,2411.293882439418 ,579.6438921550949 ,0 ,0 ,1}; // 相机内参矩阵
  std::vector<double> CCD_camera_d = { 0.0016 , -0.9509 ,0 ,0 ,9.0766}; // 相机畸变矩阵
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_
