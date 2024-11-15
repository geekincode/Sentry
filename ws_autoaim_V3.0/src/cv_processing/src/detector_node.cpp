#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cv_processing/armor.hpp"
#include "cv_processing/detector_node.hpp"


namespace rm_auto_aim
{
  // 构造函数
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options) : Node("armor_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "图像处理节点创建");

  // 图像处理初始化
  detector_ = initDetector();

  // 装甲板信息发布者
  armors_pub_ = this->create_publisher<autoaim_interfaces::msg::Armors>("/cv_processing/armors", rclcpp::SensorDataQoS());

  // 可视化信息发布者
  marker_pub_ =this->create_publisher<visualization_msgs::msg::MarkerArray>("/cv_processing/marker", 10);

  // 相机信息接收节点
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      // 相机中心(camera_info->k[2], camera_info->k[5])
      // 全向相机标定参数
      // cam_center_ = cv::Point2f(1528.024699529231, 1075.992012812182);
      // 全向参数2.0
      cam_center_ = cv::Point2f(1533.126313328456, 1103.545526256766);
      // 哨兵相机标定参数
      // cam_center_ = cv::Point2f(733.7501879050053, 2411.293882439418);
      // 相机信息
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      // pnp初始化
      pnp_solver_ = std::make_unique<PnPSolver>(CCD_camera_k, CCD_camera_d);
      cam_info_sub_.reset();
    });

  // 图像数据接收节点
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

  // 云台状态接收节点
  gimbal_sub_ = this->create_subscription<autoaim_interfaces::msg::Gimbal>(
      "/serial/gimbal", rclcpp::SensorDataQoS(),
      std::bind(&ArmorDetectorNode::gimbalallback, this, std::placeholders::_1));
}

  // 目标颜色接收回调函数
  void ArmorDetectorNode::gimbalallback(const autoaim_interfaces::msg::Gimbal::SharedPtr msg)
  {
      target_color = msg->color;
  }

// 图像数据回调函数
void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
{
  // 检测装甲板
  auto armors = detectArmors(img_msg);

  if (pnp_solver_ != nullptr) {
    // 清除发布的信息
    armors_msg_.header = armor_marker_.header = text_marker_.header = img_msg->header;
    armors_msg_.armors.clear();
    marker_array_.markers.clear();
    armor_marker_.id = 0;
    text_marker_.id = 0;

    // 装甲板信息(类型,数字,位姿)
    autoaim_interfaces::msg::Armor armor_msg;

    // 每个装甲板进行一次
    for (const auto & armor : armors) {
      cv::Mat rvec, tvec;// 旋转向量,平移向量(相机坐标系)
      // pnp解算
      bool success = pnp_solver_->solvePnP(armor, rvec, tvec);

      if (success) {
        // 装甲板类型,数字信息
        armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.type)];
        armor_msg.number = armor.number;

        // xyz坐标
        armor_msg.pose.position.x = tvec.at<double>(0);
        armor_msg.pose.position.y = tvec.at<double>(1);
        armor_msg.pose.position.z = tvec.at<double>(2);
        // 旋转向量转3x3旋转矩阵
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        // 3x3旋转矩阵转四元数
        tf2::Matrix3x3 tf2_rotation_matrix(
          rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
          rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
          rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
          rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
          rotation_matrix.at<double>(2, 2));
        tf2::Quaternion tf2_q;
        tf2_rotation_matrix.getRotation(tf2_q);
        // 旋转角度
        armor_msg.pose.orientation = tf2::toMsg(tf2_q);

        // 装甲板到图像中心的距离
        armor_msg.distance_to_image_center = pnp_solver_->calculateDistanceToCenter(armor.center);

        // 调试信息
        armor_marker_.id++;
        armor_marker_.scale.y = armor.type == ArmorType::SMALL ? 0.135 : 0.23;
        armor_marker_.pose = armor_msg.pose;
        text_marker_.id++;
        text_marker_.pose.position = armor_msg.pose.position;
        text_marker_.pose.position.y -= 0.1;
        text_marker_.text = armor.classfication_result;
        armors_msg_.armors.emplace_back(armor_msg);
        marker_array_.markers.emplace_back(armor_marker_);
        marker_array_.markers.emplace_back(text_marker_);
      } else {
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
      }
    }

    // 发布装甲板信息
    armors_pub_->publish(armors_msg_);

    // 发布可视化信息
    using Marker = visualization_msgs::msg::Marker;
    armor_marker_.action = armors_msg_.armors.empty() ? Marker::DELETE : Marker::ADD;
    marker_array_.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array_);
  }
}

// 图像检测处理初始化
std::unique_ptr<Detector> ArmorDetectorNode::initDetector()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  // 二值化阈值
  int binary_thres = declare_parameter("binary_thres", 160, param_desc);

  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  // 目标装甲板颜色
  auto detect_color = target_color;

  Detector::LightParams l_params = {
    // 灯条长宽比值范围
    .min_ratio = declare_parameter("light.min_ratio", 0.1),
    .max_ratio = declare_parameter("light.max_ratio", 0.4),
    // 灯条倾斜角范围
    .max_angle = declare_parameter("light.max_angle", 40.0)};

  Detector::ArmorParams a_params = {
    // 两灯条长度比最小值
    .min_light_ratio = declare_parameter("armor.min_light_ratio", 0.7),
    // 两灯条间距最大最小值
    .min_small_center_distance = declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = declare_parameter("armor.max_large_center_distance", 5.5),
    // 最大倾斜角
    .max_angle = declare_parameter("armor.max_angle", 35.0)};

  auto detector = std::make_unique<Detector>(binary_thres, detect_color, l_params, a_params);

  // 分类器初始化
  auto pkg_path = ament_index_cpp::get_package_share_directory("cv_processing");
  auto model_path = pkg_path + "/model/mlp.onnx";
  auto label_path = pkg_path + "/model/label.txt";
  double threshold = this->declare_parameter("classifier_threshold", 0.7);
  std::vector<std::string> ignore_classes =
    this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});
  detector->classifier =
    std::make_unique<NumberClassifier>(model_path, label_path, threshold, ignore_classes);

  return detector;
}

// 检测装甲板
std::vector<Armor> ArmorDetectorNode::detectArmors(const sensor_msgs::msg::Image::ConstSharedPtr & img_msg)
{
  // 将ros的img数据格式转换为opencv格式
  auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;

  // 更新参数
  detector_->binary_thres = get_parameter("binary_thres").as_int();
  detector_->detect_color = target_color;
  detector_->classifier->threshold = get_parameter("classifier_threshold").as_double();

  // 检测装甲板
  auto armors = detector_->detect(img);

  // 显示图像
  for (auto & armor : armors) {
      point[0] = armor.left_light.top;
      point[1] = armor.left_light.bottom;
      point[2] = armor.right_light.top;
      point[3] = armor.right_light.bottom;
      point[4] = armor.center;
  }
  cv::circle(img,point[4],10,cv::Scalar(255,0,0),-1);
  cv::line(img,point[0],point[3],cv::Scalar(255,0,0),3);
  cv::line(img,point[1],point[2],cv::Scalar(255,0,0),3);
  // cv::imshow("test",img);
  cv::waitKey(1);

  auto final_time = this->now();
  auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Latency: " << latency << "ms");

  return armors;
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorDetectorNode)
