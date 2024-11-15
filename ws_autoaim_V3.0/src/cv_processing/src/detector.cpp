// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "cv_processing/detector.hpp"
#include "autoaim_interfaces/msg/debug_armor.hpp"
#include "autoaim_interfaces/msg/debug_light.hpp"

#include "rclcpp/rclcpp.hpp"

namespace rm_auto_aim
{
// 装甲板检测类
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)
{
}

// 检测装甲板
std::vector<Armor> Detector::detect(const cv::Mat & input)
{
  // 图像预处理
  binary_img = preprocessImage(input);
  // 检测灯条
  lights_ = findLights(input, binary_img);
  // 检测装甲板
  armors_ = matchLights(lights_);

  // 装甲板图像分类器
  if (!armors_.empty()) {
    classifier->extractNumbers(input, armors_);
    classifier->classify(armors_);
  }

  return armors_;
}

// 图像预处理
cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)
{
  // 转为灰度图
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  // 图像的二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

// 检测灯条
std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  using std::vector;
  // 用于存储找到的轮廓的容器
  vector<vector<cv::Point>> contours;
  // 用于存储轮廓的层次结构信息
  vector<cv::Vec4i> hierarchy;
  // 轮廓检测
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // 存放灯条形态信息
  vector<Light> lights;
  // 清除灯条调试信息
  this->debug_lights.data.clear();

  // 每个轮廓矩形检测一次
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;

    // 对象的最小面积包围矩形框
    auto r_rect = cv::minAreaRect(contour);
    // 灯条形态解析
    auto light = Light(r_rect);

    if (isLight(light)) {
      // 包围轮廓的矩形的边界信息(包含x , y ,w ,h)
      auto rect = light.boundingRect();
      if (  // 防止判断失败
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) {
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect);
        // 重复判断ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            // 判断当前位置的点在多边形轮廓的相对位置(内部、边缘和外部)
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) {
              // 如果点在轮廓内部
              sum_r += roi.at<cv::Vec3b>(i, j)[0];// cv::Vec3b 红、绿、蓝三种颜色通道的强度值
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // 判断灯条颜色
        light.color = sum_r > sum_b ? RED : BLUE;
        // 容器的尾部添加一个灯条信息
        lights.emplace_back(light);
      }
    }
  }

  return lights;
}

// 判断是不是灯条
bool Detector::isLight(const Light & light)
{
  // 灯条长宽比值范围 (短边 / 长边)
  float ratio = light.width / light.length;
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio;

  // 灯条倾斜角
  bool angle_ok = light.tilt_angle < l.max_angle;

  // 判断标志位
  bool is_light = ratio_ok && angle_ok;

  // 调试信息
  autoaim_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

// 检测装甲板
std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights)
{
  // 存储装甲板信息
  std::vector<Armor> armors;
  // 清除装甲板调试信息
  this->debug_armors.data.clear();

  // 循环判断每个灯条
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      // 筛选敌方灯条
      if (light_1->color != detect_color || light_2->color != detect_color) continue;
      // 检查2个灯条之间是否有另一个灯条
      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      // 判断是否为装甲板
      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        // 容器的尾部添加一个装甲板信息
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// 检查2个灯条之间是否有另一个灯条
bool Detector::containLight(
  const Light & light_1, const Light & light_2, const std::vector<Light> & lights)
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue;

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center)) {
      return true;
    }
  }

  return false;
}

// 判断是否为装甲板
ArmorType Detector::isArmor(const Light & light_1, const Light & light_2)
{
  // 2个灯条的长度比（短边/长边）
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio;

  // 2个灯条中心之间的距离（单位：灯条长度) 
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);

  // 2个灯条中心连接角度 
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < a.max_angle;
  // 判读标志位
  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // 判断装甲类型 (小，大，无效)
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // 更新装甲板调试信息
  autoaim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

}  // namespace rm_auto_aim
