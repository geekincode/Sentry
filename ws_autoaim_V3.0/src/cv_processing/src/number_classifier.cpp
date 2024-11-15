// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "cv_processing/armor.hpp"
#include "cv_processing/number_classifier.hpp"

namespace rm_auto_aim
{
  // 分类器类
NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)
{
  net_ = cv::dnn::readNetFromONNX(model_path);

  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

// 提取数字图像
void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // 图片中灯条的长度
  const int light_length = 12;
  // 透视后的图像大小
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // 数字大小 
  const cv::Size roi_size(20, 28);

  // 每个装甲板进行一次
  for (auto & armor : armors) {
    // 透视变换
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    // 对图像透视变换
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // 截取数字区域
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // 二值化
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;
  }
}

// 分类器
void NumberClassifier::classify(std::vector<Armor> & armors)
{
  // 每个装甲板进行一次
  for (auto & armor : armors) {
    // 数字图像
    cv::Mat image = armor.number_img.clone();

    // 标准化
    image = image / 255.0;

    // 减均值和缩放
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // 输入预处理的图像 
    net_.setInput(blob);
    // 前向传播
    cv::Mat outputs = net_.forward();

    // softmax映射到（0,1）区间
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    // 结果解析
    double confidence; // 返回的最大值
    cv::Point class_id_point; // 可能性最大的目标
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.number = class_names_[label_id];

    // 记录结果和可信度
    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%";
    armor.classfication_result = result_ss.str();
  }

  // 排错
  armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const Armor & armor) {
        // 可信度限制
        if (armor.confidence < threshold) {
          return true;
        }
        // 忽略的装甲板号
        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }
        // 错误匹配的装甲板类型
        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) {
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == ArmorType::SMALL) {
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
}

}  // namespace rm_auto_aim
