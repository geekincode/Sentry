#ifndef AUTOAIM_DEBUG__DEBUG_NODE_HPP_
#define AUTOAIM_DEBUG__DEBUG_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoaim_interfaces/msg/target.hpp"
#include "autoaim_interfaces/msg/tracker_info.hpp"
#include "autoaim_interfaces/msg/four_armors.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "autoaim_interfaces/msg/final_angle.hpp"
#include "autoaim_interfaces/msg/gimbal.hpp"


namespace autoaim_debug
{
class autoaim_debug_node: public rclcpp::Node
{
public:
    // 构造函数
    autoaim_debug_node();

private:
    // 预测结果可视化
    rclcpp::Subscription<autoaim_interfaces::msg::Target>::SharedPtr target_sub_;

    // 真实结果可视化
    rclcpp::Subscription<autoaim_interfaces::msg::TrackerInfo>::SharedPtr real_sub_;

    // 所有目标装甲板可视化
    rclcpp::Subscription<autoaim_interfaces::msg::FourArmors>::SharedPtr target_armors_sub_;

    // 击打点发布方
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr hitting_point_pub_;

    // tf广播
    std::shared_ptr<tf2_ros::TransformBroadcaster> armor_absolute_broadcaster;

    // tf广播
    std::shared_ptr<tf2_ros::TransformBroadcaster> target_armors_Broadcaster;

    // 最终目标yawpitch轴角度信息接收节点
    rclcpp::Subscription<autoaim_interfaces::msg::FinalAngle>::SharedPtr final_angle_sub_;

    // 云台状态接收节点
    rclcpp::Subscription<autoaim_interfaces::msg::Gimbal>::SharedPtr gimbal_sub_;

    // 云台状态接收回调函数
    void gimbalallback(const autoaim_interfaces::msg::Gimbal::SharedPtr msg);

    // 预测结果可视化回调函数
    void show_predict_result(const autoaim_interfaces::msg::Target::SharedPtr msg);

    // 真实结果可视化回调函数
    void show_real_result(const autoaim_interfaces::msg::TrackerInfo::SharedPtr msg);

    // 所有目标装甲板回调函数
    void show_target_armors(const autoaim_interfaces::msg::FourArmors::SharedPtr msg);

    // 最终目标yawpitch轴角度信息接收回调函数
    void final_angle_callback(const autoaim_interfaces::msg::FinalAngle::SharedPtr final_angle_msg);

    // 云台状态
    struct debug_gimbal_state_t
    {
        double roll;
        double yaw;
        double pitch;
        double speed;

    } __attribute__((packed));
    debug_gimbal_state_t debug_gimbal_state; 
};

}

#endif 