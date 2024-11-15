#ifndef BALLISTIC_CALCULATING__BALLISTIC_CALCULATING_NODE_HPP_
#define BALLISTIC_CALCULATING__BALLISTIC_CALCULATING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "autoaim_interfaces/msg/target.hpp"
#include "autoaim_interfaces/msg/tracker_info.hpp"
#include "autoaim_interfaces/msg/gimbal.hpp"
#include "autoaim_interfaces/msg/four_armors.hpp"
#include "autoaim_interfaces/msg/final_angle.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>


namespace ballistic_calculating
{
class ballistic_calculating_node: public rclcpp::Node
{
public:
    // 构造函数
    ballistic_calculating_node();

private:
    // 预测结果接收
    rclcpp::Subscription<autoaim_interfaces::msg::Target>::SharedPtr predict_sub_;

    // 真实结果接收
    rclcpp::Subscription<autoaim_interfaces::msg::TrackerInfo>::SharedPtr real_sub_;

    // 云台状态接收节点
    rclcpp::Subscription<autoaim_interfaces::msg::Gimbal>::SharedPtr gimbal_sub_;

    // 所有目标装甲板发布者
    rclcpp::Publisher<autoaim_interfaces::msg::FourArmors>::SharedPtr target_armors_pub_;

    // 最终yawpitch角度发布者
    rclcpp::Publisher<autoaim_interfaces::msg::FinalAngle>::SharedPtr final_angle_pub_;

    // 云台状态接收回调函数
    void gimbalallback(const autoaim_interfaces::msg::Gimbal::SharedPtr msg);

    // 预测结果回调函数
    void predict_result_callback(const autoaim_interfaces::msg::Target::SharedPtr msg);

    // 真实结果回调函数
    void real_result_callback(const autoaim_interfaces::msg::TrackerInfo::SharedPtr msg);

    // 17mm弹丸弹道解析
    void ballistic_calculating_17mm(const autoaim_interfaces::msg::Target::SharedPtr msg);

    // 单方向空气阻力模型
    double Unidirectional_air_resistanc_model(double s, double v, double angle);

    // PI
    double PI = 3.1415926535;
    // 弹丸飞行时间
    double time_of_flight ;

    // 云台状态
    struct gimbal_state_t
    {
        double roll;
        double yaw;
        double pitch;
        double speed;

    } __attribute__((packed));
    gimbal_state_t gimbal_state; 

    // 目标装甲板坐标
    struct target_armor_position
    {
        double x;
        double y;
        double z;
        double yaw;

    } __attribute__((packed));
    // 目标装甲板坐标(最多4个)
    target_armor_position target_armors[4];

    // 最终打击目标坐标
    double Final_Target_x; 
    double Final_Target_y; 
    double Final_Target_z; 

    // 最终yaw轴pitch轴目标角度
    double Final_Target_yaw;
    double Final_Target_pitch;

    /* 目标状态 */ 
    double tracker_state; // 追踪器状态
    double armors_num; //目标装甲板号
    double target_yaw; // 目标yaw值
    double current_r; // 当前装甲板半径
    double other_r; // 另一对装甲板半径
    double vx; // xyz线速度
    double vy;
    double vz;
    double px; // xyz坐标
    double py;
    double pz;
    double dz;// 装甲板高度差
    double time_delay; // 时延
    double vyaw; // yaw轴转速

    // param
    // 时间校准常数(s)
    double Time_calibration_constant = 0.01;
    // 枪口前推距离(前为正)
    double muzzle_advance_distance = 0.0; 
    // 枪口相机垂直距离(摄像头在枪口上方距离为正)
    double muzzle_vertical_distance = 0.08; 
    // 弹道系数
    double ballistic_coefficient = 0.038;
    // 重力加速度
    double GRAVITY = 9.78;
};

}

#endif 