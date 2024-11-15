#ifndef AUTOAIM_SERIAL__AUTOAIM_SERIAL_DRIVER_HPP_
#define AUTOAIM_SERIAL__AUTOAIM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <autoaim_serial/autoaim_packect.hpp>
#include "autoaim_interfaces/msg/gimbal.hpp"
#include "autoaim_interfaces/msg/final_angle.hpp"
#include "serial/serial.h"


namespace autoaim_serial
{
class autoaim_serial_node: public rclcpp::Node
{
public:
    // 构造函数
    autoaim_serial_node();

private:
    //创建一个serial对象
    serial::Serial sp; 

    // tf广播
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 云台状态发布者
    rclcpp::Publisher<autoaim_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;

    // 最终目标yawpitch轴角度信息接收节点
    rclcpp::Subscription<autoaim_interfaces::msg::FinalAngle>::SharedPtr final_angle_sub_;

    // 最终目标yawpitch轴角度信息接收回调函数
    void final_angle_callback(const autoaim_interfaces::msg::FinalAngle::SharedPtr final_angle_msg);

    // 串口接收线程
    void serial_receive_thread(void);

    // 定时器
    void on_timer(void);

    // 串口接收数据包
    Serial_ReceivePacket packect;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    //发送的内容
    int16_t senddata[2]; 
    uint8_t shoot_flag;
    //接收的内容
    int16_t receivedata[4]; 

    // 发送的包头
    uint8_t send_header = 0xA0;
    // 接收的包头
    uint8_t receive_header = 0xA1;

    // 串口接收超时等待计数器
    uint16_t serial_timed_out_count;

    // test
    uint8_t test;

    // param
    // 通信延时
    double timestamp_offset_ = 0;
    // 默认出弹速度
    double default_speed = 30.0;
    // 串口接收超时等待(ms)
    uint16_t serial_timed_out_liimt = 1000;

    // 使用的串口设备(0:CH340 ; 1:ACM0)
    uint8_t Serial_COM = 0;
};

}

#endif 