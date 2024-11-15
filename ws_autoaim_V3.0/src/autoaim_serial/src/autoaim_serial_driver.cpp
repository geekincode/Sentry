#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <autoaim_serial/autoaim_serial_driver.hpp>
#include <autoaim_serial/autoaim_packect.hpp>
#include "serial/serial.h"

using namespace std::chrono_literals;


namespace autoaim_serial
{
    // 构造函数
    autoaim_serial_node::autoaim_serial_node() : Node("autoaim_serial_node")
    {
        RCLCPP_INFO(this->get_logger(), "串口节点创建");

        if(Serial_COM==0)
        {
            sp.setPort("/dev/ttyUSB0");//选择要开启的串口号
        }
        if(Serial_COM==1)
        {
            sp.setPort("/dev/ttyACM0");//选择要开启的串口号
        }

        sp.setBaudrate(115200);//设置波特率
        serial::Timeout _time =serial::Timeout::simpleTimeout(2000);//超时等待
        sp.setTimeout(_time);

        // TF广播
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建定时器
        timer_=this->create_wall_timer(1ms,std::bind(&autoaim_serial_node::on_timer,this));

        // 串口接收线程
        std::thread(std::bind(&autoaim_serial_node::serial_receive_thread,this)).detach();

        // 云台状态发布者
        gimbal_pub_ = this->create_publisher<autoaim_interfaces::msg::Gimbal>("/serial/gimbal", rclcpp::SensorDataQoS());

        // 最终目标yawpitch轴角度信息接收节点
        final_angle_sub_ = this->create_subscription<autoaim_interfaces::msg::FinalAngle>(
            "/ballistic/final_angle", rclcpp::SensorDataQoS(),
            std::bind(&autoaim_serial_node::final_angle_callback, this, std::placeholders::_1));

        // 默认出弹速度
        packect.speed = default_speed ;
    }

    // tf广播线程
    void autoaim_serial_node::serial_receive_thread(void)
    {
        rclcpp::Rate rate(2.0);//一秒钟执行两次
        while (!sp.isOpen())
        {
            try
            {
                //打开串口
                sp.open();
            }
            catch (serial::IOException& e)
            {
                RCLCPP_FATAL(this->get_logger(),"Unable to open port");
                rate.sleep();
            }
        }

        while(1)
        {
            // 接收的包头
            uint8_t original_receivedata_header[1];
            // 接收的原始数据
            uint8_t original_receivedata[sizeof(receivedata) + 2];

            if(sp.available()!=0)
            {
                sp.read(original_receivedata_header,sizeof(original_receivedata_header));

                if(original_receivedata_header[0]==receive_header)
                {
                    // 接收数据
                    sp.read(original_receivedata,sizeof(original_receivedata));

                    // 合校验
                    if (original_receivedata[sizeof(receivedata)+1] == (original_receivedata[0]>>4) + (original_receivedata[2]>>4))
                    {
                        receivedata[0] = ((int16_t)original_receivedata[0]<<8) + (int16_t)original_receivedata[1];
                        receivedata[1] = ((int16_t)original_receivedata[2]<<8) + (int16_t)original_receivedata[3];
                        receivedata[2] = ((int16_t)original_receivedata[4]<<8) + (int16_t)original_receivedata[5];
                        receivedata[3] = ((int16_t)original_receivedata[6]<<8) + (int16_t)original_receivedata[7];

                        packect.roll = (double)receivedata[0]/10000;
                        packect.pitch = (double)receivedata[1]/10000;
                        packect.yaw = (double)receivedata[2]/10000;
                        packect.speed = (double)receivedata[3]/100;
                        packect.color = original_receivedata[8];

                        RCLCPP_INFO(this->get_logger(),"接收数据%.2f;%.2f,%.2f;%.2f;%d",packect.roll,packect.yaw,packect.pitch,packect.speed,packect.color);
                        serial_timed_out_count = 0;
                    }
                }
            }
        }
    }

    // 定时器
    void autoaim_serial_node::on_timer(void)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
        t.header.frame_id = "odom";
        t.child_frame_id = "gimbal_link";
        tf2::Quaternion q;
        // 定轴欧拉角旋转
        q.setRPY(packect.roll, packect.pitch, packect.yaw);
        t.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(t);

        // 发布云台状态信息
        autoaim_interfaces::msg::Gimbal gimbal_msg;
        gimbal_msg.roll = packect.roll;
        gimbal_msg.yaw = packect.yaw ;
        gimbal_msg.pitch = packect.pitch;
        gimbal_msg.speed = packect.speed;
        gimbal_msg.color = packect.color;
        gimbal_pub_->publish(gimbal_msg);

        serial_timed_out_count++;
    }

    // 最终目标yawpitch轴角度信息接收回调函数
    void autoaim_serial_node::final_angle_callback(const autoaim_interfaces::msg::FinalAngle::SharedPtr final_angle_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Final_Target_yaw:%.2f,Final_Target_pitch:%.2f",final_angle_msg->final_yaw,final_angle_msg->final_pitch);

        // 发送的数据
        senddata[0] = (int16_t)(final_angle_msg->final_yaw * 10000);
        senddata[1] = (int16_t)(final_angle_msg->final_pitch * 10000);
        shoot_flag = final_angle_msg->shoot_flag;

        // 发送的原始数据
        uint8_t original_senddata[sizeof(senddata) + sizeof(shoot_flag) + 2];
        original_senddata[0] = send_header;
        original_senddata[1] = senddata[0] >> 8;
        original_senddata[2] = senddata[0];
        original_senddata[3] = senddata[1] >> 8;
        original_senddata[4] = senddata[1];
        original_senddata[5] = shoot_flag;
        original_senddata[6] = original_senddata[2]+original_senddata[4];

        // 串口发送
        if (sp.isOpen() && serial_timed_out_count<serial_timed_out_liimt)
        {
            sp.write(original_senddata,sizeof(original_senddata));//两个参数，第一个参数是要发送的数据地址，第二个数据是要发送数据的长度
            RCLCPP_INFO(this->get_logger(),"发送数据*1");
        }
    }
}


int main(int argc, char const *argv[])
{
    // 初始化ros2客户端
    rclcpp::init(argc, argv);
    // 调用spin函数
    rclcpp::spin(std::make_shared<autoaim_serial::autoaim_serial_node>());
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
