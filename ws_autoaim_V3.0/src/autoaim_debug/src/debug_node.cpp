#include <autoaim_debug/debug_node.hpp>


namespace autoaim_debug
{
    // 构造函数
    autoaim_debug_node::autoaim_debug_node() : Node("autoaim_debug_node")
    {
        RCLCPP_INFO(this->get_logger(), "可视化节点创建");

        // 预测结果可视化
        target_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
            "/tracker/target", rclcpp::SensorDataQoS(),
            std::bind(&autoaim_debug_node::show_predict_result, this, std::placeholders::_1));

        // 真实结果可视化
        real_sub_ = this->create_subscription<autoaim_interfaces::msg::TrackerInfo>(
            "/tracker/info", rclcpp::SensorDataQoS(),
            std::bind(&autoaim_debug_node::show_real_result, this, std::placeholders::_1));

        // 所有目标装甲板可视化
        target_armors_sub_ = this->create_subscription<autoaim_interfaces::msg::FourArmors>(
            "/ballistic/target_armors", rclcpp::SensorDataQoS(),
            std::bind(&autoaim_debug_node::show_target_armors, this, std::placeholders::_1));

        // 创建一个动态广播器
        armor_absolute_broadcaster=std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 创建目标装甲板结果广播器
        target_armors_Broadcaster=std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 创建击打点发布方
        hitting_point_pub_=this->create_publisher<geometry_msgs::msg::PointStamped>("/ballistic/point",10);

        // 最终目标yawpitch轴角度信息接收节点
        final_angle_sub_ = this->create_subscription<autoaim_interfaces::msg::FinalAngle>(
            "/ballistic/final_angle", rclcpp::SensorDataQoS(),
            std::bind(&autoaim_debug_node::final_angle_callback, this, std::placeholders::_1));

        // 云台状态接收节点
        gimbal_sub_ = this->create_subscription<autoaim_interfaces::msg::Gimbal>(
            "/serial/gimbal", rclcpp::SensorDataQoS(),
            std::bind(&autoaim_debug_node::gimbalallback, this, std::placeholders::_1));
    };

    // 预测结果可视化回调函数
    void autoaim_debug_node::show_predict_result(const autoaim_interfaces::msg::Target::SharedPtr msg)
    {
        // tf广播预测整车中心
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp=this->now();//时间戳
        ts.header.frame_id="odom";
        ts.child_frame_id="propredict_center";
        ts.transform.translation.x=msg->position.x;
        ts.transform.translation.y=msg->position.y;
        ts.transform.translation.z=msg->position.z;
        tf2::Quaternion q;
        q.setRPY(0, 0, msg->yaw);
        ts.transform.rotation = tf2::toMsg(q);
        // 发布tf
        armor_absolute_broadcaster->sendTransform(ts);
    } 

    // 真实结果可视化回调函数
    void autoaim_debug_node::show_real_result(const autoaim_interfaces::msg::TrackerInfo::SharedPtr msg)
    {
        // tf广播真实装甲板位置
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp=this->now();//时间戳
        ts.header.frame_id="odom";
        ts.child_frame_id="real_armor_1";
        ts.transform.translation.x=msg->position.x;
        ts.transform.translation.y=msg->position.y;
        ts.transform.translation.z=msg->position.z;
        tf2::Quaternion q;
        q.setRPY(0, 0.27, msg->yaw);
        ts.transform.rotation = tf2::toMsg(q);
        // 发布tf
        armor_absolute_broadcaster->sendTransform(ts);
        // 发布tf
        armor_absolute_broadcaster->sendTransform(ts);
    } 

    // 所有目标装甲板回调函数
    void autoaim_debug_node::show_target_armors(const autoaim_interfaces::msg::FourArmors::SharedPtr msg)
    {
        // tf广播所有装甲板
        geometry_msgs::msg::TransformStamped ts;
        tf2::Quaternion q;
        ts.header.stamp=this->now();//时间戳
        ts.header.frame_id="odom";

        ts.child_frame_id="armor1";
        ts.transform.translation.x=msg->armor1_x;
        ts.transform.translation.y=msg->armor1_y;
        ts.transform.translation.z=msg->armor1_z;     
        q.setRPY(0, 0.27, msg->armor1_yaw);
        ts.transform.rotation = tf2::toMsg(q);
        target_armors_Broadcaster->sendTransform(ts);
        ts.child_frame_id="armor2";
        ts.transform.translation.x=msg->armor2_x;
        ts.transform.translation.y=msg->armor2_y;
        ts.transform.translation.z=msg->armor2_z;
        q.setRPY(0, 0.27, msg->armor2_yaw);
        ts.transform.rotation = tf2::toMsg(q);
        target_armors_Broadcaster->sendTransform(ts);
        ts.child_frame_id="armor3";
        ts.transform.translation.x=msg->armor3_x;
        ts.transform.translation.y=msg->armor3_y;
        ts.transform.translation.z=msg->armor3_z;
        q.setRPY(0, 0.27, msg->armor3_yaw);
        ts.transform.rotation = tf2::toMsg(q);
        target_armors_Broadcaster->sendTransform(ts);
        ts.child_frame_id="armor4";
        ts.transform.translation.x=msg->armor4_x;
        ts.transform.translation.y=msg->armor4_y;
        ts.transform.translation.z=msg->armor4_z;
        q.setRPY(0, 0.27, msg->armor4_yaw);
        ts.transform.rotation = tf2::toMsg(q);
        target_armors_Broadcaster->sendTransform(ts);

        // 发布击打点
        geometry_msgs::msg::PointStamped ps;
        ps.header.stamp=this->now();
        ps.header.frame_id="odom";//所属坐标系
        ps.point.x=msg->final_point_x;
        ps.point.y=msg->final_point_y;
        ps.point.z=msg->final_point_z;
        // 发布消息
        hitting_point_pub_->publish(ps);
    }

    // 最终目标yawpitch轴角度信息接收回调函数
    void autoaim_debug_node::final_angle_callback(const autoaim_interfaces::msg::FinalAngle::SharedPtr final_angle_msg)
    {
        // 发布云台目标姿态tf变换
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "odom";
        t.child_frame_id = "target_gimbal";
        tf2::Quaternion q;
        // 定轴欧拉角旋转
        q.setRPY(debug_gimbal_state.roll, final_angle_msg->final_pitch, final_angle_msg->final_yaw);
        t.transform.rotation = tf2::toMsg(q);
        armor_absolute_broadcaster->sendTransform(t);
    }

    // 云台状态接收回调函数
    void autoaim_debug_node::gimbalallback(const autoaim_interfaces::msg::Gimbal::SharedPtr msg)
    {
        // 更新云台状态
        debug_gimbal_state.roll = msg->roll;
        debug_gimbal_state.yaw = msg->yaw;
        debug_gimbal_state.pitch = msg->pitch;
    }
}

int main(int argc, char const *argv[])
{
    // 初始化ros2客户端
    rclcpp::init(argc,argv);
    // 调用spin函数
    rclcpp::spin(std::make_shared<autoaim_debug::autoaim_debug_node>());
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
