#include <ballistic_calculating/ballistic_calculating_node.hpp>


namespace ballistic_calculating
{
    // 构造函数
    ballistic_calculating_node::ballistic_calculating_node() : Node("ballistic_calculating_node")
    {
        RCLCPP_INFO(this->get_logger(), "弹道解析节点创建");

        // 预测结果接收
        predict_sub_ = this->create_subscription<autoaim_interfaces::msg::Target>(
            "/tracker/target", rclcpp::SensorDataQoS(),
            std::bind(&ballistic_calculating_node::predict_result_callback, this, std::placeholders::_1));

        // 真实结果接收
        real_sub_ = this->create_subscription<autoaim_interfaces::msg::TrackerInfo>(
            "/tracker/info", rclcpp::SensorDataQoS(),
            std::bind(&ballistic_calculating_node::real_result_callback, this, std::placeholders::_1));

        // 云台状态接收节点
        gimbal_sub_ = this->create_subscription<autoaim_interfaces::msg::Gimbal>(
            "/serial/gimbal", rclcpp::SensorDataQoS(),
            std::bind(&ballistic_calculating_node::gimbalallback, this, std::placeholders::_1));

        // 所有目标装甲板发布者
        target_armors_pub_ = this->create_publisher<autoaim_interfaces::msg::FourArmors>("/ballistic/target_armors", rclcpp::SensorDataQoS());

        // 最终yawpitch角度发布者
        final_angle_pub_ = this->create_publisher<autoaim_interfaces::msg::FinalAngle>("/ballistic/final_angle", rclcpp::SensorDataQoS());
    };

    // 云台状态接收回调函数
    void ballistic_calculating_node::gimbalallback(const autoaim_interfaces::msg::Gimbal::SharedPtr msg)
    {
        // 更新云台状态
        gimbal_state.roll = msg->roll;
        gimbal_state.yaw = msg->yaw;
        gimbal_state.pitch = msg->pitch;
        gimbal_state.speed = msg->speed;
    }

    // 预测结果回调函数
    void ballistic_calculating_node::predict_result_callback(const autoaim_interfaces::msg::Target::SharedPtr msg)
    {
        // 17mm弹丸弹道解析
        ballistic_calculating_17mm(msg);
    }

    // 真实结果回调函数
    void ballistic_calculating_node::real_result_callback(const autoaim_interfaces::msg::TrackerInfo::SharedPtr msg)
    {
        (void)msg;
    } 

    // 17mm弹丸弹道解析
    void ballistic_calculating_node::ballistic_calculating_17mm(const autoaim_interfaces::msg::Target::SharedPtr msg)
    {
        // 追踪器状态
        tracker_state = msg->tracking;

        // 追踪器正常
        if (tracker_state==1)
        {
            // xyz坐标
            px = msg->position.x;
            py = msg->position.y;
            pz = msg->position.z;
            // 目标yaw值
            target_yaw = msg->yaw;

            // 当前装甲板半径
            current_r = msg->radius_1;
            // 另一对装甲板半径
            other_r = msg->radius_2;
            // xyz线速度
            vx = msg->velocity.x;
            vy = msg->velocity.y;
            vz = msg->velocity.z;
            // 装甲板高度差
            dz = msg->dz;

            // (延时补偿+飞行时间)
            time_delay =  Time_calibration_constant + time_of_flight;
            // time_delay =  Time_calibration_constant;

            // 线性预测(目标yaw角度补偿)
            target_yaw += msg->v_yaw * time_delay;
        }

        // 无目标
        if (tracker_state==0)
        {
            // 清零
            px=py=pz=target_yaw=0;
            armors_num=current_r=other_r=vx=vy=vz=dz=time_delay=vyaw; 
        }

        /*    2
           3     1  <--装甲板id顺序，逆时针编号
              0
        */
        // 计算四块装甲板位置
        int use_l = 1; // 哪一对装甲板
        int i = 0; // 装甲板数量
        int idx = 0; // 选择的装甲板

        // 目标为平衡步兵
        if (msg->armors_num==2)
        {
            // 得出平步两块装甲板坐标
            for (i = 0; i < 2; i++)
            {
                // 装甲板坐标和角度
                double temp_yaw = target_yaw + i*PI;
                target_armors[i].x = px - current_r*cos(temp_yaw);
                target_armors[i].y = py - current_r*sin(temp_yaw);
                target_armors[i].z = pz;
                target_armors[i].yaw = temp_yaw;
            }
            // 判断距离近的装甲板
            double yaw_diff_min = fabs(gimbal_state.yaw - target_armors[0].yaw);
            double temp_yaw_diff = fabs(gimbal_state.yaw - target_armors[1].yaw);
            if (temp_yaw_diff<yaw_diff_min)
            {
                // 改变目标装甲板
                yaw_diff_min = temp_yaw_diff;
                idx=1;
            }
        }

        // 目标为前哨站
        else if(msg->armors_num==3)
        {
            // 得出前哨三块装甲板坐标
            for (i = 0; i < 3; i++)
            {
                // 装甲板坐标和角度
                double temp_yaw = target_yaw + i*2.0*PI/3.0;
                double r = (current_r+other_r)/2; // 取r1r2平均值
                target_armors[i].x = px - r*cos(temp_yaw);
                target_armors[i].y = py - r*sin(temp_yaw);
                target_armors[i].z = pz;
                target_armors[i].yaw = temp_yaw;
            }

            // 选择追踪的装甲板(计算距离yaw角度最小的)
            double yaw_diff_min = fabs(gimbal_state.yaw - target_armors[0].yaw);
            for (i = 0; i < 3; i++)
            {
                double temp_yaw_diff = fabs(gimbal_state.yaw - target_armors[i].yaw);
                if (temp_yaw_diff < yaw_diff_min)
                {
                    // 改变目标装甲板
                    yaw_diff_min = temp_yaw_diff;
                    idx = i;
                }
            }
        }

        // 目标为其他(四块装甲板)
        else
        {
            // 得出装甲板坐标
            for (i = 0; i < 4; i++)
            {
                // 装甲板坐标和角度
                double temp_yaw = target_yaw + i*PI/2.0;
                double r = use_l ? current_r : other_r; 
                target_armors[i].x = px - r*cos(temp_yaw);
                target_armors[i].y = py - r*sin(temp_yaw);
                target_armors[i].z = use_l ? pz : pz+dz;
                target_armors[i].yaw = temp_yaw;
                use_l = !use_l;
            }

            // 选择追踪的装甲板(计算距离yaw角度最小的)
            double yaw_diff_min = fabs(gimbal_state.yaw - target_armors[0].yaw);
            for (i = 0; i < 4; i++)
            {
                double temp_yaw_diff = fabs(gimbal_state.yaw - target_armors[i].yaw);
                if (temp_yaw_diff < yaw_diff_min)
                {
                    // 改变目标装甲板
                    yaw_diff_min = temp_yaw_diff;
                    idx = i;
                }
            }
            
            // // 选择距离最近的装甲板
            // float dis_diff_min = sqrt(target_armors[0].x*target_armors[0].x + target_armors[0].y*target_armors[0].y);
            // for (i = 0; i < 4; i++)
            // {
            //     float temp_dis_diff = sqrt(target_armors[i].x*target_armors[i].x + target_armors[i].y*target_armors[i].y);
            //     if (temp_dis_diff<dis_diff_min)
            //     {
            //         // 改变目标装甲板
            //         dis_diff_min = temp_dis_diff;
            //         idx = i;
            //     }
            // }
        }

        // 打击目标的坐标
        Final_Target_x = target_armors[idx].x + vx*time_delay;
        Final_Target_y = target_armors[idx].y + vy*time_delay;
        Final_Target_z = target_armors[idx].z + vz*time_delay;

        // 计算yaw和pitch目标角度
        // pitch轴重力补偿
        // 目标距离
        double target_distance = sqrt(Final_Target_x*Final_Target_x + Final_Target_y*Final_Target_y) - muzzle_advance_distance; 
        // 目标垂直距离
        double target_vertical_distance = -(Final_Target_z - muzzle_vertical_distance);

        //test
        double real_distance = pow(msg->real_position.x*msg->real_position.x + msg->real_position.y*msg->real_position.y
                                       + msg->real_position.z*msg->real_position.z ,1.0/3);
        RCLCPP_INFO(this->get_logger(), "真实距离:%.2f",real_distance);
        
        // 设置目标点和临时坐标点
        double z_temp,z_actual,z_diff;
        double angle_pitch;
        z_temp = target_vertical_distance;
        // 循环n次
        for (i = 0; i < 100; i++)
        {
            // pitch角度
            angle_pitch = atan2(z_temp,target_distance);
            // 单方向空气阻力模型
            z_actual = Unidirectional_air_resistanc_model(target_distance,gimbal_state.speed,angle_pitch);
            z_diff = 0.3*(target_vertical_distance - z_actual);
            z_temp = z_temp+z_diff;
            if (fabs(z_diff)<0.00001)
            {
                break;
            }
        }
        // angle_pitch = atan2(z_temp,target_distance);
        // z_actual = Unidirectional_air_resistanc_model(target_distance,gimbal_state.speed,angle_pitch);
        // z_diff = 0.3*(target_vertical_distance - z_actual);
        // z_temp = z_temp+z_diff;
        // angle_pitch = atan2(z_temp,target_distance);

        // pitch轴
        Final_Target_pitch = angle_pitch;
        // yaw轴
        Final_Target_yaw = atan2(Final_Target_y,Final_Target_x);

        // 发布最终目标yawpitch角度
        autoaim_interfaces::msg::FinalAngle final_target_angle_msg;
        final_target_angle_msg.final_yaw = Final_Target_yaw;
        final_target_angle_msg.final_pitch = Final_Target_pitch;
        final_target_angle_msg.shoot_flag = tracker_state;
        final_angle_pub_->publish(final_target_angle_msg);

        // 发布所有装甲板位置
        autoaim_interfaces::msg::FourArmors target_armors_msg;
        target_armors_msg.armor1_x=target_armors[0].x;
        target_armors_msg.armor1_y=target_armors[0].y;
        target_armors_msg.armor1_z=target_armors[0].z;
        target_armors_msg.armor1_yaw=target_armors[0].yaw;
        target_armors_msg.armor2_x=target_armors[1].x;
        target_armors_msg.armor2_y=target_armors[1].y;
        target_armors_msg.armor2_z=target_armors[1].z;
        target_armors_msg.armor2_yaw=target_armors[1].yaw;
        target_armors_msg.armor3_x=target_armors[2].x;
        target_armors_msg.armor3_y=target_armors[2].y;
        target_armors_msg.armor3_z=target_armors[2].z;
        target_armors_msg.armor3_yaw=target_armors[2].yaw;
        target_armors_msg.armor4_x=target_armors[3].x;
        target_armors_msg.armor4_y=target_armors[3].y;
        target_armors_msg.armor4_z=target_armors[3].z;
        target_armors_msg.armor4_yaw=target_armors[3].yaw;
        target_armors_msg.final_point_x=Final_Target_x;
        target_armors_msg.final_point_y=Final_Target_y;
        target_armors_msg.final_point_z=Final_Target_z;
        target_armors_pub_->publish(target_armors_msg);
    }

    // 单方向空气阻力模型(目标距离，出弹速度，pitch角度)
    double ballistic_calculating_node::Unidirectional_air_resistanc_model(double s,double v, double angle)
    {
        double z;
        // 由v和angle计算飞行时间
        time_of_flight = (double)((exp(ballistic_coefficient * s) -1) / (ballistic_coefficient * v * cos(angle)));
        // 给定v与angle时的高度
        z = (double)(v * sin(angle) * time_of_flight - GRAVITY * time_of_flight * time_of_flight /2);
        return z;
    }
}

int main(int argc, char const *argv[])
{
    // 初始化ros2客户端
    rclcpp::init(argc,argv);
    // 调用spin函数
    rclcpp::spin(std::make_shared<ballistic_calculating::ballistic_calculating_node>());
    // 释放资源
    rclcpp::shutdown();
    return 0;
}