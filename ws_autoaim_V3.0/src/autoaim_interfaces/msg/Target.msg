std_msgs/Header header
int32 tracking # 0无目标.1追踪器正常,2有目标追踪器挂掉
string id
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz
# 测量真实装甲板位置
geometry_msgs/Point real_position
float64 real_yaw
