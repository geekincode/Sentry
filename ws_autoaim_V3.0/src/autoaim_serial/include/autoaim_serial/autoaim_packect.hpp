#ifndef AUTOAIM_SERIAL__AUTOAIM_PACKECT_HPP_
#define AUTOAIM_SERIAL__AUTOAIM_PACKECT_HPP_


namespace autoaim_serial
{
    // 串口接收数据包
    struct Serial_ReceivePacket
    {
        float roll;
        float pitch;
        float yaw;
        float speed;
        uint8_t color;

    } __attribute__((packed));
}

#endif 
