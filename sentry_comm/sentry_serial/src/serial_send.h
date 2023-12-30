#ifndef SERIAL_SEND_H
#define SERIAL_SEND_H
#include <serial/serial.h>  
#include <std_msgs/String.h>  


serial::Serial sentry_ser; //声明串口对象

uint8_t data_len = 13;
std::string cmd_vel_topic;

union Serial_Package
{
    #pragma pack (1) 
    struct
    {
        uint8_t  header = 0xA5;
        float  linear_x;
        float  linear_y;
        float  angular_z;

    };
    #pragma pack ()
    uint8_t Send_Buffer[13];
};

#endif

