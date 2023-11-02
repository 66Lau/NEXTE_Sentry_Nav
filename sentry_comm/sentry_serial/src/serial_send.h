#ifndef SERIAL_SEND_H
#define SERIAL_SEND_H
#include <serial/serial.h>  
#include <std_msgs/String.h>  


serial::Serial sentry_ser; //声明串口对象

uint8_t data_len = 49;
std::string cmd_vel_topic;

union Serial_Package
{
    struct
    {
        uint8_t  header = 0xA5;
        double  linear_x;
        double  linear_y;
        double   linear_z;
        double   angular_x;
        double   angular_y;
        double   angular_z;

    };
    uint8_t Send_Buffer[49];
};

#endif

