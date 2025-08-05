#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <string>

const std::string portName = "/dev/ttyUSB1"; // 连接esp32的串口名

void sendData(const std::string& portName, const std::string& command, const int index) //传入index只是为了调试输出
{
    int baudrate = 115200;
    serial::Serial ser;
    try {
        ser.setPort(portName);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << portName);
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized: " << portName);
    } 

    // 发送数据
    ser.write(command);
    ROS_INFO_STREAM("Sent command to servo" << index << ": " << command);

    ser.close();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "magnet_test");
    ros::NodeHandle nh;
    while(ros::ok()){
        //TODO: 手动输入
        ROS_INFO("请输入指令");
        std::string str;
        while (!(std::cin >> str)) {
            std::cin.clear();  // 清除错误标志
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 跳过错误行
        }
        int index = std::stoi(str.substr(0,1));
        sendData(portName,str,index);
    }
    return 0;
}
