#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <string>

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

void manba_cb(const std_msgs::Int32::ConstPtr &manba_value)
{
    std::string portName = "/dev/ttyUSB1"; // 连接esp32的串口名
    std::string command = std::to_string(manba_value->data + 1) + std::to_string(0) + "\n"; // manba_value.data是0,1,2
    try {
        sendData(portName, command, manba_value->data + 1);
        std::cout << "Sent command: " << command;
        ros::Duration(1.0).sleep(); // 等待1秒，确保命令发送完成
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "waypoint_tracking");
    ros::NodeHandle nh;
    ros::Subscriber manba_sub = nh.subscribe<std_msgs::Int32>("manba_input", 10, manba_cb);
    ros::spin(); // 进入循环，等待回调函数被调用
    return 0;
}
