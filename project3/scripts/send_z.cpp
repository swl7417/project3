#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <serial/serial.h>
#include <iostream>

bool flag = false;

using namespace std;
using namespace ros;

serial::Serial ser;

Publisher z_pub;
Publisher completion_pub;

void read_and_publish();

void flag_callback(const std_msgs::Int32::ConstPtr& msg) {
    if (msg->data == 1 && !flag) {
        ROS_INFO("Sending start signal to arduino");
        flag = true;
        read_and_publish();
    }
}

void read_and_publish() {
    if (flag) {
        try {
            ser.write("1");
            ROS_INFO("get distance");
            string distance_str = ser.readline(1024, "\n");
            distance_str.erase(remove(distance_str.begin(), distance_str.end(), '\r'), distance_str.end());
            distance_str.erase(remove(distance_str.begin(), distance_str.end(), '\n'), distance_str.end());
            ROS_INFO("distance_str: %s", distance_str.c_str());

            try {
                int distance = stoi(distance_str) - 20;
                float z = distance / 10.0;
                if (z > 14.2) {
                    z = 14.2;
                }
                std_msgs::Float32 z_msg;
                z_msg.data = z;
                z_pub.publish(z_msg);
                ROS_INFO("Received z data: %f", z);
                std_msgs::Int32 completion_msg;
                completion_msg.data = 1;
                completion_pub.publish(completion_msg);
                flag = false;
            } catch (const std::invalid_argument& e) {
                ROS_WARN("Received invalid data: %s", distance_str.c_str());
            }
        } catch (const std::exception& e) {
            ROS_WARN("Error occurred: %s", e.what());
        }
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    init(argc, argv, "z_publisher_node");
    NodeHandle nh;

    // Create a publisher object
    z_pub = nh.advertise<std_msgs::Float32>("z_input", 10);
    completion_pub = nh.advertise<std_msgs::Int32>("z_call_done", 10);

    // Setup the serial connection (adjust '/dev/ttyUSB0' to your serial port)
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open port");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO("Serial Port initialized");
    } else {
        return -1;
    }

    Subscriber sub = nh.subscribe("get_z", 10, flag_callback);

    spin();

    ser.close();
    return 0;
}
