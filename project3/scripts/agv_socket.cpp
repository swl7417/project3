#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

bool Avalue_received = false;
bool Bvalue_received = false;

boost::asio::io_service io_service;
boost::asio::ip::tcp::socket socket(io_service);

void send_value_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "A") {
        std::cout << "sending A signal to socket" << std::endl;
        if (!Avalue_received) {
            Avalue_received = true;
        }
    }

    if (msg->data == "B") {
        std::cout << "sending B signal to socket" << std::endl;
        if (!Bvalue_received) {
            Bvalue_received = true;
        }
    }
}

void keyboard_listener() {
    std::cout << "Press 'A' or 'B': ";
    char key;
    std::cin >> key;

    if (key == 'A') {
        Avalue_received = true;
        std::cout << "Avalue_received set to True" << std::endl;
    } else if (key == 'B') {
        Bvalue_received = true;
        std::cout << "Bvalue_received set to True" << std::endl;
    }
}

void main_loop() {
    while (ros::ok()) {
        try {
            if (Bvalue_received) {
                Bvalue_received = false;
                std::string message = "1";
                std::cout << "Sending: " << message << std::endl;
                boost::asio::write(socket, boost::asio::buffer(message));
                ros::Duration(5).sleep();
                keyboard_listener();
            }
            if (Avalue_received) {
                Avalue_received = false;
                std::string message = "2";
                std::cout << "Sending: " << message << std::endl;
                boost::asio::write(socket, boost::asio::buffer(message));
                ros::Duration(5).sleep();
                keyboard_listener();
            }
        } catch (std::exception& e) {
            std::cerr << "Error occurred: " << e.what() << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "step_publisher_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("done_value", 1000, send_value_callback);

    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string("172.30.1.29"), 8080);
    socket.connect(endpoint);

    keyboard_listener();
    main_loop();

    return 0;
}
