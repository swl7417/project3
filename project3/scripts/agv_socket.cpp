#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

bool Avalue_received = false;
bool Bvalue_received = false;

using namespace std;
using namespace ros;

boost::asio::io_service io_service;
boost::asio::ip::tcp::socket socket(io_service);

void send_value_callback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "A") {
        cout << "sending A signal to socket" << endl;
        if (!Avalue_received) {
            Avalue_received = true;
        }
    }

    if (msg->data == "B") {
        cout << "sending B signal to socket" << endl;
        if (!Bvalue_received) {
            Bvalue_received = true;
        }
    }
}

void keyboard_listener() {
    cout << "Press 'A' or 'B': ";
    char key;
    cin >> key;

    if (key == 'A') {
        Avalue_received = true;
        cout << "Avalue_received set to True" << endl;
    } else if (key == 'B') {
        Bvalue_received = true;
        cout << "Bvalue_received set to True" << endl;
    }
}

void main_loop() {
    while (ros::ok()) {
        try {
            if (Bvalue_received) {
                Bvalue_received = false;
                string message = "1";
                cout << "Sending: " << message << endl;
                boost::asio::write(socket, boost::asio::buffer(message));
                Duration(5).sleep();
                keyboard_listener();
            }
            if (Avalue_received) {
                Avalue_received = false;
                string message = "2";
                cout << "Sending: " << message << endl;
                boost::asio::write(socket, boost::asio::buffer(message));
                Duration(5).sleep();
                keyboard_listener();
            }
        } catch (exception& e) {
            cerr << "Error occurred: " << e.what() << endl;
        }
    }
}

int main(int argc, char** argv) {
    init(argc, argv, "step_publisher_node");
    NodeHandle nh;

    Subscriber sub = nh.subscribe("done_value", 1000, send_value_callback);

    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string("172.30.1.29"), 8080);
    socket.connect(endpoint);

    keyboard_listener();
    main_loop();

    return 0;
}
