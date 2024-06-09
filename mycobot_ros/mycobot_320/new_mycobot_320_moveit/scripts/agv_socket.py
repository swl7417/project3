#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import socket
import time

Avalue_received = False
Bvalue_received = False

# ROS 노드 초기화
rospy.init_node('step_publisher_node', anonymous=True)

def send_value_callback(msg):
    global Avalue_received, Bvalue_received

    if msg.data == "A":
        print("sending A signal to socket")
        if not Avalue_received:
            Avalue_received = True
    
    if msg.data == "B":
        print("sending B signal to socket")
        if not Bvalue_received:
            Bvalue_received = True

# Subscriber 초기화
rospy.Subscriber("done_value", String, send_value_callback)

#소켓 초기화
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('172.30.1.29', 8080)
print("Connecting to server at", server_address)
client_socket.connect(server_address)

def keyboard_listener():
    global Avalue_received, Bvalue_received

    key = input("Press 'A' or 'B': ")
    if key == 'A':
        Avalue_received = True
        print("Avalue_received set to True")
        main()

    elif key == 'B':
        Bvalue_received = True
        print("Bvalue_received set to True")
        main()    

def main():
    global Avalue_received, Bvalue_received

    while not rospy.is_shutdown():
        try:
            if Bvalue_received:
                Bvalue_received = False
                # "B" 메시지 송신
                try:
                    message = "3"
                    print("Sending:", message)
                    client_socket.sendall(message.encode())
                    time.sleep(5)
                    keyboard_listener()
                except Exception as e:
                    print("Error occurred:", str(e))
            if Avalue_received:
                Avalue_received = False
                # "A" 메시지 송신
                try:
                    message = "4"
                    print("Sending:", message)
                    client_socket.sendall(message.encode())
                    time.sleep(5)
                    keyboard_listener()
                except Exception as e:
                    print("Error occurred:", str(e))

        except Exception as e:
            print("Error occurred:", str(e))

if __name__ == '__main__':
    # main()  # 메인 루프 실행
    keyboard_listener()