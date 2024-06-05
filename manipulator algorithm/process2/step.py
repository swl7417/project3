#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Int32, String
import socket
import time
import struct

process_start = 0
stop_signal_received = False
start_signal_received = False
Avalue_received = False
Bvalue_received = False
step_received = False
barcode_recieved = False

# ROS 노드 초기화
rospy.init_node('step_publisher_node', anonymous=True)

# Publisher 초기화
step_pub = rospy.Publisher("step_input", Float64MultiArray, queue_size=10)
start_pub = rospy.Publisher("process_start", Int32, queue_size=10)
barcode_pub = rospy.Publisher("barcode_data", String, queue_size=10)

def send_start_signal_callback(msg):
    global start_signal_received

    if msg.data == 1:
        if not start_signal_received:
            print("Sending start signal to socket")
            start_signal_received = True

def send_stop_signal_callback(msg):
    global stop_signal_received

    if msg.data == 1:
        if not stop_signal_received:
            print("Sending stop signal to socket")
            stop_signal_received = True


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
rospy.Subscriber("stop", Int32, send_stop_signal_callback)
rospy.Subscriber("start", Int32, send_start_signal_callback)
rospy.Subscriber("done_value", String, send_value_callback)

# 소켓 초기화
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('172.30.1.30', 8080)
print("Connecting to server at", server_address)
client_socket.connect(server_address)


def main():
    global process_start, stop_signal_received, start_signal_received, Avalue_received, Bvalue_received, step_received, barcode_recieved

    while not rospy.is_shutdown():
        try:
            if process_start == 0:
                # 서버에서 데이터 수신
                print("receive data from socket")
                receive_data = client_socket.recv(16)
                process_start = int(receive_data.decode())
                print("Received data:", process_start)
                
                # "Start!" 메시지 송신
                try:
                    message = "Start!"
                    print("Sending: ", message)
                    client_socket.sendall(message.encode())
                except Exception as e:
                    print("Error occurred:", str(e))

                start_pub.publish(1)
                print("Received start signal from the server. Ready to run.")

            if process_start == 1:
                if start_signal_received:
                    start_signal_received = False  # flag 초기화
                    
                    # "Run!" 메시지 송신
                    try:
                        message = "Run!"
                        print("Sending:", message)
                        client_socket.sendall(message.encode())
                        print("Send success")
                    except Exception as e:
                        print("Error occurred:", str(e))

                    # 서버에서 응답 수신
                    received_data1 = client_socket.recv(1024)
                    moveit_x, moveit_y, angle, gripper = struct.unpack('!ffff', received_data1)
                    print(f"Received data: MoveIt_X={moveit_x}, MoveIt_Y={moveit_y}, Angle={angle}, gripper={gripper}")
                    step_received = True


                    if step_received:
                        step_received = False
                        try:
                            message = "send barcode!"
                            print("Sending:", message)
                            client_socket.sendall(message.encode())
                            print("Send success")
                            barcode_recieved = True
                        except Exception as e:
                            print("Error occurred:", str(e))

                        if barcode_recieved:
                            barcode_recieved = False
                            barcode_data = client_socket.recv(1024)
                            barcode = barcode_data.decode()
                            print("barcode: ", barcode)

                            # if not barcode == "None":    
                            barcode_str = str(barcode)
                            print(f"Received barcode data: barcode={barcode}")
                            # 메시지 생성 및 발행
                            step_data = Float64MultiArray(data=[moveit_x, moveit_y, angle, gripper])
                            step_pub.publish(step_data)
                            time.sleep(0.5)
                            barcode_pub.publish(barcode_str)
                            print("step publish done")
                            print("barcode publish done")
                        
                    else:
                        print("step recieved failed")
                
                if stop_signal_received:
                    stop_signal_received = False
                    
                    # "Stop!" 메시지 송신 및 process_start 초기화
                    try:
                        message = "Stop!"
                        print("Sending:", message)
                        client_socket.sendall(message.encode())
                        process_start = 0
                        print("process_start: ", process_start)
                    except Exception as e:
                        print("Error occurred:", str(e))

                if Avalue_received:
                    Avalue_received = False

                    # "A" 메시지 송신
                    try:
                        message = "A"
                        print("Sending:", message)
                        client_socket.sendall(message.encode())
                    except Exception as e:
                        print("Error occurred:", str(e))

                if Bvalue_received:
                    Bvalue_received = False

                    # "B" 메시지 송신
                    try:
                        message = "B"
                        print("Sending:", message)
                        client_socket.sendall(message.encode())
                    except Exception as e:
                        print("Error occurred:", str(e))

        except Exception as e:
            print("Error occurred:", str(e))

if __name__ == '__main__':
    main()  # 메인 루프 실행
