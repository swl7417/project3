from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
import rospy
import socket
import time
import struct

has_main_been_executed = False
start_signal_received = False


def main():
    step_pub = rospy.Publisher("step_input", Int32MultiArray, queue_size=10)

    global start_signal_received

    while not rospy.is_shutdown():
        if start_signal_received:
            start_signal_received = False  # Reset the flag

            try:
                # Create a TCP/IP socket
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                # Connect the socket to the server's address and port
                server_address = ('172.30.1.32', 8080)
                print("Connecting to server at", server_address)
                client_socket.connect(server_address)


                # Send data to the server
                message = "Run!"
                print("Sending:", message)
                client_socket.sendall(message.encode())

                # Receive response from the server
                recieved_data = client_socket.recv(16)
                moveit_x, moveit_y, angle, id = struct.unpack('!iiii', recieved_data)
                print(f"Received data: MoveIt_X={moveit_x}, MoveIt_Y={moveit_y}, Angle={angle}, ID={id}")

                # Create a Float32MultiArray message and assign values
                step_data = Int32MultiArray(data=[moveit_x, moveit_y, angle, id])

                # Publish the message
                step_pub.publish(step_data)
                recieved_data = 0

            except Exception as e:
                print("Error occurred:", str(e))

def send_start_signal_callback(msg):
    global has_main_been_executed
    global start_signal_received

    if msg.data == 1:
        print("Send start signal to socket")
        if not has_main_been_executed:
            start_signal_received = True
            has_main_been_executed = True
    
    if msg.data == 1 and has_main_been_executed:
        print("Received start signal again. Resetting...")
        has_main_been_executed = False  # 다음 main() 실행을 위해 플래그 리셋
        start_signal_received = True  # start_signal_received도 재설정

if __name__ == '__main__':
    try:
        rospy.init_node('yolo_publisher_node', anonymous=True)
        rospy.Subscriber("start", Int32, send_start_signal_callback)
        main()  # 메인 루프 직접 호출
    except rospy.ROSInterruptException:
        pass