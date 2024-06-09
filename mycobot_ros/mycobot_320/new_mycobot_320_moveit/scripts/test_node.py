import rospy
from std_msgs.msg import String
import random

def barcode_publisher():
    # ROS 노드 초기화
    rospy.init_node('barcode_publisher_node', anonymous=True)

    # 바코드를 발행할 토픽 생성
    barcode_pub = rospy.Publisher('barcode', String, queue_size=10)

    rate = rospy.Rate(1)  # 발행 속도를 5Hz로 설정

    while not rospy.is_shutdown():
        # 임의의 바코드 생성 (예시)
        barcode_pub.publish("038000265013")
        barcode_pub.publish("IM120417006")

        rate.sleep()

if __name__ == '__main__':
    try:
        barcode_publisher()
    except rospy.ROSInterruptException:
        pass