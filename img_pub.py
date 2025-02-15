#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():
    # 初始化 ROS 节点
    rospy.init_node('image_publisher', anonymous=True)
    # 创建发布者，发布到 topic 'camera/image_raw'
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    # 初始化 CvBridge
    bridge = CvBridge()
    # 设置摄像头
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    cap.set(cv2.CAP_PROP_EXPOSURE,0.5)
    cap.set(cv2.CAP_PROP_BRIGHTNESS,0.75)
    
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    while not rospy.is_shutdown():
        # 从摄像头捕获一帧图像
        ret, frame = cap.read()
        
        if not ret:
            print("无法获取图像")
            break

        try:
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # 发布图像
            image_pub.publish(ros_image)
            cv2.imshow("Captured Image",frame)
        except Exception as e:
            print(e)

        # 设置发布频率为10Hz
        rospy.sleep(0.005)

    # 释放摄像头
    cap.release()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass
