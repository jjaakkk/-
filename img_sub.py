#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 在函数外部创建一个 CvBridge 实例
bridge = CvBridge()

def image_callback(msg, image_pub):
    try:
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
       
    except Exception as e:
        print(f"图像转换失败: {e}")

    # 处理图像: 红色物体检测
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 定义红色的HSV范围（色相环绕情况）
    lower_red1 = np.array([0, 50, 30])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 30])
    upper_red2 = np.array([180, 255, 255])

    # 创建掩膜
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)#按位或操作合并

    # 形态学操作去除噪声
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 处理检测到的轮廓
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 100:  # 过滤小面积噪声
            # 获取矩形框的坐标
            x, y, w, h = cv2.boundingRect(cnt)
            
            # 绘制红色矩形框
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            
            # 计算中心坐标
            center_x = x + w // 2
            center_y = y + h // 2
            
            # 在矩形框上方显示中心坐标
            text = f"{center_x}, {center_y}"
            cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # 发布处理后的图像
    processed_image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    image_pub.publish(processed_image_msg)

def subscribe_image():
    # 初始化 ROS 节点
    rospy.init_node('image_subscriber', anonymous=True)
    
    # 创建发布者，发布处理后的图像
    image_pub = rospy.Publisher('camera/image_processed', Image, queue_size=10)

    # 创建订阅者，订阅话题 'camera/image_raw'
    rospy.Subscriber('camera/image_raw', Image, image_callback, callback_args=image_pub)
    
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        subscribe_image()
    except rospy.ROSInterruptException:
        pass
