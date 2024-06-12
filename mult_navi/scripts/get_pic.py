#! /usr/bin/env python
# coding:UTF-8
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # 将ROS图像消息转换为OpenCV图像格式
    # 在这里可以对图像进行处理或其他操作
    cv2.imshow("ROS Image", cv_image)
    cv2.imwrite("1.jpg", cv_image)
    cv2.waitKey(0)  # 等待按下任意键后关闭图像窗口
    rospy.signal_shutdown("Image received")  # 接收到图像后关闭ROS节点

rospy.init_node("image_subscriber")  # 初始化ROS节点

# 创建一个图像订阅者，订阅指定的图像话题（这里假设图像话题名为/image_topic）
rospy.Subscriber("/rgb/image", Image, image_callback)

rospy.spin()  # 运行ROS节点，等待一次图像消息的到达

