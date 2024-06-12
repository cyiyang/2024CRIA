#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
from pill_recognizer.srv import (
    Pill_RecognizationMsg,
    Pill_RecognizationMsgResponse
    )
import cv2
import random
import numpy as np
from tensorflow import keras
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

def imgmsg_to_cv2(img_msg):
    # if img_msg.encoding != "bgr8":
    #     rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually         trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr         natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
        
    image_opencv = np.array(image_opencv)
    b = image_opencv[:, :, 0].copy()
    r = image_opencv[:, :, 2].copy()
    image_opencv[:, :, 0] = r
    image_opencv[:, :, 2] = b
    return image_opencv

def pilltest(path):
    # 创建字典，将数字标签映射到对应的药物类型名称
    flower_dict = {0: '鱼肝油', 1: '单色胶囊', 2: '椭圆药丸', 3: '双色胶囊', 4: '圆形药丸'}
    # 指定图片高度为180x180
    img_height = 180
    img_width = 180
    # 加载模型，compile：布尔值，加载后是否编译模型
    model = keras.models.load_model('/home/EPRobot/catkin_ws/src/pill_recognizer/model/model3.h5', compile=True)
    # 加载图像并调整大小
    data = keras.preprocessing.image.load_img(path, target_size=(img_height, img_width))
    # 将PIL图像转换为numpy数组
    # img = cv2.imread(img)
    #img = cv2.resize(img, (180,180))
    data = keras.preprocessing.image.img_to_array(data)
    # 变为三维矩阵，并按行排列
    data = np.expand_dims(data, axis=0)
    data = np.vstack([data])
    # 将数据导入模型，返回numpy数组，索引其中最大值
    result = np.argmax(model.predict(data))
    # 输出字典序中对应的结果
    print(flower_dict[result])
    return result



class Pill_recognizer:
    def __init__(self):
        rospy.init_node("pill_recognizer")
        self.pill_watch_srv=rospy.Service("/pill_channel",Pill_RecognizationMsg,self.callback_pill_recognizer)
        rospy.wait_for_service("/pill_channel")
        rospy.loginfo('pill recognizer is working')
        rospy.spin()
    
    def Photograph(self):
        pill_photo = rospy.Subscriber("/camera/rgb/image_raw", Image)
        rospy.sleep(1)  # 延时1s, 等待相机自动曝光
        rospy.loginfo("now is waiting for /camera/rgb/image_raw")
        board_image = rospy.wait_for_message(
                "/camera/rgb/image_raw", Image
            )  # 订阅一次照片
        rospy.loginfo("image_raw订阅成功")
        pill_photo.unregister()  # 获取到照片后, 取消临时订阅
        
        # if board_image.empty() :
        #    rospy.loginfo("eeee")
          
        rospy.loginfo("11")
        board_image = imgmsg_to_cv2(board_image)  # 转换为cv的bgr格式
        
        #调整图像大小并裁剪
        board_image = cv2.resize(board_image, (360, 360))
        board_image = board_image[130:250, 60:300]
        # 转换为灰度图
        gray = cv2.cvtColor(board_image, cv2.COLOR_BGR2GRAY)
        # 边缘检测
        edges = cv2.Canny(gray, 50, 150)
        # 轮廓检测
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 遍历每个轮廓
        for contour in contours:
        # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area > 300:
                # 计算轮廓的外接矩形
                x, y, w, h = cv2.boundingRect(contour)
                # 绘制外接矩形
                cv2.rectangle(board_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
                # 截取显示器内的图片并输出
                board_image = board_image[y:y+h, x:x+w]

        image_name = (
                "/home/EPRobot/catkin_ws/src/pill_recognizer/board2_image/board2_%s.jpg"
                    % str(random.randint(10000, 99999))
            )  # 保存pill picture地址
        rospy.loginfo("22")
        cv2.imwrite(image_name, board_image)
        rospy.loginfo("pill picture is saves in the file")
        return board_image,image_name

    def callback_pill_recognizer(self,req):
        if req.pill2see == True :
            _,img_path_name = self.Photograph()
            pill_index = pilltest(img_path_name)
            rospy.loginfo("返回的药丸类型为:%d",pill_index)
            res = Pill_RecognizationMsgResponse(pill_index)
            return res
               


# pilltest('/home/EPRobot/catkin_ws/src/savepic/pic/7.jpg') #图片路径
if __name__ == "__main__":

    PillRecognizer = Pill_recognizer()