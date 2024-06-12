#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import glob
import random
import sys

import cv2
import cv_bridge
import imutils
import numpy as np
import rospy
from char_recognizer.srv import (
    NeedToSeeMsg,
    NeedToSeeMsgRequest,
    NeedToSeeMsgResponse,
    DestinationMsg,    
    Letter_RecognizationMsg,
    Letter_RecognizationMsgRequest,
    Letter_RecognizationMsgResponse,
)
from sensor_msgs.msg import Image  # 读取ros图片string
from std_msgs.msg import String,Int32

class ActualCharRecognizer:
    def __init__(self, tmpl_path):
        """
        :funciton: recognizerChar实例化
        :param tmpl_path: 字母库在系统中的绝对路径, 末尾不含斜杠
        """
        self.charTmplLib = []

        for char in ("A", "B", "C"):
            path = glob.glob("%s/%s/*.jpg" % (tmpl_path, char))  # 寻找到tmpl_path文件夹下的 模板 文件夹 所有jpg图片
            pathAndChar = list(zip(path, [char] * len(path)))  # 生成每个图片的路径 [路径 字母类型]
            self.charTmplLib.extend(pathAndChar)  # 将pathAndChar中的元素逐个添加到list中 #字符abc

    def vertexFind(self, contours):
        """
        :function: vertexFind寻找顶点(输入近似矩形的轮廓, 返回它的四个顶点, 代替原cv2.approxPolyDP())
        :param contours: 轮廓
        :return: 四个顶点(list, array(4,1,2))
        """
        cnt_reshaped = contours.reshape(contours.shape[0], 2)
        vertexes = [0 for _ in range(4)]
        sumYX = cnt_reshaped.sum(axis=1)
        vertexes[0] = cnt_reshaped[sumYX.argmin()]  # 上左
        vertexes[2] = cnt_reshaped[sumYX.argmax()]  # 下右
        diffYX = np.diff(cnt_reshaped, axis=1)
        vertexes[3] = cnt_reshaped[diffYX.argmin()]  # 上右
        vertexes[1] = cnt_reshaped[diffYX.argmax()]  # 下左
        # 顶点顺序: 上左, 下左, 下右, 上右(逆时针)
        return np.array(vertexes).reshape(4, 1, 2)

    def perspTrans(self, image, approx, size):
        """
        :function: perspectiveTransform透视变换
        :param image: 待变换原图Error: perspTrans() takes 3 positional arguments but 4 were given\n']

        :param approx: 原图中边界坐标(array(4,1,2))
        :param size: 新图尺寸(list, [width, height])
        :return: 变换后的新图(BGR)
        """
        # 读入数据: pts: 原图中边界四点坐标, rect: 按顺序排列四点坐标
        pts = approx.reshape(4, 2)  # array(4,1,2) --> array(4,2)
        rect = np.zeros((4, 2), dtype="float32")
        # 四点坐标排序: rect[上左, 上右, 下右, 下左]
        sumYX = pts.sum(axis=1)
        rect[0] = pts[sumYX.argmin()]  # 上左
        rect[2] = pts[sumYX.argmax()]  # 下右
        diffYX = np.diff(pts, axis=1)
        rect[1] = pts[diffYX.argmin()]  # 上右
        rect[3] = pts[diffYX.argmax()]  # 下左
        # 确定变换后字母框的四点坐标
        dst = np.array(
            [[0, 0], [size[0] - 1, 0], [size[0] - 1, size[1] - 1], [0, size[1] - 1]],
            dtype="float32",
        )
        # 透视变换: ptMatrix: 变换矩阵, warped: 变换后图片
        ptMatrix = cv2.getPerspectiveTransform(rect, dst)
        
        warped = cv2.warpPerspective(image, ptMatrix, (size[0], size[1]))
        cv2.imwrite("/home/EPRobot/catkin_ws/src/warpPerspective_image/"+str(random.randint(0,100))+'.jpg',warped)
        return warped

    def cntsDetect(self, image):
        """
        :function: contoursDetect轮廓检测
        :param image: 待检测原图
        :return: 4个字母框的近似矩形轮廓在原图中的坐标(list, array(4,1,2), len=4)
        """
        # Step1: 边缘检测
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_gray = cv2.GaussianBlur(image_gray, (5, 5), 0)  # [参数!] 高斯模糊: 滤除噪声
        image_edged = cv2.Canny(image_gray, 80, 200)  # [参数!] 边缘检测(两个阈值越大, 保留的轮廓越少)

        # Step2: 轮廓提取(目标: 识别板内四个字母框的轮廓, 思路: 先提取面积最大轮廓, 若长宽比近似于1则认为面积前四即字母框, 否则认为是识别板外轮廓)
        found = 0
        image_forCnt = image_edged.copy()  # 用于提取轮廓的edged图
       
        cv2.imwrite("/home/EPRobot/catkin_ws/src/"+str(random.randint(0,100))+'.jpg',image_forCnt)
        self.image_forChar = image.copy()  # 用于抠出字母框作匹配的color图

        rospy.loginfo("go to while ")
        count = 0
        while not found:
            temp, cnts, hier = cv2.findContours(
                image_forCnt, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)  # 取面积最大轮廓
            approx = self.vertexFind( cnts[0])
            aprx = approx.reshape(4, 2)
            widthT = np.sqrt(
                (aprx[0][0] - aprx[3][0]) ** 2 + (aprx[0][1] - aprx[3][1]) ** 2
            )
            widthB = np.sqrt(
                (aprx[1][0] - aprx[2][0]) ** 2 + (aprx[1][1] - aprx[2][1]) ** 2
            )
            heightL = np.sqrt(
                (aprx[0][0] - aprx[1][0]) ** 2 + (aprx[0][1] - aprx[1][1]) ** 2
            )
            heightR = np.sqrt(
                (aprx[2][0] - aprx[3][0]) ** 2 + (aprx[2][1] - aprx[3][1]) ** 2
            )
            width = max(widthT, widthB)
            height = max(heightL, heightR)
            if (
                    width < height
            ):  # 理论上width > height, 但实际上由于四点顺序不同可能有width < height出现, 此时交换二者
                width, height = height, width
            r = width / height

            if r <= 1.2:
                found = 1
            else:
                image_forCnt = self.perspTrans(
                    image_forCnt, approx, [int(630 * r), 630]
                )
                self.image_forChar = self.perspTrans(
                    self.image_forChar, approx, [int(630 * r), 630]
                )  # 确保image_forCnt和image_forChar的内容保持同步(仅色彩不同)
            count+=1
            if(count > 20):
                pass
        rospy.loginfo("while ended")

        aprxes = []
        for i in range(4):
            approxI = self.vertexFind(cnts[i])
            aprxes.append(approxI)
        return aprxes

    def charDetect(self, aprxes):
        """
        :function: charDetect字母检测
        :param aprxes: 4个字母框的近似矩形轮廓在原图中的坐标(list, array(4,1,2), len=4)
        :return: 纠正后的4个字母框图片(list, len=4)
        """
        aprxes_tl = np.array([aprxes[i][0][0] for i in range(4)])
        aprxes_ordered = [0 for _ in range(4)]
        sumYX = aprxes_tl.sum(axis=1)
        aprxes_ordered[0] = aprxes[sumYX.argmin()]  # 上左(1)
        aprxes_ordered[3] = aprxes[sumYX.argmax()]  # 下右(4)
        diffYX = np.diff(aprxes_tl, axis=1)
        aprxes_ordered[1] = aprxes[diffYX.argmin()]  # 上右(2)
        aprxes_ordered[2] = aprxes[diffYX.argmax()]  # 下左(3)
        charImages = []
        for aprx in aprxes_ordered:
            charImage = self.perspTrans(self.image_forChar, aprx, [110, 110])
            charImage = self.cropImg(charImage, 0.1)
            charImages.append(charImage)
        return charImages

    def cropImg(self, image, ratio):
        """
        :funciton: cropImage裁剪图片
        :param image: 欲裁剪原图
        :param ratio: 上下左右裁剪的长度比例
        :return: 裁剪后的图片crop_img
        """
        height, width, _ = image.shape
        x = int(width * ratio)
        y = int(height * ratio)
        w = int(width * (1 - ratio * 2))
        h = int(height * (1 - ratio * 2))
        crop_img = image[y: y + h, x: x + w]
        return crop_img

    def tmplMatch(self, charImage, threshold):
        """
        :function: templateMatch模板匹配
        :param charImage: 待识别字母图片(经过透视变换)
        :param threshold: 识别阈值(一旦与某个模板的匹配度超过该值, 即返回该模板对应字母)
        :return: 识别结果(字母)
        """
        image_gray = cv2.cvtColor(charImage, cv2.COLOR_BGR2GRAY)
        blank_var = np.var(image_gray, axis=None, dtype=None, ddof=0, keepdims=False)
        if blank_var < 500:
            return " "
        for tmpl_char in self.charTmplLib:
            tmpl = cv2.imread(tmpl_char[0])
            tmpl_gray = cv2.cvtColor(tmpl, cv2.COLOR_BGR2GRAY)
            # 放大待识别图片image
            for scale in np.linspace(1.0, 1.3, 30):
                image_resized = imutils.resize(
                    image_gray, width=int(image_gray.shape[1] * scale)
                )
                tmplRes = cv2.matchTemplate(
                    image_resized, tmpl_gray, cv2.TM_CCOEFF_NORMED
                )
                minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(tmplRes)
                if maxVal >= threshold:
                    return tmpl_char[1]
            # 缩小模板图片tmpl
            for scale in np.linspace(0.7, 1.0, 30):
                tmpl_resized = imutils.resize(
                    tmpl_gray, width=int(tmpl_gray.shape[1] * scale)
                )
                tmplRes = cv2.matchTemplate(
                    image_resized, tmpl_resized, cv2.TM_CCOEFF_NORMED
                )
                minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(tmplRes)
                if maxVal >= threshold:
                    return tmpl_char[1]  # 返回字符字母ABC
        return " "

    def recognize(self, image):
        """
        :function: recognize识别(主函数) 模板匹配
        :param image: 待识别目标板原图
        :return: 识别结果(字母)(list, [char1, char2, char3, char4])
        """
        rospy.loginfo("0")
        charAprxes = self.cntsDetect(image)
        rospy.loginfo("1")
        charImages = self.charDetect(charAprxes)
        rospy.loginfo("2")
        charResult = ["", "", "", ""]
        for i in range(4):
            char_single = self.tmplMatch(charImages[i], 0.7)  # [参数!]
            charResult[i] = char_single

            rospy.loginfo("[%d]"%i)
            
        return charResult


class CharRecognizer:
    def __init__(self, use_fake_img):
        self.use_fake_img = use_fake_img  # 是否使用假图的参数
        rospy.init_node("char_recognizer")
        self.bridge = cv_bridge.CvBridge()  # 创建CV桥

        # self.string_letter_pub=rospy.Publisher("string_letter",String,queue_size=1)
        # self.flag_recognition_mode=rospy.Subscriber("request",Int32,self.callback_recognition)

        self.string_letter_pub=rospy.Service("letter_channel",Letter_RecognizationMsg,self.callback_recognition)  #
        # self.flag_recognition_mode=rospy.Service("request",)


        self.actRecognizer = ActualCharRecognizer(
            "/home/EPRobot/catkin_ws/src/char_recognizer/template"
        )  # 创建字母识别器(实际)

        # 服务请求
        # self.scheduler_client = rospy.ServiceProxy(
        #     "mission", DestinationMsg
        # )  # 创建调度器请求客户  返回句柄

        # self.return_letter_recognition = rospy.ServiceProxy(
        #     "get_letter", Letter_RecognizationMsg
        # )  # 发送服务请求request  创建服务代理

        self.board_reminder_server = rospy.Service(
            "board_reminder_server", NeedToSeeMsg, self.ReminderHandler
        )  # 创建目标板提示服务器 服务端

        rospy.loginfo("[recognizer] recognizer_server正常启动")
        # rospy.wait_for_service("mission")
        # rospy.wait_for_service("/my_img_group/get_letter")  # 等待服务打开
        # rospy.loginfo('连接get_letter服务成功')
        # rospy.loginfo("[recognizer] 连接actuator_server成功")
        rospy.spin()

    def callback_recognition(self,req):
        if req.signal_letter == True:
            #enter the letter mode
            rospy.loginfo("enter the 'letter' recognization mode")
            string_temp = self.RecognizeHandler()  # “能看”以后开始识别
             
            resp=Letter_RecognizationMsgResponse(string_temp)
            rospy.loginfo(
                    "the data type:%s"%type(string_temp))
            # rospy.loginfo("[recognizer]%s"% (resp.))
            return resp


    def ReminderHandler(self, req):
        rospy.loginfo("[recognizer] 收到reminder请求“需要看”: %d" % req.need_to_see)
        rospy.loginfo("[recognizer] 请发送int 1告诉reminder可以看了")

        #if req.need_to_see:
        #    can_see = 0  # 默认“不能看”
        #
        #    rospy.loginfo("[recognizer] 收到actuator回复“能看”")
        #    self.RecognizeHandler()  # “能看”以后开始识别
        self.RecognizeHandler()  # “能看”以后开始识别
        rospy.loginfo("ssss%s"%Letter_RecognizationMsgResponse.letter_string)
        
        return Letter_RecognizationMsg

    def RecognizeHandler(self):
        def Photograph():
            """
            由摄像机或者文件夹 获取opencv可以处理的图像
            """
            if not self.use_fake_img:  # 不使用假图: 向相机订阅照片

                temp_sub = rospy.Subscriber("/camera/rgb/image_raw", Image)  # 创建照片订阅
                rospy.sleep(1)  # 延时1s, 等待相机自动曝光
                rospy.loginfo("now is waiting for /camera/rgb/image_raw")
                board_image = rospy.wait_for_message(
                    "/camera/rgb/image_raw", Image
                )  # 订阅一次照片
                rospy.loginfo("image_raw订阅成功")
                temp_sub.unregister()  # 获取到照片后, 取消临时订阅
                
                
                board_image = self.bridge.imgmsg_to_cv2(board_image, "bgr8")  # 转换为cv的bgr格式
                image_name = (
                        "/home/EPRobot/catkin_ws/src/char_recognizer/board1_image/board1_%s.jpg"
                        % str(random.randint(10000, 99999))
                )  # 保存地址
                cv2.imwrite(image_name, board_image)
                rospy.loginfo("picture is saves in the file ")

            else:  
                # 使用假图进行使用，实际是已经存储好的图片
                board_image = cv2.imread(
                    "/home/EPRobot/catkin_ws/src/char_recognizer/board1_ABC.jpg"
                )
            return board_image

        rospy.loginfo("[recognizer] 开始识别...")
        # Step1. 获取目标板照片
        board_image = Photograph()
        # board_image = imutils.resize(board_image, width=1000)     # 原640*480照片需放大才能识别

        # Step2. 识别目标板字母
        rospy.loginfo("start recognize ABC")
        charResult = self.actRecognizer.recognize(board_image)
        rospy.loginfo("识别到了！！！")
                
        while charResult == [" ", " ", " ", " "]:
            rospy.loginfo("[recognizer] 识别到四个空白框! 等待3s后重新识别...")
            rospy.sleep(3)
            board_image = Photograph()
            charResult = self.actRecognizer.recognize(board_image)
        rospy.loginfo(
            "[recognizer] 识别成功, 结果: 1[%c] 2[%c] 3[%c] 4[%c]"
            % (charResult[0], charResult[1], charResult[2], charResult[3])
        )

        rospy.loginfo("[recognizer] 已向actuator请求“看完了”")

        # Step3. 向scheduler请求新的配送需求(作为client)
        drugTypeToInt = {"A": 0, "B": 1, "C": 2, "N": 4}
        self.string_letter=''#the string which is intended to send
        for i in range(4):
            if charResult[i] != " ":
                # self.scheduler_client.call(
                #     0, 0, drugTypeToInt[charResult[i]], i + 1
                # )  # 请求下一个目标 (i+1): 目的地编号为1~4   #参数排序  车编号-字段需求-获取字母位置012-获取数字位置1234
                
                # request_return_letter_recognition=Letter_RecognizationMsgRequest()
                # request_return_letter_recognition.car_no=0
                # request_return_letter_recognition.request_drug_type=drugTypeToInt[charResult[i]]
                # request_return_letter_recognition.request_deliver_destination=i+1
                # self.return_letter_recognition.call(request_return_letter_recognition)
                
                #self.return_letter_recognition.call(
                #    0, drugTypeToInt[charResult[i]], i + 1
                #)  # 请求下一个目标 (i+1): 目的地编号为1~4   #参数排序  车编号-字段需求-获取字母位置012-获取数字位置1234

                rospy.loginfo(
                    "[recognizer] 已向scheduler请求新配送需求: 药品[%c] --> 窗口[%d]"
                    % (charResult[i], i + 1)
                )
                self.string_letter+=str(charResult[i])
            
            else:# 没有识别到就是空白
                
                #     request_return_letter_recognition=Letter_RecognizationMsgRequest()
                #     request_return_letter_recognition.car_no=0
                #     request_return_letter_recognition.request_drug_type=drugTypeToInt["N"]
                #     request_return_letter_recognition.request_deliver_destination=i+1
                #     self.return_letter_recognition.call(request_return_letter_recognition)
                
                
                #self.return_letter_recognition.call(
                #    0, drugTypeToInt["N"], i + 1
                #)

                rospy.loginfo(
                    "[recognizer] 已向scheduler请求新配送需求: 药品[%c] --> 窗口[%d]"
                    % (charResult[i], i + 1)
                )
                self.string_letter += 'N'

            self.string_letter+=str(i+1)

        if len(self.string_letter) == 8:
            # self.string_letter_pub.publish(self.string_letter)
            
            return self.string_letter
            

if __name__ == "__main__":
    # 读入是否使用假图的参数
    myargv = rospy.myargv(argv=sys.argv)
    rospy.loginfo('输入的参数为：%s',sys.argv)
    if "--use_fake_img" in myargv:  # 如果输入的参数中含有这个元素
        use_fake_img = int(myargv.index("--use_fake_img") + 1)  # 获取参数索引
    else:
        use_fake_img = 0
    charRecognizer = CharRecognizer(use_fake_img)
