import cv2 as cv
import numpy as np

# 读取输入图像
img = cv.imread('sxt2.jpg')

# 转换为灰度图像
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# 二值化处理
thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)[1]

# 查找轮廓
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# 计算每个轮廓的面积
areas = [cv.contourArea(c) for c in contours]

# 遍历每个轮廓
for i in range(len(contours)):
    # 如果该轮廓的面积小于一定阈值，则跳过该轮廓
    if areas[i] < 500:
        continue
    # 如果该轮廓在其他轮廓内部，则跳过该轮廓
    is_inside = False
    for j in range(len(contours)):
        if i != j and areas[j] > areas[i]:
            x, y, w, h = cv.boundingRect(contours[j])
            if x <= contours[i][0][0][0] <= x + w and y <= contours[i][0][0][1] <= y + h:
                is_inside = True
                break
    if is_inside:
        continue
    # 计算外接矩形
    x, y, w, h = cv.boundingRect(contours[i])
    # 在原始图像上绘制外接矩形
    cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
    #提取外接矩形中的像素
    roi = thresh[y:y + h, x:x + w]
    # 显示分割结果
    name = './pic_seg/Contours' + str(i) + '.png'
    cv.imwrite(name, roi)
# 保存结果图像
# cv.imshow('result.png', img)
cv.imshow('test.png', img)
cv.waitKey(0)
