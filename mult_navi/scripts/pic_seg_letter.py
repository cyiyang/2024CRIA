import cv2 as cv
import numpy as np

# 读取输入图像
img = cv.imread('pic_seg/1.png')

# 转换为灰度图像
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

# 二值化处理
thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV | cv.THRESH_OTSU)[1]
cv.imwrite('result11.png', thresh)
# 查找轮廓
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

# 计算每个轮廓的面积，并按面积从大到小进行排序
areas = [cv.contourArea(c) for c in contours]
sorted_areas_idx = sorted(range(len(areas)), key=lambda k: areas[k], reverse=True)

# 选择前n个轮廓，其中n是需要保留的轮廓数
n = 5
top_contours = [i for i in sorted_areas_idx[:n]]

# 遍历每个轮廓，如果当前轮廓不在前n个轮廓中，则绘制外接矩形
for i in range(len(contours)):
    if i not in top_contours:
        x, y, w, h = cv.boundingRect(contours[i])
        cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        
# 保存结果图像
cv.imshow('result.png', img)
cv.imwrite('result.png', img)
cv.waitKey(0)
