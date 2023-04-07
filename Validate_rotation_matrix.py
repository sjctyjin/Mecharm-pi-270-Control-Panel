"""
將旋轉矩陣轉回尤拉角做驗證
"""
import cv2
import numpy as np
from math import *


# 定義一個旋轉矩陣
R = np.array([[-3.49814160e-03,  9.99649514e-01,  2.62414274e-02, -4.98332786e+01],
 [-9.99975308e-01, -3.65680852e-03,  6.00087928e-03,  3.13171589e+00],
 [ 6.09473593e-03, -2.62197875e-02,  9.99637623e-01,  2.55245992e+01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#旋轉矩陣 to 尤拉角
# print(R[1,0],R[1,1])
#theda(x)
thedax = atan2(R[2,1],R[2,2])* 180 / np.pi
print(thedax)
#theda(y)
theday = atan2(R[2,0]*-1,sqrt(R[2,0]**2+R[2,2]**2))* 180 / np.pi
print(theday)
#theda(z)
thedaz = atan2(R[1,0],R[0,0])* 180 / np.pi
print(thedaz)
#
# print(atan2(R[1,:2][0],R[1,:2][1])* 180 / np.pi)
# print(atan2(R[1,:2][0],R[1,:2][1])* 180 / np.pi)

#將旋轉向量轉回旋轉矩陣
# print(cv2.Rodrigues(np.array([thedax,theday,thedaz]))[0])

rvec, _ = cv2.Rodrigues(R[:3,:3])#正轉
rvec2, _ = cv2.Rodrigues(np.linalg.inv(R[:3,:3]))#反轉
print("旋转向量：", rvec* 180 / np.pi)
print("旋转向量_反轉：", rvec2* 180 / np.pi)
print("旋转矩陣：", cv2.Rodrigues(rvec)[0])
print("旋转矩陣_反轉：", cv2.Rodrigues(rvec2)[0])
