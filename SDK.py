import cv2
from math import *
import traceback
import numpy as np








def define_chess(chess_board_x_num,chess_board_y_num,square_size):
    '''
        :param chess_board_x_num: 棋盤格x方向格子數
        :param chess_board_y_num: 棋盤格y方向格子數
        :param square_size: 單位棋盤格長度,cm
        :return: 棋盤格世界座標，投影坐標系
    '''
    #定義棋盤格世界座標
    objp = np.zeros((chess_board_x_num*chess_board_y_num,3), np.float32)
    objp[:,:2] = np.mgrid[0:chess_board_x_num,0:chess_board_y_num].T.reshape(-1,2)*square_size

    #繪製出棋盤格投影座標-長度為3格，Z軸設定為-3即出紙面 為-Z
    axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)*square_size
    return objp,axis


#畫出投影坐標系
def draw(img, corners, imgpts,tvecs):
    '''
        :param img: 圖片
        :param corners: 棋盤格上的每個角點
        :param imgpts: projectPoints求得的投影座標
        :param tvecs: solvepnp求得的位移向量
        :return: img : 畫上投影座標後的圖片
    '''
    corner = tuple(map(int, corners[0].ravel()))
    img = cv2.line(img, corner, tuple(map(int,imgpts[0].ravel())), (0,0,255), 5)#X
    img = cv2.line(img, corner, tuple(map(int,imgpts[1].ravel())), (0,255,0), 5)#Y
    img = cv2.line(img, corner, tuple(map(int,imgpts[2].ravel())), (255,0,0), 5)#Z
    x, y = corner
    cv2.putText(img, f"(X:{round(tvecs[0][0], 2)*10}mm,Y:{round(tvecs[1][0], 2)*10}mm,Z:{round(tvecs[2][0], 2)*10}mm)",
                (x,y-45), cv2.FONT_HERSHEY_SIMPLEX,
                1, (55, 200, 255), 2, cv2.LINE_AA)
    return img


#根據歐拉角計算旋轉矩陣
def myRPY2R_robot(x, y, z):
    '''
        :param x: 手臂歐拉角 rx (弧度)
        :param y: 手臂歐拉角 ry (弧度)
        :param z: 手臂歐拉角 rz (弧度)
        :return: R : 手臂末端相對於手臂基座的旋轉矩陣
    '''
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R

#根據手臂位置姿態計算變換矩陣
def pose_robot(x, y, z, Tx, Ty, Tz):
    '''
        :param x: 手臂歐拉角 rx (弧度)
        :param y: 手臂歐拉角 ry (弧度)
        :param z: 手臂歐拉角 rz (弧度)
        :param Tx: 手臂X座標
        :param Ty: 手臂Y座標
        :param Tz: 手臂Z座標
        :return: RT1 : 手臂末端相對於手臂基座的變換矩陣
    '''
    thetaX = x / 180 * pi #將角度轉弧度
    thetaY = y / 180 * pi
    thetaZ = z / 180 * pi
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)#計算旋轉矩陣
    t = np.array([[Tx], [Ty], [Tz]])#手臂座標 = 末端位移向量
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
    return RT1

#取得相機外參: 標定板座標原點相對於相機原點的變換矩陣
def get_RT_from_chessboard(img_path,chess_board_x_num,chess_board_y_num,K,check,objp,axis,dist,Cam_point,mtx):
    '''
    :param img_path: 讀取圖片
    :param chess_board_x_num: 棋盤格x方向格子數
    :param chess_board_y_num: 棋盤格y方向格子數
    :param K: 相機內參
    :param check: 是否停下查看圖片,0 = 不停下快速讀完,1 = 停下查看投影圖片
    :param objp: 棋盤格世界座標
    :param axis: 投影座標
    :param dist: 畸變系數
    :param Cam_point: 存放每張棋盤個原點solvepnp後得到的位移向量
    :param mtx: 相機內參

    :return: RT 相機外參
    '''

    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (chess_board_x_num, chess_board_y_num), None)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    if ret == True:
        try:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Find the rotation and translation vectors.
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, K, dist)
            #平移向量 = 標定板原點相對於相機座標系的座標值
            # print("旋轉向量 : ", rvecs)
            # print("平移向量 : ", tvecs)
            # print("旋轉角度 : ",np.degrees(rvecs))
            Cam_point.append(tvecs)#獲取每張圖的棋盤格原點
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)

            img = draw(img, corners2, imgpts,tvecs)
            img = cv2.circle(img, (img.shape[1] // 2, img.shape[0] // 2), 2, (255, 0, 0), -1)

        except:
            print("wrong")
            print(traceback.print_exc())

    RT=np.column_stack(((cv2.Rodrigues(rvecs))[0],tvecs))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))
    img = draw(img, corners2, imgpts,tvecs)
    cv2.imshow('img', img)
    #check = 1 停下查看棋盤格投影座標圖片, check = 0 快速完成
    if check:
        cv2.waitKey(0)
    else:
        cv2.waitKey(1)

    return RT
