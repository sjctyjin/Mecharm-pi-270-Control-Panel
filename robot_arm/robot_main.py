import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import time
from View.robotmove_ui_5 import *
import pandas as pd
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Coord
import time
from pymycobot import MyCobotSocket
from pymycobot.genre import Angle
import threading
import traceback
import cv2
import depthai as dai
import os
import yaml
import numpy as np
from math import *
import dlib

mc = None
server_ip = ""
s = 0.1
step = 10
take_picture = False
chess_finder = False
trace_object = False

drag_start = None
selection = None
track_window = None
start_flag = True
set_cood_for_obj = False#判斷是否追蹤到物體
X = Y = 0

id = time.strftime('%Y_%m_%d_%H_%M_%S')
Correction_X = 0
Correction_Y = 0
Correction_Z = 0


#物件追蹤
class track_object_sample():

    def onMouseClicked(events, x, y, flags, param):
        global selection, track_window, drag_start

        if events == cv2.EVENT_LBUTTONDOWN:
            drag_start = (x, y)
            track_window = None

        if drag_start:
            xMin = min(x, drag_start[0])
            yMin = min(y, drag_start[1])
            xMax = max(x, drag_start[0])
            yMax = max(y, drag_start[1])
            selection = (xMin, yMin, xMax, yMax)
        if events == cv2.EVENT_LBUTTONUP:
            drag_start = None
            track_window = selection
            selection = None

    def draw(self,frame,tracker,d=80):

        global  start_flag

        X,Y = 0,0

        if start_flag == True:
            cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            cv2.setMouseCallback('image', track_object_sample.onMouseClicked)
            while True:
                if trace_object == False:
                    break
                img_frame = frame.copy()
                img_frame = cv2.cvtColor(img_frame, cv2.COLOR_BGR2RGB)
                if track_window:
                    cv2.rectangle(img_frame, (track_window[0], track_window[1]),
                                  (track_window[2], track_window[3]),
                                  (0, 0, 255), 1)
                    cv2.putText(img_frame,
                                f"(X:{np.sqrt((track_window[0] - track_window[2]) ** 2)}mm,Y:{np.sqrt((track_window[1] - track_window[3]) ** 2)}mm)",
                                (track_window[0], track_window[1] - 45), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (55, 200, 255), 2, cv2.LINE_AA)
                    # cv2.putText(img_frame,
                    #             f"(X:{(track_window[0] + track_window[2]) // 2}mm,Y:{(track_window[1] + track_window[3]) // 2}mm)",
                    #             (track_window[0], track_window[1] - 45), cv2.FONT_HERSHEY_SIMPLEX,
                    #             1, (55, 200, 255), 2, cv2.LINE_AA)
                    # cv2.circle(img_frame, ((track_window[0] - track_window[2]) // 2,
                    #                        (track_window[1] - track_window[3]) // 2), 5, (255, 0, 0), -1)
                elif selection:
                    cv2.rectangle(img_frame, (selection[0], selection[1]), (selection[2], selection[3]),
                                  (0, 0, 255), 1)
                    cv2.putText(img_frame,
                                f"(X:{(selection[0] + selection[2]) // 2}mm,Y:{(selection[1] + selection[3]) // 2}mm)",
                                (selection[0], selection[1] - 45), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (55, 200, 255), 2, cv2.LINE_AA)
                    cv2.circle(img_frame,
                               (int((selection[0] + selection[2]) // 2), int((selection[1] + selection[3]) // 2)), 2,
                               (255, 0, 0), -1)

                cv2.imshow('image', img_frame)
                if cv2.waitKey(5) == 13:
                    break
            #如果開啟相機
            if trace_object:
                start_flag = False
                cv2.destroyAllWindows()
                tracker.start_track(frame,
                                    dlib.rectangle(track_window[0], track_window[1], track_window[2],
                                                   track_window[3]))
        else:
            #即時追蹤
            tracker.update(frame)
            box = tracker.get_position()
            cv2.rectangle(frame, (int(box.left()), int(box.top())), (int(box.right()), int(box.bottom())),
                          (0, 0, 255), 1)

            x = (int((box.left() + box.right()) / 2) - 640) / 1576
            y = (int((box.top() + box.bottom()) / 2) - 540) / 1576

            X = round(d * x,2)
            Y = round(d * y,2)
            cv2.putText(frame,
                        f"(X:{X}mm,Y:{Y}mm)",
                        (int(box.left()), int(box.top()) - 45), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (55, 200, 255), 2, cv2.LINE_AA)
            # bounding box 中心點
            cv2.circle(frame, (
                int((box.left() + box.right()) // 2), int((box.top() + box.bottom()) // 2)),
                       2, (255, 0, 0), -1)
            # 圖像中心點
            cv2.circle(frame, (frame.shape[1] // 2, frame.shape[0] // 2), 5, (0, 255, 0), -1)
        cv2.destroyAllWindows()
        return X,Y
            # cv2.imshow('image', frame)
            # cv2.waitKey(0)
            # if cv2.waitKey(5) == 27:
            #     break

class PyQt_MVC_Main(QMainWindow):
    def __init__(self,parent=None):
        super(QMainWindow,self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowTitle(f'Mecharm Pi 270 Control Panel_{id}')
        self.linkEvent()
        self.show()

    def connect_to_robot(self):
        try:
            mc = MyCobotSocket("192.168.0.10", 9000)
            mc.connect()
            mc.power_on()

            coords = mc.get_coords()
            print(coords)
            self.ui.lineEdit.setText("192.168.0.10")
            self.ui.Z_value.setText(str(coords[2]))
            self.ui.Z_value_slider.setValue(int(coords[2]))
            self.ui.connect_button.setText("連線成功!")
        except:
            self.ui.lineEdit.setText("連線失敗，請重試!")
    #用于根据欧拉角计算旋转矩阵
    def myRPY2R_robot(self,x, y, z):
        Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
        Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
        Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R

    #用于根据位姿计算变换矩阵
    def pose_robot(self,x, y, z, Tx, Ty, Tz):
        thetaX = x / 180 * pi
        thetaY = y / 180 * pi
        thetaZ = z / 180 * pi
        R = PyQt_MVC_Main.myRPY2R_robot(self,thetaX, thetaY, thetaZ)
        t = np.array([[Tx], [Ty], [Tz]])
        # print(R)
        # print("回推",cv2.Rodrigues(R)[0] * 180 / np.pi)
        # print("回推",cv2.RQDecomp3x3(R)[0])
        # print("實際",(x,y,z))
        thedaX = atan2(R[2, 1], R[2, 2]) * 180 / np.pi
        thedaY = atan2(R[2, 0] * -1,
                       sqrt(R[2, 0] ** 2 + R[2, 2] ** 2)) * 180 / np.pi
        thedaZ = atan2(R[1, 0], R[0, 0]) * 180 / np.pi
        print('theda(X) : ', thedaX)
        # theda(y)
        print('theda(Y) : ', thedaY)
        # theda(z)
        print('theda(Z) : ', thedaZ)
        RT1 = np.column_stack([R, t])  # 列合并
        RT1 = np.row_stack((RT1, np.array([0,0,0,1])))
        # print(RT1)
        # RT1=np.linalg.inv(RT1)
        return RT1



    def find_corner(self,gray, img,data,cam2end_data):
        global Correction_X
        global Correction_Y
        global Correction_Z
        global mc
        ax = 0
        ay = 0
        az = 0
        rx = 1
        ry = 1
        rz = 1
        try:
            # coords = mc.get_coords()
            # time.sleep(0.2)
            coords = mc.get_coords()
            if coords != []:
                ax = float(coords[0])
                ay = float(coords[1])
                az = float(coords[2])
                rx = float(coords[3])
                ry = float(coords[4])
                rz = float(coords[5])
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    # return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    ax = 0
                    ay = 0
                    az = 0
                    rx = 1
                    ry = 1
                    rz = 1
                    # return


        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        #相機到末端
        cam2end = np.array(cam2end_data["camera_to_end"], dtype=np.float64)
        # 末端到基座
        end_to_base = PyQt_MVC_Main.pose_robot(self,rx, ry, rz, ax, ay, az)
        square_size = 1.2  # 标定板方格大小（单位：cm）
        objp = np.zeros((8 * 6, 3), np.float32)
        objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2) * square_size
        axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)* square_size
        mtx = np.array(data["camera_matrix"], dtype=np.float64)
        dist = np.array(data["dist_coeff"])
        dist = None
        fx = mtx[0][0]
        cx = mtx[0][2]
        cy = mtx[1][2]
        # print(cx,cy)
        # pass
        ret, corners = cv2.findChessboardCorners(gray, (8, 6), None)
        # print(corners)
        if ret == True:
            try:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
                R, _ = cv2.Rodrigues(rvecs)

                #標定板到相機
                RT_chess_to_cam = np.column_stack((R, tvecs))
                RT_chess_to_cam = np.row_stack((RT_chess_to_cam, np.array([0, 0, 0, 1])))
                print("原點座標 : ", corners2[0][0])
                # project 3D points to image plane
                imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
                corner = tuple(map(int, corners[0].ravel()))
                x, y = corner
                print(tuple(map(int, imgpts[0].ravel())))
                img = cv2.line(img, corner, tuple(map(int, imgpts[0].ravel())), (0, 0, 255), 5)#Y
                img = cv2.line(img, corner, tuple(map(int, imgpts[1].ravel())), (0, 255, 0), 5)#X
                img = cv2.line(img, corner, tuple(map(int, imgpts[2].ravel())), (255, 0, 0), 5)#Z
                # RT_chess_to_base = np.linalg.inv(end_to_base @ cam2end @ RT_chess_to_cam)
                # RT_chess_to_base = end_to_base @ cam2end
                RT_chess_to_base = end_to_base @ cam2end
                print("相機座標至末端座標值 : ", RT_chess_to_base @ [round(tvecs[0][0], 2)*10,round(tvecs[1][0]*10, 2),round(tvecs[2][0]*10, 2),1])
                print("棋盤格座標轉換為機械手臂座標 : ", RT_chess_to_base @ [round(tvecs[0][0], 2)*10,round(tvecs[1][0]*10, 2),round(tvecs[2][0]*10, 2),1])


                cv2.putText(img,
                            f"(X:{round(tvecs[0][0], 2) }mm,Y:{round(tvecs[1][0], 2)}mm,Z:{round(tvecs[2][0], 2)}mm)",
                            (x, y - 45), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (100, 0, 255), 2, cv2.LINE_AA)


                """
                x = (u - cx) / fx
                y = (v - cy) / fy

                """
                u, v = corners2[0][0]
                Correction_X = round(((v-cy)/fx) * float(round(tvecs[2][0]* -10, 2)),2)
                Correction_Y = round(((u-cx)/fx) * float(round(tvecs[2][0]* -10, 2)),2)
                Correction_Z = float(round(tvecs[2][0]* -10, 2))
                print(Correction_X,Correction_Y)

                cv2.circle(img, (img.shape[1] // 2, img.shape[0] // 2), 2, (255, 0, 0), -1)

                # cv2.imshow('img2', img)
                # cv2.waitKey(1)
                # print("img is here")
                # print(b-a)

            except:
                print("wrong")
                print(traceback.format_exc())
        else:
            pass
    def cam(self):
        global take_picture
        global chess_finder
        global trace_object
        global set_cood_for_obj
        global X,Y

        #載入OAK-D 內參矩陣
        with open('calibration_matrix_OAK-D_1280x1080.yaml', 'r') as f:
            data = yaml.load(f.read(), Loader=yaml.FullLoader)
        #載入相機相對於末端的變換矩陣 - 手眼標定結果
        with open('2023_03_20_11_29_41/camera_to_end_20230328_105620.yaml', 'r') as f:
            cam2end_data = yaml.load(f.read(), Loader=yaml.FullLoader)
        # 定義管道
        pipeline = dai.Pipeline()
        # 定義取用RGB相機
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(1280, 1080)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        # XLinkOut 是設備的“輸出”。您要傳輸到主機的任何數據都需要通過 XLink 發送
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        # 將相機輸出串流畫面端命名為rgb
        xout_rgb.setStreamName("rgb")
        # 將上面定義的RGB相機預覽輸入至XLinkOut，以便將幀發送至主機
        cam_rgb.preview.link(xout_rgb.input)
        tracker = dlib.correlation_tracker()

        try:
            with dai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue("rgb")

                while True:
                    in_rgb = q_rgb.get()

                    frame = in_rgb.getCvFrame()
                    #開啟投影座標
                    if chess_finder:
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        PyQt_MVC_Main.find_corner(self,gray, frame,data,cam2end_data)
                        # threading.Thread(target=PyQt_MVC_Main.find_corner, args=(self,gray,frame,data,cam2end_data)).start()



                    # frame = cv2.resize(frame, (1280, 1080))  # 改變尺寸和視窗相同
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 轉換成 RGB
                    height, width, channel = frame.shape  # 讀取尺寸和 channel數量
                    bytesPerline = channel * width  # 設定 bytesPerline ( 轉換使用 )
                    #拍照
                    if take_picture:
                        take_picture = False
                        path = f'{id}/img'
                        img_id = 1
                        if not os.path.isdir(path):
                            os.makedirs(path)

                        for i in os.listdir(path):
                            img_id+=1
                        cv2.imwrite(f'{path}/{img_id}.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    #物件追蹤
                    if trace_object:
                        X, Y = track_object_sample.draw(self, frame, tracker, d=70)
                        print(X,Y)
                        if X**2 < 1000 and Y**2 < 1000:
                            print(X,Y)
                            # PyQt_MVC_Main.move_by_cam(self, X, Y)
                            threading.Thread(target=PyQt_MVC_Main.move_by_cam, args=(self, X, Y)).start()
                            # trace_object = False


                    # 轉換影像為 QImage，讓 PyQt5 可以讀取
                    img = QImage(frame, width, height, bytesPerline, QImage.Format_RGB888)
                    self.ui.cam.setPixmap(QPixmap.fromImage(img))  # QLabel 顯示影像

        except:
            print(traceback.format_exc())
            cap = cv2.VideoCapture(0)  # 設定攝影機鏡頭
            if not cap.isOpened():
                print("Cannot open camera")
                exit()
            while True:
                ret, frame = cap.read()  # 讀取攝影機畫面
                if not ret:
                    print("Cannot receive frame")
                    break
                frame = cv2.resize(frame, (1280, 1080))  # 改變尺寸和視窗相同
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 轉換成 RGB
                if take_picture:
                    take_picture = False
                    path = f'{id}/img'
                    img_id = 1
                    if not os.path.isdir(path):
                        os.makedirs(path)

                    for i in os.listdir(path):
                        img_id += 1
                    cv2.imwrite(f'{path}/{img_id}.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                # 物件追蹤
                if trace_object:
                    X, Y = track_object_sample.draw(self, frame, tracker, d=70)
                    print(X, Y)
                    if X ** 2 < 150 and Y ** 2 < 150:
                        print(X, Y)
                        threading.Thread(target=PyQt_MVC_Main.move_by_cam, args=(self, X, Y)).start()
                        # trace_object = False

                height, width, channel = frame.shape  # 讀取尺寸和 channel數量
                bytesPerline = channel * width  # 設定 bytesPerline ( 轉換使用 )
                # 轉換影像為 QImage，讓 PyQt5 可以讀取
                img = QImage(frame, width, height, bytesPerline, QImage.Format_RGB888)
                self.ui.cam.setPixmap(QPixmap.fromImage(img))  # QLabel 顯示影像



    def linkEvent(self):
        global mc
        global X, Y
        self.ui.lineEdit.setText("連線中，請稍後。。。")
        threading.Thread(target=PyQt_MVC_Main.connect_to_robot,args=(self,)).start()
        threading.Thread(target=PyQt_MVC_Main.cam,args=(self,)).start()
        self.ui.home.clicked.connect(self.home)
        self.ui.X_plus.clicked.connect(self.X_PLUS)
        self.ui.X_miner.clicked.connect(self.X_MINUS)
        self.ui.Y_plus.clicked.connect(self.Y_PLUS)
        self.ui.Y_miner.clicked.connect(self.Y_MINUS)
        self.ui.coords.clicked.connect(self.coords)
        self.ui.Power_Button.clicked.connect(self.on_off_power)
        self.ui.coords_save.clicked.connect(self.record_coords)
        self.ui.clear_table.clicked.connect(self.clear_table)
        self.ui.excute_coord.clicked.connect(self.excute_coord)
        self.ui.connect_button.clicked.connect(self.connect_Robot)
        self.ui.export_coords.clicked.connect(self.export_csv)
        self.ui.chess_axis.clicked.connect(self.chess_axis_print)
        self.ui.trace_obj.clicked.connect(self.trace_obj)
        self.ui.moveTo.clicked.connect(self.movecoord)

        self.ui.Z_value_slider.valueChanged.connect(self.getslidervalue)
        self.ui.J5_degree_slider.valueChanged.connect(self.J5value)
        self.ui.J6_degree_slider.valueChanged.connect(self.J6value)
        self.ui.Z_value_slider.sliderReleased.connect(self.Z_change)
        self.ui.J5_degree_slider.sliderReleased.connect(self.J5value_change)
        self.ui.J6_degree_slider.sliderReleased.connect(self.J6value_change)
        return

   # [151.5, -0.6, 211.0, 97.28, 25.21, 92.88]



    def home(self):
        global mc
        try:
            mc.send_angles([0, 0, 0, 0, 0, 0], 50)
            time.sleep(s)
            coords = mc.get_coords()
            angles = mc.get_angles()
            print(angles)
            print(coords)
            if coords != []:
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))
                self.ui.Z_value.setText(f"{float(coords[2])}")
                self.ui.Z_value_slider.setValue(float(coords[2]))
            return
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return

    def X_PLUS(self):
        global mc
        try:
            print("X加")
            coords = mc.get_coords()
            time.sleep(s)
            mc.send_coords([ coords[0]+step, coords[1], coords[2],  0, 80,0], 50, 1)
            time.sleep(s)
            # mc.send_angle(Angle.J6.value, 0, 50)
            # time.sleep(0.2)
            coords = mc.get_coords()
            print(coords)
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                # self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))

            return
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def Y_PLUS(self):
        global mc
        try:
            print("Y加")
            coords = mc.get_coords()
            time.sleep(s)
            mc.send_coords([coords[0] , coords[1]+ step, coords[2], 0, 80,0], 50, 1)
            time.sleep(s)
            # mc.send_angle(Angle.J6.value, 0, 50)
            # time.sleep(0.2)
            coords = mc.get_coords()
            print(coords)
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                # self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))

            return
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def X_MINUS(self):
        global mc
        try:
            print("X減")
            coords = mc.get_coords()
            time.sleep(s)
            mc.send_coords([coords[0] - step, coords[1], coords[2],  0, 80,0], 50, 1)
            time.sleep(s)
            # mc.send_angle(Angle.J6.value, 0, 50)
            # time.sleep(0.2)
            coords = mc.get_coords()
            print(coords)
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                # self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))

            return
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def Y_MINUS(self):
        global mc
        try:
            print("Y減")
            coords = mc.get_coords()
            time.sleep(s)
            mc.send_coords([coords[0], coords[1] - step, coords[2],0, 80,0], 50, 1)
            time.sleep(s)
            # mc.send_angle(Angle.J6.value, 0, 50)
            # time.sleep(0.2)
            coords = mc.get_coords()
            print(coords)
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                # self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))
            return
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def movecoord(self):
        global Correction_X
        global Correction_Y
        global Correction_Z

        X = float(self.ui.X_pos.text())+Correction_X+6
        Y = float(self.ui.Y_pos.text())+Correction_Y+6
        Z = float(self.ui.Z_pos.text())
        RX = float(self.ui.RX_pos.text())
        RY = float(self.ui.RY_pos.text())
        RZ = float(self.ui.RZ_pos.text())
        # coords = mc.get_coords()
        # time.sleep(s)
        mc.send_coords([X, Y, Z, RX, RY, RZ], 50, 1)
        time.sleep(s)
    def getslidervalue(self):
        self.ui.Z_value.setText(f"{float(self.ui.Z_value_slider.value())}")
    def J5value(self):
        self.ui.J5_degree.setText(f"{float(self.ui.J5_degree_slider.value())}")
    def J6value(self):
        self.ui.J6_degree.setText(f"{float(self.ui.J6_degree_slider.value())}")

    def Z_change(self):
        global mc
        try:
            print(self.ui.Z_value_slider.value())
            coords = mc.get_coords()
            print("1----: ",coords)
            # time.sleep(0.1)
            if coords != []:
                mc.send_coords([coords[0], coords[1], float(self.ui.Z_value_slider.value()),  0, 80,0], 50, 1)
            time.sleep(1)
            coords = mc.get_coords()
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def J5value_change(self):
        global mc
        try:
            print(self.ui.Z_value_slider.value())
            coords = mc.get_coords()
            print("1----: ", coords)
            # time.sleep(0.1)
            if coords != []:
                mc.send_angle(Angle.J5.value, float(self.ui.J5_degree_slider.value()), 50)
            time.sleep(1)
            coords = mc.get_coords()
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def J6value_change(self):
        global mc
        try:
            print(self.ui.Z_value_slider.value())
            coords = mc.get_coords()
            print("1----: ", coords)
            # time.sleep(0.1)
            if coords != []:
                mc.send_angle(Angle.J6.value, float(self.ui.J6_degree_slider.value()), 50)
            time.sleep(1)
            coords = mc.get_coords()
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    #取得座標值
    def coords(self):
        global mc
        try:
            coords = mc.get_coords()
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))

        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    #斷電:示教
    def on_off_power(self):
        global mc
        try:
            if self.ui.Power_Button.text() == "OFF":
                self.ui.Power_Button.setText("ON")
                mc.power_off()
            else:
                self.ui.Power_Button.setText("OFF")
                mc.power_on()
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    #紀錄座標值
    def record_coords(self):
        global take_picture
        global mc
        try:
            coords = mc.get_coords()
            print(coords)
            if coords != []:
                self.ui.Z_value.setText(f"{float(coords[2])}")
                self.ui.Z_value_slider.setValue(float(coords[2]))
                self.ui.X_pos.setText(str(coords[0]))
                self.ui.Y_pos.setText(str(coords[1]))
                self.ui.Z_pos.setText(str(coords[2]))
                self.ui.RX_pos.setText(str(coords[3]))
                self.ui.RY_pos.setText(str(coords[4]))
                self.ui.RZ_pos.setText(str(coords[5]))
                # 假设有一个名为 tableWidget 的 QTableWidget 实例
                table = self.ui.tableWidget

                # 获取当前行数
                current_row = table.rowCount()

                # 插入一行
                table.insertRow(current_row)

                # 在每个单元格中插入数据
                for i, item in enumerate(coords):
                    cell = QtWidgets.QTableWidgetItem(str(item))
                    table.setItem(current_row, i, cell)
                take_picture = True
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")

    #清空紀錄座標
    def clear_table(self):
        # 清空行数
        self.ui.tableWidget.setRowCount(0)
        # # 清空列数
        self.ui.tableWidget.setColumnCount(0)
        # 清空数据
        self.ui.tableWidget.clearContents()

        self.ui.tableWidget.setRowCount(0)
        self.ui.tableWidget.setColumnCount(6)
        self.ui.tableWidget.setHorizontalHeaderLabels(['x','y','z','rx','ry','rz'])
    def excute_coord(self):
        global mc
        try:
            table = self.ui.tableWidget

            for row in range(table.rowCount()):
                data = []
                # 获取每一行的数据
                # print(table.item(row))
                for column in range(table.columnCount()):
                    item = table.item(row, column)
                    if item is not None:
                        data.append(float(item.text()))
                    else:
                        data.append('')
                # 处理数据
                print(data)
                mc.send_coords(data, 50, 1)
                while(mc.is_moving() == 1) :
                    print("moving")
        except:
            if self.ui.lineEdit.text() != "":
                try:
                    mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                    mc.connect()
                    mc.power_on()
                    self.ui.connect_button.setText("連線成功!")
                    return
                    # self.ui.connect_button.setDisabled()
                except:
                    self.ui.lineEdit.setText("connect Error!")
                    return
    def connect_Robot(self):
        global mc
        global server_ip
        server_ip = self.ui.lineEdit.text()
        print("按了",self.ui.lineEdit.text())
        if self.ui.lineEdit.text() != "" :
            try:
                mc = MyCobotSocket(self.ui.lineEdit.text(), 9000)
                mc.connect()
                mc.power_on()
                self.ui.connect_button.setText("連線成功!")
                # self.ui.connect_button.setDisabled()
            except:
                self.ui.lineEdit.setText("connect Error!")
    def export_csv(self):
        table = self.ui.tableWidget
        datalist = []
        for row in range(table.rowCount()):
            data = []
            # 获取每一行的数据
            # print(table.item(row))
            for column in range(table.columnCount()):
                item = table.item(row, column)
                if item is not None:
                    data.append(float(item.text()))
            datalist.append(data)
        csv_data = pd.DataFrame(datalist)
        csv_data.columns=['dx','dy','dz','ax','ay','az']
        path = f'{id}/pos'
        if not os.path.isdir(path):
            os.makedirs(path)
        csv_data.to_csv(f'{path}/pos.csv', index=False)
        self.ui.check.setText("匯出成功!")
    def chess_axis_print(self):
        global chess_finder
        if self.ui.chess_axis.text() == "開啟座標投影":
            chess_finder = True
            self.ui.chess_axis.setText('關閉座標投影')
        else:
            chess_finder = False
            self.ui.chess_axis.setText('開啟座標投影')

    def trace_obj(self):
        global trace_object
        global set_cood_for_obj

        print(trace_object)
        if self.ui.trace_obj.text() == "開啟追蹤":
            trace_object = True
            set_cood_for_obj = True
            start_flag = True

            self.ui.trace_obj.setText('關閉追蹤')
        else:
            trace_object = False
            self.ui.trace_obj.setText('開啟追蹤')
    #手臂追蹤物件
    def move_by_cam(self,X,Y):
        global mc
        global set_cood_for_obj
        print("啟動移動", set_cood_for_obj)

        # set_cood_for_obj = False
        if set_cood_for_obj:
            try:
                if X ** 2 > 26 or Y ** 2 > 26:
                    # trace_object = False
                    coords = mc.get_coords()
                    print("當前座標 : ", coords)
                    print(f"補正值 Y = {coords[1] - X*2}補正值 Z = {coords[2] - Y*2}")
                    time.sleep(0.2)
                    mc.send_coords(
                        [coords[0], coords[1], coords[2]+5 ,  coords[3] + round(X,2), coords[4] - round(Y,2), coords[5]], 50, 1)
                        # [coords[0], round(coords[1] - X*3,2), round(coords[2] - Y*3.5,2),  coords[3], coords[4], coords[5]], 50, 1)
                        # [coords[0]-2, round(coords[1] - X*3,2), round(coords[2] - Y*3.5,2), 0,85,0], 50, 1)
                    time.sleep(0.2)
                    # mc.send_angle(Angle.J5.value, 6, 50)

                    time.sleep(5)
                else:
                    print("移動完畢")
                # if Y ** 2 > 1:
                #     coords = mc.get_coords()
                #     print("當前座標 : ", coords)
                #     time.sleep(s)
                #     # mc.send_coords([coords[0], coords[1], coords[2]+Y, 0, 80, 0], 50, 1)
                #     time.sleep(0.3)
                #     mc.send_coords([coords[0], coords[1] , coords[2]- Y,0,80,0], 50, 1)
                #     time.sleep(0.3)
                #     print("移動完畢")
                #     time.sleep(s)
                # if countk == 4:
                #     set_cood_for_obj = False
            except:
                print("too fast")

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = PyQt_MVC_Main()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
