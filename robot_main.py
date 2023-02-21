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

mc = None
server_ip = ""
s = 0.1
step = 10
take_picture = False
id = time.strftime('%Y_%m_%d_%H_%M_%S')
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
            mc = MyCobotSocket("192.168.2.149", 9000)
            mc.connect()
            mc.power_on()

            coords = mc.get_coords()
            print(coords)
            self.ui.lineEdit.setText("192.168.2.149")
            self.ui.Z_value.setText(str(coords[2]))
            self.ui.Z_value_slider.setValue(int(coords[2]))
            self.ui.connect_button.setText("連線成功!")
        except:
            self.ui.lineEdit.setText("連線失敗，請重試!")

    def cam(self):
        global take_picture
        # 定義管道
        pipeline = dai.Pipeline()
        # 定義取用RGB相機
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(416, 416)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        # XLinkOut 是設備的“輸出”。您要傳輸到主機的任何數據都需要通過 XLink 發送
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        # 將相機輸出串流畫面端命名為rgb
        xout_rgb.setStreamName("rgb")
        # 將上面定義的RGB相機預覽輸入至XLinkOut，以便將幀發送至主機
        cam_rgb.preview.link(xout_rgb.input)
        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue("rgb")

            while True:
                in_rgb = q_rgb.get()

                frame = in_rgb.getCvFrame()
                frame = cv2.resize(frame, (416, 416))  # 改變尺寸和視窗相同
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 轉換成 RGB
                height, width, channel = frame.shape  # 讀取尺寸和 channel數量
                bytesPerline = channel * width  # 設定 bytesPerline ( 轉換使用 )
                if take_picture:
                    take_picture = False
                    path = f'{id}/img'
                    img_id = 1
                    if not os.path.isdir(path):
                        os.makedirs(path)

                    for i in os.listdir(path):
                        img_id+=1
                    cv2.imwrite(f'{path}/{img_id}.jpg', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                # 轉換影像為 QImage，讓 PyQt5 可以讀取
                img = QImage(frame, width, height, bytesPerline, QImage.Format_RGB888)
                self.ui.cam.setPixmap(QPixmap.fromImage(img))  # QLabel 顯示影像
    def linkEvent(self):
        global mc
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
        self.ui.Z_value_slider.valueChanged.connect(self.getslidervalue)
        self.ui.Z_value_slider.sliderReleased.connect(self.Z_change)
        return

   # [151.5, -0.6, 211.0, 97.28, 25.21, 92.88]



    def home(self):
        global mc
        try:
            mc.send_angles([0, -15, 0, 0, 0, 0], 50)
            time.sleep(s)
            coords = mc.get_coords()
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
            mc.send_coords([ coords[0]+step, coords[1], coords[2],  -165, 85,-165], 50, 1)
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
            mc.send_coords([coords[0] , coords[1]+ step, coords[2], -165, 85,-165], 50, 1)
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
            mc.send_coords([coords[0] - step, coords[1], coords[2],  -165, 85,-165], 50, 1)
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
            mc.send_coords([coords[0], coords[1] - step, coords[2], -165, 85,-165], 50, 1)
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
    def getslidervalue(self):
        self.ui.Z_value.setText(f"{float(self.ui.Z_value_slider.value())}")


    def Z_change(self):
        global mc
        try:
            print(self.ui.Z_value_slider.value())
            coords = mc.get_coords()
            print("1----: ",coords)
            # time.sleep(0.1)
            if coords != []:
                mc.send_coords([coords[0], coords[1], float(self.ui.Z_value_slider.value()),  -165, 85,-165], 50, 1)
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
            os.mkdir(path)
        csv_data.to_csv(f'{path}/pos.csv', index=False)
        self.ui.check.setText("匯出成功!")

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = PyQt_MVC_Main()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
