'''

最後修正日期：2023/03/28

將手眼校正的 眼在手內，校正完成

重點: 在手眼校正函數 calibrateHandeye() 中，需設定校正方法 method = 3 = cv2.CALIB_HAND_EYE_ANDREFF (針對眼在手上)

'''
import pandas as pd
import cv2
import numpy as np
import glob
from math import *
import pandas as pd
import os
import yaml
import traceback
import time
from SDK import *
import argparse

'''
   dir           : 校正影像及pos手臂資訊，儲存資料夾路徑
   chess_board_x_num img  : 標定板x軸點數量 預設8   
   chess_board_y_num      : 標定板x軸點數量 預設6   
   save_RT                : 儲存手眼標定結果 預設0 不儲存，1 儲存
   show_RT_validate       : 顯示手眼標定驗證結果 預設1 顯示，0 顯示
   
'''
def HandEye_Calibration(dir,camera_matrix,chess_board_x_num=8,chess_board_y_num=6,save_RT=0,show_RT_validate=1):
    #機械手臂位置及相機拍攝圖片
    lsdir = file
    square_size = 1.2 # 標定板方格大小（单位：cm）
    Cam_point = []#存放標定板原點相對於相機的座標
    #讀取相機內參
    with open(camera_matrix, 'r') as f:
        data = yaml.load(f.read(),Loader = yaml.FullLoader)

    #相機內參矩陣
    mtx  = np.array(data["camera_matrix"], dtype=np.float64)
    dist = None

    #取得棋盤格世界座標及投影座標
    objp,axis = define_chess(chess_board_x_num,chess_board_y_num,square_size)

    good_picture=[]#存放棋盤格圖片
    pic_num = 1
    for i in os.listdir(f'{lsdir}/img'):
        if i != None:
            if i.split('.')[1] == "jpg":
                # print(i)
                good_picture.append(pic_num)
                pic_num += 1

    # print(good_picture)
    folder = f'{lsdir}/img'

    """
        獲取每一張標定板到相機的變換矩陣
    """
    #計算board to cam 變換矩陣
    R_all_chess_to_cam_1=[]
    T_all_chess_to_cam_1=[]

    #找出標定板相對於相機座標的變換矩陣，並依序存放至陣列中
    for i in good_picture:

        image_path=folder+'/'+str(i)+'.jpg'
        RT=get_RT_from_chessboard(image_path, chess_board_x_num, chess_board_y_num, mtx,0,objp,axis,dist,Cam_point,mtx)
        R_all_chess_to_cam_1.append(RT[:3,:3])
        T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3,1)))

    """
        獲取每一張相片拍攝時對應的手臂座標姿態變換矩陣
    """
    file_address=f'{lsdir}/pos/pos.csv'#从记录文件读取机器人六个位姿
    sheet_1 = pd.read_csv(file_address)

    #計算end to base 變換矩陣
    R_all_end_to_base_1=[]
    T_all_end_to_base_1=[]

    #找出相機拍攝照片時對應的手臂姿態位置，並依序存放至陣列中
    for i in range(len(good_picture)):

        RT=pose_robot(sheet_1.iloc[i]['ax'],sheet_1.iloc[i]['ay'],sheet_1.iloc[i]['az'],sheet_1.iloc[i]['dx'],
                                          sheet_1.iloc[i]['dy'],sheet_1.iloc[i]['dz'])

        R_all_end_to_base_1.append(RT[:3, :3])
        T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))

    # print("末端到基座旋轉矩陣 : ",R_all_end_to_base_1)
    # print("末端到基座平移向量 : ",T_all_end_to_base_1)
    # print("標定板到相機旋轉矩陣 : ",R_all_chess_to_cam_1)
    # print("標定板到相機平移向量 : ",T_all_chess_to_cam_1)

    #手眼標定函數，method=3代表使用CALIB_HAND_EYE_ANDREFF 標定方法
    R,T=cv2.calibrateHandEye(R_all_end_to_base_1,T_all_end_to_base_1,R_all_chess_to_cam_1,T_all_chess_to_cam_1,method=3)#,method=cv2.CALIB_HAND_EYE_ANDREFF)#手眼标定

    print(f'相機相對於末端的旋轉矩陣 : \n{R}\n相機相對於末端的位移向量 : \n{T}')


    #寫入相機與手臂末端的變換矩陣
    if save_RT:
        data= {'camera_to_end': np.asarray(RT).tolist()}
        with open(f"{lsdir}/camera_to_end_{time.strftime('%Y%m%d_%H%M%S')}.yaml", "w")as f:
            yaml.dump(data, f)

    # 根據每張圖片，與對應的手臂座標姿態，做驗證
    for i in range(len(good_picture)):
        vali = show_RT_validate
        print('=' * 30)
        print('第', i+1, '張校正板圖片')
        print('=' * 30)


        ''' Step 1. 標定板相對於相機的變換矩陣 OK'''

        RT_chess_to_cam=np.column_stack((R_all_chess_to_cam_1[i],T_all_chess_to_cam_1[i]))
        RT_chess_to_cam=np.row_stack((RT_chess_to_cam,np.array([0,0,0,1])))
        print("1.驗證標定板相對於相機的變換矩陣 : ",np.dot(RT_chess_to_cam, [0,0,0,1])) if vali else print('')
        print('=' * 200)

        '''==============================================================================='''


        '''Step 2.相機相對於末端的變換矩陣'''

        RT_cam_to_end=np.column_stack((R,T))
        RT_cam_to_end=np.row_stack((RT_cam_to_end,np.array([0,0,0,1])))
        print("2.驗證相機相對於手臂末端的變換矩陣 = (相機與末端的安裝距離):　", np.dot(RT_cam_to_end, [0, 0, 0, 1])) if vali else print('')# 是相機到末端
        print('=' * 200)

        '''==============================================================================='''

        ''' Step 3.末端到基座的變換矩陣 OK'''

        RT_end_to_base = np.column_stack((R_all_end_to_base_1[i], T_all_end_to_base_1[i]))
        RT_end_to_base = np.row_stack((RT_end_to_base, np.array([0, 0, 0, 1])))
        # 將變換矩陣轉變為旋轉矩陣

        print("3.末端相對於基座的變換矩陣 : \n",RT_end_to_base)
        print('=' * 200)

        thedaX = atan2(RT_end_to_base[2, 1], RT_end_to_base[2, 2]) * 180 / np.pi
        thedaY = atan2(RT_end_to_base[2, 0] * -1, sqrt(RT_end_to_base[2, 0] ** 2 + RT_end_to_base[2, 2] ** 2)) * 180 / np.pi
        thedaZ = atan2(RT_end_to_base[1, 0], RT_end_to_base[0, 0]) * 180 / np.pi
        print('3.1 驗證末端相對於基座的變換矩陣 = (當前手臂座標) ',np.dot(RT_end_to_base, [0,0,0,1])) if vali else print('')
        print("3.2 驗證手臂末端姿態 :　",(thedaX,thedaY,thedaZ))
        print(f"3.3 實際手臂座標 : \nX:{sheet_1.iloc[i]['dx']},Y:{sheet_1.iloc[i]['dy']},Z:{sheet_1.iloc[i]['dz']}\nrx:{sheet_1.iloc[i]['ax']},ry:{sheet_1.iloc[i]['ay']},rz:{sheet_1.iloc[i]['az']}")
        print('=' * 200)

        '''==============================================================================='''

        ''' 標定板 to 相機 to 末端 to 基座 '''
        RT_chess_to_base=RT_end_to_base@RT_cam_to_end#即为固定的棋盘格相对于机器人基坐标系位姿
        image_path = folder + '/' + str(i+1) + '.jpg'
        coord = [Cam_point[i][0][0]*10, Cam_point[i][1][0]*10, Cam_point[i][2][0]*10,1]  # 座標，待測
        print("4.標定板原點相對於相機座標 (相機原點與標定板距離): ",coord) if vali else print('')
        print('=' * 200)
        print("5.標定板相對於末端座標 : ",np.dot(RT, coord)) if vali else print('')
        print('=' * 200)
        print("6.末端轉換手臂基座座標為 : ", (RT_end_to_base@RT_cam_to_end)@coord) if vali else print('')
        print('=' * 200)
        print("7.相機座標相對於機械手臂基座的變換矩陣 : \n",RT_chess_to_base)
        print('*'*200)
        # 畫出棋盤格投影座標
        get_RT_from_chessboard(image_path, chess_board_x_num, chess_board_y_num, mtx, 1, objp, axis, dist, Cam_point, mtx)
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir",default='2023_03_20_11_29_41',help="請輸入校正影像及pos手臂資訊的儲存資料夾路徑")
    parser.add_argument("--mtx",default='calibration_matrix_OAK-D_1280x1080.yaml',help="請輸入相機校正的內參矩陣yaml檔",type=str)
    parser.add_argument("--conner_x_num",default=8,help="標定板x方向角點數量，預設8")
    parser.add_argument("--conner_y_num",default=6,help="標定板y方向角點數量，預設6")
    parser.add_argument("--save_RT",default=0,help="是否儲存手眼標定矩陣,預設 0 不儲存")
    parser.add_argument("--show_RT_validate",default=1,help="顯示手眼標定驗證結果,預設1 顯示")
    args = parser.parse_args()
    HandEye_Calibration(args.dir,args.mtx,args.conner_x_num,args.conner_y_num,args.save_RT,args.show_RT_validate)