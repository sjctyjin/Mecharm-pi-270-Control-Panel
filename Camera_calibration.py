import time
import numpy as np
import cv2
import yaml
import os
import argparse

#import pathlab



# source_path = "" #ini untuk source image kita simpan kat mane??

'''
   dir           : 校正影像儲存資料夾路徑
   dst img        : 儲存畸變校正結果     dst img : 1 預設0,
   findconner_img : 儲存角點檢測校正結果  findconner_img : 1 預設0
   
'''

def calibration(file,dist_img=0,findconner_img=0,corner_x = 8,corner_y=6):
    print('image found :',len(os.listdir()))
    # corner_x = 8  # number of chessboard corner in x direction
    # corner_y = 6  # number of chessboard corner in y direction

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((corner_y * corner_x, 3), np.float32)
    objp[:, :2] = np.mgrid[0:corner_x, 0:corner_y].T.reshape(-1, 2)

    # Arrays to store object points and image points from a ll the images.
    objpoints = []  # 3d point in real world space
    jpegpoints = []  # 2d points in image plane.
    images = []
    for i in os.listdir(f"{file}"):

        if os.path.splitext(i)[1] == ".jpg":

            images.append(f"{file}/"+i)
            # break
    if dist_img:
        if not os.path.isdir("dist"):
            os.makedirs("dist")
    if findconner_img:
        if not os.path.isdir("chessboard"):
            os.makedirs("chessboard")

    cv2.namedWindow("chessboard", cv2.WINDOW_NORMAL)
    cv2.namedWindow("undistort", cv2.WINDOW_NORMAL)
    cv2.namedWindow("undistort2", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("chessboard", 600, 400)
    cv2.resizeWindow("undistort", 600, 400)
    cv2.resizeWindow("undistort2", 600, 400)
    found = 0
    k = 0
    for fname in images: # here, 10 can be changed to whatever number you like to choose
        print(fname)
        # if k < 15:
        print(k)
        k+=1
        jpeg = cv2.imread(fname) # capture frame by frame
        # cv2.imshow('jpeg', jpeg)

        # cv2.waitKey(1)
        # print(fname)
        gray = cv2.cvtColor(jpeg, cv2.COLOR_BGRA2GRAY)

        # find the chess noard corners
        ret, corners = cv2.findChessboardCorners(gray, (corner_x,corner_y), None)
        # if found, ass object points, image points (after refining them)
        x, y = gray.shape[:2]
        print(x, y)
        cv2.circle(jpeg,(int(y/2),int(x/2)), 5, (0, 0, 255), -1)
        if ret == True:
            print(corners)
            objpoints.append(objp) #Certainly, every loop objp is the same in 3D
            corners2 = cv2.cornerSubPix(gray,corners,(20,5),(-1,-5),criteria)
            jpegpoints.append(corners2)
            # Draw and display the corners
            jpeg = cv2.drawChessboardCorners(jpeg, (corner_x,corner_y), corners2, ret)
            found += 1
            cv2.imshow('chessboard', jpeg)
            cv2.imwrite(f'chessboard/{fname.split("/")[2]}_chessboard.jpg', jpeg)
            cv2.waitKey(1)
    print("number of images used for calibration: ", found)


    #calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, jpegpoints, gray.shape[::-1], None, None)

    # transforms the matrix distortion coefficients to writeable lists
    data= {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
    print(mtx)
    print(dist)
    # and save it to a file
    with open(f"calibration_matrix_{time.strftime('%Y%m%d_%H%M%S')}_OAK-D_1280x1080.yaml", "w")as f:
        yaml.dump(data, f)


    #undistort image

    for fname in images: # here, 10 can be changed to whatever number you like to choose
         jpeg = cv2.imread(fname) # Capture frame-by-frame
         #cv2.imshow('jpeg', jpeg)
         #cv2.waitkey(500)
         h, w = jpeg.shape[:2]

         newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx, dist , (w,h), 1, (w,h))
         #print(newcameramtx)
         #undistort
         dst = cv2.undistort(jpeg, mtx, dist, None, newcameramtx)
         cv2.imshow('undistort', dst)
         cv2.waitKey(1)

         # crop the image
         x, y, w, h = roi
         dst = dst[y:y+h, x:x+w]
         cv2.imshow('undistort2', dst)
         cv2.imwrite(f'dist/{fname}_2.jpg', dst) if dist_img else print("no save dist img")
         cv2.waitKey(1)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default='2023_03_20_11_29_41/img', help="請輸入校正影像儲存資料夾路徑")
    parser.add_argument("--save_dist", default=0, help="儲存畸變校正結果")
    parser.add_argument("--save_corner", default=0, help="儲存角點檢測校正結果")
    parser.add_argument("--conner_x_num", default=8, help="標定板x方向角點數量，預設8")
    parser.add_argument("--conner_y_num", default=6, help="標定板y方向角點數量，預設6")
    args = parser.parse_args()
    calibration(args.dir,args.save_dist,args.save_corner,args.conner_x_num,args.conner_y_num)