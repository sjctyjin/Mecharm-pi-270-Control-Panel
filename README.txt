在此目錄下執行cmd

在終端機下輸入 : venv\Scripts\activate 啟動本專案python 虛擬環境，以執行以下程式測試。

手眼校正步驟 : 參考檔案 眼在手上_手眼標定.pdf。

Camera_calibration.py  		-- 執行相機校正，取得相機內參與畸變參數yaml檔。

	執行指令 例 : python Camera_calibration.py --file 2023_03_20_11_29_41/img --save_dist 0 --save_corner 0

HandEye_calibration.py 		-- 執行手眼校正，取得手臂末端與相機的變換矩陣。

	執行指令 例 : python HandEye_calibration.py --file 2023_03_20_11_29_41 --mtx calibration_matrix_OAK-D_1280x1080.yaml --save_RT 0 --show_RT_validate 1

SDK.py		     		-- 本案使用的函式庫

Validate_rotation_matrix.py   -- 驗證旋轉矩陣(將旋轉矩陣轉回歐拉角提供對應驗證是否與原始值相等)。
robot arm 	  	     		-- 機械手臂圖形化操作介面原始碼。
robot_control_ui.bat		-- 啟動手臂操作圖形化介面。