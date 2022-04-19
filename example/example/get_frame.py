from concurrent.futures import wait
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from Ui_untitled import Ui_MainWindow
from PyQt5 import QtWidgets,QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import sys
import cv2
import time
import torch
import numpy as np
from threading import Thread


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__() # 繼承QMainWindow的init
        self.ui = Ui_MainWindow() # 建立UI介面物件
        self.ui.setupUi(self) # 執行setupUi
        
        self.camera = False # camera flag 用來判斷相機是否打開
        self.detect = False # detect falg 用來判斷是否要使用yolov5做物件偵測

        """
        stream_video_thread:
            UI介面上的相機畫面必須在額外的線程執行，這樣才不會阻塞原本UI介面的線程
        """
        self.stream_video_thread = StreamVideoThread() # 建立StreamVideoThread物件
        self.stream_video_thread.img.connect(self.show_frame) # 將img值傳到show_frame涵式
        self.stream_video_thread.fps.connect(self.update_fps) # 將fps值傳到show_frame涵式
        self.stream_video_thread.delay.connect(self.update_delay) # 將delay值傳到show_frame涵式
        self.stream_video_thread.start() # 開始執行StreamVideoThread線程

        """
        count_time_thread:
            建立一個用來計時的線程
        """
        self.count_time_thread = Thread(target = self.count_time) #建立一個count_time_thread物件
        self.count_time_thread.start() #開始執行count_time_thread線程

        """
        btn_connect:
            在初始化的時候建立UI介面上按鈕按下時所要觸發的對象
        """
        self.btn_connect() # 執行btn_connect涵式
        
        """
        使用torch從torch hub匯入yolov5程式和yolov5s模型，並且使用GPU裝置
        """
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s',device='0', force_reload=True) # 建立yolov5物件
        
    """
    btn_connect:
        設置UI介面上按鈕元件所要觸發的函數
    """    
    def btn_connect(self):
        self.ui.camera_btn.clicked.connect(self.open_stream) # 當UI介面上的camera_btn物件被點擊時呼叫open_stream
        self.ui.detect_checkbox.clicked.connect(self.checkbox_click_event) # 當UI介面上的detect_checkbox物件被點擊時呼叫checkbox_click_event
        
    """
    checkbox_click_event:
        當checkbox勾選時會進入checkbox_click_event函數，然後再判斷checkbox的狀態做對應的事
    """
    def checkbox_click_event(self):
        if self.ui.detect_checkbox.isChecked(): 
            self.detect = True 
        else:
            self.detect = False 

    """
    update_fps:
        當camera的flag為"true"的時後設置UI介面上的fps_value物件的text屬性的值為fps值
        當camera的flag為"false"的時後設置UI介面上的fps_value物件的text屬性的值為0
    """
    def update_fps(self,fps):
        if self.camera: 
            self.ui.fps_value.setText(str(fps)) 
        else:
            self.ui.fps_value.setText(str(0)) 

    """
    update_delay:
        當camera的flag為"true"的時後設置UI介面上的delay_value物件的text屬性的值為delay值
        當camera的flag為"false"的時後設置UI介面上的delay_value物件的text屬性的值為0
    """
    def update_delay(self,delay):
        if self.camera:
            self.ui.delay_value.setText(str(delay)+"ms") 
        else:
            self.ui.delay_value.setText(str(0)+"ms")
            
    """
    open_stream:
        當camera的falg為"true"的時後則
        當camera的falg為"false"的時後則
    """
    def open_stream(self):
        if not self.camera:
            self.ui.detect_checkbox.setCheckable(True)
            self.camera = True
        else:
            self.ui.detect_checkbox.setCheckable(False)
            self.ui.detect_checkbox.setCheckState(False)
            self.camera = False
            
    """ 
    count_time:
        當camera的flag為"true"的時後則計算時間，並且將time_value物件的text屬性設為計算後的值
        當camera的flag為"false"的時後設置UI介面的time_value物件的text屬性為"00:00"
    """
    def count_time(self):
        s = 0 
        m = 0 
        while True:
            if self.camera:
                min, s = divmod(s, 60) 
                if min == 1:m+=1
                self.ui.time_value.setText('{:02d}:{:02d}'.format(m, s))
                s += 1
            else:
                self.ui.time_value.setText("00:00") 
                s = 0 
                m = 0 
            time.sleep(1)
            
    """
    window_proportion:
        先判斷camera的flag，
        當camera的flag為"true"則再判斷detect的flag
        當detect的flag為"true"則將影像傳入yolov5物件內做物件偵測，再將物件偵測後的影像透過cv2的resize將影像的解析度設置為640x480
        當detect的flag為"false"則將影像直接透過cv2的resize將影像的解析度設置為640x480
        當camera的flag為"false"則用numpy產生雜訊畫面
    """
    def window_proportion(self,img):
        w,h,_ = img.shape # 取得影像的寬、高和通道數
        if self.camera: 
            if self.detect:
                img = self.model(img)
                img = img.render()[0]
            img = cv2.resize(img,(640,480))
        else :
            img = np.random.randint(255, size=(640, 480, 3),dtype=np.uint8)
        return img ,w ,h

    """
    show_frame:
        將影像設置在UI介面上的view_label上顯示，並且設置UI介面上resultion_value的text值
    """
    def show_frame(self,img):
        try:
            img, w, h  = self.window_proportion(img) # 使用window_proportion將影像轉成
            img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB) # 使用cv2的cvtColor將影像通道從BGR改成RGB
            img = QImage(img,w,h,3*w,QImage.Format_RGB888)  # 使用QT的QImage將影像的格式改成Format_RGB888
            self.ui.view_label.setGeometry(10,10,w+2,h+2) # 設置UI介面的view_label的大小
            self.ui.view_label.setPixmap(QPixmap.fromImage(img)) # 將影像設置在UI介面上的view_label顯示
            self.ui.resultion_value.setText(f"{w}X{h}") # 設置UI介面上的resultion_value為w值和h值
        except KeyboardInterrupt:
            sys.exit()


class StreamVideoThread(QThread):
    img = QtCore.pyqtSignal(np.ndarray) 
    fps = QtCore.pyqtSignal(int) 
    delay = QtCore.pyqtSignal(float)

    def __init__(self):
        super().__init__() # 繼承QThread的init
        rclpy.init() # rclpy初始化
        self.get_frame_node = GetFrameNode() #建立GetFrameNode物件
        exe = SingleThreadedExecutor() # 建立SingleThreadedExecutor物件
        exe.add_node(self.get_frame_node) # 把SingleThreadedExecutor新增get_frame_node物件
        self.get_img_thread = Thread(target=exe.spin,daemon=True) # 建立get_img_thread物件
        self.get_img_thread.start() # 開始get_img_thread線程

    def run(self):
        try:
            while True:
                img = self.get_frame_node.get_frames() # 將get_frame_node的get_frame函數回傳的影像放到img
                fps = self.get_frame_node.fps # 將get_frame_node的fps值放到fps
                delay = self.get_frame_node.delay # 將get_frame_node的delay值放到delay
                self.img.emit(img) # 發射影像到
                self.fps.emit(fps) # 發射影像到
                self.delay.emit(delay)# 發射影像到
                time.sleep(0.1)
        except KeyboardInterrupt:
            sys.exit()

class GetFrameNode(Node):
    def __init__(self):
        super().__init__("jason_ros") # 繼承Node類的init，並設置ros節點名稱為Jason_ros
        self.bridge = CvBridge() #建立cvbridge物件
        msg_type = Image # subscription的訊息格式
        name = "image_raw/uncompressed" # subscription的名子
        sub = self.create_subscription(msg_type,name,self.listener_callback,10) #建立一個subscription

        self.data = None # 用來存放節點收到的訊息
        self.img = None # 用來存放訊息轉換成影像後的值

        self.t1 = time.time() # 用來存放時間
        self.delay = 0 # 用來存放時間延遲的值
        self.fps = 0 # 用來存放fps值

        sub # 執行subscription
        
    def listener_callback(self, data):
        self.data = data # 將data放到self.data
        self.delay = round((time.time()-self.t1)*1000,2) # 計算延遲的時間
        self.fps = int(1/(time.time()-self.t1)) #計算fps值
        self.t1 = time.time() # 更新上次回傳的時間值

    def get_frames(self):
        try:
            if self.count_publishers("/image_raw/compressed") and self.data != None: # 當網域中沒有名為/image_raw/compressed的publisher和self.data為空的時候
                self.img = self.bridge.imgmsg_to_cv2(self.data, "bgr8") # 將self.data透過cvbridge轉換成影像存放在self.img
            else:
                self.img = np.random.randint(255, size=(640,480,3),dtype=np.uint8) #產生雜訊畫面
            return self.img
        except CvBridgeError as e:
            print(e)
        except KeyboardInterrupt: 
            sys.exit()


def main():
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()



