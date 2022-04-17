import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from .Ui_untitled import Ui_MainWindow
from PyQt5 import QtWidgets,QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import sys
import time
import cv2
import torch
import numpy as np
from threading import Thread

from Moildev import Moildev


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.camera = False
        self.detect = False
        self.start_count_time = False

        self.stream_video_thread = StreamVideoThread()
        self.stream_video_thread.img_0.connect(self.show_frame_0)
        self.stream_video_thread.img_1.connect(self.show_frame_1)
        self.stream_video_thread.fps.connect(self.update_fps)
        self.stream_video_thread.start()

        self.count_time_thread = Thread(target = self.count_time)
        self.count_time_thread.start()

        self.btn_connect()
        
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s',device='0')
    
    def btn_connect(self):
        self.ui.camera_btn.clicked.connect(self.open_stream)
        self.ui.detect_checkbox.clicked.connect(self.checkbox_click_event)

    def checkbox_click_event(self):
        if self.ui.detect_checkbox.isChecked():
            self.detect = True
        else:
            self.detect = False

    def update_fps(self,fps):
        if self.camera:
            self.ui.fps_value.setText(str(fps))
        else:
            self.ui.fps_value.setText(str(0))

    def open_stream(self):
        if not self.camera:
            self.ui.detect_checkbox.setCheckable(True)
            self.camera = True
        else:
            self.ui.detect_checkbox.setCheckable(False)
            self.ui.detect_checkbox.setCheckState(False)
            self.camera = False

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

    def show_frame_0(self,img):
        max_width = 500
        max_heigh = 500
        if self.camera:
            heigh, width, _ = img.shape
            if self.detect:
                img = self.model(img)
                img = img.render()[0]

            if width <= max_width and heigh <= max_heigh:
                pass
            elif width>heigh:
                scale = 1.0*width/max_width
                width = int(width/scale)
                heigh = int(heigh/scale)
            elif width<heigh:
                scale = 1.0*heigh/max_heigh
                width = int(width/scale)
                heigh = int(heigh/scale)

            img = cv2.resize(img,(width,heigh))
        else :
            width = max_width
            heigh = max_heigh
            img = np.random.randint(255, size=(width, heigh,3),dtype=np.uint8)

        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)    
        img = QImage(img,width,heigh,3*width,QImage.Format_RGB888)    
        self.ui.resultion_value.setText(f"{width}X{heigh}")
        self.ui.view_label.setGeometry(10,10,width,heigh)
        self.ui.view_label.setPixmap(QPixmap.fromImage(img))

    def show_frame_1(self,img):
        max_width = 500
        max_heigh = 500
        if self.camera:
            heigh, width, _ = img.shape
            if self.detect:
                img = self.model(img)
                img = img.render()[0]

            if width <= max_width and heigh <= max_heigh:
                pass
            elif width>heigh:
                scale = 1.0*width/max_width
                width = int(width/scale)
                heigh = int(heigh/scale)
            elif width<heigh:
                scale = 1.0*heigh/max_heigh
                width = int(width/scale)
                heigh = int(heigh/scale)

            img = cv2.resize(img,(width,heigh))
        else :
            width = max_width
            heigh = max_heigh
            img = np.random.randint(255, size=(width, heigh,3),dtype=np.uint8)

        img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)    
        img = QImage(img,width,heigh,3*width,QImage.Format_RGB888)    
        # self.ui.resultion_value.setText(f"{width}X{heigh}")
        self.ui.view_label_2.setGeometry(10,10,width,heigh)
        self.ui.view_label_2.setPixmap(QPixmap.fromImage(img))

    
class StreamVideoThread(QThread):
    img_0 = QtCore.pyqtSignal(np.ndarray) 
    img_1 = QtCore.pyqtSignal(np.ndarray) 
    fps = QtCore.pyqtSignal(int) 

    def __init__(self,*args,**kwargs):
        super(StreamVideoThread, self).__init__(*args,**kwargs)
        rclpy.init()
        self.moildev = Moildev("Parameter_Ethaniya_VR220.json")
        self.alpha, self.beta = self.moildev.getAlphaBeta(900, 0, 1)
        self.get_frame_node = GetFrameNode()
        exe = SingleThreadedExecutor()
        exe.add_node(self.get_frame_node)
        self.get_img_thread = Thread(target=exe.spin,daemon=True)
        self.get_img_thread.start()

    def run(self):
        while True:
            img = self.get_frame_node.get_frames()
            fps = self.get_frame_node.fps
            self.img_0.emit(img)
            self.img_1.emit(self.img_convert(img))
            self.fps.emit(fps)
            time.sleep(0.1)
            
    def img_convert(self, img):
        img = self.moildev.anypoint(img, self.alpha,self.beta,4, 1)
        img = cv2.resize(img, (800, 600), interpolation=cv2.INTER_AREA)
        return img


class GetFrameNode(Node):
    def __init__(self):
        super().__init__("jason_ros")
        self.bridge = CvBridge()
        self.msg_type = Image
        self.name = "image_raw/uncompressed"
        self.sub = self.create_subscription(self.msg_type,self.name,self.listener_callback,10)

        self.data = None
        self.img = None

        self.t1 = time.time()
        self.fps = 0
        self.count_frame = 0
        self.last_count_frame = 0
        
        self.sub
        
    def listener_callback(self, data):
        self.data = data
        self.fps = int(1/(time.time()-self.t1))
        self.t1 = time.time()

    def get_frames(self):
        try:
            if self.count_publishers("/image_raw/compressed") and self.data != None:
                self.img = self.bridge.imgmsg_to_cv2(self.data, "bgr8")
            else:
                self.img = np.random.randint(255, size=(400, 500,3),dtype=np.uint8)
                print("not get frame")
            return self.img
        except CvBridgeError as e:
            print(e)


def main():
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
    main()



