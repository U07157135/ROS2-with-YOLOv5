# 在無人機上以ROS2技術實現YOLOv5物件偵測

## Requirements
### 本地端的電腦

* **Install ROS2**
    * 安裝ROS2到本地端電腦，安裝的步驟請參考[ROS2官方網站](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)安裝教學
    * 本專案在本地端電腦使用的ROS2版本為**foxy**

* **Install ROS2-v4l2-camera Package**
    * 安裝ROS2的[ROS2-v4l2-camera](https://index.ros.org/r/v4l2_camera/)包到本地端，版本為foxy
```
sudo apt install ros-foxy-v4l2-camera
```

* **Install Cuda**
    * 先確認本地端顯示卡(GPU)的驅動版本，再透過[官方提供的文檔](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html)選擇與顯示卡驅動對應之Cuda版本
    * 選擇要安裝的Cuda版本，https://developer.nvidia.com/cuda-toolkit-archive
    * 本專案使用的Cuda版本為11.6

* **Install PyTorch**
    * [官方](https://pytorch.org/)
    * 本專案使用的PyTorch版本為

* **Install YOLOv5 dependencies**
    * 從YOLOv5的github上把包Clone下來，再進入到yolov5文件內使用pip安裝requirement.txt文件內的所有依賴項。
```
git clone https://github.com/ultralytics/yolov5  # clone
cd yolov5
pip install -r requirements.txt  # install
```

### 無人機上的樹莓派

* **Install ROS2**
    * 安裝ROS2到樹莓派上，安裝的步驟請參考[ROS2官方網站](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)安裝教學
    * 本專案在樹莓派使用的ROS2版本為**galactic**

* **Install ROS2-v4l2-camera Package**
    * 安裝ROS2的[ROS2-v4l2-camera](https://index.ros.org/r/v4l2_camera/)包到樹莓派上，版本為galactic
```
sudo apt install ros-galactic-v4l2-camera
```
## 環境布置

1. 在本地端建立工作目錄並進入工作目錄
```
mkdir -p ~/dev_ws/src && cd ~/dev_ws/src
```
2. 在工作目錄建立ROS2套件，&lt;Package Name&gt;為ROS2包的名稱
```
ros2 pkg create --build-type ament_python <Package Name>
```
3. 將本專案中的example資料夾內的檔案複製至./&lt;Package Name&gt;/&lt;Package Name&gt;下
```
cd ~
git clone https://github.com/U07157135/ros2-yolov5.git
cp ~/ros2-yolov5/example/* ~/dev_ws/src/<Package Name>/<Package Name>/
```
4. 安裝依賴並建構ROS2套件
```
cd ~/dev_ws/src
rosdep install -i --from-path src --rosdistro foxy -y
colcon build 
```

## 使用方式
### 本地端的電腦
本地端有兩個節點一個節點為解壓影像縮節點另一個節點為顯示影像節點，因為解壓縮影像的節點是使用ROS2的[v4l2-camera](https://index.ros.org/r/v4l2_camera/)包，而顯示影像的節點是我們自己寫的，所以必須分開啟動。
* 解壓縮影像節點啟動方式
```
ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed
```
* 顯示影像節點啟動方式  
&lt;Package Name&gt;為ROS2包的名稱
```
python3 ~/dev_ws/src/<Package Name>/<Package Name>/get_frame.py
```
 
### 無人機上的樹梅派
無人機擷取影像的節點和壓縮影像的節點為同個包所以只需開起壓縮影像的節點就可以了。
```
ros2 run image_transport republish compressed in/compressed:=image_raw/compressed raw out:=image_raw/uncompressed
```


## UI介面說明    
![](https://i.imgur.com/oc7veKg.jpg)

我們將UI介面分為四個部分，如上圖我們用紅色框框起來的為一部分並在上面標號。
1. 紅框1為樹苺派的Raspberry Pi VR220 Camera取得影像透過ROS2傳遞影像資訊到本地端的電腦，最後再對影像的解析度做調整，最後再顯示再UI介面上。
    
2. 紅框2為影像數據，影像數據提供了影像的偵率、影像的解析度和影像顯示時間。

3. 紅框3為物件偵測checkbox，當checkbox勾選時會使用YOLOv5偵測技術來進行物件偵測，我們使用YOLOv5的yolov5s模型來進行物件偵測，而物件偵測可以偵測的物件有[80種](https://cocodataset.org/#explore)像是人、腳踏車、摩托車、飛機等等的物件都可以偵測得到。
4. 紅框4為畫面顯示開關按鈕，第一次開啟UI時顯示的影像會為雜訊畫面這表示畫面為關閉的，當按下第二次時為開啟畫面，開啟畫面後會顯示樹苺派的Raspberry Pi VR220 Camera取得的影像。
    
## ROS2架構

![](https://github.com/U07157135/ros2-yolov5/blob/main/img/ROS2.gif)

在無人機上以ROS2技術實現YOLOv5物件偵測使用了兩個裝置一個是樹苺派一個是本地端電腦，而我們在樹苺派和本地端各設置了兩個節點，在樹苺派上的兩個節點一個是負責錄製影像的節點另一個是負責壓縮影像，而在本地端的兩個節點一個是負責顯示影像的節點一個是負責解壓縮的節點，每個節點都是通過ROS2的Subscriber和Publisher來進行溝通，當Subscriber訂閱了Publisher之後每當Publisher將資料發布底下的Subscriber就會接收到來自Publisher發布的資料。
    
## YOLOv5物件偵測
![](https://i.imgur.com/3TCFnxi.jpg)

物件偵測我們使用比較熱門的YOLOv5，YOLOv5提供了許多開發環境和已經訓練好的權重模型，權重模型是使用MS COCO Dataset來進行訓練，它是一個Microsoft、Facebook等組織所提供的一個大型開源圖片數據集，而圖片數據集的種類有80多種，還有我們可以透過PyTorch Hub和PyTorch的API輕鬆的加載已經訓練完成的YOLOv5s模型和對圖像進行推理，這使得開發更加的容易。


## UI介面設計  
![](https://i.imgur.com/sALGlaT.png)

UI是使用PyQt5來進行開發，搭配Qt Designer可以使得UI設計更視覺化可以簡單且直覺的繪製想要的功能，繪製完後再通過PyUIC這個工具將繪製好的.ui檔轉換成.py檔，接著再透過python import模組來呼叫UI介面的物件，最後再把UI介面的功能設計就完成了，目前只是初步簡單設計只有一些功能，未來可能會再新增。
    
## 無人機模具設計
![](https://i.imgur.com/Vw2PUr7.png)

為了可以在無人機上安裝樹莓派和樹莓派相機所以必須設計可以用來安裝樹莓派和樹梅派相機的模具，我們使用FreeCAD來進行模具的繪製，對於設計要符合無人機機身的模具和在不擋住無人機的鏡頭的同事要保持樹莓派相機的穩定我們是花了數個月才解決，由於無人機上有許多不規則的形狀在設計時只能大概猜測要印的模具大小，透過不斷嘗試和失敗最後終於印製符合機身的模具。