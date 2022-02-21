# 在無人機上以ROS2技術實現YOLOv5物件偵測

## Requirements
### 本地端的電腦

* **Install ROS2**
    * 安裝ROS2到本地端電腦，安裝的步驟請參考[ROS2官方網站](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html)安裝教學
    * 本專案在本地端電腦使用的ROS2版本為**foxy**

* **Install ROS2-v4l2-camera Package**
    * 安裝ROS2的ROS2-v4l2-camera包到本地端
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
    * 安裝ROS2的v4l2-camera包到樹莓派上
```
sudo apt install ros-galactic-v4l2-camera
```
## 環境布置

1. 在本地端建立工作目錄並進入工作目錄
```
mkdir -p ~/dev_ws/src && cd ~/dev_ws/src
```
2. 在工作目錄建立ROS2套件，<Package Name>為ROS2包的名稱
```
ros2 pkg create --build-type ament_python <Package Name>
```
3. 將本專案中的example資料夾內的檔案複製至./<Package Name>/<Package Name>下
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
<Package Name>為ROS2包的名稱
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
    
## 成果展示
    
UI介面    
    
無人機上的魚眼相機支架和樹苺派支架
    
## ROS2架構

![](https://github.com/U07157135/ros2-yolov5/blob/main/img/result.gif)
