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

* **Install Python dependencies**
    * 安裝本專案所需要的Python依賴項
```
git clone https://github.com/U07157135/ros2-yolov5.git  # clone
cd ros2-yolov5
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

## UI介面使用說明    

## 成果展示
    
## ROS2架構


