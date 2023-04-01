# PCL Object Detection
- Point Cloud Libraryを用いたルールベース物体検出パッケージ

# Contents
- [How to Use](#how-to-use)
    - [point_cloud_object_detection.launch](#point_cloud_object_detectionlaunch)
    - [line_detection.launch](#line_detectionlaunch)
    - [demo.launch](#demolaunch)

# How to Use
## [point_cloud_object_detection.launch](launch/point_cloud_object_detection.launch)
- 机、床、棚上の物体検出と配置位置の検出をします
- 検出位置はトピック通信とTFで出力されます
- 各検出モードはサービス通信によって変更可能
- 詳細は[こちら](doc/md/point_cloud_object_detection.md)

```bash
# rvizあり
roslaunch point_cloud_object_detection point_cloud_object_detection.launch
# rvizなし
roslaunch point_cloud_object_detection point_cloud_object_detection.launch rviz:=false
# rqt_reconfigureによるパラメータ調整
roslaunch point_cloud_object_detection point_cloud_object_detection.launch rqt_reconfigure:=true
```
※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

<div align="center">
    <img src="doc/img/table.png" width="1080">
    <!-- <img src="doc/img/floor.png" width="1080"> -->
    <!-- <img src="doc/img/placeable.png" width="1080"> -->
</div>

- [Topに戻る](#pcl-object-detection)

## [line_detection.launch](launch/line_detection.launch)
- 2D-LiDARセンサから得た点群から直線を検出します
- 詳細は[こちら](doc/md/line_detection.md)

```bash
# rvizあり
roslaunch point_cloud_object_detection line_detection_param.launch
# rvizなし
roslaunch point_cloud_object_detection line_detection_param.launch rviz:=false
# rqt_reconfigureによるパラメータ調整
roslaunch point_cloud_object_detection line_detection_param.launch rqt_reconfigure:=true
```
※rqt_reconfigureはパラメータを動的に変更できるが，パラメータファイルは上書きされません。  
※rqt_reconfigureでパラメータを調整後、パラメータファイルを手打ちで変更してください。  

<div align="center">
    <img src="doc/img/line_detection.png" width="1080">
</div>

- [Topに戻る](#pcl-object-detection)

## [demo.launch](launch/demo/demo.launch)
```bash
# TABLE_MODE
$ roslaunch pcl_object_detection demo.launch detection_mode:=1
# FLOOR_MODE
$ roslaunch pcl_object_detection demo.launch detection_mode:=2
# SHELF_MODE
$ roslaunch pcl_object_detection demo.launch detection_mode:=3
# PLACEABLE_POSITION
$ roslaunch pcl_object_detection demo.launch detection_mode:=4
# line_detection
$ roslaunch pcl_object_detection demo_line.launch
```
<div align="center">
    <img src="doc/img/demo_rqt_reconfigure.png" width="1080">
</div>

---

- [Topに戻る](#pcl-object-detection)
