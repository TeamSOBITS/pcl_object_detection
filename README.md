<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# PCL Object Detection

<!-- 目次 -->
<details>
  <summary>目次</summary>
  <ol>
    <li>
      <a href="#概要">概要</a>
    </li>
    <li>
      <a href="#セットアップ">セットアップ</a>
      <ul>
        <li><a href="#環境条件">環境条件</a></li>
        <li><a href="#インストール方法">インストール方法</a></li>
      </ul>
    </li>
    <li><a href="#実行・操作方法">実行・操作方法</a></li>
    <li><a href="#マイルストーン">マイルストーン</a></li>
    <li><a href="#変更履歴">変更履歴</a></li>
    <li><a href="#参考文献">参考文献</a></li>
  </ol>
</details>


<!-- リポジトリの概要 -->
## 概要

- Point Cloud Libraryを用いたルールベース物体検出パッケージ．
- Lidar情報から直線を検出．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

正常動作のため，以下の必要な環境を整えてください．

| System | Version |
| --- | --- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS    | Noetic Ninjemys |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
  ```bash
  $ roscd
  # roscdは設定によって"cd ~/catkin_ws/"のコマンドと同様
  $ cd src/
  ```

2. 本リポジトリをcloneします．
  ```bash
  $ git clone https://github.com/TeamSOBITS/pcl_object_detection.git
  ```

3. 依存パッケージをインストールします．
  ```bash
  $ bash install.sh
  ```

4. パッケージをコンパイルします．
  ```bash
  $ roscd
  $ catkin_make
  ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 実行・操作方法 -->
## 実行・操作方法

### [point_cloud_object_detection.launch](launch/point_cloud_object_detection.launch)

- 机、床、棚上の物体検出と配置位置の検出をします．
- 検出位置はトピック通信とTFで出力されます．
- 各検出モードはサービス通信によって変更可能．

| モード | 機能 |
| --- | --- |
| 0 | OFF |
| 1 | table mode - [パラメータ](param/object_detection_table_param.yaml)  |
| 2 | floor mode - [パラメータ](param/object_detection_floor_param.yaml) |
| 3 | shelf mode - [パラメータ](param/object_detection_shelf_param.yaml) |
| 4 | placeble mode - [パラメータ](param/placeable_postion_detection_param.yaml) |

- 詳細は[こちら](doc/md/point_cloud_object_detection.md)．

```bash
# rvizあり
$ roslaunch point_cloud_object_detection point_cloud_object_detection.launch
# rvizなし
$ roslaunch point_cloud_object_detection point_cloud_object_detection.launch rviz:=false
# rqt_reconfigureによるパラメータ調整
$ roslaunch point_cloud_object_detection point_cloud_object_detection.launch rqt_reconfigure:=true
```

> [!NOTE]
> rqt_reconfigureはパラメータを動的に変更できるが，保存はできませんので，手打ちで更新してください．

<div align="center">
    <img src="doc/img/table.png" width="1080">
    <!-- <img src="doc/img/floor.png" width="1080"> -->
    <!-- <img src="doc/img/placeable.png" width="1080"> -->
</div>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### [line_detection.launch](launch/line_detection.launch)

- 2D-LiDARセンサから得た点群から直線を検出します．
- 詳細は[こちら](doc/md/line_detection.md)．

```bash
# rvizあり
$ roslaunch point_cloud_object_detection line_detection_param.launch
# rvizなし
$ roslaunch point_cloud_object_detection line_detection_param.launch rviz:=false
# rqt_reconfigureによるパラメータ調整
$ roslaunch point_cloud_object_detection line_detection_param.launch rqt_reconfigure:=true
```

> [!NOTE]
> rqt_reconfigureはパラメータを動的に変更できるが，保存はできませんので，手打ちで更新してください．

<div align="center">
    <img src="doc/img/line_detection.png" width="1080">
</div>

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### [demo.launch](launch/demo/demo.launch)

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

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- マイルストーン -->
## マイルストーン

- [x] ドキュメントの充実
- [x] OSS化
  - [x] tfからtf2への移行
  - [x] 独自のメッセージ型から公開メッセージへ更新

現時点のバッグや新規機能の依頼を確認するために[Issueページ][issues-url] をご覧ください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 変更履歴 -->
## 変更履歴

変更履歴は[CHANGELOG.rst](CHANGELOG.rst)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

- [Point Cloud Library](https://pointclouds.org/)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/ar_track_alvar.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/ar_track_alvar/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/ar_track_alvar.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/ar_track_alvar/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/ar_track_alvar.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/ar_track_alvar/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/ar_track_alvar.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/ar_track_alvar/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/ar_track_alvar.svg?style=for-the-badge
[license-url]: LICENSE