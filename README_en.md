<a name="readme-top"></a>

[JP](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# PCL Object Detection

<!-- Table of Contents -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#overview">Overview</a>
    </li>
    <li>
      <a href="#setup">Setup</a>
      <ul>
        <li><a href="#environment">Environment</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#milestones">Milestones</a></li>
    <li><a href="#change-log">Change Log</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>


<!-- Overview of the Repository -->
## Overview

- Rule-based object detection package using the Point Cloud Library.
- Detects lines from Lidar information.

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


<!-- Setup -->
## Setup

Here, we describe the setup process for this repository.

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


### Environment

Below are the system requirements for normal operation.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


### Installation
1. Move to the `src` folder of your ROS workspace.
   ```bash
   $ roscd
   # roscd is equivalent to "cd ~/catkin_ws/" depending on the configuration
   $ cd src/
   ```

2. Clone this repository.
   ```bash
   $ git clone https://github.com/TeamSOBITS/pcl_object_detection.git
   ```

3. Install the dependent packages.
   ```bash
   $ bash install.sh
   ```

4. Compile the package.
   ```bash
   $ roscd
   $ catkin_make
   ```

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


<!-- Usage -->
## Usage

### [point_cloud_object_detection.launch](launch/point_cloud_object_detection.launch)
<!-- It would be helpful to have a demo execution method or screenshots -->
- Performs object detection on the table, floor, and shelf, and detects placement positions
- Detection positions are output through topic communication and TF
- Each detection mode can be changed through service communication

| Mode | Function |
| --- | --- |
| 0 | OFF |
| 1 | table mode - [Parameters](param/object_detection_table_param.yaml)  |
| 2 | floor mode - [Parameters](param/object_detection_floor_param.yaml) |
| 3 | shelf mode - [Parameters](param/object_detection_shelf_param.yaml) |
| 4 | placeble mode - [Parameters](param/placeable_postion_detection_param.yaml) |

- Details can be found [here](doc/md/point_cloud_object_detection.md).

```bash
# with rviz
$ roslaunch point_cloud_object_detection point_cloud_object_detection.launch
# without rviz
$ roslaunch point_cloud_object_detection point_cloud_object_detection.launch rviz:=false
# parameter adjustment via rqt_reconfigure
$ roslaunch point_cloud_object_detection point_cloud_object_detection.launch rqt_reconfigure:=true
```

> [!NOTE]
> While rqt_reconfigure allows dynamic parameter changes, the parameter file is not overwritten. Therefore, modify manually the parameter file.

<div align="center">
    <img src="doc/img/table.png" width="1080">
    <!-- <img src="doc/img/floor.png" width="1080"> -->
    <!-- <img src="doc/img/placeable.png" width="1080"> -->
</div>

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


### [line_detection.launch](launch/line_detection.launch)

- Detects lines from point clouds obtained from a 2D-LiDAR sensor
- Details can be found [here](doc/md/line_detection.md).

```bash
# with rviz
$ roslaunch point_cloud_object_detection line_detection_param.launch
# without rviz
$ roslaunch point_cloud_object_detection line_detection_param.launch rviz:=false
# parameter adjustment via rqt_reconfigure
$ roslaunch point_cloud_object_detection line_detection_param.launch rqt_reconfigure:=true
```

> [!NOTE]
> While rqt_reconfigure allows dynamic parameter changes, the parameter file is not overwritten. Therefore, modify manually the parameter file.

<div align="center">
    <img src="doc/img/line_detection.png" width="1080">
</div>

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


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

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


<!-- Milestones -->
## Milestones

- [x] Comprehensive documentation
- [x] Open Source Software (OSS) release
  - [x] Migration from tf to tf2
  - [x] Update from custom message types to public messages

To check current bugs or request new features, please visit the [Issue page][issues-url].

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


<!-- Change Log -->
## Change Log

Please refer to the [CHANGELOG.rst](CHANGELOG.rst) for the complete change log.

<!-- References -->
## References

- [Point Cloud Library](https://pointclouds.org/)

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/pcl_object_detection.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/pcl_object_detection/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/pcl_object_detection.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/pcl_object_detection/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/pcl_object_detection.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/pcl_object_detection/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/pcl_object_detection.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/pcl_object_detection/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/pcl_object_detection.svg?style=for-the-badge
[license-url]: LICENSE