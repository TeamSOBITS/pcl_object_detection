<a name="readme-top"></a>

[JP](README.md) | [EN](README.en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

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
    <li><a href="#execution-and-operation">Execution and Operation</a></li>
    <li><a href="#milestones">Milestones</a></li>
    <li><a href="#change-log">Change Log</a></li>
    <!-- <li><a href="#contributing">Contributing</a></li> -->
    <!-- <li><a href="#license">License</a></li> -->
    <li><a href="#references">References</a></li>
  </ol>
</details>



<!-- Overview of the Repository -->
## Overview

<!-- [![Product Name Screen Shot][product-screenshot]](https://example.com) -->

- Rule-based object detection package using the Point Cloud Library
- Detects lines from Lidar information

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>



<!-- Setup -->
## Setup

Here, we describe the setup process for this repository.

### Environment
Below are the system requirements for normal operation.
| System  | Version |
| ------------- | ------------- |
| Ubuntu | 20.04 (Focal Fossa) |
| ROS | Noetic Ninjemys |
| C++ | 3.10.2~ |

### Installation
1. Move to the `src` folder of your ROS workspace.
   ```sh
   $ roscd
   # roscd is equivalent to "cd ~/catkin_ws/" depending on the configuration
   $ cd src/
   ```
2. Clone this repository.
   ```sh
   $ git clone https://github.com/TeamSOBITS/pcl_object_detection.git
   ```
3. Compile the package.
   ```sh
   $ roscd
   $ catkin_make
   ```

<!-- Execution and Operation -->
## Execution and Operation

### [point_cloud_object_detection.launch](launch/point_cloud_object_detection.launch)
<!-- It would be helpful to have a demo execution method or screenshots -->
- Performs object detection on the table, floor, and shelf, and detects placement positions
- Detection positions are output through topic communication and TF
- Each detection mode can be changed through service communication
  - mode0: OFF
  - mode1: table mode (adjustable height via yaml file)
  - mode2: floor mode (adjustable height via yaml file)
  - mode3: shelf mode (adjustable height via yaml file)
  - mode4: placeable detection (placement position detection)

- Details can be found [here](doc/md/point_cloud_object_detection.md)

```bash
# with rviz
roslaunch point_cloud_object_detection point_cloud_object_detection.launch
# without rviz
roslaunch point_cloud_object_detection point_cloud_object_detection.launch rviz:=false
# parameter adjustment via rqt_reconfigure
roslaunch point_cloud_object_detection point_cloud_object_detection.launch rqt_reconfigure:=true
```
※ While rqt_reconfigure allows dynamic parameter changes, the parameter file is not overwritten.
※ After adjusting parameters with rqt_reconfigure, manually modify the parameter file.

<div align="center">
    <img src="doc/img/table.png" width="1080">
    <!-- <img src="doc/img/floor.png" width="1080"> -->
    <!-- <img src="doc/img/placeable.png" width="1080"> -->
</div>

- [Back to top](#pcl-object-detection)

## [line_detection.launch](launch/line_detection.launch)
- Detects lines from point clouds obtained from a 2D-LiDAR sensor
Details can be found [here](doc/md/line_detection.md)

```bash
# with rviz
roslaunch point_cloud_object_detection line_detection_param.launch
# without rviz
roslaunch point_cloud_object_detection line_detection_param.launch rviz:=false
# parameter adjustment via rqt_reconfigure
roslaunch point_cloud_object_detection line_detection_param.launch rqt_reconfigure:=true
```
※ While rqt_reconfigure allows dynamic parameter changes, the parameter file is not overwritten.
※ After adjusting parameters with rqt_reconfigure, manually modify the parameter file.

<div align="center">
    <img src="doc/img/line_detection.png" width="1080">
</div>

- [Back to Top](#pcl-object-detection)

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

<!-- Milestones -->
## Milestones
<!-- Milestones -->
## Milestones

- [ ] Comprehensive documentation
- [ ] Open Source Software (OSS) release
  - [ ] Migration from tf to tf2
  - [ ] Update from custom message types to public messages

To check current bugs or request new features, please visit the [Issue page](https://github.com/github_username/repo_name/issues).

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>

<!-- Change Log -->
## Change Log
Please refer to the rst.file for the change log.

<!-- References -->
## References

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