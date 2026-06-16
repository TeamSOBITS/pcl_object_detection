<a name="readme-top"></a>

[JP](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# PCL Object Detection

## Overview

- A high-performance, component-based object detection package for ROS 2 Jazzy.
- Utilizes the **Point Cloud Library (PCL)** for rule-based detection.
- Architecture: **ROS 2 Lifecycle Nodes** with **Zero-Copy Intra-Process Communication**.
- Modes: Floor, Table, Shelf, Placeable Positions, and 2D-LiDAR Line Detection.

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


<!-- Setup -->
## Setup

Here, we describe the setup process for this repository.

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


### Environment

Below are the system requirements for normal operation.

| System  | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |

> [!NOTE]
> If you need to install `Ubuntu` or `ROS`, please check our [SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6).

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


### Installation

1. Move to the `src` folder of your ROS workspace.
   ```bash
   $ cd ~/colcon_ws/src/
   ```

2. Clone this repository.
   ```bash
   $ git clone -b jazzy-devel https://github.com/TeamSOBITS/pcl_object_detection.git
   ```

3. Install the dependent packages.
   ```bash
   $ bash install.sh
   ```

4. Compile the package.
   ```bash
   $ cd ~/colcon_ws
   $ colcon build --symlink-install
   ```

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


## Usage
The system runs as a single `ComposableNodeContainer` to enable zero-copy memory sharing. 

Launch the container:
```bash
ros2 launch pcl_object_detection pcl_object_detection.launch.py
```

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


### Lifecycle Management

This package uses **ROS 2 Lifecycle Nodes**. By default, all detection workers are `Unconfigured`. You must manage their state to begin processing.

#### Workflow Example (Table Detection)

1. **Configure** (Allocates memory and loads parameters):
   ```bash
   ros2 lifecycle set /pcl_object_detection/table_detection configure
   ```
2. **Activate** (Starts data subscription and processing):
   ```bash
   ros2 lifecycle set /pcl_object_detection/table_detection activate
   ```
3. **Deactivate** (Stops processing immediately, 0% CPU overhead):
   ```bash
   ros2 lifecycle set /pcl_object_detection/table_detection deactivate
   ```

| Mode | Node Name | Function |
| --- | --- | --- |
| 1 | `table_detection` | Detect objects on horizontal surface |
| 2 | `floor_detection` | Detect objects on floor (includes leg filtering) |
| 3 | `shelf_detection` | Detect objects in storage bins |
| 4 | `placeable_detection`| Find empty space for object placement |
| 5 | `line_detection` | Detect lines from 2D LiDAR |

※ All parameters are defined in the [config](./config/) directory and can be tuned independently per node.

<p align="right">(<a href="#readme-top">Back to Top</a>)</p>


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
