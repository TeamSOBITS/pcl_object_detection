<a name="readme-top"></a>

[JP](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# PCL Object Detection

## 概要

- ROS 2 Jazzy向けの高性能かつコンポーネント指向な物体検出パッケージです。
- **Point Cloud Library (PCL)** を利用したルールベースの物体検出を行います。
- アーキテクチャ: **ROS 2 Lifecycle Nodes** を採用し、**ゼロコピー・プロセス間通信**を実現しています。
- モード: 床、机、棚、配置可能位置の検出、および2D-LiDARによる直線検出に対応しています。

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

正常に動作させるため、以下の環境を整えてください。

| System | Version |
| ------------- | ------------- |
| Ubuntu | 24.04 (Noble Numbat) |
| ROS | Jazzy Jalisco |

> [!NOTE]
> `Ubuntu` や `ROS` のインストール方法については、[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6) を参照してください。

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法
1. ROSワークスペースの `src` フォルダに移動します。
   ```bash
   $ cd ~/colcon_ws/src/
   ```

2. 本リポジトリをクローンします。
   ```bash
   $ git clone -b jazzy-devel https://github.com/TeamSOBITS/pcl_object_detection.git
   ```

3. 依存パッケージをインストールします。
   ```bash
   $ bash install.sh
   ```

4. パッケージをコンパイルします。
   ```bash
   $ cd ~/colcon_ws
   $ colcon build --symlink-install
   ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 実行・操作方法

本システムは、ゼロコピー・メモリ共有を実現するため、単一の `ComposableNodeContainer` として動作します。

コンテナを起動するコマンド:
```bash
ros2 launch pcl_object_detection pcl_object_detection.launch.py
```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### ライフサイクル管理

本パッケージは **ROS 2 Lifecycle Nodes** を使用しています。デフォルトでは、すべての検出ノードは `Unconfigured` 状態です。処理を開始するには、その状態を管理する必要があります。

#### ワークフローの例 (Table Detectionの場合)

1. **Configure** (メモリの確保とパラメータの読み込み):
   ```bash
   ros2 lifecycle set /pcl_object_detection/table_detection configure
   ```
2. **Activate** (データ購読と処理の開始):
   ```bash
   ros2 lifecycle set /pcl_object_detection/table_detection activate
   ```
3. **Deactivate** (処理の即時停止、CPU負荷ゼロ):
   ```bash
   ros2 lifecycle set /pcl_object_detection/table_detection deactivate
   ```

| モード | ノード名 | 機能 |
| --- | --- | --- |
| 1 | `table_detection` | 水平面上の物体を検出 |
| 2 | `floor_detection` | 床上の物体を検出 (脚のフィルタリング機能付き) |
| 3 | `shelf_detection` | 収納棚の中の物体を検出 |
| 4 | `placeable_detection`| 物体の配置可能な空きスペースを検出 |
| 5 | `line_detection` | 2D-LiDARを用いた直線の検出 |

※ すべてのパラメータは [config](./config/) ディレクトリ内で定義されており、ノードごとに個別に調整可能です。

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

- [Point Cloud Library](https://pointclouds.org/)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



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
