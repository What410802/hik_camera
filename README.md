<font face="STCAIYUN">[English](#en) | [中文](#zh)</font>

<a id="en"></a>

# HIKVISION Camera Package in ROS2

## Purpose

This package provides the functionality of using Hikvision cameras in ROS2 Humble. This includes reading and setting exposure time, gain, frame rate, pixel format, etc. from the ROS2 parameter system, as well as reading sequence numbers, frame rates, and other parameters from the camera and publishing them to the parameter system, and the most basic function of obtaining images from the camera and publishing them to the '/image_raw' topic.

## Use

Prerequisite:
- Ubuntu 22.04, with the latest gcc/g++and make have been installed.

Place the root directory of the package `hik_comamera` in your ROS2 workspace, change to the workspace directory, and (change execution permissions and) run `hik_comamera/build.zsh` and `source hik_comamera/launch.zsh` in sequence to build the workspace and start the node.

<!-- ## Improvement made in building process -->
<!-- - Removed HIKVISION SDK package from the requirement list and put the SDK include files and libs inside the project, in order to improve the easiness of installing the dependencies. -->

---

<a id="zh"></a>

# 海康威视相机ROS2 Humble包

## 目的

本包提供了在ROS2 Humble中使用海康相机的功能。包括从ROS2参数系统中读取并设置曝光时间 (Exposure Time)、增益 (Gain)、帧率 (Frame Rate)、图像格式 (Pixel Format)等，以及从相机读取序列号、帧率等参数并发布到参数系统，和最基本的从相机获取图片并发布到`/image_raw`话题的功能。

## 使用

前提条件：
- Ubuntu 22.04，已安装最新gcc/g++和make。

将包的根目录`hik_camera`放置到ROS2工作区中，切换到工作区目录下，（增加执行权限并）依次运行`hik_camera/build.zsh`,`source hik_camera/launch.zsh`以构建工作区并启动节点。

## Bibliography
- [Enable Multicast for `lo` in order to pass the test case on the website tutorial of ROS2](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/enable-multicast-for-lo/)


## Problems encountered and reflections

- Decided not to use try...catch to handle errors. https://learn.microsoft.com/en-us/cpp/cpp/errors-and-exception-handling-modern-cpp?view=msvc-170