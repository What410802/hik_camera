<font face="STCAIYUN">[English](#en) | [中文](#zh)</font>

<a id="en"></a>

# HikVision Camera Package in ROS2 Humble

## Purpose

This package provides the functionality of using Hikvision cameras in ROS2 Humble. This includes reading and setting exposure time, gain, frame rate, pixel format, etc. from the ROS2 parameter system, as well as reading sequence numbers, frame rates, and other parameters from the camera and publishing them to the parameter system, and the most basic function of obtaining images from the camera and publishing them to the '/image_raw' topic.

## Usage

Prerequisite:
- Ubuntu 22.04, with the latest gcc/g++and make have been installed.

Place the root directory of the package `hik_comamera` in your ROS2 workspace, change to the workspace directory, and (change execution permissions and) run `hik_comamera/build.zsh` and `source hik_comamera/launch.zsh` in sequence to build the workspace and start the node.

## Code description/explanation

The definition part of `HikCameraNode` contains:
- Functional `#define`s, used to output logs. `#undef`s has been done orderly at the end of the node definition.
- The constructor and destructor of `public` attribute.
- Member variables of `protected` attribute.
	- Small variables: including MVS SDK-oriented and ROS2-oriented variables, some of which are cache variables (can also be implemented with `static` at where they are used).
	- Auxiliary functions of some of these small variables.
	- Several lookup tables: implemented with `std::map`. They are used to elegantly communicate the parameter names and setting methods between the ROS2 parameter side and the SDK camera side, and act as a list for traversal. See the note at the definition for details.
	- Ancillary function of lookup table: used to traverse these lookup tables conveniently.
- Member functions of `protected` attribute.
	- Initialization functions encapsulated by each layer, anti-initialization function encapsulated by each layer, and connection check function.
	- `this` management part: responsible for the initialization ~~and de-initialization~~ (there is no such requirement at present) of the `HikCameraNode`.
	- Image transmission section. The core functionality.
		- Since the pixel format has been changed through the SDK when the camera starts (including the initial startup and restart when changing some parameters), it is unnecessary and not recommended to change the pixel format here through `MV_CC_ConvertPixelTypeEx`.
		- The frame fetching function `MV_CC_GetOneFrameTimeout`, which is managed by the user according to the official document, meeting the requirement of loading frames into the member `data` of the message struct.
		- Among this function, timing functionalities are mixed within, which can output the time consumption of each internal functionality at runtime by opening the `TIMING_ON` macro at the beginning of the code file.


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

将包的根目录`hik_camera`放置到ROS2工作区中，切换到工作区目录下，（增加执行权限并）依次运行`src/hik_camera/build.zsh`,`source src/hik_camera/launch.zsh`以构建工作区并启动节点。

## 代码说明

`HikCameraNode`的定义部分，代码中依次包含：
- 功能性`#define`，用于输出日志。已在节点末尾按序`#undef`。
- `public`属性的节点构造函数和析构函数。
- `protected`属性的成员变量。
	- 小型变量：包括面向MVS SDK的和面向ROS2的，其中有些是缓存变量（也可在使用处用`static`实现）。
	- 某些小型变量的附属功能函数。
	- 几个查找表：用`std::map`实现，用于优雅地沟通ROS2参数端与SDK相机端的参数名称和设置方法，同时起到列表的作用，以便遍历。详见定义处注释。
	- 查找表的附属功能函数：用于方便地遍历这些查找表。
- `protected`属性的成员函数。
	- 各层封装的初始化函数、各层封装的反初始化函数、连接检查函数。
	- `this`管理部分，负责管理`HikCameraNode`的初始化~~和反初始化~~（暂无此需求）。
	- 图像发送部分。核心功能。
		- 由于已在相机启动（包括初次启动和更改某些参数时的重新启动）时通过SDK更改了像素格式，故无需且不应通过`MV_CC_ConvertPixelTypeEx`在此处更改像素格式。
		- 采用官方文档中说的由用户自行管理的取帧函数`MV_CC_GetOneFrameTimeout`，符合将帧加载到消息的`data`成员中的需求。
		- 其中穿插计时功能，通过打开代码文件开头的`TIMING_ON`宏，可在运行时输出每次调用内部各功能的时间消耗。

## Bibliography
- [Enable Multicast for `lo` in order to pass the test case on the website tutorial of ROS2](https://autowarefoundation.github.io/autoware-documentation/main/installation/additional-settings-for-developers/network-configuration/enable-multicast-for-lo/)


## Problems encountered and reflections

- Decided not to use try...catch to handle errors. https://learn.microsoft.com/en-us/cpp/cpp/errors-and-exception-handling-modern-cpp?view=msvc-170