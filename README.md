

# 项目结构

```shell
.
├── apriltag_detection
│   ├── apriltag.rviz
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── ost.yaml
│   │   ├── settings.yaml
│   │   └── tags.yaml
│   ├── launch
│   │   ├── camera.launch
│   │   ├── camerawithoutimageproc.launch
│   │   ├── continuous_detection.launch
│   │   └── image_proc.launch
│   └── package.xml
├── apriltag_position
│   ├── CMakeLists.txt
│   ├── config
│   ├── include
│   │   └── apriltag_position
│   ├── launch
│   │   ├── apriltag_position.launch
│   │   └── start_all.launch
│   ├── msg
│   │   └── DetectionInfo.msg
│   ├── package.xml
│   └── src
│       └── apriltag_position_node.cpp
├── att_control_autonomous_landing
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── autonomous_landing
│   │   ├── mission_utils.h
│   │   └── printf_utils.h
│   ├── package.xml
│   └── src
│       ├── att_control_autonomous_landing.cpp
│       └── autonomous_landing.cpp
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── common
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── geometry_utils.h
│   │   ├── math_utils.h
│   │   └── printf_utils.h
│   ├── prometheus_msgs
│   │   ├── action
│   │   │   └── CheckForObjects.action
│   │   ├── CMakeLists.txt
│   │   ├── msg
│   │   │   ├── ArucoInfo.msg
│   │   │   ├── BoundingBoxes.msg
│   │   │   ├── BoundingBox.msg
│   │   │   ├── Bspline.msg
│   │   │   ├── DetectionInfo.msg
│   │   │   ├── DetectionInfoSub.msg
│   │   │   ├── FormationAssign.msg
│   │   │   ├── GimbalControl.msg
│   │   │   ├── GimbalState.msg
│   │   │   ├── GlobalAruco.msg
│   │   │   ├── GPSData.msg
│   │   │   ├── LinktrackNode2.msg
│   │   │   ├── LinktrackNodeframe2.msg
│   │   │   ├── MultiArucoInfo.msg
│   │   │   ├── MultiBsplines.msg
│   │   │   ├── MultiDetectionInfo.msg
│   │   │   ├── MultiDetectionInfoSub.msg
│   │   │   ├── MultiUAVState.msg
│   │   │   ├── OffsetPose.msg
│   │   │   ├── RheaCommunication.msg
│   │   │   ├── RheaGPS.msg
│   │   │   ├── RheaState.msg
│   │   │   ├── SwarmCommand.msg
│   │   │   ├── TextInfo.msg
│   │   │   ├── UAVCommand.msg
│   │   │   ├── UAVControlState.msg
│   │   │   ├── UAVSetup.msg
│   │   │   ├── UAVState.msg
│   │   │   ├── VisionDiff.msg
│   │   │   └── WindowPosition.msg
│   │   └── package.xml
│   └── quadrotor_msgs
│       ├── CMakeLists.txt
│       ├── msg
│       │   ├── AuxCommand.msg
│       │   ├── Corrections.msg
│       │   ├── Gains.msg
│       │   ├── LQRTrajectory.msg
│       │   ├── Odometry.msg
│       │   ├── OutputData.msg
│       │   ├── PolynomialTrajectory.msg
│       │   ├── PositionCommand.msg
│       │   ├── PPROutputData.msg
│       │   ├── Px4ctrlDebug.msg
│       │   ├── Serial.msg
│       │   ├── SO3Command.msg
│       │   ├── StatusData.msg
│       │   └── TRPYCommand.msg
│       └── package.xml
├── image-20241122142559152.png
├── image_distortion_correction
│   ├── CMakeLists.txt
│   ├── include
│   │   └── image_distortion_correction
│   ├── launch
│   │   └── image_distortion_correction.launch
│   ├── package.xml
│   └── src
│       ├── image_distortion_correction_node.cpp
│       └── rev.jpg
└── README.md

```

# Apriltag功能如何使用
注意git下来的是一个叫apriltag_transformer的文件夹，但是这个使用的时候子文件夹是ros工作空间中的src中的内容，使用需要复制出来。

## 安装apriltag_ros功能包

```she
sudo apt install ros-$ROS_DISTRO-apriltag-ros
```



## 启动摄像头

### 安装摄像头需要的包

```shell
sudo apt install ros-noetic-usb-cam
```



### 启动

如果是英伟达的nx平台，请跳过下面这一步，跳到注意的地方。

这里使用的是项目里面的，如果自己的项目有了就不需要启动这个了。

```shell	
roslaunch apriltag_detection camera.launch
```



## 注意

因为image_proc在英伟达的nx平台出现了问题，比如/usb_cam/image_raw接收不到，或者出现段错误，为了临时解决这个问题，直接使用opencv写一个包替换掉原来的image_proc(见image_distortion_correction文件夹)



临时的启动方案如下：

启动相机：

```shell
roslaunch apriltag_detection camerawithoutimageproc.launch
```



启动image_distortion_correction

```shell
roslaunch image_distortion_correction image_distortion_correction.launch
```



## 启动apriltag检测

```shell
roslaunch apriltag_detection continuous_detection.launch 
```

这个结点主要封装apriltag_ros

![image-20241122142559152](image-20241122142559152.png)



主要修改了参数文件载入路径和相机名称，可以根据需要来灵活修改

```xml
<launch>
	<arg name="launch_prefix" default="" /> <!-- set to value="gdbserver localhost:10000" for remote debugging -->
	<arg name="node_namespace" default="apriltag_ros_continuous_node" />
	<arg name="camera_name" default="/usb_cam" />
	<arg name="image_topic" default="image_rect" />

	<!-- Set parameters -->
	<rosparam command="load" file="$(find apriltag_detection)/config/settings.yaml" ns="$(arg node_namespace)" />
	<rosparam command="load" file="$(find apriltag_detection)/config/tags.yaml" ns="$(arg node_namespace)" />

	<node pkg="apriltag_ros" type="apriltag_ros_continuous_node" name="$(arg node_namespace)" clear_params="true" output="screen" launch-prefix="$(arg launch_prefix)" >
		<!-- Remap topics from those used in code to those on the ROS network -->
		<remap from="image_rect" to="$(arg camera_name)/$(arg image_topic)" />
		<remap from="camera_info" to="$(arg camera_name)/camera_info" />

		<param name="publish_tag_detections_image" type="bool" value="true" />      <!-- default: false -->
	</node>
</launch>

```



这里需要根据实际使用的相机和具体使用的二维码，替换config文件夹里面的tags.yaml和ost.yaml文件

## 启动话题转化结点


```shell
roslaunch apriltag_position apriltag_position.launch
```



这个代码文件为apriltag_position_node.cpp文件，代码的基本逻辑就是订阅apriltag_ros_continuous_node的/tag_detecion信息，处理五个标签的相对位置信息，然后偏移到中间的标签，进行平均，如果修改标签的距离，需要修改代码中的以下信息，并重新进行编译：

```cpp
    //偏移量初始化
    offset_1.x = 0.06;
    offset_1.y = 0.06;
    offset_1.z = 0.0;

    offset_2.x = -0.06;
    offset_2.y = 0.06;
    offset_2.z = 0.0;

    offset_3.x = 0.06;
    offset_3.y = -0.06;
    offset_3.z = 0.0;

    offset_4.x = -0.06;
    offset_4.y = -0.06;
    offset_4.z = 0.0;
```



进行聚合之后，就会发布"/uav1/prometheus/object_detection/landpad_det"话题，这个是原项目平台（  https://github.com/haolin11/myautoland_ws  ）的src文件中的tutorial_demo/advanced/autonomous_landing/src/autonomous_landing.cpp中订阅的，注意，还需要做一个工作，就是原来项目的src/object_detection/cpp_nodes的markers_landpad_det.cpp文件中的发布"/uav1/prometheus/object_detection/landpad_det"的代码给注释掉。



# 相对位置转化mavros_msgs/AttitudeTarget

因为时间关系，这个代码中的很多内容参数就不能调了，先使用原来的那一版基于速度的控制

这里主要修改https://github.com/haolin11/myautoland_ws里的src/tutorial_demo/advanced/autonomous_landing/src/autonomous_landing.cpp文件，添加的功能函数为computeAttitudeTarget。

computeAttitudeTarget是通过计算无人机相对于目标位置的误差，并结合比例-积分-微分控制（PID），生成一个包含姿态四元数和推力值的 `AttitudeTarget` 消息，以便调整无人机的姿态和推力，最终实现位置控制。

# 如何整合到原来的基于peomethus的项目代码中

注意这里是不需要基于四元数和推力的控制的，所以需要删除一些代码，首先删除autonomous_landing和common文件夹，然后在原项目代码中，在src文件夹中，创建一个apriltag_att_control文件夹，将这些删除后的代码复制进去。



# 参考链接

1. https://bingda.yuque.com/staff-hckvzc/ai5gkn/easncqctnt0dilp6
2. https://april.eecs.umich.edu/software/apriltag.html
3. https://wiki.ros.org/apriltag_ros
4. https://bingda.yuque.com/staff-hckvzc/ai5gkn/nu8kcf



