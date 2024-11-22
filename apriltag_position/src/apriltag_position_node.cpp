#include <time.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <numeric>
// ros头文件
#include <ros/ros.h>


// topic 头文件
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>
#include <prometheus_msgs/DetectionInfo.h>

//apriltag的消息文件
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h> 


using namespace std;

ros::Publisher position_pub;
ros::Subscriber apriltag_subscriber;

prometheus_msgs::DetectionInfo pose_now;


// 偏移量 (根据你的坐标系和标签实际布局设置)
geometry_msgs::Vector3 offset_1;
geometry_msgs::Vector3 offset_2;
geometry_msgs::Vector3 offset_3;
geometry_msgs::Vector3 offset_4;



// 存放上一次的值
float last_x(0), last_y(0), last_z(0), last_yaw(0), last_az(0), last_ay(0), last_ax(0), last_qx(0), last_qy(0), last_qz(0), last_qw(0);

// 计算位姿平均值
geometry_msgs::Point averagePosition(const std::vector<geometry_msgs::Point>& positions)
{
    geometry_msgs::Point avg_pos;
    avg_pos.x = avg_pos.y = avg_pos.z = 0.0;
    
    for (const auto& pos : positions)
    {
        avg_pos.x += pos.x;
        avg_pos.y += pos.y;
        avg_pos.z += pos.z;
    }
    
    avg_pos.x /= positions.size();
    avg_pos.y /= positions.size();
    avg_pos.z /= positions.size();

    return avg_pos;
}


// 回调函数
void apriltagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // 打印检测到的标签数量
    ROS_INFO("Received AprilTag detections. Count: %lu", msg->detections.size());

    //如果detections为空，就使用上一次的值，如果不为空就重新计算
    if(msg->detections.size()!=0){
        std::vector<geometry_msgs::Point> positions;
        std::vector<geometry_msgs::Quaternion> orientations;

        // 获取检测到的标签
        for (size_t i = 0; i < msg->detections.size(); ++i)
        {
            const auto& detection = msg->detections[i];

            // 根据标签 ID 进行不同的处理
            geometry_msgs::Point position = detection.pose.pose.pose.position;
            geometry_msgs::Quaternion orientation = detection.pose.pose.pose.orientation;

            // 打印每个标签的原始位置信息
            // ROS_INFO("Tag ID: %d, Original Position: x=%.2f, y=%.2f, z=%.2f", 
            //          detection.id[0], position.x, position.y, position.z);

            // 处理角落标签的偏移量
            if (detection.id[0] == 1) {
                position.x += offset_1.x; position.y += offset_1.y; position.z += offset_1.z;
            }
            if (detection.id[0] == 2) {
                position.x += offset_2.x; position.y += offset_2.y; position.z += offset_2.z;
            }
            if (detection.id[0] == 3) {
                position.x += offset_3.x; position.y += offset_3.y; position.z += offset_3.z;
            }
            if (detection.id[0] == 4) {
                position.x += offset_4.x; position.y += offset_4.y; position.z += offset_4.z;
            }

            // 打印偏移后的位置信息
            ROS_INFO("Tag ID: %d, Adjusted Position: x=%.2f, y=%.2f, z=%.2f", 
                     detection.id[0], position.x, position.y, position.z);

            // 保存位姿数据
            positions.push_back(position);
            orientations.push_back(orientation);
        }

        // 如果有标签数据，则进行聚合
        if (!positions.empty())
        {
            // 计算平均位置
            geometry_msgs::Point avg_position = averagePosition(positions);

            // 对姿态（四元数）进行平均
            // 对四元数的平均处理需要谨慎，这里我们简单地取平均值（可以使用四元数插值方法来提高精度）
            geometry_msgs::Quaternion avg_orientation;
            avg_orientation.x = avg_orientation.y = avg_orientation.z = avg_orientation.w = 0.0;
            
            for (const auto& quat : orientations)
            {
                avg_orientation.x += quat.x;
                avg_orientation.y += quat.y;
                avg_orientation.z += quat.z;
                avg_orientation.w += quat.w;
            }
            
            avg_orientation.x /= orientations.size();
            avg_orientation.y /= orientations.size();
            avg_orientation.z /= orientations.size();
            avg_orientation.w /= orientations.size();
            
            // 设置聚合的位姿数据
            pose_now.position[0] = avg_position.x;
            pose_now.position[1] = avg_position.y;
            pose_now.position[2] = avg_position.z;

            // 将四元数的平均值作为目标姿态
            pose_now.attitude_q[0] = avg_orientation.x;
            pose_now.attitude_q[1] = avg_orientation.y;
            pose_now.attitude_q[2] = avg_orientation.z;
            pose_now.attitude_q[3] = avg_orientation.w;

            //欧拉角置为0
            pose_now.attitude[0]=0;
            pose_now.attitude[1]=0;
            pose_now.attitude[2]=0;

            //填充其他的值
            pose_now.sight_angle[0] = atan(avg_position.x / avg_position.z);
            pose_now.sight_angle[1] = atan(avg_position.y / avg_position.z);
            pose_now.yaw_error = 0;
            pose_now.header.stamp = ros::Time::now();
            pose_now.detected = true;
            pose_now.frame = 0;

            // 记录历史值
            last_x = pose_now.position[0];
            last_y = pose_now.position[1];
            last_z = pose_now.position[2];
            last_az = pose_now.attitude[0];
            last_ay = pose_now.attitude[1];
            last_ax = pose_now.attitude[2];
            last_qx = pose_now.attitude_q[0];
            last_qy = pose_now.attitude_q[1];
            last_qz = pose_now.attitude_q[2];
            last_qw = pose_now.attitude_q[3];
            last_yaw = pose_now.yaw_error;

            // 打印聚合后的平均位置信息
            ROS_INFO("Average Position: x=%.2f, y=%.2f, z=%.2f", 
                     avg_position.x, avg_position.y, avg_position.z);
            ROS_INFO("Average Orientation (Quaternion): x=%.2f, y=%.2f, z=%.2f, w=%.2f", 
                     avg_orientation.x, avg_orientation.y, avg_orientation.z, avg_orientation.w);
        }
    }else{
        pose_now.header.stamp = ros::Time::now();
        pose_now.detected = false;
        pose_now.frame = 0;
        pose_now.position[0] = last_x;
        pose_now.position[1] = last_y;
        pose_now.position[2] = last_z;
        pose_now.attitude[0] = last_az;
        pose_now.attitude[1] = last_ay;
        pose_now.attitude[2] = last_ax;
        pose_now.attitude_q[0] = last_qx;
        pose_now.attitude_q[1] = last_qy;
        pose_now.attitude_q[2] = last_qz;
        pose_now.attitude_q[3] = last_qw;
        pose_now.sight_angle[0] = atan(last_x / last_z);
        pose_now.sight_angle[1] = atan(last_y / last_z);
        pose_now.yaw_error = last_yaw;

        // 打印使用的历史值
        ROS_WARN("No tags detected. Using last known values: x=%.2f, y=%.2f, z=%.2f", 
                 last_x, last_y, last_z);
    }

    // 发布目标信息
    position_pub.publish(pose_now);
    ROS_INFO("Published pose_now message.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_position");
    ros::NodeHandle nh("~");

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

    int uav_id=1;
    position_pub = nh.advertise<prometheus_msgs::DetectionInfo>("/uav" + std::to_string(uav_id) + "/prometheus/object_detection/landpad_det", 10);


    // 接受Apriltag检测结果的话题
    apriltag_subscriber=nh.subscribe("/tag_detections",10,apriltagCallback);

    // 节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate loopRate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loopRate.sleep();
    }
}
