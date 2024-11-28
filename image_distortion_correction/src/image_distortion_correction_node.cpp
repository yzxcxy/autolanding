#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

cv::Mat K, D;  // 相机内参和畸变系数

//自定义的计算内参矩阵的函数
cv::Mat customGetOptimalNewCameraMatrix(
    const cv::Mat& K,    // 原始相机内参矩阵
    const cv::Mat& D,    // 畸变系数
    const cv::Size& image_size,  // 图像大小
    double alpha = 1.0   // alpha 参数：决定是否裁剪去畸变后的图像
)
{
    // 获取原始内参矩阵的焦距和光心位置
    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);

    // 畸变系数 k1, k2, p1, p2, k3
    double k1 = D.at<double>(0);
    double k2 = D.at<double>(1);
    double p1 = D.at<double>(2);
    double p2 = D.at<double>(3);
    double k3 = D.at<double>(4);

    // 假设 alpha 为 1，表示保留所有图像的有效区域（无裁剪）
    // 计算新的相机矩阵

    // 如果 alpha 为 1，我们不裁剪，只是更新 K
    double new_fx = fx * alpha;
    double new_fy = fy * alpha;
    double new_cx = cx;
    double new_cy = cy;

    // 创建新的相机矩阵
    cv::Mat new_K = (cv::Mat_<double>(3, 3) <<
        new_fx, 0.0, new_cx,
        0.0, new_fy, new_cy,
        0.0, 0.0, 1.0
    );

    // 返回新的相机矩阵
    return new_K;
}

// 回调函数：接收 CameraInfo 消息，更新内参和畸变系数
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{

    //直接在代码中定义 K 和 D
    // K = (cv::Mat_<double>(3, 3) << 673.99486, 0, 966.41707,
    //                                     0, 673.86971, 546.53779,
    //                                     0, 0, 1);

    // D = (cv::Mat_<double>(1, 5) << -0.004264, -0.016536, 0.001005, -0.000402, 0.000000);
    // K = (cv::Mat_<double>(3, 3) << 0, 0, 0,
    //                                     0, 0, 0,
    //                                     0, 0, 0);

    // D = (cv::Mat_<double>(1, 5) << 0.000000, 0.000000, 0.000000, 0.000000, 0.000000);
    // 从 CameraInfo 消息中提取相机内参和畸变系数
    // 手动将 msg->K 和 msg->D 中的数据提取并赋值给 cv::Mat避免值异常
    K = cv::Mat(3, 3, CV_64F);
    K.at<double>(0, 0) = msg->K[0];
    K.at<double>(0, 1) = msg->K[1];
    K.at<double>(0, 2) = msg->K[2];
    K.at<double>(1, 0) = msg->K[3];
    K.at<double>(1, 1) = msg->K[4];
    K.at<double>(1, 2) = msg->K[5];
    K.at<double>(2, 0) = msg->K[6];
    K.at<double>(2, 1) = msg->K[7];
    K.at<double>(2, 2) = msg->K[8];

    D = cv::Mat(1, 5, CV_64F);
    D.at<double>(0) = msg->D[0];
    D.at<double>(1) = msg->D[1];
    D.at<double>(2) = msg->D[2];
    D.at<double>(3) = msg->D[3];
    D.at<double>(4) = msg->D[4];

    ROS_INFO("Received camera info");
}

// 回调函数：接收 Image 消息，进行图像畸变矫正
void imageCallback(const sensor_msgs::Image::ConstPtr& msg, image_transport::Publisher& pub)
{
    // ROS_INFO("Received iamge info");
    //检查相机内参和畸变系数是否有效
    if (K.empty() || D.empty())
    {
        ROS_WARN("Camera info not received yet, skipping image processing");
        return;
    }

    ROS_INFO_STREAM("k matrix:"<< K);
    ROS_INFO_STREAM("D matrix:"<< D);

    // if(msg==NULL){
    //     ROS_INFO("image is  NULL");    
    //     return;
    // }else{
    //     ROS_INFO("image is not NULL");
    // }

    // ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());

    // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 进行畸变矫正
    cv::Mat undistorted_image;
    int width = cv_ptr->image.cols;
    int height = cv_ptr->image.rows;

    // 计算新的相机矩阵（去畸变后的内参矩阵）
    //cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(K, D, cv::Size(width, height), 1, cv::Size(width, height), 0);

    // 手动定义的去畸变后的内参矩阵（假设没有大变化）
    // cv::Mat new_camera_matrix = (cv::Mat_<double>(3, 3) << 
    //     670.0, 0.0, 960.0,   // 焦距 fx, cx
    //     0.0, 670.0, 540.0,   // 焦距 fy, cy
    //     0.0, 0.0, 1.0        // 齐次坐标
    // );

    // 写死内参矩阵后结果成功

    //调用自定义的内参矩阵计算函数
    cv::Mat new_camera_matrix=customGetOptimalNewCameraMatrix(K,D,cv::Size(width, height), 1);
    ROS_INFO_STREAM("new_camera_matrix"<<new_camera_matrix);

    // 进行畸变矫正
    cv::undistort(cv_ptr->image, undistorted_image, K, D, new_camera_matrix);
 

    // 将矫正后的图像转换回 ROS 图像消息并发布
    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, undistorted_image).toImageMsg();
    pub.publish(output_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_distortion_correction_node");
    ros::NodeHandle nh;

    // 创建图像发布器
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/usb_cam/image_rect", 1);

    // 订阅相机信息话题
    ros::Subscriber sub_info = nh.subscribe("/usb_cam/camera_info", 1, cameraInfoCallback);

    // 订阅原始图像话题
    image_transport::Subscriber sub_image = it.subscribe("/usb_cam/image_raw", 1, boost::bind(imageCallback, _1, pub));

    ROS_INFO("Image Distortion Correction Node Started");


    // 循环执行
    ros::spin();

    return 0;
}
