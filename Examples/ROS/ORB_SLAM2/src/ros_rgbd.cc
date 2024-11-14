/**
 * @file ros_rgbd.cc
 * @author guoqing (1337841346@qq.com)
 * @brief ORB RGB-D 输入的ROS节点实现
 * @version 0.1
 * @date 2019-08-06
 *
 * @copyright Copyright (c) 2019
 *
 */

/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

// #include"../../../include/System.h"
#include "System.h"

// new1 文件流
#include <fstream>

using namespace std;

class ImageGrabber // 图像采集类
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM2::System *mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD"); // 初始化启动ros
    ros::start();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // 创建ORB_SLAM2系统对象
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    // new1 加载已经保存的地图
    // new1 加载地图 or 创建新地图？
    cout << "加载地图中..." << endl;
    SLAM.LoadMap("/home/ghr/orbslam_ws/src/ORB_SLAM2//Examples/ROS/ORB_SLAM2/map/map_4.20_2.bin"); // Load the map

    // 创建 ImagGrabb 对象 igb
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    // todo 作用：读取 rgb 和深度信息👇
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/usb_cam/image_raw", 1);

    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/usb_cam/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // TODO 以下均为 slam 系统操作，加载、保存、关闭👇

    

    // new0 是否保存地图？
    // TODO （调用 SaveMap 函数的位置，可以在 SLAM.Shutdown(); 之前，也可以在之后。
    cout << endl;
    char IsSaveMap;
    cerr << "Save the Map?(Y/N)" << endl;
    cin >> IsSaveMap;
    if (IsSaveMap == 'Y' || IsSaveMap == 'y')
    {
        SLAM.SaveMap("/home/ghr/orbslam_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/map/map.bin");
    }

    // new0 是否保存关键帧的轨迹？
    char IsSaveKeyFrameTrajectoryTUM;
    cerr << "Save the KeyFrameTrajectoryTUM?(Y/N)" << endl;
    cin >> IsSaveKeyFrameTrajectoryTUM;
    if (IsSaveKeyFrameTrajectoryTUM == 'Y' || IsSaveKeyFrameTrajectoryTUM == 'y')
    {
        SLAM.SaveKeyFrameTrajectoryTUM("/home/ghr/orbslam_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/KeyFrameTrajectory/KeyFrameTrajectory.txt");
    }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB; // RGB信息
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD; // 深度信息
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
}
