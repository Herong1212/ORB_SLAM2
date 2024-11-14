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

/**
 * @file  Converter.h
 * @author guoqing (1337841346@qq.com)
 * @brief 提供了一系列的常见转换。\n
 * orb 中以 cv::Mat 为基本存储结构，到 g2o 和 Eigen 需要一个转换。
 * 这些转换都很简单，整个文件可以单独从 orbslam 里抽出来而不影响其他功能.
 * @version 0.1
 * @date 2019-01-03
 */

#ifndef CONVERTER_H
#define CONVERTER_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

/**
 * @brief ORB-SLAM2 自定义的命名空间。
 * @details 该命名空间中包含了所有的 ORB-SLAM2 的组件。
 *
 */
namespace ORB_SLAM2
{

    /**
     * @brief 实现了 ORB-SLAM2 中的一些常用的转换。
     * @details 注意这是一个完全的静态类，没有成员变量，所有的成员函数均为静态的。
     */
    class Converter
    {
    public:
        /**
         * @brief 描述子矩阵到单行的描述子向量的转换.
         * @details cv::Mat -> std::vector<cv::Mat> \n
         * 转换后的结果就是把 cv::Mat 的每一行直接串联起来。
         *
         * @param[in] Descriptors 待转换的描述子
         * @return std::vector<cv::Mat> 转换结果
         * @note  应当注意，这里的描述子矩阵是多个单行的 cv::Mat 组成的。
         */
        static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

        /**
         * @name toSE3Quat
         * @details 将不同格式存储的位姿统一转换成为 g2o::SE3Quat 格式存储
         * @{
         */

        /**
         * @brief 将以 cv::Mat 格式存储的位姿转换成为 g2o::SE3Quat 类型
         *
         * @param[in] 以 cv::Mat 格式存储的位姿
         * @return g2o::SE3Quat 将以 g2o::SE3Quat 格式存储的位姿
         */
        static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
        /**
         * @brief 将以 g2o::Sim3 格式存储的位姿转换成为 g2o::SE3Quat 类型
         *
         * @param[in] gSim3 以 g2o::Sim3 格式存储的位姿
         * @return g2o::SE3Quat
         */
        static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

        /** @} */

        /**
         * @name toCvMat \n
         * @details 将各种格式转换成为cv::Mat存储
         * @{
         */

        /**
         * @brief 将以g2o::SE3Quat格式存储的位姿转换成为cv::Mat格式
         *
         * @param[in] SE3 输入的g2o::SE3Quat格式存储的、待转换的位姿
         * @return cv::Mat 转换结果
         * @remark
         */
        static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
        /**
         * @brief 将以g2o::Sim3格式存储的位姿转换成为cv::Mat格式
         *
         * @param[in] Sim3 输入的g2o::Sim3格式存储的、待转换的位姿
         * @return cv::Mat 转换结果
         * @remark
         */
        static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
        /**
         * @brief 将4x4 double型Eigen矩阵存储的位姿转换成为cv::Mat格式
         *
         * @paramp[in] m 输入Eigen矩阵
         * @return cv::Mat 转换结果
         * @remark
         */
        static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
        /**
         * @brief 将一个3x1的Eigen行向量转换成为cv::Mat格式
         *
         * @param[in] m 3x1的Eigen行向量
         * @return cv::Mat 转换结果
         */
        static cv::Mat toCvMat(const Eigen::Matrix3d &m);
        /**
         * @brief 将一个3x1的Eigen行向量转换成为cv::Mat格式
         *
         * @param[in] m 3x1的Eigen行向量
         * @return cv::Mat 转换结果
         */
        static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
        /**
         * @brief 将给定的旋转矩阵和平移向量转换为以cv::Mat存储的李群SE3
         * @details 其实就是组合旋转矩阵和平移向量来构造SE3
         * @param[in] R 旋转矩阵
         * @param[in] t 平移向量
         * @return cv::Mat 李群SE3
         */
        static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R, const Eigen::Matrix<double, 3, 1> &t);
        /** @} */

        /**
         * @name toEigen
         * @details 将给出的数据以 Eigen 库中对应的数据类型存储
         * @{
         */

        /**
         * @brief 将cv::Mat类型数据转换成为3x1的Eigen矩阵
         *
         * @param[in] cvVector 待转换的数据
         * @return Eigen::Matrix<double,3,1> 转换结果
         * @note 需要确保输入的数据大小尺寸正确。
         */
        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Mat &cvVector);
        /**
         * @brief 将cv::Point3f转换成为Eigen中3x1的矩阵
         *
         * @param[in] cvPoint 输入的cv表示的三维点坐标
         * @return Eigen::Matrix<double,3,1> 以Eigen中3x1向量表示的三维点坐标
         */
        static Eigen::Matrix<double, 3, 1> toVector3d(const cv::Point3f &cvPoint);
        /**
         * @brief 将一个3x3的cv::Mat矩阵转换成为Eigen中的矩阵
         *
         * @param[in] cvMat3 输入
         * @return Eigen::Matrix<double,3,3> 转换结果
         */
        static Eigen::Matrix<double, 3, 3> toMatrix3d(const cv::Mat &cvMat3);
        /**
         * @brief 将给定的cv::Mat类型的旋转矩阵转换成以std::vector<float>类型表示的四元数
         *
         * @param[in] M 以cv::Mat表示的旋转矩阵
         * @return std::vector<float> 四元数
         * @note 需要自己保证参数M满足旋转矩阵的定义
         * @remark
         */
        static std::vector<float> toQuaternion(const cv::Mat &M);

        /** @} */

        // new0 自定义的一个矩阵和四元数相互转换的函数
        static cv::Mat toCvMat(const std::vector<float> &v);
    };

} // namespace ORB_SLAM

#endif // CONVERTER_H
