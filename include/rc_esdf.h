/**
 * MIT License
 *
 * Copyright (c) 2026 Chunyu Ju
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file      rc_esdf.h
 * @brief     RC-ESDF: Robo-Centric 2D Signed Distance Field.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-02-15 09:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 * This library provides real-time distance and gradient queries for
 * robot-centric navigation and collision avoidance.
 */

#ifndef RC_ESDF_H
#define RC_ESDF_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl/octree/octree.h>

/**
 * @brief Robo-Centric ESDF (2D Version for Ground Robots)
 * 存储方式：一维数组模拟二维网格
 * 坐标系：Robot Body Frame (原点通常在机器人中心)
 * 约定：内部距离为负，外部距离为 0 (Paper setting)
 */
class RcEsdfMap {
public:
    RcEsdfMap() = default;

    /**
     * @brief 初始化地图参数
     * @param width_m  地图物理宽度 (米)
     * @param height_m 地图物理高度 (米)
     * @param resolution 分辨率 (米/像素)
     */
    void initialize(double width_m, double height_m, double resolution);

    /**
     * @brief (离线阶段) 根据多边形生成 ESDF
     * 简单的暴力生成法，实际使用时只在程序启动时跑一次
     * @param polygon 机器人的顶点列表 (按顺序，Body Frame)
     */
    void generateFromPolygon(const std::vector<Eigen::Vector2d> &polygon);

    /**
     * @brief 添加单个障碍物点 (Body Frame)
     * @param obs 障碍物点坐标
     */
    void addObstacle(const Eigen::Vector2d &obs);

    /**
     * @brief 批量添加障碍物点
     * @param obs_list 障碍物点列表
     */
    void addObstacles(const std::vector<Eigen::Vector2d> &obs_list);

    /**
     * @brief 清除所有障碍物
     */
    void clearObstacles();

    /**
     * @brief 移除指定范围内的障碍物
     * @param center 中心点
     * @param radius 半径范围
     */
    void removeObstaclesInRadius(const Eigen::Vector2d &center, double radius);

    /**
     * @brief 更新障碍物后的ESDF (增量更新)
     * 使用障碍物膨胀方法，距离为min(机器人距离, 障碍物距离)
     */
    void updateEsdfWithObstacles();

    /**
     * @brief 完全重新生成ESDF (机器人轮廓 + 障碍物)
     * 当障碍物大幅变化时调用
     */
    void regenerateEsdf();

    /**
     * @brief (在线阶段) 查询距离和梯度
     * 使用双线性插值 (Bilinear Interpolation)
     *
     * @param pos_body [输入] 障碍物点在 Body Frame 下的坐标
     * @param dist     [输出] 距离 (内部为负，外部为0)
     * @param grad     [输出] 梯度 (指向距离增加的方向，即指向体外)
     * @return true 如果点在地图范围内, false 如果点在地图范围外
     */
    bool query(const Eigen::Vector2d &pos_body, double &dist, Eigen::Vector2d &grad) const;

    void visualizeEsdf(const std::vector<Eigen::Vector2d> &footprint);

private:
    // 辅助：世界坐标转栅格索引
    inline void posToGrid(const Eigen::Vector2d &pos, double &gx, double &gy) const
    {
        gx = (pos.x() - origin_x_) / resolution_;
        gy = (pos.y() - origin_y_) / resolution_;
    }

    // 辅助：获取原始网格值
    inline float getRaw(int x, int y) const
    {
        if (x < 0 || x >= grid_size_x_ || y < 0 || y >= grid_size_y_)
            return 0.0f; // 越界视为外部
        return data_[y * grid_size_x_ + x];
    }

    // 辅助：点到线段距离平方
    double pointToSegmentDistSq(const Eigen::Vector2d &p, const Eigen::Vector2d &v, const Eigen::Vector2d &w);
    // 辅助：判断点是否在多边形内
    bool isPointInPolygon(const Eigen::Vector2d &p, const std::vector<Eigen::Vector2d> &poly);
    // 辅助：重建Octree
    void rebuildOctree();

    double resolution_;
    double width_m_, height_m_;
    double origin_x_, origin_y_; // 地图左下角在 Body Frame 的坐标
    int grid_size_x_, grid_size_y_;

    std::vector<float> data_;                    // 存储 SDF 值
    std::vector<float> base_data_;               // 机器人轮廓的基础ESDF
    std::vector<Eigen::Vector2d> obstacles_;     // 障碍物列表
    std::vector<Eigen::Vector2d> robot_polygon_; // 机器人轮廓

    // PCL Octree for fast nearest neighbor search
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
};

#endif // RC_ESDF_H