/**
 * @file      main.cpp
 * @brief     Testing RC-ESDF: Robo-Centric 2D Signed Distance Field.
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-02-15 10:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
*/
#include "rc_esdf.h"
#include <iostream>
#include <vector>
#include <chrono> 

int main() 
{
    // 1. 定义机器人形状 (Body Frame)
    std::vector<Eigen::Vector2d> footprint;
    footprint.push_back(Eigen::Vector2d(0.5, 0.3));
    footprint.push_back(Eigen::Vector2d(-0.5, 0.3));
    footprint.push_back(Eigen::Vector2d(-0.5, -0.3));
    footprint.push_back(Eigen::Vector2d(0.5, -0.3));
  
    // 2. 初始化并生成基础 ESDF (机器人轮廓)
    RcEsdfMap rc_map;
    rc_map.initialize(5.0, 5.0, 0.1); 
    rc_map.generateFromPolygon(footprint);

    // 3. 添加障碍物并更新 ESDF
    std::cout << "\n=== 添加障碍物 ===" << std::endl;
    std::vector<Eigen::Vector2d> obstacles;
    obstacles.push_back(Eigen::Vector2d(1.0, 0.0));
    obstacles.push_back(Eigen::Vector2d(1.5, 0.5));
    obstacles.push_back(Eigen::Vector2d(-1.0, -0.5));
    rc_map.addObstacles(obstacles);
    rc_map.updateEsdfWithObstacles();

    // 4. 查询测试
    std::cout << "\n=== 查询测试 ===" << std::endl;
    std::vector<Eigen::Vector2d> obs_points_body;
    obs_points_body.push_back(Eigen::Vector2d(0.0, 0.0));
    obs_points_body.push_back(Eigen::Vector2d(0.4, 0.2));
    obs_points_body.push_back(Eigen::Vector2d(0.6, 0.6));
    obs_points_body.push_back(Eigen::Vector2d(1.0, 0.0));
    obs_points_body.push_back(Eigen::Vector2d(1.5, 0.5));

    auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& p : obs_points_body) {
        double dist;
        Eigen::Vector2d grad;
        bool in_box = rc_map.query(p, dist, grad);
        
        std::cout << "Point: (" << p.x() << ", " << p.y() << ") ";
        if (!in_box) {
            std::cout << "-> Out of Box" << std::endl;
        } else {
            std::cout << "-> Dist: " << dist << " | Grad: (" << grad.x() << ", " << grad.y() << ")";
            if (dist < 0) {
                std::cout << " [COLLISION]";
            }
            std::cout << std::endl;
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_ms = end_time - start_time;
    std::cout << "\nQuery Time: " << elapsed_ms.count() << " ms" << std::endl;

    // 5. 模拟障碍物动态更新
    std::cout << "\n=== 动态更新障碍物 ===" << std::endl;
    rc_map.clearObstacles();
    rc_map.addObstacles({Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(1.5, 1.5)});
    rc_map.updateEsdfWithObstacles();
   
    rc_map.visualizeEsdf(footprint);  // 注释掉可视化以便快速测试
    
    // // 移除范围内的障碍物
    // std::cout << "\n=== 移除障碍物 (半径1.0) ===" << std::endl;
    // rc_map.removeObstaclesInRadius(Eigen::Vector2d(2.0, 0.0), 1.0);
    // rc_map.updateEsdfWithObstacles();

    // rc_map.visualizeEsdf(footprint);  // 注释掉可视化以便快速测试

    return 0;
}