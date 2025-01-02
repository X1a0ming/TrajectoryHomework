#include <ros/ros.h>
#include <utility>
#include <iostream>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <cmath>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"

// 定义路径格式
typedef std::vector<Eigen::Vector2d> Path;

struct Node {
    int x,y; // 节点所在的网格坐标
    double vel,acc,jerk,snap; // 节点的速度、加速度、加加速度、加加加速度
    std::shared_ptr<Node> parent,child; // 父节点、子节点
}

// Step 4: 自行实现轨迹生成类
class TrajectoryGenerator {
    // 轨迹规划的目标是根据A*算法给出的无碰撞路径，计算轨迹航点（控制点），从而生成一条以时间参数化的平滑轨迹，可以用于控制移动机器人跟踪
    // 本次作业中，我们要求生成一条分段多项式轨迹，即每段轨迹均为一个多项式函数
    // 你可以选择使用多项式、B样条、贝塞尔曲线、MINCO等多种轨迹基函数实现
    // 每段轨迹的连接处需要满足一定的连续性条件，如位置连续、速度连续、加速度连续等，这将构成轨迹优化的主要约束条件
    // 轨迹的初始和终止状态为到达指定位置，速度、加速度等状态均为0
    // 优化目标可以是最小化轨迹的加加速度（jerk）或加加加速度（snap），请自行选择合适的优化目标及多项式阶数
    // 本次作业对轨迹段的时间选择不做进一步要求，可以自行选择固定时间或自定义策略生成时间分配
    // 可以任意选用求解器，如qpOASES、OSQP-Eigen等，也可以自行实现闭式解法
public:
    TrajectoryGenerator(int width,int height, double m_min, double m_max, double res) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res){
        
    }

        // your code
    std::vector<Eigen::Vector2d> GenerateTraJ(std::vector<Eigen::Vector2d> Path){

    }

private:
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1);
    ros::Rate rate(10);

    // 读取路径
    Path path;
    // your code

    // 生成轨迹
    TrajectoryGenerator generator;
    // your code

    while (ros::ok()) {
        // 发布轨迹
        nav_msgs::Path msg;
        // your code
        path_pub.publish(msg);
        rate.sleep();
    }

    return 0;
}