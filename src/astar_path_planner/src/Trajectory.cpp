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
#include <nav_msgs/Path.h>
#include <Eigen/QR>

// 定义路径格式
typedef std::vector<Eigen::Vector2d> Path;

// Step 4: 自行实现轨迹生成类
class TrajectoryGenerator {
public:
    TrajectoryGenerator(int width, int height, double m_min, double m_max, double res) 
        : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res) {}

    // 生成五次多项式轨迹
    std::vector<Eigen::Vector2d> GenerateTrajectory(const std::vector<Eigen::Vector2d>& path) {
        std::vector<Eigen::Vector2d> trajectory;
        if (path.size() < 2) {
            return trajectory; // 路径点不足，无法生成轨迹
        }

        // 假设每段轨迹的时间间隔为1秒
        double dt = 1.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            Eigen::Vector2d start = path[i];
            Eigen::Vector2d end = path[i + 1];

            // 生成五次多项式系数
            Eigen::VectorXd x_coeffs = computeQuinticPolynomialCoefficients(start.x(), 0.0, 0.0, end.x(), 0.0, 0.0, dt);
            Eigen::VectorXd y_coeffs = computeQuinticPolynomialCoefficients(start.y(), 0.0, 0.0, end.y(), 0.0, 0.0, dt);

            // 生成轨迹点
            for (double t = 0; t <= dt; t += 0.1) {
                double x = x_coeffs[0] + x_coeffs[1] * t + x_coeffs[2] * t * t + x_coeffs[3] * t * t * t + x_coeffs[4] * t * t * t * t + x_coeffs[5] * t * t * t * t * t;
                double y = y_coeffs[0] + y_coeffs[1] * t + y_coeffs[2] * t * t + y_coeffs[3] * t * t * t + y_coeffs[4] * t * t * t * t + y_coeffs[5] * t * t * t * t * t;
                trajectory.push_back(Eigen::Vector2d(x, y));
            }
        }

        return trajectory;
    }

private:
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;

    // 计算五次多项式系数
    Eigen::VectorXd computeQuinticPolynomialCoefficients(double start_pos, double start_vel, double start_acc, double end_pos, double end_vel, double end_acc, double dt) {
        Eigen::MatrixXd A(6, 6);
        A << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 2, 0, 0, 0,
             1, dt, dt*dt, dt*dt*dt, dt*dt*dt*dt, dt*dt*dt*dt*dt,
             0, 1, 2*dt, 3*dt*dt, 4*dt*dt*dt, 5*dt*dt*dt*dt,
             0, 0, 2, 6*dt, 12*dt*dt, 20*dt*dt*dt;

        Eigen::VectorXd b(6);
        b << start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;

        return A.colPivHouseholderQr().solve(b);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1);
    ros::Rate rate(10);

    // 读取路径
    Path path;
    bool new_path_received = false;  // 标志位，表示是否收到新路径
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("path", 1, [&path, &new_path_received](const nav_msgs::Path::ConstPtr& msg) {
        path.clear();
        for (const auto& pose : msg->poses) {
            path.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
        }
        new_path_received = true;  // 标记收到新路径
    });

    // 生成轨迹
    TrajectoryGenerator generator(100, 100, -5.0, 5.0, 0.1); // 假设地图参数与astar_planner一致
    std::vector<Eigen::Vector2d> trajectory;

    while (ros::ok()) {
        ros::spinOnce();

        if (new_path_received) {  // 只在收到新路径时生成轨迹
            printf("New path received, size: %d\n", path.size());
            trajectory = generator.GenerateTrajectory(path);
            new_path_received = false;  // 重置标志位
        }

        // 发布轨迹
        if (!trajectory.empty()) {
            nav_msgs::Path msg;
            msg.header.frame_id = "map";  // 设置坐标系为 map
            msg.header.stamp = ros::Time::now();
            for (const auto& point : trajectory) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
                msg.poses.push_back(pose);
            }
            path_pub.publish(msg);  // 发布轨迹
        }

        rate.sleep();
    }

    return 0;
}