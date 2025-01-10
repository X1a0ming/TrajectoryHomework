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

typedef std::vector<Eigen::Vector2d> Path;

// Step 4: 自行实现轨迹生成类
class TrajectoryGenerator
{
public:
    TrajectoryGenerator(int width, int height, double m_min, double m_max, double res)
        : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res) {}

    std::vector<Eigen::Vector2d> GenerateTrajectory(const std::vector<Eigen::Vector2d> &path)
    {

        int num_points = 30;

        std::vector<Eigen::Vector2d> trajectory;
        if (path.size() < num_points)
        {
            return trajectory;
        }

        Path sample_path;
        for (size_t i = 0; i < path.size(); i += path.size() / num_points)
        {
            sample_path.push_back(path[i]);
        }
        sample_path.push_back(path.back());

        double dt = 1.0;
        for (size_t i = 0; i < sample_path.size() - 1; ++i)
        {
            Eigen::Vector2d start = sample_path[i];
            Eigen::Vector2d end = sample_path[i + 1];

            Eigen::VectorXd x_coeffs = computeQuinticPolynomialCoefficients(start.x(), 0.0, 0.0, end.x(), 0.0, 0.0, dt);
            Eigen::VectorXd y_coeffs = computeQuinticPolynomialCoefficients(start.y(), 0.0, 0.0, end.y(), 0.0, 0.0, dt);

            for (double t = 0; t <= dt; t += 0.1)
            {
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

    Eigen::VectorXd computeQuinticPolynomialCoefficients(double start_pos, double start_vel, double start_acc, double end_pos, double end_vel, double end_acc, double dt)
    {
        Eigen::MatrixXd A(6, 6);
        A << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 2, 0, 0, 0,
            1, dt, dt * dt, dt * dt * dt, dt * dt * dt * dt, dt * dt * dt * dt * dt,
            0, 1, 2 * dt, 3 * dt * dt, 4 * dt * dt * dt, 5 * dt * dt * dt * dt,
            0, 0, 2, 6 * dt, 12 * dt * dt, 20 * dt * dt * dt;

        Eigen::VectorXd b(6);
        b << start_pos, start_vel, start_acc, end_pos, end_vel, end_acc;

        return A.colPivHouseholderQr().solve(b);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory", 1);
    ros::Rate rate(10);

    Path path;
    bool new_path_received = false; // 标志位，表示是否收到新路径
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("path", 1, [&path, &new_path_received](const nav_msgs::Path::ConstPtr &msg)
                                                            {
        path.clear();
        for (const auto& pose : msg->poses) {
            path.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
        }
        new_path_received = true; });

    TrajectoryGenerator generator(100, 100, -5.0, 5.0, 0.1);
    std::vector<Eigen::Vector2d> trajectory;

    while (ros::ok())
    {
        ros::spinOnce();

        if (new_path_received)
        {
            ros::Time start_time = ros::Time::now();

            trajectory = generator.GenerateTrajectory(path);
            ros::Duration search_time = ros::Time::now() - start_time;
            ROS_INFO("trajectory found in %.3f seconds", search_time.toSec());
            new_path_received = false;
        }

        // 发布轨迹
        if (!trajectory.empty())
        {
            nav_msgs::Path msg;
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();
            for (const auto &point : trajectory)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                pose.pose.position.z = 0.0;
                msg.poses.push_back(pose);
            }
            path_pub.publish(msg);
        }

        rate.sleep();
    }

    return 0;
}