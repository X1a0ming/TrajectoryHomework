#include <ros/ros.h>
#include <utility>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"

struct Node
{
    int x, y;                     // 节点所在的网格坐标
    double g_cost;                // 从起点到当前节点的代价
    double h_cost;                // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent; // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
        : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值
};
struct cmp
{
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b)
    {
        return a->f() > b->f();
    }
};
struct GridMap
{
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res) : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius)
    {
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution) + 1;
        int grid_radius_square = grid_radius * grid_radius;
        // Step 1: 将圆形区域标记为占用
        for (int i = grid_cx - grid_radius; i <= grid_cx + grid_radius; i++)
        {
            for (int j = grid_cy - grid_radius; j <= grid_cy + grid_radius; j++)
            {
                if ((i - grid_cx) * (i - grid_cx) + (j - grid_cy) * (j - grid_cy) <= grid_radius_square)
                {
                    if (i >= 0 && i < width && j >= 0 && j < height)
                    {
                        grid[i][j] = 1;
                    }
                }
            }
        }
        // finish
    }
};
class AStarPlanner
{
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0)
    {
    }

    void setObstacle(double cx, double cy, double radius)
    {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    void printGridMap()
    {
        for (int i = 0; i < width_; i++)
        {
            for (int j = 0; j < height_; j++)
            {
                std::cout << grid_map_.grid[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "num of obstacles: " << num_of_obs_ << std::endl;
    }

    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal)
    {

        if (num_of_obs_ == 0)
        {
            return {};
        }
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        open_list.push(std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal))));
        // Step 3： 实现 A* 算法，搜索结束调用 reconstructPath 返回路径

        while (!open_list.empty())
        {
            auto current = open_list.top();
            open_list.pop();
            if (current->x == gridGoal.first && current->y == gridGoal.second)
            {

                return reconstructPath(current);
            }
            closed_list[current->x][current->y] = true;
            auto neighbors = getNeighbors(*current);
            for (auto &neighbor : neighbors)
            {
                if (closed_list[neighbor.x][neighbor.y] || grid_map_.grid[neighbor.x][neighbor.y] == 1)
                {
                    continue;
                }
                double new_g_cost = current->g_cost + distance(*current, neighbor);
                bool in_open_list = false;
                std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list_temp;
                while (!open_list.empty())
                {
                    auto temp = open_list.top();
                    open_list_temp.push(temp);
                    open_list.pop();
                    if (temp->x == neighbor.x && temp->y == neighbor.y)
                    {
                        in_open_list = true;
                        break;
                    }
                }
                while (!open_list_temp.empty())
                {
                    open_list.push(open_list_temp.top());
                    open_list_temp.pop();
                }
                if (!in_open_list || new_g_cost < neighbor.g_cost)
                {
                    neighbor.g_cost = new_g_cost;
                    neighbor.h_cost = heuristic({neighbor.x, neighbor.y}, gridGoal);
                    neighbor.parent = current;
                    if (!in_open_list)
                    {
                        open_list.push(std::make_shared<Node>(neighbor));
                    }
                }
            }
        }

        // finish

        return {};
    }
    void reset()
    {
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }

private:
    double heuristic(const std::pair<int, int> &from, const std::pair<int, int> &to)
    {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    double distance(const Node &a, const Node &b)
    {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    std::pair<int, int> worldToGrid(const Eigen::Vector2d &position)
    {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    Eigen::Vector2d gridToWorld(int x, int y)
    {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    std::vector<Node> getNeighbors(const Node &current)
    {
        std::vector<Node> neighbors;

        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto &dir : directions)
        {
            // Step 2: 根据当前节点和方向计算邻居节点的坐标，并将其加入 neighbors
            int x = current.x + dir.first;
            int y = current.y + dir.second;
            if (x >= 0 && x < width_ && y >= 0 && y < height_)
            {
                neighbors.push_back(Node(x, y, 999, 0.0));
            }
            // finish
        }

        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node)
    {
        std::vector<Eigen::Vector2d> path;
        while (node)
        {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        reset();
        return path;
    }

    // 地图数据
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_; // 栅格地图，0: 空闲，1: 障碍物
    int num_of_obs_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_planner");
    ros::NodeHandle nh;
    double map_min_, map_max_, grid_resolution_;
    double start_x_, start_y_, goal_x_, goal_y_;
    nh.param("astar_planner/map_min", map_min_, -5.0);
    nh.param("astar_planner/map_max", map_max_, 5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution_, 0.1);
    nh.param("astar_planner/start_x", start_x_, -4.5);
    nh.param("astar_planner/start_y", start_y_, -4.5);
    nh.param("astar_planner/goal_x", goal_x_, 4.5);
    nh.param("astar_planner/goal_y", goal_y_, 4.5);

    // 地图参数
    int grid_width = std::round((map_max_ - map_min_) / grid_resolution_);
    int grid_height = grid_width;

    AStarPlanner planner(grid_width, grid_height, map_min_, map_max_, grid_resolution_);
    // 障碍物订阅
    ros::Subscriber obstacle_sub = nh.subscribe<visualization_msgs::MarkerArray>("obstacles", 1,
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr &msg)
                                                                                 {
                                                                                     for (const auto &marker : msg->markers)
                                                                                     {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });

    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok())
    {
        planner.reset();
        //        // 等待障碍物加载
        //        ros::Duration(1.0).sleep();
        ros::spinOnce();
        // 执行路径搜索
        ros::Time start_time = ros::Time::now();
        std::vector<Eigen::Vector2d> path = planner.findPath(start, goal);
        ros::Duration search_time = ros::Time::now() - start_time;
        ROS_INFO("A* Path found in %.3f seconds", search_time.toSec());
        // 路径可视化
        if (path.empty())
        {
            continue;
        }
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto &point : path)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}