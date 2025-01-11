#include <ros/ros.h>
#include <utility>
#include <vector>
#include <queue>
#include <cmath>
#include <Eigen/Dense>
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>
#include "nav_msgs/Path.h"
#include <unordered_map>
//#include "planner.h"
//ros::Publisher _path_minimum_pub;//发布minimum-snap轨迹
std::vector<Eigen::Vector2d> path;
struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x, int y, double g_cost, double h_cost, std::shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(std::move(parent)) {}

    double f() const { return g_cost + h_cost; } // 总代价值

};
// 比较器，用于优先队列
struct cmp{
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b){
        return a->f() > b->f();
    }

};
struct GridMap {
    int width;
    int height;
    double map_max;
    double map_min;
    double grid_resolution;//实际栅格分辨率，如0.5则说明世界坐标系占2格
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 占用

    GridMap(int w, int h, double map_min_, double map_max_, double res) : width(w), height(h), map_min(map_min_), map_max(map_max_), grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    void markObstacle(double cx, double cy, double radius) {
        int grid_cx = std::round((cx - map_min) / grid_resolution);//取x，y的离散变量
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);//取半径的离散变量
        // Step 1: 将圆形区域标记为占用
	    for (int i = -grid_radius;i < grid_radius;i++){
		    for(int j=-grid_radius;j<grid_radius;j++){
                if (i * i + j * j <= grid_radius * grid_radius) {//以grid_radius为半径，(cx，cy)为原点，i，j表示偏移量，勾股定理
                    int nx = grid_cx + i;
                    int ny = grid_cy + j;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        grid[nx][ny] = 1; // 标记为占用
                    }
                    }
                    }  
			    }
            // your code
        // finish
    }
};
class AStarPlanner {
public:
    AStarPlanner(int width, int height, double m_min, double m_max, double res) : width_(width), height_(height), map_min_(m_min), map_max_(m_max), grid_resolution_(res), grid_map_(width, height, map_min_, map_max_, grid_resolution_), num_of_obs_(0) {

    }

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    void printGridMap(){
        for(int i = 0; i < width_; i++){
            for(int j = 0; j < height_; j++){
                std::cout<<grid_map_.grid[i][j]<<" ";
            }
            std::cout<<std::endl;
        }
        std::cout<<"num of obstacles: "<<num_of_obs_<<std::endl;
    }

    std::vector<Eigen::Vector2d> findPath(Eigen::Vector2d start, Eigen::Vector2d goal) {
        if(num_of_obs_ == 0){
            return {};
        }
        // 起点和终点转换为网格坐标
        auto gridStart = worldToGrid(start);
        auto gridGoal = worldToGrid(goal);

        // 开放列表和关闭列表
        std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));
        std::unordered_map<long, std::shared_ptr<Node>> open_list_lookup; // 辅助哈希表,用于获取open_list的内部队列并遍历
        // 起点加入开放列表
        auto startNode = std::make_shared<Node>(Node(gridStart.first, gridStart.second, 0.0, heuristic(gridStart, gridGoal)));
        open_list.push(startNode);
        long startIdx = gridStart.first * width_ + gridStart.second;
        open_list_lookup[startIdx] = startNode;
        // Step 3： 实现 A* 算法，搜索结束调用 reconstructPath 返回路径
        while (!open_list.empty()) {//循环直到所有节点被找到
            // 取出f值最小的节点
            auto current = open_list.top();
            open_list.pop();
            long currentIndex = current->x * width_ + current->y;
            open_list_lookup.erase(currentIndex); // 从辅助哈希表中移除
            // 如果当前节点是目标节点，则重建路径并返回
            if (current->x == gridGoal.first && current->y == gridGoal.second) {
                return reconstructPath(current);
            }

            // 标记当前节点为已访问（放入关闭列表）
            closed_list[current->x][current->y] = true;

            // 获取当前节点的所有邻居节点
            auto neighbors = getNeighbors(*current, goal);
            for (auto& neighbor : neighbors) {
                long neighborIndex = neighbor.x * width_ + neighbor.y;
                // 如果邻居已经在关闭列表中，跳过
                if (closed_list[neighbor.x][neighbor.y]) continue;

                // 计算新路径到达该邻居的成本
                double tentative_g_cost = current->g_cost + distance(*current, neighbor);
                if (open_list_lookup.find(neighborIndex) == open_list_lookup.end()) { // 如果不在开放列表中
                    // 设置邻居节点信息并添加到开放列表和辅助哈希表
                    neighbor.parent = current;
                    neighbor.g_cost = tentative_g_cost;
                    neighbor.h_cost = heuristic(std::make_pair(neighbor.x, neighbor.y), gridGoal);
                    open_list.push(std::make_shared<Node>(neighbor));
                    open_list_lookup[neighborIndex] = open_list.top();
                }
                else if (tentative_g_cost < open_list_lookup[neighborIndex]->g_cost) { // 找到了更好的路径
                    // 更新开放列表中的节点
                    open_list_lookup[neighborIndex]->parent = current;
                    open_list_lookup[neighborIndex]->g_cost = tentative_g_cost;
                    // 重新排序优先队列可能需要手动实现或者考虑其他数据结构
                }
                // 检查邻居是否在开放列表中以及新的g_cost是否更优。但priority_queue并没有提供直接访问内部容易的方法，所以报错
                /*
                bool in_open_list = false;
                for (auto& node : open_list.queue) {
                    if (node->x == neighbor.x && node->y == neighbor.y) {
                        in_open_list = true;
                        if (tentative_g_cost >= node->g_cost) continue; // 如果已有路径更好，跳过
                        else {
                            node->parent = current; // 更新父节点以反映更优路径
                            node->g_cost = tentative_g_cost;
                            break;
                        }
                    }
                }
          
                // 如果邻居不在开放列表中，或者找到了更好的路径
                if (!in_open_list) {
                    neighbor.parent = std::make_shared<Node>(*current); // 设置父节点
                    open_list.push(std::make_shared<Node>(neighbor)); // 添加到开放列表
                }
                */
            }
        }
            // 样例路径，用于给出路径形式，实现 A* 算法时请删除
            /*
                std::vector<Eigen::Vector2d> path;
                int num_points = 100; // 生成路径上的点数
                for (int i = 0; i <= num_points; ++i) {
                    double t = static_cast<double>(i) / num_points;
                    Eigen::Vector2d point = start + t * (goal - start);
                    path.push_back(point);
                }
                return path;
                */
            // 注释结束
            // your code

        // finish

        // 如果没有找到路径，返回空路径
        return {};
    }
    void reset(){
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }
private:

    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int, int>& from, const std::pair<int, int>& to) {
        return std::sqrt(std::pow(from.first - to.first, 2) + std::pow(from.second - to.second, 2));
    }

    // 计算两节点之间的距离（用于邻居代价计算）
    double distance(const Node& a, const Node& b) {
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    // 从世界坐标转换到栅格坐标
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& position) {
        int x = std::round((position.x() - map_min_) / grid_resolution_);
        int y = std::round((position.y() - map_min_) / grid_resolution_);
        return {x, y};
    }

    // 从栅格坐标转换到世界坐标（主要用于路径结果显示）
    Eigen::Vector2d gridToWorld(int x, int y) {
        double wx = x * grid_resolution_ + map_min_;
        double wy = y * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    // 获取当前节点的所有邻居节点
    std::vector<Node> getNeighbors(const Node& current, const Eigen::Vector2d& goal) {
        std::vector<Node> neighbors;

        // 八连通邻居
        std::vector<std::pair<int, int>> directions = {
                {1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
        for (const auto& dir : directions) {
            // Step 2: 根据当前节点和方向计算邻居节点的坐标，并将其加入 neighbors
            int nx = current.x + dir.first;//访问容器中的第一个数，即{1，0}中的1
            int ny = current.y + dir.second;//访问容器中第二个数，即{1,0}中的0
            if (nx >= 0 && ny >= 0 && nx <= width_ && ny <= height_ && grid_map_.grid[nx][ny] == 0)//只有当在地图内且没有障碍物的时候，才把邻居节点添加进去
            {
                double move_cost;
                move_cost = distance(current, Node(nx, ny, 0, 0));
                neighbors.emplace_back(nx, ny, current.g_cost + move_cost, heuristic({ nx, ny }, worldToGrid(goal)), std::make_shared<Node>(current));
            }
                // your code
            // finish
        }

        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) {
        std::vector<Eigen::Vector2d> path;
        while (node) {
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
int main(int argc, char** argv) {
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
                                                                                 [&planner, &grid_resolution_, &map_min_](const visualization_msgs::MarkerArray::ConstPtr& msg) {
                                                                                     for (const auto& marker : msg->markers) {
                                                                                         planner.setObstacle(marker.pose.position.x, marker.pose.position.y, marker.scale.x / 2.0);
                                                                                     }
                                                                                 });



    // 发布路径
    ros::Rate rate(10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    //_path_minimum_pub         = nh.advertise<nav_msgs::Path>("/tra_generation", 10); 
    // 起点和终点参数
    Eigen::Vector2d start(start_x_, start_y_);
    Eigen::Vector2d goal(goal_x_, goal_y_);
    while (ros::ok()) {
        planner.reset();
//        // 等待障碍物加载
//        ros::Duration(1.0).sleep();
        ros::spinOnce();
        // 执行路径搜索
        path = planner.findPath(start, goal);

        // 路径可视化
        if (path.empty()){
            continue;
        }
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (const auto& point : path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        //CalShow_minimum_tra(path);
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}
