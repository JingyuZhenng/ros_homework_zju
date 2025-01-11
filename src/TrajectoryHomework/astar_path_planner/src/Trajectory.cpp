#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <qpOASES.hpp>

typedef std::vector<Eigen::Vector2d> Path;

class TrajectoryGenerator {
public:
    TrajectoryGenerator() = default;

    // 生成轨迹的方法
    bool generateTrajectory(const Path& path, std::vector<Eigen::VectorXd>& trajectory_x, std::vector<Eigen::VectorXd>& trajectory_y) {
        if (path.size() < 2) {
            std::cerr << "Path too short to generate trajectory." << std::endl;
            return false;
        }

        // 分配每段的时间，简单假设每段时间相等
        int n_segments = path.size() - 1;
        double total_time = 10.0; // 总时间可以根据需要调整
        double dt = total_time / n_segments;

        // 初始化轨迹存储
        trajectory_x.clear();
        trajectory_y.clear();

        // 对每个维度分别进行处理
        std::vector<double> waypoints_x, waypoints_y;
        for (const auto& point : path) {
            waypoints_x.push_back(point.x());
            waypoints_y.push_back(point.y());
        }

        // 生成x和y方向的轨迹
        bool success_x = generateQuinticPolynomial(waypoints_x, dt, trajectory_x);
        bool success_y = generateQuinticPolynomial(waypoints_y, dt, trajectory_y);

        return success_x && success_y;
    }

private:
    // 生成五次多项式轨迹
    bool generateQuinticPolynomial(const std::vector<double>& waypoints, double dt, std::vector<Eigen::VectorXd>& trajectory) {
        int n = waypoints.size();
        int n_segments = n - 1;

        // 每个多项式有6个系数
        int n_coeffs = 6;

        // 构建矩阵A和向量b
        // 需要满足每个段的起始和结束位置、速度、加速度
        // 总共有 2*n_segments * 3 constraints
        int n_constraints = 6 * n_segments;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_constraints, n_coeffs * n_segments);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(n_constraints);

        for (int i = 0; i < n_segments; ++i) {
            // Start conditions
            double t0 = 0.0;
            double tf = dt;

            // Position at t0
            A(6*i, i*n_coeffs + 0) = 1;
            b(6*i) = waypoints[i];
            // Velocity at t0
            A(6*i+1, i*n_coeffs + 1) = 1;
            b(6*i+1) = 0.0;
            // Acceleration at t0
            A(6*i+2, i*n_coeffs + 2) = 2;
            b(6*i+2) = 0.0;

            // Position at tf
            A(6*i+3, i*n_coeffs + 0) = 1;
            A(6*i+3, i*n_coeffs + 1) = tf;
            A(6*i+3, i*n_coeffs + 2) = pow(tf, 2);
            A(6*i+3, i*n_coeffs + 3) = pow(tf, 3);
            A(6*i+3, i*n_coeffs + 4) = pow(tf, 4);
            A(6*i+3, i*n_coeffs + 5) = pow(tf, 5);
            b(6*i+3) = waypoints[i+1];
            // Velocity at tf
            A(6*i+4, i*n_coeffs + 1) = 1;
            A(6*i+4, i*n_coeffs + 2) = 2*tf;
            A(6*i+4, i*n_coeffs + 3) = 3*pow(tf, 2);
            A(6*i+4, i*n_coeffs + 4) = 4*pow(tf, 3);
            A(6*i+4, i*n_coeffs + 5) = 5*pow(tf, 4);
            b(6*i+4) = 0.0;
            // Acceleration at tf
            A(6*i+5, i*n_coeffs + 2) = 2;
            A(6*i+5, i*n_coeffs + 3) = 6*tf;
            A(6*i+5, i*n_coeffs + 4) = 12*pow(tf, 2);
            A(6*i+5, i*n_coeffs + 5) = 20*pow(tf, 3);
            b(6*i+5) = 0.0;
        }

        // 设置连续性约束
        // 位置、速度、加速度在每个连接点处连续
        // 这里简化处理，只保证每段的末端和下一段的起始端一致
        // 可以进一步优化以确保更高的连续性

        // 构建QP问题
        // 目标是最小化所有段的加加速度（jerk）
        // 可以通过最小化系数矩阵的某些组合来实现

        // 这里为了简化，使用最小二乘法求解
        // A * x = b
        // x = (A^T A)^-1 A^T b

        Eigen::VectorXd coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

        if ((A * coeffs - b).norm() > 1e-6) {
            std::cerr << "Warning: Trajectory generation might be inaccurate." << std::endl;
        }

        // 将系数分段存储
        for (int i = 0; i < n_segments; ++i) {
            Eigen::VectorXd segment_coeffs(n_coeffs);
            for (int j = 0; j < n_coeffs; ++j) {
                segment_coeffs(j) = coeffs(i*n_coeffs + j);
            }
            trajectory.push_back(segment_coeffs);
        }

        return true;
    }
};

int main() {
    // 示例路径
    Path path = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 2),
        Eigen::Vector2d(4, 3),
        Eigen::Vector2d(6, 5)
    };

    TrajectoryGenerator generator;
    std::vector<Eigen::VectorXd> trajectory_x, trajectory_y;

    if (generator.generateTrajectory(path, trajectory_x, trajectory_y)) {
        std::cout << "Trajectory generated successfully." << std::endl;

        // 打印轨迹系数
        for (size_t i = 0; i < trajectory_x.size(); ++i) {
            std::cout << "Segment " << i << " X coefficients: " << trajectory_x[i].transpose() << std::endl;
            std::cout << "Segment " << i << " Y coefficients: " << trajectory_y[i].transpose() << std::endl;
        }
    } else {
        std::cerr << "Failed to generate trajectory." << std::endl;
    }

    return 0;
}

