#include "EigenUtils.h"

RigidTransform EigenUtils::computeRigidTransform(
    const std::vector<Eigen::Vector3d>& scanner_points,
    const std::vector<Eigen::Vector3d>& machine_points)
{
    size_t N = scanner_points.size();
    if (N < 3 || N != machine_points.size()) {
        throw std::runtime_error("至少需要3个点且点数要对应");
    }

    // 1. 计算质心
    Eigen::Vector3d c_scanner(0, 0, 0);
    Eigen::Vector3d c_machine(0, 0, 0);
    for (size_t i = 0; i < N; ++i) {
        c_scanner += scanner_points[i];
        c_machine += machine_points[i];
    }
    c_scanner /= N;
    c_machine /= N;

    // 2. 中心化
    Eigen::MatrixXd Q_scanner(3, N);
    Eigen::MatrixXd Q_machine(3, N);
    for (size_t i = 0; i < N; ++i) {
        Q_scanner.col(i) = scanner_points[i] - c_scanner;
        Q_machine.col(i) = machine_points[i] - c_machine;
    }

    // 3. 协方差矩阵
    Eigen::Matrix3d H = Q_scanner * Q_machine.transpose();

    // 4. SVD 分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // 5. 求旋转矩阵
    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0) {
        // 修正镜像情况
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    // 6. 求平移向量
    Eigen::Vector3d T = c_machine - R * c_scanner;

    return { R, T };
}