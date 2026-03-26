#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <vector>
#include <stdexcept>

struct RigidTransform {
    Eigen::Matrix3d R; // 旋转矩阵
    Eigen::Vector3d T; // 平移向量
};

class EigenUtils {
public:
    // 输入：
    // - scanner_points: 扫描仪坐标系的 N 个球心 (std::vector<Eigen::Vector3d>)
    // - machine_points: 机床坐标系的 N 个球心
    // 输出：
    // - RigidTransform {R, T}
    static RigidTransform computeRigidTransform(
        const std::vector<Eigen::Vector3d>& scanner_points,
        const std::vector<Eigen::Vector3d>& machine_points);
};

#endif // EIGEN_UTILS_H
