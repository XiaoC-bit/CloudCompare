#ifndef EIGEN_UTILS_H
#define EIGEN_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <stdexcept>

struct RigidTransform {
    Eigen::Matrix3d R; // 旋转矩阵
    Eigen::Vector3d T; // 平移向量
};

class EigenUtils {
public:

        void RTCP();
};

#endif // EIGEN_UTILS_H
