#include "EigenUtils.h"


constexpr double PI = 3.14159265358979323846;
void EigenUtils::RTCP()
 {
    // === 参数定义 ===
    Eigen::Vector3d g54_in_machine(25, -90, -80);   // G54在机械坐标系下的位置（B=0 C=0时录入）
    double g54_B_angle = 10;
    double g54_C_angle = 0.0;

    Eigen::Vector3d pivot_B_in_machine(-52.170 - 0.025, 0, -174.543 + 0.039);
    Eigen::Vector3d pivot_C_in_machine(-52.170, -81.618, 0);

    double target_X = 1;
    double target_Y = 2;
    double target_Z = 3;
    double target_B = 25.0;
    double target_C = 0.0;

    // =====================================================
    // 第一步：计算pivot在工件系下的位置
    // XYZ计算始终以 B=0 C=0 为基准，g54_B_angle/g54_C_angle 不参与XYZ计算
    // =====================================================
    Eigen::Vector3d pivot_B_in_work = pivot_B_in_machine - g54_in_machine;
    Eigen::Vector3d pivot_C_in_work = pivot_C_in_machine - g54_in_machine;

    // =====================================================
    // 第二步：计算目标旋转矩阵（相对 B=0 C=0 基准的绝对角度）
    // =====================================================
    double angle_C = target_C * PI / 180.0;
    Eigen::AngleAxisd rot_C(-angle_C, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R_C = rot_C.toRotationMatrix();

    double angle_B = target_B * PI / 180.0;
    Eigen::AngleAxisd rot_B(-angle_B, Eigen::Vector3d::UnitY());
    Eigen::Matrix3d R_B = rot_B.toRotationMatrix();

    // =====================================================
    // 第三步：RTCP补偿（工件系下）
    // =====================================================
    Eigen::Vector3d tool_tip_in_work(target_X, target_Y, target_Z);

    Eigen::Vector3d compensation_C = pivot_C_in_work - R_C * pivot_C_in_work;
    Eigen::Vector3d compensation_B = pivot_B_in_work - R_B * pivot_B_in_work;
    Eigen::Vector3d total_compensation = compensation_C + compensation_B;

    // =====================================================
    // 第四步：转回机械坐标
    // =====================================================
    Eigen::Vector3d machine_before = g54_in_machine;
    Eigen::Vector3d machine_after = g54_in_machine
        + R_B * R_C * tool_tip_in_work + total_compensation;

    // =====================================================
    // 第五步：BC机械坐标（叠加G54录入时的角度）
    // =====================================================
    double machine_B = g54_B_angle + target_B;
    double machine_C = g54_C_angle + target_C;

    // =====================================================
    // 输出
    // =====================================================
    /*std::cout << std::fixed << std::setprecision(3);
    std::cout << "=== 输入参数 ===" << "\n";
    std::cout << "G54 in machine: " << g54_in_machine.transpose() << "\n";
    std::cout << "pivot_B in work: " << pivot_B_in_work.transpose() << "\n";
    std::cout << "pivot_C in work: " << pivot_C_in_work.transpose() << "\n\n";

    std::cout << "=== RTCP结果 ===" << "\n";
    std::cout << "compensation_C:  " << compensation_C.transpose() << "\n";
    std::cout << "compensation_B:  " << compensation_B.transpose() << "\n\n";
    std::cout << "机械坐标 初始时: "
        << "X=" << machine_before.x()
        << " Y=" << machine_before.y()
        << " Z=" << machine_before.z()
        << " B=" << g54_B_angle
        << " C=" << g54_C_angle << "\n";
    std::cout << "机械坐标 目标时: "
        << "X=" << machine_after.x()
        << " Y=" << machine_after.y()
        << " Z=" << machine_after.z()
        << " B=" << machine_B
        << " C=" << machine_C << "\n";*/
}
