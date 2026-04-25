#ifndef PROBE_FIT_6DOF_BC_H
#define PROBE_FIT_6DOF_BC_H

#include "Eigen/Dense"
#include <vector>
#include <cstddef>

/**
 * 测头数据 6DOF 刚体变换拟合（含 BC 轴摆角还原）
 *
 * 适用场景：
 *   测量机测量工件 → 拟合 6DOF 偏差 → 火花机补偿
 *
 * 特点：
 *   - B/C 轴独立旋转中心
 *   - 分轴剥离旋转中心伴随平移（支持 B/C 旋转中心不同）
 *   - SVD 求解旋转中心 = 测点质心
 */
class ProbeFit6DOF_BC
{
public:
    // ─────────────────────────────────────────
    // 常量
    // ─────────────────────────────────────────
    static constexpr double PI = 3.14159265358979323846;

    // ─────────────────────────────────────────
    // 数据结构
    // ─────────────────────────────────────────

    struct ProbePoint {
        Eigen::Vector3d nominal;  // 理论点 XYZ（G54 坐标系）
        Eigen::Vector3d actual;   // 实际点 XYZ（G54 坐标系）
        Eigen::Vector3d ijk;      // 法向（当前 BC 姿态下的测量坐标系）
        double B_deg = 0.0;       // 打点时 G54 下的 B 摆角（增量）
        double C_deg = 0.0;       // 打点时 G54 下的 C 摆角（增量）
    };

    struct G54Config {
        Eigen::Vector3d xyz;       // G54 的 XYZ 偏置（机床坐标系）
        double B_deg = 0.0;        // G54 的 B 角偏置（度）
        double C_deg = 0.0;        // G54 的 C 角偏置（度）
    };

    struct Result {
        Eigen::Matrix3d R;             // 旋转矩阵（G54 零姿态下，绕质心）
        Eigen::Vector3d t;             // 平移向量（绕质心）
        Eigen::Vector3d centroid;      // 测点质心（G54 零姿态下）
        Eigen::Vector3d omega;         // SVD 旋转向量 (rx, ry, rz) [rad]
        double rms = 0.0;
        double maxResidual = 0.0;
        int    maxResidualIndex = 0;
        std::vector<double> residuals;
        int dof = 6;
    };

    /**
     * 剥离旋转中心后的补偿结果
     */
    struct Compensation {
        Eigen::Matrix3d R;         // 旋转矩阵
        Eigen::Vector3d t_true;    // 真实平移偏差
        double B_comp_deg;         // B 轴补偿角度（度）
        double C_comp_deg;         // C 轴补偿角度（度）
        double B_machine_comp_deg;      // 火花机实际补偿角度（已考虑符号）
        double C_machine_comp_deg;
    };

    // ─────────────────────────────────────────
    // 构造
    // ─────────────────────────────────────────

    /**
     * @param g54               测量机 G54 偏置
     * @param B_center_machine  B 轴旋转中心（测量机机床坐标系）
     * @param C_center_machine  C 轴旋转中心（测量机机床坐标系）
     * @param signB             B 轴旋转方向符号（默认 -1.0）
     * @param signC             C 轴旋转方向符号（默认 -1.0）
     */
    ProbeFit6DOF_BC(const G54Config& g54,
        const Eigen::Vector3d& B_center_machine,
        const Eigen::Vector3d& C_center_machine,
        double signB = -1.0,
        double signC = -1.0);

    // ─────────────────────────────────────────
    // 点管理
    // ─────────────────────────────────────────

    void addPoint(const ProbePoint& pt);
    void addPoint(const Eigen::Vector3d& nominal,
        const Eigen::Vector3d& actual,
        const Eigen::Vector3d& ijk,
        double B_deg = 0.0,
        double C_deg = 0.0);
    void clearPoints();
    size_t pointCount() const;

    // ─────────────────────────────────────────
    // 参数设置
    // ─────────────────────────────────────────

    void setG54(const G54Config& g54);
    void setBCenter(const Eigen::Vector3d& c);
    void setCCenter(const Eigen::Vector3d& c);

    // ─────────────────────────────────────────
    // 求解
    // ─────────────────────────────────────────

    /**
     * 执行 SVD 拟合
     * @return Result（旋转中心 = 测点质心）
     */
    Result solve() const;

    /**
     * 剥离指定火花机旋转中心，得到最终补偿值
     *
     * @param result          solve() 的返回值
     * @param B_center_machine 火花机 B 轴旋转中心（机床坐标系）
     * @param C_center_machine 火花机 C 轴旋转中心（机床坐标系）
     * @param sparkG54_xyz     火花机 G54 的 XYZ 偏置（机床坐标系）
     * @return Compensation    旋转矩阵 + 真实平移 + BC 补偿角
     */
    static Compensation computeCompensation(const Result& result,
        const Eigen::Vector3d& B_center_machine,
        const Eigen::Vector3d& C_center_machine,
        const Eigen::Vector3d& sparkG54_xyz,
        double sparkSignB = -1.0,
        double sparkSignC = -1.0);

    // ─────────────────────────────────────────
    // 基础工具（公开，供外部使用）
    // ─────────────────────────────────────────

    static double deg2rad(double deg);
    static double rad2deg(double rad);

private:
    void decomposeRotation(const Eigen::Matrix3d& R,
        Eigen::Matrix3d& Rb,
        Eigen::Matrix3d& Rc,
        double& ry,
        double& rz) const;

    static Eigen::Matrix3d rotY(double rad);
    static Eigen::Matrix3d rotZ(double rad);
    static Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R,
        const Eigen::Vector3d& t);
    static Eigen::Matrix4d rotateAroundPoint(const Eigen::Matrix3d& R,
        const Eigen::Vector3d& center);

    Eigen::Matrix4d buildBCTransform(double B_actual_deg,
        double C_actual_deg,
        const Eigen::Vector3d& B_center,
        const Eigen::Vector3d& C_center) const;

    Eigen::Vector3d resetMeasureRotation(const Eigen::Vector3d& p_g54,
        double B_deg,
        double C_deg,
        const G54Config& g54,
        const Eigen::Vector3d& B_center_machine,
        const Eigen::Vector3d& C_center_machine) const;

    Eigen::Vector3d resetIJKRotation(const Eigen::Vector3d& ijk,
        double B_deg,
        double C_deg,
        const G54Config& g54) const;

    G54Config g54_;
    Eigen::Vector3d B_center_machine_;
    Eigen::Vector3d C_center_machine_;
    double kSignB_;
    double kSignC_;
    std::vector<ProbePoint> points_;
};

#endif // PROBE_FIT_6DOF_BC_H