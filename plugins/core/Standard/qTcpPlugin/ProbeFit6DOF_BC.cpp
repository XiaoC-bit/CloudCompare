#include "ProbeFit6DOF_BC.h"
#include "Eigen/SVD"
#include <cmath>
#include <stdexcept>

// ─────────────────────────────────────────────
// 基础工具
// ─────────────────────────────────────────────

double ProbeFit6DOF_BC::deg2rad(double deg) { return deg * PI / 180.0; }
double ProbeFit6DOF_BC::rad2deg(double rad) { return rad * 180.0 / PI; }

Eigen::Matrix3d ProbeFit6DOF_BC::rotY(double rad) {
    double c = std::cos(rad), s = std::sin(rad);
    Eigen::Matrix3d R;
    R << c, 0, s,
        0, 1, 0,
        -s, 0, c;
    return R;
}

Eigen::Matrix3d ProbeFit6DOF_BC::rotZ(double rad) {
    double c = std::cos(rad), s = std::sin(rad);
    Eigen::Matrix3d R;
    R << c, -s, 0,
        s, c, 0,
        0, 0, 1;
    return R;
}

Eigen::Matrix4d ProbeFit6DOF_BC::makeTransform(const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

Eigen::Matrix4d ProbeFit6DOF_BC::rotateAroundPoint(const Eigen::Matrix3d& R,
    const Eigen::Vector3d& center) {
    return makeTransform(Eigen::Matrix3d::Identity(), center)
        * makeTransform(R, Eigen::Vector3d::Zero())
        * makeTransform(Eigen::Matrix3d::Identity(), -center);
}

// ─────────────────────────────────────────────
// 构造 / 点管理 / 参数设置
// ─────────────────────────────────────────────

ProbeFit6DOF_BC::ProbeFit6DOF_BC(const G54Config& g54,
    const Eigen::Vector3d& B_center,
    const Eigen::Vector3d& C_center,
    double signB,
    double signC)
    : g54_(g54)
    , B_center_machine_(B_center)
    , C_center_machine_(C_center)
    , kSignB_(signB)
    , kSignC_(signC)
{}

void ProbeFit6DOF_BC::addPoint(const ProbePoint& pt) {
    points_.push_back(pt);
}

void ProbeFit6DOF_BC::addPoint(const Eigen::Vector3d& nominal,
    const Eigen::Vector3d& actual,
    const Eigen::Vector3d& ijk,
    double B_deg,
    double C_deg) {
    points_.push_back({ nominal, actual, ijk, B_deg, C_deg });
}

void ProbeFit6DOF_BC::clearPoints() { points_.clear(); }
size_t ProbeFit6DOF_BC::pointCount() const { return points_.size(); }

void ProbeFit6DOF_BC::setG54(const G54Config& g54) { g54_ = g54; }
void ProbeFit6DOF_BC::setBCenter(const Eigen::Vector3d& c) { B_center_machine_ = c; }
void ProbeFit6DOF_BC::setCCenter(const Eigen::Vector3d& c) { C_center_machine_ = c; }

// ─────────────────────────────────────────────
// BC 摆角还原
// ─────────────────────────────────────────────

Eigen::Matrix4d ProbeFit6DOF_BC::buildBCTransform(double B_deg, double C_deg,
    const Eigen::Vector3d& Bc,
    const Eigen::Vector3d& Cc) const {
    Eigen::Matrix3d Rb = rotY(kSignB_ * deg2rad(B_deg));
    Eigen::Matrix3d Rc = rotZ(kSignC_ * deg2rad(C_deg));
    return rotateAroundPoint(Rb, Bc) * rotateAroundPoint(Rc, Cc);
}

Eigen::Vector3d ProbeFit6DOF_BC::resetMeasureRotation(
    const Eigen::Vector3d& p, double Bd, double Cd,
    const G54Config& g, const Eigen::Vector3d& Bm, const Eigen::Vector3d& Cm) const
{
    Eigen::Vector3d Bg = Bm - g.xyz;
    Eigen::Vector3d Cg = Cm - g.xyz;

    Eigen::Matrix4d M_base = buildBCTransform(g.B_deg, g.C_deg, Bg, Cg);
    Eigen::Matrix4d M_cur = buildBCTransform(Bd + g.B_deg, Cd + g.C_deg, Bg, Cg);

    Eigen::Vector4d ph(p.x(), p.y(), p.z(), 1.0);
    Eigen::Matrix4d M_reset = M_cur * M_base.inverse();
    return (M_reset.inverse() * ph).head<3>();
}

Eigen::Vector3d ProbeFit6DOF_BC::resetIJKRotation(const Eigen::Vector3d& ijk,
    double Bd, double Cd,
    const G54Config& g) const {
    Eigen::Matrix3d R_cur = rotY(kSignB_ * deg2rad(Bd + g.B_deg))
        * rotZ(kSignC_ * deg2rad(Cd + g.C_deg));
    Eigen::Matrix3d R_base = rotY(kSignB_ * deg2rad(g.B_deg))
        * rotZ(kSignC_ * deg2rad(g.C_deg));
    Eigen::Matrix3d R_net = R_cur * R_base.transpose();
    return (R_net.transpose() * ijk).normalized();
}

// ─────────────────────────────────────────────
// 旋转矩阵分解
// ─────────────────────────────────────────────

void ProbeFit6DOF_BC::decomposeRotation(const Eigen::Matrix3d& R,
    Eigen::Matrix3d& Rb,
    Eigen::Matrix3d& Rc,
    double& ry,
    double& rz) const {
    // R = Rb * Rc（先 C 后 B 的运动链）
    // Rc = rotZ(rz), Rb = rotY(ry)
    rz = std::atan2(R(1, 0), R(0, 0));
    Rc = Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Rb = R * Rc.transpose();
    ry = std::atan2(Rb(2, 0), Rb(0, 0));
}

// ─────────────────────────────────────────────
// SVD 求解
// ─────────────────────────────────────────────

ProbeFit6DOF_BC::Result ProbeFit6DOF_BC::solve() const
{
    using namespace Eigen;
    const int N = static_cast<int>(points_.size());
    if (N < 6) throw std::runtime_error("ProbeFit6DOF_BC: at least 6 points required");

    MatrixXd nominal(N, 3), actual(N, 3), normals(N, 3);
    std::vector<ProbePoint> pts = points_;
    for (auto& p : pts) p.ijk.normalize();

    for (int i = 0; i < N; ++i) {
        nominal.row(i) = resetMeasureRotation(pts[i].nominal, pts[i].B_deg, pts[i].C_deg,
            g54_, B_center_machine_, C_center_machine_);
        actual.row(i) = resetMeasureRotation(pts[i].actual, pts[i].B_deg, pts[i].C_deg,
            g54_, B_center_machine_, C_center_machine_);
        normals.row(i) = resetIJKRotation(pts[i].ijk, pts[i].B_deg, pts[i].C_deg, g54_);
    }

    VectorXd dev(N);
    for (int i = 0; i < N; ++i)
        dev(i) = normals.row(i).dot(actual.row(i) - nominal.row(i));

    MatrixXd A(N, 6);
    for (int i = 0; i < N; ++i) {
        Vector3d ni = normals.row(i);
        Vector3d pi = nominal.row(i);
        A.row(i).head<3>() = ni;
        A.row(i).tail<3>() = pi.cross(ni);
    }

    BDCSVD<MatrixXd, ComputeThinU | ComputeThinV> svd(A);
    VectorXd x = svd.solve(dev);

    Vector3d omega(x(3), x(4), x(5));
    double   ang = omega.norm();
    Matrix3d R = Matrix3d::Identity();
    if (ang > 1e-10) {
        Vector3d k = omega / ang;
        Matrix3d K;
        K << 0, -k(2), k(1),
            k(2), 0, -k(0),
            -k(1), k(0), 0;
        R = Matrix3d::Identity() + std::sin(ang) * K + (1 - std::cos(ang)) * K * K;
    }
    Vector3d t(x(0), x(1), x(2));

    Vector3d centroid = nominal.colwise().mean();

    VectorXd pred = A * x;
    VectorXd res = dev - pred;

    Result r;
    r.R = R;
    r.t = t;
    r.centroid = centroid;
    r.omega = omega;
    r.rms = std::sqrt(res.squaredNorm() / N);
    r.maxResidual = res.cwiseAbs().maxCoeff(&r.maxResidualIndex);
    r.residuals.resize(N);
    for (int i = 0; i < N; ++i) r.residuals[i] = res(i);
    return r;
}

// ─────────────────────────────────────────────
// 补偿计算（分轴剥离火花机旋转中心）
// ─────────────────────────────────────────────

ProbeFit6DOF_BC::Compensation
ProbeFit6DOF_BC::computeCompensation(const Result& result,
    const Eigen::Vector3d& B_center_machine,
    const Eigen::Vector3d& C_center_machine,
    const Eigen::Vector3d& sparkG54_xyz,
    double sparkSignB,
    double sparkSignC)
{
    using namespace Eigen;

    // 1. 分解 R = Rb * Rc
    double rz = std::atan2(result.R(1, 0), result.R(0, 0));
    Matrix3d Rc = AngleAxisd(rz, Vector3d::UnitZ()).toRotationMatrix();
    Matrix3d Rb = result.R * Rc.transpose();
    double ry = std::atan2(Rb(2, 0), Rb(0, 0));

    // 2. 旋转中心在火花机 G54 下的坐标
    Vector3d B_pivot = B_center_machine - sparkG54_xyz;
    Vector3d C_pivot = C_center_machine - sparkG54_xyz;

    // 3. 分轴剥离
    Vector3d t1 = result.t - (Matrix3d::Identity() - Rc) * C_pivot;
    Vector3d t_true = t1 - (Matrix3d::Identity() - Rb) * B_pivot;

    // 4. 输出
    Compensation comp;
    comp.R = result.R;
    comp.t_true = t_true;

    // 工件在标准坐标系下的旋转量
    comp.B_comp_deg = rad2deg(ry);
    comp.C_comp_deg = rad2deg(rz);

    // 火花机实际补偿角度（补偿 = -偏差，再乘以机床符号）
    // 测量机拟合出的 ry/rz 是工件偏差
    // 机床补偿 = -偏差（把工件转回去）
    // 考虑机床符号：机床实际编码 = sparkSign * 补偿角
    comp.B_machine_comp_deg = sparkSignB * (-comp.B_comp_deg);
    comp.C_machine_comp_deg = sparkSignC * (-comp.C_comp_deg);

    return comp;
}