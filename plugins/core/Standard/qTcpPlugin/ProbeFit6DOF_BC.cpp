#include "ProbeFit6DOF_BC.h"

#include "Eigen/SVD"

#include <cmath>
#include <iostream>
#include <stdexcept>

// ─────────────────────────────────────────────
// 基础工具
// ─────────────────────────────────────────────

double ProbeFit6DOF_BC::deg2rad(double deg)
{
	return deg * PI / 180.0;
}
double ProbeFit6DOF_BC::rad2deg(double rad)
{
	return rad * 180.0 / PI;
}

Eigen::Matrix3d ProbeFit6DOF_BC::rotY(double rad)
{
	double          c = std::cos(rad), s = std::sin(rad);
	Eigen::Matrix3d R;
	R << c, 0, s,
	    0, 1, 0,
	    -s, 0, c;
	return R;
}

Eigen::Matrix3d ProbeFit6DOF_BC::rotZ(double rad)
{
	double          c = std::cos(rad), s = std::sin(rad);
	Eigen::Matrix3d R;
	R << c, -s, 0,
	    s, c, 0,
	    0, 0, 1;
	return R;
}

Eigen::Matrix4d ProbeFit6DOF_BC::makeTransform(const Eigen::Matrix3d& R,
                                               const Eigen::Vector3d& t)
{
	Eigen::Matrix4d T   = Eigen::Matrix4d::Identity();
	T.block<3, 3>(0, 0) = R;
	T.block<3, 1>(0, 3) = t;
	return T;
}

Eigen::Matrix4d ProbeFit6DOF_BC::rotateAroundPoint(const Eigen::Matrix3d& R,
                                                   const Eigen::Vector3d& center)
{
	return makeTransform(Eigen::Matrix3d::Identity(), center)
	       * makeTransform(R, Eigen::Vector3d::Zero())
	       * makeTransform(Eigen::Matrix3d::Identity(), -center);
}

// ─────────────────────────────────────────────
// 构造 / 点管理 / 参数设置
// ─────────────────────────────────────────────

ProbeFit6DOF_BC::ProbeFit6DOF_BC(const G54Config&       g54,
                                 const Eigen::Vector3d& B_center,
                                 const Eigen::Vector3d& C_center,
                                 double                 signB,
                                 double                 signC)
    : g54_(g54)
    , B_center_machine_(B_center)
    , C_center_machine_(C_center)
    , kSignB_(signB)
    , kSignC_(signC)
{
}

void ProbeFit6DOF_BC::addPoint(const ProbePoint& pt)
{
	points_.push_back(pt);
}

void ProbeFit6DOF_BC::addPoint(const Eigen::Vector3d& nominal,
                               const Eigen::Vector3d& actual,
                               const Eigen::Vector3d& ijk,
                               double                 B_deg,
                               double                 C_deg)
{
	points_.push_back({nominal, actual, ijk, B_deg, C_deg});
}

void ProbeFit6DOF_BC::clearPoints()
{
	points_.clear();
}
size_t ProbeFit6DOF_BC::pointCount() const
{
	return points_.size();
}

void ProbeFit6DOF_BC::setG54(const G54Config& g54)
{
	g54_ = g54;
}
void ProbeFit6DOF_BC::setBCenter(const Eigen::Vector3d& c)
{
	B_center_machine_ = c;
}
void ProbeFit6DOF_BC::setCCenter(const Eigen::Vector3d& c)
{
	C_center_machine_ = c;
}

// ─────────────────────────────────────────────
// BC 摆角还原
// ─────────────────────────────────────────────

Eigen::Matrix4d ProbeFit6DOF_BC::buildBCTransform(double B_deg, double C_deg, const Eigen::Vector3d& Bc, const Eigen::Vector3d& Cc) const
{
	Eigen::Matrix3d Rb = rotY(kSignB_ * deg2rad(B_deg));
	Eigen::Matrix3d Rc = rotZ(kSignC_ * deg2rad(C_deg));
	return rotateAroundPoint(Rb, Bc) * rotateAroundPoint(Rc, Cc);
}

Eigen::Vector3d ProbeFit6DOF_BC::resetMeasureRotation(
    const Eigen::Vector3d& p,
    double                 Bd,
    double                 Cd,
    const G54Config&       g,
    const Eigen::Vector3d& Bm,
    const Eigen::Vector3d& Cm) const
{
	Eigen::Vector3d Bg = Bm - g.xyz;
	Eigen::Vector3d Cg = Cm - g.xyz;

	Eigen::Matrix4d M_base = buildBCTransform(g.B_deg, g.C_deg, Bg, Cg);
	Eigen::Matrix4d M_cur  = buildBCTransform(Bd + g.B_deg, Cd + g.C_deg, Bg, Cg);

	Eigen::Vector4d ph(p.x(), p.y(), p.z(), 1.0);
	Eigen::Matrix4d M_reset = M_cur * M_base.inverse();
	return (M_reset.inverse() * ph).head<3>();
}

Eigen::Vector3d ProbeFit6DOF_BC::resetIJKRotation(const Eigen::Vector3d& ijk,
                                                  double                 Bd,
                                                  double                 Cd,
                                                  const G54Config&       g) const
{
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
                                        Eigen::Matrix3d&       Rb,
                                        Eigen::Matrix3d&       Rc,
                                        double&                ry,
                                        double&                rz) const
{
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

bool ProbeFit6DOF_BC::solve(Result& result) const
{
	using namespace Eigen;
	const int N = static_cast<int>(points_.size());
	if (N < 6)
		return false;

	MatrixXd                nominal(N, 3), actual(N, 3), normals(N, 3);
	std::vector<ProbePoint> pts = points_;
	for (auto& p : pts)
		p.ijk.normalize();

	for (int i = 0; i < N; ++i)
	{
		nominal.row(i) = resetMeasureRotation(pts[i].nominal, pts[i].B_deg, pts[i].C_deg, g54_, B_center_machine_, C_center_machine_);
		actual.row(i)  = resetMeasureRotation(pts[i].actual, pts[i].B_deg, pts[i].C_deg, g54_, B_center_machine_, C_center_machine_);
		normals.row(i) = resetIJKRotation(pts[i].ijk, pts[i].B_deg, pts[i].C_deg, g54_);
	}

	VectorXd dev(N);
	for (int i = 0; i < N; ++i)
		dev(i) = normals.row(i).dot(actual.row(i) - nominal.row(i));

	MatrixXd A(N, 6);
	for (int i = 0; i < N; ++i)
	{
		Vector3d ni        = normals.row(i);
		Vector3d pi        = nominal.row(i);
		A.row(i).head<3>() = ni;
		A.row(i).tail<3>() = pi.cross(ni);
	}

	BDCSVD<MatrixXd, ComputeThinU | ComputeThinV> svd(A);
	VectorXd                                      x = svd.solve(dev);

	Vector3d omega(x(3), x(4), x(5));
	double   ang = omega.norm();
	Matrix3d R   = Matrix3d::Identity();
	if (ang > 1e-10)
	{
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
	VectorXd res  = dev - pred;

	result.R           = R;
	result.t           = -R.transpose() * t;
	result.centroid    = centroid;
	result.omega       = omega;
	result.rms         = std::sqrt(res.squaredNorm() / N);
	result.maxResidual = res.cwiseAbs().maxCoeff(&result.maxResidualIndex);
	result.residuals.resize(N);
	for (int i = 0; i < N; ++i)
		result.residuals[i] = res(i);
	return true;
}


bool ProbeFit6DOF_BC::solveKabsch(Result& result) const
{
	using namespace Eigen;
	const int N = static_cast<int>(points_.size());
	if (N < 3)
		return false;

	MatrixXd                nominal(N, 3), actual(N, 3);
	std::vector<ProbePoint> pts = points_;

	for (int i = 0; i < N; ++i)
	{
		nominal.row(i) = resetMeasureRotation(pts[i].nominal, pts[i].B_deg, pts[i].C_deg, g54_, B_center_machine_, C_center_machine_);
		actual.row(i)  = resetMeasureRotation(pts[i].actual, pts[i].B_deg, pts[i].C_deg, g54_, B_center_machine_, C_center_machine_);
	}

	// 质心
	Vector3d centP = nominal.colwise().mean();
	Vector3d centQ = actual.colwise().mean();

	// 去质心
	MatrixXd Pc = nominal.rowwise() - centP.transpose();
	MatrixXd Qc = actual.rowwise() - centQ.transpose();

	// SVD
	//Matrix3d            H = Pc.transpose() * Qc;
	Matrix3d            H = Qc.transpose() * Pc; // actual → nominal
	JacobiSVD<Matrix3d> svd(H, ComputeFullU | ComputeFullV);
	Matrix3d            U = svd.matrixU();
	Matrix3d            V = svd.matrixV();

	// 处理反射（det=-1的情况）
	Matrix3d D = Matrix3d::Identity();
	D(2, 2)    = (V * U.transpose()).determinant();

	Matrix3d R = V * D * U.transpose();

	Vector3d t = centP - R * centQ; // t使 R·actual + t ≈ nominal

	// 提取欧拉角 ZYX：C(Rz) B(Ry) A(Rx)
	double angleC = atan2(R(1, 0), R(0, 0));
	double angleB = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
	double angleA = atan2(R(2, 1), R(2, 2));

	// 残差（point-to-point 欧氏距离）
	result.residuals.resize(N);
	double sumSq  = 0.0;
	double maxRes = 0.0;
	int    maxIdx = 0;
	for (int i = 0; i < N; ++i)
	{
		Vector3d err = nominal.row(i).transpose() - (R * actual.row(i).transpose() + t);

		result.residuals[i] = err.norm();
		sumSq += result.residuals[i] * result.residuals[i];
		if (result.residuals[i] > maxRes)
		{
			maxRes = result.residuals[i];
			maxIdx = i;
		}
	}

	result.R                = R;
	result.t                = t;
	result.centroid         = centP;
	result.omega            = Vector3d(angleA, angleB, angleC); // 复用omega字段存欧拉角
	result.rms              = sqrt(sumSq / N);
	result.maxResidual      = maxRes;
	result.maxResidualIndex = maxIdx;

	// NG判断
	/*double toDeg  = 180.0 / M_PI;
	result.angleA = angleA * toDeg;
	result.angleB = angleB * toDeg;
	result.angleC = angleC * toDeg;
	result.isNG   = (fabs(result.angleA) > toleranceA_deg_) || (fabs(result.angleB) > toleranceB_deg_);*/

	return true;
}

// ─────────────────────────────────────────────
// 双管电极补偿计算
// ─────────────────────────────────────────────

/**
 * fitTubeAxis — 从 N 个点（N≥2）拟合空间直线轴线
 *
 * 算法：PCA / SVD
 *   1. 计算点云质心 centroid
 *   2. 对去质心矩阵做 JacobiSVD，第一右奇异向量即主方向 dir
 *   3. 将每个点投影到轴线（标量参数 t = (p-c)·dir），取 min/max
 *   4. origin = c + t_min * dir，endpoint = c + t_max * dir
 */
ProbeFit6DOF_BC::TubeAxis
ProbeFit6DOF_BC::fitTubeAxis(const std::vector<Eigen::Vector3d>& pts)
{
    using namespace Eigen;

    const int N = static_cast<int>(pts.size());

    // 1. 质心
    Vector3d centroid = Vector3d::Zero();
    for (const auto& p : pts)
        centroid += p;
    centroid /= N;

    // 2. 去质心矩阵 (N×3)
    MatrixXd A(N, 3);
    for (int i = 0; i < N; ++i)
        A.row(i) = (pts[i] - centroid).transpose();

    // 3. SVD — 第一列右奇异向量 = 主方向
    JacobiSVD<MatrixXd> svd(A, ComputeThinV);
    Vector3d            dir = svd.matrixV().col(0).normalized();

    // 4. 投影区间
    double tMin = std::numeric_limits<double>::max();
    double tMax = -std::numeric_limits<double>::max();
    for (const auto& p : pts)
    {
        double t = (p - centroid).dot(dir);
        if (t < tMin) tMin = t;
        if (t > tMax) tMax = t;
    }

    TubeAxis ax;
    ax.midpoint  = centroid;
    ax.direction = dir;
    ax.origin    = centroid + tMin * dir;
    ax.endpoint  = centroid + tMax * dir;
    return ax;
}

/**
 * mergeTubeAxes — 将同一根管的两段轴线（已各自拟合好）合并为一条整体轴线
 *
 * 策略：
 *   1. 合并两段所有点（用 origin/endpoint/midpoint 近似代表各段极值）
 *   2. 对 6 个代表点重新调用 fitTubeAxis
 *
 * 这里直接把两段的 origin、midpoint、endpoint 拼成 6 个点再 fitTubeAxis，
 * 等效于对整管做一次 PCA。
 */
static ProbeFit6DOF_BC::TubeAxis mergeTubeAxes(const ProbeFit6DOF_BC::TubeAxis& seg1,
                                                const ProbeFit6DOF_BC::TubeAxis& seg2)
{
    std::vector<Eigen::Vector3d> pts = {
        seg1.origin, seg1.midpoint, seg1.endpoint,
        seg2.origin, seg2.midpoint, seg2.endpoint
    };
    return ProbeFit6DOF_BC::fitTubeAxis(pts);
}

/**
 * computeElectrodeFromPoints — 顶层接口
 *
 * 每根管提供两段点云（每段通常 8 点），共 4 × N_seg 个实测点 + 同样数量理论点。
 *
 * 步骤：
 *   1. 对 8 段点云分别调用 fitTubeAxis()
 *   2. 对同一管的两段轴线调用 mergeTubeAxes()，得到 4 条整管轴线
 *   3. 调用 computeElectrodeCompensation()
 */
ProbeFit6DOF_BC::ElectrodeResult ProbeFit6DOF_BC::computeElectrodeFromPoints(
    const std::vector<Eigen::Vector3d>& tube1_actual_seg1,
    const std::vector<Eigen::Vector3d>& tube1_actual_seg2,
    const std::vector<Eigen::Vector3d>& tube2_actual_seg1,
    const std::vector<Eigen::Vector3d>& tube2_actual_seg2,
    const std::vector<Eigen::Vector3d>& tube1_nominal_seg1,
    const std::vector<Eigen::Vector3d>& tube1_nominal_seg2,
    const std::vector<Eigen::Vector3d>& tube2_nominal_seg1,
    const std::vector<Eigen::Vector3d>& tube2_nominal_seg2,
    double                              nominalSpacing,
    double                              parallelTol_deg,
    double                              spacingTol_mm)
{
    // ── 1. 每段拟合轴线 ──────────────────────────────────────────────────────
    TubeAxis t1a_s1 = fitTubeAxis(tube1_actual_seg1);
    TubeAxis t1a_s2 = fitTubeAxis(tube1_actual_seg2);
    TubeAxis t2a_s1 = fitTubeAxis(tube2_actual_seg1);
    TubeAxis t2a_s2 = fitTubeAxis(tube2_actual_seg2);

    TubeAxis t1n_s1 = fitTubeAxis(tube1_nominal_seg1);
    TubeAxis t1n_s2 = fitTubeAxis(tube1_nominal_seg2);
    TubeAxis t2n_s1 = fitTubeAxis(tube2_nominal_seg1);
    TubeAxis t2n_s2 = fitTubeAxis(tube2_nominal_seg2);

    // ── 2. 合并两段为整管轴线 ────────────────────────────────────────────────
    TubeAxis tube1Actual  = mergeTubeAxes(t1a_s1, t1a_s2);
    TubeAxis tube2Actual  = mergeTubeAxes(t2a_s1, t2a_s2);
    TubeAxis tube1Nominal = mergeTubeAxes(t1n_s1, t1n_s2);
    TubeAxis tube2Nominal = mergeTubeAxes(t2n_s1, t2n_s2);

    // ── 3. 调用已有接口计算补偿量 ────────────────────────────────────────────
    return computeElectrodeCompensation(
        tube1Actual.origin,   tube1Actual.endpoint,
        tube2Actual.origin,   tube2Actual.endpoint,
        tube1Nominal.origin,  tube1Nominal.endpoint,
        tube2Nominal.origin,  tube2Nominal.endpoint,
        nominalSpacing,
        parallelTol_deg,
        spacingTol_mm);
}

/**
 * computeElectrodeCompensation
 *
 * 算法说明：
 *   1. 由两端点构造各管轴线（方向向量 & 中点）
 *   2. 平行度 parallelAngleDeg：两实测轴线方向向量的夹角（acos of dot product）
 *   3. 实测间距 actualSpacing：两管中点连线在 XY 平面内的投影长度
 *   4. deltaC：两管实测中点连线方向与理论中点连线方向在 XY 平面内的夹角（绕 Z 轴）
 *      公式：deltaC = atan2(id × in, id · in)，其中
 *            id = (actualMid2 - actualMid1).normalized() 在 XY 平面的投影
 *            in = (nominalMid2 - nominalMid1).normalized() 在 XY 平面的投影
 *   5. deltaY：两管实测中点均值 与 理论中点均值 的 Y 方向偏差
 *      公式：deltaY = actualMidAvg.y() - nominalMidAvg.y()
 *   6. deltaZ：两管实测中点均值 与 理论中点均值 的 Z 方向偏差
 *      公式：deltaZ = actualMidAvg.z() - nominalMidAvg.z()
 */
ProbeFit6DOF_BC::ElectrodeResult ProbeFit6DOF_BC::computeElectrodeCompensation(
    const Eigen::Vector3d& tube1A_actual,
    const Eigen::Vector3d& tube1B_actual,
    const Eigen::Vector3d& tube2A_actual,
    const Eigen::Vector3d& tube2B_actual,
    const Eigen::Vector3d& tube1A_nominal,
    const Eigen::Vector3d& tube1B_nominal,
    const Eigen::Vector3d& tube2A_nominal,
    const Eigen::Vector3d& tube2B_nominal,
    double                 nominalSpacing,
    double                 parallelTol_deg,
    double                 spacingTol_mm)
{
    using namespace Eigen;

    ElectrodeResult res;

    // ── 1. 构造轴线 ──────────────────────────────────────────────────────────

    auto makeAxis = [](const Vector3d& A, const Vector3d& B) -> TubeAxis {
        TubeAxis ax;
        ax.origin    = A;
        ax.endpoint  = B;
        ax.direction = (B - A).normalized();
        ax.midpoint  = 0.5 * (A + B);
        return ax;
    };

    res.tube1Actual  = makeAxis(tube1A_actual,  tube1B_actual);
    res.tube2Actual  = makeAxis(tube2A_actual,  tube2B_actual);
    res.tube1Nominal = makeAxis(tube1A_nominal, tube1B_nominal);
    res.tube2Nominal = makeAxis(tube2A_nominal, tube2B_nominal);

    // ── 2. 平行度：两实测轴线夹角 ────────────────────────────────────────────

    double cosAngle       = std::clamp(std::abs(res.tube1Actual.direction.dot(res.tube2Actual.direction)), 0.0, 1.0);
    res.parallelAngleDeg  = rad2deg(std::acos(cosAngle));
    res.parallelOK        = (res.parallelAngleDeg <= parallelTol_deg);

    // ── 3. 实测间距（XY 平面投影）────────────────────────────────────────────

    Vector3d midDiff_actual = res.tube2Actual.midpoint - res.tube1Actual.midpoint;
    // XY 平面投影长度
    res.actualSpacing = Vector2d(midDiff_actual.x(), midDiff_actual.y()).norm();
    res.spacingError  = res.actualSpacing - nominalSpacing;
    res.spacingOK     = (std::abs(res.spacingError) <= spacingTol_mm);

    // ── 4. deltaC：绕 Z 轴旋转偏差 ──────────────────────────────────────────
    //   id = 实测中点连线在 XY 平面的单位向量
    //   in = 理论中点连线在 XY 平面的单位向量
    //   deltaC = signed angle from in to id（右手 Z+ 为正）

    Vector3d midDiff_nominal = res.tube2Nominal.midpoint - res.tube1Nominal.midpoint;

    Vector2d id2(midDiff_actual.x(),  midDiff_actual.y());
    Vector2d in2(midDiff_nominal.x(), midDiff_nominal.y());

    // 叉积的 z 分量（2D）= id2.x()*in2.y() - id2.y()*in2.x()
    // 点积
    double cross2d = id2.x() * in2.y() - id2.y() * in2.x();
    double dot2d   = id2.dot(in2);
    // deltaC: 需要把 id 转回到 in 的方向，补偿量 = -angle(id, in)
    res.deltaC = rad2deg(std::atan2(-cross2d, dot2d));

    // ── 5. deltaY / deltaZ：中点均值的平移偏差 ───────────────────────────────

    Vector3d actualMidAvg  = 0.5 * (res.tube1Actual.midpoint  + res.tube2Actual.midpoint);
    Vector3d nominalMidAvg = 0.5 * (res.tube1Nominal.midpoint + res.tube2Nominal.midpoint);

    res.deltaY = actualMidAvg.y() - nominalMidAvg.y();
    res.deltaZ = actualMidAvg.z() - nominalMidAvg.z();

    return res;
}

// ─────────────────────────────────────────────
// 补偿计算（分轴剥离火花机旋转中心）
// ─────────────────────────────────────────────

ProbeFit6DOF_BC::Compensation
ProbeFit6DOF_BC::computeCompensation(const Result&          result,
                                     const Eigen::Vector3d& B_center_machine,
                                     const Eigen::Vector3d& C_center_machine,
                                     const Eigen::Vector3d& sparkG54_xyz,
                                     const Eigen::Vector2d& spindleC_eccentricity,
                                     double                 sparkSignA,
                                     double                 sparkSignB,
                                     double                 sparkSignC) // 主轴C偏心 (ex, ey)
{
	using namespace Eigen;

	const Matrix3d& R = result.R;

	// ZYZ分解 R = Rz(a) * Ry(b) * Rz(c)
	// a = 主轴C, b = 转台B, c = 转台C
	double b = std::acos(std::clamp(R(2, 2), -1.0, 1.0));
	// acos永远非负，用omega.y()的符号修正
	if (result.omega.y() < 0)
		b = -b;

	double a, c;
	if (std::abs(std::sin(b)) < 1e-1)
	{
		// 万向锁：sin(b)≈0，a和c无法分离，只能求a+c或a-c
		// 约定a=0，全部归给c
		a = 0.0;
		if (R(2, 2) > 0)
		{
			// b≈0: R≈Rz(a+c)
			c = std::atan2(-R(0, 1), R(0, 0));
		}
		else
		{
			// b≈π: R≈Rz(a-c)，令a=0则c = -atan2(R(0,1), -R(0,0))
			c = std::atan2(-R(0, 1), R(0, 0));
		}
	}
	else
	{
		b = std::acos(std::clamp(R(2, 2), -1.0, 1.0));
		if (result.omega.y() < 0)
			b = -b;
		c = std::atan2(R(2, 1), -R(2, 0)); // atan2(sinb*sinc, -sinb*cosc)
		a = std::atan2(R(1, 2), R(0, 2));  // atan2(sina*sinb,  cosa*sinb)
	}

	// 2. 旋转中心在火花机G54下的坐标
	Vector3d B_pivot = B_center_machine - sparkG54_xyz;
	Vector3d C_pivot = C_center_machine - sparkG54_xyz;
	// 3. 构造各轴旋转矩阵
	Matrix3d Ra = AngleAxisd(a, Vector3d::UnitZ()).toRotationMatrix();
	Matrix3d Rb = AngleAxisd(b, Vector3d::UnitY()).toRotationMatrix();
	Matrix3d Rc = AngleAxisd(c, Vector3d::UnitZ()).toRotationMatrix();

	// 4. 剥离旋转中心引入的等效平移
	Vector3d t1 = result.t - (Matrix3d::Identity() - Rc) * C_pivot;
	Vector3d t2 = t1 - (Matrix3d::Identity() - Rb) * B_pivot;
	std::cout << t2 << std::endl;

	// 5. 主轴C偏心引入的等效平移
	Vector3d ecc(spindleC_eccentricity.x(), spindleC_eccentricity.y(), 0.0);
	Vector3d t_true = t2 - (Matrix3d::Identity() - Ra) * ecc;

	std::cout << t_true << std::endl;
	// 5. 输出
	Compensation comp;
	comp.R      = R;
	comp.t_true = t_true;

	comp.B_comp_deg = rad2deg(b);
	comp.C_comp_deg = rad2deg(c);
	comp.A_comp_deg = rad2deg(a); // 主轴C

	comp.B_machine_comp_deg = sparkSignB * (-comp.B_comp_deg);
	comp.C_machine_comp_deg = sparkSignC * (-comp.C_comp_deg);
	comp.A_machine_comp_deg = sparkSignA * (-comp.A_comp_deg); // 主轴C符号待确认

	return comp;
}
