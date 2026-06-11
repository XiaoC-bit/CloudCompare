#ifndef PROBE_FIT_6DOF_BC_H
#define PROBE_FIT_6DOF_BC_H

#include "Eigen/Dense"

#include <cstddef>
#include <vector>

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

	struct ProbePoint
	{
		Eigen::Vector3d nominal;     // 理论点 XYZ（G54 坐标系）
		Eigen::Vector3d actual;      // 实际点 XYZ（G54 坐标系）
		Eigen::Vector3d ijk;         // 法向（当前 BC 姿态下的测量坐标系）
		double          B_deg = 0.0; // 打点时 G54 下的 B 摆角（增量）
		double          C_deg = 0.0; // 打点时 G54 下的 C 摆角（增量）
	};

	struct G54Config
	{
		Eigen::Vector3d xyz;         // G54 的 XYZ 偏置（机床坐标系）
		double          B_deg = 0.0; // G54 的 B 角偏置（度）
		double          C_deg = 0.0; // G54 的 C 角偏置（度）
	};

	struct Result
	{
		Eigen::Matrix3d     R;        // 旋转矩阵（G54 零姿态下，绕质心）
		Eigen::Vector3d     t;        // 平移向量（绕质心）
		Eigen::Vector3d     centroid; // 测点质心（G54 零姿态下）
		Eigen::Vector3d     omega;    // SVD 旋转向量 (rx, ry, rz) [rad]
		double              rms              = 0.0;
		double              maxResidual      = 0.0;
		int                 maxResidualIndex = 0;
		std::vector<double> residuals;
		int                 dof = 6;
	};

	/**
	 * 剥离旋转中心后的补偿结果
	 */
	struct Compensation
	{
		Eigen::Matrix3d R;          // 旋转矩阵
		Eigen::Vector3d t_true;     // 真实平移偏差
		double          B_comp_deg; // B 轴补偿角度（度）
		double          C_comp_deg; // C 轴补偿角度（度）
		double          A_comp_deg;
		double          B_machine_comp_deg; // 火花机实际补偿角度（已考虑符号）
		double          C_machine_comp_deg;
		double          A_machine_comp_deg;
	};

	// ─────────────────────────────────────────
	// 电极管轴线结构（用于双管电极计算）
	// ─────────────────────────────────────────

	/**
	 * 管轴线：由两点定义的空间直线
	 *   origin  — 轴线上靠近工件端的点（测量点之一）
	 *   direction — 单位方向向量（从 origin 指向另一端）
	 *   midpoint  — 两测量点的中点
	 */
	struct TubeAxis
	{
		Eigen::Vector3d origin;    // 轴线起点（端点 A）
		Eigen::Vector3d endpoint;  // 轴线终点（端点 B）
		Eigen::Vector3d direction; // 单位方向向量 (B-A).normalized()
		Eigen::Vector3d midpoint;  // (A+B)/2
	};

	/**
	 * 双管电极测量结果 & 补偿量
	 *
	 * 坐标系约定（与调用方一致）：
	 *   X — 机床左右（或工件宽度方向）
	 *   Y — 两管间距方向
	 *   Z — 电极轴线主方向（进给方向）
	 *
	 * 补偿量定义：
	 *   deltaC  — 绕 Z 轴的旋转偏差 [deg]，正值 = 需顺时针补偿
	 *   deltaY  — 两管中点连线在 Y 方向的平移偏差 [mm]
	 *   deltaZ  — 两管中点在 Z 方向的平移偏差 [mm]
	 */
	struct ElectrodeResult
	{
		TubeAxis tube1Actual;   // 管1 实测轴线
		TubeAxis tube2Actual;   // 管2 实测轴线

		TubeAxis tube1Nominal;  // 管1 理论轴线
		TubeAxis tube2Nominal;  // 管2 理论轴线

		double parallelAngleDeg; // 两管实测轴线夹角 [deg]（理想为 0）

		double actualSpacing;    // 实测中心间距 [mm]（两管中点连线长度在 XY 平面投影）
		double spacingError;     // 间距误差 = actualSpacing - nominalSpacing [mm]

		double deltaC;           // C 轴旋转补偿量 [deg]
		double deltaY;           // Y 方向平移补偿量 [mm]
		double deltaZ;           // Z 方向平移补偿量 [mm]

		bool parallelOK;         // 平行度是否在公差内
		bool spacingOK;          // 间距是否在公差内
	};

	// ─────────────────────────────────────────
	// 双管电极补偿计算
	// ─────────────────────────────────────────

	/**
	 * 从一组散点拟合空间直线轴线（主成分方向，PCA/SVD）
	 *
	 * 输入：任意数量的空间点（至少 2 个），通常为每段 8 个测量点
	 *
	 * 输出：TubeAxis
	 *   direction — 点云主方向（单位向量，SVD 最大奇异值对应列）
	 *   midpoint  — 点云质心
	 *   origin    — 质心沿 -direction 方向延伸到点云范围端点
	 *   endpoint  — 质心沿 +direction 方向延伸到点云范围端点
	 *
	 * 算法：对去质心后的点矩阵做 SVD，取第一右奇异向量作为轴线方向
	 */
	static TubeAxis fitTubeAxis(const std::vector<Eigen::Vector3d>& points);

	/**
	 * 顶层接口：直接输入 4 × N 组测量点（实测 + 理论），一次计算电极补偿量
	 *
	 * 数据组织（每根管各有两段，每段 N 个点，N 通常为 8）：
	 *   tube1_actual_seg1   — 管1 第1段实测点（靠近工件端，N 个点）
	 *   tube1_actual_seg2   — 管1 第2段实测点（远离工件端，N 个点）
	 *   tube2_actual_seg1   — 管2 第1段实测点
	 *   tube2_actual_seg2   — 管2 第2段实测点
	 *   tube1_nominal_seg1  — 管1 第1段理论点
	 *   tube1_nominal_seg2  — 管1 第2段理论点
	 *   tube2_nominal_seg1  — 管2 第1段理论点
	 *   tube2_nominal_seg2  — 管2 第2段理论点
	 *
	 * 内部步骤：
	 *   1. 对每段点云调用 fitTubeAxis()，得到各段轴线
	 *   2. 将同一管的两段轴线合并：方向取两段方向的平均（重新 SVD 或平均后归一化），
	 *      origin/endpoint 取两段端点在主方向上的极值
	 *   3. 调用 computeElectrodeCompensation() 得到最终结果
	 */
	static ElectrodeResult computeElectrodeFromPoints(
	    const std::vector<Eigen::Vector3d>& tube1_actual_seg1,
	    const std::vector<Eigen::Vector3d>& tube1_actual_seg2,
	    const std::vector<Eigen::Vector3d>& tube2_actual_seg1,
	    const std::vector<Eigen::Vector3d>& tube2_actual_seg2,
	    const std::vector<Eigen::Vector3d>& tube1_nominal_seg1,
	    const std::vector<Eigen::Vector3d>& tube1_nominal_seg2,
	    const std::vector<Eigen::Vector3d>& tube2_nominal_seg1,
	    const std::vector<Eigen::Vector3d>& tube2_nominal_seg2,
	    double                              nominalSpacing,
	    double                              parallelTol_deg = 0.5,
	    double                              spacingTol_mm   = 0.05);

	/**
	 * 由两根管的实测/理论端点计算电极补偿量
	 *
	 * 输入：
	 *   tube1A/B_actual   — 管1 两端的实测点（G54 坐标系，已完成 BC 还原）
	 *   tube2A/B_actual   — 管2 两端的实测点
	 *   tube1A/B_nominal  — 管1 两端的理论点
	 *   tube2A/B_nominal  — 管2 两端的理论点
	 *   nominalSpacing    — 两管理论中心间距 [mm]
	 *   parallelTol_deg   — 平行度公差 [deg]
	 *   spacingTol_mm     — 间距公差 [mm]
	 *
	 * 输出：ElectrodeResult（见结构体说明）
	 *
	 * 算法：
	 *   1. 由端点构造各管轴线（方向 & 中点）
	 *   2. 平行度：两实测轴线方向向量夹角
	 *   3. 间距：两管中点连线在 XY 平面内的投影长度
	 *   4. deltaC：两管中点连线方向与理论连线方向的夹角（绕 Z 轴）
	 *   5. deltaY：两管中点平均值在 Y 方向的偏差
	 *   6. deltaZ：两管中点平均值在 Z 方向的偏差
	 */
	static ElectrodeResult computeElectrodeCompensation(
	    const Eigen::Vector3d& tube1A_actual,
	    const Eigen::Vector3d& tube1B_actual,
	    const Eigen::Vector3d& tube2A_actual,
	    const Eigen::Vector3d& tube2B_actual,
	    const Eigen::Vector3d& tube1A_nominal,
	    const Eigen::Vector3d& tube1B_nominal,
	    const Eigen::Vector3d& tube2A_nominal,
	    const Eigen::Vector3d& tube2B_nominal,
	    double                 nominalSpacing,
	    double                 parallelTol_deg = 0.5,
	    double                 spacingTol_mm   = 0.05);

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
	ProbeFit6DOF_BC(const G54Config&       g54,
	                const Eigen::Vector3d& B_center_machine,
	                const Eigen::Vector3d& C_center_machine,
	                double                 signB = 1.0,
	                double                 signC = 1.0);

	// ─────────────────────────────────────────
	// 点管理
	// ─────────────────────────────────────────

	void   addPoint(const ProbePoint& pt);
	void   addPoint(const Eigen::Vector3d& nominal,
	                const Eigen::Vector3d& actual,
	                const Eigen::Vector3d& ijk,
	                double                 B_deg = 0.0,
	                double                 C_deg = 0.0);
	void   clearPoints();
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
	 * @param result 输出参数，拟合结果
	 * @return true 表示成功，false 表示失败（如点数不足）
	 */
	bool solve(Result& result) const;

	bool solveKabsch(Result& result) const;
	/**
	 * 剥离指定火花机旋转中心，得到最终补偿值
	 *
	 * @param result          solve() 的返回值
	 * @param B_center_machine 火花机 B 轴旋转中心（机床坐标系）
	 * @param C_center_machine 火花机 C 轴旋转中心（机床坐标系）
	 * @param sparkG54_xyz     火花机 G54 的 XYZ 偏置（机床坐标系）
	 * @return Compensation    旋转矩阵 + 真实平移 + BC 补偿角
	 */
	static Compensation computeCompensation(const Result&          result,
	                                        const Eigen::Vector3d& B_center_machine,
	                                        const Eigen::Vector3d& C_center_machine,
	                                        const Eigen::Vector3d& sparkG54_xyz,
	                                        const Eigen::Vector2d& spindleC_eccentricity,
	                                        double                 sparkSignA = 1.0,
	                                        double                 sparkSignB = 1.0,
	                                        double                 sparkSignC = 1.0);

	// ─────────────────────────────────────────
	// 基础工具（公开，供外部使用）
	// ─────────────────────────────────────────

	static double deg2rad(double deg);
	static double rad2deg(double rad);

  private:
	void decomposeRotation(const Eigen::Matrix3d& R,
	                       Eigen::Matrix3d&       Rb,
	                       Eigen::Matrix3d&       Rc,
	                       double&                ry,
	                       double&                rz) const;

	static Eigen::Matrix3d rotY(double rad);
	static Eigen::Matrix3d rotZ(double rad);
	static Eigen::Matrix4d makeTransform(const Eigen::Matrix3d& R,
	                                     const Eigen::Vector3d& t);
	static Eigen::Matrix4d rotateAroundPoint(const Eigen::Matrix3d& R,
	                                         const Eigen::Vector3d& center);

	Eigen::Matrix4d buildBCTransform(double                 B_actual_deg,
	                                 double                 C_actual_deg,
	                                 const Eigen::Vector3d& B_center,
	                                 const Eigen::Vector3d& C_center) const;

	Eigen::Vector3d resetMeasureRotation(const Eigen::Vector3d& p_g54,
	                                     double                 B_deg,
	                                     double                 C_deg,
	                                     const G54Config&       g54,
	                                     const Eigen::Vector3d& B_center_machine,
	                                     const Eigen::Vector3d& C_center_machine) const;

	Eigen::Vector3d resetIJKRotation(const Eigen::Vector3d& ijk,
	                                 double                 B_deg,
	                                 double                 C_deg,
	                                 const G54Config&       g54) const;

	G54Config               g54_;
	Eigen::Vector3d         B_center_machine_;
	Eigen::Vector3d         C_center_machine_;
	double                  kSignB_;
	double                  kSignC_;
	std::vector<ProbePoint> points_;
};

#endif // PROBE_FIT_6DOF_BC_H
