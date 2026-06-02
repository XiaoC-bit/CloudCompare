# 第四部分：手眼标定与工业坐标系统（核心应用）

这一部分开始进入**真实项目**。手眼标定是工业点云系统的核心技术，用于建立传感器与机床/机器人之间的坐标关系。

---

## 9. 手眼标定的数学模型（AX = XB）

### 9.1 问题定义

手眼标定的目标是找到传感器坐标系相对于机器人/机床末端执行器的变换矩阵 `X`。

**典型场景**：

```
机器人基坐标系 (Base)
    ↓ A_i（机器人运动）
机器人末端 (End-effector)
    ↓ X（手眼关系，待求）
传感器坐标系 (Sensor)
    ↓ B_i（观测变换）
标定目标 (Calibration Target)
```

### 9.2 数学模型

当机器人移动时，我们有以下关系：

```
A_i * X = X * B_i
```

其中：
- `A_i`：机器人从位姿i到位姿i+1的变换
- `B_i`：传感器观测到的标定目标的变换
- `X`：手眼变换矩阵（从末端执行器到传感器）

**代码中的标定流程**（来自 `PointCloudService.h`）：

```cpp
// 执行相机标定
bool executeCalibration(const QVector<QVector3D>& positions, 
                        CalibrationProgressCallback progressCallback = nullptr);
```

### 9.3 机器人/机床/测量系统统一建模

**统一坐标变换链**：

```
机床坐标系 (Machine)
    ↓ T_MB（机床到B轴）
B轴坐标系 (B-Axis)
    ↓ T_BC（B轴到C轴）
C轴坐标系 (C-Axis)
    ↓ T_CW（C轴到工件）
工件坐标系 (Workpiece/G54)
    ↓ T_WS（工件到传感器）
传感器坐标系 (Sensor)
```

**代码中的坐标变换**（来自 `ProbeFit6DOF_BC.cpp`）：

```cpp
// 重置测量旋转，将点从测量时的姿态还原到G54零姿态
Eigen::Vector3d resetMeasureRotation(
    const Eigen::Vector3d& p,           // 测量点（G54坐标系）
    double Bd, double Cd,               // 测量时的B/C角度
    const G54Config& g,                 // G54配置
    const Eigen::Vector3d& Bm,          // B轴中心（机床坐标系）
    const Eigen::Vector3d& Cm) const    // C轴中心（机床坐标系）
{
    // 计算旋转中心在工件坐标系中的位置
    Eigen::Vector3d Bg = Bm - g.xyz;
    Eigen::Vector3d Cg = Cm - g.xyz;
    
    // 构建基准姿态和当前姿态的变换矩阵
    Eigen::Matrix4d M_base = buildBCTransform(g.B_deg, g.C_deg, Bg, Cg);
    Eigen::Matrix4d M_cur = buildBCTransform(Bd + g.B_deg, Cd + g.C_deg, Bg, Cg);
    
    // 计算从基准到当前的变换，然后求逆得到重置变换
    Eigen::Matrix4d M_reset = M_cur * M_base.inverse();
    return (M_reset.inverse() * ph).head<3>();
}
```

### 9.4 约束条件

**标定的约束条件**：

1. **至少需要3组变换对**：才能唯一确定X
2. **变换对之间要有足够的运动**：避免数值病态
3. **标定目标要在视野内**：确保每次都能观测到

---

## 10. 基于运动学的点云坐标转换

### 10.1 点云 → 工件坐标系

**转换流程**：

```cpp
// 将传感器坐标系下的点转换到工件坐标系
Eigen::Vector3d sensor_point(1.0, 2.0, 3.0);  // 传感器坐标系
Eigen::Vector3d work_point = T_WS * sensor_point;  // 转换到工件坐标系
```

**代码中的实现**（来自 `PointCloudService.h`）：

```cpp
// 计算相机运动变换
static Eigen::Matrix4d computeCameraMotion(
    const Eigen::Matrix4d& T_cam2robot,  // 相机到机器人的变换
    double x, double y, double z,         // 机器人位置
    double B_deg, double C_deg,           // B/C角度
    const Eigen::Vector3d& pivot_B,       // B轴旋转中心
    const Eigen::Vector3d& pivot_C);      // C轴旋转中心
```

### 10.2 工件系 → 机床坐标系

**G54偏置的应用**：

```cpp
// 从工件坐标系转换到机床坐标系
Eigen::Vector3d work_point(10.0, 20.0, 30.0);  // 工件坐标系
Eigen::Vector3d machine_point = work_point + g54_in_machine;  // 加上G54偏置
```

**代码中的转换**（来自 `EigenUtils.cpp`）：

```cpp
// G54在机床坐标系下的位置
Eigen::Vector3d g54_in_machine(25, -90, -80);  

// 将工件坐标系下的点转换到机床坐标系
Eigen::Vector3d machine_point = g54_in_machine 
    + R_B * R_C * tool_tip_in_work + total_compensation;
```

### 10.3 多系统链式变换

**完整的变换链**：

```cpp
// 点云 → 传感器 → 工件 → 机床
Eigen::Vector3d point_machine = 
    T_MW * T_WS * T_SP * point_cloud;
```

**代码中的链式变换**（来自 `ProbeFit6DOF_BC.cpp`）：

```cpp
// 构建完整的BC变换链
Eigen::Matrix4d buildBCTransform(double B_deg, double C_deg,
                                  const Eigen::Vector3d& B_center,
                                  const Eigen::Vector3d& C_center) const
{
    Eigen::Matrix3d Rb = rotY(kSignB_ * deg2rad(B_deg));
    Eigen::Matrix3d Rc = rotZ(kSignC_ * deg2rad(C_deg));
    
    // 先绕B中心旋转，再绕C中心旋转
    return rotateAroundPoint(Rb, B_center) * rotateAroundPoint(Rc, C_center);
}
```

---

## 11. 工业场景中的误差来源分析

### 11.1 标定误差

**相机标定误差**：

```cpp
// 标定结果包含误差指标
struct Result {
    Eigen::Matrix3d R;        // 旋转矩阵
    Eigen::Vector3d t;        // 平移向量
    double rms;               // 均方根误差
    double maxResidual;       // 最大残差
};
```

**标定误差的影响**：
- 直接影响所有测量结果
- 是系统误差的主要来源
- 需要定期重新标定

### 11.2 机床重复定位误差

**重复定位精度**：机床回到同一位置的能力。

**代码中的处理**：

```cpp
// 检查机床状态
bool waitForMachineIdle(int timeoutSeconds, QString* errorMessage = nullptr);

// 获取机床坐标
bool getDeviceMainAxisCoor(double& x, double& y, double& z, 
                           double& a, double& b, double& c, 
                           QString* errorMessage = nullptr);
```

### 11.3 测头误差 / 相机误差

**测头误差来源**：
- 测头触发误差
- 测杆变形
- 温度影响

**相机误差来源**：
- 镜头畸变
- 标定板精度
- 光照条件

**代码中的误差补偿**（来自 `ProbeFit6DOF_BC.cpp`）：

```cpp
// 补偿计算（分轴剥离火花机旋转中心）
Compensation computeCompensation(const Result& result,
                                 const Eigen::Vector3d& B_center_machine,
                                 const Eigen::Vector3d& C_center_machine,
                                 const Eigen::Vector3d& sparkG54_xyz,
                                 const Eigen::Vector2d& spindleC_eccentricity,
                                 double sparkSignA,
                                 double sparkSignB,
                                 double sparkSignC)
{
    // 分解旋转矩阵
    // 剥离旋转中心引入的等效平移
    // 补偿主轴偏心
    // ...
}
```

---

## 关键代码文件

| 文件 | 内容 |
|------|------|
| [PointCloudService.h](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/PointCloudService.h) | 点云服务与标定接口 |
| [PointCloudService.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/PointCloudService.cpp) | 坐标变换与标定实现 |
| [ProbeFit6DOF_BC.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/ProbeFit6DOF_BC.cpp) | 6DOF拟合与补偿计算 |
| [CalibrationDialog.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/CalibrationDialog.cpp) | 标定UI与流程控制 |

---

## 小结

手眼标定是连接传感器与机床的桥梁：

1. **数学模型**：AX = XB，通过多组变换对求解
2. **坐标转换**：点云 → 传感器 → 工件 → 机床
3. **误差分析**：标定误差、重复定位误差、传感器误差

理解手眼标定是实现工业点云系统的关键。