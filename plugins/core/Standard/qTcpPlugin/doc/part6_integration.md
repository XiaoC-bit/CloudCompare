# 第六部分：系统集成

---

## 15. 点云在数控与机床系统中的应用

### 15.1 加工前对齐

**场景**：工件装夹后，需要确定工件坐标系的准确位置。

**流程**：

```
1. 采集工件点云
2. 与理论模型配准
3. 计算位姿偏差
4. 更新G54工件坐标系
```

**代码中的实现**（来自 `ProbeFit6DOF_BC.cpp`）：

```cpp
// 6DOF偏差拟合
bool solve(Result& result) const
{
    // 收集测量点
    const int N = static_cast<int>(points_.size());
    if (N < 6) return false;  // 至少需要6个点
    
    // 重置测量旋转（将不同BC姿态下的点统一到基准姿态）
    for (int i = 0; i < N; ++i) {
        nominal.row(i) = resetMeasureRotation(pts[i].nominal, pts[i].B_deg, pts[i].C_deg, ...);
        actual.row(i) = resetMeasureRotation(pts[i].actual, pts[i].B_deg, pts[i].C_deg, ...);
        normals.row(i) = resetIJKRotation(pts[i].ijk, pts[i].B_deg, pts[i].C_deg, ...);
    }
    
    // 构建点到平面配准问题
    for (int i = 0; i < N; ++i) {
        A.row(i).head<3>() = normals.row(i);
        A.row(i).tail<3>() = nominal.row(i).cross(normals.row(i));
    }
    
    // SVD求解
    BDCSVD<MatrixXd, ComputeThinU | ComputeThinV> svd(A);
    VectorXd x = svd.solve(dev);
    
    // 构建旋转矩阵（从旋转向量）
    Vector3d omega(x(3), x(4), x(5));
    double ang = omega.norm();
    if (ang > 1e-10) {
        Vector3d k = omega / ang;
        Matrix3d K;  // 反对称矩阵
        R = Matrix3d::Identity() + sin(ang) * K + (1 - cos(ang)) * K * K;
    }
    
    // 返回结果
    result.R = R;
    result.t = Vector3d(x(0), x(1), x(2));
    result.rms = sqrt(res.squaredNorm() / N);
    
    return true;
}
```

### 15.2 放电补偿

**场景**：电火花加工中，需要根据电极磨损进行补偿。

**流程**：

```
1. 测量电极实际形状
2. 与理论模型对比
3. 计算磨损量
4. 生成补偿NC程序
```

**代码中的补偿计算**（来自 `ProbeFit6DOF_BC.cpp`）：

```cpp
Compensation computeCompensation(const Result& result,
                                 const Eigen::Vector3d& B_center_machine,
                                 const Eigen::Vector3d& C_center_machine,
                                 const Eigen::Vector3d& sparkG54_xyz,
                                 const Eigen::Vector2d& spindleC_eccentricity,
                                 double sparkSignA, double sparkSignB, double sparkSignC)
{
    const Matrix3d& R = result.R;
    
    // ZYZ角度分解
    double b = acos(clamp(R(2, 2), -1.0, 1.0));
    if (result.omega.y() < 0) b = -b;
    
    // 处理万向锁情况
    double a, c;
    if (abs(sin(b)) < 1e-1) {
        a = 0.0;
        c = atan2(-R(0, 1), R(0, 0));
    } else {
        c = atan2(R(2, 1), -R(2, 0));
        a = atan2(R(1, 2), R(0, 2));
    }
    
    // 计算旋转中心在工件坐标系中的位置
    Vector3d B_pivot = B_center_machine - sparkG54_xyz;
    Vector3d C_pivot = C_center_machine - sparkG54_xyz;
    
    // 剥离旋转中心引入的等效平移
    Vector3d t1 = result.t - (Matrix3d::Identity() - Rc) * C_pivot;
    Vector3d t2 = t1 - (Matrix3d::Identity() - Rb) * B_pivot;
    
    // 补偿主轴偏心
    Vector3d ecc(spindleC_eccentricity.x(), spindleC_eccentricity.y(), 0.0);
    Vector3d t_true = t2 - (Matrix3d::Identity() - Ra) * ecc;
    
    // 返回补偿结果
    Compensation comp;
    comp.R = R;
    comp.t_true = t_true;
    comp.B_comp_deg = rad2deg(b);
    comp.C_comp_deg = rad2deg(c);
    comp.A_comp_deg = rad2deg(a);
    
    return comp;
}
```

### 15.3 工件坐标修正

**场景**：加工过程中，工件可能发生微小位移，需要实时修正。

**流程**：

```
1. 定期测量工件特征点
2. 计算坐标偏差
3. 动态更新工件坐标系
```

**代码中的实现**（来自 `EigenUtils.cpp` 的RTCP算法）：

```cpp
void EigenUtils::RTCP()
{
    // G54在机床坐标系下的位置（B=0 C=0时录入）
    Eigen::Vector3d g54_in_machine(25, -90, -80);
    double g54_B_angle = 10;
    double g54_C_angle = 0.0;
    
    // 旋转中心在机床坐标系下的位置
    Eigen::Vector3d pivot_B_in_machine(-52.170, 0, -174.543);
    Eigen::Vector3d pivot_C_in_machine(-52.170, -81.618, 0);
    
    // 目标位置
    double target_X = 1, target_Y = 2, target_Z = 3;
    double target_B = 25.0, target_C = 0.0;
    
    // 计算旋转中心在工件坐标系下的位置
    Eigen::Vector3d pivot_B_in_work = pivot_B_in_machine - g54_in_machine;
    Eigen::Vector3d pivot_C_in_work = pivot_C_in_machine - g54_in_machine;
    
    // 计算目标旋转矩阵
    Eigen::Matrix3d R_C = Eigen::AngleAxisd(-target_C * PI / 180.0, Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Matrix3d R_B = Eigen::AngleAxisd(-target_B * PI / 180.0, Vector3d::UnitY()).toRotationMatrix();
    
    // RTCP补偿（工件坐标系下）
    Eigen::Vector3d compensation_C = pivot_C_in_work - R_C * pivot_C_in_work;
    Eigen::Vector3d compensation_B = pivot_B_in_work - R_B * pivot_B_in_work;
    Eigen::Vector3d total_compensation = compensation_C + compensation_B;
    
    // 转换到机床坐标系
    Eigen::Vector3d machine_after = g54_in_machine 
        + R_B * R_C * tool_tip_in_work + total_compensation;
}
```

---

## 16. 多设备系统中的坐标统一问题

### 16.1 CMM / EDM / CNC 坐标一致性

**场景**：多个设备需要共享同一工件坐标系。

**统一流程**：

```
1. 使用标准球/标准块作为参考
2. 在每个设备上测量参考特征
3. 建立设备间的坐标变换关系
4. 定期校准维护一致性
```

**代码中的多设备标定**（来自 `PointCloudService.h`）：

```cpp
// 执行探针标定
bool executeProbeCalibration();

// 执行环规标定
bool executeRingCalibration();

// 执行相机标定
bool executeCalibration(const QVector<QVector3D>& positions, 
                        CalibrationProgressCallback progressCallback = nullptr);
```

### 16.2 多设备链路变换

**变换链**：

```
CMM坐标系
    ↓ T_CMM_WORLD
世界坐标系
    ↓ T_WORLD_CNC
CNC机床坐标系
    ↓ T_CNC_EDM
EDM火花机坐标系
```

**代码中的实现**（来自 `PointCloudService.cpp`）：

```cpp
// 计算相机运动
static Eigen::Matrix4d computeCameraMotion(
    const Eigen::Matrix4d& T_cam2robot,
    double x, double y, double z,
    double B_deg, double C_deg,
    const Eigen::Vector3d& pivot_B,
    const Eigen::Vector3d& pivot_C)
{
    // 构建机器人运动变换
    Eigen::Matrix4d T_robot = buildRobotMotion(x, y, z, B_deg, C_deg, pivot_B, pivot_C);
    
    // 相机运动 = 机器人运动 * 手眼变换的逆
    return T_robot * T_cam2robot.inverse();
}
```

### 16.3 数据一致性问题

**数据一致性保证**：

| 问题 | 解决方案 |
|------|----------|
| 坐标漂移 | 定期重新标定 |
| 数据格式不一致 | 使用统一的数据交换格式 |
| 时间同步 | 使用NTP时间同步 |
| 设备间通信 | 使用TCP/IP协议 |

**代码中的通信实现**（来自 `CcTcpServer.cpp`）：

```cpp
// TCP服务器处理
void CcTcpServer::onNewConnection()
{
    QTcpSocket* socket = m_server->nextPendingConnection();
    
    // 设置接收缓冲区
    connect(socket, &QTcpSocket::readyRead, this, [this, socket]() {
        QByteArray data = socket->readAll();
        processData(data, socket);
    });
    
    // 设置断开连接处理
    connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);
}

// 数据处理
void CcTcpServer::processData(const QByteArray& data, QTcpSocket* socket)
{
    // 解析JSON命令
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(data, &error);
    
    if (error.error != QJsonParseError::NoError) {
        sendError(socket, "Invalid JSON");
        return;
    }
    
    // 分发命令
    QString command = doc["command"].toString();
    QJsonObject params = doc["params"].toObject();
    QString idCode = doc["id"].toString();
    
    m_dispatcher->dispatch(command, params, socket, idCode);
}
```

---

## 17. 一个完整工业点云系统架构设计

### 17.1 数据流

**数据流架构**：

```
传感器层
    ├── 激光扫描仪
    ├── 接触式测头
    └── 视觉相机
            ↓
数据采集层
    ├── 数据获取
    ├── 格式转换
    └── 初步处理
            ↓
算法处理层
    ├── 点云配准
    ├── 特征提取
    └── 误差计算
            ↓
应用层
    ├── 工件检测
    ├── 电极检测
    └── NC程序生成
            ↓
输出层
    ├── 检测报告
    ├── NC代码
    └── 数据归档
```

**代码中的数据流程**（来自 `PointCloudService.cpp`）：

```cpp
// 工件检测流程
bool executePartInspect(const QString& partType, const QString& rfid)
{
    // 1. 获取测量点
    QVector<QVector3D> measurementPoints = acquireMeasurement();
    
    // 2. 加载理论模型
    ccHObject* model = loadModel(partType);
    
    // 3. 点云配准
    ccGLMatrix transform;
    double rms;
    icpInternal(model, measurementPoints, transform, rms);
    
    // 4. 误差计算
    QJsonObject result = computeDeviations(model, measurementPoints, transform);
    
    // 5. 保存结果
    savePartInspectResult(rfid, result);
    
    return true;
}
```

### 17.2 坐标流

**坐标变换流程**：

```
点云坐标系 (Sensor)
    ↓ T_SP（传感器标定）
工件坐标系 (Workpiece)
    ↓ T_WM（G54偏置）
机床坐标系 (Machine)
    ↓ T_MB（B轴旋转）
B轴坐标系 (B-Axis)
    ↓ T_BC（C轴旋转）
C轴坐标系 (C-Axis)
```

**代码中的坐标管理**（来自 `PointCloudService.h`）：

```cpp
// 旋转中心配置
Eigen::Vector3d m_bAxisCenter;  // B轴旋转中心
Eigen::Vector3d m_cAxisCenter;  // C轴旋转中心

// G54工件坐标系配置
struct G54Config {
    Eigen::Vector3d xyz;  // G54的XYZ偏置
    double B_deg;         // G54的B角偏置
    double C_deg;         // G54的C角偏置
};
G54Config m_g54Config;
```

### 17.3 算法模块化设计

**模块划分**：

```
┌──────────────────────────────────────────────────────────┐
│                    应用层                                │
│  CalibrationDialog | PartInspectDialog | EdmProgram     │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                    服务层                                │
│  PointCloudService | CommandDispatcher | MachineProxy   │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                    算法层                                │
│  ProbeFit6DOF_BC | EigenUtils | ccRegistrationTools    │
└──────────────────────────────────────────────────────────┘
                           │
                           ▼
┌──────────────────────────────────────────────────────────┐
│                    基础层                                │
│  Eigen | CloudCompare CCCoreLib | Qt                    │
└──────────────────────────────────────────────────────────┘
```

### 17.4 工程系统边界

**系统边界定义**：

| 边界 | 职责 |
|------|------|
| **硬件边界** | 传感器、机床、控制器 |
| **软件边界** | CloudCompare、插件、通信模块 |
| **数据边界** | 点云数据、配置文件、日志 |
| **网络边界** | TCP/IP通信、协议规范 |

**代码中的边界处理**（来自 `MachineProxy.cpp`）：

```cpp
// 机床通信代理
bool MachineProxy::sendCommand(const QString& command, QJsonObject& response)
{
    // 检查连接状态
    if (!ensureConnected()) {
        return false;
    }
    
    // 发送命令
    QJsonDocument doc;
    doc.setObject({{"command", command}});
    
    m_socket->write(doc.toJson());
    m_socket->flush();
    
    // 等待响应
    if (!m_socket->waitForReadyRead(m_timeout)) {
        return false;
    }
    
    // 解析响应
    QByteArray data = m_socket->readAll();
    response = QJsonDocument::fromJson(data).object();
    
    return true;
}
```

---

## 关键代码文件

| 文件 | 内容 |
|------|------|
| [ProbeFit6DOF_BC.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/ProbeFit6DOF_BC.cpp) | 6DOF拟合与补偿 |
| [EigenUtils.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/EigenUtils.cpp) | RTCP算法 |
| [PointCloudService.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/PointCloudService.cpp) | 点云服务与流程 |
| [CcTcpServer.cpp](file:///d:/SVN/X-MASTER-PROJECT/X-MASTER-8664-EDM-XIANHANGFA/3%20Source%20Code/CloudCompare/plugins/core/standard/qtcpplugin/CcTcpServer.cpp) | TCP通信 |

---

## 小结

工业点云系统集成需要考虑：

1. **机床应用**：加工前对齐、放电补偿、坐标修正
2. **多设备协调**：CMM/EDM/CNC坐标统一
3. **系统架构**：数据流、坐标流、模块化设计
4. **工程边界**：硬件、软件、数据、网络边界

这是将理论知识转化为实际生产力的关键一步。