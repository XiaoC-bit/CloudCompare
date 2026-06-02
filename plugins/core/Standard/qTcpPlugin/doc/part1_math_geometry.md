# 第一部分：数学与几何基础（统一符号层）

本部分目标是建立后续所有算法的统一数学表达体系，重点约束如下：

* 坐标系表达统一为 SE(3)
* 旋转部分严格归属 SO(3)
* 全流程采用左乘约定
* 所有工业系统中的运动均可归约为坐标变换链

---

## 1. 工业几何计算中的坐标系统与变换表示

### 1.1 齐次坐标与 4×4 变换矩阵

三维刚体变换统一表示为齐次矩阵：

```
T = [ R | t ]
    [ 0 | 1 ]
```

其中：

* R ∈ SO(3)，表示旋转
* t ∈ ℝ³，表示平移
* T ∈ SE(3)，表示刚体变换

工程上该表示的核心作用是将旋转与平移统一为一次矩阵乘法，从而支持变换链的代数闭包性质。

在代码层通常构造为：

```cpp
Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
T.block<3, 3>(0, 0) = R;
T.block<3, 1>(0, 3) = t;
```

---

### 1.2 SE(3) 刚体变换结构

刚体变换可抽象为：

```cpp
struct RigidTransform {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};
```

该结构满足 SE(3) 群性质：

* 封闭性：T₁T₂ ∈ SE(3)
* 可逆性：T⁻¹ ∈ SE(3)
* 单位元存在

---

### 1.3 左乘约定（统一约束）

本体系固定采用**左乘约定**：

```
p_B = T_AB · p_A
```

含义：

* T_AB 表示 A → B 的坐标变换
* 向量均采用列向量形式
* 变换作用于坐标系表达（被动变换语义）

对比关系：

| 语义   | 数学形式       | 工程含义  |
| ---- | ---------- | ----- |
| 被动变换 | p' = T p   | 坐标系变换 |
| 主动变换 | p' = T⁻¹ p | 点运动   |

本体系统一采用被动变换模型。

---

### 1.4 工业坐标系链路模型

工业系统中的典型坐标链为：

```
World
  ↓ T_WM
Machine
  ↓ T_MT
Tool
  ↓ T_TS
Sensor
  ↓ T_SP
PointCloud
```

整体表达为链式乘积：

```
p_world = T_WM · T_MT · T_TS · T_SP · p_cloud
```

在代码中通常体现为坐标偏置与局部变换叠加，例如：

```cpp
Eigen::Vector3d pivot_in_work =
    pivot_in_machine - g54_in_machine;
```

该类操作本质为 SE(3) 中平移分量的坐标重表达。

---

## 2. 空间刚体变换的数学基础

### 2.1 SO(3) 旋转矩阵性质

旋转矩阵 R ∈ SO(3) 满足约束：

```
RᵀR = I
det(R) = 1
```

性质推导：

* 保长性：‖Rv‖ = ‖v‖
* 保角性：角度不变
* 逆等于转置：R⁻¹ = Rᵀ

典型实现：

```cpp
Eigen::Matrix3d rotY(double rad)
{
    double c = cos(rad), s = sin(rad);
    return (Eigen::Matrix3d() <<
        c, 0, s,
        0, 1, 0,
       -s, 0, c).finished();
}
```

---

### 2.2 变换组合规则

左乘体系下：

```
T_AC = T_BC · T_AB
```

对应执行顺序：

* 右侧先作用
* 左侧后作用

代码中体现为：

```cpp
Eigen::Matrix3d R_total = R_B * R_C;
```

该顺序表示：

* 先 C 轴旋转
* 再 B 轴旋转

---

### 2.3 逆变换结构

对于：

```
T = [R t]
    [0 1]
```

其逆为：

```
T⁻¹ = [Rᵀ  -Rᵀt]
       [0     1  ]
```

实现：

```cpp
T_inv.block<3,3>(0,0) = R.transpose();
T_inv.block<3,1>(0,3) = -R.transpose() * t;
```

该结构是 SE(3) 群闭合性的直接体现。

---

### 2.4 主动/被动变换区分

| 类型 | 数学含义  | 工程语义  |
| -- | ----- | ----- |
| 被动 | 坐标系变化 | 标定/转换 |
| 主动 | 点变化   | 运动学   |

工业系统中：

* 标定问题 → 被动变换
* 运动控制 → 主动变换

---

## 3. SVD 与最小二乘基础（ICP 前置）

### 3.1 最小二乘问题

标准形式：

```
min ||Ax - b||²
```

几何意义：

* A：观测映射
* x：未知变换参数
* b：目标约束

在点云中对应：

* 点集对齐误差最小化

---

### 3.2 SVD 分解

```
A = U Σ Vᵀ
```

结构意义：

* U：输出空间正交基
* V：输入空间正交基
* Σ：尺度权重（奇异值）

工程意义：

* 提取主方向
* 降维稳定解
* 消除噪声方向

---

### 3.3 Kabsch 算法（刚体配准）

目标：

```
min Σ ||R p_i + t - q_i||²
```

约束：

* R ∈ SO(3)

步骤：

1. 计算质心
2. 去中心化数据
3. 构造协方差矩阵 H
4. SVD：H = UΣVᵀ
5. 计算旋转：R = VUᵀ
6. 若 det(R) < 0，修正反射
7. 平移：t = q̄ - R p̄

代码实现结构：

```cpp
Eigen::Matrix3d H = p_centered.transpose() * q_centered;

Eigen::BDCSVD<Eigen::MatrixXd> svd(
    H,
    Eigen::ComputeFullU | Eigen::ComputeFullV
);

Eigen::Matrix3d R = V * U.transpose();

if (R.determinant() < 0) {
    V.col(2) *= -1;
    R = V * U.transpose();
}

Eigen::Vector3d t = centroid_q - R * centroid_p;
```

---

## 关键工程文件映射

| 文件                  | 功能            |
| ------------------- | ------------- |
| EigenUtils.h        | SE(3)基础结构     |
| EigenUtils.cpp      | RTCP变换链实现     |
| ProbeFit6DOF_BC.h   | 6DOF配准接口      |
| ProbeFit6DOF_BC.cpp | Kabsch + 标定求解 |

---

## 小结（结构约束层）

本部分建立统一数学系统：

* SE(3)：刚体变换基本单位
* SO(3)：旋转约束空间
* 左乘体系：统一坐标传播方向
* 链式乘法：工业坐标系统表达方式
* SVD/Kabsch：点云刚体对齐解析解

该体系是 ICP、手眼标定、RTCP、坐标系转换的共同数学底座。
