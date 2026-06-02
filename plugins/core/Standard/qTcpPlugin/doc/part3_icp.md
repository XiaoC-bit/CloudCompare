# 第三部分：点云配准核心算法（ICP体系）

本部分定义点云配准的统一优化框架，并给出工程可实现的 ICP 数值结构。

核心问题：

* 多视角点云在 SE(3) 下的对齐
* 非凸优化问题
* 依赖初值的局部收敛系统

---

## 6. 点云配准的数学定义

### 6.1 Point Set Registration（刚体配准）

点云配准定义为：

```id="reg_problem"
Q ≈ T(P),   T ∈ SE(3)
```

其中：

* P：源点云
* Q：目标点云
* T：刚体变换（R, t）

目标是求解：

```id="opt_problem"
T* = argmin_T Σ || q_i - T p_i ||²
```

---

### 6.2 误差模型（Energy Formulation）

#### 点到点模型

```id="p2p"
E(T) = Σ || q_i - (R p_i + t) ||²
```

特点：

* 形式简单
* 对噪声敏感
* 无几何约束

---

#### 点到平面模型

```id="p2plane"
E(T) = Σ ( n_i · (q_i - (R p_i + t)) )²
```

特点：

* 引入局部流形约束
* 收敛更快
* 工业常用

---

### 6.3 线性化形式

点到平面误差可写为线性最小二乘：

```id="linear_ls"
min ||A x - d||²
```

其中：

* x：SE(3)局部参数（平移 + 小角度旋转）
* A：由法向量与几何梯度构造
* d：残差

工程构造方式：

```cpp id="design_matrix"
A.row(i).head<3>() = n_i;
A.row(i).tail<3>() = p_i.cross(n_i);
```

含义：

* 平移影响：nᵢ
* 旋转影响：p × n

---

## 7. ICP算法（Iterative Closest Point）

### 7.1 数值优化框架

ICP本质为交替优化问题：

```id="icp_opt"
min_T Σ || q_i - T p_i ||²
subject to correspondence: q_i ↔ p_i
```

分解为两步：

* 对应关系估计（non-convex step）
* 变换求解（convex LS step）

---

### 7.2 ICP迭代结构

算法形式：

```id="icp_loop"
T₀ = initial_guess

repeat:
    1. correspondence:
        q_i = NN(T p_i)
    
    2. optimization:
        T' = argmin Σ || q_i - T p_i ||²
    
    3. update:
        T ← T' · T
until convergence
```

结构本质：

* EM-like alternating minimization
* 外层非凸
* 内层闭式解

---

### 7.3 最近邻匹配机制

对应关系：

```id="nn"
q_i = argmin || q - T p_i ||
```

复杂度对比：

| 方法          | 复杂度        |
| ----------- | ---------- |
| brute force | O(NM)      |
| KD-Tree     | O(N log M) |

工程系统默认使用 KD-Tree 或近似搜索。

CloudCompare 实现本质：

* 基于空间索引结构
* 距离阈值过滤候选集
* 输出稳定匹配对

---

### 7.4 SVD刚体求解（Kabsch Step）

每次迭代的闭式解：

```id="svd_step"
H = P_cᵀ Q_c
H = U Σ Vᵀ
R = V Uᵀ
t = q̄ - R p̄
```

关键约束：

* R ∈ SO(3)
* det(R) = 1（需反射修正）

实现机制：

```cpp id="kabsch_impl"
Eigen::Matrix3d H = P_centered.transpose() * Q_centered;

Eigen::BDCSVD<Eigen::MatrixXd> svd(H);

Eigen::Matrix3d R = V * U.transpose();

if (R.determinant() < 0) {
    V.col(2) *= -1;
    R = V * U.transpose();
}
```

---

### 7.5 收敛判定机制

ICP停止条件：

```id="convergence"
|E_k - E_{k-1}| < ε
or
||T_k - T_{k-1}|| < ε
or
k ≥ K_max
```

CloudCompare实现本质：

* RMS误差下降检测
* 迭代次数上限约束
* 阈值稳定性控制

---

### 7.6 局部最优结构

ICP是非凸优化：

```id="nonconvex"
E(T) 非凸
```

因此存在：

* 局部极小值
* 初值敏感性
* 错误吸引域

典型失败模式：

* 错误对应关系稳定化
* 收敛到错误几何结构

---

## 8. ICP工程增强机制

### 8.1 下采样（Downsampling）

目标：

* 降低 N
* 提升匹配稳定性
* 降低噪声密度

方法：

| 方法               | 特征   |
| ---------------- | ---- |
| voxel grid       | 工业标准 |
| uniform sampling | 简单稳定 |

本质：

```id="sampling"
P' ⊂ P,  |P'| << |P|
```

---

### 8.2 离群点抑制（Robust Correspondence）

目标函数隐含假设：

* 对应关系大多正确

实际违反 → 需要鲁棒机制

方法：

* 距离阈值截断
* 分位数剪枝
* robust loss（Huber / Tukey）

工程实现：

```cpp id="outlier"
if (d_i > threshold)
    discard
```

---

### 8.3 权重建模（Weighted ICP）

扩展目标：

```id="weighted_icp"
min Σ w_i || q_i - T p_i ||²
```

权重来源：

| 来源    | 含义    |
| ----- | ----- |
| 法向一致性 | 几何可靠性 |
| 距离    | 匹配置信度 |
| 曲率    | 局部稳定性 |

---

### 8.4 初值敏感性（Critical Factor）

ICP收敛域依赖：

* 初始旋转误差
* 初始平移误差
* 重叠率

经验条件：

| 初值质量 | 结果     |
| ---- | ------ |
| 高    | 快速收敛   |
| 中    | 局部最优   |
| 低    | 发散或错误解 |

---

### 8.5 全局-局部混合策略

工程常见结构：

```
Global Registration → ICP Refinement
```

全局方法：

* FPFH / SHOT 特征
* 4PCS
* RANSAC

局部方法：

* ICP
* Point-to-plane ICP

---

## 关键代码映射

| 文件                      | 作用         |
| ----------------------- | ---------- |
| ccRegistrationTools.cpp | ICP调度与参数层  |
| CloudCompare ICP core   | 最近邻 + 迭代框架 |
| ProbeFit6DOF_BC.cpp     | 点到平面最小二乘   |

---

## 小结

ICP体系可归纳为三个层次：

* **几何模型**：SE(3)下的刚体对齐问题
* **数值结构**：交替优化（correspondence + LS）
* **工程约束**：采样、鲁棒性、初值依赖

本质上：

> ICP是一个在非凸空间中，通过局部线性化不断逼近刚体最优解的迭代系统。

---
