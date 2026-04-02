# rus_sim_force

> 基于雅可比转置法的机械臂末端六维力旋量估算节点。


## 功能概述

- ✅ 功能点1：从 URDF 自动解析运动学与惯性参数
- ✅ 功能点2：基于逆动力学的关节力矩补偿
- ✅ 功能点3：雅可比转置法估算 wrist3 处六维力旋量
- ✅ 功能点4：力旋量坐标变换至法兰原点
- ✅ 功能点5：奇异位形检测（条件数阈值 + 阻尼最小二乘）
- ✅ 功能点6：工具负载补偿（附加质量 + 质心偏置）
- ✅ 功能点7：仿真模式（`SetSimForce` 自动构造 tau_meas）
- 📝 功能点8：ROS2 话题实时发布估算结果
- 📝 功能点9：三维仿真moveit进一步验证结果

> 状态标记：✅ 已完成  🚧 进行中  📝 待办  ❌ 废弃


## 第三方依赖

| 依赖 | 说明 | 状态 |
|------|------|------|
| Eigen3 | 线性代数，雅可比/逆动力学计算 | ✅ |
| urdf | URDF 解析，加载运动学与惯性参数 | ✅ |
| rclcpp | ROS2 C++ 客户端 | ✅ |


## 节点说明

### wrench_estimate_node

当前无话题订阅/发布（估算接口以库形式调用，ROS2话题发布待实现）。

**启动方式**

```bash
ros2 run rus_sim_force wrench_estimate_node          # 正常运行
ros2 run rus_sim_force wrench_estimate_node --test   # 运行验证测试
```

## 核心接口（`RusSimForce::WrenchEstimate`）

**配置接口**

| 方法 | 说明 | 状态 |
|------|------|------|
| `SetGravity(v)` | 设置重力向量，默认 `[0,0,-9.81]` | ✅ |
| `SetFlangeConfig(cfg)` | 设置法兰偏置，默认 `[0,0,0.094]` m | ✅ |
| `SetSingularityThreshold(t)` | 条件数阈值，默认 200 | ✅ |
| `SetDampingFactor(λ)` | 阻尼最小二乘系数，默认 0.001 | ✅ |
| `SetToolPayload(m, com)` | 工具质量与质心偏置 | ✅ |
| `SetSimForce(w)` / `ClearSimForce()` | 注入/清除仿真外力 `[nx,ny,nz,fx,fy,fz]` | ✅ |

**估算接口**

| 方法 | 返回 | 说明 | 状态 |
|------|------|------|------|
| `Estimate(q,qd,qdd,tau)` | `ForceResult` | wrist3 处力旋量，base 系 | ✅ |
| `Estimate(state)` | `ForceResult` | 同上，`RobotState` 输入 | ✅ |
| `EstimateAtFlange(q,qd,qdd,tau)` | `EndEffectorWrench` | 变换至法兰，有实测力矩 | ✅ |
| `EstimateAtFlange(state)` | `EndEffectorWrench` | 同上，`RobotState` 输入 | ✅ |
| `EstimateAtFlange(q,qd,qdd)` | `EndEffectorWrench` | 仿真模式，无需 tau_meas | ✅ |
| `build_sim_tau_meas(q,qd,qdd)` | `VectorXd` | 正向构造仿真关节力矩 | ✅ |


## 数据结构

| 结构体 | 关键字段 |
|--------|----------|
| `RobotState` | `q, qd, qdd, tau_meas` |
| `ForceResult` | `force, torque, cond_num, is_valid`（wrist3，base系）|
| `EndEffectorWrench` | `force, torque`（base系）+ `force_in_flange, torque_in_flange`（法兰系）+ `p_flange, R_flange` |
| `FlangeConfig` | `flange_in_wrist3`（默认 `[0,0,0.094]` m）|


## 测试说明

| 测试文件 | 类型 | 内容 |
|----------|------|------|
| `test_wrench_estimate_smoke.cpp` | Smoke | 接口可调用、不崩溃 |
| `test_wrench_estimate_val.cpp` | 数值验证 | 5组共18个 case，全位形/全轴力旋量回路验证 | 但是这部分AI程度较大，我怀疑可能陷入自循环,但是在之前3D仿真结果无误，所以先这么放着吧，后面有待完善

验证通过标准：`force` 恢复误差 `< 0.05 N`，`torque` 一致性误差 `< 1e-3 Nm`。
