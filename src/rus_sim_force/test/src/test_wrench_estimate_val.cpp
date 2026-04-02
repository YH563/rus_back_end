#include "test_wrench_estimate_val.hpp"

using RusSimForce::vec3_to_str;
using RusSimForce::vec_to_str;

namespace TestRusSimForceVal {

// ─────────────────────────────────────────────────────────────────────
// 内部辅助：检查力误差，打印 PASS/FAIL，返回是否通过
// ─────────────────────────────────────────────────────────────────────
static bool check_force(
    const rclcpp::Logger& logger,
    const std::string& tag,
    const Eigen::Vector3d& got,
    const Eigen::Vector3d& expected,
    double tol = 0.05)
{
    const Eigen::Vector3d err = got - expected;
    const double norm = err.norm();
    if (norm < tol) {
        RCLCPP_INFO(logger,
            "  [PASS] %-35s err = %.2e N  (< %.2f)",
            tag.c_str(), norm, tol);
        return true;
    } else {
        RCLCPP_ERROR(logger,
            "  [FAIL] %-35s err = %.4f N  (>= %.2f)",
            tag.c_str(), norm, tol);
        RCLCPP_ERROR(logger,
            "         got=[%s]  expected=[%s]",
            vec3_to_str(got).c_str(),
            vec3_to_str(expected).c_str());
        return false;
    }
}

static bool check_torque(
    const rclcpp::Logger& logger,
    const std::string& tag,
    const Eigen::Vector3d& got,
    const Eigen::Vector3d& expected,
    double tol = 0.05)
{
    const Eigen::Vector3d err = got - expected;
    const double norm = err.norm();
    if (norm < tol) {
        RCLCPP_INFO(logger,
            "  [PASS] %-35s err = %.2e Nm (< %.2f)",
            tag.c_str(), norm, tol);
        return true;
    } else {
        RCLCPP_ERROR(logger,
            "  [FAIL] %-35s err = %.4f Nm (>= %.2f)",
            tag.c_str(), norm, tol);
        RCLCPP_ERROR(logger,
            "         got=[%s]  expected=[%s]",
            vec3_to_str(got).c_str(),
            vec3_to_str(expected).c_str());
        return false;
    }
}

// ─────────────────────────────────────────────────────────────────────
// 单组 SetSimForce 测试：注入 F_true，验证 force_base 恢复精度
// torque_base 真值需要手工算，这里只验证 force 部分
// ─────────────────────────────────────────────────────────────────────
static bool run_one_case(
    const rclcpp::Logger& logger,
    RusSimForce::WrenchEstimate& est,
    const std::string& case_name,
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& qd,
    const Eigen::VectorXd& qdd,
    const Eigen::VectorXd& F_true,   // [nx ny nz fx fy fz]
    double tol = 0.05)
{
    RCLCPP_INFO(logger, "── %s ──", case_name.c_str());
    RCLCPP_INFO(logger, "  F_true : [%s]", vec_to_str(F_true).c_str());

    est.SetSimForce(F_true);
    const RusSimForce::EndEffectorWrench res = est.EstimateAtFlange(q, qd, qdd);
    est.ClearSimForce();

    if (!res.is_valid) {
        RCLCPP_ERROR(logger, "  [FAIL] is_valid=false, cond=%.2e", res.cond_num);
        return false;
    }

    RCLCPP_INFO(logger, "  cond_num = %.4f  (valid)", res.cond_num);
    RCLCPP_INFO(logger, "  force_base   = [%s]", vec3_to_str(res.force).c_str());
    RCLCPP_INFO(logger, "  torque_base  = [%s]", vec3_to_str(res.torque).c_str());
    RCLCPP_INFO(logger, "  force_flange = [%s]", vec3_to_str(res.force_in_flange).c_str());
    RCLCPP_INFO(logger, "  torque_flange= [%s]", vec3_to_str(res.torque_in_flange).c_str());

    // force_base 真值就是 F_true 后三维（base 坐标系）
    const Eigen::Vector3d force_true = F_true.tail<3>();
    return check_force(logger, "force_base vs F_true.force", res.force, force_true, tol);
}

// ─────────────────────────────────────────────────────────────────────
// 主测试函数
// 注意：需要把 wrench_estimate.hpp 里 build_sim_tau_meas 临时改为 public
// ─────────────────────────────────────────────────────────────────────
void TestWrenchDemo(const std::string& urdf_path) {
    auto logger = rclcpp::get_logger("TestWrenchDemo");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "   WrenchEstimate 数值验证测试");
    RCLCPP_INFO(logger, "========================================");

    RusSimForce::WrenchEstimate est(urdf_path);
    int pass_count = 0;
    int total_count = 0;

    // ─── 公共位形 ─────────────────────────────────────────────────────
    // 非奇异、接近典型工作位形，条件数预期 < 50
    Eigen::VectorXd q(6);
    q << 0.0, -0.5236, 1.0472, -0.5236, 1.5708, 0.0;
    // [0°, -30°, 60°, -30°, 90°, 0°]

    // 静止状态（先排除动力学干扰）
    const Eigen::VectorXd qd_zero  = Eigen::VectorXd::Zero(6);
    const Eigen::VectorXd qdd_zero = Eigen::VectorXd::Zero(6);

    // 动态状态（用于验证动力学补偿）
    Eigen::VectorXd qd(6), qdd(6);
    qd  << 0.1, -0.05,  0.08, -0.03,  0.05,  0.02;
    qdd << 0.05, -0.02, 0.03, -0.01,  0.02,  0.01;

    // ================================================================
    RCLCPP_INFO(logger, "【第一组】静态 + 单轴力验证（排除动力学）");
    // ================================================================

    // Case 1-1: 纯 -Z 重力载荷（2kg 物体）
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  0, 0, -19.62;
        total_count++;
        if (run_one_case(logger, est, "Case1-1: fz=-19.62N (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case 1-2: 纯 +X 方向力
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  10.0, 0, 0;
        total_count++;
        if (run_one_case(logger, est, "Case1-2: fx=+10N (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case 1-3: 纯 +Y 方向力
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  0, 10.0, 0;
        total_count++;
        if (run_one_case(logger, est, "Case1-3: fy=+10N (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case 1-4: 对角力 fx+fz（验证各轴不互串扰）
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  10.0, 0, -10.0;
        total_count++;
        if (run_one_case(logger, est, "Case1-4: fx=+10, fz=-10N (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case 1-5: 三轴力同时存在
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  5.0, -8.0, -15.0;
        total_count++;
        if (run_one_case(logger, est, "Case1-5: fx=5,fy=-8,fz=-15N (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // ================================================================
    RCLCPP_INFO(logger, "【第二组】静态 + 力矩验证");
    // ================================================================

    // Case 2-1: 纯力矩 nz（绕 Z 轴）
    {
        Eigen::VectorXd F(6); F << 0, 0, 5.0,  0, 0, 0;
        total_count++;
        if (run_one_case(logger, est, "Case2-1: nz=+5Nm (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case 2-2: 力 + 力矩同时存在
    {
        Eigen::VectorXd F(6); F << 3.0, 0, 0,  0, 0, -10.0;
        total_count++;
        if (run_one_case(logger, est, "Case2-2: nx=3Nm, fz=-10N (静)", q, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // ================================================================
    RCLCPP_INFO(logger, "【第三组】动态 + 力验证（验证动力学补偿）");
    // ================================================================

    // Case 3-1: 有速度/加速度，只有重力载荷
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  0, 0, -19.62;
        total_count++;
        if (run_one_case(logger, est, "Case3-1: fz=-19.62N (动态)", q, qd, qdd, F))
            pass_count++;
    }

    // Case 3-2: 有速度/加速度，三轴力同时存在
    {
        Eigen::VectorXd F(6); F << 0, 0, 0,  5.0, -8.0, -15.0;
        total_count++;
        if (run_one_case(logger, est, "Case3-2: fx=5,fy=-8,fz=-15N (动态)", q, qd, qdd, F))
            pass_count++;
    }

    // ================================================================
    RCLCPP_INFO(logger, "【第四组】Method1 自洽性验证（build_sim_tau_meas 需改为 public）");
    // ================================================================
    // 注意：如果 build_sim_tau_meas 还是 private，这段编译会报错，先注释掉
    // 等改成 public 后取消注释
    
    {
        total_count++;
        Eigen::VectorXd F(6); F << 2.0, 0, 0,  5.0, -3.0, -10.0;
        est.SetSimForce(F);
        const Eigen::VectorXd tau_auto = est.build_sim_tau_meas(q, qd, qdd);
        est.ClearSimForce();

        RCLCPP_INFO(logger, "── Case4-1: Method1 自洽（build_sim_tau_meas）──");
        RCLCPP_INFO(logger, "  tau_auto = [%s]", vec_to_str(tau_auto).c_str());

        const RusSimForce::ForceResult res = est.Estimate(q, qd, qdd, tau_auto);

        RCLCPP_INFO(logger, "  force_base  = [%s]", vec3_to_str(res.force).c_str());
        RCLCPP_INFO(logger, "  torque_base = [%s]", vec3_to_str(res.torque).c_str());
        RCLCPP_INFO(logger, "  cond_num    = %.4f", res.cond_num);

        bool ok = true;
        ok &= check_force (logger, "Case4-1 force",  res.force,  F.tail<3>());
        ok &= check_torque(logger, "Case4-1 torque", res.torque, F.head<3>());
        if (ok) pass_count++;
    }
    

    // ================================================================
RCLCPP_INFO(logger, "【第五组】法兰坐标变换数学一致性验证");
// 核心策略：
//   1. 用 build_sim_tau_meas 生成 tau_meas
//   2. Estimate()       → raw (wrist3 处力旋量)
//   3. EstimateAtFlange → res (法兰处力旋量)
//   4. 手算: expected_torque = raw.torque + (-R_flange * [0,0,0.094]) × raw.force
//   5. 验证 res.torque 与手算一致
// 完全不依赖外部硬编码期望值，换任何位形/力都成立
// ================================================================

// 通用验证 lambda（复用，不重复写）
auto verify_flange_transform = [&](
    const std::string& tag,
    const Eigen::VectorXd& q_in,
    const Eigen::VectorXd& qd_in,
    const Eigen::VectorXd& qdd_in,
    const Eigen::VectorXd& F_in) -> bool
{
    RCLCPP_INFO(logger, "── %s ──", tag.c_str());
    RCLCPP_INFO(logger, "  q      : [%s]", vec_to_str(q_in).c_str());
    RCLCPP_INFO(logger, "  F_true : [%s]", vec_to_str(F_in).c_str());

    // Step1: 生成一致的 tau_meas
    est.SetSimForce(F_in);
    const Eigen::VectorXd tau = est.build_sim_tau_meas(q_in, qd_in, qdd_in);
    est.ClearSimForce();

    // Step2: 分别获取 wrist3 处 raw 和法兰处 res
    const RusSimForce::ForceResult       raw = est.Estimate(q_in, qd_in, qdd_in, tau);
    const RusSimForce::EndEffectorWrench res = est.EstimateAtFlange(q_in, qd_in, qdd_in, tau);

    if (!raw.is_valid) {
        RCLCPP_WARN(logger, "  [SKIP] 接近奇异位形 cond=%.2e，跳过", raw.cond_num);
        return true; // 奇异不计入失败
    }

    // Step3: 用 R_flange 反推搬移向量（不需要任何外部知识）
    // d = p_wrist3 - p_flange = -R_flange * flange_in_wrist3
    const Eigen::Vector3d flange_in_wrist3(0.0, 0.0, 0.094);
    const Eigen::Vector3d d = -(res.R_flange * flange_in_wrist3);

    // Step4: 手算期望 torque_at_flange
    const Eigen::Vector3d expected_torque = raw.torque + d.cross(raw.force);

    // Step5: 打印中间量，便于调试
    RCLCPP_INFO(logger, "  cond         : %.4f", raw.cond_num);
    RCLCPP_INFO(logger, "  raw.force    : [%s]  (wrist3, base系)",
                vec3_to_str(raw.force).c_str());
    RCLCPP_INFO(logger, "  raw.torque   : [%s]  (wrist3, base系)",
                vec3_to_str(raw.torque).c_str());
    RCLCPP_INFO(logger, "  d_base(偏置) : [%s]  (-R_flange*[0,0,0.094])",
                vec3_to_str(d).c_str());
    RCLCPP_INFO(logger, "  d×F(贡献)   : [%s]",
                vec3_to_str(d.cross(raw.force)).c_str());
    RCLCPP_INFO(logger, "  res.torque   : [%s]  (法兰, 代码输出)",
                vec3_to_str(res.torque).c_str());
    RCLCPP_INFO(logger, "  expect torque: [%s]  (公式手算)",
                vec3_to_str(expected_torque).c_str());

    // Step6: 验证（容差 1e-3 Nm，比前几组更严）
    return check_torque(logger, tag + " torque一致性", res.torque, expected_torque, 1e-3);
};

// ── 位形 A：原始工作位形 ─────────────────────────────────────────
// q = [0°, -30°, 60°, -30°, 90°, 0°]，已知条件数≈17.7

// Case5-1: 纯力矩（无力分量）→ d×F = 0，torque 原样传递
// 这是最直接的特例验证：force=0 时 torque_flange 必须等于 torque_wrist3
{
    total_count++;
    Eigen::VectorXd F(6);
    F << 5.0, -3.0, 2.0,   0.0, 0.0, 0.0;
    if (verify_flange_transform("Case5-1: 纯力矩_force=0_传递不变", q, qd_zero, qdd_zero, F))
        pass_count++;
}

// Case5-2: 全 6D 非零力旋量，位形 A，静态
{
    total_count++;
    Eigen::VectorXd F(6);
    F << 2.0, -3.0, 1.5,   8.0, -5.0, -12.0;
    if (verify_flange_transform("Case5-2: 完整6D力旋量_位形A_静态", q, qd_zero, qdd_zero, F))
        pass_count++;
}

// Case5-3: 全 6D 非零力旋量，位形 A，动态（有速度加速度）
{
    total_count++;
    Eigen::VectorXd F(6);
    F << 2.0, -3.0, 1.5,   8.0, -5.0, -12.0;
    if (verify_flange_transform("Case5-3: 完整6D力旋量_位形A_动态", q, qd, qdd, F))
        pass_count++;
}

// ── 位形 B：完全不同的关节角，wrist 充分旋转 ─────────────────────
// q = [45°, -45°, 90°, -45°, 60°, 30°]
// 目的：验证 R_flange 不同时，坐标变换仍然正确
{
    Eigen::VectorXd q_B(6);
    q_B << M_PI/4.0, -M_PI/4.0, M_PI/2.0, -M_PI/4.0, M_PI/3.0, M_PI/6.0;

    // Case5-4: 位形B，大幅值 6D 力旋量，静态
    {
        total_count++;
        Eigen::VectorXd F(6);
        F << -1.0, 4.0, -2.5,   6.0, 3.0, -9.0;
        if (verify_flange_transform("Case5-4: 完整6D力旋量_位形B_静态", q_B, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case5-5: 位形B，纯力矩（再次验证 force=0 的特例）
    {
        total_count++;
        Eigen::VectorXd F(6);
        F << -4.0, 2.0, -6.0,   0.0, 0.0, 0.0;
        if (verify_flange_transform("Case5-5: 纯力矩_位形B_传递不变", q_B, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case5-6: 位形B，动态状态
    {
        total_count++;
        Eigen::VectorXd F(6);
        F << 1.0, -2.0, 3.0,   0.0, 0.0, -19.62;
        if (verify_flange_transform("Case5-6: 6D力旋量_位形B_动态", q_B, qd, qdd, F))
            pass_count++;
    }
}

// ── 位形 C：更大幅度旋转 ─────────────────────────────────────────
// q = [-30°, -60°, 120°, -30°, 45°, -45°]
// 目的：再换一组位形，排除特定 R_flange 巧合
{
    Eigen::VectorXd q_C(6);
    q_C << -M_PI/6.0, -M_PI/3.0, 2.0*M_PI/3.0, -M_PI/6.0, M_PI/4.0, -M_PI/4.0;

    // Case5-7: 大幅值 6D 力旋量，静态
    {
        total_count++;
        Eigen::VectorXd F(6);
        F << 3.0, -1.0, 0.5,   15.0, -8.0, 6.0;
        if (verify_flange_transform("Case5-7: 大幅值6D力旋量_位形C_静态", q_C, qd_zero, qdd_zero, F))
            pass_count++;
    }

    // Case5-8: 位形C，纯力矩（三次验证 force=0 特例）
    {
        total_count++;
        Eigen::VectorXd F(6);
        F << 8.0, -6.0, 4.0,   0.0, 0.0, 0.0;
        if (verify_flange_transform("Case5-8: 纯力矩_位形C_传递不变", q_C, qd_zero, qdd_zero, F))
            pass_count++;
    }
}

    // ================================================================
    // 汇总
    // ================================================================
    RCLCPP_INFO(logger, "========================================");
    if (pass_count == total_count) {
        RCLCPP_INFO(logger,
            "[SUCCESS] 全部通过 %d / %d", pass_count, total_count);
    } else {
        RCLCPP_ERROR(logger,
            "[FAILED]  通过 %d / %d，请检查上方 FAIL 项",
            pass_count, total_count);
    }
    RCLCPP_INFO(logger, "========================================");
}

} // namespace TestRusSimForceVal