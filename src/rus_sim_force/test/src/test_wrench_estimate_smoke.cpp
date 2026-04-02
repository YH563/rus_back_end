#include "test_wrench_estimate_smoke.hpp"

using RusSimForce::vec3_to_str;
using RusSimForce::vec_to_str;


void TestRusSimForceSmoke::TestWrenchEstimatorSmoke(const std::string& urdf_path) {
    auto logger = rclcpp::get_logger("TestWrenchEstimatorSmoke");

    RCLCPP_INFO(logger, "========== TestWrenchEstimatorSmoke ==========");

    try {
        // 1. 构造对象
        RusSimForce::WrenchEstimate est(urdf_path);
        RCLCPP_INFO(logger, "[PASS] WrenchEstimate 构造成功");

        // 2. 最小输入
        Eigen::VectorXd q        = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qd       = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd qdd      = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd tau_meas = Eigen::VectorXd::Zero(6);

        // 3. 原始估算接口
        const RusSimForce::ForceResult raw = est.Estimate(q, qd, qdd, tau_meas);
        RCLCPP_INFO(logger, "[PASS] Estimate() 调用成功");
        RCLCPP_INFO(logger, "       force  = [%s]", vec3_to_str(raw.force).c_str());
        RCLCPP_INFO(logger, "       torque = [%s]", vec3_to_str(raw.torque).c_str());
        RCLCPP_INFO(logger, "       cond   = %.4f, valid = %s",
            raw.cond_num, raw.is_valid ? "true" : "false");

        // 4. 法兰接口（真实输入模式）
        const RusSimForce::EndEffectorWrench flange_res =est.EstimateAtFlange(q, qd, qdd, tau_meas);
        RCLCPP_INFO(logger, "[PASS] EstimateAtFlange(q,qd,qdd,tau) 调用成功");
        RCLCPP_INFO(logger, "       p_flange       = [%s]", vec3_to_str(flange_res.p_flange).c_str());
        RCLCPP_INFO(logger, "       force(base)    = [%s]", vec3_to_str(flange_res.force).c_str());
        RCLCPP_INFO(logger, "       torque(base)   = [%s]", vec3_to_str(flange_res.torque).c_str());
        RCLCPP_INFO(logger, "       force(flange)  = [%s]", vec3_to_str(flange_res.force_in_flange).c_str());
        RCLCPP_INFO(logger, "       torque(flange) = [%s]", vec3_to_str(flange_res.torque_in_flange).c_str());

        // 5. RobotState 接口
        RusSimForce::RobotState state;
        state.q        = q;
        state.qd       = qd;
        state.qdd      = qdd;
        state.tau_meas = tau_meas;

        const RusSimForce::ForceResult raw_from_state   = est.Estimate(state);
        const RusSimForce::EndEffectorWrench flange_from_state = est.EstimateAtFlange(state);

        RCLCPP_INFO(logger, "[PASS] RobotState 接口调用成功");
        RCLCPP_INFO(logger, "       state.force    = [%s]", vec3_to_str(raw_from_state.force).c_str());
        RCLCPP_INFO(logger, "       state.p_flange = [%s]", vec3_to_str(flange_from_state.p_flange).c_str());

        // 6. 仿真注入力接口
        Eigen::VectorXd wrench_6d(6);
        wrench_6d << 0.0, 0.0, 0.0,   0.0, 0.0, -19.62;
        est.SetSimForce(wrench_6d);

        const RusSimForce::EndEffectorWrench sim_res = est.EstimateAtFlange(q, qd, qdd);

        RCLCPP_INFO(logger, "[PASS] SetSimForce() + EstimateAtFlange(q,qd,qdd) 调用成功");
        RCLCPP_INFO(logger, "       sim.force(base)    = [%s]", vec3_to_str(sim_res.force).c_str());
        RCLCPP_INFO(logger, "       sim.torque(base)   = [%s]", vec3_to_str(sim_res.torque).c_str());
        RCLCPP_INFO(logger, "       sim.force(flange)  = [%s]", vec3_to_str(sim_res.force_in_flange).c_str());
        RCLCPP_INFO(logger, "       sim.torque(flange) = [%s]", vec3_to_str(sim_res.torque_in_flange).c_str());

        // 7. 清除仿真配置
        est.ClearSimForce();
        RCLCPP_INFO(logger, "[PASS] ClearSimForce() 调用成功");

        RCLCPP_INFO(logger, "[SUCCESS] WrenchEstimate Smoke Test 全部通过");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "[FAIL] WrenchEstimate Smoke Test 异常: %s", e.what());
    }

    RCLCPP_INFO(logger, "================================================");

} // namespace TestRusSimForce