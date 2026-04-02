#include"rclcpp/rclcpp.hpp"
#include"rus_sim_force/wrench_estimate.hpp"



#ifdef ENABLE_TESTING
#include "test_wrench_estimate_smoke.hpp"
#include "test_wrench_estimate_val.hpp"
namespace TestRusSimForceSmoke {
void TestWrenchEstimatorSmoke(const std::string& urdf_path);

} 
namespace TestRusSimForceVal {
void TestWrenchDemo(const std::string& urdf_path);

} 
#endif

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto logger = rclcpp::get_logger("main");

    const std::string urdf_path ="src/frcobot_ros2/fairino_description/urdf/fairino3_v6.urdf";

    bool run_test = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--test") {
            run_test = true;
            break;
        }
    }

#ifdef ENABLE_TESTING
    if (run_test) {
        // TestRusSimForceSmoke::TestWrenchEstimatorSmoke(urdf_path);
        TestRusSimForceVal::TestWrenchDemo(urdf_path);
        rclcpp::shutdown();
        return 0;
    }

#else
    if (run_test) {
        RCLCPP_ERROR(logger,
            "[main] 当前编译未包含测试代码，请用 -DENABLE_TEST=ON 重新编译");
        rclcpp::shutdown();
        return 1;
    }
#endif

    rclcpp::shutdown();
    return 0;
}