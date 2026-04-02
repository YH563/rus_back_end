#pragma once

#include <string>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <urdf/model.h>

#include <rclcpp/rclcpp.hpp>

namespace RusSimForce {

// 连杆静态惯性参数
struct LinkParam {
    std::string name;           // 连杆名
    double mass;                // 质量 (kg)
    Eigen::Vector3d com;        // 质心在本连杆坐标系下的位置 (m)
    Eigen::Matrix3d inertia;    // 惯性张量关于质心，在连杆坐标系下 (kg·m^2)
};        

// 关节静态运动学参数
struct JointParam {
    std::string name;           // 关节名
    int parent_idx;             // 父连杆索引
    int child_idx;              // 子连杆索引
    Eigen::Matrix4d T_fixed;    // 父连杆到子连杆零位坐标系的固定变换
    Eigen::Vector3d axis;       // 关节转轴
};

// 法兰
struct FlangeConfig {
    Eigen::Vector3d flange_in_wrist3 = {0.0, 0.0, 0.094};  // 法兰原点在 wrist3_link 坐标系下的位置
};

// 外力配置
struct ForceConfig {
    bool enabled = false;
    Eigen::VectorXd wrench_6d = Eigen::VectorXd::Zero(6);  // [nx, ny, nz, fx, fy, fz]
};

// 机械臂状态输入
struct RobotState {
    Eigen::VectorXd q;         // 关节位置
    Eigen::VectorXd qd;        // 关节速度
    Eigen::VectorXd qdd;       // 关节加速度
    Eigen::VectorXd tau_meas;  // 实测关节力矩
};

// wrist3_link 原点处的原始估算结果
struct ForceResult {
    Eigen::Vector3d torque;   // [nx, ny, nz]，base 坐标系
    Eigen::Vector3d force;    // [fx, fy, fz]，base 坐标系
    double cond_num;          // 条件数
    bool is_valid;            // 是否可靠
};

// 法兰原点处的完整末端结果
struct EndEffectorWrench {
    Eigen::Vector3d force;             // 法兰处接触力，base 坐标系
    Eigen::Vector3d torque;            // 法兰处接触力矩，base 坐标系
    Eigen::Vector3d force_in_flange;   // 法兰处接触力，法兰坐标系
    Eigen::Vector3d torque_in_flange;  // 法兰处接触力矩，法兰坐标系
    Eigen::Vector3d p_flange;          // 法兰原点在 base 坐标系下的位置
    Eigen::Matrix3d R_flange;          // 法兰姿态旋转矩阵
    double cond_num = 0.0;
    bool is_valid = false;
};

class WrenchEstimate{
    //机械臂基础配置，硬件角度
    public:
    WrenchEstimate(const std::string& urdf_path);
    void SetGravity(const Eigen::Vector3d& gravity);
    void SetFlangeConfig(const FlangeConfig& config);
    void SetSingularityThreshold(double threshold);
    void SetDampingFactor(double lambda);
    void SetToolPayload(double mass, const Eigen::Vector3d& com_in);
    private:
    bool load_urdf(const std::string& urdf_path);
    
    //运动学
    public:
    void SetSimForce(const Eigen::VectorXd& wrench_6d);
    void ClearSimForce();

    private:
    Eigen::Matrix4d rot_z_homogeneous(double q) const;
    std::vector<Eigen::Matrix4d> forward_kinematics(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const;

    //逆动力学
    private:
    double compute_condition_number(const Eigen::MatrixXd& J) const;//雅可比矩阵条件数
    //Eigen::VectorXd build_sim_tau_meas(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const;// 构建仿真关节力矩测量值
    EndEffectorWrench transform_to_flange(const Eigen::VectorXd& q,const ForceResult& raw) const; //将力/力矩变换到法兰中心
    Eigen::VectorXd inverse_dynamics(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const;

    //对外估算接口
    public:
    Eigen::VectorXd build_sim_tau_meas(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const;// 构建仿真关节力矩测量值

    // 基础估算：返回 wrist3 原点处的力旋量（base 坐标系）
    ForceResult Estimate(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd,const Eigen::VectorXd& tau_meas) const;
    ForceResult Estimate(const RobotState& state) const;

    // 变换到法兰中心（有实测力矩）
    EndEffectorWrench EstimateAtFlange(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd,const Eigen::VectorXd& tau_meas) const;
    EndEffectorWrench EstimateAtFlange(const RobotState& state) const;

    // 变换到法兰中心（仿真模式，无需传入 tau_meas）
    EndEffectorWrench EstimateAtFlange(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const;
    

    //涉及的参数
    private:
    std::vector<JointParam> joints_;
    std::vector<LinkParam> links_;
    Eigen::Vector3d gravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);

    double singularity_threshold_ = 200.0;
    double lambda_ = 0.001;

    FlangeConfig flange_config_;
    ForceConfig sim_config_;

};

    std::string vec_to_str(const Eigen::VectorXd& v);// 辅助：把 Eigen::VectorXd 转成字符串，方便 RCLCPP_INFO 打印
    std::string vec3_to_str(const Eigen::Vector3d& v);// 辅助：把 Eigen::Vector3d 转成字符串，方便 RCLCPP_INFO 打印

}

