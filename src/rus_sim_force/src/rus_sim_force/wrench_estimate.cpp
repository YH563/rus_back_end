#include"rus_sim_force/wrench_estimate.hpp"

namespace RusSimForce{

WrenchEstimate::WrenchEstimate(const std::string& urdf_path) {
    if (!load_urdf(urdf_path)) {
        throw std::runtime_error("[EeWrenchEstimator] URDF 加载失败: " + urdf_path);
    }
}

bool WrenchEstimate::load_urdf(const std::string& urdf_path) {
    urdf::Model model;
    // 检查返回值
    if (!model.initFile(urdf_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("urdf_loader"),"Failed to load URDF: %s", urdf_path.c_str());
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("urdf_loader"),"Robot name: %s", model.getName().c_str());  //读取名字

    links_.clear();
    joints_.clear();//清缓存

    std::map<std::string, int> link_index_map; //绑定机械臂部位名称与索引序号

    // ── 解析 link ─────────────────────────────────────────
    for (const auto& kv : model.links_) {
        const auto& link = kv.second;

        LinkParam lp;   //初始化LinkParam对象
        lp.name    = link->name;
        lp.mass    = 0.0;
        lp.com     = Eigen::Vector3d::Zero();
        lp.inertia = Eigen::Matrix3d::Zero();

        if (link->inertial) {
            lp.mass = link->inertial->mass;  //传入质量

            const auto& pos = link->inertial->origin.position;  //传入初始位姿
            lp.com = Eigen::Vector3d(pos.x, pos.y, pos.z);

            const auto& I = link->inertial;  //传入惯性张量
            lp.inertia << I->ixx, I->ixy, I->ixz,
                        I->ixy, I->iyy, I->iyz,
                        I->ixz, I->iyz, I->izz;
        }

        link_index_map[lp.name] = static_cast<int>(links_.size()); //建立名称到索引的映射关系
        links_.push_back(lp);
    }

    // ── 解析 revolute joint ───────────────────────────────
    for (const auto& kv : model.joints_) {
        const auto& joint = kv.second;
        //旋转关节判断
        if (joint->type != urdf::Joint::REVOLUTE) {
            continue;
        }
        //关节初始化
        JointParam jp;
        jp.name       = joint->name;
        jp.parent_idx = -1;
        jp.child_idx  = -1;
        jp.axis       = Eigen::Vector3d(0.0, 0.0, 1.0);
        //建立关节前后索引
        auto it_p = link_index_map.find(joint->parent_link_name);
        auto it_c = link_index_map.find(joint->child_link_name);

        if (it_p == link_index_map.end() || it_c == link_index_map.end()) {
            RCLCPP_WARN(rclcpp::get_logger("urdf_loader"),"Joint %s: parent/child link not found, skipped",jp.name.c_str());
            continue;
        }

        jp.parent_idx = it_p->second;
        jp.child_idx  = it_c->second;

         // 获取关节的固定变换（父link坐标系 → 关节坐标系）
        const auto& pose = joint->parent_to_joint_origin_transform;
        const auto& p    = pose.position;   // 平移向量 (x, y, z)
        const auto& q    = pose.rotation;   // 旋转四元数 (x, y, z, w)

        // 构造Eigen四元数（注意：Eigen构造器参数顺序为 w, x, y, z）
        Eigen::Quaterniond eq(q.w, q.x, q.y, q.z);
        
        // 组装4x4齐次变换矩阵
        jp.T_fixed = Eigen::Matrix4d::Identity();           // 初始化为单位矩阵
        jp.T_fixed.block<3, 3>(0, 0) = eq.toRotationMatrix(); // 左上3x3：旋转矩阵
        jp.T_fixed.block<3, 1>(0, 3) = Eigen::Vector3d(p.x, p.y, p.z); // 右上3x1：平移向量

        // 关节旋转轴（在关节坐标系中），归一化确保单位向量
        jp.axis = Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z).normalized();

        joints_.push_back(jp);  // 保存关节参数
    }

    // 检查是否成功加载到旋转关节
    if (joints_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("urdf_loader"),"No revolute joints found in URDF");
        return false;  // 没有可用关节，加载失败
    }

    // 打印加载结果：link数量、关节数量
    RCLCPP_INFO(rclcpp::get_logger("urdf_loader"),"Loaded %zu links, %zu revolute joints",links_.size(), joints_.size());

    return true;  // URDF加载成功
}

void WrenchEstimate::SetGravity(const Eigen::Vector3d& gravity) {
    gravity_ = gravity;
}

void WrenchEstimate::SetFlangeConfig(const FlangeConfig& config) {
    flange_config_ = config;
}

void WrenchEstimate::SetSingularityThreshold(double threshold) {
    singularity_threshold_ = threshold;
}

void WrenchEstimate::SetDampingFactor(double lambda) {
    lambda_ = lambda;
}

void WrenchEstimate::SetToolPayload(double mass, const Eigen::Vector3d& com_in)
{
    // 取末端 link（最后一个关节的子 link）
    const int last_child = joints_.back().child_idx;
    LinkParam& last_link  = links_[last_child];

    const double m0    = last_link.mass;
    const double mt    = mass;
    const double m_new = m0 + mt;

    if (m_new < 1e-12) return;  // 防除零

    // 质量加权平均求新质心
    const Eigen::Vector3d com_new = (m0 * last_link.com + mt * com_in) / m_new;

    // Steiner 平行轴修正项：m*(||d||²·I - d·dᵀ)
    auto steiner = [](double m, const Eigen::Vector3d& d) -> Eigen::Matrix3d {
        return m * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
    };

    // 工具自身惯量视为质点（零），两次平行轴定理合并惯量，针对小质量工具的逻辑简化措施
    last_link.inertia += steiner(m0, last_link.com - com_new)
                       + steiner(mt, com_in   - com_new);

    last_link.com  = com_new;
    last_link.mass = m_new;
}

void WrenchEstimate::SetSimForce(const Eigen::VectorXd& wrench_6d)
{
    if (wrench_6d.size() != 6) {
        throw std::invalid_argument("[WrenchEstimate] SetSimForce 输入必须为 6 维");
    }
    sim_config_.enabled   = true;
    sim_config_.wrench_6d = wrench_6d;
}

void WrenchEstimate::ClearSimForce()
{
    sim_config_.enabled   = false;
    sim_config_.wrench_6d = Eigen::VectorXd::Zero(6);
}

Eigen::Matrix4d WrenchEstimate::rot_z_homogeneous(double q) const
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const double c = std::cos(q);
    const double s = std::sin(q);
    T(0, 0) = c;
    T(0, 1) = -s;
    T(1, 0) = s;
    T(1, 1) = c;
    return T;
}

std::vector<Eigen::Matrix4d> WrenchEstimate::forward_kinematics(const Eigen::VectorXd& q) const
{
    const int dof = static_cast<int>(joints_.size());
 if (q.size() != dof) {
        throw std::invalid_argument("[WrenchEstimate] ForwardKinematics 输入维度与 DOF 不匹配");
    }

    std::vector<Eigen::Matrix4d> T_link(dof + 1);
    T_link[0] = Eigen::Matrix4d::Identity();

    for (int i = 0; i < dof; ++i) {
        T_link[i + 1] = T_link[i] * joints_[i].T_fixed * rot_z_homogeneous(q(i));
    }

    return T_link;
}

Eigen::MatrixXd WrenchEstimate::jacobian(const Eigen::VectorXd& q) const 
{
    const int dof = static_cast<int>(joints_.size());
    const auto T_link = forward_kinematics(q);
    const Eigen::Vector3d p_ee = T_link[dof].block<3, 1>(0, 3);

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, dof);

    for (int i = 0; i < dof; ++i) {
        const Eigen::Matrix4d T_joint_i = T_link[i] * joints_[i].T_fixed;
        const Eigen::Vector3d z_i = T_joint_i.block<3, 1>(0, 2);
        const Eigen::Vector3d p_i = T_joint_i.block<3, 1>(0, 3);

        J.block<3, 1>(0, i) = z_i;
        J.block<3, 1>(3, i) = z_i.cross(p_ee - p_i);
    }

    return J;
}


double WrenchEstimate::compute_condition_number(const Eigen::MatrixXd& J) const
{
    const Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
    const auto& sv = svd.singularValues();
    if (sv.minCoeff() < 1e-12) {
        return 1e18;
    }
    return sv.maxCoeff() / sv.minCoeff();
}

Eigen::VectorXd WrenchEstimate::build_sim_tau_meas(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const
{
    const Eigen::VectorXd tau_model = inverse_dynamics(q, qd, qdd);
    const Eigen::MatrixXd J = jacobian(q);
    return tau_model + J.transpose() * sim_config_.wrench_6d;
}

EndEffectorWrench WrenchEstimate::transform_to_flange(const Eigen::VectorXd& q,const ForceResult& raw) const
{
    const auto T_fk = forward_kinematics(q);
    const Eigen::Matrix3d R_wrist3 = T_fk[6].block<3, 3>(0, 0);
    const Eigen::Vector3d p_wrist3 = T_fk[6].block<3, 1>(0, 3);

    // 法兰原点在 base 坐标系下的位置
    const Eigen::Vector3d p_flange =
        p_wrist3 + R_wrist3 * flange_config_.flange_in_wrist3;
    const Eigen::Matrix3d R_flange = R_wrist3;

    // 力矩从 wrist3 原点搬移到法兰原点：M_B = M_A + r_{AB} × F
    const Eigen::Vector3d torque_at_flange =
        raw.torque + (p_wrist3 - p_flange).cross(raw.force);

    EndEffectorWrench res;
    res.force            = raw.force;
    res.torque           = torque_at_flange;
    res.force_in_flange  = R_flange.transpose() * raw.force;
    res.torque_in_flange = R_flange.transpose() * torque_at_flange;
    res.p_flange         = p_flange;
    res.R_flange         = R_flange;
    res.cond_num         = raw.cond_num;
    res.is_valid         = raw.is_valid;
    return res;
}

Eigen::VectorXd WrenchEstimate::inverse_dynamics(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const
{
    const int dof = static_cast<int>(joints_.size());
    if (q.size() != dof || qd.size() != dof || qdd.size() != dof) {
        throw std::invalid_argument(
            "[WrenchEstimate] inverse_dynamics 输入维度与 DOF 不匹配");
    }

    const auto T_link = forward_kinematics(q);

    // ── 预计算每个关节的位置、轴、质心、惯量（在 base 坐标系下）──
    std::vector<Eigen::Vector3d> o(dof + 1);   // 关节原点
    std::vector<Eigen::Vector3d> z(dof);        // 关节转轴
    std::vector<Eigen::Vector3d> com_b(dof);    // 质心（base 系）
    std::vector<Eigen::Matrix3d> I_base(dof);   // 惯量（base 系）

    for (int i = 0; i < dof; ++i) {
        o[i] = T_link[i].block<3, 1>(0, 3);

        const Eigen::Matrix4d T_joint = T_link[i] * joints_[i].T_fixed;
        z[i] = T_joint.block<3, 1>(0, 2);

        const int child = joints_[i].child_idx;
        const Eigen::Matrix3d R_i = T_link[i + 1].block<3, 3>(0, 0);

        com_b[i]  = T_link[i + 1].block<3, 1>(0, 3) + R_i * links_[child].com;
        I_base[i] = R_i * links_[child].inertia * R_i.transpose();
    }
    o[dof] = T_link[dof].block<3, 1>(0, 3);

    // ── 正向递推：角速度、角加速度、线加速度 ──
    std::vector<Eigen::Vector3d> omega(dof + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> alpha(dof + 1, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> a_o  (dof + 1, Eigen::Vector3d::Zero());
    a_o[0] = -gravity_;   // 将重力等效为根节点处的向上加速度

    for (int i = 0; i < dof; ++i) {
        omega[i + 1] = omega[i] + z[i] * qd(i);
        alpha[i + 1] = alpha[i] + z[i] * qdd(i)
                     + omega[i].cross(z[i] * qd(i));

        const Eigen::Vector3d r_oi = o[i + 1] - o[i];
        a_o[i + 1] = a_o[i]
                   + alpha[i].cross(r_oi)
                   + omega[i].cross(omega[i].cross(r_oi));
    }

    // ── 质心加速度 ──
    std::vector<Eigen::Vector3d> a_c(dof, Eigen::Vector3d::Zero());
    for (int i = 0; i < dof; ++i) {
        const Eigen::Vector3d r_ci = com_b[i] - o[i + 1];
        a_c[i] = a_o[i + 1]
               + alpha[i + 1].cross(r_ci)
               + omega[i + 1].cross(omega[i + 1].cross(r_ci));
    }

    // ── 反向递推：力、力矩、关节力矩 ──
    std::vector<Eigen::Vector3d> f(dof + 2, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> n(dof + 2, Eigen::Vector3d::Zero());
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof);

    for (int i = dof; i >= 1; --i) {
        const int child = joints_[i - 1].child_idx;
        const double m   = links_[child].mass;

        const Eigen::Vector3d F_i = m * a_c[i - 1];
        const Eigen::Vector3d N_i = I_base[i - 1] * alpha[i]
                                  + omega[i].cross(I_base[i - 1] * omega[i]);

        f[i] = F_i + f[i + 1];

        Eigen::Vector3d moment_carry = Eigen::Vector3d::Zero();
        if (i < dof) {
            moment_carry = (o[i + 1] - o[i]).cross(f[i + 1]);
        }

        n[i] = N_i
             + n[i + 1]
             + (com_b[i - 1] - o[i]).cross(F_i)
             + moment_carry;

        tau(i - 1) = n[i].dot(z[i - 1]);
    }

    return tau;
}

ForceResult WrenchEstimate::Estimate(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd,const Eigen::VectorXd& tau_meas) const
{
    const int dof = static_cast<int>(joints_.size());
    if (q.size() != dof || qd.size() != dof ||
        qdd.size() != dof || tau_meas.size() != dof) {
        throw std::invalid_argument(
            "[WrenchEstimate] Estimate 输入维度与 DOF 不匹配");
    }

    // 1. 力矩残差：外力在关节上的"指纹"
    const Eigen::VectorXd tau_model = inverse_dynamics(q, qd, qdd);
    const Eigen::VectorXd tau_ext   = tau_meas - tau_model;

    // 2. 雅可比矩阵与奇异性判断
    const Eigen::MatrixXd J    = jacobian(q);
    const double          cond = compute_condition_number(J);
    const bool            valid = (cond < singularity_threshold_);

    if (!valid) {
        RCLCPP_WARN(rclcpp::get_logger("WrenchEstimate"),
                    "[WrenchEstimate] 条件数 %.2f 超过阈值，结果不可靠", cond);
    }

    // 3. 阻尼最小二乘求解末端力旋量：F = J(JᵀJ + λ²I)⁻¹ τ_ext
    const Eigen::MatrixXd JtJ = J.transpose() * J;
    const Eigen::MatrixXd A   = JtJ + lambda_ * lambda_ *
                                Eigen::MatrixXd::Identity(dof, dof);
    const Eigen::VectorXd F_vec = J * A.ldlt().solve(tau_ext);

    // 4. 打包结果（前3维力矩，后3维力）
    ForceResult result;
    result.torque   = F_vec.head<3>();
    result.force    = F_vec.tail<3>();
    result.cond_num = cond;
    result.is_valid = valid;
    return result;
}


ForceResult WrenchEstimate::Estimate(const RobotState& state) const
{
    return Estimate(state.q, state.qd, state.qdd, state.tau_meas);
}


EndEffectorWrench WrenchEstimate::EstimateAtFlange(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd,const Eigen::VectorXd& tau_meas) const
{
    return transform_to_flange(q, Estimate(q, qd, qdd, tau_meas));
}


EndEffectorWrench WrenchEstimate::EstimateAtFlange(const RobotState& state) const
{
    return EstimateAtFlange(state.q, state.qd, state.qdd, state.tau_meas);
}


EndEffectorWrench WrenchEstimate::EstimateAtFlange(const Eigen::VectorXd& q,const Eigen::VectorXd& qd,const Eigen::VectorXd& qdd) const
{
    if (!sim_config_.enabled) {
        throw std::runtime_error("[WrenchEstimate] 仿真模式未启用，请先调用 SetSimForce()");
    }

    const Eigen::VectorXd tau_meas = build_sim_tau_meas(q, qd, qdd);
    return EstimateAtFlange(q, qd, qdd, tau_meas);
}

std::string vec_to_str(const Eigen::VectorXd& v) {
    std::ostringstream ss;
    for (int i = 0; i < v.size(); ++i) {
        ss << v(i);
        if (i < v.size() - 1) ss << " ";
    }
    return ss.str();
}

std::string vec3_to_str(const Eigen::Vector3d& v) {
    std::ostringstream ss;
    ss << v(0) << " " << v(1) << " " << v(2);
    return ss.str();
}


};