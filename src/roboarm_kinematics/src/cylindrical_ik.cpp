#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>

namespace roboarm_kinematics {
using Pose = geometry_msgs::msg::Pose;
using Error = moveit_msgs::msg::MoveItErrorCodes;
using Options = kinematics::KinematicsQueryOptions;
class CylindricalIK : public kinematics::KinematicsBase {
  std::vector<std::string> joints_, links_;
  double radius_{}, height_{};
  const moveit::core::JointModelGroup* group_{};
  bool solve(const Pose& p, const std::vector<double>& seed, const std::vector<double>& limits,
             std::vector<double>& out, const IKCallbackFn& cb, Error& e) const {
    e.val = Error::NO_IK_SOLUTION;
    if (seed.size() != 2 || (!limits.empty() && limits.size() != 2)) return false;
    const auto& v = p.position;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
        !q.coeffs().allFinite() || std::abs(q.norm() - 1) > 1e-3 ||
        std::abs(std::hypot(v.x, v.y) - radius_) > 0.001) return false;
    for (double value : seed) if (!std::isfinite(value)) return false;
    for (double value : limits) if (!std::isfinite(value) || value < 0) return false;
    const double theta = std::atan2(v.y, v.x);
    const auto expected = Eigen::Quaterniond(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
    if (q.normalized().angularDistance(expected) > 0.01) return false;
    std::vector<std::vector<double>> candidates;
    // Physical joint is bounded, not continuous; enumerate equivalent angles within one turn.
    for (int k = -1; k <= 1; ++k) candidates.push_back({v.z - height_, theta + k * 2 * std::acos(-1.0)});
    std::sort(candidates.begin(), candidates.end(), [&](const auto& a, const auto& b) {
      return std::abs(a[1] - seed[1]) < std::abs(b[1] - seed[1]);
    });
    for (const auto& candidate : candidates) {
      moveit::core::RobotState state(robot_model_);
      state.setToDefaultValues();
      state.setJointGroupPositions(group_, candidate);
      if (!state.satisfiesBounds(group_)) continue;
      if (!limits.empty() && (std::abs(candidate[0]-seed[0]) > limits[0] ||
                              std::abs(candidate[1]-seed[1]) > limits[1])) continue;
      e.val = Error::SUCCESS;
      if (cb) cb(p, candidate, e);
      if (e.val == Error::SUCCESS) {out = candidate; return true;}
    }
    e.val = Error::NO_IK_SOLUTION;
    return false;
  }
public:
  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& model,
                  const std::string& group, const std::string& base,
                  const std::vector<std::string>& tips, double discretization) override {
    if (base != "base_link" || tips != std::vector<std::string>{"tool0"}) return false;
    node_ = node;
    storeValues(model, group, base, tips, discretization);
    group_ = model.getJointModelGroup(group);
    if (!group_) return false;
    joints_ = group_->getVariableNames();
    links_ = group_->getLinkModelNames();
    if (joints_ != std::vector<std::string>{"z_joint", "rotation_joint"}) return false;
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setJointGroupPositions(group_, std::vector<double>{0, 0});
    state.update();
    const auto transform = state.getGlobalLinkTransform(base).inverse() * state.getGlobalLinkTransform(tips[0]);
    radius_ = transform.translation().x();
    height_ = transform.translation().z();
    return radius_ > 0 && std::abs(transform.translation().y()) < 1e-9;
  }
  const std::vector<std::string>& getJointNames() const override {return joints_;}
  const std::vector<std::string>& getLinkNames() const override {return links_;}
  bool getPositionIK(const Pose& p, const std::vector<double>& s, std::vector<double>& o,
                     Error& e, const Options&) const override {return solve(p,s,{},o,{},e);}
  bool searchPositionIK(const Pose& p, const std::vector<double>& s, double,
                        std::vector<double>& o, Error& e, const Options&) const override {return solve(p,s,{},o,{},e);}
  bool searchPositionIK(const Pose& p, const std::vector<double>& s, double,
                        const std::vector<double>& l, std::vector<double>& o, Error& e,
                        const Options&) const override {return solve(p,s,l,o,{},e);}
  bool searchPositionIK(const Pose& p, const std::vector<double>& s, double,
                        std::vector<double>& o, const IKCallbackFn& cb, Error& e,
                        const Options&) const override {return solve(p,s,{},o,cb,e);}
  bool searchPositionIK(const Pose& p, const std::vector<double>& s, double,
                        const std::vector<double>& l, std::vector<double>& o,
                        const IKCallbackFn& cb, Error& e, const Options&) const override {return solve(p,s,l,o,cb,e);}
  bool getPositionFK(const std::vector<std::string>& names, const std::vector<double>& angles,
                     std::vector<Pose>& poses) const override {
    if (angles.size() != 2 || !std::isfinite(angles[0]) || !std::isfinite(angles[1])) return false;
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues(); state.setJointGroupPositions(group_, angles); state.update();
    poses.clear();
    for (const auto& name : names) {
      if (!robot_model_->hasLinkModel(name)) return false;
      const auto t = state.getGlobalLinkTransform(getBaseFrame()).inverse() * state.getGlobalLinkTransform(name);
      const Eigen::Quaterniond q(t.rotation());
      Pose p;
      p.position.x = t.translation().x(); p.position.y = t.translation().y(); p.position.z = t.translation().z();
      p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
      poses.push_back(p);
    }
    return true;
  }
};
} // namespace roboarm_kinematics
PLUGINLIB_EXPORT_CLASS(roboarm_kinematics::CylindricalIK, kinematics::KinematicsBase)
