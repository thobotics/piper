/**
 *  @file  ISAM2TrajOptimizerNavFn-inl.h
 *  @brief incremental planner using iSAM2
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  Dec 1, 2015
 **/

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <NavFnDiff.h>
#include <NavFnHingleDiff.h>
#include <gtsam/base/debug.h>

namespace piper {

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    ISAM2TrajOptimizerNavFn(const ROBOT& arm, const SDF& sdf, const NavPotential& pot, const TrajOptimizerSetting& setting) :
      setting_(setting), arm_(arm), sdf_(sdf), pot_(pot), isam_(gtsam::ISAM2Params(
      gtsam::ISAM2DoglegParams(), 1e-3, 1)), goal_conf_factor_idx_(0),
      goal_vel_factor_idx_(0) {}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    initFactorGraph(const Pose& start_conf, const Velocity& start_vel,
    const Pose& goal_conf, const Velocity& goal_vel) {

  using namespace gtsam;

  // GP interpolation setting
  const double delta_t = setting_.total_time / static_cast<double>(setting_.total_step);
  const double inter_dt = delta_t / static_cast<double>(setting_.obs_check_inter + 1);

  // build graph
  for (size_t i = 0; i <= setting_.total_step; i++) {
    Key pose_key = Symbol('x', i);
    Key vel_key = Symbol('v', i);

    // start and end
    if (i == 0) {
      inc_graph_.add(PriorFactor<Pose>(pose_key, start_conf, setting_.conf_prior_model));
      inc_graph_.add(PriorFactor<Velocity>(vel_key, start_vel, setting_.vel_prior_model));

    } else if (i == setting_.total_step) {
      // inc_graph_.add(PriorFactor<Pose>(pose_key, goal_conf, setting_.conf_prior_model));
      // goal_conf_factor_idx_ = inc_graph_.size() - 1;        // cache goal factor index here
      inc_graph_.add(PriorFactor<Velocity>(vel_key, goal_vel, setting_.vel_prior_model));
      // goal_vel_factor_idx_ = inc_graph_.size() - 1;         // cache goal factor index here
      // goal_removed_ = false;
    }

    if (setting_.flag_pos_limit) {
      // joint position limits
      inc_graph_.add(LIMIT_FACTOR_POS(pose_key, setting_.pos_limit_model, setting_.joint_pos_limits_down,
          setting_.joint_pos_limits_up, setting_.pos_limit_thresh));
    }
    if (setting_.flag_vel_limit) {
      // velocity limits
      inc_graph_.add(LIMIT_FACTOR_VEL(vel_key, setting_.vel_limit_model, setting_.vel_limits,
          setting_.vel_limit_thresh));
    }

    // Navigation Function factor
    inc_graph_.add(NavFnDiff(pose_key, pot_, setting_.cost_sigma, arm_.dof()));

    if (i > 0) {
      Key last_pose_key = Symbol('x', i-1);
      Key last_vel_key = Symbol('v', i-1);

      // GP factor
      inc_graph_.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t,
          setting_.Qc_model));
    }
  }
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    initValues(const gtsam::Values& init_values) {

  init_values_ = init_values;
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    update() {

  // update isam
  ISAM2Result result = isam_.update(inc_graph_, init_values_);

  // estimated values
  opt_values_ = isam_.calculateEstimate();

  // clean iSAM used things
  init_values_.clear();
  inc_graph_.resize(0);
  removed_factor_index_.clear();
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    changeGoalConfigAndVel(const Pose& goal_conf, const Velocity& goal_vel) {

  using namespace gtsam;

  // add old eq factor' index in remove list
  removed_factor_index_.push_back(goal_conf_factor_idx_);
  removed_factor_index_.push_back(goal_vel_factor_idx_);

  // add new eq factor of goal conf
  inc_graph_.add(PriorFactor<Pose>(Symbol('x', setting_.total_step), goal_conf,
      setting_.conf_prior_model));
  // cache this factors's idx
  goal_conf_factor_idx_ = isam_.getFactorsUnsafe().size() + inc_graph_.size() - 1;

  // add new eq factor of goal vel
  inc_graph_.add(PriorFactor<Velocity>(Symbol('v', setting_.total_step), goal_vel,
      setting_.vel_prior_model));
  // cache this factors's idx
  goal_vel_factor_idx_ = isam_.getFactorsUnsafe().size() + inc_graph_.size() - 1;
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    removeGoalConfigAndVel() {

  using namespace gtsam;

  // add goal factor index in remove list
  if (!goal_removed_) {
    removed_factor_index_.push_back(goal_conf_factor_idx_);
    removed_factor_index_.push_back(goal_vel_factor_idx_);
    goal_removed_ = true;
  }
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
	  fixConfigAndVel(size_t state_idx, const Pose& conf_fix, const Velocity& vel_fix) {

  using namespace gtsam;

  // fix conf and vel at given pose index
  inc_graph_.add(PriorFactor<Pose>(Symbol('x', state_idx), conf_fix, setting_.conf_prior_model));
  inc_graph_.add(PriorFactor<Velocity>(Symbol('v', state_idx), vel_fix, setting_.vel_prior_model));
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    addPoseEstimate(size_t state_idx, const Pose& pose, const Matrix& pose_cov) {

  using namespace gtsam;

  // estimate for pose at given index
  inc_graph_.add(PriorFactor<Pose>(Symbol('x', state_idx), pose, noiseModel::Gaussian::Covariance(pose_cov)));
}

/* ************************************************************************** */
template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
    class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
void ISAM2TrajOptimizerNavFn<ROBOT, GP, SDF, OBS_FACTOR, OBS_FACTOR_GP, LIMIT_FACTOR_POS, LIMIT_FACTOR_VEL>::
    addStateEstimate(size_t state_idx, const Pose& pose, const Matrix& pose_cov,
    const Velocity& vel, const Matrix& vel_cov) {

  using namespace gtsam;

  // estimate for pose and vel at given index
  inc_graph_.add(PriorFactor<Pose>(Symbol('x', state_idx), pose, noiseModel::Gaussian::Covariance(pose_cov)));
  inc_graph_.add(PriorFactor<Velocity>(Symbol('v', state_idx), vel, noiseModel::Gaussian::Covariance(vel_cov)));
}

}   // namespace piper
