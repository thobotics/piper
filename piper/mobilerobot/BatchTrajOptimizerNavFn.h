/**
 *  @file  BatchTrajOptimizer-inl.h
 *  @brief batch trajectory optimizer
 *  @author Jing Dong, Mustafa Mukadam
 *  @date  May 10, 2015
 **/
#pragma once

#include <gpmp2/planner/TrajOptimizerSetting.h>
#include <gpmp2/kinematics/ArmModel.h>
#include <gpmp2/kinematics/Pose2MobileBaseModel.h>
#include <gpmp2/kinematics/Pose2MobileArmModel.h>
#include <gpmp2/kinematics/Pose2Mobile2ArmsModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLinArmModel.h>
#include <gpmp2/kinematics/Pose2MobileVetLin2ArmsModel.h>
#include <gpmp2/obstacle/PlanarSDF.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/config.h>
#include <NavFnDiff.h>
#include <NavFnHingleDiff.h>

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/inference/Symbol.h>

using namespace gpmp2;

namespace piper {

  /* ************************************************************************** */
  template <class ROBOT, class GP, class SDF, class OBS_FACTOR, class OBS_FACTOR_GP,
      class LIMIT_FACTOR_POS, class LIMIT_FACTOR_VEL>
  gtsam::Values BatchTrajOptimizeNavFn(
      const ROBOT& arm, const SDF& sdf, const NavPotential& pot,
      const typename ROBOT::Pose& start_conf, const typename ROBOT::Velocity& start_vel,
      const typename ROBOT::Pose& end_conf, const typename ROBOT::Velocity& end_vel,
      const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

    using namespace gtsam;

    // GP interpolation setting
    const double delta_t = setting.total_time / static_cast<double>(setting.total_step);
    const double inter_dt = delta_t / static_cast<double>(setting.obs_check_inter + 1);

    // build graph
    NonlinearFactorGraph graph;

    for (size_t i = 0; i <= setting.total_step; i++) {
      Key pose_key = Symbol('x', i);
      Key vel_key = Symbol('v', i);

      // start and end
      if (i == 0) {
        graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, start_conf, setting.conf_prior_model));
        graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, start_vel, setting.vel_prior_model));

      } else if (i == setting.total_step) {
        // graph.add(PriorFactor<typename ROBOT::Pose>(pose_key, end_conf, setting.conf_prior_model));
        graph.add(PriorFactor<typename ROBOT::Velocity>(vel_key, end_vel, setting.vel_prior_model));
      }

      if (setting.flag_pos_limit) {
        // joint position limits
        graph.add(LIMIT_FACTOR_POS(pose_key, setting.pos_limit_model, setting.joint_pos_limits_down,
            setting.joint_pos_limits_up, setting.pos_limit_thresh));
      }
      if (setting.flag_vel_limit) {
        // velocity limits
        graph.add(LIMIT_FACTOR_VEL(vel_key, setting.vel_limit_model, setting.vel_limits,
            setting.vel_limit_thresh));
      }

      // Navigation Function factor
      graph.add(NavFnDiff(pose_key, pot, setting.cost_sigma, arm.dof()));

      if (i > 0) {
        Key last_pose_key = Symbol('x', i-1);
        Key last_vel_key = Symbol('v', i-1);

        // GP factor
        graph.add(GP(last_pose_key, last_vel_key, pose_key, vel_key, delta_t,
            setting.Qc_model));
      }
    }

    return optimize(graph, init_values, setting);
  }

}   // namespace piper
