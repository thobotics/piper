
#ifndef DIFF_ALL_H_
#define DIFF_ALL_H_

#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

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

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/kinematics/JointLimitFactorVector.h>
#include <gpmp2/kinematics/JointLimitFactorPose2Vector.h>
#include <gpmp2/kinematics/VelocityLimitFactorVector.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileBase.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileBase.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileBase.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstaclePlanarSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2Mobile2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLinArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLinArm.h>
#include <gpmp2/obstacle/ObstacleSDFFactorPose2MobileVetLin2Arms.h>
#include <gpmp2/obstacle/ObstacleSDFFactorGPPose2MobileVetLin2Arms.h>
#include <gpmp2/gp/GaussianProcessPriorLinear.h>
#include <gpmp2/gp/GaussianProcessPriorPose2Vector.h>

#include <gpmp2/planner/BatchTrajOptimizer.h>
#include <gpmp2/planner/ISAM2TrajOptimizer.h>

#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/gp/GPutils.h>
#include <gtsam/base/OptionalJacobian.h>
#include <boost/serialization/array.hpp>

#include <diff_drive.h>
#include <GaussianProcessPriorDiff.h>
#include <GaussianProcessInterpolatorDiff.h>

using namespace gpmp2;

namespace piper {

  /* ************************************************************************** */
  gtsam::Values interpolatePose2DiffArmTraj(const gtsam::Values& opt_values,
    const gtsam::SharedNoiseModel Qc_model, double delta_t, size_t inter_step,
    size_t start_index, size_t end_index) {

    Values results;

    double inter_dt = delta_t / static_cast<double>(inter_step + 1);
    size_t result_index = 0;

    for (size_t i = start_index; i < end_index; i++) {

      results.insert(Symbol('x', result_index), opt_values.at<Pose2Vector>(Symbol('x', i)));
      results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', i)));

      for (size_t inter_idx = 1; inter_idx <= inter_step; inter_idx++) {

        result_index++;
        double tau = static_cast<double>(inter_idx) * inter_dt;
        GaussianProcessInterpolatorDiff gp_inter(Qc_model, delta_t, tau);
        Pose2Vector conf1 = opt_values.at<Pose2Vector>(Symbol('x', i));
        Vector vel1  = opt_values.at<Vector>(Symbol('v', i));
        Pose2Vector conf2 = opt_values.at<Pose2Vector>(Symbol('x', i+1));
        Vector vel2  = opt_values.at<Vector>(Symbol('v', i+1));
        Pose2Vector conf  = gp_inter.interpolatePose(conf1, vel1, conf2, vel2);
        Vector vel  = gp_inter.interpolateVelocity(conf1, vel1, conf2, vel2);
        results.insert(Symbol('x', result_index), conf);
        results.insert(Symbol('v', result_index), vel);
      }

      result_index++;
    }

    results.insert(Symbol('x', result_index), opt_values.at<Pose2Vector>(Symbol('x', end_index)));
    results.insert(Symbol('v', result_index), opt_values.at<Vector>(Symbol('v', end_index)));

    return results;
  }


  /* ************************************************************************** */

  // template uses Pose2MobileArmModel as robot type
  typedef ObstacleSDFFactorGP<Pose2MobileArmModel, GaussianProcessInterpolatorDiff>
      ObstacleSDFFactorGPDiffPose2MobileArm;

  gtsam::Values BatchTrajOptimizePose2MobileDiff(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

      return gpmp2::internal::BatchTrajOptimize<Pose2MobileArmModel, GaussianProcessPriorDiff,
          SignedDistanceField, ObstacleSDFFactorPose2MobileArm, ObstacleSDFFactorGPDiffPose2MobileArm,
          JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, start_conf, start_vel, end_conf, end_vel, init_values, setting);
  };

  /// 3D mobile arm specialization
  typedef gpmp2::internal::ISAM2TrajOptimizer<Pose2MobileArmModel, GaussianProcessPriorDiff,
      SignedDistanceField, ObstacleSDFFactorPose2MobileArm, ObstacleSDFFactorGPDiffPose2MobileArm,
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>
  ISAM2TrajOptimizerPose2MobileDiff;

}

#endif
