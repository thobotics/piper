
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
#include <BatchTrajOptimizerNavFn.h>
#include <ISAM2TrajOptimizerNavFn.h>
#include <GaussianProcessPriorDiff.h>
#include <GaussianProcessInterpolatorDiff.h>

using namespace gpmp2;

namespace piper {

  /* ************************************************************************** */
  gtsam::Values interpolatePose2DiffArmTraj(const gtsam::Values&,
    const gtsam::SharedNoiseModel, double, size_t, size_t, size_t);

  /* ************************************************************************** */

  // template uses Pose2MobileArmModel as robot type
  typedef ObstacleSDFFactorGP<Pose2MobileArmModel, GaussianProcessInterpolatorDiff>
      ObstacleSDFFactorGPDiffPose2MobileArm;

  gtsam::Values BatchTrajOptimizePose2MobileDiff(
    const Pose2MobileArmModel& marm, const SignedDistanceField& sdf, const NavPotential& pot,
    const Pose2Vector& start_conf, const gtsam::Vector& start_vel,
    const Pose2Vector& end_conf, const gtsam::Vector& end_vel,
    const gtsam::Values& init_values, const TrajOptimizerSetting& setting) {

      return BatchTrajOptimizeNavFn<Pose2MobileArmModel, GaussianProcessPriorDiff,
          SignedDistanceField, ObstacleSDFFactorPose2MobileArm, ObstacleSDFFactorGPDiffPose2MobileArm,
          JointLimitFactorPose2Vector, VelocityLimitFactorVector>(
          marm, sdf, pot, start_conf, start_vel, end_conf, end_vel, init_values, setting);
  };

  typedef ISAM2TrajOptimizerNavFn<Pose2MobileArmModel, GaussianProcessPriorDiff,
      SignedDistanceField, ObstacleSDFFactorPose2MobileArm, ObstacleSDFFactorGPDiffPose2MobileArm,
      JointLimitFactorPose2Vector, VelocityLimitFactorVector>
  ISAM2TrajOptimizerPose2MobileDiff;

  // /// 3D mobile arm specialization
  // typedef gpmp2::internal::ISAM2TrajOptimizer<Pose2MobileArmModel, GaussianProcessPriorDiff,
  //     SignedDistanceField, ObstacleSDFFactorPose2MobileArm, ObstacleSDFFactorGPDiffPose2MobileArm,
  //     JointLimitFactorPose2Vector, VelocityLimitFactorVector>
  // ISAM2TrajOptimizerPose2MobileDiff;

}

#endif
