#include <diff_base.h>

namespace piper{

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

};
