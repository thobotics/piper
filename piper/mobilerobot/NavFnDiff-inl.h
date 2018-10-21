#include <NavFnDiff.h>
#include <gpmp2/obstacle/SignedDistanceField.h>
#include <gpmp2/obstacle/SDFexception.h>

using namespace std;
using namespace gtsam;
using namespace gpmp2;

namespace piper {

/// hinge loss obstacle cost function, planar version
inline double navCost(const gtsam::Point2& point, const NavPotential& pot,
    gtsam::OptionalJacobian<1, 2> H_point = boost::none) {

  gtsam::Vector2 field_gradient;
  double dist_signed;
  try {
    dist_signed = pot.getSignedDistance(point, field_gradient);
  } catch (SDFQueryOutOfRange&) {
    std::cout << "[navCost] WARNING: querying navigation potential out of range, "
       "assume zero potential cost." << std::endl;
    if (H_point) *H_point = gtsam::Matrix12::Zero();
    return 0.0;
  }

  if (H_point) *H_point = field_gradient.transpose();
  return dist_signed;

}

/* ************************************************************************** */
gtsam::Vector NavFnDiff::evaluateError(
    const gpmp2::Pose2Vector& pose, boost::optional<gtsam::Matrix&> H1) const {

    // if Jacobians used, initialize as zeros
    // size: arm_nr_points_ * DOF
    if (H1) *H1 = Matrix::Zero(1, dof_);

    // allocate cost vector
    Vector err(1);
    gtsam::Pose2 p_pose = pose.pose();

    if (H1) {
      Matrix12 Jerr_point;
      const Point2 current_point(p_pose.x(), p_pose.y());
      err(0) = navCost(current_point, pot_, Jerr_point);

      // chain rules
      H1->row(0) = ( Vector(dof_) << Jerr_point[0],
            Jerr_point[1],  Vector::Zero(dof_-2) ).finished();

    } else {
      const Point2 current_point(p_pose.x(), p_pose.y());
      err(0) = navCost(current_point, pot_);
    }

    return err;

}

} // end piper
