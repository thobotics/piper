#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <cmath>

namespace piper {

  gtsam::Vector3 differential_drive(const gtsam::Vector&, const gtsam::Vector&,
    boost::optional<gtsam::Matrix&>,
    boost::optional<gtsam::Matrix&>);

}
