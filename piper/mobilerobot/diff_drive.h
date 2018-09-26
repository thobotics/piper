#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <cmath>

namespace piper {

  gtsam::Vector3 differential_drive(const gtsam::Vector& pose, const gtsam::Vector& vel,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none){
      using namespace gtsam;

      double phi = pose[2];
      double v = vel[0];
      double w = vel[1];

      double cos_phi = cos(phi);
      double sin_phi = sin(phi);

      if (H1)
         *H1 = (Matrix(3, 3) <<
            Vector3(0,  0,  -v*sin_phi),
            Vector3(0,  0,  v*cos_phi),
            Vector3(0,  0,  0)
          ).finished().transpose();
      if (H2)
         *H2 = (Matrix(2, 3) <<
           Vector2(cos_phi,  0),
           Vector2(sin_phi,  0),
           Vector2(0,        1)
         ).finished().transpose();

      return gtsam::Vector3(v*cos(phi), v*sin(phi), w);
  }
  
}
