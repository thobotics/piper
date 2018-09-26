#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gpmp2/geometry/Pose2Vector.h>
#include <gpmp2/gp/GPutils.h>
#include <gpmp2/config.h>
#include <cmath>

using namespace gpmp2;

namespace piper{
  /**
   * 4-way factor for Gaussian Process prior factor, linear version
   */
  class GaussianProcessPriorDiff: public gtsam::NoiseModelFactor4<
      gpmp2::Pose2Vector, gtsam::Vector, gpmp2::Pose2Vector, gtsam::Vector> {

    private:
      size_t dof_;
      double delta_t_;

      typedef GaussianProcessPriorDiff This;
      typedef gtsam::NoiseModelFactor4<gpmp2::Pose2Vector, gtsam::Vector, gpmp2::Pose2Vector,
          gtsam::Vector> Base;

    public:

      GaussianProcessPriorDiff() {}	/* Default constructor only for serialization */

      /// Constructor
      /// @param delta_t is the time between the two states
      GaussianProcessPriorDiff(gtsam::Key poseKey1, gtsam::Key velKey1,
          gtsam::Key poseKey2, gtsam::Key velKey2,
          double delta_t, const gtsam::SharedNoiseModel Qc_model) :
            Base(gtsam::noiseModel::Gaussian::Covariance(calcQ(getQc(Qc_model), delta_t)),
            poseKey1, velKey1, poseKey2, velKey2), dof_(Qc_model->dim()),
            delta_t_(delta_t) {}

      virtual ~GaussianProcessPriorDiff() {}


      /// @return a deep copy of this factor
      virtual gtsam::NonlinearFactor::shared_ptr clone() const {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

      /// factor error function
      gtsam::Vector evaluateError(
          const gpmp2::Pose2Vector& pose1, const gtsam::Vector& vel1,
          const gpmp2::Pose2Vector& pose2, const gtsam::Vector& vel2,
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 = boost::none,
          boost::optional<gtsam::Matrix&> H3 = boost::none,
          boost::optional<gtsam::Matrix&> H4 = boost::none) const {

        using namespace gtsam;

        gtsam::Pose2 p_pose1 = pose1.pose();
        Vector v_pose1 = pose1.configuration();
        gtsam::Pose2 p_pose2 = pose2.pose();
        Vector v_pose2 = pose2.configuration();

        Vector mstate1 = (Vector(dof_) << p_pose1.x(), p_pose1.y(), p_pose1.theta(), v_pose1).finished();
        Vector mstate2 = (Vector(dof_) << p_pose2.x(), p_pose2.y(), p_pose2.theta(), v_pose2).finished();

        // convert naive vector state to differential drive kinematic model
        Matrix Hdiff1_1, Hdiff1_2, Hdiff2_1, Hdiff2_2;
        Vector3 diff_vel1 = piper::differential_drive(
          Vector3(p_pose1.x(), p_pose1.y(), p_pose1.theta()), Vector2(vel1[0], vel1[1]),
          Hdiff1_1, Hdiff1_2
        );
        Vector3 diff_vel2 = piper::differential_drive(
          Vector3(p_pose2.x(), p_pose2.y(), p_pose2.theta()), Vector2(vel2[0], vel2[1]),
          Hdiff2_1, Hdiff2_2
        );
        Vector mvel1 = (Vector(dof_) << diff_vel1, v_pose1.tail(dof_-3)).finished();
        Vector mvel2 = (Vector(dof_) << diff_vel2, v_pose2.tail(dof_-3)).finished();

        // state vector
        Vector x1 = (Vector(2*dof_) << mstate1, mvel1).finished();
        Vector x2 = (Vector(2*dof_) << mstate2, mvel2).finished();

        // Jacobians
        if (H1){
            // gtsam::Matrix de_x1_h = (Matrix(3, 3) <<
            //   Vector3(1.0,  0,    -delta_t_*vel1[0]*sin(p_pose1.theta())),
            //   Vector3(0,    1.0,  delta_t_*vel1[0]*cos(p_pose1.theta())),
            //   Vector3(0,    0,    1.0)
            // ).finished();
            // gtsam::Matrix de_x1_l = (Matrix(3, 3) <<
            //    Vector3(0,    0,   -vel1[0]*sin(p_pose1.theta())),
            //    Vector3(0,    0,   vel1[0]*cos(p_pose1.theta())),
            //    Vector3(0,    0,   0)
            //  ).finished();

            gtsam::Matrix high = (Matrix(dof_, dof_) << Matrix::Identity(3, 3) + Hdiff1_1,
                Matrix::Zero(3, dof_-3), Matrix::Zero(dof_-3, 3),
                Matrix::Identity(dof_-3, dof_-3)).finished();
            gtsam::Matrix low = (Matrix(dof_, dof_) << Hdiff1_1, Matrix::Zero(3, dof_-3),
                Matrix::Zero(dof_-3, 3), Matrix::Zero(dof_-3, dof_-3)).finished();

           *H1 = (Matrix(2*dof_, dof_) << high, low).finished();
          //  *H1 = (Matrix(2*dof_, dof_) << Matrix::Identity(dof_, dof_),
          //   Matrix::Zero(dof_, dof_)).finished();
        }
        if (H2){
          // gtsam::Matrix de_dx1_h = (Matrix(3, 3) <<
          //    Vector3(delta_t_*cos(p_pose1.theta()),  0,  0),
          //    Vector3(delta_t_*sin(p_pose1.theta()),  0,  0),
          //    Vector3(0,    0,    delta_t_)
          //  ).finished();
          //  gtsam::Matrix de_dx1_l = (Matrix(3, 3) <<
          //     Vector3(cos(p_pose1.theta()),    0,   0),
          //     Vector3(sin(p_pose1.theta()),    0,   0),
          //     Vector3(0,    0,   1)
          //   ).finished();
          //
          //  gtsam::Matrix high = (Matrix(dof_, dof_) << de_dx1_h.transpose(), Matrix::Zero(3, dof_-3),
          //      Matrix::Zero(dof_-3, 3), delta_t_ * Matrix::Identity(dof_-3, dof_-3)).finished();
          //  gtsam::Matrix low = (Matrix(dof_, dof_) << de_dx1_l.transpose(), Matrix::Zero(3, dof_-3),
          //      Matrix::Zero(dof_-3, 3), Matrix::Identity(dof_-3, dof_-3)).finished();

          // gtsam::Matrix de_dx1_h = (Matrix(2, 3) <<
          //    Vector2(delta_t_*cos(p_pose1.theta()),  0),
          //    Vector2(delta_t_*sin(p_pose1.theta()),  0),
          //    Vector2(0,     delta_t_)
          //  ).finished();
          //  gtsam::Matrix de_dx1_l = (Matrix(2, 3) <<
          //     Vector2(cos(p_pose1.theta()),  0),
          //     Vector2(sin(p_pose1.theta()),  0),
          //     Vector2(0,  1)
          //   ).finished();

           gtsam::Matrix high = (Matrix(dof_, dof_-1) << delta_t_ * Hdiff1_2,
               Matrix::Zero(3, dof_-3), Matrix::Zero(dof_-3, 2),
               delta_t_ * Matrix::Identity(dof_-3, dof_-3)).finished();
           gtsam::Matrix low = (Matrix(dof_, dof_-1) << Hdiff1_2, Matrix::Zero(3, dof_-3),
               Matrix::Zero(dof_-3, 2), Matrix::Identity(dof_-3, dof_-3)).finished();

          *H2 = (Matrix(2*dof_, dof_-1) << high, low).finished();
          //  *H2 = (Matrix(2*dof_, dof_) << delta_t_ * Matrix::Identity(dof_, dof_),
          //   Matrix::Identity(dof_, dof_)).finished();

        }
        if (H3){

          //  gtsam::Matrix de_x2_h = (Matrix(3, 3) <<
          //    Vector3(-1.0,  0,    0),
          //    Vector3(0,    -1.0,  0),
          //    Vector3(0,    0,    -1.0)
          //  ).finished();
          //  gtsam::Matrix de_x2_l = (Matrix(3, 3) <<
          //     Vector3(0,    0,   vel2[0]*sin(p_pose2.theta())),
          //     Vector3(0,    0,   -vel2[0]*cos(p_pose2.theta())),
          //     Vector3(0,    0,   0)
          //   ).finished();

           gtsam::Matrix high = (Matrix(dof_, dof_) << -1.0 * Matrix::Identity(3, 3),
               Matrix::Zero(3, dof_-3), Matrix::Zero(dof_-3, 3),
               -1.0 * Matrix::Identity(dof_-3, dof_-3)).finished();
           gtsam::Matrix low = (Matrix(dof_, dof_) << -1.0 * Hdiff2_1, Matrix::Zero(3, dof_-3),
               Matrix::Zero(dof_-3, 3), Matrix::Zero(dof_-3, dof_-3)).finished();

          *H3 = (Matrix(2*dof_, dof_) << high, low).finished();
          // *H3 = (Matrix(2*dof_, dof_) << -1.0 * Matrix::Identity(dof_, dof_),
          //   Matrix::Zero(dof_, dof_)).finished();
        }
        if (H4){

          // gtsam::Matrix de_dx2_h = (Matrix(2, 3) <<
          //    Vector3(0,  0,  0),
          //    Vector3(0,  0,  0),
          //    Vector3(0,  0,  0)
          //  ).finished();
          //  gtsam::Matrix de_dx2_l = (Matrix(3, 3) <<
          //     Vector3(-cos(p_pose2.theta()),    0,   0),
          //     Vector3(-sin(p_pose2.theta()),    0,   0),
          //     Vector3(0,    0,   -1)
          //   ).finished();
          //
          //  gtsam::Matrix high = (Matrix(dof_, dof_) << de_dx2_h.transpose(), Matrix::Zero(3, dof_-3),
          //      Matrix::Zero(dof_-3, 3), Matrix::Zero(dof_-3, dof_-3)).finished();
          //  gtsam::Matrix low = (Matrix(dof_, dof_) << de_dx2_l.transpose(), Matrix::Zero(3, dof_-3),
          //      Matrix::Zero(dof_-3, 3), -1.0 * Matrix::Identity(dof_-3, dof_-3)).finished();

          //  gtsam::Matrix de_dx2_h = (Matrix(2, 3) <<
          //    Vector2(0, 0),
          //    Vector2(0, 0),
          //    Vector2(0, 0)
          //  ).finished();
          //  gtsam::Matrix de_dx2_l = (Matrix(2, 3) <<
          //     Vector2(-cos(p_pose2.theta()),   0),
          //     Vector2(-sin(p_pose2.theta()),   0),
          //     Vector2(0,   -1)
          //   ).finished();

           gtsam::Matrix high = (Matrix(dof_, dof_-1) << Matrix::Zero(3, 2), Matrix::Zero(3, dof_-3),
               Matrix::Zero(dof_-3, 2), Matrix::Zero(dof_-3, dof_-3)).finished();
           gtsam::Matrix low = (Matrix(dof_, dof_-1) << -1.0 * Hdiff2_2, Matrix::Zero(3, dof_-3),
               Matrix::Zero(dof_-3, 2), -1.0 * Matrix::Identity(dof_-3, dof_-3)).finished();

          *H4 = (Matrix(2*dof_, dof_-1) << high, low).finished();
          // *H4 = (Matrix(2*dof_, dof_) << Matrix::Zero(dof_, dof_),
          //   -1.0 * Matrix::Identity(dof_, dof_)).finished();
        }

        // transition matrix & error
        return calcPhi(dof_, delta_t_) * x1 - x2;
      }


      /** demensions */
      size_t dim() const { return dof_; }

      /** number of variables attached to this factor */
      size_t size() const {
        return 4;
      }

      /** equals specialized to this factor */
      virtual bool equals(const gtsam::NonlinearFactor& expected, double tol=1e-9) const {
        const This *e =  dynamic_cast<const This*> (&expected);
        return e != NULL && Base::equals(*e, tol) && fabs(this->delta_t_ - e->delta_t_) < tol;
      }

      /** print contents */
      void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
        std::cout << s << "4-way Gaussian Process Factor Linear(" << dof_ << ")" << std::endl;
        Base::print("", keyFormatter);
      }

    private:

      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
        ar & BOOST_SERIALIZATION_NVP(dof_);
        ar & BOOST_SERIALIZATION_NVP(delta_t_);
      }

  }; // GaussianProcessPriorDiff
}
