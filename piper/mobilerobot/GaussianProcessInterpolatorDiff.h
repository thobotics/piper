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
   * 4-way factor for Gaussian Process interpolator, linear version
   * interpolate pose and velocity given consecutive poses and velocities
   */
  class GaussianProcessInterpolatorDiff {

    private:
      typedef GaussianProcessInterpolatorDiff This;

      size_t dof_;
      double delta_t_;		// t_{i+1} - t_i
      double tau_;			// tau - t_i. we use tau as time interval from t_i instead of from t_0 as in Barfoot papers

      gtsam::Matrix Qc_;
      gtsam::Matrix Lambda_;
      gtsam::Matrix Psi_;

    public:

      /// Default constructor: only for serialization
      GaussianProcessInterpolatorDiff() {}

      /**
       * Constructor
       * @param Qc noise model of Qc
       * @param delta_t the time between the two states
       * @param tau the time of interval status
       */
      GaussianProcessInterpolatorDiff(const gtsam::SharedNoiseModel Qc_model,
          double delta_t, double tau) : dof_(Qc_model->dim()), delta_t_(delta_t), tau_(tau) {

        // Calcuate Lambda and Psi
        Qc_ = getQc(Qc_model);
        Lambda_ = calcLambda(Qc_, delta_t_, tau_);
        Psi_ = calcPsi(Qc_, delta_t_, tau_);
      }

      /** Virtual destructor */
      virtual ~GaussianProcessInterpolatorDiff() {}


      /// interpolate pose with Jacobians
      gpmp2::Pose2Vector interpolatePose(
          const gpmp2::Pose2Vector& pose1, const gtsam::Vector& vel1,
          const gpmp2::Pose2Vector& pose2, const gtsam::Vector& vel2,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = boost::none,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = boost::none,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H3 = boost::none,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H4 = boost::none) const {

        using namespace gtsam;

        gtsam::Pose2 p_pose1 = pose1.pose();
        Vector c_pose1 = pose1.configuration();
        gtsam::Pose2 p_pose2 = pose2.pose();
        Vector c_pose2 = pose2.configuration();

        Vector mstate1 = (Vector(dof_) << p_pose1.x(), p_pose1.y(), p_pose1.theta(), c_pose1).finished();
        Vector mstate2 = (Vector(dof_) << p_pose2.x(), p_pose2.y(), p_pose2.theta(), c_pose2).finished();

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
        Vector mvel1 = (Vector(dof_) << diff_vel1, c_pose1.tail(dof_-3)).finished();
        Vector mvel2 = (Vector(dof_) << diff_vel2, c_pose2.tail(dof_-3)).finished();

        // state vector
        Vector x1 = (Vector(2*dof_) << mstate1, mvel1).finished();
        Vector x2 = (Vector(2*dof_) << mstate2, mvel2).finished();

        // jacobians
        if (H1)
          *H1 = (Matrix(dof_, dof_) <<
            Lambda_.block(0, 0, 3, 3) + Lambda_.block(0, dof_, 3, 3)*Hdiff1_1,
            Lambda_.block(0, 3, 3, dof_-3),
            Lambda_.block(3, 0, dof_-3, 3),
            Lambda_.block(3, 3, dof_-3, dof_-3)).finished();
        if (H2)
          *H2 = (Matrix(dof_, dof_-1) <<
            Lambda_.block(0, dof_, 3, 3)*Hdiff1_2, // 3 x 2
            Lambda_.block(0, dof_+3, 3, dof_-3), // 3 x dof_-3
            Lambda_.block(3, dof_, dof_-3, 3)*Hdiff1_2, // dof_-3 x 2
            Lambda_.block(3, dof_+3, dof_-3, dof_-3)).finished(); // dof_-3 x dof_-3
        if (H3)
          *H3 = (Matrix(dof_, dof_) <<
            Psi_.block(0, 0, 3, 3) + Psi_.block(0, dof_, 3, 3)*Hdiff2_1,
            Psi_.block(0, 3, 3, dof_-3),
            Psi_.block(3, 0, dof_-3, 3),
            Psi_.block(3, 3, dof_-3, dof_-3)).finished();
        if (H4)
          *H4 = (Matrix(dof_, dof_-1) <<
            Psi_.block(0, dof_, 3, 3)*Hdiff2_2, // 3 x 2
            Psi_.block(0, dof_+3, 3, dof_-3), // 3 x dof_-3
            Psi_.block(3, dof_, dof_-3, 3)*Hdiff2_2, // dof_-3 x 2
            Psi_.block(3, dof_+3, dof_-3, dof_-3)).finished(); // dof_-3 x dof_-3

        Vector pose_vec = Lambda_.block(0, 0, dof_, 2*dof_) * x1 + Psi_.block(0, 0, dof_, 2*dof_) * x2;
        gpmp2::Pose2Vector pose = gpmp2::Pose2Vector(gtsam::Pose2(pose_vec[0], pose_vec[1], pose_vec[2]), pose_vec.tail(dof_-3));

        // interpolate pose (just calculate upper part of the interpolated state vector to save time)
        return pose;
      }


      /// update jacobian based on interpolated jacobians
      static void updatePoseJacobians(const gtsam::Matrix& Hpose,  const gtsam::Matrix& Hint1,
          const gtsam::Matrix& Hint2, const gtsam::Matrix& Hint3, const gtsam::Matrix& Hint4,
          boost::optional<gtsam::Matrix&> H1, boost::optional<gtsam::Matrix&> H2,
          boost::optional<gtsam::Matrix&> H3, boost::optional<gtsam::Matrix&> H4) {
        if (H1) *H1 = Hpose * Hint1;
        if (H2) *H2 = Hpose * Hint2;
        if (H3) *H3 = Hpose * Hint3;
        if (H4) *H4 = Hpose * Hint4;
      }

      /// interpolate velocity with Jacobians
      gtsam::Vector interpolateVelocity(
          const gpmp2::Pose2Vector& pose1, const gtsam::Vector& vel1,
          const gpmp2::Pose2Vector& pose2, const gtsam::Vector& vel2,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H1 = boost::none,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H2 = boost::none,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H3 = boost::none,
          gtsam::OptionalJacobian<Eigen::Dynamic, Eigen::Dynamic> H4 = boost::none) const {

        using namespace gtsam;

        gtsam::Pose2 p_pose1 = pose1.pose();
        Vector c_pose1 = pose1.configuration();
        gtsam::Pose2 p_pose2 = pose2.pose();
        Vector c_pose2 = pose2.configuration();

        Vector mstate1 = (Vector(dof_) << p_pose1.x(), p_pose1.y(), p_pose1.theta(), c_pose1).finished();
        Vector mstate2 = (Vector(dof_) << p_pose2.x(), p_pose2.y(), p_pose2.theta(), c_pose2).finished();

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
        Vector mvel1 = (Vector(dof_) << diff_vel1, c_pose1.tail(dof_-3)).finished();
        Vector mvel2 = (Vector(dof_) << diff_vel2, c_pose2.tail(dof_-3)).finished();

        // state vector
        Vector x1 = (Vector(2*dof_) << mstate1, mvel1).finished();
        Vector x2 = (Vector(2*dof_) << mstate2, mvel2).finished();

        // interpolate on full state and transform back to control v, w
        Vector mvel = Lambda_.block(dof_, 0, dof_, 2*dof_) * x1 + Psi_.block(dof_, 0, dof_, 2*dof_) * x2;
        int sign = atan2(mvel[0], mvel[1]) > 0 ? 1 : -1;
        double mv = sign * sqrt((pow(mvel[0], 2) + pow(mvel[1], 2)));
        Vector vel = (Vector(dof_-1) << mv, mvel[2], mvel.tail(dof_-3)).finished();

        // Unimplemented jacobians
        if(H1 || H2 || H3 || H4)
          throw runtime_error("[GaussianProcessInterpolatorDiff] TODO: jacobian for interpolated velocity not implemented");

        return vel;
      }

      /** demensions */
      size_t dim() const { return dof_; }


      /**
       * Testables
       */

      /** equals specialized to this factor */
      virtual bool equals(const This& expected, double tol=1e-9) const {
        return fabs(this->delta_t_ - expected.delta_t_) < tol &&
            fabs(this->tau_ - expected.tau_) < tol &&
            gtsam::equal_with_abs_tol(this->Qc_, expected.Qc_, tol) &&
            gtsam::equal_with_abs_tol(this->Lambda_, expected.Lambda_, tol) &&
            gtsam::equal_with_abs_tol(this->Psi_, expected.Psi_, tol);
      }

      /** print contents */
      void print(const std::string& s="") const {
        std::cout << s << "GaussianProcessInterpolatorDiff(" << dof_ << ")" << std::endl;
        std::cout << "delta_t = " << delta_t_ << ", tau = " << tau_ << std::endl;
        //std::cout << "Qc = " << Qc_ << std::endl;
      }


    private:

      /** Serialization function */
      friend class boost::serialization::access;
      template<class ARCHIVE>
      void serialize(ARCHIVE & ar, const unsigned int version) {
        ar & BOOST_SERIALIZATION_NVP(dof_);
        ar & BOOST_SERIALIZATION_NVP(delta_t_);
        ar & BOOST_SERIALIZATION_NVP(tau_);
        using namespace boost::serialization;
        ar & make_nvp("Qc", make_array(Qc_.data(), Qc_.size()));
        ar & make_nvp("Lambda", make_array(Lambda_.data(), Lambda_.size()));
        ar & make_nvp("Psi", make_array(Psi_.data(), Psi_.size()));
      }
  };

}
