#pragma once

#include <NavPotential.h>

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>
#include <gpmp2/geometry/Pose2Vector.h>

#include <iostream>
#include <vector>

using namespace gpmp2;
using namespace gtsam;

namespace piper{

  /// hinge loss obstacle cost function, planar version
  inline double hingeLossNavCost(const gtsam::Point2& point, const NavPotential& pot,
      double eps, gtsam::OptionalJacobian<1, 2> H_point = boost::none) {

    gtsam::Vector2 field_gradient;
    double dist_signed;
    try {
      dist_signed = pot.getSignedDistance(point, field_gradient);
    } catch (SDFQueryOutOfRange&) {
      std::cout << "[hingeLossNavCost] WARNING: querying signed distance out of range, "
         "assume zero potential cost." << std::endl;
      if (H_point) *H_point = gtsam::Matrix12::Zero();
      return 0.0;
    }

    if (H_point) *H_point = field_gradient.transpose();
    return dist_signed;

    // if (dist_signed < eps) {
    //   // faraway no error
    //   if (H_point) *H_point = gtsam::Matrix12::Zero();
    //   return 0.0;
    //
    // } else {
    //   // outside but > eps or inside object
    //   if (H_point) *H_point = field_gradient.transpose();
    //   return dist_signed - eps;
    // }

  }

  template <class ROBOT>
  class NavFnHingleDiff: public gtsam::NoiseModelFactor1<typename ROBOT::Pose> {

  public:
    // typedefs
    typedef ROBOT Robot;
    typedef typename Robot::Pose Pose;

  private:

    typedef NavFnHingleDiff This;
    typedef gtsam::NoiseModelFactor1<Pose> Base;

    // obstacle cost settings
    double epsilon_;      // distance from object that start non-zero cost

    // arm: planar one, all alpha = 0
    const Robot& robot_;

    const NavPotential& pot_;

    size_t robot_size_ = 1; // robot.nr_body_spheres();

  public:

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /* Default constructor do nothing */
    NavFnHingleDiff() : robot_(Robot()), pot_(NavPotential()) {}

    /**
     * Constructor
     * @param cost_model cost function covariance, should to identity model
     * @param field      signed distance field
     * @param nn_index   nearest neighbour index of signed distance field
     */
    NavFnHingleDiff(gtsam::Key poseKey, const Robot& robot,
        const NavPotential& pot, double cost_sigma, double epsilon) :
          Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey),
          epsilon_(epsilon), robot_(robot), pot_(pot) {}


    virtual ~NavFnHingleDiff() {}


    /// error function
    /// numerical jacobians / analytic jacobians from cost function
    gtsam::Vector evaluateError(const typename Robot::Pose& conf,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const {

          // if Jacobians used, initialize as zeros
          // size: arm_nr_points_ * DOF
          if (H1) *H1 = Matrix::Zero(robot_size_, robot_.dof());

          // run forward kinematics of this configuration
          vector<Point3> sph_centers;
          vector<Matrix> J_px_jp;
          if (H1)
            robot_.sphereCenters(conf, sph_centers, J_px_jp);
          else
            robot_.sphereCenters(conf, sph_centers);


          // allocate cost vector
          Vector err(robot_size_);

          // for each point on arm stick, get error
          for (size_t sph_idx = 0; sph_idx < robot_size_; sph_idx++) {

            const double total_eps = robot_.sphere_radius(sph_idx) + epsilon_;

            // printf("Sphere %d -- %f, %f\n", sph_idx, 
            //     sph_centers[sph_idx].x(), sph_centers[sph_idx].y());

            if (H1) {
              Matrix12 Jerr_point;
              const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());
              err(sph_idx) = hingeLossNavCost(sph_center_2d, pot_, total_eps, Jerr_point);

              // chain rules
              H1->row(sph_idx) = Jerr_point * J_px_jp[sph_idx].topRows<2>();

            } else {
              const Point2 sph_center_2d(sph_centers[sph_idx].x(), sph_centers[sph_idx].y());
              err(sph_idx) = hingeLossNavCost(sph_center_2d, pot_, total_eps);
            }
          }

          return err;
    }


    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** print contents */
    void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
      std::cout << s << "NavFnHingleDiff :" << std::endl;
      Base::print("", keyFormatter);
    }


    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor1",
          boost::serialization::base_object<Base>(*this));
    }
  };

}

// #include <NavFnDiff-inl.h>
