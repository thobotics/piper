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

  class NavFnDiff: public gtsam::NoiseModelFactor1<gpmp2::Pose2Vector> {

  private:
    size_t dof_;

    typedef NavFnDiff This;
    typedef gtsam::NoiseModelFactor1<gpmp2::Pose2Vector> Base;

    const NavPotential& pot_;

  public:

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /* Default constructor do nothing */
    NavFnDiff() : pot_(NavPotential()) {}

    /**
     * Constructor
     * @param cost_sigma cost function covariance, should to identity model
     * @param field      signed distance field
     * @param dof        degree of freedom
     */
    NavFnDiff(gtsam::Key poseKey, const NavPotential& pot,
      double cost_sigma, double dof) :
          Base(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), poseKey),
          pot_(pot), dof_(dof) {}

    virtual ~NavFnDiff() {}


    /// error function
    /// numerical jacobians / analytic jacobians from cost function
    gtsam::Vector evaluateError(const gpmp2::Pose2Vector& pose,
        boost::optional<gtsam::Matrix&> H1 = boost::none) const ;


    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** print contents */
    void print(const std::string& s="", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const {
      std::cout << s << "NavFnDiff :" << std::endl;
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

#include <NavFnDiff-inl.h>
