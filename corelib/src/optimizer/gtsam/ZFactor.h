
/**
 * This file is a copy of GPSPose2Factor.h of gtsam examples
 */

/**
 * A simple 3D 'GPS' like factor
 * The factor contains a Z position measurement (mz) for a Pose
 * The error vector will be [x, y, z-mz]'
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


namespace rtabmap {

template<class VALUE>
class ZFactor: public gtsam::NoiseModelFactor1<VALUE> {

private:
  // measurement information
  double mz_;

public:

  /**
   * Constructor
   * @param poseKey    associated pose variable key
   * @param model      noise model for sensor, in XYZ
   * @param m          Point3 measurement
   */
  ZFactor(gtsam::Key poseKey, const gtsam::Point3 m, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<VALUE>(model, poseKey), mz_(m.z()) {}

  // error function
  // @param p    the pose in Pose
  // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
  gtsam::Vector evaluateError(const gtsam::Pose3& p, boost::optional<gtsam::Matrix&> H = boost::none) const {
    if(H)
    {
      p.translation(H);
    }
    return (gtsam::Vector3() << std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), p.z() - mz_).finished();
  }
  gtsam::Vector evaluateError(const gtsam::Point3& p, boost::optional<gtsam::Matrix&> H = boost::none) const {
    return (gtsam::Vector3() << std::numeric_limits<double>::min(), std::numeric_limits<double>::min(), p.z() - mz_).finished();
  }

};

} // namespace gtsamexamples

