/*
 * ArbitraryPoseConstraint.h
 *
 *  Created on: 2023-04-27
 *      Author: laurtom
 */

 #ifndef ARBITRARYPOSECONSTRAINT_H_
 #define ARBITRARYPOSECONSTRAINT_H_
 
 #include <opencv2/core/core.hpp>
 #include <rtabmap/utilite/UEvent.h>
 #include <rtabmap/core/Transform.h>
 
 namespace rtabmap {
 
 
 // Class for arbitrary pose constraints
 class ArbitraryPoseConstraint
 {
 public:
 
   ArbitraryPoseConstraint() {}
 
   ArbitraryPoseConstraint(const Transform &pose, const cv::Mat &covariance)
       : pose_(pose), covariance_(covariance) {}
 
   Transform pose() const { return pose_; }
 
   cv::Mat covariance() const { return covariance_; }
 
   bool empty() const { return covariance_.empty(); }
 
 private:
 
     Transform pose_;
 
     // 6x6 double
     cv::Mat covariance_;
 };
 
 }
 
 
 #endif /* ARBITRARYPOSECONSTRAINT_H_ */
 