/*
 * Depth.h
 *
 *  Created on: 2023-04-17
 *      Author: laurtom
 */

 #ifndef DEPTH_H_
 #define DEPTH_H_
 
 #include <opencv2/core/core.hpp>
 #include <rtabmap/utilite/UEvent.h>
 #include <rtabmap/core/Transform.h>
 
 namespace rtabmap {
 
 
 // Class for depth measurement messages for unary absolute depth constraints
 class Depth
 {
 public:
     // Default initializer
     Depth() {}
 
     // Initializer with depth measurement and variance
     Depth(const float & depth,
           const float & variance,
           const Transform & baseLinkToDepthSensor = Transform::getIdentity())
         : originalDepthMeasurement_(depth),
           variance_(variance),
           baseLinkToDepthSensor_(baseLinkToDepthSensor)
     {}
 
     // Getter for original depth measurement in sensor link
     const float & originalDepthMeasurement() const { return originalDepthMeasurement_; }
 
     // Getter for variance
     float getVariance() const { return variance_; }
 
     // Getter for depth measurement in base link
     float depthInBaseLink(const Transform & originalOffsetTransform = Transform::getIdentity()) const {
         // Initialize depth transformation matrix as the identity matrix
         Transform depthTransform = Transform::getIdentity();
         // Set the transalation z component to the depth measurement in the sensor link in global coordinates
         depthTransform.z() = originalDepthMeasurement_;
         // Note: As defined in corelib/src/Memory.cpp, the originalOffsetTransform is the transform of the world with
         // respecto to the original measurement, i.e., Two. Therefore, to get the relative depth transform (Tov), we
         // need to return Tov = Tow * Twv <-> Tov = originalOffsetTransform * depthTransform.
         return (originalOffsetTransform * depthTransform).z();
     }
 
     // Getter for transform from base link to depth sensor
     const Transform & localTransform() const {return baseLinkToDepthSensor_;}
 
     // Getter to check if the base link to depth measurement is not available
     bool empty() const
     {
         return baseLinkToDepthSensor_.isNull();
     }
 
 private:
     float originalDepthMeasurement_;  // Depth measurement in sensor link
     float variance_;  // Variance of the depth measurement
     Transform baseLinkToDepthSensor_;  // Transform from base link to depth sensor
 };
 
 }
 
 
 #endif /* DEPTH_H_ */
