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


// Class for depth measurement messages
class Depth
{
public:
    Depth() {}
    Depth(const float & depth,
          cv::Mat covariance,
            const Transform & baseLinkToDepthSensor = Transform::getIdentity()) :
        originalDepthMeasurement_(depth),
        covariance_(covariance),
        baseLinkToDepthSensor_(baseLinkToDepthSensor)
    {
    }

    const float & originalDepthMeasurement() const {return originalDepthMeasurement_;}
    cv::Mat getCovariance() const {return covariance_;}
    float depthInBaseLink(const Transform & originalOffsetTransform = Transform::getIdentity()) const {
        // The measurement is already in base_link
        // Offset depth by difference between original and rotated base_link_to_depth transforms
        // const float depthDiff = baseLinkToDepthSensor_.z() - (currentBaseLinkRotation * baseLinkToDepthSensor_).z(); 
        
        // Transform matrix with original depth measurement
        Transform depthTransform = Transform::getIdentity();
        depthTransform.z() = originalDepthMeasurement_;
        return (originalOffsetTransform * depthTransform).z();
    }
    const Transform & localTransform() const {return baseLinkToDepthSensor_;}

    bool empty() const
    {
        return baseLinkToDepthSensor_.isNull();
    }

private:
    float originalDepthMeasurement_;
    cv::Mat covariance_;

    // Transform from base link to depth sensor
    Transform baseLinkToDepthSensor_;
};

}


#endif /* DEPTH_H_ */
