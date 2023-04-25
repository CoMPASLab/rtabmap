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
            const Transform & baseLinkToDepthSensor = Transform::getIdentity()) :
        originalDepthMeasurement_(depth),
        baseLinkToDepthSensor_(baseLinkToDepthSensor)
    {
    }

    const float & originalDepthMeasurement() const {return originalDepthMeasurement_;}
    float depthInBaseLink(const Transform & currentBaseLinkRotation = Transform::getIdentity()) const {
        // Offset depth by difference between original and rotated base_link_to_depth transforms
        const float depthDiff = baseLinkToDepthSensor_.z() - (currentBaseLinkRotation * baseLinkToDepthSensor_).z(); 
        Transform depthTransform = Transform::getIdentity();
        depthTransform.z() = originalDepthMeasurement_;
        return (baseLinkToDepthSensor_ * depthTransform).z() - depthDiff;
    }
    const Transform & localTransform() const {return baseLinkToDepthSensor_;}

    bool empty() const
    {
        return baseLinkToDepthSensor_.isNull();
    }

private:
    float originalDepthMeasurement_;

    // Transform from base link to depth sensor
    Transform baseLinkToDepthSensor_;
};

}


#endif /* DEPTH_H_ */
