/*
Copyright (c) 2025 Felix Toft
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/core/odometry/OdometryCuVSLAM.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include <cmath>

#ifdef RTABMAP_CUVSLAM
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/StereoCameraModel.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/util3d_transforms.h"
#include <cuvslam/cuvslam2.h>
#include <cuvslam/ground_constraint2.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <cuda_runtime.h>
#include <unordered_set>

// ============================================================================
// Coordinate System Transformation Constants
// Based on Isaac ROS implementation:
// Source: isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp
// ============================================================================

// Transformation converting from
// Canonical ROS Frame (x-forward, y-left, z-up) to
// cuVSLAM Frame       (x-right, y-up, z-backward)
//  x     ->    -z
//  y     ->    -x
//  z     ->     y
const rtabmap::Transform cuvslam_pose_canonical(
    0, -1, 0, 0,
    0, 0, 1, 0,
    -1, 0, 0, 0
);

// Transformation converting from
// cuVSLAM Frame       (x-right, y-up, z-backward) to
// Canonical ROS Frame (x-forward, y-left, z-up)
const rtabmap::Transform canonical_pose_cuvslam = cuvslam_pose_canonical.inverse();

// Transformation converting from
// Optical Frame    (x-right, y-down, z-forward) to
// cuVSLAM Frame    (x-right, y-up, z-backward)
// Optical   ->  cuVSLAM
//    x      ->     x
//    y      ->    -y
//    z      ->    -z
const rtabmap::Transform cuvslam_pose_optical(
    1, 0, 0, 0,
    0, -1, 0, 0,
    0, 0, -1, 0
);

// Transformation converting from
// cuVSLAM Frame    (x-right, y-up, z-backward) to
// Optical Frame    (x-right, y-down, z-forward)
const rtabmap::Transform optical_pose_cuvslam = cuvslam_pose_optical.inverse();


// ============================================================================
// Forward Declarations
// ============================================================================

namespace rtabmap {

bool initializeCuVSLAM(const SensorData & data,
                       std::unique_ptr<cuvslam::Odometry> & odometry,
                       std::unique_ptr<cuvslam::GroundConstraint> & ground_constraint,
                       bool planar_constraints,
                       int multicam_mode,
                       std::vector<uint8_t *> & gpu_left_image_data,
                       std::vector<uint8_t *> & gpu_right_image_data,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes,
                       cudaStream_t & cuda_stream);

cuvslam::Odometry::Config CreateConfiguration(const SensorData & data, int multicam_mode);

bool prepareImages(const SensorData & data,
                   std::vector<cuvslam::Image> & cuvslam_images,
                   std::vector<uint8_t *> & gpu_left_image_data,
                   std::vector<uint8_t *> & gpu_right_image_data,
                   std::vector<size_t> & gpu_left_image_sizes,
                   std::vector<size_t> & gpu_right_image_sizes,
                   cudaStream_t & cuda_stream);

cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance, bool use_raw_covariance);


// ============================================================================
// Transform Conversion Functions and Misc Helpers
// ============================================================================

// Convert RTAB-Map Transform to cuvslam::Pose (quaternion-based)
cuvslam::Pose TocuVSLAMPose(const Transform & rtabmap_transform)
{
    cuvslam::Pose pose;
    Eigen::Quaternionf q = rtabmap_transform.getQuaternionf();
    pose.rotation = {q.x(), q.y(), q.z(), q.w()};
    pose.translation = {rtabmap_transform.x(), rtabmap_transform.y(), rtabmap_transform.z()};
    return pose;
}

// Convert cuvslam::Pose to RTAB-Map Transform
Transform FromcuVSLAMPose(const cuvslam::Pose & pose)
{
    // rotation is (x, y, z, w); Eigen::Quaternionf constructor takes (w, x, y, z)
    Eigen::Quaternionf q(pose.rotation[3], pose.rotation[0], pose.rotation[1], pose.rotation[2]);
    Eigen::Matrix3f rot = q.toRotationMatrix();
    return Transform(
        rot(0,0), rot(0,1), rot(0,2), pose.translation[0],
        rot(1,0), rot(1,1), rot(1,2), pose.translation[1],
        rot(2,0), rot(2,1), rot(2,2), pose.translation[2]
    );
}

} // namespace rtabmap

#endif

// ============================================================================
// OdometryCuVSLAM Class Implementation
// ============================================================================

namespace rtabmap {

OdometryCuVSLAM::OdometryCuVSLAM(const ParametersMap & parameters) :
    Odometry(parameters)
#ifdef RTABMAP_CUVSLAM
    ,
    odometry_(nullptr),
    ground_constraint_(nullptr),
    initialized_(false),
    lost_(false),
    tracking_(false),
    planar_constraints_(false),
    multicam_mode_(0),
    previous_pose_(Transform::getIdentity()),
    last_timestamp_(-1.0),
    gpu_left_image_data_(),
    gpu_right_image_data_(),
    gpu_left_image_sizes_(),
    gpu_right_image_sizes_(),
    cuda_stream_(nullptr)
#endif
{
#ifdef RTABMAP_CUVSLAM
    Parameters::parse(parameters, Parameters::kRegForce3DoF(), planar_constraints_);
    Parameters::parse(parameters, Parameters::kOdomCuVSLAMMulticamMode(), multicam_mode_);
    UASSERT(multicam_mode_ >= 0 && multicam_mode_ <= 2);
    UINFO("%s=%d", Parameters::kOdomCuVSLAMMulticamMode().c_str(), multicam_mode_);
    // Warm up GPU and create CUDA context before tracker initialization
    cuvslam::WarmUpGPU();
#endif
}

OdometryCuVSLAM::~OdometryCuVSLAM()
{
#ifdef RTABMAP_CUVSLAM
    if(cuda_stream_) cudaStreamSynchronize(cuda_stream_);
    odometry_.reset();
    ground_constraint_.reset();

    for(uint8_t * gpu_ptr : gpu_left_image_data_) {
        if(gpu_ptr) cudaFree(gpu_ptr);
    }
    for(uint8_t * gpu_ptr : gpu_right_image_data_) {
        if(gpu_ptr) cudaFree(gpu_ptr);
    }
    if(cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }
#endif
}

void OdometryCuVSLAM::reset(const Transform & initialPose)
{
    Odometry::reset(initialPose);

#ifdef RTABMAP_CUVSLAM
    this->cleanupCuVSLAMResources();
#endif
}

void OdometryCuVSLAM::cleanupCuVSLAMResources()
{
#ifdef RTABMAP_CUVSLAM
    if(cuda_stream_) cudaStreamSynchronize(cuda_stream_);
    odometry_.reset();
    ground_constraint_.reset();

    for(uint8_t * gpu_ptr : gpu_left_image_data_) {
        if(gpu_ptr) cudaFree(gpu_ptr);
    }
    gpu_left_image_data_.clear();
    for(uint8_t * gpu_ptr : gpu_right_image_data_) {
        if(gpu_ptr) cudaFree(gpu_ptr);
    }
    gpu_right_image_data_.clear();
    if(cuda_stream_) {
        cudaStreamDestroy(cuda_stream_);
        cuda_stream_ = nullptr;
    }

    gpu_left_image_sizes_.clear();
    gpu_right_image_sizes_.clear();
    initialized_ = false;
    lost_ = false;
    tracking_ = false;
    previous_pose_ = Transform::getIdentity();
    last_timestamp_ = -1.0;
#endif
}

Transform OdometryCuVSLAM::computeTransform(
    SensorData & data,
    const Transform & guess,
    OdometryInfo * info)
{
#ifdef RTABMAP_CUVSLAM
    UTimer timer;

    UDEBUG("=== computeTransform ENTRY === lost_=%s, tracking_=%s, initialized_=%s",
          lost_ ? "true" : "false",
          tracking_ ? "true" : "false",
          initialized_ ? "true" : "false");

    if(lost_ && tracking_) {
        UDEBUG("EARLY EXIT: lost_ && tracking_ is true, returning null");
        if(info) {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        return Transform();
    }

    if(data.imageRaw().empty() || data.rightRaw().empty())
    {
        UERROR("cuVSLAM odometry only works with stereo cameras! Left: %s, Right: %s",
               data.imageRaw().empty() ? "empty" : "ok",
               data.rightRaw().empty() ? "empty" : "ok");
        return Transform();
    }

    if(data.stereoCameraModels().size() == 0)
    {
        UERROR("cuVSLAM odometry requires stereo camera models!");
        return Transform();
    }

    if(!initialized_)
    {
        if(!initializeCuVSLAM(
            data,
            odometry_,
            ground_constraint_,
            planar_constraints_,
            multicam_mode_,
            gpu_left_image_data_,
            gpu_right_image_data_,
            gpu_left_image_sizes_,
            gpu_right_image_sizes_,
            cuda_stream_))
        {
            UERROR("Failed to initialize cuVSLAM tracker");
            return Transform();
        }
    }

    std::vector<cuvslam::Image> cuvslam_image_objects;
    if(!prepareImages(
        data,
        cuvslam_image_objects,
        gpu_left_image_data_,
        gpu_right_image_data_,
        gpu_left_image_sizes_,
        gpu_right_image_sizes_,
        cuda_stream_))
    {
        UERROR("Failed to prepare images for cuVSLAM");
        return Transform();
    }

    if(!data.imu().empty())
    {
        UWARN("IMU data available but processing not implemented yet");
    }

    if(cuvslam_image_objects.empty()) {
        UERROR("No images prepared for cuVSLAM tracking");
        return Transform();
    }
    if(!odometry_) {
        UERROR("cuVSLAM tracker is null! initialized_: %s", initialized_ ? "true" : "false");
        return Transform();
    }

    // cuVSLAM does not accept an external predicted pose; the internal motion model is used instead.
    if(!guess.isNull()) {
        UDEBUG("External guess provided but ignored: cuVSLAM uses internal motion model only.");
    }

    cuvslam::PoseEstimate pose_estimate;
    try {
        pose_estimate = odometry_->Track(cuvslam_image_objects);
    } catch (const std::exception & e) {
        UERROR("cuVSLAM Track() threw exception: %s", e.what());
        last_timestamp_ = data.stamp();
        if(info) {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        return Transform();
    }

    if(!pose_estimate.world_from_rig.has_value())
    {
        UWARN("cuVSLAM tracking lost (no pose estimate returned)");
        lost_ = true;
        last_timestamp_ = data.stamp();
        if(info) {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        return Transform();
    }

    const cuvslam::PoseWithCovariance & pwc = *pose_estimate.world_from_rig;

    // Validate covariance
    std::array<float, 36> covariance_copy = pwc.covariance;
    for(int i = 0; i < 6; i++)
    {
        float & diag_val = covariance_copy[i*6+i];

        if(!std::isfinite(diag_val) || diag_val < 0.0f)
        {
            diag_val = 9999.0f;
        }
        // Tracker returns near-zero covariance before motion begins; clamp it.
        if(std::abs(diag_val) < 1e-7f)
        {
            diag_val = 0.0001f;
        }
        if(diag_val > 0.1f)
        {
            if(!use_raw_covariance_) {
                UWARN("Covariance diagonal[%d]=%.8f is invalid; marking as lost.", i, diag_val);
                lost_ = true;
                last_timestamp_ = data.stamp();
                if(info) {
                    info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
                    info->timeEstimation = timer.ticks();
                }
                return Transform();
            }
        }
    }

    cv::Mat covMat = convertCuVSLAMCovariance(covariance_copy.data(), use_raw_covariance_);

    // Apply ground constraint
    cuvslam::Pose constrained_pose = pwc.pose;
    if(planar_constraints_ && ground_constraint_) {
        try {
            ground_constraint_->AddNextPose(constrained_pose);
            constrained_pose = ground_constraint_->GetPoseOnGround();
        } catch (const std::exception & e) {
            UERROR("Ground constraint failed: %s", e.what());
            return Transform();
        }
    }

    // Convert cuVSLAM absolute pose to incremental RTAB-Map Transform
    Transform current_pose = FromcuVSLAMPose(constrained_pose);
    current_pose = canonical_pose_cuvslam * current_pose * cuvslam_pose_canonical;
    UASSERT(!previous_pose_.isNull());
    Transform transform = previous_pose_.inverse() * current_pose;

    tracking_ = true;

    if(info)
    {
        info->reg.covariance = covMat;
        info->timeEstimation = timer.ticks();
    }

    // Extract 3D landmarks for visualization and initialization check
    std::vector<cuvslam::Landmark> landmarks;
    try {
        landmarks = odometry_->GetLastLandmarks();
    } catch (const std::exception & e) {
        UDEBUG("GetLastLandmarks() failed: %s", e.what());
    }
    int landmarks_num = static_cast<int>(landmarks.size());

    if(info)
    {
        info->type = kTypeCuVSLAM;

        // Build a fast-lookup set of landmark IDs that were used in this pose estimate.
        // Observations whose IDs appear here are true inliers (green); all other
        // observations are unmatched / not-yet-triangulated features (yellow).
        std::unordered_set<uint64_t> landmark_ids;
        landmark_ids.reserve(landmarks.size());
        for(const auto & lm : landmarks) {
            landmark_ids.insert(lm.id);
        }

        int image_width = data.imageRaw().cols / (int)data.stereoCameraModels().size();

        for(size_t cam_idx = 0; cam_idx < data.stereoCameraModels().size(); ++cam_idx)
        {
            std::vector<cuvslam::Observation> cam_observations;
            try {
                cam_observations = odometry_->GetLastObservations((int)(cam_idx * 2));
            } catch (const std::exception & e) {
                UDEBUG("GetLastObservations(%d) failed: %s", (int)(cam_idx * 2), e.what());
                continue;
            }
            float x_offset = (float)(cam_idx * image_width);
            for(const auto & obs : cam_observations)
            {
                int id = static_cast<int>(obs.id);
                // First camera that sees this landmark wins for the 2D display position.
                if(info->words.find(id) == info->words.end())
                {
                    info->words.insert(std::make_pair(id, cv::KeyPoint(obs.u + x_offset, obs.v, 3)));
                }
                if(landmark_ids.count(obs.id))
                {
                    info->reg.inliersIDs.push_back(id);
                }
            }
        }
        info->features = (int)info->words.size();
        info->reg.inliers = (int)info->reg.inliersIDs.size();

        // Populate localMap with 3D landmark positions in world frame.
        if(!landmarks.empty())
        {
            Transform absolute_pose = this->getPose() * transform;
            for(const auto & landmark : landmarks)
            {
                cv::Point3f pt = util3d::transformPoint(
                    cv::Point3f(landmark.coords[0], landmark.coords[1], landmark.coords[2]),
                    canonical_pose_cuvslam);
                info->localMap.insert(std::make_pair(
                    landmark.id,
                    util3d::transformPoint(pt, absolute_pose)));
            }
        }
        info->localMapSize = (int)landmarks.size();
    }

    if(landmarks_num < min_landmarks_threshold_ && !initialized_) {
        if(info) {
            info->reg.covariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
            info->timeEstimation = timer.ticks();
        }
        cleanupCuVSLAMResources();
        lost_ = true;
        tracking_ = false;
        initialized_ = false;
        return Transform();
    } else {
        initialized_ = true;
    }

    previous_pose_ = current_pose;
    last_timestamp_ = data.stamp();
    UINFO("Odom update time = %fs lost=%s inliers=%d features=%d variance:lin=%f ang=%f local_map=%d",
        timer.elapsed(),
        transform.isNull()?"true":"false",
        info?info->reg.inliers:landmarks_num,
        info?info->features:0,
        info?(float)info->reg.covariance.at<double>(0,0):0.0f,
        info?(float)info->reg.covariance.at<double>(5,5):0.0f,
        landmarks_num);
    return transform;
#else
    UERROR("cuVSLAM support not compiled in RTAB-Map");
    return Transform();
#endif
}

#ifdef RTABMAP_CUVSLAM

// ============================================================================
// cuVSLAM Initialization and Configuration
// ============================================================================

bool initializeCuVSLAM(const SensorData & data,
                       std::unique_ptr<cuvslam::Odometry> & odometry,
                       std::unique_ptr<cuvslam::GroundConstraint> & ground_constraint,
                       bool planar_constraints,
                       int multicam_mode,
                       std::vector<uint8_t *> & gpu_left_image_data,
                       std::vector<uint8_t *> & gpu_right_image_data,
                       std::vector<size_t> & gpu_left_image_sizes,
                       std::vector<size_t> & gpu_right_image_sizes,
                       cudaStream_t & cuda_stream)
{
    cuvslam::SetVerbosity(0);

    cuvslam::Rig rig;
    rig.cameras.resize(data.stereoCameraModels().size() * 2);

    for(size_t i = 0; i < data.stereoCameraModels().size(); ++i)
    {
        const StereoCameraModel & stereoModel = data.stereoCameraModels()[i];
        if(!stereoModel.isValidForProjection())
        {
            UERROR("Invalid stereo camera model %d for cuVSLAM initialization!", static_cast<int>(i));
            return false;
        }
        const CameraModel & leftModel = stereoModel.left();
        const CameraModel & rightModel = stereoModel.right();

        cuvslam::Camera & cam_left  = rig.cameras[i*2];
        cuvslam::Camera & cam_right = rig.cameras[i*2+1];

        // Left camera
        cam_left.size       = {leftModel.imageWidth(), leftModel.imageHeight()};
        cam_left.principal  = {(float)leftModel.cx(), (float)leftModel.cy()};
        cam_left.focal      = {(float)leftModel.fx(), (float)leftModel.fy()};
        cam_left.distortion = cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole, {}};

        // rig_from_camera = cuvslam_pose_canonical * localTransform
        // localTransform maps camera optical frame → rtabmap canonical body frame.
        // When absent from calibration YAML (identity default), substitute the standard
        // optical→canonical rotation so the camera faces forward in the rig.
        rtabmap::Transform lt = stereoModel.localTransform();
        if(lt.isIdentity()) {
            lt = rtabmap::Transform(0, 0, 1, 0,
                                   -1, 0, 0, 0,
                                    0,-1, 0, 0);
        }
        rtabmap::Transform extrinsics = cuvslam_pose_canonical * lt;
        cam_left.rig_from_camera = TocuVSLAMPose(extrinsics);

        // Right camera
        cam_right.size       = {rightModel.imageWidth(), rightModel.imageHeight()};
        cam_right.principal  = {(float)rightModel.cx(), (float)rightModel.cy()};
        cam_right.focal      = {(float)rightModel.fx(), (float)rightModel.fy()};
        cam_right.distortion = cuvslam::Distortion{cuvslam::Distortion::Model::Pinhole, {}};

        // Right camera offset: baseline along optical x-right, applied before lt converts to canonical
        Transform baseline_transform(1, 0, 0, stereoModel.baseline(),
                                     0, 1, 0, 0,
                                     0, 0, 1, 0);
        extrinsics = cuvslam_pose_canonical * lt * baseline_transform;
        cam_right.rig_from_camera = TocuVSLAMPose(extrinsics);
    }

    const cuvslam::Odometry::Config config = CreateConfiguration(data, multicam_mode);

    try {
        odometry = std::make_unique<cuvslam::Odometry>(rig, config);
    } catch (const std::exception & e) {
        UERROR("Failed to initialize cuvslam::Odometry: %s", e.what());
        return false;
    }

    size_t stereo_pairs_count = data.stereoCameraModels().size();
    gpu_left_image_data.resize(stereo_pairs_count, nullptr);
    gpu_right_image_data.resize(stereo_pairs_count, nullptr);
    gpu_left_image_sizes.resize(stereo_pairs_count, 0);
    gpu_right_image_sizes.resize(stereo_pairs_count, 0);

    if(planar_constraints)
    {
        cuvslam::Pose identity_pose;  // default-constructed: identity quaternion, zero translation
        try {
            ground_constraint = std::make_unique<cuvslam::GroundConstraint>(
                identity_pose, identity_pose, identity_pose);
        } catch (const std::exception & e) {
            UERROR("Failed to initialize cuvslam::GroundConstraint: %s", e.what());
            return false;
        }
    }

    return true;
}

/*
Implementation based on Isaac ROS VisualSlamNode::VisualSlamImpl::CreateConfiguration()
Source: isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp:379-422
*/
cuvslam::Odometry::Config CreateConfiguration(const SensorData & data, int multicam_mode)
{
    cuvslam::Odometry::Config config = cuvslam::Odometry::GetDefaultConfig();

    config.use_motion_model          = true;
    config.use_denoising             = false;
    config.use_gpu                   = true;
    config.rectified_stereo_camera   = false;
    config.enable_observations_export = true;
    config.enable_landmarks_export   = true;
    config.odometry_mode             = cuvslam::Odometry::OdometryMode::Multicamera;

    // Map user parameter (0=moderate, 1=performance, 2=precision) to enum
    // cuvslam::Odometry::MulticameraMode: Performance=0, Precision=1, Moderate=2
    switch(multicam_mode) {
        case 0: config.multicam_mode = cuvslam::Odometry::MulticameraMode::Moderate;     break;
        case 1: config.multicam_mode = cuvslam::Odometry::MulticameraMode::Performance;  break;
        case 2: config.multicam_mode = cuvslam::Odometry::MulticameraMode::Precision;    break;
        default: config.multicam_mode = cuvslam::Odometry::MulticameraMode::Moderate;    break;
    }

    return config;
}

// ============================================================================
// GPU Memory Management
// ============================================================================

bool allocateGpuMemory(size_t size, uint8_t ** gpu_ptr, size_t * current_size)
{
    if(*current_size != size) {
        if(*gpu_ptr != nullptr) {
            cudaFree(*gpu_ptr);
        }
        *gpu_ptr = nullptr;
        cudaError_t cuda_err = cudaMalloc(gpu_ptr, size);
        if(cuda_err != cudaSuccess) {
            UERROR("Failed to allocate GPU memory: %s", cudaGetErrorString(cuda_err));
            return false;
        }
        *current_size = size;
    }
    return true;
}

bool copyToGpuAsync(const cv::Mat & cpu_image, uint8_t * gpu_ptr, size_t size, cudaStream_t & cuda_stream)
{
    if(cuda_stream == nullptr) {
        cudaError_t stream_err = cudaStreamCreate(&cuda_stream);
        if(stream_err != cudaSuccess) {
            UERROR("Failed to create CUDA stream: %s", cudaGetErrorString(stream_err));
            return false;
        }
    }

    cudaError_t cuda_err = cudaMemcpyAsync(gpu_ptr, cpu_image.data, size,
                                           cudaMemcpyHostToDevice, cuda_stream);
    if(cuda_err != cudaSuccess) {
        UERROR("Failed to copy image to GPU: %s", cudaGetErrorString(cuda_err));
        return false;
    }

    return true;
}

bool synchronizeGpuOperations(cudaStream_t & cuda_stream)
{
    if(cuda_stream) {
        cudaError_t cuda_err = cudaStreamSynchronize(cuda_stream);
        if(cuda_err != cudaSuccess) {
            UERROR("Failed to synchronize GPU operations: %s", cudaGetErrorString(cuda_err));
            return false;
        }
    }
    return true;
}

// ============================================================================
// Image Processing and Preparation
// ============================================================================

bool prepareImages(const SensorData & data,
                   std::vector<cuvslam::Image> & cuvslam_images,
                   std::vector<uint8_t *> & gpu_left_image_data,
                   std::vector<uint8_t *> & gpu_right_image_data,
                   std::vector<size_t> & gpu_left_image_sizes,
                   std::vector<size_t> & gpu_right_image_sizes,
                   cudaStream_t & cuda_stream)
{
    int64_t timestamp_ns = static_cast<int64_t>(data.stamp() * 1000000000.0);

    cv::Mat left_image  = data.imageRaw();
    cv::Mat right_image = data.rightRaw();

    if(left_image.empty() || right_image.empty()) {
        UERROR("No left or right image available for stereo camera");
        return false;
    }
    if(left_image.channels() != 1 && left_image.channels() != 3) {
        UERROR("Unsupported left image format: %d channels", left_image.channels());
        return false;
    }
    if(right_image.channels() != 1 && right_image.channels() != 3) {
        UERROR("Unsupported right image format: %d channels", right_image.channels());
        return false;
    }

    // cuVSLAM requires the same encoding for both cameras. Always use MONO
    // to avoid mismatches when e.g. the right image has been converted to
    // grayscale by CameraStereoImages but the left is still colour.
    cv::Mat processed_left_image  = left_image;
    cv::Mat processed_right_image = right_image;
    const cuvslam::ImageData::Encoding left_encoding  = cuvslam::ImageData::Encoding::MONO;
    const cuvslam::ImageData::Encoding right_encoding = cuvslam::ImageData::Encoding::MONO;

    if(left_image.channels() != 1) {
        cv::cvtColor(left_image, processed_left_image, cv::COLOR_BGR2GRAY);
    }
    if(right_image.channels() != 1) {
        cv::cvtColor(right_image, processed_right_image, cv::COLOR_BGR2GRAY);
    }

    int camera_index = 0;
    int stereo_index = 0;
    for(const StereoCameraModel & model : data.stereoCameraModels()) {
        int left_w  = model.left().imageWidth();
        int right_w = model.right().imageWidth();
        int left_h  = model.left().imageHeight();
        int right_h = model.right().imageHeight();

        cv::Mat left_slice  = processed_left_image(cv::Rect(stereo_index * left_w,  0, left_w,  left_h)).clone();
        cv::Mat right_slice = processed_right_image(cv::Rect(stereo_index * right_w, 0, right_w, right_h)).clone();

        size_t left_size  = left_slice.total()  * left_slice.elemSize();
        size_t right_size = right_slice.total() * right_slice.elemSize();

        if(!allocateGpuMemory(left_size, &gpu_left_image_data[stereo_index], &gpu_left_image_sizes[stereo_index])) {
            UERROR("PREPARE IMAGES: Failed to allocate GPU memory for left image");
            return false;
        }
        if(!copyToGpuAsync(left_slice, gpu_left_image_data[stereo_index], left_size, cuda_stream)) {
            UERROR("PREPARE IMAGES: Failed to copy left image to GPU");
            return false;
        }

        cuvslam::Image left_img;
        left_img.pixels       = gpu_left_image_data[stereo_index];
        left_img.width        = left_w;
        left_img.height       = left_h;
        left_img.pitch        = static_cast<int32_t>(left_slice.step);
        left_img.encoding     = left_encoding;
        left_img.data_type    = cuvslam::ImageData::DataType::UINT8;
        left_img.is_gpu_mem   = true;
        left_img.timestamp_ns = timestamp_ns;
        left_img.camera_index = camera_index;
        cuvslam_images.push_back(left_img);
        camera_index++;

        if(!allocateGpuMemory(right_size, &gpu_right_image_data[stereo_index], &gpu_right_image_sizes[stereo_index])) {
            UERROR("PREPARE IMAGES: Failed to allocate GPU memory for right image");
            return false;
        }
        if(!copyToGpuAsync(right_slice, gpu_right_image_data[stereo_index], right_size, cuda_stream)) {
            UERROR("PREPARE IMAGES: Failed to copy right image to GPU");
            return false;
        }

        cuvslam::Image right_img;
        right_img.pixels       = gpu_right_image_data[stereo_index];
        right_img.width        = right_w;
        right_img.height       = right_h;
        right_img.pitch        = static_cast<int32_t>(right_slice.step);
        right_img.encoding     = right_encoding;
        right_img.data_type    = cuvslam::ImageData::DataType::UINT8;
        right_img.is_gpu_mem   = true;
        right_img.timestamp_ns = timestamp_ns;
        right_img.camera_index = camera_index;
        cuvslam_images.push_back(right_img);

        stereo_index++;
        camera_index++;
    }

    if(!synchronizeGpuOperations(cuda_stream)) {
        UERROR("PREPARE IMAGES: Failed to synchronize GPU operations");
        return false;
    }

    return true;
}


/*
Convert cuVSLAM covariance to RTAB-Map format.
Based on Isaac ROS implementation: FromcuVSLAMCovariance()
Source: isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp:275-299
*/
cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance, bool use_raw_covariance)
{
    const double scaling_factor = use_raw_covariance ? 1.0 : 10.0;

    if(cuvslam_covariance == nullptr)
    {
        UWARN("Covariance was received as a nullptr, proceeding with default infinite covariance");
        return cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
    }

    // Build rotation matrix for coordinate frame transformation
    // cuVSLAM frame (x-right, y-up, z-backward) to RTAB-Map frame (x-forward, y-left, z-up)
    Eigen::Matrix<float, 3, 3> canonical_pose_cuvslam_mat;
    canonical_pose_cuvslam_mat <<
        canonical_pose_cuvslam.r11(), canonical_pose_cuvslam.r12(), canonical_pose_cuvslam.r13(),
        canonical_pose_cuvslam.r21(), canonical_pose_cuvslam.r22(), canonical_pose_cuvslam.r23(),
        canonical_pose_cuvslam.r31(), canonical_pose_cuvslam.r32(), canonical_pose_cuvslam.r33();

    Eigen::Matrix<float, 6, 6> block_R = Eigen::Matrix<float, 6, 6>::Zero();
    block_R.block<3, 3>(0, 0) = canonical_pose_cuvslam_mat;
    block_R.block<3, 3>(3, 3) = canonical_pose_cuvslam_mat;

    // Map cuVSLAM covariance (row-major) to Eigen matrix
    Eigen::Matrix<double, 6, 6> covariance_mat =
        Eigen::Map<const Eigen::Matrix<float, 6, 6, Eigen::RowMajor>>(cuvslam_covariance).cast<double>();

    // Reorder: cuVSLAM order is (Rx, Ry, Rz, x, y, z); RTAB-Map order is (x, y, z, Rx, Ry, Rz)
    Eigen::Matrix<double, 6, 6> reordered = Eigen::Matrix<double, 6, 6>::Zero();
    reordered.block<3, 3>(0, 0) = covariance_mat.block<3, 3>(3, 3);  // translation-translation
    reordered.block<3, 3>(0, 3) = covariance_mat.block<3, 3>(3, 0);  // translation-rotation
    reordered.block<3, 3>(3, 0) = covariance_mat.block<3, 3>(0, 3);  // rotation-translation
    reordered.block<3, 3>(3, 3) = covariance_mat.block<3, 3>(0, 0);  // rotation-rotation

    Eigen::Matrix<double, 6, 6> block_R_double = block_R.cast<double>();
    Eigen::Matrix<double, 6, 6> cov_transformed =
        block_R_double * reordered * block_R_double.transpose();

    // Scale using a congruence transform D*C*D with
    // D = diag(1,1,1, sqrt(s), sqrt(s), sqrt(s)) so that PSD is preserved:
    //   trans-trans block : x1
    //   trans-rot cross   : x sqrt(s)
    //   rot-rot block     : x s
    const double sqrt_s = std::sqrt(scaling_factor);
    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Identity();
    D(3,3) = D(4,4) = D(5,5) = sqrt_s;
    Eigen::Matrix<double, 6, 6> cov_scaled = D * cov_transformed * D;

    // Guard against non-PSD result: inverting a non-PSD covariance produces
    // negative information-matrix diagonal entries that crash Link::setInfMatrix.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(cov_scaled);
    if(eigensolver.info() != Eigen::Success || eigensolver.eigenvalues().minCoeff() <= 0.0)
    {
        UWARN("cuVSLAM covariance is not positive semi-definite (min eigenvalue=%g); "
              "falling back to infinite covariance. Check stereo calibration.",
              eigensolver.info() == Eigen::Success ? eigensolver.eigenvalues().minCoeff() : 0.0);
        return cv::Mat::eye(6, 6, CV_64FC1) * 9999.0;
    }

    cv::Mat cv_covariance(6, 6, CV_64FC1);
    for(int i = 0; i < 6; i++)
        for(int j = 0; j < 6; j++)
            cv_covariance.at<double>(i, j) = cov_scaled(i, j);

    return cv_covariance;
}

#endif // RTABMAP_CUVSLAM

} // namespace rtabmap
