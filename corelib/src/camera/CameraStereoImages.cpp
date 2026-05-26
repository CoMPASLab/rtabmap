/*
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

#include <rtabmap/core/camera/CameraStereoImages.h>
#include <rtabmap/utilite/UStl.h>
#include <opencv2/imgproc/types_c.h>
#include <algorithm>
#include <opencv2/imgcodecs.hpp>

namespace rtabmap
{

bool CameraStereoImages::available()
{
	return true;
}

CameraStereoImages::CameraStereoImages(
		const std::string & pathLeftImages,
		const std::string & pathRightImages,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		CameraImages(pathLeftImages, imageRate, localTransform),
		camera2_(new CameraImages(pathRightImages)),
		rightGrayScale_(true)
{
	this->setImagesRectified(rectifyImages);
}

CameraStereoImages::CameraStereoImages(
		const std::string & pathLeftRightImages,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
		CameraImages("", imageRate, localTransform),
		camera2_(0),
		rightGrayScale_(true)
{
	std::vector<std::string> paths = uListToVector(uSplit(pathLeftRightImages, uStrContains(pathLeftRightImages, ":")?':':';'));
	if(paths.size() >= 1)
	{
		this->setPath(paths[0]);
		this->setImagesRectified(rectifyImages);

		if(paths.size() >= 2)
		{
			camera2_ = new CameraImages(paths[1]);
		}
	}
	else
	{
		UERROR("The path is empty!");
	}
}

CameraStereoImages::~CameraStereoImages()
{
	UDEBUG("");
	delete camera2_;
	UDEBUG("");
}

bool CameraStereoImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UINFO("Calibration folder: \"%s\", name=\"%s\"", calibrationFolder.c_str(), cameraName.c_str());

	// look for calibration files
	if(!calibrationFolder.empty() && !cameraName.empty())
	{
		if(!stereoModel_.load(calibrationFolder, cameraName, false) && !stereoModel_.isValidForProjection())
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
					cameraName.c_str(), calibrationFolder.c_str());
		}
		else
		{
			UINFO("Stereo parameters: fx=%f cx=%f cy=%f baseline=%f",
					stereoModel_.left().fx(),
					stereoModel_.left().cx(),
					stereoModel_.left().cy(),
					stereoModel_.baseline());
		}
	}

	stereoModel_.setLocalTransform(this->getLocalTransform());
	stereoModel_.setName(cameraName);
	if(this->isImagesRectified() && !stereoModel_.isValidForRectification())
	{
		UWARN("Parameter \"rectifyImages\" is set, but no stereo model is loaded or valid for rectification. This can be ignored if input images are already rectified.");
	}

	//desactivate before init as we will do it in this class instead for convenience
	bool rectify = this->isImagesRectified();
	this->setImagesRectified(false);

	rightStamps_.clear();
	rightFilenames_.clear();
	rightPath_.clear();

	bool success = false;
	if(CameraImages::init())
	{
		if(camera2_)
		{
			camera2_->setBayerMode(this->getBayerMode());
			camera2_->setTimestamps(this->isFilenamesAreTimestamps(), "", this->isSyncImageRateWithStamps());
			if(camera2_->init())
			{
				const std::list<double> & leftStampsList  = this->stamps();
				const std::list<double> & rightStampsList = camera2_->stamps();

				if(!leftStampsList.empty() && !rightStampsList.empty())
				{
					// Build sorted (stamp, filename) pairs for the right side so binary
					// search in captureImage() is valid even if filenames were not
					// perfectly monotonic.
					std::vector<std::string> rf = camera2_->filenames();
					UASSERT_MSG(rightStampsList.size() == rf.size(),
						uFormat("Right stamps (%d) and filenames (%d) count mismatch",
							(int)rightStampsList.size(), (int)rf.size()).c_str());

					std::vector<std::pair<double,std::string>> pairs;
					pairs.reserve(rf.size());
					auto sit = rightStampsList.begin();
					for(size_t i = 0; i < rf.size(); ++i, ++sit)
						pairs.push_back({*sit, rf[i]});
					std::sort(pairs.begin(), pairs.end());

					rightStamps_.reserve(pairs.size());
					rightFilenames_.reserve(pairs.size());
					for(auto & p : pairs)
					{
						rightStamps_.push_back(p.first);
						rightFilenames_.push_back(p.second);
					}
					rightPath_ = camera2_->getPath();

					UINFO("Stereo nearest-timestamp matching enabled: %d left, %d right frames",
						(int)leftStampsList.size(), (int)rightStamps_.size());
					success = true;
				}
				else if(this->imagesCount() == camera2_->imagesCount())
				{
					success = true;
				}
				else
				{
					UERROR("Cameras don't have the same number of images (%d vs %d). "
						"Enable \"Filenames are timestamps\" on both sides to use "
						"nearest-timestamp matching with unequal counts.",
						this->imagesCount(), camera2_->imagesCount());
				}
			}
			else
			{
				UERROR("Cannot initialize the second camera.");
			}
		}
		else
		{
			success = true;
		}
	}
	this->setImagesRectified(rectify); // reset the flag
	return success;
}

bool CameraStereoImages::isCalibrated() const
{
	return stereoModel_.isValidForProjection();
}

std::string CameraStereoImages::getSerial() const
{
	return stereoModel_.name();
}

SensorData CameraStereoImages::captureImage(SensorCaptureInfo * info)
{
	SensorData data;

	SensorData left = CameraImages::captureImage(info);
	if(left.imageRaw().empty())
		return data;

	cv::Mat rightImage;

	if(!rightStamps_.empty())
	{
		// Nearest-timestamp matching: find the right frame closest in time to this left frame.
		double leftStamp = left.stamp();
		auto it = std::lower_bound(rightStamps_.begin(), rightStamps_.end(), leftStamp);
		int rightIdx;
		if(it == rightStamps_.end())
			rightIdx = (int)rightStamps_.size() - 1;
		else if(it == rightStamps_.begin())
			rightIdx = 0;
		else
		{
			auto prev = std::prev(it);
			rightIdx = (leftStamp - *prev) <= (*it - leftStamp)
				? (int)(prev - rightStamps_.begin())
				: (int)(it  - rightStamps_.begin());
		}

		UDEBUG("Left stamp=%f matched right[%d] stamp=%f (diff=%fs)",
			leftStamp, rightIdx, rightStamps_[rightIdx],
			leftStamp - rightStamps_[rightIdx]);

		rightImage = cv::imread(rightPath_ + rightFilenames_[rightIdx], cv::IMREAD_UNCHANGED);

		if(!rightImage.empty())
		{
			int bayerMode = this->getBayerMode();
			if(rightImage.channels() > 3)
			{
				cv::Mat tmp;
				cv::cvtColor(rightImage, tmp, CV_BGRA2BGR);
				rightImage = tmp;
			}
			else if(bayerMode >= 0 && bayerMode <= 3)
			{
				cv::Mat tmp;
				cv::cvtColor(rightImage, tmp, CV_BayerBG2BGR + bayerMode);
				rightImage = tmp;
			}
		}
		else
		{
			UERROR("Failed to load right image: %s", (rightPath_ + rightFilenames_[rightIdx]).c_str());
		}
	}
	else
	{
		SensorData right;
		if(camera2_)
		{
			camera2_->setBayerMode(this->getBayerMode());
			right = camera2_->takeImage(info);
		}
		else
		{
			right = this->takeImage(info);
		}
		rightImage = right.imageRaw();
	}

	if(!rightImage.empty())
	{
		cv::Mat leftImage = left.imageRaw();
		if(rightImage.type() != CV_8UC1 && rightGrayScale_)
		{
			cv::Mat tmp;
			cv::cvtColor(rightImage, tmp, CV_BGR2GRAY);
			rightImage = tmp;
		}
		if(this->isImagesRectified() && stereoModel_.isValidForRectification())
		{
			leftImage = stereoModel_.left().rectifyImage(leftImage);
			rightImage = stereoModel_.right().rectifyImage(rightImage);
		}
		if(stereoModel_.left().imageHeight() == 0 || stereoModel_.left().imageWidth() == 0)
		{
			stereoModel_.setImageSize(leftImage.size());
		}
		data = SensorData(left.laserScanRaw(), leftImage, rightImage, stereoModel_, left.id()/(camera2_?1:2), left.stamp());
		data.setGroundTruth(left.groundTruth());
	}
	return data;
}

} // namespace rtabmap
