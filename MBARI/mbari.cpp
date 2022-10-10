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

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UProcessInfo.h"
#include "rtabmap/core/IMUFilter.h"
#include <pcl/common/common.h>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <signal.h>
#include <fstream>

#ifdef BUILD_WITH_3D_MAPPING
  #include "MapBuilder.h"
  #include <QApplication>
#endif

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-mbari [options] path\n"
			"  path               Root folder of the sequence (e.g., \"~/mbari-datasets/SE/simulation_0038\")\n"
			"  left_image_dir     Left image directory (e.g., \"color/PROSILICA_L\")\n"
			"  right_image_dir    Right image directory (e.g., \"color/PROSILICA_R\")\n"
			"  --imu_data_file    IMU data file (optional) (e.g., \"imu.csv\")\n"
			"  --imu_calib_file   IMU calib YAML (optional) (e.g., \"imu_calib.yaml\")\n"
			"  --output           Output directory. By default, results are saved in \"path\".\n"
			"  --output_name      Output database name (default \"rtabmap\").\n"
			"  --calib_prefix     Calib file prefix (default \"rtabmap\").\n"
			"  --quiet            Don't show log messages and iteration updates.\n"
			"  --exposure_comp    Do exposure compensation between left and right images.\n"
			"  --disp             Generate full disparity.\n"
			"  --raw              Use raw images (not rectified, this only works with okvis, msckf or vins odometry).\n"
			"%s\n"
			"Example:\n\n"
			"   $ rtabmap-mbari \\\n"
			"       --Rtabmap/PublishRAMUsage true\\\n"
			"       --Rtabmap/DetectionRate 2\\\n"
			"       --RGBD/LinearUpdate 0\\\n"
			"       --Mem/STMSize 30\\\n"
			"       ~/mbari-datasets/SE/simulation_0038\n\n", rtabmap::Parameters::showUsage());
	exit(1);
}

// catch ctrl-c
bool g_forever = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	g_forever = false;
}

int main(int argc, char * argv[])
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	ParametersMap parameters;
	std::string path;
	std::string output;
	std::string outputName = "rtabmap";
	std::string calibPrefix = "rtabmap";
	std::string seq;
	std::string leftImageDirName;
	std::string rightImageDirName;
	std::string imuDataFileName = "";
	std::string imuCalibFileName = "";
	bool disp = false;
	bool raw = true;
	bool exposureCompensation = false;
	bool quiet = false;
	int imuFilter = 1;

    bool useImu = false;

	if(argc < 2)
	{
		showUsage();
	}
	else
	{
		for(int i=1; i<argc; ++i)
		{
			if(std::strcmp(argv[i], "--output") == 0)
			{
				output = argv[++i];
			}
			else if(std::strcmp(argv[i], "--output_name") == 0)
			{
				outputName = argv[++i];
			}
			else if(std::strcmp(argv[i], "--calib_prefix") == 0)
			{
				calibPrefix = argv[++i];
			}
			else if(std::strcmp(argv[i], "--quiet") == 0)
			{
				quiet = true;
			}
			else if(std::strcmp(argv[i], "--disp") == 0)
			{
				disp = true;
			}
			else if(std::strcmp(argv[i], "--raw") == 0)
			{
				raw = true;
			}
			else if(std::strcmp(argv[i], "--exposure_comp") == 0)
			{
				exposureCompensation = true;
			}
			else if(std::strcmp(argv[i], "--imu_data_file") == 0)
			{
				imuDataFileName = argv[++i];
			}
			else if(std::strcmp(argv[i], "--imu_calib_file") == 0)
			{
				imuCalibFileName = argv[++i];
			}
		}
		parameters = Parameters::parseArguments(argc, argv);
		path = argv[1];
		path = uReplaceChar(path, '~', UDirectory::homeDir());
		path = uReplaceChar(path, '\\', '/');
		if(output.empty())
		{
			output = path;
		}
		else
		{
			output = uReplaceChar(output, '~', UDirectory::homeDir());
			UDirectory::makeDir(output);
		}
		leftImageDirName = argv[2];
		rightImageDirName = argv[3];
        if (!imuDataFileName.empty() && !imuCalibFileName.empty()) {
            useImu = true;
        }
        else
        {
            printf("IMU disabled as params were not provided\n");
        }
		parameters.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), output));
		parameters.insert(ParametersPair(Parameters::kRtabmapPublishRAMUsage(), "true"));
		if(raw)
		{
			parameters.insert(ParametersPair(Parameters::kRtabmapImagesAlreadyRectified(), "false"));
		}
	}

	seq = uSplit(path, '/').back();
	std::string pathLeftImages  = path + leftImageDirName;
	std::string pathRightImages = path + rightImageDirName;
	std::string pathImuData = path + imuDataFileName;
	std::string pathImuCalib = path + imuCalibFileName;

	printf("Paths:\n"
			"   Sequence number:  %s\n"
			"   Sequence path:    %s\n"
			"   Output:           %s\n"
			"   Output name:      %s\n"
			"   Calib prefix:     %s\n"
			"   left images:      %s\n"
			"   right images:     %s\n",
			seq.c_str(),
			path.c_str(),
			output.c_str(),
			outputName.c_str(),
			calibPrefix.c_str(),
			pathLeftImages.c_str(),
			pathRightImages.c_str());
	if(useImu)
	{
		printf("   IMU data:         %s\n", pathImuData.c_str());
	    printf("   IMU calib:        %s\n", pathImuCalib.c_str());
		printf("   IMU filter:       %d\n", imuFilter);
	}

	printf("   Exposure Compensation: %s\n", exposureCompensation?"true":"false");
	printf("   Disparity:        %s\n", disp?"true":"false");
	printf("   Raw images:       %s\n", raw?"true (Rtabmap/ImagesAlreadyRectified set to false)":"false");

	if(!parameters.empty())
	{
		printf("Parameters:\n");
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			printf("   %s=%s\n", iter->first.c_str(), iter->second.c_str());
		}
	}
	printf("RTAB-Map version: %s\n", RTABMAP_VERSION);

    YAML::Node left_calib = YAML::LoadFile(path + calibPrefix + "_calib_left.yaml");
    if(left_calib.IsNull())
    {
        UERROR("Cannot open calibration file \"%s\"", (path + calibPrefix + "_calib_left.yaml").c_str());
        return -1;
    }

    UASSERT(left_calib["rate_hz"]);
    float rateHz = left_calib["rate_hz"].as<float>();
    YAML::Node local = left_calib["local_transform"]["data"];
    UASSERT(local.size() == 12);
    Transform baseToCam0(local[0].as<float>(), local[1].as<float>(), local[2].as<float>(), local[3].as<float>(),
                local[4].as<float>(), local[5].as<float>(), local[6].as<float>(), local[7].as<float>(),
                local[8].as<float>(), local[9].as<float>(), local[10].as<float>(), local[11].as<float>());

	int odomStrategy = Parameters::defaultOdomStrategy();
	Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);

	if(quiet)
	{
		ULogger::setLevel(ULogger::kError);
	}

    Transform baseToImu = {cv::Mat::eye(3,4,CV_64FC1)};

    if (useImu) {
        // Load IMU calibration
        YAML::Node config = YAML::LoadFile(pathImuCalib);
        if(config.IsNull())
        {
            UERROR("Cannot open IMU calibration file \"%s\"", pathImuCalib.c_str());
            return -1;
        }

        YAML::Node T_BS = config["T_IMU"];
        YAML::Node data = T_BS["data"];
        UASSERT(data.size() == 16);

        baseToImu = {data[0].as<float>(), data[1].as<float>(), data[2].as<float>(), data[3].as<float>(),
                     data[4].as<float>(), data[5].as<float>(), data[6].as<float>(), data[7].as<float>(),
                     data[8].as<float>(), data[9].as<float>(), data[10].as<float>(), data[11].as<float>()};
    }

	// We use CameraThread only to use postUpdate() method

	CameraThread cameraThread(new
		CameraStereoImages(
				pathLeftImages,
				pathRightImages,
				!raw,
				0.0f,
                baseToCam0 * CameraModel::opticalRotation().inverse()), parameters);
    std::cout << "baseToImu:\n" << baseToImu << std::endl;
	std::cout << "baseToCam0:\n" << baseToCam0 << std::endl;
	std::cout << "imuToCam0:\n" << baseToImu.inverse()*baseToCam0 << std::endl;
	((CameraStereoImages*)cameraThread.camera())->setTimestamps(true, "", false);
	if(exposureCompensation)
	{
		cameraThread.setStereoExposureCompensation(true);
	}
	if(disp)
	{
		cameraThread.setStereoToDepth(true);
	}

	float detectionRate = Parameters::defaultRtabmapDetectionRate();
	bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
	Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), detectionRate);
	Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);

	int mapUpdate = rateHz / detectionRate;
	if(mapUpdate < 1)
	{
		mapUpdate = 1;
	}

	std::string databasePath = output + outputName + ".db";
	UFile::erase(databasePath);
	if(cameraThread.camera()->init(output, calibPrefix + "_calib"))
	{
		int totalImages = (int)((CameraStereoImages*)cameraThread.camera())->filenames().size();

		printf("Processing %d images...\n", totalImages);

		ParametersMap odomParameters = parameters;
		odomParameters.erase(Parameters::kRtabmapPublishRAMUsage()); // as odometry is in the same process than rtabmap, don't get RAM usage in odometry.
		Odometry * odom = Odometry::create(odomParameters);

        std::ifstream imu_file;

        if (useImu)
        {
            // open the IMU file
            std::string line;
            imu_file.open(pathImuData.c_str());
            if (!imu_file.good()) {
                UERROR("no imu file found at %s",pathImuData.c_str());
                return -1;
            }
            int number_of_lines = 0;
            while (std::getline(imu_file, line))
                ++number_of_lines;
            printf("No. IMU measurements: %d\n", number_of_lines-1);
            if (number_of_lines - 1 <= 0) {
                UERROR("no imu messages present in %s", pathImuData.c_str());
                return -1;
            }
            // set reading position to second line
            imu_file.clear();
            imu_file.seekg(0, std::ios::beg);
            std::getline(imu_file, line);

            cameraThread.enableIMUFiltering(imuFilter, parameters);
        }

		Rtabmap rtabmap;
		rtabmap.init(parameters, databasePath);

		UTimer totalTime;
		UTimer timer;
		CameraInfo cameraInfo;
		UDEBUG("");
		SensorData data = cameraThread.camera()->takeImage(&cameraInfo);
		UDEBUG("");
		int iteration = 0;
		double start = data.stamp();

#ifdef BUILD_WITH_3D_MAPPING
        printf("Starting 3D mapping\n");
        QApplication app(argc, argv);
        MapBuilder mapBuilder;
        mapBuilder.show();
        QApplication::processEvents();
#endif
		/////////////////////////////
		// Processing dataset begin
		/////////////////////////////
		cv::Mat covariance;
		int odomKeyFrames = 0;
		while(data.isValid() && g_forever)
		{
			UDEBUG("");

            if (useImu)
            {
                // get all IMU measurements till then
                double t_imu = start;
                do {
                    std::string line;
                    if (!std::getline(imu_file, line)) {
                        std::cout << std::endl << "Finished parsing IMU." << std::endl << std::flush;
                        break;
                    }

                    std::stringstream stream(line);
                    std::string s;
                    std::getline(stream, s, ',');
                    std::string nanoseconds = s.substr(s.size() - 9, 9);
                    std::string seconds = s.substr(0, s.size() - 9);

                    cv::Vec3d gyr;
                    for (int j = 0; j < 3; ++j) {
                        std::getline(stream, s, ',');
                        gyr[j] = uStr2Double(s);
                    }

                    cv::Vec3d acc;
                    for (int j = 0; j < 3; ++j) {
                        std::getline(stream, s, ',');
                        acc[j] = uStr2Double(s);
                    }

                    t_imu = double(uStr2Int(seconds)) + double(uStr2Int(nanoseconds))*1e-9;

                    if (t_imu - start + 1 > 0) {

                        SensorData dataImu(IMU(gyr, cv::Mat(3,3,CV_64FC1), acc, cv::Mat(3,3,CV_64FC1), baseToImu), 0, t_imu);
                        cameraThread.postUpdate(&dataImu);
                        odom->process(dataImu);
                    }

                } while (t_imu <= data.stamp());
            }

			cameraThread.postUpdate(&data, &cameraInfo);
			cameraInfo.timeTotal = timer.ticks();

			OdometryInfo odomInfo;
			UDEBUG("");
			Transform pose = odom->process(data, &odomInfo);
			UDEBUG("");

			if(odomInfo.keyFrameAdded)
			{
				++odomKeyFrames;
			}

			if(odomStrategy == Odometry::kTypeFovis)
			{
				//special case for FOVIS, set covariance 1 if 9999 is detected
				if(!odomInfo.reg.covariance.empty() && odomInfo.reg.covariance.at<double>(0,0) >= 9999)
				{
					odomInfo.reg.covariance = cv::Mat::eye(6,6,CV_64FC1);
				}
			}

			bool processData = true;
			if(iteration % mapUpdate != 0)
			{
				// set negative id so rtabmap will detect it as an intermediate node
				data.setId(-1);
				data.setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat());// remove features
				processData = intermediateNodes;
			}
			if(covariance.empty() || odomInfo.reg.covariance.at<double>(0,0) > covariance.at<double>(0,0))
			{
				covariance = odomInfo.reg.covariance;
			}

			timer.restart();
			if(processData)
			{
				std::map<std::string, float> externalStats;
				// save camera statistics to database
				externalStats.insert(std::make_pair("Camera/BilateralFiltering/ms", cameraInfo.timeBilateralFiltering*1000.0f));
				externalStats.insert(std::make_pair("Camera/Capture/ms", cameraInfo.timeCapture*1000.0f));
				externalStats.insert(std::make_pair("Camera/Disparity/ms", cameraInfo.timeDisparity*1000.0f));
				externalStats.insert(std::make_pair("Camera/ImageDecimation/ms", cameraInfo.timeImageDecimation*1000.0f));
				externalStats.insert(std::make_pair("Camera/Mirroring/ms", cameraInfo.timeMirroring*1000.0f));
				externalStats.insert(std::make_pair("Camera/ExposureCompensation/ms", cameraInfo.timeStereoExposureCompensation*1000.0f));
				externalStats.insert(std::make_pair("Camera/ScanFromDepth/ms", cameraInfo.timeScanFromDepth*1000.0f));
				externalStats.insert(std::make_pair("Camera/TotalTime/ms", cameraInfo.timeTotal*1000.0f));
				externalStats.insert(std::make_pair("Camera/UndistortDepth/ms", cameraInfo.timeUndistortDepth*1000.0f));
				// save odometry statistics to database
				externalStats.insert(std::make_pair("Odometry/LocalBundle/ms", odomInfo.localBundleTime*1000.0f));
				externalStats.insert(std::make_pair("Odometry/LocalBundleConstraints/", odomInfo.localBundleConstraints));
				externalStats.insert(std::make_pair("Odometry/LocalBundleOutliers/", odomInfo.localBundleOutliers));
				externalStats.insert(std::make_pair("Odometry/TotalTime/ms", odomInfo.timeEstimation*1000.0f));
				externalStats.insert(std::make_pair("Odometry/Registration/ms", odomInfo.reg.totalTime*1000.0f));
				externalStats.insert(std::make_pair("Odometry/Inliers/", odomInfo.reg.inliers));
				externalStats.insert(std::make_pair("Odometry/Features/", odomInfo.features));
				externalStats.insert(std::make_pair("Odometry/DistanceTravelled/m", odomInfo.distanceTravelled));
				externalStats.insert(std::make_pair("Odometry/KeyFrameAdded/", odomInfo.keyFrameAdded));
				externalStats.insert(std::make_pair("Odometry/LocalKeyFrames/", odomInfo.localKeyFrames));
				externalStats.insert(std::make_pair("Odometry/LocalMapSize/", odomInfo.localMapSize));
				externalStats.insert(std::make_pair("Odometry/LocalScanMapSize/", odomInfo.localScanMapSize));

				OdometryEvent e(SensorData(), Transform(), odomInfo);
				if (rtabmap.process(data, pose, covariance, e.velocity(), externalStats)) {
#ifdef BUILD_WITH_3D_MAPPING
                    // Map processing
                    mapBuilder.processStatistics(rtabmap.getStatistics());
#endif

                    if(rtabmap.getLoopClosureId() > 0)
                    {
                        printf("Loop closure detected!\n");
                    }
                }

				covariance = cv::Mat();

			}

			++iteration;
			if(!quiet || iteration == totalImages)
			{
				double slamTime = timer.ticks();

				float rmse = -1;
				if(rtabmap.getStatistics().data().find(Statistics::kGtTranslational_rmse()) != rtabmap.getStatistics().data().end())
				{
					rmse = rtabmap.getStatistics().data().at(Statistics::kGtTranslational_rmse());
				}

				if(data.keypoints().size() == 0 && data.laserScanRaw().size())
				{
					if(rmse >= 0.0f)
					{
						printf("Iteration %d/%d: camera=%dms, odom(quality=%f, kfs=%d)=%dms, slam=%dms, rmse=%fm",
								iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.icpInliersRatio, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f), rmse);
					}
					else
					{
						printf("Iteration %d/%d: camera=%dms, odom(quality=%f, kfs=%d)=%dms, slam=%dms",
								iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.icpInliersRatio, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f));
					}
				}
				else
				{
					if(rmse >= 0.0f)
					{
						printf("Iteration %d/%d: camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms, rmse=%fm",
								iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f), rmse);
					}
					else
					{
						printf("Iteration %d/%d: camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms",
								iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f));
					}
				}

				if(processData && rtabmap.getLoopClosureId()>0)
				{
					printf(" *");
				}
				printf("\n");
#ifdef BUILD_WITH_3D_MAPPING
                mapBuilder.processOdometry(data, pose, odomInfo);
#endif
			}
			else if(iteration % (totalImages/10) == 0)
			{
				printf(".");
				fflush(stdout);
			}

#ifdef BUILD_WITH_3D_MAPPING
            // Draw map
            QApplication::processEvents();

            while(mapBuilder.isPaused() && mapBuilder.isVisible())
            {
                uSleep(100);
                QApplication::processEvents();
            }
#endif

			cameraInfo = CameraInfo();
			timer.restart();
			data = cameraThread.camera()->takeImage(&cameraInfo);
		}
		delete odom;

#ifdef BUILD_WITH_3D_MAPPING
		if(mapBuilder.isVisible())
		{
			printf("Processed all frames\n");
			app.exec();
		}
#endif

		printf("Total time=%fs\n", totalTime.ticks());
		/////////////////////////////
		// Processing dataset end
		/////////////////////////////

		// Save trajectory
		printf("Saving trajectory ...\n");
		std::map<int, Transform> poses;
		std::map<int, Transform> vo_poses;
		std::multimap<int, Link> links;
		std::map<int, Signature> signatures;
		std::map<int, double> stamps;
		rtabmap.getGraph(vo_poses, links, false, true);
		links.clear();
		rtabmap.getGraph(poses, links, true, true, &signatures);
		for(std::map<int, Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			stamps.insert(std::make_pair(iter->first, iter->second.getStamp()));
		}
		std::string pathTrajectory = output + outputName + "-trajectory.txt";
		if(poses.size() && graph::exportPoses(pathTrajectory, 10, poses, links, stamps))
		{
			printf("Saving %s... done!\n", pathTrajectory.c_str());
		}
		else
		{
			printf("Saving %s... failed!\n", pathTrajectory.c_str());
		}
	}
	else
	{
		UERROR("Camera init failed!");
	}

	printf("Saving rtabmap database (with all statistics) to \"%s\"\n", (output + outputName + ".db").c_str());
	printf("Do:\n"
			" $ rtabmap-databaseViewer %s\n\n", (output + outputName + ".db").c_str());

	return 0;
}
