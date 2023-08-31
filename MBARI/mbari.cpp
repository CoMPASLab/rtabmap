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
#include "rtabmap/core/Depth.h"
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
            "  path               Root folder of the sequence (e.g., \"~/mbari-datasets/SE/simulation_0038/\")\n"
            "  left_image_dir     Left image directory (e.g., \"color/PROSILICA_L\")\n"
            "  right_image_dir    Right image directory (e.g., \"color/PROSILICA_R\")\n"
            "  calib_file_dir     Path to calibration file folder (e.g., \"~/calibrations/PROSILICA_2020/\")\n"
            "  --odom_data_file   (Optional) Data file to be used as odometry guesses, must be in forward-left-up frame (e.g., \"odom.csv\")\n"
            "  --depth_data_file  (Optional) Data file with absolute depth info (values should be negative as per FLU) (e.g., \"depth_data.csv\")\n"
            "  --imu_data_file    (Optional) IMU data file (e.g., \"imu.csv\")\n"
            "  --imu_calib_file   (Optional) IMU calib YAML (e.g., \"imu_calib.yaml\")\n"
            "  --output           Output directory. By default, results are saved in \"path\".\n"
            "  --output_name      Output database name (default \"rtabmap\").\n"
            "  --calib_prefix     Calib file prefix (default \"rtabmap\").\n"
            "  --quiet            Don't show log messages and iteration updates.\n"
            "  --exposure_comp    Do exposure compensation between left and right images.\n"
            "  --disp             Generate full disparity.\n"
            "  --raw              Use raw images (not rectified, this only works with okvis, msckf or vins odometry).\n"
            "  --save_db          Save the mapping database created.\n"
            "  --camera_transform_offset Provide camera transform offset quaternion for debugging purposes.\n"
            "  --sensor_time_offset Provide sensor time offset for debugging purposes.\n"
            "%s\n", rtabmap::Parameters::showUsage());
    exit(1);
}


int main(int argc, char * argv[])
{
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
    std::string calibFileDirPath;
    std::string imuDataFileName = "";
    std::string sensorCalibFileName = "";
    std::string filterOdometryFileName = "";
    std::string arbitraryPoseConstraintFileName = "";
    std::string depthDataFileName = "";
    bool disp = false;
    bool raw = false;
    bool exposureCompensation = false;
    bool quiet = false;
    int imuFilter = 1;
    bool saveDB = false;

    bool useImu = false;
    bool useFilterOdometry = false;
    bool useAbsoluteDepths = false;
    bool useArbitraryPoseConstraints = false;

    Transform cameraTransformOffset;
    double sensorTimeOffset = 0.0;

#ifdef BUILD_WITH_3D_MAPPING
    int pointCloudDecimation = 4;
#endif

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
            else if(std::strcmp(argv[i], "--save_db") == 0)
            {
                saveDB = true;
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
            else if(std::strcmp(argv[i], "--odom_data_file") == 0)
            {
                filterOdometryFileName = argv[++i];
            }
            else if(std::strcmp(argv[i], "--arbitrary_pose_constraint_file") == 0)
            {
                arbitraryPoseConstraintFileName = argv[++i];
            }
            else if(std::strcmp(argv[i], "--depth_data_file") == 0)
            {
                depthDataFileName = argv[++i];
            }
            else if(std::strcmp(argv[i], "--imu_data_file") == 0)
            {
                imuDataFileName = argv[++i];
            }
            else if(std::strcmp(argv[i], "--sensor_calib_file") == 0)
            {
                sensorCalibFileName = argv[++i];
            }
            else if(std::strcmp(argv[i], "--info_prints") == 0)
            {
                ULogger::setLevel(ULogger::kInfo);
            }
            else if(std::strcmp(argv[i], "--debug_prints") == 0)
            {
                ULogger::setLevel(ULogger::kDebug);
            }
            else if(std::strcmp(argv[i], "--camera_transform_offset") == 0) {
                float x = atof(argv[i + 1]);
                float y = atof(argv[i + 2]);
                float z = atof(argv[i + 3]);
                float qx = atof(argv[i + 4]);
                float qy = atof(argv[i + 5]);
                float qz = atof(argv[i + 6]);
                float qw = atof(argv[i + 7]);
                cameraTransformOffset = { x, y, z, qx, qy, qz, qw };
            }
            else if(std::strcmp(argv[i], "--sensor_time_offset") == 0) {
                sensorTimeOffset = atof(argv[++i]);
            }
#ifdef BUILD_WITH_3D_MAPPING
            else if(std::strcmp(argv[i], "--point_cloud_decimation") == 0) {
                pointCloudDecimation = atof(argv[++i]);
            }
#endif
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
        calibFileDirPath = argv[4];
        if (!filterOdometryFileName.empty())
        {
            useFilterOdometry = true;
            printf("Using filter odometry data for pose guesses\n");
        }
        else if (!imuDataFileName.empty())
        {
            useImu = true;
            printf("Using IMU data for pose guesses\n");
        }
        else
        {
            printf("Not using IMU nor odometry as necessary params were not provided\n");
        }
        if (!depthDataFileName.empty())
        {
            useAbsoluteDepths = true;
            printf("Using depth data for absolute constraints\n");
        }
        if (!arbitraryPoseConstraintFileName.empty())
        {
            useArbitraryPoseConstraints = true;
            printf("Using arbitrary pose constraints\n");
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
    std::string pathFilterOdometryData = path + filterOdometryFileName;
    std::string pathImuData = path + imuDataFileName;
    std::string pathSensorCalib = calibFileDirPath + sensorCalibFileName;
    std::string pathDepthData = path + depthDataFileName;
    std::string pathArbitraryPoseConstraintData = path + arbitraryPoseConstraintFileName;

    printf("Paths:\n"
            "   Sequence number:  %s\n"
            "   Sequence path:    %s\n"
            "   Output:           %s\n"
            "   Output name:      %s\n"
            "   Calib directory:  %s\n"
            "   Calib prefix:     %s\n"
            "   left images:      %s\n"
            "   right images:     %s\n",
            seq.c_str(),
            path.c_str(),
            output.c_str(),
            outputName.c_str(),
            calibFileDirPath.c_str(),
            calibPrefix.c_str(),
            pathLeftImages.c_str(),
            pathRightImages.c_str());
    if(useImu)
    {
        printf("   IMU data:         %s\n", pathImuData.c_str());
        printf("   IMU filter:       %d\n", imuFilter);
    }
    if(useFilterOdometry)
    {
        printf("   Odometry data:    %s\n", pathFilterOdometryData.c_str());
    }
    if(useAbsoluteDepths)
    {
        printf("   Depth data:       %s\n", pathDepthData.c_str());
    }
    if(!sensorCalibFileName.empty())
    {
        printf("   Sensor calib:     %s\n", pathSensorCalib.c_str());
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

    YAML::Node left_calib = YAML::LoadFile(calibFileDirPath + calibPrefix + "_calib_left.yaml");
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

    if (!cameraTransformOffset.isNull())
    {
        std::cout << "cameraTransformOffset:\n" << cameraTransformOffset << std::endl;
        baseToCam0 = baseToCam0 * cameraTransformOffset;
    }

    int odomStrategy = Parameters::defaultOdomStrategy();
    Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);

    if(quiet)
    {
        ULogger::setLevel(ULogger::kError);
    }

    Transform baseToImu = {cv::Mat::eye(3,4,CV_64FC1)};
    Transform baseToDepth = {cv::Mat::eye(3,4,CV_64FC1)};

    if (!sensorCalibFileName.empty()) {
        // Load sensor calibration file
        YAML::Node config = YAML::LoadFile(pathSensorCalib);
        if(config.IsNull())
        {
            UERROR("Cannot open sensor calibration file \"%s\"", pathSensorCalib.c_str());
            return -1;
        }
        if (config["T_IMU"]) 
        {
            YAML::Node T_BS = config["T_IMU"];
            YAML::Node data = T_BS["data"];
            UASSERT(data.size() == 16);

            baseToImu = {data[0].as<float>(), data[1].as<float>(), data[2].as<float>(), data[3].as<float>(),
                         data[4].as<float>(), data[5].as<float>(), data[6].as<float>(), data[7].as<float>(),
                         data[8].as<float>(), data[9].as<float>(), data[10].as<float>(), data[11].as<float>()};
            std::cout << "baseToImu:\n" << baseToImu << std::endl;
        }
        if (config["T_DEPTH"]) 
        {
            YAML::Node T_BS = config["T_DEPTH"];
            YAML::Node data = T_BS["data"];
            UASSERT(data.size() == 16);

            baseToDepth = {data[0].as<float>(), data[1].as<float>(), data[2].as<float>(), data[3].as<float>(),
                           data[4].as<float>(), data[5].as<float>(), data[6].as<float>(), data[7].as<float>(),
                           data[8].as<float>(), data[9].as<float>(), data[10].as<float>(), data[11].as<float>()};
            std::cout << "baseToDepth:\n" << baseToDepth << std::endl;
        }
    }

    // We use CameraThread only to use postUpdate() method

    // Note: The optical rotation is applied within
    CameraThread cameraThread(new
        CameraStereoImages(
                pathLeftImages,
                pathRightImages,
                !raw,
                0.0f,
                baseToCam0), parameters);
    std::cout << "baseToCam0:\n" << baseToCam0 << std::endl;
    ((CameraStereoImages*)cameraThread.camera())->setTimestamps(false, path + "image_timestamps.txt", false);
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

    std::string databasePath = saveDB ? output + outputName + ".db" : "";
    UFile::erase(databasePath);
    if(cameraThread.camera()->init(calibFileDirPath, calibPrefix + "_calib"))
    {
        int totalImages = (int)((CameraStereoImages*)cameraThread.camera())->filenames().size();

        printf("Processing %d images...\n", totalImages);

        ParametersMap odomParameters = parameters;
        odomParameters.erase(Parameters::kRtabmapPublishRAMUsage()); // as odometry is in the same process than rtabmap, don't get RAM usage in odometry.
        Odometry * odom = Odometry::create(odomParameters);

        std::ifstream imuDataFile;

        if (useImu)
        {
            // open the IMU file
            std::string line;
            imuDataFile.open(pathImuData.c_str());
            if (!imuDataFile.good()) {
                UERROR("no imu file found at %s",pathImuData.c_str());
                return -1;
            }
            int number_of_lines = 0;
            while (std::getline(imuDataFile, line))
                ++number_of_lines;
            printf("No. IMU measurements: %d\n", number_of_lines-1);
            if (number_of_lines - 1 <= 0) {
                UERROR("no imu messages present in %s", pathImuData.c_str());
                return -1;
            }
            imuDataFile.clear();
            imuDataFile.seekg(0, std::ios::beg);

            cameraThread.enableIMUFiltering(imuFilter, parameters);
        }

        std::ifstream filterOdometryFile;

        if (useFilterOdometry)
        {
            std::string line;
            filterOdometryFile.open(pathFilterOdometryData.c_str());
            if (!filterOdometryFile.good()) {
                UERROR("no odom file found at %s",pathFilterOdometryData.c_str());
                return -1;
            }
            int number_of_lines = 0;
            while (std::getline(filterOdometryFile, line))
                ++number_of_lines;
            printf("No. odom measurements: %d\n", number_of_lines-1);
            if (number_of_lines - 1 <= 0) {
                UERROR("no odom messages present in %s", pathFilterOdometryData.c_str());
                return -1;
            }
            filterOdometryFile.clear();
            filterOdometryFile.seekg(0, std::ios::beg);
        }

        std::ifstream depthDataFile;

        if (useAbsoluteDepths)
        {
            std::string line;
            depthDataFile.open(pathDepthData.c_str());
            if (!depthDataFile.good()) {
                UERROR("no depth file found at %s",pathDepthData.c_str());
                return -1;
            }
            int number_of_lines = 0;
            while (std::getline(depthDataFile, line))
                ++number_of_lines;
            printf("No. depth measurements: %d\n", number_of_lines-1);
            if (number_of_lines - 1 <= 0) {
                UERROR("no depth messages present in %s", pathDepthData.c_str());
                return -1;
            }
            depthDataFile.clear();
            depthDataFile.seekg(0, std::ios::beg);
        }

        std::ifstream arbitraryPoseConstraintFile;

        if (useArbitraryPoseConstraints)
        {
            std::string line;
            arbitraryPoseConstraintFile.open(pathArbitraryPoseConstraintData.c_str());
            if (!arbitraryPoseConstraintFile.good()) {
                UERROR("no pose constraint file found at %s", pathArbitraryPoseConstraintData.c_str());
                return -1;
            }
            int number_of_lines = 0;
            while (std::getline(arbitraryPoseConstraintFile, line))
                ++number_of_lines;
            printf("No. pose constraints: %d\n", number_of_lines-1);
            if (number_of_lines - 1 <= 0) {
                UERROR("no pose constraints present in %s", pathArbitraryPoseConstraintData.c_str());
                return -1;
            }
            arbitraryPoseConstraintFile.clear();
            arbitraryPoseConstraintFile.seekg(0, std::ios::beg);
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
        MapBuilder mapBuilder(pointCloudDecimation);
        mapBuilder.show();
        QApplication::processEvents();
#endif
        /////////////////////////////
        // Processing dataset begin
        /////////////////////////////
        cv::Mat covariance;
        int odomKeyFrames = 0;

        Transform lastFilterOdometry = {cv::Mat::eye(3,4,CV_64FC1)};
        Transform lastPoseConstraint = {cv::Mat::eye(3,4,CV_64FC1)};
        double lastTimestamp = -1.0;
        double lastAbsoluteDepth = 0.0;

        while(data.isValid())
        {
            UDEBUG("");

            Transform newFilterOdometry = {cv::Mat::eye(3,4,CV_64FC1)};
            Transform newPoseConstraint = {cv::Mat::eye(3,4,CV_64FC1)};
            float newAbsoluteDepth = 0.f;

            if (useImu)
            {
                // get all IMU measurements till then
                double t_imu = start;
                do {
                    std::string line;
                    if (!std::getline(imuDataFile, line)) {
                        UINFO("\nFinished parsing IMU.");
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

                    t_imu = double(uStr2Int(seconds)) + double(uStr2Int(nanoseconds))*1e-9 + sensorTimeOffset;

                    if (t_imu - start + 1 > 0) {
                        SensorData dataImu(IMU(gyr, cv::Mat(3,3,CV_64FC1), acc, cv::Mat(3,3,CV_64FC1), baseToImu), 0, t_imu);
                        cameraThread.postUpdate(&dataImu);
                        odom->process(dataImu);
                    }
                } while (t_imu <= data.stamp());
            }
            if (useFilterOdometry) {
                double t_loc = lastTimestamp;
                double t_prev = lastTimestamp;
                Transform newestOdometry = lastFilterOdometry;
                Transform previousOdometry = lastFilterOdometry;
                do {
                    std::string line;
                    if (!std::getline(filterOdometryFile, line)) {
                        UINFO("\nFinished parsing localization data.\n");
                        break;
                    }
                    previousOdometry = newestOdometry;
                    t_prev = t_loc;

                    std::stringstream stream(line);
                    std::string s;
                    std::getline(stream, s, ',');
                    std::string nanoseconds = s.substr(s.size() - 9, 9);
                    std::string seconds = s.substr(0, s.size() - 9);

                    float odom[7];
                    for (int j = 0; j < 7; ++j) {
                        std::getline(stream, s, ',');
                        odom[j] = uStr2Double(s);
                    }

                    t_loc = double(uStr2Int(seconds)) + double(uStr2Int(nanoseconds))*1e-9 + sensorTimeOffset;

                    newestOdometry = { odom[0], odom[1], odom[2], odom[3], odom[4], 
                         odom[5], odom[6] };

                } while (t_loc <= data.stamp());

                // Interpolate odometry
                if (t_prev != -1.0 && t_loc - start > 1)
                {
                    float scalar = (data.stamp() - t_prev) / (t_loc - t_prev); 
                    newFilterOdometry = previousOdometry.interpolate(scalar, newestOdometry);
                }
            }
            if (useAbsoluteDepths)
            {
                double t_dep = lastTimestamp;
                double t_prev = lastTimestamp;
                float newestDepth = lastAbsoluteDepth;
                float previousDepth = lastAbsoluteDepth;
                do {
                    std::string line;
                    if (!std::getline(depthDataFile, line)) {
                        UINFO("\nFinished parsing depth data.\n");
                        printf("\nFinished parsing depth data.\n");
                        break;
                    }
                    previousDepth = newestDepth;
                    t_prev = t_dep;

                    std::stringstream stream(line);
                    std::string s;
                    std::getline(stream, s, ',');
                    std::string nanoseconds = s.substr(s.size() - 9, 9);
                    std::string seconds = s.substr(0, s.size() - 9);

                    std::getline(stream, s, ',');
                    newestDepth = uStr2Double(s);
                    t_dep = double(uStr2Int(seconds)) + double(uStr2Int(nanoseconds))*1e-9 + sensorTimeOffset;
                } while (t_dep <= data.stamp());

                // Interpolate depth
                if (t_prev != -1.0)
                {
                    float scalar = (data.stamp() - t_prev) / (t_dep - t_prev); 
                    newAbsoluteDepth = previousDepth + (newestDepth - previousDepth) * scalar;
                }
            }
            if (useArbitraryPoseConstraints) {
                double t_loc = lastTimestamp;
                double t_prev = lastTimestamp;
                Transform newestOdometry = lastPoseConstraint;
                Transform previousOdometry = lastPoseConstraint;
                do {
                    std::string line;
                    if (!std::getline(arbitraryPoseConstraintFile, line)) {
                        UINFO("\nFinished parsing pose constraint data.\n");
                        break;
                    }
                    previousOdometry = newestOdometry;
                    t_prev = t_loc;

                    std::stringstream stream(line);
                    std::string s;
                    std::getline(stream, s, ',');
                    std::string nanoseconds = s.substr(s.size() - 9, 9);
                    std::string seconds = s.substr(0, s.size() - 9);

                    float odom[7];
                    for (int j = 0; j < 7; ++j) {
                        std::getline(stream, s, ',');
                        odom[j] = uStr2Double(s);
                    }

                    t_loc = double(uStr2Int(seconds)) + double(uStr2Int(nanoseconds))*1e-9 + sensorTimeOffset;

                    newestOdometry = { odom[0], odom[1], odom[2], odom[3], odom[4], 
                         odom[5], odom[6] };

                } while (t_loc <= data.stamp());

                // Interpolate odometry
                if (t_prev != -1.0 && t_loc - start > 1)
                {
                    float scalar = (data.stamp() - t_prev) / (t_loc - t_prev); 
                    newPoseConstraint = previousOdometry.interpolate(scalar, newestOdometry);
                }
            }

            cameraThread.postUpdate(&data, &cameraInfo);
            cameraInfo.timeTotal = timer.ticks();

            OdometryInfo odomInfo;
            UDEBUG("");

            Transform pose = useFilterOdometry ? odom->process(data, lastFilterOdometry.inverse() * newFilterOdometry, &odomInfo) : odom->process(data, &odomInfo);   
            lastFilterOdometry = newFilterOdometry;
            lastTimestamp = data.stamp();
            UDEBUG("");

            if (useAbsoluteDepths) {
                lastAbsoluteDepth = newAbsoluteDepth;
                // TODO: Make covariance matrix for absolute depth to be configurable
                data.setAbsoluteDepth({newAbsoluteDepth, cv::Mat::eye(6, 6, CV_32FC1) * 1e-6, baseToDepth});
            }

            if (useArbitraryPoseConstraints && data.id() > 1) {
                Transform diff = lastPoseConstraint.inverse() * newPoseConstraint;
                auto diag = cv::Mat(cv::Mat::diag(cv::Mat{1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2}).inv());
                data.addArbitraryPoseConstraint({diff, diag});
                lastPoseConstraint = newPoseConstraint;
            }

            if(odomInfo.keyFrameAdded)
            {
                ++odomKeyFrames;
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
        std::string pathTrajectory = output + outputName + "-trajectory.csv";
        rtabmap.saveCurrentTrajectory(pathTrajectory);
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
