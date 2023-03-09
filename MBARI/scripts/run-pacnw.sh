#!/bin/bash

/home/tomi/colcon_ws/build/rtabmap/bin/rtabmap-mbari /home/tomi/mbari-ws/src/auto_analysis_scripts/datasets/PACNW/oi_survey_1521/ color/PROSILICA_L/ color/PROSILICA_R/ /home/tomi/mbari-ws/src/auto_analysis_scripts/calibrations/PROSILICA_2020/ --output_name pacnw1521 --depth_data_file depth_data.csv --sensor_calib_file imu_calib.yaml --odom_data_file localization_odom.csv --Kp/DetectorStrategy 7 --Vis/FeatureType 7 --Optimizer/Strategy 2
