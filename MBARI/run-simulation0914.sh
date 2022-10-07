#!/bin/bash

../build/bin/rtabmap-mbari ~/mbari-datasets/SE/simulation_0914/ color/PROSILICA_L/ color/PROSILICA_R/ --imu_data_file imu.csv --imu_calib_file imu_calib.yaml --ImuFilter/ComplementaryDoBiasEstimation false
