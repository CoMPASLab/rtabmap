#!/bin/bash

../build/bin/rtabmap-mbari ~/mbari-datasets/SE/simulation_0038/ color/PROSILICA_L/ color/PROSILICA_R/ left_calib.yaml right_calib.yaml --imu_data_file imu.csv --imu_calib_file imu_calib.yaml
