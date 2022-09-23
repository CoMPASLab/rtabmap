# rtabmap

TBD

## Installation

    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

The rtabmap-mbari executable is created in the build/bin folder.
You can also use the premade one in the MBARI folder.

## Docker Image Build

### Prerequisites

If you will run the 3D mapping feature youn will need an extra step previously to build the image. All this step are a summary from the [NVIDIA support page](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

```
# Setting up NVIDIA Container Toolkit

distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
      && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
      && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install nvidia-docker2 package
sudo apt-get update
sudo apt-get install nvidia-docker2

# Restart the docker daemon to complete the installation after updating the package listing
sudo systemctl restart docker
```

At this point, a working setup can be tested by running a base CUDA container.

```
sudo docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi
```

The output should be:

```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 450.51.06    Driver Version: 450.51.06    CUDA Version: 11.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  Tesla T4            On   | 00000000:00:1E.0 Off |                    0 |
| N/A   34C    P8     9W /  70W |      0MiB / 15109MiB |      0%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+

+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|  No running processes found                                                 |
+-----------------------------------------------------------------------------+
```

### Build

To build the validation we will be using a docker container. To do this, you can use the included `build-docker-image.sh` script, which is simply a wrapper for the `docker build`. The image is build by default with 16 threads, but if you plan to use a different amount of threads, add the number of threads.

```
bash build-docker-image.sh [number_threads]
```

If you can build the image by yourself, =you have to use the  `Dockerfile` that is provided in the repository at the directory `/MBARI`, running the following command:

```
docker build --rm -t rtabmap -f ./MBARI/Dockerfile . --build-arg NUM_THREADS=[number_threads]
```

This command will build the image using the dockerfile: `./MBARI/Dockerfile` with the tag `rtabmap`. The option `--rm` will remove the intermediate images after the image is build succesfully, and path `.` indicate to mount the repository directories in the docker daemon. Finally, the `--build-arg NUM_THREADS` states the number of cores to build the image.

## Run docker Container

Once the image is built, you can run it with the included `run-docker-container.sh` script, which is simply a wrapper for the `docker run` command with some setup to enable X11 forwarding. The image has a default command of `/bin/bash`, so running this script will drop you into a shell.


If you plan to download the target data on the host, add a `-v` flag to the script to mount your dataset.

```
bash run-docker-container container.sh [-v <host_path>:<container_path>]
```

## Usage

```
./rtabmap-mbari DATASET_ROOT_PATH LEFT_IMAGE_FOLDER RIGHT_IMAGE_FOLDER LEFT_CALIB_YAML RIGHT_CALIB_YAML  
```
or 
```
./rtabmap-mbari DATASET_ROOT_PATH LEFT_IMAGE_FOLDER RIGHT_IMAGE_FOLDER LEFT_CALIB_YAML RIGHT_CALIB_YAML IMU_CSV_FILE IMU_CALIB_YAML  
```

See the scripts in MBARI folder for examples.

## Other

[Original README](./README-original.md)
