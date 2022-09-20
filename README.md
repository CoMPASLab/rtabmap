# rtabmap

TBD

## Installation

    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

The rtabmap-mbari executable is created in the build/bin folder.
You can also use the premade one in the MBARI folder.

## Docker Image Build

To build the validation we will be using a docker container. To build the image, you have to use the  `Dockerfile` that is provided in the repository at the directory `/MBARI`, running the following command:

```
docker build --rm -t rtabmap -f ./MBARI/Dockerfile . --build-arg NUM_THREADS=32
```

This command will build the image using the dockerfile: `./MBARI/Dockerfile` with the tag `rtabmap`. The option `--rm` will remove the intermediate images after the image is build succesfully, and path `.` indicate to mount the repository directories in the docker daemon. Finally, the `--build-arg NUM_THREADS` states the number of cores to build the image, which for `bigtuna` is set to 32 by default.

## Run docker Container


Once the image is built, you can run it with the included `run_container.sh` script, which is simply a wrapper for the `docker run` command with some setup to enable X11 forwarding. The image has a default command of `/bin/bash`, so running this script will drop you into a shell. The `--rm` flag is recommended to remove the container after it exits; exclude this flag if you want to keep it around for debugging.


If you plan to download the target data on the host, add a `-v` flag to the script to mount your dataset.

```
./run_container.sh [--rm] [-v <host_path>:<container_path>]
```

## Usage

```
./rtabmap-mbari path/to/your/euroc/format/data/folder
```

## Other

[Original README](./README-original.md)
