FROM ros:noetic-perception
ENV DEBIAN_FRONTEND=noninteractive

ENV DIRPATH /root
WORKDIR ${DIRPATH}

# Install build dependencies
RUN apt-get update && \
    apt-get install -y git wget software-properties-common ros-noetic-rtabmap-ros libpdal-dev && \
    apt-get remove -y ros-noetic-rtabmap && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=1

# Clone GTSAM @ commit: 0909c46339e0c604b0f26c8a67c7144c1358db4c
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 0909c46339e0c604b0f26c8a67c7144c1358db4c

# Build and Install GTSAM
RUN cd gtsam && \
    mkdir build && \
    cd build && \
    cmake \
        -D CMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX \
        -D CMAKE_BUILD_TYPE=Release \
        -D GTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -D GTSAM_USE_SYSTEM_EIGEN=ON \
        -D GTSAM_BUILD_TESTS=OFF \
        -D GTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        .. && \
    make -j${NUM_THREADS} install && \
    cd ${DIRPATH} && rm -rf gtsam

# Git clone libnabo @ commit: 3cab7eed92bd5d4aed997347b8c8a2692a83a532
RUN git clone https://github.com/ethz-asl/libnabo.git && \
    cd libnabo && \
    git checkout 3cab7eed92bd5d4aed997347b8c8a2692a83a532

# Build and install libnabo
RUN cd libnabo && \
    mkdir build && \
    cd build && \
    cmake \
    -D CMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX \
    -D CMAKE_BUILD_TYPE=Release \
    .. && \
    make -j${NUM_THREADS} install && \
    cd ${DIRPATH} && rm -rf libnabo

# Git clone libpointmatcher @ commit: 76f99fce0fe69e6384102a0343fdf8d262626e1f
RUN git clone https://github.com/ethz-asl/libpointmatcher.git && \
    cd libpointmatcher && \
    git checkout 76f99fce0fe69e6384102a0343fdf8d262626e1f

# Build and install libpointmatcher
RUN cd libpointmatcher && \
    mkdir build && \
    cd build && \
    cmake \
    -D CMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX \
    -D CMAKE_BUILD_TYPE=Release \
    .. && \
    make -j${NUM_THREADS} install && \
    cd ${DIRPATH} && rm -rf libpointmatcher

# Git clone OpenGV @ commit: 91f4b19c73450833a40e463ad3648aae80b3a7f3
RUN git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && \
    git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3 && \
    wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch && \
    git apply opengv_disable_march_native.patch
    
# Build and install OpenGV
RUN cd opengv && \
    mkdir build && \
    cd build && \
    cmake \
    -D CMAKE_INSTALL_PREFIX=$CMAKE_INSTALL_PREFIX \
    -D CMAKE_BUILD_TYPE=Release \
    .. && \
    make -j${NUM_THREADS} install && \
    cd ${DIRPATH} && rm -rf opengv

# Copy current source code
COPY ./rtabmap ${DIRPATH}/rtabmap

# Build RTAB-Map project
RUN cd rtabmap/build && \
    cmake \
        -D WITH_OPENGV=ON \
        -D BUILD_TOOLS=OFF \
        -D BUILD_EXAMPLES=OFF \
        .. && \
    make -j${NUM_THREADS} && \
    ldconfig

# Set default command to bash
CMD ["/bin/bash"]