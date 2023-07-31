FROM ros:noetic-perception

# Install build dependencies
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y gdb wget git ros-noetic-imu-tools ros-noetic-imu-pipeline software-properties-common python3-catkin-tools libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libfmt-dev && \
    rm -rf /var/lib/apt/lists/

RUN wget ceres-solver.org/ceres-solver-2.1.0.tar.gz
RUN tar zxf ceres-solver-2.1.0.tar.gz
RUN mkdir ceres-bin
WORKDIR /ceres-bin
RUN cmake ../ceres-solver-2.1.0
RUN make -j8
RUN make install

WORKDIR /
RUN wget https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.tar.gz
RUN tar xzf 1.22.10.tar.gz
WORKDIR /Sophus-1.22.10/
RUN mkdir build
WORKDIR /Sophus-1.22.10/build
RUN cmake .. -DUSE_BASIC_LOGGING=ON
RUN make -j4
RUN make install 

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

USER root
WORKDIR /home/root
RUN source /ros_entrypoint.sh
RUN echo "source /ros_entrypoint.sh" >> .bashrc