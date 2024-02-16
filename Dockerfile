#FROM nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_33836e394da2d095a59afd2d151038f8
#ARG CUDA_VERSION=12.2.0
#ARG BASE_DIST=ubuntu22.04
#FROM nvidia/cuda:${CUDA_VERSION}-devel-${BASE_DIST} as ros_builder

FROM ubuntu:22.04

## necessary tooling for px4
RUN apt-get update 
RUN apt-get install -y python3 python3-dev python3-pip
RUN python3 -m pip install --user -U empy pyros-genmsg setuptools

RUN apt-get install -y git vim

## install mavlink-router
WORKDIR /root
RUN git clone https://github.com/intel/mavlink-router.git
WORKDIR /root/mavlink-router
RUN git submodule update --init --recursive
RUN apt-get install -y --no-install-recommends git ninja-build pkg-config gcc g++ systemd python3-pip
RUN pip3 install meson
RUN meson setup build . --buildtype=release
RUN ninja -C build
RUN ninja -C build install

## install python empy 
RUN pip3 install empy==3.3.4

## install microXRCE agent
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN apt install -y cmake
RUN cmake -DCMAKE_BUILD_TYPE=Release .. 
RUN make -j$(($(nproc) - 1))
RUN make install -j$(($(nproc) - 1))
RUN ldconfig /usr/local/lib/

RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update

ARG DEBIAN_FRONTEND=noninteractive

RUN apt install -y ros-humble-ros-base ros-dev-tools
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN apt-get install -y libboost-all-dev ros-humble-diagnostic-updater
RUN apt install -y tmux iputils-ping

WORKDIR /home/colcon_ws
RUN echo "source /home/colcon_ws/install/setup.bash" >> ~/.bashrc
