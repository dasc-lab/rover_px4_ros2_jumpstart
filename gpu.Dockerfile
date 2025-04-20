FROM nvcr.io/nvidia/jax:23.10-py3
RUN apt-get update
RUN apt-get install -y gedit
RUN apt-get install -y '^libxcb.*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev
RUN pip3 install PyQt5
RUN pip3 install matplotlib

## NOTE: gpjax version dependent on JAX version. Should change depending on which jax nvidia image has been pulled up
# with nvcr.io/nvidia/jax:23.10-py3
#RUN pip3 install gpjax==0.8.0 


ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get install -y libqt5gui5
RUN apt-get install -y texlive-fonts-recommended texlive-fonts-extra texlive-latex-extra dvipng cm-super
RUN apt install -y vim

# ROS installation
RUN apt install -y software-properties-common
RUN add-apt-repository universe
RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt install -y ros-humble-ros-base ros-dev-tools
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN apt-get install -y libboost-all-dev ros-humble-diagnostic-updater
RUN apt install -y tmux iputils-ping



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

## install microXRCE agent
WORKDIR /root
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
WORKDIR /root/Micro-XRCE-DDS-Agent/build
RUN apt install -y cmake
RUN cmake -DCMAKE_BUILD_TYPE=Release .. 
RUN make -j$(($(nproc) - 1))
RUN make install -j$(($(nproc) - 1))
RUN ldconfig /usr/local/lib/


WORKDIR /home/colcon_ws
RUN echo "source /home/colcon_ws/install/setup.bash" >> ~/.bashrc

## install python libraries
RUN pip3 install rosbags
RUN pip3 install matplotlib
RUN pip3 install pymavlink


# RUN pip3 install setuptools==58.0.4
RUN pip3 install empy==3.3.4
RUN echo 'alias px4="ros2 launch all_launch px4.launch.py"' >> ~/.bashrc

RUN apt-get install ros-humble-desktop
RUN source /opt/ros/humble/setup.bash

