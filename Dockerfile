FROM ros:foxy as dev

ENV DEBIAN_FRONTEND=noninteractive

# For Chinese mainland user.
# RUN  sed -i "s/ports.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list
# RUN  sed -i "s/archive.ubuntu.com/mirrors.ustc.edu.cn/g" /etc/apt/sources.list

# Builder dependencies installation
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \    
    curl \	
    libusb-1.0-0 \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    swig \
    software-properties-common \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://ghproxy.com/https://github.com/YDLIDAR/YDLidar-SDK.git &&\
    mkdir -p YDLidar-SDK/build && \
    cd YDLidar-SDK/build &&\
    cmake ..&&\
    make &&\
    make install &&\
    cd .. &&\
    pip install . &&\
    cd .. && rm -r YDLidar-SDK 

RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

WORKDIR /root/agx_ws

# 更新rosdep和apt缓存
RUN apt-get update && rosdep update --rosdistro foxy

# 下载limo厂家代码
RUN mkdir src &&\
    cd src &&\
    git clone https://github.com/HaojieZhang6848/limo_ros2.git -b foxy &&\
    cd ..

# 安装navigation2
RUN apt-get install -y ros-foxy-navigation2 ros-foxy-nav2-bringup

# 下载astra-camera https://github.com/orbbec/ros2_astra_camera/tree/master
RUN sudo apt install -y nlohmann-json3-dev libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev &&\
    cd /root && \
    git clone https://github.com/libuvc/libuvc.git && \
    cd libuvc && \
    mkdir build && cd build && \
    cmake .. && make -j4 && make install && \
    ldconfig && \
    rm -rf /root/libuvc && \
    cd /root/agx_ws/src && \
    git clone https://github.com/orbbec/ros2_astra_camera.git

# 下载async-web-server-cpp 和 web-video-server的源码
RUN cd /root/agx_ws/src && \
    git clone https://github.com/fkie/async_web_server_cpp.git -b ros2-releases && \
    git clone https://github.com/RobotWebTools/web_video_server.git -b ros2

# rosdep安装依赖
RUN . /opt/ros/foxy/setup.sh &&\
    rosdep install --from-paths src --ignore-src -r -y

# 编译一切
RUN . /opt/ros/foxy/setup.sh &&\
    cd /root/agx_ws &&\
    colcon build --symlink-install

# 编辑.bashrc，打开bash的时候输出环境变量
RUN echo "source /root/agx_ws/install/setup.bash" >> /root/.bashrc &&\
    echo "echo ROS_DOMAIN_ID=$ROS_DOMAIN_ID" >> /root/.bashrc

# 默认的cmd是永远sleep
CMD ["sleep", "infinity"]