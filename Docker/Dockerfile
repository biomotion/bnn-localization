FROM osrf/ros:kinetic-desktop-full

RUN apt-get update && apt-get install -y \
    vim \
    ros-${ROS_DISTRO}-navigation \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf-conversions \
    && rm -rf /var/lib/apt/lists/* && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# when you use melodic, uncomment this to avoid confliction
# RUN rm /usr/include/flann/ext/lz4.h /usr/include/flann/ext/lz4hc.h && \
#     ln -s /usr/include/lz4.h /usr/include/flann/ext/lz4.h && \
#     ln -s /usr/include/lz4hc.h /usr/include/flann/ext/lz4hc.h

# ENV PCL_VERSION 1.8.0
# WORKDIR /tmp
# RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-${PCL_VERSION}.tar.gz&& \
#     tar -zxvf pcl-${PCL_VERSION}.tar.gz && rm pcl-${PCL_VERSION}.tar.gz && \
#     mv pcl-pcl-${PCL_VERSION} pcl && \
#     mkdir -p pcl/build && cd pcl/build && \
#     cmake -DCMAKE_BUILD_TYPE=Release .. && make && \
#     make install

# RUN apt-get update && \
#     apt-get install -y \
#     libpcl-all