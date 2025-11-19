# 注意：这个文件需要放在catkin_ws中的src同级目录下！

ARG WORKSPACE_DIR=localization_ws
ARG INSTALL_DIR=ros_install

ARG USERNAME=ubuntu
ARG PASSWORD=password
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# docker镜像的构建都是从一个目前已有的镜像开始，
# 从干净的ubuntu开始构建太麻烦也不需要。
FROM ros:noetic-ros-base AS base

# 换源
# gpg和apt-key使不会直接读取终端中设置的代理，需要单独设置。所以要单独地设置代理 --keyserver-options http-proxy=$http_proxy
RUN bash -c 'arch=$(uname -m); case "${arch,,}" in \
    aarch64) \
    printf "\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ focal main restricted universe multiverse\n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ focal-updates main restricted universe multiverse\n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ focal-backports main restricted universe multiverse\n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/ focal-security main restricted universe multiverse\n" > /etc/apt/sources.list &&\
    echo "USING AARCH64" \
    ;;\
    x86_64) \
    printf "\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal main restricted universe multiverse\n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-updates main restricted universe multiverse\n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-backports main restricted universe multiverse\n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ focal-security main restricted universe multiverse\n" > /etc/apt/sources.list &&\
    echo "USING X86_64" \
    ;; esac &&\
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros1-snapshots.list &&\
    apt-key adv --keyserver "hkp://keyserver.ubuntu.com:80" --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 &&\
    cat /etc/apt/sources.list &&\
    cat /etc/apt/sources.list.d/ros1-snapshots.list'

ARG DEBIAN_FRONTEND=noninteractive
# 安装依赖包
RUN DEBIAN_FRONTEND=noninteractive apt update && \
    apt install --no-install-recommends -y \
    build-essential \
    cmake \
    git \
    vim \
    curl \
    zsh \
    libtool \
    libeigen3-dev \
    libopencv-dev \
    libgoogle-glog-dev \
    libyaml-cpp-dev \
    libzmqpp-dev \
    libboost-all-dev \
    libboost-filesystem-dev \
    gdb \
    systemd-coredump \
    libgflags-dev \
    libdw-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    openssh-server && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 安装必要的ros工具和依赖
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt update && \
    apt install --no-install-recommends -y \
    ros-noetic-pcl-ros \
    ros-noetic-cv-bridge \
    ros-noetic-laser-geometry \
    ros-noetic-rviz \
    ros-noetic-mavros \
    ros-noetic-ackermann-msgs \
    ros-noetic-vrpn-client-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-roslint \
    ros-noetic-image-transport-plugins \
    ros-noetic-rosmon && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# 新搞一个，专门处理编译
FROM base AS builder

ARG DEBIAN_FRONTEND=noninteractive
# 安装只有编译/打包才用得到的软件包
RUN apt-get update && apt install -y \
    python3-bloom \
    dh-make \
    fakeroot
# 给rosdep换源
ENV ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
RUN mkdir -p /etc/ros/rosdep/sources.list.d/ && \
    curl -o /etc/ros/rosdep/sources.list.d/20-default.list -L https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list && \
    rosdep update --include-eol-distros

# 创建ubuntu用户，因为rosdep以root运行似乎有问题，但是目前还没有进行消融实验
ARG USERNAME
ARG USER_UID
ARG USER_GID
ARG PASSWORD

# 创建用户组和用户
RUN groupadd -g $USER_GID $USERNAME && useradd -m -u $USER_UID -g $USER_GID $USERNAME && \
    echo "${USERNAME}:${PASSWORD}" | chpasswd && \
    usermod -aG sudo $USERNAME && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME

# 设置工作目录
USER $USERNAME
ARG WORKSPACE_DIR
WORKDIR /home/$USERNAME/$WORKSPACE_DIR

# 复制代码到docker内，复制完这个代码就基本无法从docker的镜像中彻底消除了，所以后面用复制的方法进行打包
COPY --chown=${USERNAME}:${PASSWORD} ./src/message_typedef /home/$USERNAME/$WORKSPACE_DIR/src/message_typedef
COPY --chown=${USERNAME}:${PASSWORD} ./src/dr-visual-frontend /home/$USERNAME/$WORKSPACE_DIR/src/dr-visual-frontend
COPY --chown=${USERNAME}:${PASSWORD} ./src/fusion_odometry /home/$USERNAME/$WORKSPACE_DIR/src/fusion_odometry
COPY --chown=${USERNAME}:${PASSWORD} ./src/image_transport /home/$USERNAME/$WORKSPACE_DIR/src/image_transport
COPY --chown=${USERNAME}:${PASSWORD} ./src/locator_fdi /home/$USERNAME/$WORKSPACE_DIR/src/locator_fdi

# 正式开始编译并生成install，如果需要生成deb或者不进行任何编译则进行对应修改
# 这里的mount=type=cache是为了将编译结果保存，等下次进入时无须从头编译
ARG INSTALL_DIR
RUN --mount=type=cache,uid=${USER_GID},gid=${USER_GID},target=/home/$USERNAME/.cache,rw \
    bash -c 'echo "$(ls /home/$USERNAME/.cache -A 2>/dev/null)" && \
    if [ -n "$(ls /home/$USERNAME/.cache -A 2>/dev/null)" ]; then cp -a -r /home/$USERNAME/.cache/* /home/$USERNAME/$WORKSPACE_DIR; fi && \
    source /opt/ros/noetic/setup.bash && \
    cd /home/$USERNAME/$WORKSPACE_DIR/src && \
    pwd && \
    cd /home/$USERNAME/$WORKSPACE_DIR && \
    catkin_make -DCMAKE_BUILD_TYPE=Release && \
    catkin_make -DCMAKE_INSTALL_PREFIX=/home/$USERNAME/$INSTALL_DIR install && \
    cp -a -r /home/$USERNAME/$WORKSPACE_DIR/build /home/$USERNAME/.cache/build && \
    cp -a -r /home/$USERNAME/$WORKSPACE_DIR/devel /home/$USERNAME/.cache/devel'

# 获得最终编译的install文件，这样在新的docker中就没有src的数据了，同时可以减少每次更新的体积
FROM base AS final

# dockerfile中的arg每次FROM都会清空，这里需要重新声明/定义
ARG USERNAME
ARG USER_GID
ARG WORKSPACE_DIR
ARG INSTALL_DIR

# 把make install所得内容，拷贝到最终的docker中，并赋予可执行权限
COPY --from=builder --chmod=0755 /home/$USERNAME/$INSTALL_DIR /root/$INSTALL_DIR

# install方法必须要加，更新动态链接库的位置信息
RUN ldconfig
