###### tags: `Robotics`

# Cross-compile ROS2

參考：https://index.ros.org/doc/ros2/Tutorials/Cross-compilation

以使用者來說並不需要自行編譯 ROS2，但有的時候執行環境的資源有限，可能無法編譯 ROS2 pacakge (像 raspberry pi 3+ 就是)，因此會需要 cross-compile

docker 會是一個很有用的工具，它能直接弄好目標的執行環境，以 cross-compile 來說會是最快能夠上手的。其他方式像是用 apt 等方法下載 cross compiler 套件，但是這會需要設定編譯時連結的函式庫，ROS2 (其實 ROS 也是) C++ package 都是用 cmake 工具組建，要確保每個 package 都能連結到目標架構的函式庫其實是非常困難的，在這邊踩了相當多的雷，因此不建議直接用 cross compiler，例如用 apt 下載的 aarch64-linux-gnu-g++ toolchain。

使用 multiarch docker 時，每個命令執行都會使用到 qemu-user-static，因此執行效能會比較差，建議使用比較好的電腦來跑。

其他類似的工具 (還沒研究)： [ros_cross_compile](https://pypi.org/project/ros-cross-compile/)

## 所需工具

- **docker**
- **qemu-user-static**

docker 命令簡單筆記

### **`docker build`**: 建立一個映像檔
```
$ docker build [OPTIONS] PATH
```
Options:

- `-f`: Name of the Dockerfile (Default is 'PATH/Dockerfile')
    用來指定 Dockerfile 的位置
- `-t`: 'name:tag' format
    可以指定產生映像檔的 name 和 tag

### **`docker run`**: 啟動 container
```
$ docker run [OPTIONS] IMAGE [COMMAND] [ARG...]
```

- `-t`: Allocate a pseudo-TTY
- `-i`: interactive，使用互動模式，如果沒有指定這個選項需要餵給 docker 一個腳本
- `--name string`: Assign a name to the container

```
$ docker run -it <image> /bin/bash
```

### 其他 docker 指令

**`docker pull`**: 從 dockerhub 下載映像檔
**`docker images`**: 列出系統上有的映像檔
**`docker container export`**: 輸出 container 的檔案系統到一個 tar 檔

### qemu-user-static
Interpret other machine's binary in user mode

使用 qemu-arm-static 因為 target 為 arm32v7 架構的機器，-static 代表 qemu-arm 是靜態產生的，不會動態連結其他函式庫。


## build a docker sysroot

以下為 workspace directory tree:
ros2_ws 目錄有所有 ros2 相關的原始碼，entry_point.sh 為 ros2 cross compile 的自動化腳本，

```
.
├── entry_point.sh
├── Dockerfile_debian_arm
├── qemu-user-static
│   └── qemu-arm-static
└─── ros2_ws
    ├── ros2.repos
    └── src
        ├── ...
        └── ...
```

dockerfile for building:
```dockerfile Dockerfile_debian_arm
# Copyright (c) 2018, ARM Limited.
# SPDX-License-Identifier: Apache-2.0

ARG ARM_ARCH=arm32v7
FROM ${ARM_ARCH}/debian:stable

COPY ./qemu-user-static/qemu-arm-static /usr/bin/
COPY ./ros2_ws/src /ros2_ws/src

RUN apt update && apt install -y \
    pkg-config \
    lsb-release \
    curl \
    gfortran \
    libpcre3 \
    libpcre3-dev \
    bash-completion \
    dirmngr \
    gnupg2 \
    ca-certificates \
    tzdata

COPY ./localtime /etc/localtime
RUN echo "Asia/Taipei" > /etc/timezone
RUN cat /etc/timezone
RUN date

# ROS2 dependencies
RUN apt install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget

ENV LANG en_US.UTF-8
ENV LC_ALL C.UTF-8

RUN curl http://repo.ros2.org/repos.key | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main \
    `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt update
RUN apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep

RUN python3 -m pip install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

RUN python3 -m pip install -U \
    pytest \
    pytest-cov \
    pytest-runner \
    setuptools

RUN apt install --no-install-recommends -y \
    libasio-dev \
    libtinyxml2-dev

WORKDIR /ros2_ws
RUN c_rehash /etc/ssl/certs
RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths src \
    --ignore-src \
    --rosdistro crystal -y \
    --skip-keys "console_bridge \
        fastcdr \
        fastrtps \
        libopensplice67 \
        libopensplice69 \
        rti-connext-dds-5.3.1 \
        urdfdom_headers"

WORKDIR /
RUN apt-get install -y symlinks
RUN symlinks -rc .
```

[解決 `rosdep init` 出現 initialization error](https://answers.ros.org/question/54150/rosdep-initialization-error/)，在 dockerfile 加入這幾行
[c_rehash](https://johnpfield.wordpress.com/category/openssl/)
```dockerfile
RUN apt update && apt install -y ca-certificates
RUN c_rehash /etc/ssl/certs
```
使用上方的 dockerfile 建立映像檔：
```bash
$ docker build -t arm_ros2:latest -f Dockerfile_debian_arm .
```

建立 sysroot

```
$ docker run --name arm_sysroot arm_ros2:latest
$ docker container export -o sysroot_docker.tar arm_sysroot
$ mkdir sysroot_docker
$ tar -C sysroot_docker -xf sysroot_docker.tar lib usr opt etc
$ docker rm arm_sysroot
```

## Cross compile ROS2 core stack

使用以下腳本，腳本的環境變數會傳遞給 ros2/cross_compile 這個 package 並設定相關的 CMAKE 變數如 `CMAKE_C_COMPILER`, `CMAKE_CXX_COMPILER` 等

```bash
export TARGET_ARCH=arm
export TARGET_TRIPLE=arm-linux-gnueabihf

export CC=/usr/bin/$TARGET_TRIPLE-gcc
export CXX=/usr/bin/$TARGET_TRIPLE-g++
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

export SYSROOT=`pwd`/sysroot_docker
export PYTHON_SOABI=cpython-36m-$TARGET_TRIPLE
export ROS2_INSTALL_PATH=`pwd`/ros2_ws/install

# Hack to find Poco
## This is temporarily required to find the Poco libraries on the SYSROOT.
## The exported target comming with the pre-build binaries has a hard-coded
## path to "/usr/lib/<arch>/libz.so" and "/usr/lib/<arch>/libpcre.so"
sudo ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libz.so.1 /usr/lib/$TARGET_TRIPLE/libz.so
sudo ln -s `pwd`/sysroot_docker/lib/$TARGET_TRIPLE/libpcre.so.3 /usr/lib/$TARGET_TRIPLE/libpcre.so

# Ignore some package
touch \
    ros2_ws/src/ros2/rviz/COLCON_IGNORE \
    ros2_ws/src/ros-visualization/COLCON_IGNORE

cd ros2_ws

# Trigger a build
colcon build --merge-install \
    --cmake-force-configure \
    --cmake-args \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        -DCMAKE_TOOLCHAIN_FILE="$(pwd)/src/ros2/cross_compile/cmake-toolchains/generic_linux.cmake"
```

目前問題：

1. ld errors
2. librt.so.1 找不到 `fcntl@GLIBC_2.28`
    (sysroot 的 glibc 版本跟 cross compiler 的不一致)
3. cmake variable `TINYXML2_LIBRARY` not set
    [tinyxml2_vendor/issues](https://github.com/ros2/tinyxml2_vendor/issues/4)


## References
- arm docker build, https://www.balena.io/blog/building-arm-containers-on-any-x86-machine-even-dockerhub/
- qemu and binfmt, https://ownyourbits.com/2018/06/13/transparently-running-binaries-from-any-architecture-in-linux-with-qemu-and-binfmt_misc/
- 快速建立你的第一個Docker服務,  https://joshhu.gitbooks.io/dockercommands/content/Containers/DockerRun.html
- multiarch docker, https://eyskens.me/multiarch-docker-images/