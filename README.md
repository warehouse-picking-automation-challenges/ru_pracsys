# Rutgers APC Main Development Repository

This is the set of code used by the Rutgers University team for the APC in Seattle. Some code is adapted from other open source packages, and any updates to those external repositories should be considered over the versions here. This repository corresponds to the entire catkin_workspace of code used for our contribution.

Below, you will find the instructions compiled for installing this code on a new Ubuntu 12.04 machine in ROS Hydro. Some of the code may still be compatible with ROS Indigo, but will require the corresponding ROS package updates in the instructions. 

There are some dependency changes required in packages such as OpenCV, which are included in a separate repository on Bitbucket. If you require access to this repository, contact the PRACSYS lab at Rutgers at pracsyslab.org

## Installation:

### First, download code from this repository onto your machine (without compiling!)

Note: For this and the rest of the install, please make sure and replace {USERNAME} with your appropriate username
 

### Next, edit your `~/.bashrc` file

Include the lines:
```
#ROS
source /opt/ros/hydro/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/{USERNAME}/apc_pkgs/code
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/{USERNAME}/apc_main

#PRACSYS
export LIBRARY_PATH=$LIBRARY_PATH:$LD_LIBRARY_PATH
export PRACSYS_PATH=/home/{USERNAME}/apc_main/src
export PRACSYS_MODELS_PATH=/home/{USERNAME}/apc_main/src/prx_models

#OPENCV (compiled from source below)
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

### Now, we'll walk through installation of the software's dependencies

Note: If some of the apt-get install below fail (find inconsistences), you might need to apt-get upgrade. But be careful with this command because it can break you Ubuntu for good.

#### Install g++
```
$ sudo apt-get install g++
```

#### Install ODE
Dowload the latest version from to a suitable directory (i.e. ~/LIBRARIES): http://sourceforge.net/projects/opende/
```
$ cd ~/LIBRARIES/
$ bzip2 -d ode-0.13.tar.bz2
$ tar xvf ode-0.13.tar
$ cd ode-0.13
$ ./configure --enable-double-precision --enable-shared
$ make
$ sudo make install
```

#### Install OpenSceneGraph
```
$ sudo apt-get install libopenscenegraph-dev
```

#### Install CMake
```
$ sudo apt-get install cmake
```

#### Install bullet (download from http://code.google.com/p/bullet/downloads/detail?name=bullet-2.78-r2387.tgz&can=2&q=) -> use a newer one (above r2393)
```
$ sudo mkdir /usr/local/bullet (this step doesn't seem to do anything because the library is not installed locally by default, but into /usr/local/lib and /usr/local/include)
$ sudo mv bullet-2.78-r2387.tgz  /usr/local/bullet/ (see comment above)
$ cd /usr/local/bullet/ (see comment above)
$ sudo tar xzvf bullet-2.78-r2387.tgz 
$ cd bullet-2.78-r2387
$ cmake . -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON
$ sudo make
$ sudo make install
```

#### Install lapack
```
$ sudo apt-get install liblapack-dev
```

#### Download our separate `apc-pkgs` repository
Directions here: https://bitbucket.org/pracsys/apc-pkgs

#### Install ROS
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu quantal main" > /etc/apt/sources.list.d/ros-latest.list' (should be precise instead of quantal)
$ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install ros-hydro-desktop-full
$ sudo apt-get install ros-hydro-qt-build ros-hydro-driver-common ros-hydro-gazebo-ros-control ros-hydro-gazebo-ros-pkgs ros-hydro-ros-control ros-hydro-control-toolbox ros-hydro-realtime-tools ros-hydro-ros-controllers ros-hydro-xacro
$ sudo apt-get install ros-hydro-moveit-core
$ sudo apt-get install ros-hydro-moveit-full
$ sudo apt-get install ros-hydro-moveit-full-pr2
$ sudo apt-get install ros-hydro-object-recognition-*
$ sudo apt-get install ros-hydro-rviz ros-hydro-rqt ros-hydro-openni*
$ sudo apt-get install libfreenect-dev
$ sudo apt-get install freenect
$ sudo apt-get install ros-hydro-freenect-launch
$ source ~/.bashrc
```

#### Install Gazebo and QT4
```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu trusty main" > /etc/apt/sources.list.d/gazebo-latest.list' (should be precise instead of trusty)
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install qt4-dev-tools qt4-designer
$ sudo apt-get install gazebo
```

#### Install DepthSense library
```
$ cd ~/apc_pkgs/libraries
$ sudo ./DepthSenseSDK-1.4.3-1527-amd64-deb.run
```

#### Fix OpenCV includes
Also, due to the order of includes established by ROS (source devel/setup.sh), you will have to:
```
$ cd apc_pkgs/code
$ sudo mv /opt/ros/hydro/include/opencv2/objdetect/objdetect.hpp /opt/ros/hydro/include/opencv2/objdetect/objdetect.hpp.old
$ sudo cp opencv-2.4.10/modules/objdetect/include/opencv2/objdetect/objdetect.hpp /opt/ros/hydro/include/opencv2/objdetect/objdetect.hpp
```

#### Install ORK dependencies
```
$ export DISTRO=hydro
$ sudo apt-get install libopenni-dev ros-${DISTRO}-catkin ros-${DISTRO}-ecto* ros-${DISTRO}-opencv-candidate ros-${DISTRO}-moveit-msgs (already done by this point)
$ sudo apt-get install couchdb (already done by this point)
$ sudo apt-get install curl libglew-dev
$ sudo apt-get install libosmesa-dev (already done by this point)
$ sudo apt-get install ros-hydro-household-objects-database
```

#### Install FCL
```
$ sudo apt-get install libfcl-dev
```

#### Compile external dependencies
```
$ sudo apt-get install libf2c2-dev
$ cd src/prx_external
$ cmake .
$ make
$ cd ../..
```

### Compile code from this repository

```
$ cd ~/apc_main
$ catkin_make 
```
Note: you may get an error about not finding libGL.so in a certain directory. If this library is on your system, just put a symlink in the expected place.)

### Post-installation

#### Change `~/.bashrc` to look like this in the end 
```
#CUDA
export PATH=$PATH:/usr/local/cuda-6.5/bin
export LD_LIBRARY_PATH=/usr/local/cuda-6.5/lib:/usr/local/cuda-6.5/lib64:$LD_LIBRARY_PATH

#ROS
source /opt/ros/hydro/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/{USERNAME}/apc_pkgs/code
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/{USERNAME}/apc_main

#PRACSYS
export LIBRARY_PATH=$LIBRARY_PATH:$LD_LIBRARY_PATH
export PRACSYS_PATH=/home/{USERNAME}/apc_main/src
export PRACSYS_MODELS_PATH=/home/{USERNAME}/apc_main/src/prx_models

#APC
source ~/apc_main/devel/setup.sh

#OPENCV (compiled from source below)
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

#### Setup object models database
```
$ sudo python get-pip.py
$ sudo pip install -U couchapp
$ cd ./src/object_recognition_core/web_ui
$ couchapp push . http://localhost:5984/or_web_ui
$ catkin_make install --pkg baxter_ikfast_right_arm_plugin
$ catkin_make install --pkg baxter_ikfast_left_arm_plugin
$ sudo apt-get install python-pyassimp
```

#### Change your Kinect driver (libfreenect) settings to work better with our software
```
$ sudo apt-get purge libfreenect libfreenect-dev libfreenect-demos
$ sudo apt-get purge ros-hydro-libfreenect
$ mkdir ~/LIBRARIES 
$ cd LIBRARIES
$ git clone https://github.com/OpenKinect/libfreenect.git
$ cd libfreenect
$ mkdir build
$ cd build
$ cmake -L ..
$ make
$ sudo make install
$ cd ~/apc_hg2
$ catkin_make
```