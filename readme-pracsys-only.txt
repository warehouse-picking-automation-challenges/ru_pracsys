================================================================================================================
Installation process for the light/PRACSYS version of the APC repository
================================================================================================================

- Change ~/.bashrc to include the following:
  #ROS
  source /opt/ros/hydro/setup.bash
  source ~/repositories/apc/devel/setup.sh
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/USERNAME/repositories/apc-svn/code
  export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/USERNAME/repositories/apc

  #PRACSYS
  export LIBRARY_PATH=$LIBRARY_PATH:$LD_LIBRARY_PATH
  export PRACSYS_PATH=/home/USERNAME/repositories/apc/src
  export PRACSYS_MODELS_PATH=/home/USERNAME/repositories/apc/src/prx_models

- Install svn
  $ sudo apt-get install subversion

- Install mercurial
  $ sudo apt-get install mercurial
   
  In order to commit changes, make sure that you have an .hgrc in your
  home directory and add the following inside it:
    [ui]
    username = SOME_USERNAME  
    [extensions]
    progress = 

- We assume here the existence of a "repositories" directory under
your home directory. The Mercurial and subversion repositories will be
installed under the "repositories" directory as "apc" and "apc-svn"
respectively.
  $ mkdir ~/repositories

- Download our code in the Mercurial repository
  $ cd ~/repositories
  $ hg clone
  ssh://YOUR_USERNAME_ON_SOURCEFORGE@hg.code.sf.net/p/rutgersamazonpickingchallenge/mercurial  apc

- Download the SVN repository
  $ cd ~/repositories
  $ svn checkout svn+ssh://YOUR_USERNAME_ON_SOURCEFORGE@svn.code.sf.net/p/rutgersamazonpickingchallenge/svn apc-svn

- Install g++
  $ sudo apt-get install g++

- Install ODE
  Dowload the latest version from to a suitable directory (i.e. ~/LIBRARIES): http://sourceforge.net/projects/opende/
  $ cd ~/LIBRARIES/
  $ bzip2 -d ode-0.13.tar.bz2
  $ tar xvf ode-0.13.tar
  $ cd ode-0.13
  $ ./configure --enable-double-precision --enable-shared
  $ make
  $ sudo make install

- Install OpenSceneGraph
  $ sudo apt-get install libopenscenegraph-dev

- Install CMake
  $ sudo apt-get install cmake

- Install bullet v. 2.82
  $ sudo tar xzvf bullet-2.82-rXXXX.tgz 
  $ cd bullet-2.82-rXXXX
  $ cmake . -DBUILD_SHARED_LIBS=ON -DUSE_DOUBLE_PRECISION=ON
  $ sudo make
  $ sudo make install

- Install lapack
  $ sudo apt-get install liblapack-dev

- Install CMAKE utilities
  $ sudo apt-get install cmake-data cmake-curses-gui libboost-dev
  
- Install ROS
  $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
  $ wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
  $ sudo apt-get update
  $ sudo apt-get install ros-hydro-desktop-full
  $ source ~/.bashrc
  
- Install FCL
  $ sudo apt-get install ros-hydro-fcl 

- Install ASSIMP
  $ sudo apt-get install ros-hydro-assimp-devel

- Installing Motoman Dependencies - RBDL
  $ cd repositories/apc-svn/libraries/rbdl-rbdl-de94c4fadf94
  $ mkdir build
  $ cd build
  $ cmake -D RBDL_BUILD_ADDON_URDFREADER=True ../
  $ make
  $ sudo make install

- Compile external dependencies (from the hg/mercurial repostiry)
  (e.g., for ANN and other pracsys dependencies)
  $ cd src/prx_external
  $ cmake .
  $ make
  $ cd ../..

- Compile code in Mercurial
  $ cd ~/repositories/apc
  $ catkin_make --only-pkg-with-deps prx_planning
  $ catkin_make --only-pkg-with-deps baxter_interface
  $ catkin_make --only-pkg-with-deps prx_decision_making
  $ catkin_make --only-pkg-with-deps robotiq_s_model_control

- Make sure that the .bashrc contains the lines indicated in the
  beginning of this readme file. 
  $ source ~/.bashrc

================================================================================================================
Running the Motoman Simulator
================================================================================================================

$ roslaunch prx_simulation motoman_sim.launch

This launches 3 nodes, the joint trajectory action server, the robot
state publisher, and an rviz node for visualization.

You can then run the sample joint sending script

$ cd src/prx_simulation/prx
$ python move_to_joint.py [] NUM

where NUM is a duration for each trajectory. 2 or 3 is the fastest we
will be making Motoman move.


================================================================================================================
Running the Motoman: Testing with Planning
================================================================================================================

On terminal 1:
$ roslaunch prx_simulation motoman_sim.launch

This launches 3 nodes, the joint trajectory action server, the robot
state publisher, and an rviz node for visualization.

On terminal 2:
$ roslaunch prx_mapping prx_mapping_no_mapping_motoman.launch

It communicates to tf a static transformation for the base of Motoman

On terminal 3:
$ roslaunch prx_planning motoman_planning.launch

This will start up the PRACSYS planning infrastructure. Give it some
time to create its internal structures, then you will see "
prx_planning is ready to receive queries."

On terminal 4: 
$ rosrun prx_decision_making ru_decision_making example2.json

Initiate the state machine that controls the robot for the Amazon
Picking Challenge. The last argument corresponds to the list of items
to be picked up.


On terminal 5:
$ rosrun prx_planning prx_task_planning.py

This initiates the higher-level task planning python code for relaying
commands to the PRACSYS motion planning code depending on the state of
the decision making automaton.

================================================================================================================
Troubleshooting
================================================================================================================

GLX:

If you have issues with GLX then: 
sudo apt-get remove --purge xserver-xorg
sudo apt-get install xserver-xorg
sudo dpkg-reconfigure xserver-xorg
