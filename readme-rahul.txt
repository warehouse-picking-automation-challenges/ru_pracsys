- Change your ~/.bashrc
  Add the line below at the end of your ~/.bashrc file
  export LIBRARY_PATH=$LIBRARY_PATH:$LD_LIBRARY_PATH

- Compile external dependencies
  $ cd src/prx_external
  $ cmake .
  $ make
  $ cd ../..
  $ catkin_make

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

- Install bullet (download from http://code.google.com/p/bullet/downloads/detail?name=bullet-2.78-r2387.tgz&can=2&q=)
  $ sudo mkdir /usr/local/bullet
  $ sudo mv bullet-2.78-r2387.tgz  /usr/local/bullet/
  $ cd /usr/local/bullet/
  $ sudo tar xzvf bullet-2.78-r2387.tgz 
  $ sudo ./configure
  $ sudo make
  $ sudo make install

- Install lapack
  $ sudo apt-get install liblapack-dev

- Set environment variables in ~/.bashrc
  $ export PRACSYS_PATH=~/RUTGERS/apc_hg/src
  $ export PRACSYS_MODELS_PATH=~/RUTGERS/apc_hg/src/prx_models

- Check the instalation instructions in readme-alberto.txt and
  $ catkin_make
  If it fails, repete the command (catkin_make)


~~~~~~~~~~~~~~~ RUNNING BAXTER SIMULATOR ~~~~~~~~~~~~~~~~~~~~~
- source ~/RUTGERS/apc_hg/devel/setup.sh
- roslaunch ~/RUTGERS/apc_hg/src/prx_packages/baxter/input/baxter_simulator.launch
