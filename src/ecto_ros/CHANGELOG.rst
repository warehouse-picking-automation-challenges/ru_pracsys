^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ecto_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.5 (2014-11-11)
------------------
* Merge pull request `#15 <https://github.com/plasmodic/ecto_ros/issues/15>`_ from stonier/synch_fix
  Synchroniser fixes to behavior introduced by subscriber DO_OVER's
* bugfixes for the synchroniser fixes.
* a plasm should not be created in the configure function
  I fixed that as the doc was broken (make sphinx-doc did not work)
* Merge pull request `#13 <https://github.com/plasmodic/ecto_ros/issues/13>`_ from stonier/lazy_publishing
  Exposing number of subscribers
* Merge pull request `#14 <https://github.com/plasmodic/ecto_ros/issues/14>`_ from stonier/lazy_mat2img_publisher
  Lazy Mat2Image publisher blackbox
* bugfix subscriber cell handling in synchronisers for recent DO_OVER update, closes `#11 <https://github.com/plasmodic/ecto_ros/issues/11>`_
* lazy mat2image publisher blackbox.
* do not be lazy if it is a latched publisher.
* publisher cells now provide a has_subscribers boolean output.
* check for number of subscribers.
* Contributors: Daniel Stonier, Vincent Rabaud

0.4.4 (2014-09-06)
------------------
* Merge pull request `#10 <https://github.com/plasmodic/ecto_ros/issues/10>`_ from stonier/get_subscribers
  Check for subscribers before publishing
* get subscribers to give the scheduler a chance to make decisions instead of permanently blocking.
* Contributors: Daniel Stonier, Vincent Rabaud

0.4.3 (2014-07-27)
------------------
* Actually apply the queue size
* cleanup tests and dot not depend on vault.willowgarage.com
* Actually apply the queue size
* remove usage of SYSTEM in include_directories.
* don't rely on the user setting up the catkin environment for the script.
* Contributors: Daniel Stonier, Vincent Rabaud

0.4.2 (2014-04-03)
------------------
* fixes `#2 <https://github.com/plasmodic/ecto_ros/issues/2>`_
* Contributors: Vincent Rabaud

0.4.1 (2014-03-10)
------------------
* fix farm errors on Indigo
* Contributors: Vincent Rabaud

0.4.0 (2014-03-04)
------------------
* add rosmsg dependency for message generation
* use proper catkin to download test data
* remove stack.xml now that Fuerte is dropped
* use floats for K and make sure D is filled
* remove Fuerte support
* update url
* remove Sphinx warnings
* cleanup the CMake
* ros_comm is a meta-package but rosbag is definitely needed
* Contributors: Vincent Rabaud

0.3.23 (2013-01-13)
-------------------
* use the proper catkin variable
* Contributors: Vincent Rabaud

0.3.22 (2013-01-04)
-------------------
* comply to the new catkin API
* comply to the new ecto API
* re-add missing dependencies
* fix the renaming of genmsg
* remove some warnings for the latest catkin
* Contributors: Vincent Rabaud

0.3.21 (2012-11-18 15:21)
-------------------------
* fix extra line
* Contributors: Vincent Rabaud

0.3.20 (2012-11-18 15:13)
-------------------------

0.3.19 (2012-11-18 15:12)
-------------------------
* fix the Python for Fuerte
* Contributors: Vincent Rabaud

0.3.18 (2012-11-02)
-------------------
* use catkin_pkg
* Contributors: Vincent Rabaud

0.3.17 (2012-11-01)
-------------------
* use the new ecto_catkin interface
* add missing dependencies
* retrieve data fro mthe package.xml
* remove electric support
* Contributors: Vincent Rabaud

0.3.16 (2012-10-09)
-------------------
* output error message if any
* use intersphinx for docs
* comply to the new API
* comply to the new catkin API
* Contributors: Vincent Rabaud

0.3.15 (2012-09-10)
-------------------
* fix bug on Lucid
* Contributors: Vincent Rabaud

0.3.14 (2012-09-09)
-------------------
* fix install on Groovy
* Contributors: Vincent Rabaud

0.3.13 (2012-09-08)
-------------------
* have code work with Electric/Fuerte/Groovy
* fix pubsub to the new API
* Merge branch 'master' of github.com:plasmodic/ecto_ros
* fix typo
* changed doc index heading
* Contributors: David Gossow, Vincent Rabaud

0.3.12 (2012-07-12)
-------------------
* implement our own queue to always get the head of the queue
* Contributors: Vincent Rabaud

0.3.11 (2012-06-07)
-------------------
* do not find gen stuff if not needed
* Contributors: Vincent Rabaud

0.3.10 (2012-06-05 18:17)
-------------------------
* remove the OpenCV tests
* Contributors: Vincent Rabaud

0.3.9 (2012-06-05 13:17)
------------------------

0.3.8 (2012-06-04)
------------------
* use a stack.xml
* remove slow tests
* Contributors: Vincent Rabaud

0.3.7 (2012-05-11)
------------------
* clean the CMake a bit
* fix some samples
* useless Makefile
* reenable some tests and make them pass
* delete useless files
* Contributors: Vincent Rabaud

0.3.6 (2012-04-30 04:11)
------------------------
* other try at fixing pubsub
* Contributors: Vincent Rabaud

0.3.5 (2012-04-30 02:11)
------------------------
* solve some install problems with gen_pun_wrap
* Contributors: Vincent Rabaud

0.3.4 (2012-04-24 17:00)
------------------------
* add missing eigen dependency
* Contributors: Vincent Rabaud

0.3.3 (2012-04-24 16:12)
------------------------
* add missing opencv2 dependency
* Contributors: Vincent Rabaud

0.3.2 (2012-04-24 14:28)
------------------------
* bump version
* rename the ecto_ros.ecto_ros module to ecto_ros.ecto_ros_main and make ecto_ros include it automatically
* use catkin for python
* update the docs
* make sure we can use the macro from an installed ecto_ros
* Contributors: Vincent Rabaud

0.3.1 (2012-04-10)
------------------
* bump the version number
* make sure it works under electric
* allow the creation of msg cells for packages not built yet
* forgot the setup.py
* simplify the CMake
* simplify CMake
* fix some bad rosbag linkage
* Contributors: Vincent Rabaud

0.3.0 (2012-03-12)
------------------
* rosbag is in ros_comm
* fix a few glitches with rosbag
* clean the find_package
* fix the bad install
* better stack dependencies
* nav_msgs is a package so depend on common_msgs
* make sure we make the genpub macros available to everybody
* bump the version number
* - remove useless files
  - add hooks
* make sure the unittests pass
* have the code be compliant with electric and fuerte, yay ...
* use proper catkin macros
* add the missing ROS include
* make the macro to create new publishers more usable by outsie projects
* fix the environment variables
* get ecto_ros compiling on ROS again
* make sure it works with catkin on fuerte
* Minor tweaks to cv bridge stuff.
* Merge branch 'master' of github.com:ethanrublee/ecto_ros
* Working on multithreaded scheduler fixes.
* Remove ROS from the python.
* Fix typo.
* - fix bad synchronizer
* Fixing Synchronizer for new cell.__impl interface to python cells.
* Test sync in ros, atleast the connections.
* tweaks for ros compile-time speedup and refactoring
* Experimenting with ros build stuff.
* Remove bogus arg.
* Clean up the sync_sub sample a bit, with comments.
* Gah!
* Clean up ros samples a bit, regarding imshow.
* - add a new conversion from point cloud message to depth image message
  Merge branch 'master' of git://github.com/plasmodic/ecto_ros
  Conflicts:
  src/cv_bridge.cpp
* - add a new conversion from a point cloud message to a depth image message
* Merge branch 'master' of git://github.com/plasmodic/ecto_ros
* Adding time tweaks so that time does not depend on ros::init being called,
  Also a bit more cv bridge stuffs.  Most likely need to move this out of
  ecto_ros.
* using rosbuild_lite_init
* Making test robust to environment.
* Removing some checks, conforming to envless cmake stuffs, this is
  still experimental.
* Quiet down now you too.
* Remove ros remapping args.
* Disable roscore tests for now, in favor of DESKTOP tests in a bit.
* Minor errors in how configs were generated.
* Adding cv::Mat to sensor_msg::PointCloud converters.
* no-strict-aliasing
* Quiet a bit and minor tweaks.
* docs for pub/sub/bag cells
* make ros message modules import ecto_ros
  make generated code dependent on the generator itself so that rebuilds are Korrect if it changes
* brief how to remap doc.
* configure and process signature changes:  const correctness
* rosbuild lite interface updates
* ref `#138 <https://github.com/plasmodic/ecto_ros/issues/138>`_
* Merge branch 'master' of git://github.com/plasmodic/ecto_ros
* Fix bagwriter for new interface.
* - add the possibility to swap channels
* Making test less anal, expecting > 0 and <= the number in the bag. Hack for ros.
* py2.6 fixes and delay to fix spurious failures, which will eventually
  just be back to haunt us in some other situation  :(
* clean up cmake output
* Merge branch 'master' of github.com:plasmodic/ecto_ros
* somewhat more printy test
* Making project explicit in tests.
* Quit synchronizer properly.
* Adding some regression tests.
* Adding test for bag reading. Fixing up bag reader, and synchronizer.
* updates for new tendrils interface
* tendril iface overhaul
* cmakelists cleanups... cache ros env variables
* merge
* Upgrades for tendrils changes.
* move make_tendril to namespace scope for symmetry with make_shared, etc
* updates for removal of read() from tendrils
* move out of 'scripts', might be confusing, hide message generation in
  cmake with the other build system stuff
* cleanups to cmake verbosity
* Merge branch 'master' of git://github.com/plasmodic/ecto_ros
* Merge branch 'master' of git://github.com/plasmodic/ecto_ros
* Reflecting rosbuild_lite in ROS. FIXME, need rosbuild_lite somewhere common.
* stub doc
* Subproject support.
* more ecto kitchen tweaks
* Just disable build of ecto_ros if ros env isn't sourced correctly
* Merge branch 'master' of github.com:straszheim/ecto_ros
* get things tuned up so's they work in the ecto kitchen
* Adding a camera info to cv::type converter.
* More sample clean up.
* Samples.
* Moving to samples.
* Adding bag writer.
* Dentation.
* Adding bag reader.
* Removing verbosity in synchronizer and adding an overload to the ros init function.
* cruft
* Bringing up to snuff with removal of spore operator() interface.
* Bit of clean up.
* Adding an ecto synchronizer. consider unstable.
* Fix typo, add verbosity to ros logging.
* Adding cmake infrastructure.
* Adding opencv types to pose support, HACK.
* Adding pose stuffs.
* Explicit with version.
* Remove windows line endings. Version str protect.
  deps to reflect what is actually needed.
  Adding argv stripping to init function.
  redentation.
  dentation.
  Will strip.
  Stripping options.
* threading and usb_camera in ecto prototype.
* Working on two way cv bridge.
* Using python based c++ code generation to enable wholesale wrapping of all of common_msgs
  in ROS. See ticket `#3 <https://github.com/plasmodic/ecto_ros/issues/3>`_.
  Adding a message wrapper script.
  Refactor, to include generated messages.
  Almost final touches on generation of all of common_msgs. Reference ticket `#1 <https://github.com/plasmodic/ecto_ros/issues/1>`_
* Bringing up to snuff with latest ecto refactor.
* Rename and add pubs.
* rosbuild_lite is rocking.
* bump.
* Adding toplevel makefile.
* ros lite.
* Rename -> stackage.
* More make.
* Adding cmake infrastructure for install and standalone rosbuild.
* Merge branch 'master' of github.com:plasmodic/ecto_ros
* Works without rosbuild.
* an envless script.
* working on rosmakeless.o
* Non local manifesto.
* manifesto.
* Fleshing out ROS. Have a templated way of wrapping a simple subscriber.
* Adding manifest export.
* Working with ros a bit more. Use strand on highgui.
* Adding sub and bridge.
* camera sub.
* compiles now.
* Adding .gitignore.
* initial add.
* Contributors: Ethan, Ethan Rublee, Troy D. Straszheim, Troy Straszheim, Vincent Rabaud
