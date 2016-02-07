#include <ecto/ecto.hpp>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <boost/python/stl_iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/python/overloads.hpp>

#include <iostream>
#include <string>

namespace
{

  struct RosLifetime
  {
    RosLifetime()
        :
          spinner_(0)//0 means many threads.
    {
      spinner_.start();
    }
    ~RosLifetime()
    {
    }
    ros::AsyncSpinner spinner_;
    ros::NodeHandle nh_; //keeps ros alive.
    static boost::scoped_ptr<RosLifetime> its_a_ros_life_;
  };

  boost::scoped_ptr<RosLifetime> RosLifetime::its_a_ros_life_;
  namespace bp = boost::python;

  void
  removeROSArgs(bp::list sys_argv)
  {
    std::vector<std::string> args;
    bp::stl_input_iterator<std::string> begin(sys_argv), end;
    std::copy(begin, end, std::back_inserter(args));
    char** argv = new char*[args.size()]; //array to emulate argv.
    //
    for(int i = 0, ie = args.size(); i < ie; ++i)
    {
      argv[i] = const_cast<char*>(args[i].data());
    }

    int ac = args.size();
    std::vector<std::string> oargs;
    ros::removeROSArgs(ac,argv,oargs);
    //forward the arg stripping back to python
    while (bp::len(sys_argv))
      sys_argv.pop();

    for (size_t i = 0; i < oargs.size(); ++i)
    {
      sys_argv.append(bp::str(oargs[i]));
    }
    delete[] argv;
  }

  void
  ros_init(bp::list sys_argv, const std::string& node_name, bool anonymous = true)
  {
    std::vector<std::string> args;
    bp::stl_input_iterator<std::string> begin(sys_argv), end;
    std::copy(begin, end, std::back_inserter(args));
    char** argv = new char*[args.size()]; //array to emulate argv.
    //
    for(int i = 0, ie = args.size(); i < ie; ++i)
    {
      argv[i] = const_cast<char*>(args[i].data());
    }

    int ac = args.size();
    int flags = ros::init_options::NoSigintHandler;
    if (anonymous)
    {
      flags |= ros::init_options::AnonymousName;
    }
    if (!ros::isInitialized()) //enable multiple calls.
    {
      ros::init(ac, argv, node_name.c_str(), flags);
      RosLifetime::its_a_ros_life_.reset(new RosLifetime());
      ROS_INFO_STREAM("Initialized ROS. node_name: " << ros::this_node::getName());
    }
    else
    {
      ROS_INFO_STREAM("System already initialized. node_name: " << ros::this_node::getName());
    }
    //forward the arg stripping back to python
    while (bp::len(sys_argv))
      sys_argv.pop();

    for (int i = 0; i < ac; ++i)
    {
      sys_argv.append(bp::str((const char*) (argv[i])));
    }
    delete[] argv;
  }
  BOOST_PYTHON_FUNCTION_OVERLOADS(ros_init_overloads, ros_init, 2, 3)
}

ECTO_DEFINE_MODULE(ecto_ros_main)
{
  using bp::arg;
  bp::def("init", ros_init, ros_init_overloads("Initialized the roscpp node context.",(arg("argv"),arg("node_name"),arg("anonymous"))));
  bp::def("strip_ros_args", removeROSArgs,"Removes the ROS remapping arguments.");

}
