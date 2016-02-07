#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream, std::stringbuf

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


// Definitions

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;

struct callback_args
{
	// structure used to pass arguments to the callback function
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr current_cloud;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};


// Global variables

PointCloudXYZRGBA::Ptr g_cloud;
ros::Time g_cloud_timestamp;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
boost::mutex cloud_mutex;
bool new_cloud_available_flag = false;

tf::TransformListener *g_listener;



boost::shared_ptr<pcl::visualization::PCLVisualizer>
create_cloud_and_objects_visualizer (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "scene cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene cloud");
//	viewer->addCoordinateSystem (0.3);
	viewer->initCameraParameters ();

	return (viewer);
}


void
detect_objects(PointCloudXYZRGBA::Ptr& cloud, LineRGBD<PointXYZRGBA>& line_rgbd,
		std::vector<LineRGBD<PointXYZRGBA>::Detection>& detections)
{
	line_rgbd.setInputCloud(cloud);
	line_rgbd.setInputColors(cloud);
//	line_rgbd.detect(detections);
	line_rgbd.detectSemiScaleInvariant(detections);
	line_rgbd.removeOverlappingDetections();
}


void
detect_objects_semi_scale_invariant(PointCloudXYZRGBA::Ptr& cloud, LineRGBD<PointXYZRGBA>& line_rgbd,
		std::vector<LineRGBD<PointXYZRGBA>::Detection>& detections)
{
	line_rgbd.setInputCloud(cloud);
	line_rgbd.setInputColors(cloud);
//	line_rgbd.detect(detections);
	line_rgbd.detectSemiScaleInvariant(detections);
	line_rgbd.removeOverlappingDetections();
}


void
oppenni_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	cloud_mutex.lock ();

	pcl::fromROSMsg(*cloud_msg, *g_cloud);
	g_cloud_timestamp = cloud_msg->header.stamp;
	new_cloud_available_flag = true;

	cloud_mutex.unlock ();
}


void
draw_objects_on_viewer(BoundingBoxXYZ best_object, LineRGBD<PointXYZRGBA>& line_rgbd,
		std::vector<LineRGBD<PointXYZRGBA>::Detection>& detections,
		PointCloudXYZRGBA::Ptr& detected_object_cloud)
{
	viewer->addCube(best_object.x, best_object.x + best_object.width,
					best_object.y, best_object.y + best_object.height,
					best_object.z, best_object.z + best_object.depth,
					1.0, 0.0, 0.0,
					"best object");

	ROS_INFO("########## object x = %f, y = %f, z = %f\n", best_object.x, best_object.y, best_object.z);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb2(detected_object_cloud);
	viewer->addPointCloud<pcl::PointXYZRGBA>(detected_object_cloud, rgb2, "detected object cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "detected object cloud");
}


bool
get_best_object(BoundingBoxXYZ& best_object, LineRGBD<PointXYZRGBA>& line_rgbd,
		std::vector<LineRGBD<PointXYZRGBA>::Detection>& detections,
		PointCloudXYZRGBA::Ptr& detected_object_cloud)
{
	if (detections.size() > 0)
	{
		double max_response = 0.5;
		size_t best_detection_id = detections[0].detection_id;

		for (size_t i = 0; i < detections.size(); ++i)
		{
			if ((detections[i].response > max_response) &&
				(fabs(detections[i].bounding_box.x) < 0.3)) // objects must be in the shelf bin in front
			{
				best_object = detections[i].bounding_box;
				max_response = detections[i].response;
				best_detection_id = detections[i].detection_id;
			}
		}
		// line_rgbd.applyProjectiveDepthICPOnDetection (best_detection_id); // TODO: Alberto: find out why it is not working.
		line_rgbd.computeTransformedTemplatePoints(best_detection_id, *detected_object_cloud);
		pcl::io::savePCDFile((char *) "tmp.pcd", *detected_object_cloud); // TODO: Alberto: I don't know why it is necessary for proper vieweing...
		pcl::io::loadPCDFile((char *) "tmp.pcd", *detected_object_cloud);

		return (true);
	}
	else
	{
		return (false);
	}
}


void
publish_object_pose(BoundingBoxXYZ best_object)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;

//	transform.setOrigin( tf::Vector3(best_object.x, best_object.y, best_object.z) ); // Object corner
	transform.setOrigin( tf::Vector3(best_object.x + best_object.width / 2.0,
									 best_object.y + best_object.height / 2.0,
									 best_object.z + best_object.depth / 2.0) ); // Object center
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, 0.0); // TODO: Alberto: find the rotation of the object.
	transform.setRotation(q);
	if (g_listener->canTransform("/camera_link", "/mapping_camera_frame", ros::Time(0)))
		br.sendTransform(tf::StampedTransform(transform, g_cloud_timestamp, "/mapping_camera_frame", "/object_frame"));
}


void
visualise_detections_in_cloud_from_device(LineRGBD<PointXYZRGBA>& line_rgbd, ros::NodeHandle nh)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr detected_object_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

	ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 5, oppenni_callback);

	ros::Rate loop_rate(40);
	while (nh.ok() && !new_cloud_available_flag)
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	cloud_mutex.lock ();

	viewer = create_cloud_and_objects_visualizer(g_cloud);

	cloud_mutex.unlock ();

	ROS_INFO("Detecting objects in clouds from device.\n");
	while (!viewer->wasStopped() && nh.ok())
	{
		ros::spinOnce();

		if (new_cloud_available_flag && cloud_mutex.try_lock())
		{
			new_cloud_available_flag = false;

			viewer->removeAllPointClouds();
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(g_cloud);
			viewer->addPointCloud<pcl::PointXYZRGBA> (g_cloud, rgb, "scene cloud");

			std::vector<LineRGBD<PointXYZRGBA>::Detection> detections;
			detect_objects_semi_scale_invariant(g_cloud, line_rgbd, detections);

			viewer->removeAllShapes();
			BoundingBoxXYZ best_object;
			if (get_best_object(best_object, line_rgbd, detections, detected_object_cloud))
			{
				draw_objects_on_viewer(best_object, line_rgbd, detections, detected_object_cloud);
				publish_object_pose(best_object);
			}
			else
				ROS_INFO("No objects detected...\n");

			viewer->spinOnce(100);
			cloud_mutex.unlock();
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
}


void
visualise_detections_in_cloud_from_file(LineRGBD<PointXYZRGBA>& line_rgbd, const PointCloudXYZRGBA::Ptr& cloud,
		std::vector<LineRGBD<PointXYZRGBA>::Detection> detections)
{
	viewer = create_cloud_and_objects_visualizer(cloud);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr detected_object_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	BoundingBoxXYZ best_object;
	if (get_best_object(best_object, line_rgbd, detections, detected_object_cloud))
		draw_objects_on_viewer(best_object, line_rgbd, detections, detected_object_cloud);
	else
		ROS_INFO("No objects detected...\n");

	draw_objects_on_viewer(best_object, line_rgbd, detections, detected_object_cloud);

	pcl::io::savePCDFile((char *) "tmp.pcd", *detected_object_cloud);
	pcl::io::loadPCDFile((char *) "tmp.pcd", *detected_object_cloud);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb2(detected_object_cloud);
	viewer->addPointCloud<pcl::PointXYZRGBA>(detected_object_cloud, rgb2, "detected object cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "detected object cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}


void
load_lmt_objects_templates(const std::vector<int>& lmt_file_indices, char** argv, LineRGBD<PointXYZRGBA>& line_rgbd)
{
	for (size_t i = 0; i < lmt_file_indices.size(); ++i)
	{
		std::string lmt_filename = argv[lmt_file_indices[i]];
		line_rgbd.loadTemplates(lmt_filename);
	}
}


void
printHelp (int, char **argv)
{
  print_error ("Syntax is: %s -input input.pcd template.lmt\n", argv[0]);
  print_info ("  where options are (defaults between <>):\n   -grad_mag_thresh <10.0> -detect_thresh <0.75>\n");
}


void
printElapsedTimeAndNumberOfPoints (double t, int w, int h=1)
{
  print_info ("[done, "); print_value ("%g", t); print_info (" ms : ");
  print_value ("%d", w*h); print_info (" points]\n");
}


bool
loadCloud (const std::string & filename, PointCloudXYZRGBA & cloud)
{
  TicToc tt;
  print_highlight ("Loading "); print_value ("%s ", filename.c_str ());

  tt.tic ();
  if (loadPCDFile (filename, cloud) < 0)
    return (false);

  printElapsedTimeAndNumberOfPoints (tt.toc (), cloud.width, cloud.height);

  print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

  return (true);
}


bool
parse_command_line_arguments(int argc, char** argv,
		PointCloudXYZRGBA::Ptr& cloud, std::vector<int>& lmt_file_indices, LineRGBD<PointXYZRGBA>& line_rgbd)
{
	if (argc == 1)
	{
		printHelp(argc, argv);
		exit(-1);
	}

	// Parse the gradient magnitude threshold
	float grad_mag_thresh = 10.0f;
	parse_argument(argc, argv, "-grad_mag_thresh", grad_mag_thresh);

	// Parse the detection threshold
	float detect_thresh = 0.75f;
	parse_argument(argc, argv, "-detect_thresh", detect_thresh);

	// Parse the command line arguments for .lmt files
	lmt_file_indices = parse_file_extension_argument(argc, argv, ".lmt");
	if (lmt_file_indices.empty())
	{
		print_error("Need at least one input LMT file.\n");
		exit(-1);
	}
	line_rgbd.setGradientMagnitudeThreshold(grad_mag_thresh);

	line_rgbd.setDetectionThreshold(detect_thresh);

	std::string input_filename;
	if (parse_argument (argc, argv, "-input", input_filename) < 0)
		return (false);

	// Load the input PCD file
	if (!loadCloud (input_filename, *cloud))
	{
		print_error("Could not read cloud from -input file\n");
		exit(-1);
	}

	return (true);
}


int
main (int argc, char** argv)
{
	bool read_input_from_file;
	std::vector<int> lmt_file_indices;
	PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);
	LineRGBD<PointXYZRGBA> line_rgbd;

	g_cloud = cloud;

	// ROS initialization and remaining line parameters evaluation
	ros::init(argc, argv, "prx_mapping");
	ros::NodeHandle nh("~");

	read_input_from_file = parse_command_line_arguments(argc, argv, cloud, lmt_file_indices, line_rgbd);

	ROS_INFO("Loading linemod templates...\n");
	load_lmt_objects_templates(lmt_file_indices, argv, line_rgbd);
	ROS_INFO("\nFinished loading linemod templates.\n");

	tf::TransformListener listener;
	g_listener = &listener;

	if (read_input_from_file)
	{
		print_info("Detecting objects in cloud from file.\n");
		std::vector<LineRGBD<PointXYZRGBA>::Detection> detections;
		detect_objects(cloud, line_rgbd, detections);
		visualise_detections_in_cloud_from_file(line_rgbd, g_cloud, detections);
	}
	else
		visualise_detections_in_cloud_from_device(line_rgbd, nh);

	return 0;
}
