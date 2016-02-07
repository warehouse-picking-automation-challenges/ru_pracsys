#include <iostream>

#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/angles.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "pcl_ros/point_cloud.h"
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/fill_image.h"
#include <std_msgs/Empty.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <dynamic_reconfigure/server.h>
#include <prx_mapping/prx_mappingConfig.h>
#include <prx_decision_making/DecisionMakingStateMessage.h>
#include <prx_decision_making/decision_making.h>


typedef pcl::ScopeTime ScopeTimeT;

using namespace std;
using namespace pcl;
using namespace pcl::gpu::kinfuLS;
using namespace pcl::gpu;
using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
namespace pc = pcl::console;

#include "kinfuLS.h"


// Global variables
int g_mapping_mode;
tf::Transform g_mapping_camera_frame_transform;
int g_robot_arm;


namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      void paint3DView(const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
    }
  }
}

namespace pcl
{
  namespace surface
  {
    void simplify(const pcl::PolygonMesh& input, pcl::PolygonMesh& output);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 void
 setViewerPose (visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
 {
 Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
 Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
 Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
 viewer.camera_.pos[0] = pos_vector[0];
 viewer.camera_.pos[1] = pos_vector[1];
 viewer.camera_.pos[2] = pos_vector[2];
 viewer.camera_.focal[0] = look_at_vector[0];
 viewer.camera_.focal[1] = look_at_vector[1];
 viewer.camera_.focal[2] = look_at_vector[2];
 viewer.camera_.view[0] = up_vector[0];
 viewer.camera_.view[1] = up_vector[1];
 viewer.camera_.view[2] = up_vector[2];
 viewer.updateCamera ();
 }

 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 Eigen::Affine3f
 getViewerPose (visualization::PCLVisualizer& viewer)
 {
 Eigen::Affine3f pose = viewer.getViewerPose();
 Eigen::Matrix3f rotation = pose.linear();

 Matrix3f axis_reorder;
 axis_reorder << 0,  0,  1,
 -1,  0,  0,
 0, -1,  0;

 rotation = rotation * axis_reorder;
 pose.linear() = rotation;
 return pose;
 }

 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 template<typename CloudT> void
 writeCloudFile (int format, const CloudT& cloud);


 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 template<typename MergedT, typename PointT>
 typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const PointCloud<RGB>& colors)
 {
 typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());

 pcl::copyPointCloud (points, *merged_ptr);
 for (size_t i = 0; i < colors.size (); ++i)
 merged_ptr->points[i].rgba = colors.points[i].rgba;

 return merged_ptr;
 }


 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 struct CurrentFrameCloudView
 {
 void
 setViewerPose (const Eigen::Affine3f& viewer_pose)
 {
 ::setViewerPose (cloud_viewer_, viewer_pose);
 }

 PointCloud<PointXYZ>::Ptr cloud_ptr_;
 DeviceArray2D<PointXYZ> cloud_device_;
 visualization::PCLVisualizer cloud_viewer_;
 };
 */


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Support functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

pcl::gpu::kinfuLS::PixelRGB*
KinFuLS::convert_sensor_msgs_image_to_gpu_pixels(
        pcl::gpu::kinfuLS::PixelRGB* pixelRgbs,
        const sensor_msgs::ImageConstPtr& rgb)
{
	//convert sensor_msgs::Image to pcl::gpu::PixelRGB
    unsigned pixelCount = rgb->height * rgb->width;
    for (unsigned i = 0; i < pixelCount; i++)
	{
		//the encoding given in the image is "bgr8"
		pixelRgbs[i].b = rgb->data[i * 3];
		pixelRgbs[i].g = rgb->data[i * 3 + 1];
		pixelRgbs[i].r = rgb->data[i * 3 + 2];
	}
	return pixelRgbs;
}


void
KinFuLS::save_snapshots(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::CameraInfoConstPtr& cameraInfo)
{
	if (enable_texture_extraction_)
	{
		if (frame_counter_ % snapshot_rate_ == 0)
		{
		    unsigned pixelCount = rgb->height * rgb->width;
		    pcl::gpu::kinfuLS::PixelRGB* pixelRgbs = new pcl::gpu::kinfuLS::PixelRGB[pixelCount];
			pixelRgbs = convert_sensor_msgs_image_to_gpu_pixels(pixelRgbs, rgb);
			pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24(rgb->height, rgb->width, pixelRgbs, rgb->step);

			/*
			 *      [fx  0 cx]
			 * K = 	[ 0 fy cy]
			 * 		[ 0  0  1]
			 */
			float focal_length = (cameraInfo->K[0] + cameraInfo->K[4]) / 2;
			screenshot_manager_.setCameraIntrinsics(focal_length, cameraInfo->height, cameraInfo->width);
			screenshot_manager_.saveImage(kinfu_->getCameraPose(), rgb24);
			delete[] pixelRgbs;
		}
	}
}


boost::shared_ptr<pcl::PolygonMesh>
convertToMesh(const DeviceArray<PointXYZ>& triangles)
{
  if (triangles.empty())
          return boost::shared_ptr<pcl::PolygonMesh>();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = (int)triangles.size();
  cloud.height = 1;
  triangles.download(cloud.points);

  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() );
  pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

  mesh_ptr->polygons.resize (triangles.size() / 3);
  cout << "  triangles " << triangles.size() << "  polygons " << mesh_ptr->polygons.size() << "  " << flush;
  for (size_t i = 0; i < mesh_ptr->polygons.size(); ++i)
  {
    pcl::Vertices v;
    v.vertices.push_back(i*3+0);
    v.vertices.push_back(i*3+2);
    v.vertices.push_back(i*3+1);
    mesh_ptr->polygons[i] = v;
  }
//  boost::shared_ptr<pcl::PolygonMesh> mesh_simplyfied_ptr( new pcl::PolygonMesh() );
//  pcl::surface::simplify(mesh_ptr, mesh_simplyfied_ptr);

//  return mesh_simplyfied_ptr;
  return mesh_ptr;
}


void
writePolygonMeshFile(int format, const pcl::PolygonMesh& mesh)
{
  if (format == KinFuLS::MESH_PLY)
  {
    cout << "Saving mesh to 'mesh.ply'... " << flush;
    pcl::io::savePLYFile("mesh.ply", mesh, 5);
  }
//  else /* if (format == KinFuLSApp::MESH_VTK) */
//  {
//    cout << "Saving mesh to to 'mesh.vtk'... " << flush;
//    pcl::io::saveVTKFile("mesh.vtk", mesh);
//  }
  cout << "Done" << endl;
}


template<typename CloudPtr> void
writeCloudFile (int format, const CloudPtr& cloud_prt)
{
  if (format == KinFuLS::PCD_BIN)
  {
    cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << flush;
    pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
  }
  else if (format == KinFuLS::PCD_ASCII)
  {
    cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << flush;
    pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
  }
  else   /* if (format == KinFuLS::PLY) */
  {
    cout << "Saving point cloud to 'cloud.ply' (ASCII)... " << flush;
    pcl::io::savePLYFileASCII ("cloud.ply", *cloud_prt);

  }
  cout << "Done" << endl;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr
KinFuLS::transform_point_cloud(const PointCloud<PointXYZ>::Ptr& cloud_ptr_)
{
/*	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	transform.translation() << 0.0, 0.0, 0.0;

	double theta = M_PI / 2.0;
	if (first_rotating_axis == 1)
		transform.rotate(Eigen::AngleAxisf(first_rotating_axis_signal * theta, Eigen::Vector3f::UnitX()));
	else if (first_rotating_axis == 2)
		transform.rotate(Eigen::AngleAxisf(first_rotating_axis_signal * theta, Eigen::Vector3f::UnitY()));
	else if (first_rotating_axis == 3)
		transform.rotate(Eigen::AngleAxisf(first_rotating_axis_signal * theta, Eigen::Vector3f::UnitZ()));

	if (second_rotating_axis == 1)
		transform.rotate(Eigen::AngleAxisf(second_rotating_axis_signal * theta, Eigen::Vector3f::UnitX()));
	else if (second_rotating_axis == 2)
		transform.rotate(Eigen::AngleAxisf(second_rotating_axis_signal * theta, Eigen::Vector3f::UnitY()));
	else if (second_rotating_axis == 3)
		transform.rotate(Eigen::AngleAxisf(second_rotating_axis_signal * theta, Eigen::Vector3f::UnitZ()));

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*cloud_ptr_, *transformed_cloud, transform);

	return (transformed_cloud);
*/
	return (cloud_ptr_);
}


const unsigned char *
convert_32FC1_to_32FC1(const sensor_msgs::ImageConstPtr& depth)
{
	static unsigned short int *data = NULL;

	if (data == NULL)
		data = (unsigned short int *) malloc(depth->height * depth->width * sizeof(unsigned short int));

	float *data_f = (float *) &(depth->data[0]);
	for (int y = 0; y < depth->height; y++)
	{
		for (int x = 0; x < depth->width; x++)
		{
			if (isnan(data_f[y * depth->width + x]))
				data[y * depth->width + x] = 0; // implementar get_average_neighbors();
			else
				data[y * depth->width + x] = (unsigned short int) (1000.0 * data_f[y * depth->width + x]);
		}
	}
	return ((const unsigned char *) data);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Publishers
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
KinFuLS::publish_3D_map_view_from_origin(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth)
{
    //	Generate 3D mesh
//	DeviceArray<PointXYZ> triangles_device = marching_cubes_->run(kinfu.volume(), triangles_buffer_device_);
//	mesh_ptr_ = convertToMesh(triangles_device); // TODO: Free memory allocated in convertToMesh()
//	writePolygonMeshFile(KinFuLS::MESH_PLY, *mesh_ptr_);

    // Downloading TSDF volume from device (not working yet)
//	tsdf_cloud_ptr_->downloadTsdfAndWeightsLocal();
//	tsdf_cloud_ptr_->setHeader(Eigen::Vector3i (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z), tsdf_cloud_ptr_->getSize());
//	tsdf_cloud_ptr_->convertToTsdfCloud(tsdf_cloud_buffer_ptr_);
//	pcl::io::savePCDFile<pcl::PointXYZI>("caco.pcd", *tsdf_cloud_buffer_ptr_, false);
//	tsdf_cloud_ptr_->save("caco.pcd", false);


	//////////////////////////////////////////////////////
	// Compute map message
	//////////////////////////////////////////////////////

    // Downloading cloud from device (working -> do note delete!)
	DeviceArray<PointXYZ> cloud_buffer_device_;
	DeviceArray<PointXYZ> extracted = kinfu.volume().fetchCloud(cloud_buffer_device_);
	PointCloud<PointXYZ> cloud_ptr_;
	extracted.download(cloud_ptr_.points);
	cloud_ptr_.width = (int) cloud_ptr_.points.size ();
	cloud_ptr_.height = 1;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud =	transform_point_cloud(cloud_ptr_);

	pcl::toROSMsg(cloud_ptr_, output_cloud);
	output_cloud.header = depth->header;
	pubmap.publish(output_cloud);
	//writeCloudFile(KinFuLS::PCD_BIN, cloud_ptr_);


/*	//////////////////////////////////////////////////////
	// Compute kinfu scene view (human view) message
	//////////////////////////////////////////////////////

	// Use current sensor pose
	// const Eigen::Affine3f& pose = kinfu.getCameraPose();

	// Use static initial pose
	Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
	Eigen::Vector3f t = volume_size * 0.5f - Vector3f(0, 0, volume_size(2) / 2 * 1.2f);
	Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

	// Compute scene view
	raycaster_ptr_->run(kinfu.volume(), pose, kinfu.getCyclicalBufferStructure());
	raycaster_ptr_->generateSceneView(view_device_);
	int cols;
	view_device_.download(view_host_, cols);
	sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
	sensor_msgs::fillImage((*msg), "rgb8", view_device_.rows(), view_device_.cols(), view_device_.cols() * 3, &view_host_[0]);
	msg->header.frame_id = depth->header.frame_id;
	pubgen.publish(msg);
*/
/*	//////////////////////////////////////////////////////
	// Compute kinfu depth message
	//////////////////////////////////////////////////////

    // Use current sensor pose
	// const Eigen::Affine3f& pose = kinfu.getCameraPose();

    // Use static initial pose
	Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
	Eigen::Vector3f t = volume_size * 0.5f - Vector3f(0, 0, volume_size(2) / 2 * 1.2f);
	Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

	// Compute depth
	raycaster_ptr_->run(kinfu.volume(), pose, kinfu.getCyclicalBufferStructure());
    raycaster_ptr_->generateDepthImage(generated_depth_); // em milimentros
	int c;
    vector<unsigned short> depth_data;
    generated_depth_.download(depth_data, c);
//	generated_depth_ = depth_device_;
//	cout << endl << flush;
//	cout << endl << flush;
//	cout << endl << flush;
//	cout << endl << flush;
    for (int i = 0; i < generated_depth_.rows(); i++)
    {
    	for (int j = 0; j < generated_depth_.cols(); j++)
    	{
    		unsigned short temp = depth_data[i * generated_depth_.cols() + j];
    		depth_data[i * generated_depth_.cols() + j] = temp * 10;
//    		cout << depth_data[i * generated_depth_.cols() + j] << " ";
    	}
//    	cout << endl << flush;
    }
	sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
	sensor_msgs::fillImage((*msg), "mono16", generated_depth_.rows(), generated_depth_.cols(), generated_depth_.cols() * 2, &depth_data[0]);

	msg->header.frame_id = depth->header.frame_id;
	pubgen.publish(msg);
*/
}


void
KinFuLS::publish_3D_map_sensor_view(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth,
		const sensor_msgs::ImageConstPtr& rgb, bool registration, Eigen::Affine3f* pose_ptr)
{
	//    if (pose_ptr)
	//    {
	//        raycaster_ptr_->run ( kinfu.volume (), *pose_ptr, kinfu.getCyclicalBufferStructure () ); //says in cmake it does not know it
	//        raycaster_ptr_->generateSceneView(view_device_);
	//    }
	//    else
	{
		kinfu.getImage(view_device_);
	}

	if (paint_image_ && registration && !pose_ptr)
	{
		//convert sensor_msgs::Image to pcl::gpu::PixelRGB
		unsigned pixelCount = rgb->height * rgb->width;
		pcl::gpu::kinfuLS::PixelRGB* pixelRgbs = new pcl::gpu::kinfuLS::PixelRGB[pixelCount];
		for (unsigned i = 0; i < pixelCount; i++)
		{
			//the encoding given in the image is "bgr8"
			pixelRgbs[i].b = rgb->data[i * 3];
			pixelRgbs[i].g = rgb->data[i * 3 + 1];
			pixelRgbs[i].r = rgb->data[i * 3 + 2];
		}
		pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24(rgb->height, rgb->width, pixelRgbs, rgb->step);
		colors_device_.upload (rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
		paint3DView(colors_device_, view_device_);

	    delete[] pixelRgbs;
	}

	int cols;
	view_device_.download(view_host_, cols);

	//convert image to sensor message

	sensor_msgs::Image msg;
	sensor_msgs::fillImage(msg, "rgb8", view_device_.rows(), view_device_.cols(), view_device_.cols() * 3, &view_host_[0]);

	msg.header.frame_id = depth->header.frame_id;
	pubKinfu.publish(msg);
}


void
KinFuLS::publish_mapping_camera_tf_transform(const sensor_msgs::ImageConstPtr& depth)
{
    if (g_robot_arm == LEFT_ARM)
        odom_broad.sendTransform(tf::StampedTransform(g_mapping_camera_frame_transform, depth->header.stamp,
                "/camera_rgb_optical_frame", "/mapping_camera_frame"));
    else
        odom_broad.sendTransform(tf::StampedTransform(g_mapping_camera_frame_transform, depth->header.stamp,
                "/camera2_rgb_optical_frame", "/mapping_camera_frame"));
}

void
KinFuLS::publish_sensor_pose(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth)
{
	Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats = kinfu.getCameraPose().linear();
	Eigen::Vector3f teVecs = kinfu.getCameraPose().translation();
	// Eigen::Vector3f camera_initial_translation_within_volume = volume_size * 0.5f - Vector3f(0, 0, volume_size(2) / 2 * 1.2f);
	// teVecs = teVecs - camera_initial_translation_within_volume;
	// cout << "x = " << teVecs[0] << "  y = " << teVecs[1] << "  z = " << teVecs[2] << "\n";

	//TODO: start position: init_tcam_ = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

	tf::Transform transform(tf::Matrix3x3(erreMats(0, 0), erreMats(0, 1), erreMats(0, 2), erreMats(1, 0), erreMats(1, 1),
			erreMats(1, 2), erreMats(2, 0), erreMats(2, 1), erreMats(2, 2)), tf::Vector3(teVecs[0], teVecs[1], teVecs[2]));

	g_mapping_camera_frame_transform = transform;
	publish_mapping_camera_tf_transform(depth);

	std::cout << "MAPPING tf ts " << depth->header.stamp << std::endl;
    std::cout.flush();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Handlers
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
KinFuLS::update_3D_map(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
		const sensor_msgs::ImageConstPtr& rgb)
{
	frame_counter_++;

	if (g_mapping_mode == mspl_mode) // mapping simple mode -> publish instantaneous map
	{
		std::cout << "== Mapping in mspl_mode\n";
		kinfu_->reset();
	}
	else if (g_mapping_mode == mstd_mode) // mapping standard -> run mapping normally
	{
		std::cout << "== Mapping in mstd_mode\n";
	}
	else if (g_mapping_mode == mfrz_mode) // mapping freeze mode -> publish last map
	{
		std::cout << "== Mapping in mfrz_mode\n";
		output_cloud.header = depth->header;
		pubmap.publish(output_cloud);
		publish_mapping_camera_tf_transform(depth);
		return;
	}
	else if (g_mapping_mode == mstp_mode) // stop mapping
	{
		std::cout << "== Mapping stopped\n";
		publish_mapping_camera_tf_transform(depth);
		return;
	}

	if (kinfu_->icpIsLost())
	{
		kinfu_->reset();
		// kinfu_->setDisableICP();
	}

	if (depth->encoding == "32FC1")
		depth_device_.upload(convert_32FC1_to_32FC1(depth), depth->step / 2, depth->height, depth->width);
	else
		depth_device_.upload(&(depth->data[0]), depth->step, depth->height, depth->width);

	if (integrate_colors_)
	{
		unsigned pixelCount = rgb->height * rgb->width;
		pcl::gpu::kinfuLS::PixelRGB* pixelRgbs = new pcl::gpu::kinfuLS::PixelRGB[pixelCount];
		pixelRgbs = convert_sensor_msgs_image_to_gpu_pixels(pixelRgbs, rgb);
		pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24(rgb->height, rgb->width, pixelRgbs, rgb->step);
		colors_device_.upload(rgb24.data, rgb24.step, rgb24.rows, rgb24.cols);
		delete[] pixelRgbs;
	}

	SampledScopeTime fps(time_ms_);

	/*
	 *      [fx  0 cx]
	 * K = 	[ 0 fy cy]
	 * 		[ 0  0  1]
	 */
	(*kinfu_).setDepthIntrinsics(cameraInfo->K[0], cameraInfo->K[4], cameraInfo->K[2], cameraInfo->K[5]);
	(*kinfu_)(depth_device_);
	if (kinfu_->isFinished())
		nh.shutdown();

	publish_sensor_pose(*kinfu_, depth);

	if ((frame_counter_ % 2) != 0)
		return;

	if (first_rotating_axis == 1)
	{
		publish_3D_map_sensor_view(*kinfu_, depth, rgb, registration_, NULL);
		publish_3D_map_view_from_origin(*kinfu_, depth);
	}
	else if (first_rotating_axis == 2)
		kinfu_->reset();
//	save_snapshots(rgb, cameraInfo);
}


void
KinFuLS::update_3D_map_camera1(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
    const sensor_msgs::ImageConstPtr& rgb)
{
  if (g_robot_arm == LEFT_ARM)
    update_3D_map(depth, cameraInfo, rgb);
}


void
KinFuLS::update_3D_map_camera2(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
    const sensor_msgs::ImageConstPtr& rgb)
{
  if (g_robot_arm == RIGHT_ARM)
    update_3D_map(depth, cameraInfo, rgb);
}



void
KinFuLS::reconfCallback(prx_mapping::prx_mappingConfig& c, uint32_t level)
{
	if (c.stop)
		kinfu_->performLastScan();

	first_rotating_axis = c.first_rotating_axis;
	first_rotating_axis_signal = c.first_rotating_axis_signal;
	second_rotating_axis = c.second_rotating_axis;
	second_rotating_axis_signal = c.second_rotating_axis_signal;
}


void
decision_making_state_handler(prx_decision_making::DecisionMakingStateMessagePtr system_state)
{
	static int previous_mapping_mode = -1;

	g_mapping_mode = system_state->mapping_mode;
	if (previous_mapping_mode != g_mapping_mode)
	{
		previous_mapping_mode = g_mapping_mode;

		if (g_mapping_mode == mstd_mode)
			ROS_WARN("current mapping mode = mstd_mode (standard), current system state = %d\n", system_state->state);
		else if (g_mapping_mode == mspl_mode)
			ROS_WARN("current mapping mode = mspl_mode (simple), current system state = %d\n", system_state->state);
		else if (g_mapping_mode == mfrz_mode)
			ROS_WARN("current mapping mode = mfrz_mode (freeze), current system state = %d\n", system_state->state);
		else
			ROS_WARN("current mapping mode = %d, current system state = %d\n", g_mapping_mode, system_state->state);
	}

    g_robot_arm = system_state->robot_arm;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Subscription to relevant messages
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
subscribe_to_messages_camera1(std::string depth_topic, std::string depth_info_topic, std::string image_topic,
    KinFuLS *kinfu_3D_mapper, ros::NodeHandle &nh)
{
  if (kinfu_3D_mapper->enable_texture_extraction_)
  {
    typedef sync_policies::ApproximateTime<Image, CameraInfo, Image> DRGBSync;

    // message_filters instead of image_transport because of synchronization over w-lan
    message_filters::Subscriber<Image> *depth_sub = new message_filters::Subscriber<Image>(nh, depth_topic, 5);
    message_filters::Subscriber<CameraInfo> *depth_info_sub = new message_filters::Subscriber<CameraInfo>(nh, depth_info_topic, 5);
    message_filters::Subscriber<Image> *rgb_sub = new message_filters::Subscriber<Image>(nh, image_topic, 5);

    //the depth and the rgb cameras are not hardware synchronized
    //hence the depth and rgb images normally do not have the EXACT timestamp
    //so use approximate time policy for synchronization
    message_filters::Synchronizer<DRGBSync> *texture_sync = new message_filters::Synchronizer<DRGBSync>(DRGBSync(500), *depth_sub, *depth_info_sub, *rgb_sub);
    texture_sync->registerCallback(boost::bind(&KinFuLS::update_3D_map_camera1, kinfu_3D_mapper, _1, _2, _3));

    ROS_WARN("Running KinFu with texture extraction on camera 1");
  }
  else
  {
    //    depth_sub = new message_filters::Subscriber<Image>(nh, "/camera/depth/image_raw", 5);
    //    info_sub = new message_filters::Subscriber<CameraInfo>(nh, "/camera/depth/camera_info", 5);
    //
    //    TimeSynchronizer<Image, CameraInfo> *depth_only_sync = new TimeSynchronizer<Image, CameraInfo>(*depth_sub, *info_sub, 500);
    //
    //    depth_only_sync->registerCallback(boost::bind(&KinFuLS::update_3D_map, kinfu_3D_mapper, _1, _2, sensor_msgs::ImageConstPtr()));
    ROS_WARN("KinFu without texture extraction not supported!!!");
  }
}


void
subscribe_to_messages_camera2(std::string depth_topic, std::string depth_info_topic, std::string image_topic,
    KinFuLS *kinfu_3D_mapper, ros::NodeHandle &nh)
{
  if (kinfu_3D_mapper->enable_texture_extraction_)
  {
    typedef sync_policies::ApproximateTime<Image, CameraInfo, Image> DRGBSync;

    // message_filters instead of image_transport because of synchronization over w-lan
    message_filters::Subscriber<Image> *depth_sub = new message_filters::Subscriber<Image>(nh, depth_topic, 5);
    message_filters::Subscriber<CameraInfo> *depth_info_sub = new message_filters::Subscriber<CameraInfo>(nh, depth_info_topic, 5);
    message_filters::Subscriber<Image> *rgb_sub = new message_filters::Subscriber<Image>(nh, image_topic, 5);

    //the depth and the rgb cameras are not hardware synchronized
    //hence the depth and rgb images normally do not have the EXACT timestamp
    //so use approximate time policy for synchronization
    message_filters::Synchronizer<DRGBSync> *texture_sync = new message_filters::Synchronizer<DRGBSync>(DRGBSync(500), *depth_sub, *depth_info_sub, *rgb_sub);
    texture_sync->registerCallback(boost::bind(&KinFuLS::update_3D_map_camera2, kinfu_3D_mapper, _1, _2, _3));

    ROS_WARN("Running KinFu with texture extraction on camera 2");
  }
  else
  {
    //    depth_sub = new message_filters::Subscriber<Image>(nh, "/camera/depth/image_raw", 5);
    //    info_sub = new message_filters::Subscriber<CameraInfo>(nh, "/camera/depth/camera_info", 5);
    //
    //    TimeSynchronizer<Image, CameraInfo> *depth_only_sync = new TimeSynchronizer<Image, CameraInfo>(*depth_sub, *info_sub, 500);
    //
    //    depth_only_sync->registerCallback(boost::bind(&KinFuLS::update_3D_map, kinfu_3D_mapper, _1, _2, sensor_msgs::ImageConstPtr()));
    ROS_WARN("KinFu without texture extraction not supported!!!");
  }
}


void
kinfu_3D_mapper_subscribe_to_relevant_messages(KinFuLS *kinfu_3D_mapper, ros::NodeHandle &nh)
{
	std::string depth_topic = "/camera/depth_registered/image_raw";
	std::string depth_info_topic = "/camera/depth_registered/camera_info";
	std::string image_topic = "/camera/rgb/image_color";

	subscribe_to_messages_camera1(depth_topic, depth_info_topic, image_topic, kinfu_3D_mapper, nh);

  depth_topic = "/camera2/depth_registered/image_raw";
  depth_info_topic = "/camera2/depth_registered/camera_info";
  image_topic = "/camera2/rgb/image_color";

  subscribe_to_messages_camera2(depth_topic, depth_info_topic, image_topic, kinfu_3D_mapper, nh);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// main() and associated functions
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int
print_help()
{
	cout << "\nprx_mapping parameters:" << endl;
	cout << "    --help, -h                        : print this message" << endl;
	cout << "\nprx_mapping node parameters:" << endl;
	cout << "    volume_size <in_meters>, vs       : define integration volume size" << endl;
	cout << "    shifting_distance <in_meters>, sd : define shifting threshold (distance target-point / cube center)" << endl;
	cout << "    snapshot_rate <X_frames>, sr      : Extract RGB textures every <X_frames>. Default: 45" << endl;
	cout << "    extract_textures, et              : extract RGB PNG images to KinFuSnapshots folder. Default: true" << endl;

	return 0;
}


KinFuLS*
kinfu_3D_mapper_initialization(ros::NodeHandle& nh)
{
	// assign value from parameter server, with default.
	int device;

	nh.param<int>("device", device, 0);

	pcl::gpu::setDevice(device);
	pcl::gpu::printShortCudaDeviceInfo(device);

	double volume_size = 1.5; // pcl::device::VOLUME_SIZE
	nh.getParam("volume_size", volume_size);
	nh.getParam("vs", volume_size);

	double shift_distance = 1.5; // pcl::device::DISTANCE_THRESHOLD;
	nh.getParam("shift_distance", shift_distance);
	nh.getParam("sd", shift_distance);

	// Create kinfu object instance
	static KinFuLS kinfu_3D_mapper(volume_size, shift_distance, nh);
	kinfu_3D_mapper.init_publishers(volume_size);

	// default value (45) is set in constructor of KinFuLSApp
	nh.getParam("snapshot_rate", kinfu_3D_mapper.snapshot_rate_);
	nh.getParam("sr", kinfu_3D_mapper.snapshot_rate_);

	// default value (false) is set in constructor of KinFuLS
	nh.getParam("extract_textures", kinfu_3D_mapper.enable_texture_extraction_);
	nh.getParam("et", kinfu_3D_mapper.enable_texture_extraction_);

	// Init global tf tranform
	g_mapping_camera_frame_transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	g_mapping_camera_frame_transform.setRotation(tf::Quaternion(1.0, 0.0, 0.0, 0.0));

	return &kinfu_3D_mapper;
}


int
main(int argc, char* argv[])
{
	// help line parameter evaluation
	if (pc::find_switch(argc, argv, "--help") || pc::find_switch(argc, argv, "-h"))
		return print_help();

	// ROS initialization and remaining line parameters evaluation
	ros::init(argc, argv, "prx_mapping");
	ros::NodeHandle nh("~");

	// kinfu_3D_mapper initialization
	KinFuLS *kinfu_3D_mapper = kinfu_3D_mapper_initialization(nh);
	kinfu_3D_mapper_subscribe_to_relevant_messages(kinfu_3D_mapper, nh);
	ros::Subscriber decision_making_sub = nh.subscribe("/decision_making_state", 1, decision_making_state_handler);
	g_mapping_mode = mstd_mode;
	g_robot_arm = LEFT_ARM;

	// reconfiguration server setup
	dynamic_reconfigure::Server<prx_mapping::prx_mappingConfig> reconfServer(nh);
	reconfServer.setCallback(boost::bind(&KinFuLS::reconfCallback, kinfu_3D_mapper, _1, _2));

	// ROS main loop
	ros::Rate loop_rate(40);
	while (nh.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
