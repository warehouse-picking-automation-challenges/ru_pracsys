/*
 * kinfuLS.h
 *
 *  Created on: Oct 29, 2014
 *      Author: alberto
 */

#ifndef KINFULS_H_
#define KINFULS_H_


struct KinFuLS
{
	bool exit_;
	bool scan_;
	bool scan_mesh_;
	bool scan_volume_;

	bool independent_camera_;
	int frame_counter_;
	bool enable_texture_extraction_;

	bool registration_;
	bool integrate_colors_;
	bool pcd_source_;
	float focal_length_;

	KinfuTracker *kinfu_;

	// Publishers
	KinfuTracker::View view_device_;

	KinfuTracker::DepthMap depth_device_;
	KinfuTracker::View colors_device_;
	bool paint_image_;

//	pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

	boost::mutex data_ready_mutex_;
	boost::condition_variable data_ready_cond_;

	std::vector<pcl::gpu::kinfuLS::PixelRGB> source_image_data_;
	std::vector<unsigned short> source_depth_data_;
	PtrStepSz<const unsigned short> depth_;
	PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24_;

	int time_ms_;
	vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;
	KinfuTracker::DepthMap generated_depth_;
	size_t lastNumberOfPoses;
	ros::Publisher pubKinfuReset;

	pcl::kinfuLS::ScreenshotManager screenshot_manager_;
	int snapshot_rate_;

	//the ros node handle used to shut down the node and stop capturing
	ros::NodeHandle & nh;

	MarchingCubes::Ptr marching_cubes_;
	boost::shared_ptr<pcl::PolygonMesh> mesh_ptr_;
	DeviceArray<PointXYZ> triangles_buffer_device_;

	bool accumulate_views_;

	RayCaster::Ptr raycaster_ptr_;

	TsdfVolume::Ptr tsdf_cloud_ptr_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_buffer_ptr_;

	image_transport::Publisher pubKinfu;
	image_transport::Publisher pubgen;
	tf::TransformBroadcaster odom_broad;
	ros::Publisher pubmap;
	sensor_msgs::PointCloud2 output_cloud;

	Eigen::Vector3f volume_size;

	int first_rotating_axis;
	double first_rotating_axis_signal;
	int second_rotating_axis;
	double second_rotating_axis_signal;

	void init_publishers(float vsz)
	{
		volume_size = Vector3f::Constant(vsz/*meters*/);
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);
		pubKinfu = it.advertise("/camera/kinfuLS/depth", 10);
		pubgen = it.advertise("/camera/kinfuLS/generated_depth", 50);
		paint_image_ = true;

		std::string topic = nh.resolveName("point_cloud");
		uint32_t queue_size = 1;
		pubmap = nh.advertise<sensor_msgs::PointCloud2> (topic, queue_size);
//		pubmap = nh.advertise<sensor_msgs::PointCloud2>(topic, queue_size);
//		pubmap = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (topic, queue_size); 


//		raycaster_ptr_ = RayCaster::Ptr( new RayCaster() );
//    		marching_cubes_ = MarchingCubes::Ptr( new MarchingCubes() );
//		tsdf_cloud_ptr_ = TsdfVolume::Ptr( new TsdfVolume(Eigen::Vector3i (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z)) );
//		tsdf_cloud_buffer_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
	}

	void publish_3D_map_view_from_origin(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth);

	void publishGeneratedDepth_old(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth);

	void publish_sensor_pose(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth);

	enum
	{
		PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8
	};


	KinFuLS(float vsz, float shiftDistance, ros::NodeHandle & nodeHandle) :
			exit_(false), scan_(false), scan_mesh_(false), scan_volume_(false),
			independent_camera_(false), registration_(true), integrate_colors_(true),
			pcd_source_(false), focal_length_(-1.f), time_ms_(0), nh(nodeHandle)
	{
		//Init Kinfu Tracker
		Eigen::Vector3f volume_size = Vector3f::Constant(vsz/*meters*/);

		ROS_INFO("--- CURRENT SETTINGS ---\n");
		ROS_INFO("Volume size is set to %.2f meters\n", vsz);
		ROS_INFO("Volume will shift when the camera target point is farther than %.2f meters from the volume center\n",
				shiftDistance);
		ROS_INFO("The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6 * vsz);
		ROS_INFO("------------------------\n");

		// warning message if shifting distance is abnormally big compared to volume size
		if (shiftDistance > 2.5 * vsz)
			ROS_WARN("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n",
					shiftDistance, vsz);

		kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shiftDistance);//, 240, 320);

		Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
		Eigen::Vector3f t = volume_size * 0.5f - Vector3f(0, 0, volume_size(2) / 2 * 1.2f);

		Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

		kinfu_->setInitialCameraPose(pose);
		kinfu_->volume().setTsdfTruncDist(0.030f/*meters*/);
		kinfu_->setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.f)));
		//kinfu_->setDepthTruncationForICP(3.f/*meters*/);
		kinfu_->setCameraMovementThreshold(0.001f);

		//Init KinFuLSApp
//		tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
		//publishers.raycaster_ptr_ = RayCaster::Ptr(new RayCaster(kinfu_->rows(), kinfu_->cols()));

		//		//TODO: use camera info msg
		//		float height = 480.0f;
		//		float width = 640.0f;
		//		//taken from gpu/kinfu_large_scale/src/internal.h
		//		//temporary constant (until we make it automatic) that holds the Kinect's focal length
		//		const float FOCAL_LENGTH = 575.816f;
		//		screenshot_manager_.setCameraIntrinsics(FOCAL_LENGTH, height, width);

		frame_counter_ = 0;
		enable_texture_extraction_ = true;
		lastNumberOfPoses = 0;
		snapshot_rate_ = 45;

		ros::NodeHandle nh;
		pubKinfuReset = nh.advertise<std_msgs::Empty>("/kinfu_reset", 2);

		first_rotating_axis = 1; // 2; // 0 = none, 1 = x, 2 = y, 3 = z
		first_rotating_axis_signal = 0.0; // 1.0;
		second_rotating_axis = 2; // 0 = none, 1 = x, 2 = y, 3 = z
		second_rotating_axis_signal = 0.0; //1.0;

		kinfu_->reset(); // Necessary for setInitialCameraPose to take effect.
	}


	~KinFuLS()
	{
	}

	void publish_3D_map_sensor_view(KinfuTracker& kinfu, const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::ImageConstPtr& rgb, bool registration, Eigen::Affine3f* pose_ptr = 0);

  void update_3D_map(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
      const sensor_msgs::ImageConstPtr& rgb = sensor_msgs::ImageConstPtr());

  void update_3D_map_camera1(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
      const sensor_msgs::ImageConstPtr& rgb = sensor_msgs::ImageConstPtr());

  void update_3D_map_camera2(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& cameraInfo,
      const sensor_msgs::ImageConstPtr& rgb = sensor_msgs::ImageConstPtr());

	void save_snapshots(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::CameraInfoConstPtr& cameraInfo);

	void reconfCallback(prx_mapping::prx_mappingConfig & c, uint32_t level);

private:
	pcl::gpu::kinfuLS::PixelRGB* convert_sensor_msgs_image_to_gpu_pixels(
	        pcl::gpu::kinfuLS::PixelRGB* pixelRgbs,
	        const sensor_msgs::ImageConstPtr& rgb);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transform_point_cloud(
			const PointCloud<PointXYZ>::Ptr& cloud_ptr_);
	void publish_mapping_camera_tf_transform(const sensor_msgs::ImageConstPtr& depth);
};


struct SampledScopeTime: public StopWatch
{
	enum
	{
		EACH = 33
	};
	SampledScopeTime(int& time_ms) :
			time_ms_(time_ms)
	{
	}
	~SampledScopeTime()
	{
		static int i_ = 0;
		time_ms_ += getTime();
		if (i_ % EACH == 0 && i_)
		{
			cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps)" << endl;
			time_ms_ = 0;
		}
		++i_;
	}
private:
	int& time_ms_;
};

#endif /* KINFULS_H_ */
