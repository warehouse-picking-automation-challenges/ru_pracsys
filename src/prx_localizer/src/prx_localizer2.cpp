#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <baxter_core_msgs/EndpointState.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <dynamic_reconfigure/server.h>
#include <prx_localizer/prx_localizerConfig.h>
#include <prx_decision_making/DecisionMakingStateMessage.h>
#include <prx_decision_making/decision_making.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>

#define BAXTER  0
#define MOTOMAN 1


double left_arm_x,left_arm_y,left_arm_z;
double left_arm_r,left_arm_p,left_arm_yaw;
double right_arm_x,right_arm_y,right_arm_z;
double right_arm_r,right_arm_p,right_arm_yaw;


using namespace std;
using namespace Eigen;
using namespace tf;
using namespace baxter_core_msgs;


tf::TransformListener *g_listener;

int g_localize_mode = lstd_mode;
tf::Transform g_localize_transform;
ros::Time g_localize_transform_timestamp;

int g_robot_arm = LEFT_ARM;

int g_robot = BAXTER;
std::string g_base_reference_frame;


tf::Transform
compute_localize_transform(tf::Transform t1, tf::Transform t2)
{
	tf::Transform transform;
	Eigen::Matrix4f mt1;
	Eigen::Matrix4f mt2;
	Eigen::Matrix4f mtransform;

	pcl_ros::transformAsMatrix(t1, mt1);
	pcl_ros::transformAsMatrix(t2, mt2);
	mtransform = mt1 * mt2;

	Eigen::Matrix4d md(mtransform.cast<double>());
	Eigen::Affine3d affine(md);
	tf::transformEigenToTF(affine, transform);

	return (transform.inverse());
}

tf::StampedTransform
compute_kinect_pose_transform_robot_arm(double arm_x,double arm_y, double arm_z, double arm_r, double arm_p, double arm_yaw)
{
    tf::StampedTransform tf_transform;
    tf_transform.setOrigin(tf::Vector3(arm_x, arm_y, arm_z));

    tf::Quaternion q;
    q.setRPY(arm_r, arm_p ,arm_yaw);
    tf_transform.setRotation(q);

    return (tf_transform);
}



tf::StampedTransform
compute_kinect_pose_transform()
{
    if (g_robot == BAXTER)
    {
        if (g_robot_arm == LEFT_ARM)
            return compute_kinect_pose_transform_robot_arm(left_arm_x,
                            left_arm_y,
                            left_arm_z,
                            left_arm_r,
                            left_arm_p,
                            left_arm_yaw);
        else
            return compute_kinect_pose_transform_robot_arm(right_arm_x,
                            right_arm_y,
                            right_arm_z,
                            right_arm_r,
                            right_arm_p,
                            right_arm_yaw);
    }
    else
    {
        if (g_robot_arm == LEFT_ARM)
            return compute_kinect_pose_transform_robot_arm(left_arm_x,
                            left_arm_y,
                            left_arm_z,
                            left_arm_r,
                            left_arm_p,
                            left_arm_yaw);
        else
            return compute_kinect_pose_transform_robot_arm(right_arm_x,
                            right_arm_y,
                            right_arm_z,
                            right_arm_r,
                            right_arm_p,
                            right_arm_yaw);
    }
}

void
save_localize_transform()
{
    std::ofstream filed_transform;
	std::stringstream file_name;

	std::string path = ros::package::getPath("prx_localizer");
	file_name << path << "/transform/localize_transform.bin";
    filed_transform.open(file_name.str().c_str(), std::ofstream::binary|std::ofstream::trunc);

    tfScalar x, y, z, w;
    x = g_localize_transform.getRotation().getX();
    y = g_localize_transform.getRotation().getY();
    z = g_localize_transform.getRotation().getZ();
    w = g_localize_transform.getRotation().getW();
    filed_transform.write((char *) &x, sizeof(tfScalar));
    filed_transform.write((char *) &y, sizeof(tfScalar));
    filed_transform.write((char *) &z, sizeof(tfScalar));
    filed_transform.write((char *) &w, sizeof(tfScalar));

    x = g_localize_transform.getOrigin().getX();
    y = g_localize_transform.getOrigin().getY();
    z = g_localize_transform.getOrigin().getZ();
    filed_transform.write((char *) &x, sizeof(tfScalar));
    filed_transform.write((char *) &y, sizeof(tfScalar));
    filed_transform.write((char *) &z, sizeof(tfScalar));

    filed_transform.close();
}


void
load_localize_transform()
{
	std::ifstream filed_transform;
	std::stringstream file_name;

	std::string path = ros::package::getPath("prx_localizer");
	file_name << path << "/transform/localize_transform.bin";
    filed_transform.open(file_name.str().c_str(), std::ofstream::binary);
    if (filed_transform.is_open())
    {
		tfScalar x, y, z, w;
		filed_transform.read((char *) &x, sizeof(tfScalar));
		filed_transform.read((char *) &y, sizeof(tfScalar));
		filed_transform.read((char *) &z, sizeof(tfScalar));
		filed_transform.read((char *) &w, sizeof(tfScalar));
		g_localize_transform.setRotation(tf::Quaternion(x, y, z, w));

		filed_transform.read((char *) &x, sizeof(tfScalar));
		filed_transform.read((char *) &y, sizeof(tfScalar));
		filed_transform.read((char *) &z, sizeof(tfScalar));
		g_localize_transform.setOrigin(tf::Vector3(x, y, z));

		filed_transform.close();
    }
    else
    	g_localize_transform = tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.0));

    g_localize_transform_timestamp = ros::Time(0);
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Publishers
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
publish_localize_transform(const sensor_msgs::ImageConstPtr& depth, std::string camera_link_tf_topic)
{
	static tf::TransformBroadcaster tb1;
	tf::StampedTransform camera_link_to_mapping_camera_transform;
	tf::StampedTransform kinect_pose_in_the_arm_transform;
	bool got_transform1 = false;
	bool got_transform2 = false;
	static bool got_both_transform = false;
	bool got_all_required_transforms = false;

	if (g_listener->canTransform(camera_link_tf_topic, "/mapping_camera_frame", ros::Time(0)))
	{
		g_listener->lookupTransform(camera_link_tf_topic, "/mapping_camera_frame",
				ros::Time(0), camera_link_to_mapping_camera_transform);

		got_transform1 = true;
		cout << "got transform 1" << endl;
	}

	if (g_listener->canTransform("/kinect_pose_in_the_arm", g_base_reference_frame, ros::Time(0)))
	{
		g_listener->lookupTransform("/kinect_pose_in_the_arm", g_base_reference_frame,
				ros::Time(0), kinect_pose_in_the_arm_transform);

		got_transform2 = true;
		cout << "got transform 2" << endl;
	}

	if (got_transform1 && got_transform2)
	{
		if (!got_both_transform)
		{
			got_both_transform = true;
			cout << "got both transforms" << endl;
		}
		if (g_localize_mode == lspl_mode)
		{
			g_localize_transform = compute_localize_transform(camera_link_to_mapping_camera_transform,
			        kinect_pose_in_the_arm_transform);
			g_localize_transform_timestamp = camera_link_to_mapping_camera_transform.stamp_;
			got_all_required_transforms = true;
		}
	}

	if (got_all_required_transforms)
	{
	    if (camera_link_to_mapping_camera_transform.stamp_ != ros::Time(0))
	    {
            cout << "HAVE " << camera_link_tf_topic.c_str() << " -> /mapping_camera_frame transform" << endl;
            save_localize_transform();
	    }
	    else
	    {
	        g_localize_transform_timestamp = depth->header.stamp;
	        cout << "DO NOT HAVE1 " << camera_link_tf_topic.c_str() << " -> /mapping_camera_frame transform" << endl;
	    }
	}
	else
	{
        g_localize_transform_timestamp = depth->header.stamp;
		cout << "DO NOT HAVE2 " << camera_link_tf_topic.c_str() << " -> /mapping_camera_frame transform" << endl;
	}

    std::cout << "WORLD tf ts " << g_localize_transform_timestamp << std::endl;
    std::cout.flush();
    tb1.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0.0, 0.0, 0.0)),
        g_localize_transform_timestamp, "/world", g_base_reference_frame));
//	tb1.sendTransform(tf::StampedTransform(g_localize_transform.inverse(),
//			g_localize_transform_timestamp, "/world", g_base_reference_frame));
    tb1.sendTransform(tf::StampedTransform(g_localize_transform, g_localize_transform_timestamp,
            g_base_reference_frame, camera_link_tf_topic));
}


void
publish_kinect_pose_in_the_arm_transform(std::string arm_joint_tf_topic)
{
	static tf::TransformBroadcaster tb2;
	tf::StampedTransform transform;
	ros::Time timestamp;

	if (g_listener->canTransform(g_base_reference_frame, arm_joint_tf_topic, ros::Time(0)))
	{
	    g_listener->lookupTransform(g_base_reference_frame, arm_joint_tf_topic, ros::Time(0), transform);
	    timestamp = transform.stamp_;
		transform = compute_kinect_pose_transform();
		tb2.sendTransform(tf::StampedTransform(transform, timestamp, arm_joint_tf_topic, "/kinect_pose_in_the_arm"));
	}
	else
        cout << "ATTENTION: " << arm_joint_tf_topic.c_str() << " -> kinect_pose_in_the_arm transform missing" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Handlers
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
kinect_depth_camera1_handler(const sensor_msgs::ImageConstPtr &depth)
{
    if (g_robot_arm == LEFT_ARM)
    {
        publish_localize_transform(depth, "/camera_link");
        if (g_robot == BAXTER)
            publish_kinect_pose_in_the_arm_transform("/left_lower_forearm");
        else // MOTOMAN
            publish_kinect_pose_in_the_arm_transform("/arm_left_link_6_b");
    }
}


void
kinect_depth_camera2_handler(const sensor_msgs::ImageConstPtr &depth)
{
    if (g_robot_arm == RIGHT_ARM)
    {
        publish_localize_transform(depth, "/camera2_link");
        if (g_robot == BAXTER)
            publish_kinect_pose_in_the_arm_transform("/left_lower_forearm");
        else // MOTOMAN
            publish_kinect_pose_in_the_arm_transform("/arm_right_link_6_b");
    }
}


void
dynamic_reconfiguration_handler(prx_localizer::prx_localizerConfig& c, uint32_t level)
{
    std::cout << "======== localizer dynamic_reconfiguration_handler() called." << std::endl;

    double conversion = M_PI / 180.0;

    left_arm_x = c.left_arm_x;
    left_arm_y = c.left_arm_y;
    left_arm_z = c.left_arm_z;
    left_arm_r = c.left_arm_r*conversion;
    left_arm_p = c.left_arm_p*conversion;
    left_arm_yaw = c.left_arm_yaw*conversion;

    right_arm_x = c.right_arm_x;
    right_arm_y = c.right_arm_y;
    right_arm_z = c.right_arm_z;
    right_arm_r = c.right_arm_r*conversion;
    right_arm_p = c.right_arm_p*conversion;
    right_arm_yaw = c.right_arm_yaw*conversion;

}


void
decision_making_state_handler(prx_decision_making::DecisionMakingStateMessagePtr system_state)
{
	static int previous_localizer_mode = -1;

	g_localize_mode = system_state->localization_mode;
	if (previous_localizer_mode != g_localize_mode)
	{
		previous_localizer_mode = g_localize_mode;

		if (g_localize_mode == lstd_mode)
			ROS_WARN("current localize mode = lstd_mode (standard -> not implemented yet), current system state = %d\n", system_state->state);
		else if (g_localize_mode == lspl_mode)
			ROS_WARN("current localize mode = lspl_mode (simple), current system state = %d\n", system_state->state);
		else
			ROS_WARN("current localize mode = %d, current system state = %d\n", g_localize_mode, system_state->state);
	}

	g_robot_arm = system_state->robot_arm;
}


void
get_robot_name()
{
    std::string robot_name;
    if (ros::param::get("/robot_name", robot_name))
    {
        if (robot_name == "baxter")
            g_robot = BAXTER;
        else if (robot_name == "motoman")
            g_robot = MOTOMAN;
        else
        {
            std::cout << "Error: unkown robot named " << robot_name.c_str() << std::endl;
            exit(1);
        }
        std::cout << "robot name: " << robot_name.c_str() << std::endl;
    }
    else
    {
        std::cout << "No robot name specified. So, we will use baxter configuration." << std::endl;
        g_robot = BAXTER;
    }
}


int
main(int argc, char** argv)
{
	ros::init(argc, argv, "prx_localizer");
	ros::NodeHandle nh;

    get_robot_name();
    if (g_robot == BAXTER)
        g_base_reference_frame = "/base";
    else // Motoman
        g_base_reference_frame = "/base_link";

    ros::Subscriber kinect_depth_camera_sub1 = nh.subscribe("/camera/depth_registered/image_raw", 1, &kinect_depth_camera1_handler);
    ros::Subscriber kinect_depth_camera_sub2 = nh.subscribe("/camera2/depth_registered/image_raw", 1, &kinect_depth_camera2_handler);
	ros::Subscriber decision_making_sub = nh.subscribe("/decision_making_state", 1, decision_making_state_handler);

	dynamic_reconfigure::Server<prx_localizer::prx_localizerConfig> server;
	dynamic_reconfigure::Server<prx_localizer::prx_localizerConfig>::CallbackType f;
	f = boost::bind(&dynamic_reconfiguration_handler, _1, _2);
	server.setCallback(f);

	tf::TransformListener listener;
	g_listener = &listener;
	load_localize_transform();

	ros::spin();

	return (0);
}
