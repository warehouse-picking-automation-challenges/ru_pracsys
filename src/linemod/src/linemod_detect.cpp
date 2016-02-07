    /*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
#define USE_GLUT 1

#include <ecto/ecto.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <time.h>
#include <iomanip>

#include <boost/foreach.hpp>

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/rgbd/rgbd.hpp>
 // Colin
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <object_recognition_core/db/ModelReader.h>
#include <object_recognition_core/common/pose_result.h>

#include "db_linemod.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <prx_decision_making/DecisionMakingStateMessage.h>
#include <prx_decision_making/decision_making.h>

#include <object_recognition_renderer/utils.h>
#include <object_recognition_renderer/renderer3d.h>

#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <stdio.h>
#include <iostream>
#include <ros/package.h>

#include "linemod_icp.h"
#include "linemod_pointcloud.h"

#include <object_recognition_core/db/prototypes/object_info.h>

// #define PROCESS_DATA_FROM_FILES
// #define BUILD_DATABASE_FOR_OBJECT_DETECTION_PERFORMANCE_EVALUATION
// #define BUILD_DATABASE_FOR_OBJECT_DETECTION_PERFORMANCE_REEVALUATION

#define BAXTER  0
#define MOTOMAN 1


using ecto::tendrils;
using ecto::spore;
using object_recognition_core::db::ObjectId;
using object_recognition_core::common::PoseResult;
using object_recognition_core::db::ObjectDbPtr;

using namespace cv;

#define LINEMOD_VIZ_IMG 1
#if LINEMOD_VIZ_IMG
  #include <opencv2/highgui/highgui.hpp>
#endif

// If you want to set LINEMOD_VIZ_PCD to 1, you have to change the line
//   if(LINEMOD_VIZ_PCD)
// in src/CMakeLists.txt to
// if(1)# LINEMOD_VIZ_PCD)
#define LINEMOD_VIZ_PCD 1
#if LINEMOD_VIZ_PCD
  #include "ros/ros.h"
  #include "linemod_pointcloud.h"
  // ros::NodeHandle node_;
  LinemodPointcloud *pci_real_icpin_model2;
  LinemodPointcloud *pci_real_icpin_model;
  LinemodPointcloud *pci_real_icpin_ref;
#endif


int g_object_pose_recognition_mode;
std::string g_current_object_of_interest;
std::string g_previous_object_of_interest;
int g_bin_id;
int g_item_counts;
bool g_finished_loading_at_least_one_model = false;
int g_robot_arm = LEFT_ARM;
int g_decision_making_state = -1;
int g_frames_to_capture = -1;

#define NUM_FRAMES_PER_CAPTURE 3
enum OBJECT_DETECTION_STATE
{
    od_reset = 1,
    od_capture_1 = 2,
    od_capture_2 = 3,
    od_capture_3 = 4,
    od_idle = 5
};
int g_object_detection_state = od_reset;
tf::Transform g_camera_pose;
string g_base_link_name;

#define S_VEC_SIZE  1000
#define R_MIN       -CV_PI
#define R_MAX        CV_PI
//#define PX_MIN       0.80
//#define PX_MAX       1.40
//#define PY_MIN      -0.50
//#define PY_MAX       0.50
//#define PZ_MIN       0.50
//#define PZ_MAX       1.75

#define PX_MIN      -1.50
#define PX_MAX       1.50
#define PY_MIN      -1.50
#define PY_MAX       1.50
#define PZ_MIN      -1.50
#define PZ_MAX       1.75

double g_pose_statistics[7][S_VEC_SIZE];
struct pose_vector
{
    tf::Transform pose;
    int template_id;
};
std::vector<struct pose_vector> g_pose_vector;


int g_robot = BAXTER;
std::string g_base_reference_frame;

bool g_data_gathering = false;
bool g_ground_truth_generation = false;
bool g_object_detection_evaluation = false;
FILE *g_object_detection_evaluation_results_file;

struct data_gathered_st
{
    char object_name[1000];
    char bin_id;
    int item_counts, state, sample_number;
};

struct ObjectData
{
    cv::Vec3f object_translation_wrt_camera;
    cv::Mat object_rotation_wrt_camera;
    cv::Vec3f object_translation_wrt_base;
    cv::Mat object_rotation_wrt_base;
    cv::Vec3f camera_translation_wrt_base;
    cv::Mat camera_rotation_wrt_base;
    int template_id;
};


void
get_robot_name()
{
    std::string robot_name;
    if (ros::param::get("/robot_name", robot_name))
    {
        if (robot_name == "baxter")
        {
            g_robot = BAXTER;
            g_base_link_name = "base";
        }
        else if (robot_name == "motoman")
        {
            g_robot = MOTOMAN;
            g_base_link_name = "base_link";
        }
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
        g_base_link_name = "base";
    }
}


void
decision_making_state_handler(prx_decision_making::DecisionMakingStateMessagePtr system_state)
{
    g_current_object_of_interest = system_state->object_name;
    g_object_pose_recognition_mode = system_state->object_pose_recognition_mode;
    g_bin_id = system_state->bin_id;
    g_item_counts = system_state->item_counts;
    g_robot_arm = system_state->robot_arm;
    g_decision_making_state = system_state->state;
}


void
drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities, cv::Mat& dst, cv::Point offset, int T)
{
    static const cv::Scalar COLORS[5] =
    { CV_RGB(0, 0, 255), CV_RGB(0, 255, 0), CV_RGB(255, 255, 0), CV_RGB(255, 140, 0), CV_RGB(255, 0, 0) };
    if (dst.channels() == 1)
        cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);

    cv::circle(dst, cv::Point(offset.x, offset.y), T / 2, COLORS[4]);
    if (num_modalities > 5)
        num_modalities = 5;
    for (int m = 0; m < num_modalities; ++m)
    {
// NOTE: Original demo recalculated max response for each feature in the TxT
// box around it and chose the display color based on that response. Here
// the display color just depends on the modality.
        cv::Scalar color = COLORS[m];

        for (int i = 0; i < (int) templates[m].features.size(); ++i)
        {
            cv::linemod::Feature f = templates[m].features[i];
            double angle = 45 * f.label;
            if (m == 0) // color has only orientation, no direction
            	angle /= 2.0;
            double length = 8.0;

            cv::Point p1 = cv::Point(f.x + offset.x, f.y + offset.y);
            cv::Point p2 = cv::Point((int) ((double) p1.x + length * cos(angle * CV_PI / 180.0) + 0.5),
            						 (int) ((double) p1.y + length * sin(angle * CV_PI / 180.0) + 0.5));
            cv::line(dst, p1, p2, COLORS[m], 1);
            cv::circle(dst, p1, 1, COLORS[m]);
        }
    }
}


void
get_data_gathering_and_ground_truth_generation_flags()
{
    if (ros::param::get("/data_gathering", g_data_gathering))
    {
        std::cout << "data_gathering: " << std::boolalpha << g_data_gathering << std::endl;
    }
    else
    {
        std::cout << "No data_gathering flag specified. Assuming false." << std::endl;
        g_data_gathering = false;
    }

    if (ros::param::get("/ground_truth_generation", g_ground_truth_generation))
    {
        std::cout << "ground_truth_generation: " << std::boolalpha << g_ground_truth_generation << std::endl;
    }
    else
    {
        std::cout << "No ground_truth_generation flag specified. Assuming false." << std::endl;
        g_ground_truth_generation = false;
    }

    if (ros::param::get("/object_detection_evaluation", g_object_detection_evaluation))
    {
        std::cout << "object_detection_evaluation: " << std::boolalpha << g_object_detection_evaluation << std::endl;
    }
    else
    {
        std::cout << "No object_detection_evaluation flag specified. Assuming false." << std::endl;
        g_object_detection_evaluation = false;
    }
}


namespace ecto_linemod
{
    struct Detector: public object_recognition_core::db::bases::ModelReaderBase
    {
        std::string
        get_object_name_by_object_id(std::string object_id)
        {
            or_json::mObject fields;
            db_->load_fields(object_id, fields);

            std::string name;
            if (fields.find("object_name") != fields.end())
              name = fields.find("object_name")->second.get_str();

            if (name.empty())
                return ("OBJECT_NOT_IN_THE_DATABASE");
            else
                return (name);
        }


        std::string
        get_object_id_by_object_name(std::string object_name)
        {
            std::string object_id;
            BOOST_FOREACH(const object_recognition_core::db::Document & document, documents_)
            {
                object_id = document.get_field<ObjectId>("object_id");
                //std::cout << "Object id in main loading templates loop: " << object_id.c_str() << std::endl;
                std::string db_object_name = get_object_name_by_object_id(object_id);
                if (db_object_name == object_name)
                    break;
            }
            return (object_id);
        }


        void
        parameter_callback(const object_recognition_core::db::Documents & db_documents)
        {
            if (!(*use_rgb_) && !(*use_depth_))
                throw std::runtime_error(
                        "Unsupported type of input data: either use_rgb or use_depth (or both) parameters shouled be true");
            if (!(*use_rgb_) && *use_depth_)
                std::cout << "WARNING:: Gradients computation will be based on depth data (but not rgb image)." << std::endl;

            detector_ = cv::linemod::getDefaultLINEMOD();
            std::string last_object_id;
            BOOST_FOREACH(const object_recognition_core::db::Document & document, db_documents)
            {
                std::string object_id = document.get_field<ObjectId>("object_id");
                std::cout << "Object id in main loading templates loop: " << object_id.c_str() << std::endl;
                std::string method = document.get_field<std::string>("method");
                std::cout << "Method: " << method.c_str() << std::endl;
                if (method != "LINEMOD")
                {
                	std::cout << "Method != LINEMOD. Skipping" << std::endl;
                	continue;
                }

                // Load the detector for that class
                cv::linemod::Detector detector;
                document.get_attachment<cv::linemod::Detector>("detector", detector);
                if (detector.classIds().empty())
                    continue;
                cv::Ptr<cv::linemod::Detector> detector__ = cv::linemod::getDefaultLINEMOD();

                std::string path = ros::package::getPath("object_recognition_linemod");
                std::stringstream file_name;
                file_name << path << "/conf/linemod_config.yml";
                std::cout << "==== Reading configuration file: " << file_name.str().c_str() << std::endl;
    			cv::FileStorage desired_config(file_name.str().c_str(), cv::FileStorage::READ);
    			if (!desired_config.isOpened())
    			{
    				std::cout << "Error: could not open " << file_name.str().c_str() << " in parameter_callback()" << std::endl;
    				exit(1);
    			}
    			cv::FileNode fn;
                fn = desired_config["weak_threshold"];
                detector__->read(fn);
                fn = desired_config["strong_threshold"];
                detector__->read(fn);
                fn = desired_config["distance_threshold"];
                detector__->read(fn);
                fn = desired_config["difference_threshold"];
                detector__->read(fn);
                fn = desired_config["extract_threshold"];
                detector__->read(fn);
    			desired_config.release();
                std::string object_id_in_db = detector.classIds()[0];
                for (size_t template_id = 0; template_id < detector.numTemplates(); ++template_id)
                {
                    const std::vector<cv::linemod::Template> &templates_original = detector.getTemplates(object_id_in_db, template_id);
                    detector__->addSyntheticTemplate(templates_original, object_id);
                }

                // Deal with the poses
                document.get_attachment<std::vector<cv::Mat> >("Rs", Rs_[object_id]);
                document.get_attachment<std::vector<cv::Mat> >("Ts", Ts_[object_id]);
                document.get_attachment<std::vector<float> >("distances", distances_[object_id]);
                document.get_attachment<std::vector<cv::Mat> >("Ks", Ks_[object_id]);
                renderer_n_points_ = document.get_field<int>("renderer_n_points");
                renderer_angle_step_ = document.get_field<int>("renderer_angle_step");
                renderer_radius_min_ = document.get_field<double>("renderer_radius_min");
                renderer_radius_max_ = document.get_field<double>("renderer_radius_max");
                renderer_radius_step_ = document.get_field<double>("renderer_radius_step");
                renderer_width_ = document.get_field<int>("renderer_width");
                renderer_height_ = document.get_field<int>("renderer_height");
                renderer_focal_length_x_ = document.get_field<double>("renderer_focal_length_x");
                renderer_focal_length_y_ = document.get_field<double>("renderer_focal_length_y");
                renderer_near_ = document.get_field<double>("renderer_near");
                renderer_far_ = document.get_field<double>("renderer_far");

                setupRenderer(object_id);
                std::string object_name = get_object_name_by_object_id(object_id);

                detectors_.insert(std::pair< std::string, cv::linemod::Detector >(object_name, *detector__));

                // Initialy, the object that will be detected is the last one learned.
                *detector_ = *detector__;
                g_current_object_of_interest = object_name;
                g_previous_object_of_interest = object_name;
                std::cout << "\nLoaded object \"" << object_name << "\", id \"" << object_id
                        << "\", with the number of samples " << Rs_[object_id].size() << std::endl << std::endl;

                g_finished_loading_at_least_one_model = true;
            }

            //initialize the visualization
        #if LINEMOD_VIZ_PCD
            if (g_ground_truth_generation || g_object_detection_evaluation)
            {
                pci_real_icpin_model2 = new LinemodPointcloud(nh_, "real_icpin_model2", "camera_rgb_optical_frame");
                pci_real_icpin_model = new LinemodPointcloud(nh_, "real_icpin_model", "camera_rgb_optical_frame");
                pci_real_icpin_ref = new LinemodPointcloud(nh_, "real_icpin_ref", "camera_rgb_optical_frame");
            }
            else
            {
                pci_real_icpin_model2 = new LinemodPointcloud(nh_, "real_icpin_model2", "mapping_camera_frame");
                pci_real_icpin_model = new LinemodPointcloud(nh_, "real_icpin_model", "mapping_camera_frame");
                pci_real_icpin_ref = new LinemodPointcloud(nh_, "real_icpin_ref", "mapping_camera_frame");
            }
        #endif
        }


        static void
        declare_params(tendrils& params)
        {
            object_recognition_core::db::bases::declare_params_impl(params, "LINEMOD");
            params.declare(&Detector::threshold_, "threshold", "Matching threshold, as a percentage", 93.0f);
            params.declare(&Detector::visualize_, "visualize", "If True, visualize the output.", false);
            params.declare(&Detector::use_rgb_, "use_rgb", "If True, use rgb-based detector.", true);
            params.declare(&Detector::use_depth_, "use_depth", "If True, use depth-based detector.", true);
            params.declare(&Detector::th_obj_dist_, "th_obj_dist", "Threshold on minimal distance between detected objects.", 0.04f);
            params.declare(&Detector::verbose_, "verbose", "If True, print.", false);
            params.declare(&Detector::depth_frame_id_, "depth_frame_id", "The depth camera frame id.", "camera_depth_optical_frame");
            params.declare(&Detector::icp_dist_min_, "icp_dist_min", "", 0.06f);
            params.declare(&Detector::px_match_min_, "px_match_min", "", 0.25f);
            params.declare(&Detector::linemod_similarity_weight, "linemod_similarity_weight", "linemod_similarity_weight", 1.0f);
            params.declare(&Detector::histogram_similarity_weight, "histogram_similarity_weight", "histogram_similarity_weight", 0.0f);
            params.declare(&Detector::TOP_N, "TOP_N", "TOP_N", 30.0f);
            params.declare(&Detector::depth_near_threshold, "depth_near", "depth_near", -1.0f);
            params.declare(&Detector::depth_far_threshold, "depth_far", "depth_far", 1000.0f);
        }


        static void
        declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
        {
            inputs.declare(&Detector::color_, "image", "An rgb full frame image.");
            inputs.declare(&Detector::depth_, "depth", "The 16bit depth image.");
            inputs.declare(&Detector::K_depth_, "K_depth", "The calibration matrix").required(true);

            outputs.declare(&Detector::pose_results_, "pose_results", "The results of object recognition");
        }


        void
        configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
        {
            configure_impl();
            g_listener = &listener;
            decision_making_sub_ = nh_.subscribe("/decision_making_state", 1, decision_making_state_handler);
            g_object_pose_recognition_mode = -1;
            get_robot_name();
            if (g_robot == BAXTER)
                g_base_reference_frame = "/base";
            else // Motoman
                g_base_reference_frame = "/base_link";
            clear_statistics();
            get_data_gathering_and_ground_truth_generation_flags();
        }


        /**
        * @brief Initializes the renderer with the parameters used in the training phase.
        * The renderer will be later used to render depth clouds for each detected object.
        * @param[in] The object id to initialize.*/
        void
        setupRenderer(const std::string &object_id)
        {
            object_recognition_core::db::ObjectDbParameters db_params(*json_db_);
            // Get the document for the object_id_ from the DB
            object_recognition_core::db::ObjectDbPtr db = db_params.generateDb();
            object_recognition_core::db::Documents documents = object_recognition_core::db::ModelDocuments(db,
                            std::vector<object_recognition_core::db::ObjectId>(1, object_id), "mesh");
            if (documents.empty())
            {
                std::cerr << "Skipping object id \"" << object_id << "\" : no mesh in the DB" << std::endl;
                return;
            }

            // Get the list of _attachments and figure out the original one
            object_recognition_core::db::Document document = documents[0];
            std::vector<std::string> attachments_names = document.attachment_names();
            std::string mesh_path;
            BOOST_FOREACH(const std::string& attachment_name, attachments_names)
            {
                if (attachment_name.find("original") != 0)
                    continue;
                std::cout << "Reading the mesh file " << attachment_name << std::endl;
                // Create a temporary file
                char mesh_path_tmp[L_tmpnam];
                strcpy(mesh_path_tmp, "tmpfileXXXXXX");
                if (mkstemp(mesh_path_tmp) == -1)
                {
                	printf("Error: Could not create tmp file %s\n", mesh_path_tmp);
                	exit(1);
                }
                std::remove((const char *) mesh_path_tmp);
                mesh_path = std::string(mesh_path_tmp) + attachment_name.substr(8);

                // Load the mesh and save it to the temporary file
                std::ofstream mesh_file;
                mesh_file.open(mesh_path.c_str());
                document.get_attachment_stream(attachment_name, mesh_file);
                mesh_file.close();
                std::string str = mesh_path.c_str();
                std::cout << "mesh_path " << std::string(mesh_path_tmp) << attachment_name.substr(8) << std::endl;
            }

            // the model name can be specified on the command line.
            Renderer3d *renderer_ = new Renderer3d(mesh_path);
            renderer_->set_parameters(renderer_width_, renderer_height_, renderer_focal_length_x_, renderer_focal_length_y_, renderer_near_, renderer_far_);

            std::remove(mesh_path.c_str());

            //initiaization of the renderer with the same parameters as used for learning
            RendererIterator *renderer_iterator_ = new RendererIterator(renderer_, renderer_n_points_);
            renderer_iterator_->angle_step_ = float(renderer_angle_step_);
            renderer_iterator_->radius_min_ = float(renderer_radius_min_);
            renderer_iterator_->radius_max_ = float(renderer_radius_max_);
            renderer_iterator_->radius_step_ = float(renderer_radius_step_);
            renderer_iterators_.insert(std::pair<std::string, RendererIterator*>(object_id, renderer_iterator_));
        }


        bool
        get_transform_components_indexes(int &p_x, int &p_y, int &p_z,
                int &r_x, int &r_y, int &r_z, int &r_w,
                tf::Transform transform)
        {
            tf::Vector3 position = transform.getOrigin();
            p_x = (int) (0.49 + (S_VEC_SIZE - 1) * (position[0] - PX_MIN) / (PX_MAX - PX_MIN));
            if ((p_x > (S_VEC_SIZE - 1)) || (p_x < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_x out of bounds" << std::endl;
                return (false);
            }
            p_y = (int) (0.49 + (S_VEC_SIZE - 1) * (position[1] - PY_MIN) / (PY_MAX - PY_MIN));
            if ((p_y > (S_VEC_SIZE - 1)) || (p_y < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_y out of bounds" << std::endl;
                return (false);
            }
            p_z = (int) (0.49 + (S_VEC_SIZE - 1) * (position[2] - PZ_MIN) / (PZ_MAX - PZ_MIN));
            if ((p_z > (S_VEC_SIZE - 1)) || (p_z < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_z out of bounds" << std::endl;
                return (false);
            }

            tf::Quaternion rotation = transform.getRotation();
            r_x = (int) (0.49 + (S_VEC_SIZE - 1) * (rotation[0] - R_MIN) / (R_MAX - R_MIN));
            if ((r_x > (S_VEC_SIZE - 1)) || (r_x < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_x out of bounds" << std::endl;
                return (false);
            }
            r_y = (int) (0.49 + (S_VEC_SIZE - 1) * (rotation[1] - R_MIN) / (R_MAX - R_MIN));
            if ((r_y > (S_VEC_SIZE - 1)) || (r_y < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_y out of bounds" << std::endl;
                return (false);
            }
            r_z = (int) (0.49 + (S_VEC_SIZE - 1) * (rotation[2] - R_MIN) / (R_MAX - R_MIN));
            if ((r_z > (S_VEC_SIZE - 1)) || (r_z < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_z out of bounds" << std::endl;
                return (false);
            }
            r_w = (int) (0.49 + (S_VEC_SIZE - 1) * (rotation[3] - R_MIN) / (R_MAX - R_MIN));
            if ((r_w > (S_VEC_SIZE - 1)) || (r_w < 0))
            {
                std::cout << "Error in get_transform_components_indexes(). r_w out of bounds" << std::endl;
                return (false);
            }

            return (true);
        }


        tf::Transform
        get_object_transform_with_respect_to_base(tf::Transform object_transform, tf::Transform camera_pose)
        {
            tf::Transform object_transform_with_respect_to_base;
            tf::Vector3 tr = camera_pose.getBasis() * object_transform.getOrigin();
            object_transform_with_respect_to_base.setOrigin(tr + camera_pose.getOrigin());
            object_transform_with_respect_to_base.setRotation(camera_pose.getRotation() * object_transform.getRotation());

            return (object_transform_with_respect_to_base);
        }


        bool
        add_pose_to_statistics(cv::Vec3f t, cv::Matx33f r, double pose_quality, int template_id)
        {
            tf::Transform transform = get_tf_tranform_from_cv_t_r(t, r);
            tf::Transform object_transform_with_respect_to_base = get_object_transform_with_respect_to_base(transform, g_camera_pose);
            int p_x, p_y, p_z, r_x, r_y, r_z, r_w;
            if (get_transform_components_indexes(p_x, p_y, p_z, r_x, r_y, r_z, r_w, object_transform_with_respect_to_base))
            {
                g_pose_statistics[0][p_x] += pose_quality;
                g_pose_statistics[1][p_y] += pose_quality;
                g_pose_statistics[2][p_z] += pose_quality;
                g_pose_statistics[3][r_x] += pose_quality;
                g_pose_statistics[4][r_y] += pose_quality;
                g_pose_statistics[5][r_z] += pose_quality;
                g_pose_statistics[6][r_w] += pose_quality;
            }
            struct pose_vector object_pose = {object_transform_with_respect_to_base, template_id};
            g_pose_vector.push_back(object_pose);
        }


        void
        print_statistics(size_t i, double pose_statistics[7][S_VEC_SIZE])
        {
            std::cout << "\n\n\n\n@@@@@@@@@@@@@@@@@@#######@@@@@@@@@@@@@@ " << i << "\n";
            for (size_t j = 0; j < S_VEC_SIZE; j++)
                std::cout << pose_statistics[i][j] << std::endl;
            std::cout << "\n\n\n\n";
        }


        bool
        get_best_pose(tf::Transform &best_object_pose_with_respect_to_camera, int &template_id)
        {
            double best_detection_quality = 0.0;
            size_t best_detection = 0;
            double pose_statistics[7][S_VEC_SIZE];
            cv::Mat cv_g_pose_statistics = cv::Mat::zeros(1, S_VEC_SIZE, CV_64FC1);
            cv::Mat cv_pose_statistics = cv::Mat::zeros(1, S_VEC_SIZE, CV_64FC1);

            for (size_t i = 0; i < 7; i++)
            {
                cv_g_pose_statistics = cv::Mat(1, S_VEC_SIZE, CV_64FC1, g_pose_statistics[i]);
                cv::GaussianBlur(cv_g_pose_statistics, cv_pose_statistics, cv::Size(51, 1), 0, 0);
                for (size_t j = 0; j < S_VEC_SIZE; j++)
                    pose_statistics[i][j] = cv_pose_statistics.at<double>(0, j);
                // print_statistics(i, pose_statistics);
            }

            for (size_t i = 0; i < g_pose_vector.size(); i++)
            {
                int p_x, p_y, p_z, r_x, r_y, r_z, r_w;
                if (get_transform_components_indexes(p_x, p_y, p_z, r_x, r_y, r_z, r_w, g_pose_vector[i].pose))
                {
                    double detection_quality =
                            pose_statistics[0][p_x] +
                            pose_statistics[1][p_y] +
                            pose_statistics[2][p_z] +
                            pose_statistics[3][r_x] +
                            pose_statistics[4][r_y] +
                            pose_statistics[5][r_z] +
                            pose_statistics[6][r_w];
                    if (detection_quality > best_detection_quality)
                    {
                        best_detection_quality = detection_quality;
                        best_detection = i;
                    }
                }
            }
            std::cout << "@@@@@@@@@@== best_detection index = " << best_detection
                    << "   best_detection_quality = " << best_detection_quality
                    << "   object detection state = " << g_object_detection_state << std::endl;
            if (best_detection_quality != 0.0)
            {
                best_object_pose_with_respect_to_camera = g_camera_pose.inverseTimes(g_pose_vector[best_detection].pose);
                template_id = g_pose_vector[best_detection].template_id;
                return (true);
            }
            else
                return (false);
        }


        cv::Mat
        get_cv_matrix(tf::Matrix3x3 rot)
        {
            cv::Mat r = Mat(3,3, CV_64F, double(0));;

            r.at<double>(0,0) = rot[0][0];
            r.at<double>(0,1) = rot[0][1];
            r.at<double>(0,2) = rot[0][2];
            r.at<double>(1,0) = rot[1][0];
            r.at<double>(1,1) = rot[1][1];
            r.at<double>(1,2) = rot[1][2];
            r.at<double>(2,0) = rot[2][0];
            r.at<double>(2,1) = rot[2][1];
            r.at<double>(2,2) = rot[2][2];

            return (r);
        }


        tf::Matrix3x3
        get_tf_matrix(cv::Matx33f r)
        {
            tf::Matrix3x3 rot;

            rot[0][0] = r(0,0);
            rot[0][1] = r(0,1);
            rot[0][2] = r(0,2);
            rot[1][0] = r(1,0);
            rot[1][1] = r(1,1);
            rot[1][2] = r(1,2);
            rot[2][0] = r(2,0);
            rot[2][1] = r(2,1);
            rot[2][2] = r(2,2);

            return (rot);
        }


        tf::Transform
        get_tf_tranform_from_cv_t_r(cv::Vec3f t, cv::Matx33f r)
        {
            tf::Vector3 trans(t[0], t[1], t[2]);
            tf::Matrix3x3 rot = get_tf_matrix(r);
            tf::Transform transform(rot, trans);

            return (transform);
        }


        void
        get_cv_t_r_from_tf_tranform(cv::Vec3f &t, cv::Matx33f &r, tf::Transform tf_tranform)
        {
            t[0] = tf_tranform.getOrigin().x();
            t[1] = tf_tranform.getOrigin().y();
            t[2] = tf_tranform.getOrigin().z();

            tf::Matrix3x3 rot = tf_tranform.getBasis();
            r(0,0) = rot[0][0];
            r(0,1) = rot[0][1];
            r(0,2) = rot[0][2];
            r(1,0) = rot[1][0];
            r(1,1) = rot[1][1];
            r(1,2) = rot[1][2];
            r(2,0) = rot[2][0];
            r(2,1) = rot[2][1];
            r(2,2) = rot[2][2];
        }


		void
		publish_object_pose(cv::Vec3f t, cv::Matx33f r, double max_similarity, int template_id)
		{
		    if (g_frames_to_capture > 0)
		        return;

			// 180degree rotation about the Y axis to fix detection rotation issues
            cv::Matx33f flip(-1.f,0.f,0.f,0.f,1.f,0.f,0.f,0.f,-1.f);
            r = r * flip;

            tf::Transform transform;
            if (max_similarity != 0.0)
            {
                 if (g_object_detection_state != od_idle)
                     add_pose_to_statistics(t, r, max_similarity, template_id);

                 int best_template_id;
                 if (!get_best_pose(transform, best_template_id))
                     transform = get_tf_tranform_from_cv_t_r(t, r);
            }
            else
                transform = get_tf_tranform_from_cv_t_r(t, r);

            static tf::TransformBroadcaster br;
			if (g_robot_arm == LEFT_ARM)
			{
                if (g_listener->canTransform("/camera_link", "/mapping_camera_frame", ros::Time(0)))
                {
                    tf::StampedTransform camera_link_to_mapping_camera_transform;
                    g_listener->lookupTransform("/camera_link", "/mapping_camera_frame", ros::Time(0), camera_link_to_mapping_camera_transform);
                    br.sendTransform(tf::StampedTransform(transform, camera_link_to_mapping_camera_transform.stamp_,
                            "/mapping_camera_frame", "/object_frame"));
                }
			}
            if (g_robot_arm == RIGHT_ARM)
            {
                if (g_listener->canTransform("/camera2_link", "/mapping_camera_frame", ros::Time(0)))
                {
                    tf::StampedTransform camera_link_to_mapping_camera_transform;
                    g_listener->lookupTransform("/camera2_link", "/mapping_camera_frame", ros::Time(0), camera_link_to_mapping_camera_transform);
                    br.sendTransform(tf::StampedTransform(transform, camera_link_to_mapping_camera_transform.stamp_,
                            "/mapping_camera_frame", "/object_frame"));
                }
            }
		}


		bool
		read_bin_dimensions(float *b_width, float *b_height, float *b_depth)
		{
			std::stringstream file_name;
			FILE *shelf_description_file;
			static bool shelf_description_read = false;
			static float x, y, z, roll, pitch, yaw, width, height, depth, rows, cols,
				a_col_width, b_col_width, c_col_width,
				a_row_height, d_row_height, g_row_height, j_row_height,
				shelfs_lip_height, support_column_width;

			if (!shelf_description_read)
			{
				std::string path = ros::package::getPath("prx_localizer");
				file_name << path << "/../../object_models/shelf_description.txt";
				shelf_description_file = fopen(file_name.str().c_str(), "r");
				int items_read = 0;
				if (shelf_description_file)
					items_read = fscanf(shelf_description_file, "{\'x\':%f, \'y\':%f, \'z\':%f, \'roll\':%f, \'pitch\':%f, \'yaw\':%f, \'width\':%f, \'height\':%f, \'depth\':%f, \'rows\':%f, \'cols\':%f, \'a_col_width\':%f, \'b_col_width\':%f, \'c_col_width\':%f, \'a_row_height\':%f, \'d_row_height\':%f, \'g_row_height\':%f, \'j_row_height\':%f, \'shelfs_lip_height\':%f, \'support_column_width\':%f}",
							&x, &y, &z, &roll, &pitch, &yaw, &width, &height, &depth, &rows, &cols,
							&a_col_width, &b_col_width, &c_col_width,
							&a_row_height, &d_row_height, &g_row_height, &j_row_height,
							&shelfs_lip_height, &support_column_width);
				else
				{
					fprintf(stderr, "====== Could not open file %s for reading in read_bin_dimensions()\n", file_name.str().c_str());
					return (false);
				}
				if (items_read != 20)
				{
					fprintf(stderr, "====== Could read all items from file %s in read_bin_dimensions()\n", file_name.str().c_str());
					return (false);
				}
				std::cout << "====== x = " << x
						<< "  y = " << y
						<< "  z = " << z
						<< "  roll = " << roll
						<< "  pitch = " << pitch
						<< "  yaw = " << yaw
						<< "  width = " << width
						<< "  height = " << height
						<< "  depth = " << depth
						<< "  rows = " << rows
						<< "  cols = " << cols
						<< "  a_col_width = " << a_col_width
						<< "  b_col_width = " << b_col_width
						<< "  c_col_width = " << c_col_width
						<< "  a_row_height = " << a_row_height
						<< "  d_row_height = " << d_row_height
						<< "  g_row_height = " << g_row_height
						<< "  j_row_height = " << j_row_height
						<< "  shelfs_lip_height = " << shelfs_lip_height
						<< "  support_column_width = " << support_column_width
						<< std::endl;
				shelf_description_read = true;
			}

		    char b = (char) g_bin_id;

		    if (b == 'A' || b == 'D' || b == 'G' || b == 'J')
				*b_width = a_col_width;
			else if (b == 'B' || b == 'E' || b == 'H' || b == 'K')
				*b_width = b_col_width;
			else if (b == 'C' || b == 'F' || b == 'I' || b == 'L')
				*b_width = c_col_width;
			else
			{
				printf("Error: Invalid bin id = %c in read_bin_dimensions()\n", b);
				exit(1);
			}

			if (b == 'A' || b == 'B' || b == 'C')
				*b_height = a_row_height;
			else if (b == 'D' || b == 'E' || b == 'F')
				*b_height = d_row_height;
			else if (b == 'G' || b == 'H' || b == 'I')
				*b_height = g_row_height;
			else if (b == 'J' || b == 'K' || b == 'L')
				*b_height = j_row_height;
			else
			{
				printf("Error: Invalid bin id = %c in read_bin_dimensions()\n", b);
				exit(1);
			}

			*b_depth = depth;

			return (true);
		}

        
		bool
		object_within_bin(object_recognition_core::db::ObjData *object)
		{
		    if (g_object_detection_evaluation)
		        return (true);

			float b_width, b_height, b_depth;
			if (!read_bin_dimensions(&b_width, &b_height, &b_depth))
				return (false);
			float bin_center_x = base_to_object_pose_hint3_transform_.getOrigin().x() + b_depth / 2.0;
			float bin_center_y = base_to_object_pose_hint3_transform_.getOrigin().y() + b_width / 2.0;
			float bin_center_z = base_to_object_pose_hint3_transform_.getOrigin().z() + b_height / 2.0;

			tf::Transform object_transform = get_tf_tranform_from_cv_t_r(object->t, object->r);
			Eigen::Matrix4f mt1;
			Eigen::Matrix4f mt2;
			pcl_ros::transformAsMatrix(base_to_mapping_camera_frame_transform_, mt1);
			pcl_ros::transformAsMatrix(object_transform, mt2);
			Eigen::Matrix4f base_to_object_transform_matrix = mt1 * mt2;
			float object_x = base_to_object_transform_matrix(0, 3);
			float object_y = base_to_object_transform_matrix(1, 3);
			float object_z = base_to_object_transform_matrix(2, 3);

			std::cout << "\n\n\n====== x_h: " << bin_center_x
					<< "  y_h: " << bin_center_y
					<< "  z_h: " << bin_center_z << std::endl;
			std::cout << "====== x_o: " << object_x
					<< "  y_o: " << object_y
					<< "  z_o: " << object_z << std::endl;

			float delta_x = fabs(object_x - bin_center_x);
			float delta_y = fabs(object_y - bin_center_y);
			float delta_z = fabs(object_z - bin_center_z);
			std::cout << "\n\n\n====== (b_depth, b_width, b_height) (" << b_depth << ", " << b_width << ", " << b_height << ")\n";
			std::cout << "====== (delta_x, delta_y, delta_z) (" << delta_x << ", " << delta_y << ", " << delta_z << ")\n";
			if ((delta_x < b_depth / 2.0) &&
				(delta_y < b_width / 2.0) &&
				(delta_z < b_height / 2.0))
				return (true);
			else
				return (false);
		}


		void
		publish_default_pose()
		{
            cv::Vec3f t(0.0, 0.0, 0.0);
            cv::Matx33f r(1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0);

            publish_object_pose(t, r, 0.0, 0);
		}


		void
		perform_state_trasition()
		{
		    static int previous_decision_making_state = -1;

		    if (g_decision_making_state != previous_decision_making_state)
		    {
		        if (g_decision_making_state == Move_to_Detect1)
		        {
		            g_object_detection_state = od_capture_1;
		            g_frames_to_capture = NUM_FRAMES_PER_CAPTURE;
		        }
		        else if (g_decision_making_state == Move_to_Detect2)
		        {
                    g_object_detection_state = od_capture_2;
                    g_frames_to_capture = NUM_FRAMES_PER_CAPTURE;
		        }
                else if (g_decision_making_state == Map_and_Detect_Object)
                {
                    g_object_detection_state = od_capture_3;
                    g_frames_to_capture = NUM_FRAMES_PER_CAPTURE;
                }
                else
                {
                    g_object_detection_state = od_reset;
                    g_frames_to_capture = -1;
                }

		        previous_decision_making_state = g_decision_making_state;
		    }
		    else if (g_frames_to_capture > 0)
		    {
		        g_frames_to_capture--;
		    }
		    else if (g_frames_to_capture == 0)
		    {
                g_object_detection_state = od_idle;
		    }

		    std::cout << "@@@@@===@@@@ Frames still to capture in this state = " << g_frames_to_capture << std::endl;
		}


		bool
		get_camera_pose_with_respect_to_the_base()
		{
            if (g_listener->canTransform(g_base_link_name, "/mapping_camera_frame", ros::Time(0)))
            {
                tf::StampedTransform camera_pose;
                g_listener->lookupTransform(g_base_link_name, "/mapping_camera_frame", ros::Time(0), camera_pose);
                g_camera_pose = camera_pose;

                return (true);
            }
            else
                return (false);
		}


		void
		clear_statistics()
		{
		    for (size_t i = 0; i < 7; i++)
		        for (size_t j = 0; j < S_VEC_SIZE; j++)
		            g_pose_statistics[i][j] = 0.0;

		    g_pose_vector.clear();
		}


		bool
		ready_to_process()
		{
            pose_results_->clear();

            if ((g_robot_arm == LEFT_ARM) && (*depth_frame_id_ != "camera_rgb_optical_frame"))
                return (false);
            if ((g_robot_arm == RIGHT_ARM) && (*depth_frame_id_ != "camera2_rgb_optical_frame"))
                return (false);

            if (!g_finished_loading_at_least_one_model || (g_object_pose_recognition_mode == 0)) // If still loading or not in ostd_mode, do nothing.
            {
                publish_default_pose();
                return (false);
            }

            if (g_current_object_of_interest != g_previous_object_of_interest)
            {
                *detector_ = detectors_.at(g_current_object_of_interest);
                if (detector_->classIds().empty())
                {
                    printf("Error: Could not setup renderer in ready_to_process() for object %s", g_current_object_of_interest.c_str());
                    exit(1);
                }
                // Remove previous version of the model
                renderer_iterators_.erase(detector_->classIds()[0]);
                // Create a new one
                setupRenderer(detector_->classIds()[0]);

                g_previous_object_of_interest = g_current_object_of_interest;
            }

            if (detector_->classIds().empty())
                return (false);

            perform_state_trasition();
            if (g_object_detection_state == od_reset)
            {
                clear_statistics();
            }
            else if (g_object_detection_state != od_idle)
            {
                if (!g_object_detection_evaluation)
                    if (!get_camera_pose_with_respect_to_the_base())
                    {
                        std::cout << "====@@@@==== In capture state but cannot compute camera pose with respect to base..." << std::endl;
                        return (false);
                    }
            }
            else if (g_object_detection_state == od_idle)
            {
               std::cout << "====@@@@==== Object detection in idle state..." << std::endl;
            }

            // Clear the point clouds
        #if LINEMOD_VIZ_PCD
            pci_real_icpin_model2->clear();
            pci_real_icpin_model->clear();
            pci_real_icpin_ref->clear();
        #endif

            return (true);
		}


		std::vector<cv::Mat>
		get_sources(cv::Mat *display)
		{
			std::vector<cv::Mat> sources;

            if (*use_rgb_)
            {
                cv::Mat color;
                if (color_->rows > 960)
                    cv::pyrDown(color_->rowRange(0, 960), color);
                else
                    color_->copyTo(color);

                if (*visualize_)
                	color_->copyTo(*display);

                sources.push_back(color);
            }
            if (*use_depth_)
            {
                if (!(*use_rgb_))
                {
                    //add a depth-based gray image to the list of sources for matching
                    depth_->convertTo(*display, CV_8U, 255.0/(1800.0));
                    cv::cvtColor(*display, *display, cv::COLOR_GRAY2BGR);
                    sources.push_back(*display);
                }

                sources.push_back(*depth_);
            }

            //imwrite("/home/alberto/RUTGERS/apc_hg/im.pgm", sources[1]);

//			{
//				cv::namedWindow("Rendering", cv::WINDOW_NORMAL);
//				if (!sources[0].empty())
//				{
//				  cv::imshow("Rendering", sources[0]);
//				  cv::waitKey(1);
//				}
//			}

            return (sources);
		}


		bool
		get_segmented_input(cv::Mat_<cv::Vec3f> *depth_real_input, cv::Vec3f *T_crop, cv::Mat *crop_input_image,
				cv::Mat_<cv::Vec3f> depth_real_input_raw, const cv::linemod::Match &match,
				cv::Rect_<int> rect_input, cv::Mat input_image)
		{
            //prepare the input data: from the sensor : crop images
            cv::Mat_<cv::Vec3f> input = depth_real_input_raw(rect_input);

            *crop_input_image = input_image(rect_input);

            //initialize the translation based on input data
            cv::Vec3f T_crop_ = input(input.rows / 2.0f, input.cols / 2.0f);

            float D_match = distances_.at(match.class_id)[match.template_id];
            //add the object's depth
            //std::cout << "\n\ndistance = " << D_match << "\nT_crop = " << T_crop_ << "\n\n";
            T_crop_(2) += D_match;

            if (!cv::checkRange(T_crop_))
                return (false);

            *depth_real_input = input;
            *T_crop = T_crop_;

            return (true);
		}


		bool
		get_bounding_boxes(cv::Rect_<int> *rect_input_, cv::Rect_<int> *rect_model_,
				int input_cols, int input_rows, int model_cols, int model_rows,
				const cv::linemod::Match &match)
		{
            //prepare the bounding box for the model and input point clouds
            cv::Rect_<int> rect_model(0, 0, model_cols, model_rows);
            //prepare the bounding box for the input point cloud: add the offset
            cv::Rect_<int> rect_input(rect_model);
            rect_input.x += match.x;
            rect_input.y += match.y;

            rect_input = rect_input & cv::Rect(0, 0, input_cols, input_rows);
            if ((rect_input.width < 5) || (rect_input.height < 5))
                return (false);

            //adjust both rectangles to be equal to the smallest among them
            if (rect_input.width > rect_model.width)
                rect_input.width = rect_model.width;
            if (rect_input.height > rect_model.height)
                rect_input.height = rect_model.height;
            if (rect_model.width > rect_input.width)
                rect_model.width = rect_input.width;
            if (rect_model.height > rect_input.height)
                rect_model.height = rect_input.height;

            *rect_input_ = rect_input;
            *rect_model_ = rect_model;

            return (true);
		}


		bool
		get_model_learned(cv::Mat_<cv::Vec3f> *depth_real_model, 
		        cv::Rect_<int> *rect_input, cv::Mat *model_image, cv::Mat *mask,
				int input_cols, int input_rows, cv::Matx33d R_match, const cv::linemod::Match &match)
		{
            if (!cv::checkRange(R_match))
                return (false);

            // Fill the Pose object
            cv::Vec3d T_match = Ts_.at(match.class_id)[match.template_id].clone();
            cv::Mat K_match = Ks_.at(match.class_id)[match.template_id];

            //get the point cloud of the rendered object model
            cv::Rect rect;
            cv::Matx33d R_temp(R_match.inv());
            cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));
            RendererIterator *it_r = renderer_iterators_.at(match.class_id);
            cv::Mat depth_model_;
            it_r->renderDepthOnly(depth_model_, *mask, rect, -T_match, up);
            it_r->renderImageOnly(*model_image, rect, -T_match, up);

            cv::Mat_<cv::Vec3f> depth_real_model_raw;
            cv::depthTo3d(depth_model_, K_match, depth_real_model_raw);

            cv::Rect_<int> rect_model;
            if (!get_bounding_boxes(rect_input, &rect_model,
            		input_cols, input_rows, depth_real_model_raw.cols, depth_real_model_raw.rows, match))
				return (false);

            *depth_real_model = depth_real_model_raw(rect_model);

            return (true);
		}

                Mat imHist(Mat hist, float scaleX=1, float scaleY=1, Mat compHist = Mat::zeros(2,2, CV_8UC3)){
                      double maxVal=0.75;
                      // if (countNonZero(compHist) < 1)
                      //   minMaxLoc(hist, 0, &maxVal, 0, 0);
                      // else
                      //   minMaxLoc(compHist, 0, &maxVal, 0, 0);
                      int rows = 64; //default height size
                      int cols = hist.rows; //get the width size from the histogram
                      Mat histImg = Mat::zeros(rows*scaleY, cols*scaleX, CV_8UC3);
                      //for each bin
                      for(int i=0;i<cols-1;i++) {
                        float histValue = hist.at<float>(i,0);
                        float nextValue = hist.at<float>(i+1,0);
                        Point pt1 = Point(i*scaleX, rows*scaleY);
                        Point pt2 = Point(i*scaleX+scaleX, rows*scaleY);
                        Point pt3 = Point(i*scaleX+scaleX, (rows-nextValue*rows/maxVal)*scaleY);
                        Point pt4 = Point(i*scaleX, (rows-nextValue*rows/maxVal)*scaleY);

                        int numPts = 5;
                        Point pts[] = {pt1, pt2, pt3, pt4, pt1};

                        fillConvexPoly(histImg, pts, numPts, Scalar(255,255,255));
                      }
                      return histImg;
                 }

                Mat bigViz (Mat base, Mat top, Mat bottom, std::string label_top, std::string label_bottom) {

                    Mat combine = Mat::zeros(base.size().height, base.size().width + top.size().width + 40, CV_8UC3);

                    Mat base_roi(combine, Rect(0,0,base.size().width,base.size().height));
                    base.copyTo(base_roi);
                    Mat top_roi(combine, Rect(base.size().width, 20, top.size().width, top.size().height));
                    top.copyTo(top_roi);
                    Mat bottom_roi(combine, Rect(base.size().width, (base.size().height/2), bottom.size().width, bottom.size().height));
                    bottom.copyTo(bottom_roi);

                    cv::putText(combine, label_top, Point(base.size().width, (base.size().height/2) - 10), FONT_HERSHEY_SIMPLEX, 0.25, Scalar(150,150,150), 1, 8, false);
                    cv::putText(combine, label_bottom, Point(base.size().width, base.size().height - 10), FONT_HERSHEY_SIMPLEX, 0.25, Scalar(150,150,150), 1, 8, false);

                    return combine;

                }

                Mat get_hogdescriptor_visual_image(Mat& origImg,
                                                   vector<float>& descriptorValues,
                                                   Size winSize,
                                                   Size cellSize,
                                                   int scaleFactor,
                                                   double viz_factor)
                {
                    // Code for visualizing HOG descriptors borrowed from here: http://www.juergenwiki.de/work/wiki/doku.php?id=public%3ahog_descriptor_computation_and_visualization
                    Mat visual_image = Mat::zeros(winSize.height, winSize.width, CV_8UC3);
                    resize(origImg, visual_image, Size(origImg.cols*scaleFactor, origImg.rows*scaleFactor));

                    int gradientBinSize = 9;
                    // dividing 180° into 9 bins, how large (in rad) is one bin?
                    float radRangeForOneBin = 3.14/(float)gradientBinSize;

                    // prepare data structure: 9 orientation / gradient strenghts for each cell
                        int cells_in_x_dir = winSize.width / cellSize.width;
                    int cells_in_y_dir = winSize.height / cellSize.height;
                    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
                    float*** gradientStrengths = new float**[cells_in_y_dir];
                    int** cellUpdateCounter   = new int*[cells_in_y_dir];
                    for (int y=0; y<cells_in_y_dir; y++)
                    {
                        gradientStrengths[y] = new float*[cells_in_x_dir];
                        cellUpdateCounter[y] = new int[cells_in_x_dir];
                        for (int x=0; x<cells_in_x_dir; x++)
                        {
                            gradientStrengths[y][x] = new float[gradientBinSize];
                            cellUpdateCounter[y][x] = 0;

                            for (int bin=0; bin<gradientBinSize; bin++)
                                gradientStrengths[y][x][bin] = 0.0;
                        }
                    }

                    // nr of blocks = nr of cells - 1
                    // since there is a new block on each cell (overlapping blocks!) but the last one
                    int blocks_in_x_dir = cells_in_x_dir - 1;
                    int blocks_in_y_dir = cells_in_y_dir - 1;

                    // compute gradient strengths per cell
                    int descriptorDataIdx = 0;
                    int cellx = 0;
                    int celly = 0;

                    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
                    {
                        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)
                        {
                            // 4 cells per block ...
                            for (int cellNr=0; cellNr<4; cellNr++)
                            {
                                // compute corresponding cell nr
                                int cellx = blockx;
                                int celly = blocky;
                                if (cellNr==1) celly++;
                                if (cellNr==2) cellx++;
                                if (cellNr==3)
                                {
                                    cellx++;
                                    celly++;
                                }

                                for (int bin=0; bin<gradientBinSize; bin++)
                                {
                                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                                    descriptorDataIdx++;

                                    gradientStrengths[celly][cellx][bin] += gradientStrength;

                                } // for (all bins)


                                // note: overlapping blocks lead to multiple updates of this sum!
                                // we therefore keep track how often a cell was updated,
                                // to compute average gradient strengths
                                cellUpdateCounter[celly][cellx]++;

                            } // for (all cells)


                        } // for (all block x pos)
                    } // for (all block y pos)


                    // compute average gradient strengths
                    for (int celly=0; celly<cells_in_y_dir; celly++)
                    {
                        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
                        {

                            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];

                            // compute average gradient strenghts for each gradient bin direction
                            for (int bin=0; bin<gradientBinSize; bin++)
                            {
                                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
                            }
                        }
                    }


//                    std::cout << "descriptorDataIdx = " << descriptorDataIdx << std::endl;

                    // draw cells
                    for (int celly=0; celly<cells_in_y_dir; celly++)
                    {
                        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
                        {
                            int drawX = cellx * cellSize.width;
                            int drawY = celly * cellSize.height;

                            int mx = drawX + cellSize.width/2;
                            int my = drawY + cellSize.height/2;

                            rectangle(visual_image,
                                      Point(drawX*scaleFactor,drawY*scaleFactor),
                                      Point((drawX+cellSize.width)*scaleFactor,
                                      (drawY+cellSize.height)*scaleFactor),
                                      CV_RGB(100,100,100),
                                      1);

                            // draw in each cell all 9 gradient strengths
                            for (int bin=0; bin<gradientBinSize; bin++)
                            {
                                float currentGradStrength = gradientStrengths[celly][cellx][bin];

                                // no line to draw?
                                if (currentGradStrength==0)
                                    continue;

                                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;

                                float dirVecX = cos( currRad );
                                float dirVecY = sin( currRad );
                                float maxVecLen = cellSize.width/2;
                                float scale = viz_factor; // just a visual_imagealization scale,
                                                          // to see the lines better

                                // compute line coordinates
                                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;

                                // draw gradient visual_imagealization
                                line(visual_image,
                                     Point(x1*scaleFactor,y1*scaleFactor),
                                     Point(x2*scaleFactor,y2*scaleFactor),
                                     CV_RGB(0,0,255),
                                     1);

                            } // for (all bins)

                        } // for (cellx)
                    } // for (celly)


                    // don't forget to free memory allocated by helper data structures!
                    for (int y=0; y<cells_in_y_dir; y++)
                    {
                      for (int x=0; x<cells_in_x_dir; x++)
                      {
                           delete[] gradientStrengths[y][x];
                      }
                      delete[] gradientStrengths[y];
                      delete[] cellUpdateCounter[y];
                    }
                    delete[] gradientStrengths;
                    delete[] cellUpdateCounter;

                    return visual_image;

                }

                bool hog_compare( Mat model_image, Mat crop_input_image, Mat mask, Mat & viz) {

                    // Mask the input image
                    Mat crop_input_image_mask;
                    crop_input_image.copyTo(crop_input_image_mask,mask);

                    // Convert to grayscale to compute HOG features
                    Mat model_gray, input_gray;
                    cvtColor(model_image, model_gray, COLOR_BGR2GRAY);
                    cvtColor(crop_input_image_mask, input_gray, COLOR_BGR2GRAY);

                    // Initialize descriptor variables and locations
                    HOGDescriptor model_hog, input_hog;
                    vector<float> model_desc, input_desc;
                    vector<Point> model_xy, input_xy;
                    double threshold = 1700.0;

                    //Resize the image to 128x128
                    resize(model_gray, model_gray, Size(128,128));
                    resize(input_gray, input_gray, Size(128,128));

                    // Compute HOG features for model and input grayscale images
                    model_hog.compute(model_gray,model_desc,Size(16,16),Size(0,0),model_xy);
                    input_hog.compute(input_gray,input_desc,Size(16,16),Size(0,0),input_xy);

                    // Show visually the HOG descriptors
                    Mat model_hog_viz = Mat::zeros(128, 128, CV_8UC3);
                    Mat input_hog_viz = Mat::zeros(128, 128, CV_8UC3);
                    model_hog_viz = get_hogdescriptor_visual_image(model_gray, model_desc, Size(128,128), Size(8,8), 1, 1);
                    input_hog_viz = get_hogdescriptor_visual_image(input_gray, input_desc, Size(128,128), Size(8,8), 1 /*scale factor */, 1);

                    cvtColor(model_hog_viz, model_hog_viz, COLOR_GRAY2BGR);
                    cvtColor(input_hog_viz, input_hog_viz, COLOR_GRAY2BGR);

                    // viz = bigViz(viz, model_hog_viz, input_hog_viz, "HOG Desc", "HOG Desc");

                    // Store descriptors in matrix form to compare
                    Mat model_hogfeat, input_hogfeat;
                    model_hogfeat.create(model_desc.size(),1,CV_32FC1);
                    input_hogfeat.create(input_desc.size(),1,CV_32FC1);

                    for (int h = 0; h < model_desc.size(); h++) {
                        model_hogfeat.at<float>(h,0) = model_desc.at(h);
                        input_hogfeat.at<float>(h,0) = input_desc.at(h);
                    }

                    // Compute simple Eucl distance between feature pairs
                    float distance = 0.0;
                    for (int i = 0; i < model_hogfeat.rows; i++) {
                        distance += fabs(model_hogfeat.at<float>(i,0) - input_hogfeat.at<float>(i,0));
//                        std::cout << i << " Adding " << model_hogfeat.at<float>(i,0) << " - " << input_hogfeat.at<float>(i,0) << " = " << fabs(model_hogfeat.at<float>(i,0) - input_hogfeat.at<float>(i,0)) << std::endl;
                    }
//                    std::cout << "Total Distance: " << distance << std::endl;

                    std::cout << std::endl << "HOG Distance between images: " << distance << std::endl;

                    if (distance < threshold)
                        return false;
                    else
                        return true;

                }

                bool
                too_different(Mat model_image, Mat crop_input_image, Mat mask, double *similarity, Mat & imgViz, int h_bins, int s_bins, Mat costRix)
                {    
                    // see http://docs.opencv.org/doc/tutorials/imgproc/histograms/histogram_comparison/histogram_comparison.html
                    
                    clock_t tStart = clock();

                    Mat crop_input_image_mask;

                    if (crop_input_image.size() != mask.size())
                        return true;
                    crop_input_image.copyTo(crop_input_image_mask, mask);
		    
		    // imgViz = bigViz(imgViz, model_image, crop_input_image_mask, "Model", "Image");

                    /// Convert to HSV
                    Mat hsv_base, hsv_test;

                    vector<Mat> tareks;
                    cv::Mat histNorm;
                    cv::cvtColor(model_image, model_image, CV_BGR2YCrCb);
                    cv::split(model_image, tareks);
                    cv::equalizeHist(tareks[0], tareks[0]);

                    //equalize red    
                    cv::equalizeHist(tareks[1], tareks[1]);


                    cv::equalizeHist(tareks[2], tareks[2]);


                    cv::merge(tareks, model_image);
                    cv::cvtColor(model_image, model_image, CV_YCrCb2BGR);
                    cv::cvtColor(model_image, hsv_base, CV_BGR2HSV);

                    cv::cvtColor(crop_input_image_mask, crop_input_image_mask, CV_BGR2YCrCb);
                    cv::split(crop_input_image_mask, tareks);
                    cv::equalizeHist(tareks[0], tareks[0]);

                    //equalize red    
                    cv::equalizeHist(tareks[1], tareks[1]);
                    cv::equalizeHist(tareks[2], tareks[2]);

                    cv::merge(tareks, crop_input_image_mask);
                    cv::cvtColor(crop_input_image_mask, crop_input_image_mask, CV_YCrCb2BGR);
                    cv::cvtColor(crop_input_image_mask, hsv_test, CV_BGR2HSV);

                    /// Using 50 bins for hue and 50 for saturation
                    int histSize[] = {h_bins};
                    int HistSize2D[] = {h_bins, s_bins};

                    // hue varies from 0 to 179, saturation from 0 to 255
                    float huerange[] = {0, 180};
                    float satrange[] = {0, 256};

                    const float *h_ranges[] = { huerange };
                    const float *s_ranges[] = { satrange };
                    const float *ranges[] = { huerange, satrange };

                    // Use the o-th and 1-st channels
                    int channels[] = {0};

                    /// Histograms
                    MatND hist_base;
                    MatND hist_test;

                    // Histo Images
                    Mat histImgBase, histImgTest;

                    // Separate Color Channels
                    vector<Mat> colorsBase;
                    vector<Mat> colorsTest;
                    split(hsv_base, colorsBase);
                    split(hsv_test, colorsTest);


                    //implement quadrants
                    vector<Mat> colorsBase_quads, colorsModel_quads, quad_masks;
                    colorsBase_quads.resize(4);
                    colorsModel_quads.resize(4);
                    quad_masks.resize(4);
                    Mat hue_map_base = colorsBase[0];
                    Mat hue_map_model = colorsTest[0];
                    Rect roi1(0, 0, floor(colorsBase[0].cols/2), floor(colorsBase[0].rows/2));
                    Rect roi2(0, floor(colorsBase[0].rows/2), floor(colorsBase[0].cols/2), floor(colorsBase[0].rows/2));
                    Rect roi3(floor(colorsBase[0].cols/2), 0, floor(colorsBase[0].cols/2), floor(colorsBase[0].rows/2));
                    Rect roi4(floor(colorsBase[0].cols/2), colorsBase[0].rows/2, floor(colorsBase[0].cols/2), floor(colorsBase[0].rows/2));
                    colorsBase_quads[0] = hue_map_base(roi1);
                    colorsBase_quads[1] = hue_map_base(roi2);
                    colorsBase_quads[2] = hue_map_base(roi3);
                    colorsBase_quads[3] = hue_map_base(roi4);
                    colorsModel_quads[0] = hue_map_model(roi1);
                    colorsModel_quads[1] = hue_map_model(roi2);
                    colorsModel_quads[2] = hue_map_model(roi3);
                    colorsModel_quads[3] = hue_map_model(roi4);
                    quad_masks[0] = mask(roi1);
                    quad_masks[1] = mask(roi2);
                    quad_masks[2] = mask(roi3);
                    quad_masks[3] = mask(roi4);


                    double quad_sim[4];
                    double sumer=0;
                    int quad_count=4;
                    for(int i=0; i<4;++i)
                    {
                        //calculate histograms for hue channel of base and model images    
                        calcHist(&colorsBase_quads[i], 1, channels, quad_masks[i], hist_base, 1, histSize, h_ranges);
                        GaussianBlur(hist_base, hist_base, Size(1,5), 0, 0);
                        normalize(hist_base, hist_base, 1.0, 0, NORM_L1, -1, Mat());
                        histImgBase = imHist(hist_base, 5, 2, hist_base);

                        calcHist(&colorsModel_quads[i], 1, channels, quad_masks[i], hist_test, 1, histSize, h_ranges);
                        GaussianBlur(hist_test, hist_test, Size(1,5), 0, 0);
                        normalize(hist_test, hist_test, 1.0, 0, NORM_L1, -1, Mat());
                        histImgTest = imHist(hist_test, 5, 2, hist_base);

			imgViz = bigViz(imgViz, histImgBase, histImgTest, "Model - Quad " + boost::to_string(i), "Image - Quad " + boost::to_string(i));

                         // Convert histos to signatures for EMD matching
                        int numrows = h_bins;
                        Mat sig1 = Mat(numrows, 2, CV_32FC1);
                        Mat sig2 = Mat(numrows, 2, CV_32FC1);

                        // std::cout << "Values for Quad: " << boost::to_string(i) << std::endl;

                        float hist_sum = 0;
                        float hist_sum_2 = 0;
                        for (int h = 0; h < h_bins; h++) 
                        {
                            float bin_val = hist_base.at<float>(h);
                            sig1.at<float>(h,0) = bin_val;
                            sig1.at<float>(h,1) = h;
                            hist_sum += bin_val;
                            // std::cout << "1_" << h << ": " << bin_val << "\t";

                            bin_val = hist_test.at<float>(h);
                            sig2.at<float>(h,0) = bin_val;
                            sig2.at<float>(h,1) = h;
                            hist_sum_2 += bin_val;
                            // std::cout << "2_" << h << ": " << bin_val << "\n";
                        }

                        // Check to see if this quadrant has a valid histogram or not
                        if (hist_sum == 0 || hist_sum_2 == 0) 
                        {
                            quad_count--;
                            continue;
                        }

                        // Compute EMD using precomputed cost matrix
                        float emd_bb_h = EMD(sig1,sig1,CV_DIST_USER,costRix);
                        float emd_bt_h = EMD(sig1,sig2,CV_DIST_USER,costRix);

                        quad_sim[i]=emd_bt_h;
                        sumer+=(quad_sim[i]*quad_sim[i]);
                    }

                    switch (quad_count) 
                    {
                        case 4:
                            break;
                        case 3:
                        {
                            sumer = sumer * (4/3);
                            break;
                        }
                        case 2:
                        {
                            sumer = sumer * 2;
                            break;
                        }     
                        case 1:
                        {
                            sumer = sumer * 4;
                            break;
                        }    
                        default:
                            break;
                    }   

                    double tMatch = (double)(clock() - tStart)*1000/CLOCKS_PER_SEC;
                    // std::cout << "Color Comparison (msec): " << tMatch << std::endl;
                    
                    *similarity = sqrt(sumer);
                    bool bad;
                    if (*similarity > 20)
                    {
                        printf( "+++++ Not so good similarity on histogram comparison -> %f\n", *similarity);
                        bad = true;
                    }
                    else
                    {
                        printf( "+++++ Good similarity on histogram comparison -> %f\n", *similarity);
                        bad = false;
                    }
                    return (bad);
                }

        void
        fill_in_the_buffer_of_detected_objects(std::vector<cv::linemod::Match> matches,
                std::vector<cv::Mat> sources, cv::Mat display)
        {

            int num_modalities = (int) detector_->getModalities().size();
            cv::Mat_<cv::Vec3f> depth_real_input_raw;
            cv::Mat_<float> K;
            K_depth_->convertTo(K, CV_32F);
            cv::depthTo3d(sources[1], K, depth_real_input_raw);

            objs_.clear();

            int h_bins = 50, s_bins = 50;
            int numrows = h_bins;// *s_bins;

            Mat sig1 = Mat(numrows, 1, CV_32FC1);
            Mat sig2 = Mat(numrows, 1, CV_32FC1);

            for (int h = 0; h < h_bins; h++) {
//                for (int s = 0; s < s_bins; s++) {
                    sig1.at<float>(h,0) = h;
//                    sig1.at<float>(h,1) = s;

                    sig2.at<float>(h,0) = h;
//                    sig2.at<float>(h,1) = s;
//                }
            }

            // Create custom cost matrix (distance matrix)
            Mat costRix = Mat(numrows, numrows, CV_32FC1);
            for (int x = 0; x < numrows; x++) {
                for (int y = 0; y < numrows; y++) {
                    int x1 = sig1.at<float>(x,0);
//                    int y1 = sig1.at<float>(x,1);
                    int x2 = sig2.at<float>(y,0);
//                    int y2 = sig2.at<float>(y,1);

                    int delta_x = std::min(std::abs(x1-x2),h_bins-std::abs(x1-x2));
//                    int delta_y = std::min(std::abs(y1-y2),h_bins-std::abs(y1-y2));

                    costRix.at<float>(x,y) = delta_x; // +delta_y;
                }
            }

            std::vector< std::pair<object_recognition_core::db::ObjData, double> > object_similarity_vec;
            BOOST_FOREACH(const cv::linemod::Match &match, matches)
            {

                if (*visualize_)
                {
                    const std::vector<cv::linemod::Template>& templates =
                    		detector_->getTemplates(match.class_id, match.template_id);
                    drawResponse(templates, num_modalities, display, cv::Point(match.x, match.y), detector_->getT(0));
                }

                cv::Matx33d R_match = Rs_.at(match.class_id)[match.template_id].clone();
                cv::Mat_<cv::Vec3f> depth_real_model;
                cv::Rect_<int> rect_input;
                cv::Mat model_image, mask;
                if (!get_model_learned(&depth_real_model, &rect_input, &model_image, &mask,
                		depth_real_input_raw.cols, depth_real_input_raw.rows, R_match, match))
                	continue;

                cv::Mat_<cv::Vec3f> depth_real_input;
                cv::Vec3f T_crop;
                cv::Mat crop_input_image;
                if (!get_segmented_input(&depth_real_input, &T_crop, &crop_input_image,
                		depth_real_input_raw, match, rect_input, sources[0]))
                	continue;

                double *similarity;
                similarity = new double;
                *similarity = 0.15;

                Mat imgBase = Mat::zeros(336, 40, CV_8UC3);

                if (too_different(model_image, crop_input_image, mask, similarity, imgBase, h_bins, s_bins, costRix))
                {
                   // namedWindow("Detection Comparison");
                   // imshow("Detection Comparison", imgBase);
                   // waitKey(10000);
                     continue;
                }

//                std::cout << "Time to compare histograms: " << (double)(clock() - tStart)/CLOCKS_PER_SEC << std::endl;

//                 if (hog_compare(model_image, crop_input_image, mask, imgBase)) {
//                     namedWindow("Detection Comparison");
//                     imshow("Detection Comparison", imgBase);
//                     waitKey(1);
//                     continue;
//                }

                // namedWindow("Detection Comparison");
                // imshow("Detection Comparison", imgBase);
                // waitKey(10000);

                cv::Vec3f T_real_icp(T_crop);
                // initialize the rotation based on model data
                cv::Matx33f R_real_icp(R_match);

                //get the point clouds (for both input and model)
                std::vector<cv::Vec3f> pts_real_model_temp;
                std::vector<cv::Vec3f> pts_real_input_temp;
                // std::cout << "depth_real_input.size() " << depth_real_input.size() << "   depth_real_model.size() " << depth_real_model.size() << std::endl;
                float px_ratio_missing = matToVec(depth_real_input, depth_real_model,
                		pts_real_input_temp, pts_real_model_temp);
                // std::cout << "pts_real_input_temp.size() " << pts_real_input_temp.size() << "   pts_real_model_temp.size() " << pts_real_model_temp.size() << std::endl;

                if (0)//px_ratio_missing > *px_match_min_)
                    continue;

                //perform the first approximate ICP
                float px_ratio_match_inliers = 0.0f;
                float icp_dist;
                icp_dist = icpCloudToCloud(pts_real_input_temp, pts_real_model_temp, R_real_icp, T_real_icp,
                		px_ratio_match_inliers, 1);
                // std::cout << "icp_dist  " << icp_dist;
                // std::cout << "icp_ratio  " << px_ratio_match_inliers;

                // cv::waitKey(100000);
//
//                //perform a finer ICP
//                icp_dist = icpCloudToCloud(pts_real_input_temp, pts_real_model_temp, R_real_icp, T_real_icp,
//                		px_ratio_match_inliers, 2);
//                std::cout << "  icp_dist2  " << icp_dist;
//                px_ratio_match_inliers = 0.0f;
//                icp_dist = icpCloudToCloud(pts_real_input_temp, pts_real_model_temp, R_real_icp, T_real_icp,
//                		px_ratio_match_inliers, 0);
//                //reject the match if the icp distance is too big
//                std::cout << "  icp_dist3  " << icp_dist << std::endl;
                if (0)//icp_dist < *icp_dist_min_)
                    continue;

                // Weighted similarity measure
                // Coefficients (parameters) multiplied by similarity score for each of linemod and color histogram comparison
                // similarity scores on scale 0-100
                double weighted_similarity = (*linemod_similarity_weight * match.similarity) + (*histogram_similarity_weight * ((50.0f-(*similarity))*2.0f));

                object_similarity_vec.push_back(std::make_pair(
                        object_recognition_core::db::ObjData(pts_real_input_temp, pts_real_model_temp,
                                match.class_id, match.template_id, match.similarity, icp_dist, px_ratio_match_inliers,
                                R_real_icp, T_crop),
                        weighted_similarity));


//                objs_.push_back(object_recognition_core::db::ObjData(pts_real_input_temp, pts_real_model_temp,
//                		match.class_id, match.similarity, icp_dist, px_ratio_match_inliers, R_real_icp, T_crop));
            }

            // Sort all detections based on weighted similarity measure ( parameterized -- weights can be passed through config files)
            std::sort(object_similarity_vec.begin(), object_similarity_vec.end(),
                      boost::bind(&std::pair<object_recognition_core::db::ObjData, double>::second, _1) >
                      boost::bind(&std::pair<object_recognition_core::db::ObjData, double>::second, _2));
            // Clear object results in order to push back top N detections by weighted similarity
            objs_.clear();

            std::cout<<"\nLinemod weight : "<<*linemod_similarity_weight<<" , hist weight : "<<*histogram_similarity_weight;
            std::cout<<"\n TOP N detections with N = 3, out of "<<object_similarity_vec.size();

            // Selecting the Top N (parameterized using config file) detections by weighted similarity
            for(int i = 0; i < *TOP_N && i < object_similarity_vec.size(); ++i)
            {
                objs_.push_back(object_similarity_vec[i].first);
                std::cout<<"\ni = "<<i<<" , with Weighted Similarity = "<<object_similarity_vec[i].second;
            }
            std::cout<<"\n";

		}


		void
		fill_in_pcl_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud, const std::vector<cv::Vec3f> &opencv_cloud)
		{
			pcl_cloud->width    = opencv_cloud.size();
			pcl_cloud->height   = 1;
			pcl_cloud->is_dense = false;
			pcl_cloud->points.resize (pcl_cloud->width * pcl_cloud->height);
			for (size_t i = 0; i < pcl_cloud->points.size(); ++i)
			{
				pcl_cloud->points[i].x = opencv_cloud[i].val[0];
				pcl_cloud->points[i].y = opencv_cloud[i].val[1];
				pcl_cloud->points[i].z = opencv_cloud[i].val[2];
			}
		}


		void
        fill_in_std_vector_cloud(std::vector<cv::Vec3f> &opencv_cloud, pcl::PointCloud<pcl::PointXYZ> pcl_cloud)
		{
			for (size_t i = 0; i < pcl_cloud.points.size(); ++i)
			{
				cv::Vec3f val;
				val[0] = pcl_cloud.points[i].x;
				val[1] = pcl_cloud.points[i].y;
				val[2] = pcl_cloud.points[i].z;
				opencv_cloud.push_back(val);
            }
        }


        void
        fill_in_std_vector_cloud2(std::vector<cv::Vec3f> &opencv_cloud, cv::Mat_<cv::Vec3f> cv_vec_cloud)
        {
            for (size_t y = 0; y < cv_vec_cloud.rows; ++y)
            {
                for (size_t x = 0; x < cv_vec_cloud.cols; ++x)
                {
                    cv::Vec3f val;
                    val[0] = cv_vec_cloud(y, x)[0];
                    val[1] = cv_vec_cloud(y, x)[1];
                    val[2] = cv_vec_cloud(y, x)[2];
                    opencv_cloud.push_back(val);
                }
			}
		}


		object_recognition_core::db::ObjData *
		align_object_point_cloud(object_recognition_core::db::ObjData *o_match)
		{	// http://docs.pointclouds.org/trunk/classpcl_1_1_iterative_closest_point.html
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ> cloud_aligned;

			fill_in_pcl_cloud(cloud_out, o_match->pts_ref);
			fill_in_pcl_cloud(cloud_in, o_match->pts_model);
			// Set the input source and target
			icp.setInputSource(cloud_in);
			icp.setInputTarget(cloud_out);
			// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
			icp.setMaxCorrespondenceDistance(0.01);
			// Set the maximum number of iterations (criterion 1)
			icp.setMaximumIterations(50);
			// Set the transformation epsilon (criterion 2)
			icp.setTransformationEpsilon(1e-8);
			// Set the euclidean distance difference epsilon (criterion 3)
			icp.setEuclideanFitnessEpsilon(1);
			// Perform the alignment
			icp.align(cloud_aligned);
			// Obtain the transformation that aligned cloud_source to cloud_aligned
			Eigen::Matrix4f transformation = icp.getFinalTransformation();
			cv::Vec3f T_optimal(transformation(0,3), transformation(1,3), transformation(2,3));
			cv::Matx33f R_optimal;
			R_optimal(0,0) = transformation(0,0);
			R_optimal(0,1) = transformation(0,1);
			R_optimal(0,2) = transformation(0,2);
			R_optimal(1,0) = transformation(1,0);
			R_optimal(1,1) = transformation(1,1);
			R_optimal(1,2) = transformation(1,2);
			R_optimal(2,0) = transformation(2,0);
			R_optimal(2,1) = transformation(2,1);
			R_optimal(2,2) = transformation(2,2);

			if (icp.hasConverged())
			{
				//update the translation matrix: turn to opposite direction at first and then do translation
				o_match->t = R_optimal * o_match->t;
			    //do translation
			    cv::add(o_match->t, T_optimal, o_match->t);
			    //update the rotation matrix
			    o_match->r = R_optimal * o_match->r;

			    o_match->pts_model.clear();
				fill_in_std_vector_cloud(o_match->pts_model, cloud_aligned);
			}
			std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;

//			std::cout << "Rotation current\n";
//			std::cout << o_match->r << std::endl;
//			std::cout << "Translation current\n";
//			std::cout << o_match->t << std::endl;
//			std::cout << "Matrix\n";
//			std::cout << transformation << std::endl;

#if LINEMOD_VIZ_PCD
			if (icp.hasConverged())
			{
				std::vector<cv::Vec3f> std_vector_cloud;
				fill_in_std_vector_cloud(std_vector_cloud, cloud_aligned);
				pci_real_icpin_model2->fill(std_vector_cloud, cv::Vec3b(255,0,0));
			}
#endif
			return (o_match);
		}


		void
		insert_object_into_ork_result_list(object_recognition_core::db::ObjData *o_match)
		{
            // 180degree rotation about the Y axis to fix detection rotation issues
            cv::Matx33f flip(-1.f,0.f,0.f,0.f,1.f,0.f,0.f,0.f,-1.f);
            cv::Mat adj_r = cv::Mat((o_match->r)*flip);

            //return the outcome object pose
            PoseResult pose_result;
            pose_result.set_object_id(db_, o_match->match_class);
            pose_result.set_confidence(o_match->match_sim);
            pose_result.set_R(adj_r);
            pose_result.set_T(cv::Mat(o_match->t));
            pose_results_->push_back(pose_result);

			//icp_dist in the same units as the sensor data
            //this distance is used to compute the ratio of inliers (points laying within this distance between the point clouds)
			float icp_dist = 0.007f;
            float px_inliers_ratio = getL2distClouds(o_match->pts_model, o_match->pts_ref, icp_dist);
            if (*verbose_)
                std::cout << o_match->match_class << " "
                << get_object_name_by_object_id(o_match->match_class) << " "
                << o_match->match_sim << " icp " << icp_dist << ", " << " ratio "
                << o_match->icp_px_match << " or " << px_inliers_ratio << std::endl;

            //add points to the clouds
#if LINEMOD_VIZ_PCD
            pci_real_icpin_model->fill(o_match->pts_model, cv::Vec3b(0,255,0));
            pci_real_icpin_ref->fill(o_match->pts_ref, cv::Vec3b(0,0,255));
#endif
		}


		object_recognition_core::db::ObjData *
		find_the_best_object_match_among_near_objects_using_hint(std::vector <object_recognition_core::db::ObjData>::iterator it_o)
		{
			std::vector <object_recognition_core::db::ObjData>::iterator it_o2 = it_o;
			object_recognition_core::db::ObjData *o_match = &(*it_o);
			for (++it_o2; it_o2 != objs_.end(); ++it_o2)
			{
				if ((!it_o2->check_done) && (cv::norm(o_match->t, it_o2->t) < *th_obj_dist_))
				{
					it_o2->check_done = true;
					if ((it_o2->match_sim > o_match->match_sim) && (g_object_detection_evaluation || object_within_bin(o_match)))
						o_match = &(*it_o2);
				}
			}
			return (o_match);
		}


		object_recognition_core::db::ObjData *
		find_the_best_object_match_among_near_objects(std::vector <object_recognition_core::db::ObjData>::iterator it_o)
		{
			std::vector <object_recognition_core::db::ObjData>::iterator it_o2 = it_o;
			object_recognition_core::db::ObjData *o_match = &(*it_o);
			for (++it_o2; it_o2 != objs_.end(); ++it_o2)
			{
				if ((!it_o2->check_done) && (cv::norm(o_match->t, it_o2->t) < *th_obj_dist_))
				{
					it_o2->check_done = true;
					if (it_o2->match_sim > o_match->match_sim)
						o_match = &(*it_o2);
				}
			}
			return (o_match);
		}


		object_recognition_core::db::ObjData *
		process_with_object_pose_hint(double *out_max_similarity)
		{
            double max_similarity = 0.0;
            object_recognition_core::db::ObjData *best_match = NULL;
            std::vector <object_recognition_core::db::ObjData>::iterator it_o;

            for (it_o = objs_.begin(); it_o != objs_.end(); ++it_o)
            {
            	if (!it_o->check_done)
            	{
					object_recognition_core::db::ObjData *o_match = find_the_best_object_match_among_near_objects_using_hint(it_o);
					o_match = align_object_point_cloud(o_match);
					insert_object_into_ork_result_list(o_match);

					if ((o_match->match_sim > max_similarity) &&
						object_within_bin(o_match))
					{
						best_match = o_match;
						max_similarity = o_match->match_sim;
					}
            	}
            }

            if ((best_match != NULL) && object_within_bin(best_match))
            {
                publish_object_pose(best_match->t, best_match->r, max_similarity, best_match->template_id);
                std::cout << "$$$$$$$$$$$$ best_match->template_id " << best_match->template_id << std::endl;
            }
            else
            	publish_default_pose();

            *out_max_similarity = max_similarity;
            return (best_match);
		}


        object_recognition_core::db::ObjData *
		process_without_object_pose_hint(double *out_max_similarity)
		{
            double max_similarity = 0.0;
            object_recognition_core::db::ObjData *best_match = NULL;
            std::vector <object_recognition_core::db::ObjData>::iterator it_o;

            for (it_o = objs_.begin(); it_o != objs_.end(); ++it_o)
            {
            	if (!it_o->check_done)
            	{
					object_recognition_core::db::ObjData *o_match = find_the_best_object_match_among_near_objects(it_o);
//					o_match = align_object_point_cloud(o_match);
//					insert_object_into_ork_result_list(o_match);

					if (o_match->match_sim > max_similarity)
					{
						best_match = o_match;
						max_similarity = o_match->match_sim;
					}
            	}
            }

            if (best_match != NULL)
            {

            	best_match = align_object_point_cloud(best_match);
				insert_object_into_ork_result_list(best_match);
                publish_object_pose(best_match->t, best_match->r, max_similarity, best_match->template_id);
            }
            else
            	publish_default_pose();

            *out_max_similarity = max_similarity;
            return (best_match);
		}


		cv::Point2d
		project_transform_origin_to_pixel(tf::StampedTransform transform, cv::Mat_<double> K)
		{
			cv::Point2d uv;
            double x, y, z;
            x = transform.getOrigin().x();
            y = transform.getOrigin().y();
            z = transform.getOrigin().z();
			double fx = K.at<double>(0,0);
			double fy = K.at<double>(1,1);
			double cx = K.at<double>(0,2);
			double cy = K.at<double>(1,2);
			double Tx = 0.0;
			double Ty = 0.0;
			uv.x = (fx * x + Tx) / z + cx;
			uv.y = (fy * y + Ty) / z + cy;

			return (uv);
		}


		bool
		get_region_of_interest(cv::Point rectangle[4])
		{
			if (g_listener->canTransform(g_base_reference_frame, "/object_pose_hint3", ros::Time(0)) &&
				g_listener->canTransform(g_base_reference_frame, "/mapping_camera_frame", ros::Time(0)) &&
				g_listener->canTransform("/mapping_camera_frame", "/object_pose_hint0", ros::Time(0)) &&
				g_listener->canTransform("/mapping_camera_frame", "/object_pose_hint1", ros::Time(0)) &&
				g_listener->canTransform("/mapping_camera_frame", "/object_pose_hint2", ros::Time(0)) &&
				g_listener->canTransform("/mapping_camera_frame", "/object_pose_hint3", ros::Time(0)))
			{
				g_listener->lookupTransform(g_base_reference_frame, "/mapping_camera_frame", ros::Time(0), base_to_mapping_camera_frame_transform_);
				g_listener->lookupTransform(g_base_reference_frame, "/object_pose_hint3", ros::Time(0), base_to_object_pose_hint3_transform_);

		        tf::StampedTransform t0, t1, t2, t3;
		        g_listener->lookupTransform("/mapping_camera_frame", "/object_pose_hint0", ros::Time(0), t0);
				g_listener->lookupTransform("/mapping_camera_frame", "/object_pose_hint1", ros::Time(0), t1);
				g_listener->lookupTransform("/mapping_camera_frame", "/object_pose_hint2", ros::Time(0), t2);
				g_listener->lookupTransform("/mapping_camera_frame", "/object_pose_hint3", ros::Time(0), t3);

                cv::Mat_<double> K;
                K_depth_->convertTo(K, CV_64F);
                static const int RADIUS = 5;
                rectangle[0] = project_transform_origin_to_pixel(t0, K);
                rectangle[1] = project_transform_origin_to_pixel(t1, K);
                rectangle[2] = project_transform_origin_to_pixel(t2, K);
                rectangle[3] = project_transform_origin_to_pixel(t3, K);
                std::cout << "+++++++++++ got hint transform\n";

                return (true);
			}
			else
			{
                std::cout << "+++++++++++ do not have hint transform\n";
				return (false);
			}
		}


		bool
		erase_regions_of_no_interest(std::vector<cv::Mat> sources, cv::Mat display, cv::Mat &mask)
		{
			cv::Point rectangle[4];
            if (get_region_of_interest(rectangle))
            {
            	mask = cv::Mat::zeros(sources[0].rows, sources[0].cols, CV_8UC1);
            	cv::fillConvexPoly(mask, rectangle, 4, cv::Scalar(255));

            	std::vector<cv::Mat> color_channel(3);
                for (int i = 0; i < 3; i++)
                {
                    cv::extractChannel(sources[0], color_channel[i], i);
                    cv::bitwise_and(color_channel[i], mask, color_channel[i]);
                }
                cv::merge(color_channel, sources[0]);

                if (*visualize_)
                {
					for (int i = 0; i < 3; i++)
					{
						cv::extractChannel(display, color_channel[i], i);
						cv::bitwise_and(color_channel[i], mask, color_channel[i]);
					}
					cv::merge(color_channel, display);
                }
                std::cout << "+++++++++ Regions not of interest erased!!\n";
            	return (true);
            }
            else
            {
                std::cout << "+++++++++ Could not erase regions of no interest...\n";
                mask = cv::Mat::ones(sources[0].rows, sources[0].cols, CV_8UC1) * 255;
            	return (false);
            }
		}


        bool
        erase_regions_of_no_interest2(std::vector<cv::Mat> sources, cv::Mat display, cv::Mat &mask)
        {
            std::vector<cv::Mat> color_channel(3);
            for (int i = 0; i < 3; i++)
            {
                cv::extractChannel(sources[0], color_channel[i], i);
                cv::bitwise_and(color_channel[i], mask, color_channel[i]);
            }
            cv::merge(color_channel, sources[0]);

            for (int i = 0; i < 3; i++)
            {
                cv::extractChannel(display, color_channel[i], i);
                cv::bitwise_and(color_channel[i], mask, color_channel[i]);
            }
            cv::merge(color_channel, display);

            return (true);
        }


        void 
        erase_depth_regions_of_no_interest(std::vector<cv::Mat> sources, cv::Mat display, float threshold_near, float threshold_far)
        {
            if (threshold_near < 0 && threshold_far > 10)
                return;

            cv::Mat mask = cv::Mat::zeros(sources[0].rows, sources[0].cols, CV_8UC1);            

            for (int i = 0; i < sources[1].rows; ++i)
            {
                for (int j = 0; j < sources[1].cols; ++j)
                {
                    if ((sources[1].at<unsigned short>(i,j) > (threshold_near * 1000)) && (sources[1].at<unsigned short>(i,j) < (threshold_far * 1000)))
                    {
                        mask.at<unsigned char>(i,j) = 255;
                    }
                    else
                    {
                        sources[1].at<unsigned short>(i,j) = 0;
                    }
                }
            }

            std::vector<cv::Mat> color_channel(3);
            for (int i = 0; i < 3; i++)
            {
                cv::extractChannel(sources[0], color_channel[i], i);
                cv::bitwise_and(color_channel[i], mask, color_channel[i]);
            }
            cv::merge(color_channel, sources[0]);

            if (*visualize_)
            {
                for (int i = 0; i < 3; i++)
                {
                    cv::extractChannel(display, color_channel[i], i);
                    cv::bitwise_and(color_channel[i], mask, color_channel[i]);
                }
                cv::merge(color_channel, display);
            }            

            // double min;
            // double max;         
            // cv::minMaxIdx(sources[1], &min, &max);
            // cv::Mat adjMap;
            // // expand your range to 0..255. Similar to histEq();
            // sources[1].convertTo(adjMap,CV_8UC1, 255 / (max-min), -min); 

            // // this is great. It converts your grayscale image into a tone-mapped one, 
            // // much more pleasing for the eye
            // // function is found in contrib module, so include contrib.hpp 
            // // and link accordingly
            // cv::Mat falseColorsMap;
            // cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);

            // cv::namedWindow("Color Display");
            // cv::imshow("Color Display", display);
            // cv::waitKey(1);

            // cv::namedWindow("Depth Display");
            // cv::imshow("Depth Display", falseColorsMap);
            // cv::waitKey(100000);

        }


		void
		visualize_results(int num_results, double max_similarity, cv::Mat display)
		{
            if (*verbose_ && (num_results > 0))
                std::cout << "detected objs  " << num_results << " / " << objs_.size() << "  max_similarity = " << max_similarity << std::endl;

            //publish the point clouds
        #if LINEMOD_VIZ_PCD
            pci_real_icpin_model2->publish();
            pci_real_icpin_model->publish();
            pci_real_icpin_ref->publish();
        #endif
        #if LINEMOD_VIZ_IMG
            if (*visualize_)
            {
                cv::namedWindow("OBJECT DETECTOR", cv::WINDOW_NORMAL);
                cv::imshow("OBJECT DETECTOR", display);
                cv::waitKey(100);
            }
        #endif
		}


		void
		save_object_database(std::vector<cv::Mat> sources, object_recognition_core::db::ObjData *best_match)
		{
            static int sample_number = 0;
            static int good_samples = 0;

            if (best_match == NULL)
                return;

            std::cout << "================= Sample evaluation. Press a key (1, 5 or x) and Enter\n";
            char ch;
            std::cin >> ch;
            switch (ch)
            {
                case '1': // save a good sample
                {
                    std::cout << "================= Saving a good sample\n";
                    std::stringstream file_name;
                    file_name << "image" << setfill('0') << setw(5) << sample_number << ".png";
                    std::cout << "image" << setfill('0') << setw(5) << sample_number << ".png,  ";
                    cv::imwrite(file_name.str().c_str(), sources[0]);
                    file_name.str("");
                    file_name << "depth" << setfill('0') << setw(5) << sample_number << ".png";
                    std::cout << "depth" << setfill('0') << setw(5) << sample_number << ".png,   ";
                    cv::imwrite(file_name.str().c_str(), sources[1]);

                    file_name.str("");
                    file_name << "pose" << setfill('0') << setw(5) << sample_number << ".yml";
                    std::cout << "pose" << setfill('0') << setw(5) << sample_number << ".yml\n";
                    cv::FileStorage pose_file(file_name.str().c_str(), cv::FileStorage::WRITE);
                    pose_file << "translation" << best_match->t;
                    pose_file << "rotation" << (cv::Mat) best_match->r;
                    pose_file.release();

                    good_samples++;
                    sample_number++;
                }
                break;

                case '5': // save a bad sample
                {
                    std::cout << "================= Saving a bad sample\n";
                    std::stringstream file_name;
                    file_name << "image" << setfill('0') << setw(5) << sample_number << ".png";
                    std::cout << "image" << setfill('0') << setw(5) << sample_number << ".png,  ";
                    cv::imwrite(file_name.str().c_str(), sources[0]);
                    file_name.str("");
                    file_name << "depth" << setfill('0') << setw(5) << sample_number << ".png";
                    std::cout << "depth" << setfill('0') << setw(5) << sample_number << ".png\n";
                    cv::imwrite(file_name.str().c_str(), sources[1]);

                    sample_number++;
                }
                break;
                default: // skip sample
                    std::cout << "================= Skip sample\n";
            }
            if (sample_number > 0)
                std::cout << "Current performance = %" << std::fixed << std::setw(3) << std::setprecision(1) <<
                    100.0 * (float) good_samples / (float) sample_number << std::endl;
            std::getchar();
            std::getchar();
		}

        cv::Matx33f
        find_min_symm_rotation(cv::Matx33f r1, cv::Matx33f r2, std::string obj_name) 
        {
            // Read in symmetries
            // Decompose rotation matrices into respective components (x,y,z)
            // Compare components individually
            // Compose new rotation matrix of individually minimal components

            char* base_dir = std::getenv("PRACSYS_PATH");
            std::string file_name(base_dir);
            file_name += ("/../rotational_symmetries.txt");
            std::ifstream file_in;
            file_in.open(file_name.c_str());
            std::string line, item;
            std::stringstream iff;
            int sym_x, sym_y, sym_z;

            while(std::getline(file_in,line))
            {
                iff << line;
                while (std::getline(iff, item, ' ')) 
                {
                    if (item == obj_name)
                    {
                        std::getline(iff,item,' ');
                        sym_x = atoi(item.c_str());
                        printf("symx: %s, length: %i\n", item.c_str(), strlen(item.c_str()));
                        std::getline(iff,item,' ');
                        sym_y = atoi(item.c_str());
                        printf("symy: %s, length: %i\n", item.c_str(), strlen(item.c_str()));
                        std::getline(iff,item);
                        sym_z = atoi(item.c_str());
                        printf("symz: %s, length: %i\n", item.c_str(), strlen(item.c_str()));
                        printf("Reading in %s symmetries: %i %i %i\n", obj_name.c_str(), sym_x, sym_y, sym_z);
                    }
                }
                iff.clear();
            }
            file_in.close();

            float r1_theta_x = atan2(r1(2,1),r1(2,2));
            float r1_theta_y = atan2(-r1(2,0),sqrt(r1(2,1)*r1(2,1) + r1(2,2)*r1(2,2)));
            float r1_theta_z = atan2(r1(1,0),r1(0,0));
            float r2_theta_x = atan2(r2(2,1),r2(2,2));
            float r2_theta_y = atan2(-r2(2,0),sqrt(r2(2,1)*r2(2,1) + r2(2,2)*r2(2,2)));
            float r2_theta_z = atan2(r2(1,0),r2(0,0));

            printf("+++++++++++++++++++++++++++++\n");
            printf("R1_theta_x: %f\n", r1_theta_x);
            printf("R1_theta_y: %f\n", r1_theta_y);
            printf("R1_theta_z: %f\n", r1_theta_z);
            printf("+++++++++++++++++++++++++++++\n");
            printf("R2_theta_x: %f\n", r2_theta_x);
            printf("R2_theta_y: %f\n", r2_theta_y);
            printf("R2_theta_z: %f\n", r2_theta_z);
            printf("+++++++++++++++++++++++++++++\n");
            printf("Theta_x delta: %f\n", (r1_theta_x - r2_theta_x));
            printf("Theta_y delta: %f\n", (r1_theta_y - r2_theta_y));
            printf("Theta_z delta: %f\n", (r1_theta_z - r2_theta_z));
            printf("=============================\n");


            printf("(abs(r1_theta_x - r2_theta_x): %f\n", abs(r1_theta_x - r2_theta_x));
            printf("(abs(r1_theta_x - r2_theta_x) > CV_PI / 2.0): %i\n", (abs(r1_theta_x - r2_theta_x) > CV_PI / 2.0));
            printf("(abs(r1_theta_x - r2_theta_x) < 3 * CV_PI / 2.0): %i\n", (abs(r1_theta_x - r2_theta_x) < 3 * CV_PI / 2.0));
            printf("+++++++++++++++++++++++++++++\n");
            printf("(abs(r1_theta_y - r2_theta_y): %f\n", abs(r1_theta_y - r2_theta_y));
            printf("(abs(r1_theta_y - r2_theta_y) > CV_PI / 2.0): %i\n", (abs(r1_theta_y - r2_theta_y) > CV_PI / 2.0));
            printf("(abs(r1_theta_y - r2_theta_y) < 3 * CV_PI / 2.0): %i\n", (abs(r1_theta_y - r2_theta_y) < 3 * CV_PI / 2.0));
            printf("+++++++++++++++++++++++++++++\n");
            printf("(abs(r1_theta_z - r2_theta_z): %f\n", abs(r1_theta_z - r2_theta_z));
            printf("(abs(r1_theta_z - r2_theta_z) > CV_PI / 2.0): %i\n", (abs(r1_theta_z - r2_theta_z) > CV_PI / 2.0));
            printf("(abs(r1_theta_z - r2_theta_z) < 3 * CV_PI / 2.0): %i\n", (abs(r1_theta_z - r2_theta_z) < 3 * CV_PI / 2.0));
            printf("=============================\n");

            if ((abs(r1_theta_x - r2_theta_x) > CV_PI / 2.0)
                && (abs(r1_theta_x - r2_theta_x) < 3 * CV_PI / 2.0)
                && (sym_x == 1))
            {
                if (r1_theta_x + CV_PI > CV_PI)
                    r1_theta_x = r1_theta_x - CV_PI;
                else
                    r1_theta_x = r1_theta_x + CV_PI;
            }
            if ((abs(r1_theta_y - r2_theta_y) > CV_PI / 2.0)
                && (abs(r1_theta_y - r2_theta_y) < 3 * CV_PI / 2.0)
                && (sym_y == 1))
            {
                if (r1_theta_y + CV_PI > CV_PI)
                    r1_theta_y = r1_theta_y - CV_PI;
                else
                    r1_theta_y = r1_theta_y + CV_PI;
            }
            if ((abs(r1_theta_z - r2_theta_z) > CV_PI / 2.0)
                && (abs(r1_theta_z - r2_theta_z) < 3 * CV_PI / 2.0)
                && (sym_z == 1))
            {
                if (r1_theta_z + CV_PI > CV_PI)
                    r1_theta_z = r1_theta_z - CV_PI;
                else
                    r1_theta_z = r1_theta_z + CV_PI;
            }

            if (sym_x == 2)
                r1_theta_x = r2_theta_x;
            if (sym_y == 2)
                r1_theta_y = r2_theta_y;
            if (sym_z == 2)
                r1_theta_z = r2_theta_z;

            printf("R1_theta_x: %f\n", r1_theta_x);
            printf("R1_theta_y: %f\n", r1_theta_y);
            printf("R1_theta_z: %f\n", r1_theta_z);
            printf("=============================\n");

            cv::Matx33f rotx(1,0,0,0,cos(r1_theta_x),-sin(r1_theta_x),0,sin(r1_theta_x),cos(r1_theta_x));
            cv::Matx33f roty(cos(r1_theta_y),0,sin(r1_theta_y),0,1,0,-sin(r1_theta_y),0,cos(r1_theta_y));
            cv::Matx33f rotz(cos(r1_theta_z),-sin(r1_theta_z),0,sin(r1_theta_z),cos(r1_theta_z),0,0,0,1);

            return rotz * roty * rotx;
        }

        double 
        get_rotational_distance(tf::Quaternion q1, tf::Quaternion q2)
        {
            double arc = 2.0 * q1.dot(q2) * q1.dot(q2) - 1.0;
            if (arc > 1.0)
                arc = 1.0;
            else if (arc < -1.0)
                arc = -1.0;

            double r_dist = 180.0 * acos(arc) / CV_PI;

            return r_dist;
        }

		void
		distance_between_objects(double &t_dist, double &r_dist, cv::Vec3f t1, cv::Matx33f r1, cv::Vec3f t2, cv::Matx33f r2, std::string obj_name)
		{
		    t_dist = norm(t1, t2, NORM_L2);

            tf::Matrix3x3 rot1 = get_tf_matrix(r1);
            tf::Matrix3x3 rot2 = get_tf_matrix(r2);
            tf::Matrix3x3 rot_min = get_tf_matrix(find_min_symm_rotation(r1,r2,obj_name));

            tf::Vector3 zero(0.0, 0.0, 0.0);
            tf::Transform tf_t1(rot1, zero);
            tf::Quaternion q1 = tf_t1.getRotation();
            tf::Transform tf_t2(rot2, zero);
            tf::Quaternion q2 = tf_t2.getRotation();
            tf::Transform tf_min(rot_min, zero);
            tf::Quaternion qmin = tf_min.getRotation();

            double direct_comp_dist = get_rotational_distance(q1,q2);
            double min_symm_dist = get_rotational_distance(qmin,q2);

            printf("Rot Dist (q1,q2): %f\n", direct_comp_dist);
            printf("Rot Dist (qmin,q2): %f\n", min_symm_dist);

            cv::waitKey(100000);

            if (abs(min_symm_dist) < abs(direct_comp_dist))
                r_dist = min_symm_dist;
            else
            {
                printf("Something's wrong here...");
                r_dist = direct_comp_dist;       
            }
		}

        void
        distance_between_objects(double &t_dist, double &r_dist, cv::Vec3f t1, cv::Matx33f r1, cv::Vec3f t2, cv::Matx33f r2)
        {
            t_dist = norm(t1, t2, NORM_L2);

            tf::Matrix3x3 rot1 = get_tf_matrix(r1);
            tf::Matrix3x3 rot2 = get_tf_matrix(r2);
            tf::Vector3 zero(0.0, 0.0, 0.0);
            tf::Transform tf_t1(rot1, zero);
            tf::Quaternion q1 = tf_t1.getRotation();
            tf::Transform tf_t2(rot2, zero);
            tf::Quaternion q2 = tf_t2.getRotation();
            r_dist = get_rotational_distance(q1,q2);
        }

		void
		reevaluate_object_database(std::vector<cv::Mat> sources, object_recognition_core::db::ObjData *best_match,
		        cv::Vec3f previous_t, cv::Matx33f previous_r, bool previous_pose_available)
		{
            static int sample_number = 0;
            static int good_samples = 0;

            std::cout << "================= Sample " << sample_number << " evaluation.\n";
            if (best_match != NULL)
            {
                if (previous_pose_available)
                {
                    double dist;
                    double theta;
                    distance_between_objects(dist, theta, best_match->t, best_match->r, previous_t, previous_r);
                    std::cout << "distance to previous pose = " << dist << "   angle = " << theta << std::endl;
                }
                else
                    std::cout << "no previous pose available" << std::endl;

                std::cout << "================= Press a key (1, 5 or x) and Enter\n";

                char ch;
                std::cin >> ch;
                switch (ch)
                {
                    case '1': // save a good sample
                    {
                        if (!previous_pose_available)
                        {
                            std::cout << "================= Saving a good sample\n";
                            std::stringstream file_name;
                            file_name << "pose" << setfill('0') << setw(5) << sample_number << ".yml";
                            std::cout << "pose" << setfill('0') << setw(5) << sample_number << ".yml\n";
                            cv::FileStorage pose_file(file_name.str().c_str(), cv::FileStorage::WRITE);
                            pose_file << "translation" << best_match->t;
                            pose_file << "rotation" << (cv::Mat) best_match->r;
                            pose_file.release();
                        }

                        good_samples++;
                    }
                    break;
                    case '5': // save a bad sample
                    {
                        if (previous_pose_available)
                        {
                            std::cout << "================= Deleting a good sample\n";
                            std::stringstream file_name;
                            file_name << "pose" << setfill('0') << setw(5) << sample_number << ".yml";
                            std::cout << "pose" << setfill('0') << setw(5) << sample_number << ".yml\n";
                            std::remove(file_name.str().c_str());
                        }
                    }
                    break;

                    default: // skip sample
                        std::cout << "================= Skip sample\n";
                }
            }

            sample_number++;

            if (sample_number > 0)
                std::cout << "Current performance = %" << std::fixed << std::setw(3) << std::setprecision(1) <<
                    100.0 * (float) good_samples / (float) sample_number << std::endl;
            std::getchar();
            std::getchar();


		}


        std::vector<cv::Mat>
        get_sources_from_files(cv::Mat *display, cv::Vec3f *previous_t, cv::Matx33f *previous_r, bool *previous_pose_available)
        {
            static int sample_number = 0;
            std::stringstream file_name;
            std::vector<cv::Mat> sources;

            std::cout << "================= Reading a samples from files\n";

            file_name << "image" << setfill('0') << setw(5) << sample_number << ".png";
            std::cout << "image" << setfill('0') << setw(5) << sample_number << ".png,  ";
            std::ifstream infile(file_name.str().c_str());
            cv::Mat image;
            if (infile.good())
                image = cv::imread(file_name.str().c_str(), CV_LOAD_IMAGE_COLOR);
            else
            {
                std::cout << "===================== File " << file_name.str().c_str()
                          << " does not exist. End of database.\n Press Enter to exit." << std::endl;

                std::getchar();
                exit(0);
            }
            file_name.str("");

            cv::imshow("Previous Image", image);
            cv::waitKey(1);

            file_name << "depth" << setfill('0') << setw(5) << sample_number << ".png";
            std::cout << "depth" << setfill('0') << setw(5) << sample_number << ".png,   ";
            cv::Mat depth = cv::imread(file_name.str().c_str(), CV_16UC1);
            sources.push_back(image);
            sources.push_back(depth);
            image.copyTo(*display);

            file_name.str("");
            file_name << "pose" << setfill('0') << setw(5) << sample_number << ".yml";
            std::cout << "pose" << setfill('0') << setw(5) << sample_number << ".yml\n";
            std::ifstream infile2(file_name.str().c_str());
            if (infile2.good())
            {
                cv::FileStorage pose_file(file_name.str().c_str(), cv::FileStorage::READ);
                pose_file["translation"] >> *previous_t;
                cv::Mat rotation;
                pose_file["rotation"] >> rotation;
                *previous_r = (cv::Matx33f) rotation;
                pose_file.release();

                *previous_pose_available = true;
            }
            else
                *previous_pose_available = false;

            sample_number++;

            return (sources);
        }


        std::vector<cv::linemod::Match>
        linemod_detect_with_threshold_refining(std::vector<cv::Mat> sources)
        {
#define		MAX_THRESHOLD 97.0
#define		MIN_THRESHOLD 50.0
            float previous_max = MAX_THRESHOLD;
            float previous_min = MIN_THRESHOLD;
            std::vector<cv::linemod::Match> matches;
            do
            {
            	std::cout << "Threshold refining" << std::endl;
                detector_->match(sources, *threshold_, matches);
                std::cout << "Current Threshold: " << *threshold_ << "   matches: " << matches.size() << "\n";
                if (matches.size() < 15)
                {
                    std::cout << "\n Detector adjusting threshold to allow for MORE matches\n";
                    previous_max = *threshold_;

                    *threshold_ = previous_min + (*threshold_ - previous_min) / 2.0;
                }
                else if (matches.size() > 30)
                {
                    std::cout << "\n Detector adjusting threshold to allow for LESS matches\n";
                    previous_min = *threshold_;

                    *threshold_ = previous_max - (previous_max - *threshold_) / 2.0;
                }
            } while (((matches.size() < 15) || (matches.size() > 30)) &&
            		(*threshold_ < MAX_THRESHOLD) && (*threshold_ > MIN_THRESHOLD) &&
                    (abs(previous_max - previous_min) > 0.5));

            return (matches);
        }


        void
        save_object_pose_data(std::string prefix, std::string object_name, std::string file_name_sufix,
                cv::Vec3f t, cv::Matx33f r, tf::Transform camera_pose, int template_id)
        {
            std::cout << "$$$$$$$$$$$$$$$$ Saving a sample pose\n";

            std::stringstream file_name;
            file_name << prefix << object_name << "-pose" << file_name_sufix << ".yml";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::FileStorage pose_file(file_name.str().c_str(), cv::FileStorage::WRITE);
            pose_file << "object_translation_wrt_camera" << t;
            pose_file << "object_rotation_wrt_camera" << (cv::Mat) r;

            tf::Transform object_transform = get_tf_tranform_from_cv_t_r(t, r);
            tf::Transform object_transform_with_respect_to_base = get_object_transform_with_respect_to_base(
                    object_transform, camera_pose);
            get_cv_t_r_from_tf_tranform(t, r, object_transform_with_respect_to_base);
            pose_file << "object_translation_wrt_base" << t;
            pose_file << "object_rotation_wrt_base" << (cv::Mat) r;

            get_cv_t_r_from_tf_tranform(t, r, camera_pose);
            pose_file << "camera_translation_wrt_base" << t;
            pose_file << "camera_rotation_wrt_base" << (cv::Mat) r;

            pose_file << "template_id" << template_id;

            pose_file.release();
        }


        void
        save_data_gathered(cv::Mat image, cv::Mat depth,
                object_recognition_core::db::ObjData *best_match, cv::Mat region_of_interest)
        {
            static bool all_frames_captured = false;

            if (g_frames_to_capture > 0)
                all_frames_captured = false;

            if ((g_frames_to_capture < 0) || (all_frames_captured == true))
                return;

            int state;
            if (g_decision_making_state == Move_to_Detect1)
                state = 1;
            else if (g_decision_making_state == Move_to_Detect2)
                state = 2;
            else if (g_decision_making_state == Map_and_Detect_Object)
                state = 3;
            else
                return;

            std::cout << "================= Saving a sample\n";

            int sample_number = NUM_FRAMES_PER_CAPTURE - g_frames_to_capture;
            std::stringstream file_name_sufix;
            file_name_sufix << "-" << (char) g_bin_id << "-" << g_item_counts << "-" << state << "-" << sample_number;
            std::string prefix = "data_gathered/";

            std::stringstream file_name;
            file_name << prefix << g_current_object_of_interest << "-image" << file_name_sufix.str() << ".png";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::imwrite(file_name.str().c_str(), image);
            file_name.str("");
            file_name << prefix << g_current_object_of_interest << "-mask" << file_name_sufix.str() << ".png";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::imwrite(file_name.str().c_str(), region_of_interest);
            file_name.str("");
            file_name << prefix << g_current_object_of_interest << "-depth" << file_name_sufix.str() << ".png";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::imwrite(file_name.str().c_str(), depth);

            std::cout<<" Depth image: "<<file_name.str().c_str()<<"\n";

            cv::Vec3f t;
            cv::Matx33f r;
            tf::Transform best_object_pose_with_respect_to_camera;
            int best_template_id;
            if ((g_frames_to_capture == 0) && get_best_pose(best_object_pose_with_respect_to_camera, best_template_id))
            {
                get_cv_t_r_from_tf_tranform(t, r, best_object_pose_with_respect_to_camera);
            }
            else if (best_match == NULL)
            {
                cv::Vec3f default_t(0.0, 0.0, 0.6);
                cv::Matx33f default_r(1.0, 0.0, 0.0,
                              0.0, 1.0, 0.0,
                              0.0, 0.0, 1.0);
                t = default_t;
                r = default_r;
                best_template_id = 0;
            }
            else
            {
                t = best_match->t;
                r = best_match->r;
                best_template_id = best_match->template_id;
            }

            save_object_pose_data(prefix, g_current_object_of_interest, file_name_sufix.str(),
                    t, r, g_camera_pose, best_template_id);

            if (g_frames_to_capture == 0)
                all_frames_captured = true;
        }


        struct ObjectData
        load_data_gathered(std::vector<cv::Mat> &sources,
                cv::Mat &region_of_interest, cv::Mat &original_image, cv::Mat &display,
                std::string object_name, char bin_id, int item_counts, int state, int sample_number)
        {
            std::cout << "================= Loading a new sample\n";

            std::stringstream file_name_sufix;
            file_name_sufix << "-" << bin_id << "-" << item_counts << "-" << state << "-" << sample_number;
            std::string prefix = "data_gathered/";

            std::stringstream file_name;
            file_name << prefix << object_name << "-image" << file_name_sufix.str() << ".png";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::Mat image = cv::imread(file_name.str().c_str(), CV_LOAD_IMAGE_COLOR);
            if (!image.data)
                std::cout <<  "Could not open or find the file " << file_name.str() << std::endl;

            file_name.str("");
            file_name << prefix << object_name << "-mask" << file_name_sufix.str() << ".png";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            region_of_interest = cv::imread(file_name.str().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
            if (!region_of_interest.data)
                std::cout <<  "Could not open or find the file " << file_name.str() << std::endl;

            file_name.str("");
            file_name << prefix << object_name << "-depth" << file_name_sufix.str() << ".png";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::Mat depth = cv::imread(file_name.str().c_str(), CV_16UC1);
            if (!depth.data)
                std::cout <<  "Could not open or find the file " << file_name.str() << std::endl;

            sources.clear();
            sources.push_back(image);
            sources.push_back(depth);
            image.copyTo(display);
            image.copyTo(original_image);

            file_name.str("");
            file_name << prefix << object_name << "-pose" << file_name_sufix.str() << ".yml";
            std::cout << "@@@@@===@@@@ " << file_name.str() << std::endl;
            cv::FileStorage pose_file(file_name.str().c_str(), cv::FileStorage::READ);
            struct ObjectData data;
            if (pose_file.isOpened())
            {
                pose_file["object_translation_wrt_camera"] >> data.object_translation_wrt_camera;
                pose_file["object_rotation_wrt_camera"] >> data.object_rotation_wrt_camera;

                pose_file["object_translation_wrt_base"] >> data.object_translation_wrt_base;
                pose_file["object_rotation_wrt_base"] >> data.object_rotation_wrt_base;

                pose_file["camera_translation_wrt_base"] >> data.camera_translation_wrt_base;
                pose_file["camera_rotation_wrt_base"] >> data.camera_rotation_wrt_base;

                pose_file["template_id"] >> data.template_id;

                pose_file.release();
            }
            else
                std::cout <<  "Could not open or find the file " << file_name.str() << std::endl;

            return (data);
        }


        struct data_gathered_st *
        read_data_gathered_file(int *num_samples)
        {
            static struct data_gathered_st *data;
            FILE *data_file = fopen("data_gathered.txt", "r");
            if (data_file)
            {
                int num_lines = 0;
                char line[1000];
                while (fgets(line, 999, data_file))
                    num_lines++;
                rewind(data_file);

                data = (struct data_gathered_st *) malloc(num_lines * sizeof(struct data_gathered_st));
                for (int i = 0; i < num_lines; i++)
                {
                    if (fgets(line, 999, data_file) == NULL)
                    {
                        printf("Error: could not read line from data_gathered.txt in read_data_gathered_file()\n");
                        exit(1);
                    }
                    char *object_name = strtok(line, "-");
                    char *file_type = strtok(NULL, "-");
                    char *bin_id = strtok(NULL, "-");
                    char *item_counts = strtok(NULL, "-");
                    char *state = strtok(NULL, "-");
                    char *sample_number = strtok(NULL, "-");
                    strcpy(data[i].object_name, object_name);
                    data[i].bin_id = bin_id[0];
                    data[i].item_counts = atoi((const char *) item_counts);
                    data[i].state = atoi((const char *) state);
                    data[i].sample_number = atoi((const char *) sample_number);
                    printf("object_name = %s, bin_id = %c, item_counts = %d, state = %d, sample_number = %d\n",
                            data[i].object_name, data[i].bin_id, data[i].item_counts, data[i].state, data[i].sample_number);
                }
                *num_samples = num_lines;
                fclose(data_file);
            }
            else
            {
                printf("Error: Could not find file data_gathered.txt\n");
                exit(1);
            }
            return (data);
        }


        void
        move_point_cloud_for_better_visualization(cv::Mat_<cv::Vec3f> &model_depth_3D, cv::Matx33f R)
        {
            cv::Vec3f T(0.0, 0.0, -0.25);
            cv::Vec3f centroid;

            get_centroid(model_depth_3D, centroid);

            for (size_t y = 0; y < model_depth_3D.rows; ++y)
                for (size_t x = 0; x < model_depth_3D.cols; ++x)
                {
                    model_depth_3D(y, x) -= centroid;
                    // model_depth_3D(y, x) = R * model_depth_3D(y, x);
                    model_depth_3D(y, x) = model_depth_3D(y, x) + T + centroid;
                }
        }


        cv::Mat_<cv::Vec3f>
        get_model_point_cloud_and_image(cv::Mat &object_model_image, std::string object_name,
                int template_id, cv::Matx33d R)
        {
            static std::string previous_objet_name = "";

            if (object_name != previous_objet_name)
            {
                *detector_ = detectors_.at(object_name);
                if (detector_->classIds().empty())
                {
                    printf("Error: Could not setup renderer in get_model_point_cloud() for object %s", object_name.c_str());
                    exit(1);
                }
                // Remove previous version of the model
                renderer_iterators_.erase(detector_->classIds()[0]);
                // Create a new one
                setupRenderer(detector_->classIds()[0]);

                previous_objet_name = object_name;
            }
            //get the point cloud of the rendered object model
            RendererIterator *it_r = renderer_iterators_.at(get_object_id_by_object_name(object_name));
            cv::Mat object_model_depth, mask;
            cv::Rect rect;

            cv::Matx33d R_match = Rs_.at(get_object_id_by_object_name(object_name))[template_id].clone();
            cv::Matx33d R_temp(R_match.inv());
            cv::Vec3d up(-R_temp(0,1), -R_temp(1,1), -R_temp(2,1));

            cv::Vec3d T_match = Ts_.at(get_object_id_by_object_name(object_name))[template_id].clone();

            it_r->renderDepthOnly(object_model_depth, mask, rect, -T_match, up);
            it_r->renderImageOnly(object_model_image, rect, -T_match, up);

            cv::Mat K_match = Ks_.at(get_object_id_by_object_name(object_name))[template_id];
            cv::Mat_<cv::Vec3f> model_depth_3D;
            cv::depthTo3d(object_model_depth, K_match, model_depth_3D);

            move_point_cloud_for_better_visualization(model_depth_3D, R);

            return (model_depth_3D);
        }


        void
        publish_ork_model(cv::Vec3d T, cv::Matx33d R, std::string object_id)
        {
            // 180degree rotation about the Y axis to fix detection rotation issues
            cv::Matx33d flip(-1.f,0.f,0.f,0.f,1.f,0.f,0.f,0.f,-1.f);
            cv::Mat adj_r = cv::Mat(R * flip);

            //return the outcome object pose
            PoseResult pose_result;
            pose_result.set_object_id(db_, object_id);
            pose_result.set_confidence(1.0);
            pose_result.set_R(adj_r);
            pose_result.set_T(cv::Mat(T));
            pose_results_->push_back(pose_result);
        }


        cv::Matx33d
        rotx(double theta)
        {
            double c = cos(theta);
            double s = sin(theta);

            return (cv::Matx33d(
                 1,  0,  0,
                 0,  c, -s,
                 0,  s,  c));
        }


        cv::Matx33d
        roty(double theta)
        {
            double c = cos(theta);
            double s = sin(theta);

            return (cv::Matx33d(
                  c,  0,  s,
                  0,  1,  0,
                 -s,  0,  c));
        }


        cv::Matx33d
        rotz(double theta)
        {
            double c = cos(theta);
            double s = sin(theta);

            return (cv::Matx33d(
                  c, -s,  0,
                  s,  c,  0,
                  0,  0,  1));
        }


        void
        display_sample_information(struct ObjectData object_data,
                struct data_gathered_st *data, std::vector<cv::Mat> sources,
                cv::Mat region_of_interest, cv::Mat original_image, cv::Mat display,
                cv::Vec3f mT, cv::Matx33d rotation, cv::Matx33d R, int current_data_index)
        {
            erase_regions_of_no_interest2(sources, display, region_of_interest);
            //erase_depth_regions_of_no_interest(sources, display, *depth_near_threshold, *depth_far_threshold);

            // Show sample image
            cv::namedWindow("OBJECT DETECTOR", cv::WINDOW_NORMAL);
            cv::imshow("OBJECT DETECTOR", display);

            // Show sample depth (rviz)
            cv::Mat_<cv::Vec3f> depth_snapshot;
            cv::Mat_<float> K;
            K_depth_->convertTo(K, CV_32F);
            cv::depthTo3d(sources[1], K, depth_snapshot);
            pci_real_icpin_model->clear();
            std::vector<cv::Vec3f> std_vector_cloud;
            fill_in_std_vector_cloud2(std_vector_cloud, depth_snapshot);
            pci_real_icpin_model->fill(std_vector_cloud, original_image);
            pci_real_icpin_model->publish();

            // Show object model (rviz)
            cv::Mat model_image;
            cv::Mat_<cv::Vec3f> model_depth_3D = get_model_point_cloud_and_image(model_image,
                    data[current_data_index].object_name, object_data.template_id, rotation);
            std::vector<cv::Vec3f> std_vector_model_depth_3D;
            fill_in_std_vector_cloud2(std_vector_model_depth_3D, model_depth_3D);
            pci_real_icpin_model2->fill(std_vector_model_depth_3D, model_image);
            pci_real_icpin_model2->publish();

            // Publish ork object (rviz)
            pose_results_->clear();
            publish_ork_model(object_data.object_translation_wrt_camera + mT, R,
                    get_object_id_by_object_name(data[current_data_index].object_name));
        }


        void
        ground_truth_generation()
        {
            static struct data_gathered_st *data = NULL;
            static int current_data_index = 0;
            static int previous_data_index = -1;
            static int num_samples;

            static cv::Vec3f mT(0.0, 0.0, 0.0);
            static cv::Vec3f mR(0.0, 0.0, 0.0);
            static struct ObjectData object_data;
            static cv::Vec3f mT_copy(0.0, 0.0, 0.0);
            static cv::Vec3f mR_copy(0.0, 0.0, 0.0);
            static struct ObjectData object_data_copy;
            static bool have_copy = false;

            static cv::Mat region_of_interest, original_image, display;
            static std::vector<cv::Mat> sources;

            pci_real_icpin_model2->clear();
            pci_real_icpin_model->clear();
            pci_real_icpin_ref->clear();

            if (data == NULL)
                data = read_data_gathered_file(&num_samples);

            // Read input from files
            if (previous_data_index != current_data_index)
                object_data = load_data_gathered(sources, region_of_interest, original_image, display,
                        data[current_data_index].object_name,
                        data[current_data_index].bin_id, data[current_data_index].item_counts,
                        data[current_data_index].state, data[current_data_index].sample_number);
            previous_data_index = current_data_index;

            cv::Matx33d rotation = rotx(mR[0]) * roty(mR[1]) * rotz(mR[2]);
            cv::Matx33d R = cv::Matx33d(object_data.object_rotation_wrt_camera) * rotation;
            // function publishes pointclouds, ork object
            display_sample_information(object_data, data, sources,
                    region_of_interest, original_image, display,
                    mT, rotation, R, current_data_index);

            // User interface
            char key = cv::waitKey(10);
            switch (key)
            {
            case 'n': // next object
                current_data_index += 1;
                if (current_data_index == num_samples)
                    current_data_index = 0;
                mT = cv::Vec3f(0.0, 0.0, 0.0);
                mR = cv::Vec3f(0.0, 0.0, 0.0);
                break;
            case 'b': // previous object
                current_data_index -= 1;
                if (current_data_index < 0)
                    current_data_index = num_samples - 1;
                mT = cv::Vec3f(0.0, 0.0, 0.0);
                mR = cv::Vec3f(0.0, 0.0, 0.0);
                break;

            case 'N': // save and go to next object
                {
                    std::stringstream file_name_sufix;
                    file_name_sufix << "-" << data[current_data_index].bin_id << "-"
                            << data[current_data_index].item_counts << "-"
                            << data[current_data_index].state << "-"
                            << data[current_data_index].sample_number;
                    std::string object_name = data[current_data_index].object_name;
                    save_object_pose_data("data_gathered/", object_name,
                            file_name_sufix.str(),
                            object_data.object_translation_wrt_camera + mT, cv::Matx33f(R),
                            get_tf_tranform_from_cv_t_r(object_data.camera_translation_wrt_base, object_data.camera_rotation_wrt_base),
                            object_data.template_id);

                    current_data_index += 1;
                    if (current_data_index == num_samples)
                        current_data_index = 0;
                    mT = cv::Vec3f(0.0, 0.0, 0.0);
                    mR = cv::Vec3f(0.0, 0.0, 0.0);
                }

            case 'y': // save
                {
                    std::stringstream file_name_sufix;
                    file_name_sufix << "-" << data[current_data_index].bin_id << "-"
                            << data[current_data_index].item_counts << "-"
                            << data[current_data_index].state << "-"
                            << data[current_data_index].sample_number;
                    std::string object_name = data[current_data_index].object_name;
                    save_object_pose_data("data_gathered/", object_name,
                            file_name_sufix.str(),
                            object_data.object_translation_wrt_camera + mT, cv::Matx33f(R),
                            get_tf_tranform_from_cv_t_r(object_data.camera_translation_wrt_base, object_data.camera_rotation_wrt_base),
                            object_data.template_id);
                }
                break;

            case 'c': // copy
                object_data_copy = object_data;
                mT_copy = mT;
                mR_copy = mR;
                have_copy = true;
                break;
            case 'p': // paste
                if (have_copy)
                {
                    // paste OBJ wrt BASE; keep CAMERA wrt BASE
                    object_data.object_translation_wrt_base = object_data_copy.object_translation_wrt_base;
                    object_data.object_rotation_wrt_base = object_data_copy.object_rotation_wrt_base;

                    mT = mT_copy;
                    mR = mR_copy;

                    // calculate OBJ wrt CAMERA
                    cv::Mat translation;
                    translation = cv::Mat(object_data.object_translation_wrt_base - object_data.camera_translation_wrt_base);
                    translation = translation.t() * object_data.camera_rotation_wrt_base;

                    object_data.object_translation_wrt_camera = cv::Vec3f(translation.at<float>(0,0), 
                        translation.at<float>(0,1), translation.at<float>(0,2));
                    object_data.object_rotation_wrt_camera = object_data.camera_rotation_wrt_base.t() * object_data.object_rotation_wrt_base;

                }
                break;

            case 'i': // y+
                mT[1] += 0.002;
                break;
            case 'm': // y-
                mT[1] -= 0.002;
                break;
            case 'j': // x+
                mT[0] += 0.002;
                break;
            case 'k': // x-
                mT[0] -= 0.002;
                break;
            case 'l': // z+
                mT[2] += 0.002;
                break;
            case 'h': // z-
                mT[2] -= 0.002;
                break;

            case 'e': // y+
                mR[1] += (1.0 / 180.0) * CV_PI;
                break;
            case 'x': // y-
                mR[1] -= (1.0 / 180.0) * CV_PI;
                break;
            case 's': // x+
                mR[0] += (1.0 / 180.0) * CV_PI;
                break;
            case 'd': // x-
                mR[0] -= (1.0 / 180.0) * CV_PI;
                break;
            case 'f': // z+
                mR[2] += (1.0 / 180.0) * CV_PI;
                break;
            case 'a': // z-
                mR[2] -= (1.0 / 180.0) * CV_PI;
                break;

            case 'E': // y+
                mR[1] += (90.0 / 180.0) * CV_PI;
                break;
            case 'X': // y-
                mR[1] -= (90.0 / 180.0) * CV_PI;
                break;
            case 'S': // x+
                mR[0] += (90.0 / 180.0) * CV_PI;
                break;
            case 'D': // x-
                mR[0] -= (90.0 / 180.0) * CV_PI;
                break;
            case 'F': // z+
                mR[2] += (90.0 / 180.0) * CV_PI;
                break;
            case 'A': // z-
                mR[2] -= (90.0 / 180.0) * CV_PI;
                break;
            }
        }


        std::vector<cv::Mat>
        get_sources_from_groud_truth(struct ObjectData &object_data, struct data_gathered_st &object_info,
                cv::Mat &display, cv::Mat &region_of_interest)
        {
            static struct data_gathered_st *data = NULL;
            static int current_data_index = 0;
            static int num_samples = -1;

            cv::Mat original_image;
            std::vector<cv::Mat> sources;

            if (data == NULL)
            {
                data = read_data_gathered_file(&num_samples);
                g_object_detection_evaluation_results_file = fopen("object_detection_performance.txt", "w");
            }

            if (current_data_index == num_samples)
            {
                std::cout << "All samples evaluated. Goodbye." << std::endl;
                fclose(g_object_detection_evaluation_results_file);
                exit (0);
            }

            // Read input from files
            object_data = load_data_gathered(sources, region_of_interest, original_image, display,
                    data[current_data_index].object_name,
                    data[current_data_index].bin_id, data[current_data_index].item_counts,
                    data[current_data_index].state, data[current_data_index].sample_number);

            object_info = data[current_data_index];
            g_current_object_of_interest = data[current_data_index].object_name;
            g_bin_id = data[current_data_index].bin_id;
            g_item_counts = data[current_data_index].item_counts;
            g_robot_arm = LEFT_ARM;
            g_camera_pose = get_tf_tranform_from_cv_t_r(object_data.camera_translation_wrt_base, cv::Matx33f(object_data.camera_rotation_wrt_base));

            if (data[current_data_index].state == 1)
                g_decision_making_state = Move_to_Detect1;
            else if (data[current_data_index].state == 2)
                g_decision_making_state = Move_to_Detect2;
            else if (data[current_data_index].state == 3)
                g_decision_making_state = Map_and_Detect_Object;

            current_data_index += 1;

            return (sources);
        }


        void
        save_object_detection_evaluation_results(struct data_gathered_st object_info, struct ObjectData object_data,
                object_recognition_core::db::ObjData *best_match, double max_similarity)
        {
            double t_dist, r_dist;
#define USE_STATISTICS
#ifdef  USE_STATISTICS


            if ((object_info.state == 1) && (object_info.sample_number == 0))
                clear_statistics();

            if ((max_similarity != 0.0) && (best_match != NULL))
                add_pose_to_statistics(best_match->t, best_match->r, max_similarity, object_data.template_id);

            int best_template_id;
            tf::Transform transform;
            if (get_best_pose(transform, best_template_id))
            {
                cv::Vec3f t;
                cv::Matx33f r;
                get_cv_t_r_from_tf_tranform(t, r, transform);
                // REMOVE printf
                printf("Object: %s-%i-%i\n", object_info.object_name, object_info.state, object_info.sample_number);
                distance_between_objects(t_dist, r_dist,
                        object_data.object_translation_wrt_camera, cv::Matx33f(object_data.object_rotation_wrt_camera),
                        t, r, (std::string) object_info.object_name);
            }
            else // failed to detect
            {
                t_dist = 1.0;
                r_dist = 45.0;
            }

            if ((object_info.state == 3) && (object_info.sample_number == 3))
                fprintf(g_object_detection_evaluation_results_file,
                    "object_name = %s, bin_id = %c, item_counts = %d, state = %d, sample_number = %d, t_dist = %2.3f, r_dist = %3.2f\n",
                    object_info.object_name, object_info.bin_id, object_info.item_counts,
                    object_info.state, object_info.sample_number,
                    t_dist, r_dist);
#else
            if ((max_similarity != 0.0) && (best_match != NULL))
            {
                distance_between_objects(t_dist, r_dist,
                        object_data.object_translation_wrt_camera, cv::Matx33f(object_data.object_rotation_wrt_camera),
                        best_match->t, best_match->r, (std::string) object_info.object_name);
            }
            else // failed to detect
            {
                t_dist = 1.0;
                r_dist = 45.0;
            }
            fprintf(g_object_detection_evaluation_results_file,
                    "object_name = %s, bin_id = %c, item_counts = %d, state = %d, sample_number = %d, t_dist = %2.3f, r_dist = %3.2f\n",
                    object_info.object_name, object_info.bin_id, object_info.item_counts,
                    object_info.state, object_info.sample_number,
                    t_dist, r_dist);
#endif
        }


        int
        process(const tendrils& inputs, const tendrils& outputs)
        {
            if (g_ground_truth_generation)
            {
                ground_truth_generation();
                return (ecto::OK);
            }
            // Get source depth and image
            cv::Mat display;
            std::vector<cv::Mat> sources;
            cv::Mat region_of_interest;
            bool have_pose_hint;
            struct ObjectData object_data;
            struct data_gathered_st object_info;
            if (g_object_detection_evaluation){
                ground_truth_generation();
                sources = get_sources_from_groud_truth(object_data, object_info, display, region_of_interest);
            }
            else
                sources = get_sources(&display);

            if (!ready_to_process())
                return (ecto::OK);

            // Erase regions of no interest
            cv::Mat original_image;
            sources[0].copyTo(original_image);
            if (g_object_detection_evaluation)
                have_pose_hint = erase_regions_of_no_interest2(sources, display, region_of_interest);
            else
                have_pose_hint = erase_regions_of_no_interest(sources, display, region_of_interest);
            erase_depth_regions_of_no_interest(sources, display, *depth_near_threshold, *depth_far_threshold);

            // Perform the actual linemod detection process
            clock_t tStart = clock();
            std::vector<cv::linemod::Match> matches = linemod_detect_with_threshold_refining(sources);
            double tMatch = (double)(clock() - tStart) / CLOCKS_PER_SEC;
            std::cout << "Matching(sec): " << tMatch << std::endl;

            // Post-processing
            fill_in_the_buffer_of_detected_objects(matches, sources, display);
            double tProcess = (double)((clock() - tStart) / CLOCKS_PER_SEC) - tMatch;
            std::cout << "Processing(sec): " << tProcess << std::endl;

            double max_similarity;
            object_recognition_core::db::ObjData *best_match;
            if (have_pose_hint)
                best_match = process_with_object_pose_hint(&max_similarity);
            else
                best_match = process_without_object_pose_hint(&max_similarity);
            double tPose = (double)((clock() - tStart)/CLOCKS_PER_SEC) - tMatch - tProcess;
            std::cout << "Pose(sec): " << tPose << std::endl;

            // Visualization
            visualize_results(matches.size(), max_similarity, display);

            if (g_data_gathering)
                save_data_gathered(original_image, sources[1], best_match, region_of_interest);

            if (g_object_detection_evaluation)
                save_object_detection_evaluation_results(object_info, object_data, best_match, max_similarity);

            return ecto::OK;
        }

        /** LINE-MOD detector */
        cv::Ptr<cv::linemod::Detector> detector_;
        std::map< std::string, cv::linemod::Detector > detectors_;
        // Parameters
        spore<float> threshold_;
        // Inputs
        spore<cv::Mat> color_, depth_;
        /** The calibration matrix of the camera */
        spore<cv::Mat> K_depth_;

        /** The buffer with detected objects and their info */
        std::vector <object_recognition_core::db::ObjData> objs_;

        /** True or False to output debug image */
        ecto::spore<bool> visualize_;
        /** True or False to use input rgb image */
        ecto::spore<bool> use_rgb_;
        /** True or False to use input depth image */
        ecto::spore<bool> use_depth_;
        /** Threshold on minimal distance between detected objects */
        ecto::spore<float> th_obj_dist_;
        /** True or False to output debug log */
        ecto::spore<bool> verbose_;
        /** The depth camera frame id*/
        ecto::spore<std::string> depth_frame_id_;
        /** The minimal accepted icp distance*/
        ecto::spore<float> icp_dist_min_;
        /** The minimal percentage of pixels with matching depth*/
        ecto::spore<float> px_match_min_;
        /** The object recognition results */
        ecto::spore<std::vector<PoseResult> > pose_results_;
        /** The rotations, per object and per template */
        std::map<std::string, std::vector<cv::Mat> > Rs_;
        /** The translations, per object and per template */
        std::map<std::string, std::vector<cv::Mat> > Ts_;
        /** The objects distances, per object and per template */
        std::map<std::string, std::vector<float> > distances_;
        /** The calibration matrices, per object and per template */
        std::map<std::string, std::vector<cv::Mat> > Ks_;
        /** The renderer initialized with objects meshes, per object*/
        std::map<std::string, RendererIterator*> renderer_iterators_;
        /** The weight applied to linemod similarity measure*/
        ecto::spore<float> linemod_similarity_weight;
        /** The weight applied to histogram similarity measure*/
        ecto::spore<float> histogram_similarity_weight;
        /** The top N selections for the detections*/
        ecto::spore<float> TOP_N;
        /** the near distance mask parameter */
        ecto::spore<float> depth_near_threshold;
        /** the far distance mask parameter */
        ecto::spore<float> depth_far_threshold;

        /** Renderer parameter: the number of points on the sphere */
        int renderer_n_points_;
        /** Renderer parameter: the angle step sampling in degrees*/
        int renderer_angle_step_;
        /** Renderer parameter: the minimum scale sampling*/
        double renderer_radius_min_;
        /** Renderer parameter: the maximum scale sampling*/
        double renderer_radius_max_;
        /** Renderer parameter: the step scale sampling*/
        double renderer_radius_step_;
        /** Renderer parameter: image width */
        int renderer_width_;
        /** Renderer parameter: image height */
        int renderer_height_;
        /** Renderer parameter: near distance */
        double renderer_near_;
        /** Renderer parameter: far distance */
        double renderer_far_;
        /** Renderer parameter: focal length x */
        double renderer_focal_length_x_;
        /** Renderer parameter: focal length y */
        double renderer_focal_length_y_;

        tf::TransformListener listener;
        tf::TransformListener *g_listener;

        ros::NodeHandle nh_;
        ros::Subscriber decision_making_sub_;

        tf::StampedTransform base_to_mapping_camera_frame_transform_;
        tf::StampedTransform base_to_object_pose_hint3_transform_;
    };

}
// namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Detector, "Detector", "Use LINE-MOD for object detection.")
