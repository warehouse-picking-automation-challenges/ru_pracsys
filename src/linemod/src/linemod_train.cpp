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

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <object_recognition_core/common/json.hpp>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/model_utils.h>

#include <object_recognition_renderer/renderer3d.h>
#include <object_recognition_renderer/utils.h>

#include <ros/ros.h>
#include <ros/package.h>

#define LINEMOD_VIZ_IMG 1
#if LINEMOD_VIZ_IMG
  #include <opencv2/highgui/highgui.hpp>
#endif

using ecto::tendrils;
using ecto::spore;


void
drawResponse(const std::vector<cv::linemod::Template>& templates, int num_modalities,
		cv::Mat& dst, cv::Point offset, int T);

namespace ecto_linemod
{
    struct Trainer
    {
        static void
        declare_params(tendrils& params)
        {
            params.declare(&Trainer::param_n_points_, "renderer_n_points", "Renderer parameter: the number of points on the sphere.", 150);
            params.declare(&Trainer::param_angle_step_, "renderer_angle_step", "Renderer parameter: the angle step sampling in degrees.", 10);
            params.declare(&Trainer::param_radius_min_, "renderer_radius_min", "Renderer parameter: the minimum scale sampling.", 0.6);
            params.declare(&Trainer::param_radius_max_, "renderer_radius_max", "Renderer parameter: the maximum scale sampling.", 1.1);
            params.declare(&Trainer::param_radius_step_, "renderer_radius_step", "Renderer parameter: the step scale sampling.", 0.4);
            params.declare(&Trainer::param_width_, "renderer_width", "Renderer parameter: the image width.", 640);
            params.declare(&Trainer::param_height_, "renderer_height", "Renderer parameter: the image height.", 480);
            params.declare(&Trainer::param_focal_length_x_, "renderer_focal_length_x", "Renderer parameter: the focal length x.", 528.0);
            params.declare(&Trainer::param_focal_length_y_, "renderer_focal_length_y", "Renderer parameter: the focal length y.", 528.0);
            params.declare(&Trainer::param_near_, "renderer_near", "Renderer parameter: near distance.", 0.1);
            params.declare(&Trainer::param_far_, "renderer_far", "Renderer parameter: far distance.", 1000.0);
            params.declare(&Trainer::visualize_, "visualize", "If True, visualize the output.", false);
        }


        static void
        declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
        {
            inputs.declare(&Trainer::json_db_, "json_db", "The DB parameters", "{}").required(true);
            inputs.declare(&Trainer::object_id_, "object_id", "The object id, to associate this model with.").required(true);

            outputs.declare(&Trainer::detector_, "detector", "The LINE-MOD detector");
            outputs.declare(&Trainer::Rs_, "Rs", "The matching rotations of the templates");
            outputs.declare(&Trainer::Ts_, "Ts", "The matching translations of the templates.");
            outputs.declare(&Trainer::distances_, "distances", "The matching depth of the templates.");
            outputs.declare(&Trainer::Ks_, "Ks", "The matching calibration matrices of the templates.");
            outputs.declare(&Trainer::renderer_n_points_, "renderer_n_points", "Renderer parameter: the number of points on the sphere.");
            outputs.declare(&Trainer::renderer_angle_step_, "renderer_angle_step", "Renderer parameter: the angle step sampling in degrees.");
            outputs.declare(&Trainer::renderer_radius_min_, "renderer_radius_min", "Renderer parameter: the minimum scale sampling.");
            outputs.declare(&Trainer::renderer_radius_max_, "renderer_radius_max", "Renderer parameter: the maximum scale sampling.");
            outputs.declare(&Trainer::renderer_radius_step_, "renderer_radius_step", "Renderer parameter: the step scale sampling.");
            outputs.declare(&Trainer::renderer_width_, "renderer_width", "Renderer parameter: the image width.");
            outputs.declare(&Trainer::renderer_height_, "renderer_height", "Renderer parameter: the image height.");
            outputs.declare(&Trainer::renderer_focal_length_x_, "renderer_focal_length_x", "Renderer parameter: the focal length x.");
            outputs.declare(&Trainer::renderer_focal_length_y_, "renderer_focal_length_y", "Renderer parameter: the focal length y.");
            outputs.declare(&Trainer::renderer_near_, "renderer_near", "Renderer parameter: near distance.");
            outputs.declare(&Trainer::renderer_far_, "renderer_far", "Renderer parameter: far distance.");
        }

        void
        configure(const tendrils& params, const tendrils& inputs, const tendrils& outputs)
        {
            //or_json::mValue submethod = object_recognition_core::to_json(*json_submethod_);
        }

        void
        visualization_for_test(cv::Mat img, cv::Mat dpt, int template_id)
        {
			std::cout << "\nThis is the easiest way to debug.\n" << std::endl;

			// For testing with the templates used for training
//			cv::Mat image = cv::Mat::zeros(cv::Size(640/2, 480/2), img.type());
//			cv::Mat depth = cv::Mat::zeros(cv::Size(640/2, 480/2), dpt.type());
////			img.copyTo(image(cv::Rect(0, 0, img.cols, img.rows)));
////			dpt.copyTo(depth(cv::Rect(0, 0, dpt.cols, dpt.rows)));
//			img.copyTo(image(cv::Rect(640/4, 480/4, img.cols, img.rows)));
//			dpt.copyTo(depth(cv::Rect(640/4, 480/4, dpt.cols, dpt.rows)));

			// For testing with a real pair image/depth used for training
			cv::Mat image_orig = cv::imread("/home/alberto/RUTGERS/apc_hg/object_models/im.png", CV_LOAD_IMAGE_COLOR);
			cv::Mat depth_orig = cv::imread("/home/alberto/RUTGERS/apc_hg/object_models/dp.png", CV_16UC1);
			cv::Mat image, depth;
			cv::Size size(640, 480);
			//cv::GaussianBlur(image_orig, image, cv::Size(21, 21), 0, 0);
			cv::resize(image_orig, image, size);
			cv::resize(depth_orig, depth, size);

			std::vector<cv::Mat> sources;
            sources.push_back(image);
            sources.push_back(depth);
            cv::Mat display;
            image.copyTo(display);

            const std::vector<cv::linemod::Template>& templates = detector_->getTemplates("object1", template_id);
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

            detector__->addSyntheticTemplate(templates, "object1");

            float threshold_ = 50.0f;
            std::vector<cv::linemod::Match> matches;

            detector__->match(sources, threshold_, matches);

            cv::FileStorage current_config("test2.yml", cv::FileStorage::WRITE);
            detector__->write(current_config);
            current_config.release();

            int num_modalities = detector__->getModalities().size();
            BOOST_FOREACH(const cv::linemod::Match &match, matches)
            {
				const std::vector<cv::linemod::Template>& templates =
						detector__->getTemplates(match.class_id, match.template_id);
				drawResponse(templates, num_modalities, display, cv::Point(match.x, match.y), detector__->getT(0));
				std::cout << "match.x = " << match.x << "  match.y = " << match.y << "  match.similarity = " << match.similarity << std::endl;
            }

            std::cout << "  Template: " << template_id << std::endl;
            std::cout << "Dimensions: [" << templates[0].width << "," << templates[0].height << "]" << std::endl;
            std::cout << "# Templates: " << templates.size() << std::endl;
            std::cout << "# Modalities: " << num_modalities << std::endl;

            // Adding SIFT feature descriptors only to training image output
//                cv::SIFT sift(75,3,0.04,3);
//                std::vector<cv::KeyPoint> keypoints;
//                cv::Mat output, descriptors, nullmask;
//                sift(image,nullmask,keypoints,descriptors);
//
//                cv::drawKeypoints(image, keypoints, output);
//
//                cv::resize(output,output,cv::Size(),2,2);

            cv::namedWindow("Template", cv::WINDOW_NORMAL);
            cv::imshow("Template", display);
            cv::waitKey(1);
            // End COLIN CHECK
        }


        int
        process(const tendrils& inputs, const tendrils& outputs)
        {
            // Get the document for the object_id_ from the DB
            object_recognition_core::db::ObjectDbPtr db = object_recognition_core::db::ObjectDbParameters(*json_db_).generateDb();
            object_recognition_core::db::Documents documents = object_recognition_core::db::ModelDocuments(db,
                    std::vector<object_recognition_core::db::ObjectId>(1, *object_id_), "mesh");
            if (documents.empty())
            {
                std::cerr << "Skipping object id \"" << *object_id_ << "\" : no mesh in the DB" << std::endl;
                return ecto::OK;
            }

            // Get the list of _attachments and figure out the original one
            object_recognition_core::db::Document document = documents[0];
            std::vector<std::string> attachments_names = document.attachment_names();
            std::string mesh_path;
            BOOST_FOREACH(const std::string& attachment_name, attachments_names)
            {
                if (attachment_name.find("original") != 0)
                    continue;

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
            }

            cv::Ptr<cv::linemod::Detector> detector_ptr = cv::linemod::getDefaultLINEMOD();
            *detector_ = *detector_ptr;

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
            std::stringstream file_name2;
            file_name2 << path << "/conf/test.yml";
            cv::FileStorage current_config(file_name2.str().c_str(), cv::FileStorage::WRITE);

            cv::FileNode fn;
            fn = desired_config["weak_threshold"];
            detector_->read(fn);
            fn = desired_config["strong_threshold"];
            detector_->read(fn);
            fn = desired_config["distance_threshold"];
            detector_->read(fn);
            fn = desired_config["difference_threshold"];
            detector_->read(fn);
            fn = desired_config["extract_threshold"];
            detector_->read(fn);

            detector_->write(current_config);
            current_config.release();
            desired_config.release();

            // Define the display
            //assign the parameters of the renderer
            *renderer_n_points_ = *param_n_points_;
            *renderer_angle_step_ = *param_angle_step_;
            *renderer_radius_min_ = *param_radius_min_;
            *renderer_radius_max_ = *param_radius_max_;
            *renderer_radius_step_ = *param_radius_step_;
            *renderer_width_ = *param_width_;
            *renderer_height_ = *param_height_;
            *renderer_near_ = *param_near_;
            *renderer_far_ = *param_far_;
            *renderer_focal_length_x_ = *param_focal_length_x_;
            *renderer_focal_length_y_ = *param_focal_length_y_;

            std::cout << "renderer_focal_length_x_ = " << *renderer_focal_length_x_
            		<< "   renderer_focal_length_y_ = " << *renderer_focal_length_y_ << std::endl;

            // the model name can be specified on the command line.
            Renderer3d renderer = Renderer3d(mesh_path);
            renderer.set_parameters(*renderer_width_,
                    *renderer_height_,
                    *renderer_focal_length_x_,
                    *renderer_focal_length_y_,
                    *renderer_near_,
                    *renderer_far_);

            RendererIterator renderer_iterator = RendererIterator(&renderer, *renderer_n_points_);
            std::remove(mesh_path.c_str());
            //set the RendererIterator parameters
            renderer_iterator.angle_step_ = float(*renderer_angle_step_);
            renderer_iterator.radius_min_ = float(*renderer_radius_min_);
            renderer_iterator.radius_max_ = float(*renderer_radius_max_);
            renderer_iterator.radius_step_ = float(*renderer_radius_step_);

            cv::Mat image, depth, mask, converted;
            cv::Matx33d R;
            cv::Vec3d T;
            cv::Matx33f K;
            int rejections = 0;
            for (size_t i = 0; !renderer_iterator.isDone(); ++i, ++renderer_iterator)
            {
                std::stringstream status;
                status << "Loading images " << (i + 1) << "/" << renderer_iterator.n_templates();
//                status << "   renderer_iterator.angle_ = " << renderer_iterator.angle_;
//                status << "   renderer_iterator.radius_ = " << renderer_iterator.radius_;
//                status << "   renderer_iterator.index_ = " << renderer_iterator.index_;
                std::cout << status.str();
                std::cout.flush();

                cv::Rect rect;
                renderer_iterator.render(image, depth, mask, rect);
            #if 1// LINEMOD_VIZ_IMG
                // Display the rendered image
                if (*visualize_)
                {
                  cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
                  //cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);
                  cv::namedWindow("Mask", cv::WINDOW_AUTOSIZE);
                    if (!image.empty())
                    {
//                        std::stringstream image_file, mask_file, depth_file;
//                        image_file << "/home/colinmatthew/test_images/" << i << ".jpg";

//                        mask_file << "mask_" << i << ".yml";
//                        depth_file << "depth_" << i << ".yml";
//                        cv::String image_f = image_file.str();
//                        cv::String mask_f = mask_file.str();
//                        cv::String depth_f = depth_file.str();
//                        cv::imwrite(image_f, image);

//                        cv::FileStorage image_file_out(image_f, cv::FileStorage::WRITE);
//                        cv::FileStorage mask_file_out(mask_f, cv::FileStorage::WRITE);
//                        cv::FileStorage depth_file_out(depth_f, cv::FileStorage::WRITE);
//                        image_file_out << "image" << image;
//                        mask_file_out << "mask" << mask;
//                        depth_file_out << "depth" << depth;


//                        cv::imwrite(image_file.str(), image);
//                        cv::imwrite(mask_file.str(), mask);
//                        cv::imwrite(depth_file.str(), depth);
                        //depth.convertTo(converted, CV_16UC1, 100.0);
                        //cv::imshow("Depth", converted);
                        cv::imshow("Image", image);
                        cv::imshow("Mask", mask);
                        cv::waitKey(10);
//                        std::stringstream image_name, image_number;
//                        image_number << std::setfill('0') << std::setw(5) << i;
//                        image_name << std::setw(0) << "/home/alberto/RUTGERS/apc_hg/images/image" << image_number.str() << ".jpg";
//                        imwrite(image_name.str(), image);
                    }
                }
            #endif


                R = renderer_iterator.R_obj();
                T = renderer_iterator.T();
                float distance = fabs(renderer_iterator.D_obj() -
                        float(depth.at<ushort>(depth.rows / 2.0f, depth.cols / 2.0f) / 1000.0f));
                K = cv::Matx33f(float(*renderer_focal_length_x_), 0.0f, float(rect.width) / 2.0f, 0.0f,
                        float(*renderer_focal_length_y_), float(rect.height) / 2.0f, 0.0f, 0.0f, 1.0f);
//                double m[3][3] = {{float(*renderer_focal_length_x_), 0.0f, float(rect.width) / 2.0f}, {0.0f,float(*renderer_focal_length_y_), float(rect.height) / 2.0f}, {0.0f, 0.0f, 1.0f}};
//                std::cout << "K: " << K << '\n';
//                std::cout << "R: " << R << '\n';
//                std::cout << "t: " << T << '\n';
//                cv::Mat X;
//                cv::hconcat(R,T,X);
//                std::cout << "X: " << X << '\n';
//                std::cout << "Xtype: " << X.type() << '\n';
//                cv::Mat K_temp = cv::Mat(3,3, CV_64F, m);
//                cv::Mat R_temp(renderer_iterator.R());
//                cv::Mat T_temp(T);
//                std::cout << "R: " << R << '\n';
//                std::cout << "R_temp: " << R_temp << '\n';
//                K_temp = (cv::Mat)K;
//                K_temp(0,0) = float(*renderer_focal_length_x_);
//                K_temp.at<float>(0,2) = float(rect.width) / 2.0f;
//                K_temp.at<float>(1,1) = float(*renderer_focal_length_y_);
//                K_temp.at<float>(1,2) = float(rect.height) / 2.0f;
//                K_temp.at<float>(2,2) = 1.0f;
//                std::cout << " K(0,0) " << K(0,0) << std::endl;
//                K_temp.at<float>(0,0) = K(0,0);
//                std::cout << " K_temp(0,0) " << K_temp.at<float>(0,0) << std::endl;
//                K_temp.at<float>(0,1) = K(0,1);
//                K_temp.at<float>(0,2) = K(0,2);
//                K_temp.at<float>(1,0) = K(1,0);
//                K_temp.at<float>(1,1) = K(1,1);
//                K_temp.at<float>(1,2) = K(1,2);
//                K_temp.at<float>(2,0) = K(2,0);
//                K_temp.at<float>(2,1) = K(2,1);
//                K_temp.at<float>(2,2) = K(2,2);
//                std::cout << "Ktemp: " << K_temp << '\n';

//                cv::Mat result = cv::Mat::zeros(3,4,CV_64F);
//                result = K_temp * X;
//                std::cout << "Proj Mat: " << result << '\n';

//                std::stringstream  image_yml;
//                image_yml << "/home/colinmatthew/test_images/" << i << ".yml";
//                cv::String image_f_yml = image_yml.str();
//                cv::FileStorage image_file_out(image_f_yml, cv::FileStorage::WRITE);
//                image_file_out << "Projection" << result;
//                image_file_out << "R" << R_temp;
//                image_file_out << "T" << T_temp;
//                image_file_out << "K" << K_temp;


                std::vector<cv::Mat> sources(2);
                sources[0] = image;
                sources[1] = depth;

                cv::Mat empty_mask;
                int template_id = detector_->addTemplate(sources, "object1", empty_mask);
//                int template_id = detector_->addTemplate(sources, "object1", mask);
                if (template_id == -1)
                {
                    rejections++;
                    // Delete the status
                    for (size_t j = 0; j < status.str().size(); ++j)
                        std::cout << '\b';
                    std::cout << "View rejected            " << std::endl;
                    continue;
                }

                // Also store the pose of each template
                Rs_->push_back(cv::Mat(R));
                Ts_->push_back(cv::Mat(T));
                distances_->push_back(distance);
                Ks_->push_back(cv::Mat(K));

                //visualization_for_test(image, depth, template_id);
                // std::cout << "\nR\n" << R << "\nT\n" << T << "\ndistance\n" << distance << "\nK\n" << K << "\n";

                // Delete the status
                for (size_t j = 0; j < status.str().size(); ++j)
                    std::cout << '\b';
            }
            std::cout << std::endl;

//            std::cout << "Template = np.array([";
//            for (int k=0; k < Rs_->size(); k++) {
//                std::cout << Ts_->at(k) << "," << std::endl;
//            }
//            std::cout << "], dtype='float64'" << std::endl;

            std::cout << "Total Rejections: " << rejections << std::endl;
            return ecto::OK;
        }

        /** True or False to output debug image */
        ecto::spore<bool> visualize_;
        /** The DB parameters as a JSON string */
        ecto::spore<std::string> json_db_;
        /** The id of the object to generate a trainer for */
        ecto::spore<std::string> object_id_;
        ecto::spore<cv::linemod::Detector> detector_;
        ecto::spore<std::vector<cv::Mat> > Rs_;
        ecto::spore<std::vector<cv::Mat> > Ts_;
        ecto::spore<std::vector<float> > distances_;
        ecto::spore<std::vector<cv::Mat> > Ks_;
        ecto::spore<int> param_n_points_;
        ecto::spore<int> param_angle_step_;
        ecto::spore<double> param_radius_min_;
        ecto::spore<double> param_radius_max_;
        ecto::spore<double> param_radius_step_;
        ecto::spore<int> param_width_;
        ecto::spore<int> param_height_;
        ecto::spore<double> param_near_;
        ecto::spore<double> param_far_;
        ecto::spore<double> param_focal_length_x_;
        ecto::spore<double> param_focal_length_y_;
        ecto::spore<int> renderer_n_points_;
        ecto::spore<int> renderer_angle_step_;
        ecto::spore<double> renderer_radius_min_;
        ecto::spore<double> renderer_radius_max_;
        ecto::spore<double> renderer_radius_step_;
        ecto::spore<int> renderer_width_;
        ecto::spore<int> renderer_height_;
        ecto::spore<double> renderer_near_;
        ecto::spore<double> renderer_far_;
        ecto::spore<double> renderer_focal_length_x_;
        ecto::spore<double> renderer_focal_length_y_;
    };
} // namespace ecto_linemod

ECTO_CELL(ecto_linemod, ecto_linemod::Trainer, "Trainer", "Train the LINE-MOD object detection algorithm.")
