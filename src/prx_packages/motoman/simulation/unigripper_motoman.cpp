/**
 * @file motoman.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */


#include "simulation/unigripper_motoman.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include "prx/utilities/definitions/string_manip.hpp"

#include <pluginlib/class_list_macros.h>
#include <urdf/model.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/param.h>
#include <boost/range/adaptor/map.hpp> //adaptors
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

PLUGINLIB_EXPORT_CLASS(prx::packages::motoman::unigripper_motoman_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace RigidBodyDynamics::Addons;
    using namespace RigidBodyDynamics::Math;
    using namespace RigidBodyDynamics;
    using namespace sim;
    using namespace util;
    namespace packages
    {
        namespace motoman
        {

            void uni_convert_matrix_to_quaternion(Matrix3d& mat, double& qx,double& qy,double& qz,double& qw)
            {
                double m00 = mat(0,0);
                double m01 = mat(0,1);
                double m02 = mat(0,2);
                double m10 = mat(1,0);
                double m11 = mat(1,1);
                double m12 = mat(1,2);
                double m20 = mat(2,0);
                double m21 = mat(2,1);
                double m22 = mat(2,2);
                double S=0;
                if (m00 + m11 + m22 > 2.9999) 
                { // check for identity matrix
                    qw = 1;
                    qx = 0;
                    qy = 0; 
                    qz = 0; 
                } 
                else if( (m00 + m11 + m22 +1)> 0.0001 ) 
                {
                    S = sqrt(m00 + m11 + m22+1)*2;
                    qw = 0.25* S;
                    qx = ( m21 - m12) /S;
                    qy = ( m02 - m20 ) /S;
                    qz = ( m10 - m01 ) /S;
                } 
                else if ((m00 > m11)&(m00 > m22)) 
                { 
                    if (( 1.0 + m00 - m11 - m22 ) <= 0) 
                    {
                        qw = 1;
                        qx = 0;
                        qy = 0; 
                        qz = 0; 
                        return;
                    }
                    S = sqrt( 1.0 + m00 - m11 - m22 ) * 2; // S=4*qx 
                    qw = (m21 - m12) / S;
                    qx = 0.25 * S;
                    qy = (m01 + m10) / S; 
                    qz = (m02 + m20) / S; 
                } 
                else if (m11 > m22) 
                { 
                    if (( 1.0 + m11 - m00 - m22 ) <= 0) 
                    {
                        qw = 1;
                        qx = 0;
                        qy = 0; 
                        qz = 0; 
                        return;
                    }
                    S = sqrt( 1.0 + m11 - m00 - m22 ) * 2; // S=4*qy
                    qw = (m02 - m20) / S;
                    qx = (m01 + m10) / S; 
                    qy = 0.25 * S;
                    qz = (m12 + m21) / S; 
                } 
                else 
                { 
                    if (( 1.0 + m22 - m00 - m11 ) <= 0) 
                    {
                        qw = 1;
                        qx = 0;
                        qy = 0; 
                        qz = 0; 
                        return;
                    }
                    S = sqrt( 1.0 + m22 - m00 - m11 ) * 2; // S=4*qz
                    qw = (m10 - m01) / S;
                    qx = (m02 + m20) / S; 
                    qy = (m12 + m21) / S; 
                    qz = 0.25 * S;
                }
            }

            unigripper_motoman_plant_t::unigripper_motoman_plant_t() : derivative_function(boost::bind(&unigripper_motoman_plant_t::update_derivative, this, _1))
            {
                integrator = NULL;
            }
            void unigripper_motoman_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                // std::string filename = parameters::get_attribute_as<std::string>("urdf_file",reader,template_reader);
                root_offset = parameters::get_attribute_as<std::vector<double> >("root_offset",reader,template_reader);
                MAX_IK_STEP = parameters::get_attribute_as<double>("max_ik_step", reader, template_reader, 0.01);
                hand = parameters::get_attribute("hand", reader, template_reader, "left");
                bool robotiq_hand = parameters::get_attribute_as<bool>("robotiq", reader, template_reader, false);


                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                if(!robotiq_hand)
                    filename += ("/prx_planning/templates/motoman/sda10f_unigripper.urdf");
                else
                    filename += ("/prx_planning/templates/motoman/motoman_robotiq.urdf");
                my_tree = new KDL::Tree();
                URDFReadFromFile(filename.c_str(), &model, false);
                if (!kdl_parser::treeFromFile(filename.c_str(), *my_tree)){
                  PRX_ERROR_S("Failed to construct kdl tree");
                }
                model.gravity = Vector3d (0., 0., 0. );
                Q = VectorNd::Zero (model.dof_count);
                QDot = VectorNd::Zero (model.dof_count);
                Q[0] = root_offset[0];
                Q[1] = root_offset[1];
                Q[2] = root_offset[2];

                for(unsigned i=6;i<14; i++)
                {
                    state_memory.push_back(&Q[i]);
                }
                for(unsigned i=15;i<22; i++)
                {
                    state_memory.push_back(&Q[i]);
                }
                //extra degree of freedom
                state_memory.push_back(&Q[14]);
                for(unsigned i=6;i<14; i++)
                {
                    control_memory.push_back(&Q[i]);
                }
                for(unsigned i=15;i<22; i++)
                {
                    control_memory.push_back(&Q[i]);
                }
                //extra degree of freedom
                control_memory.push_back(&Q[14]);

                state_memory.push_back(&_grasping);
                control_memory.push_back(&_grasping_control);

                state_space = new space_t("Motoman_unigripper", state_memory);
                input_control_space = new space_t("Motoman_unigripper", control_memory);
                prev_st = state_space->alloc_point();
                inter_st = state_space->alloc_point();

                if( template_reader != NULL )
                {
                    PRX_DEBUG_COLOR("Template integrator", PRX_TEXT_CYAN);
                    integrator = template_reader->initialize_from_loader< integrator_t > ("integrator", "prx_simulation");
                }
                else
                {
                    PRX_DEBUG_COLOR("Normal integrator", PRX_TEXT_CYAN);
                    integrator = reader->initialize_from_loader< integrator_t > ("integrator", "prx_simulation");
                }
                integrator->init_space(state_space);

                //    PRX_WARN_S("Init for rigid body ");
                manipulator_plant_t::init(reader, template_reader);
                foreach(std::string name, config_names)
                {
                    simple_rigid_body_names.push_back(reverse_split_path(name).second);
                }

                UpdateKinematicsCustom(model, &Q, NULL, NULL);
                
                for(unsigned i=0;i<simple_rigid_body_names.size();i++)
                {
                    unsigned int id = model.GetBodyId(simple_rigid_body_names[i].c_str());
                    body_ids.push_back(id);
                    // PRX_INFO_S(simple_rigid_body_names[i]<<" "<<id);
                }

                effector_name = pathname + "/vacuum_volume";

            }

            void unigripper_motoman_plant_t::propagate(const double simulation_step)
            {
                integrator->integrate(derivative_function, simulation_step);
                UpdateKinematicsCustom(model, &Q, NULL, NULL);
            }

            void unigripper_motoman_plant_t::steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan)
            {
                //each joint is independent
                double max_time = 0;
                for(unsigned i=0;i<16; i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    double vel = input_control_space->get_bounds()[i]->get_upper_bound();
                    double test_time = fabs(dist)/vel;
                    if(vel <= 0.000000001)
                        test_time = 0;
                    if(test_time>max_time)
                    {
                        max_time = test_time;
                    }
                }
                double steps = std::ceil(max_time / simulation::simulation_step);
                max_time = steps * simulation::simulation_step;
                result_plan.append_onto_back(max_time);
                std::vector<double> new_control;
                for(unsigned i=0;i<16; i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    new_control.push_back(dist/max_time);
                }
                new_control.push_back(goal->memory[16]);
                input_control_space->set_from_vector(new_control, result_plan.back().control);
            }

            void unigripper_motoman_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                bool update = true;
                VectorNd tmpVec = VectorNd::Zero(3);
                VectorNd pos;
                Matrix3d orient;
                Quaternion quat;
                double qw,qx,qy,qz;
                for(unsigned i=0;i<simple_rigid_body_names.size();i++)
                {
                    unsigned int id = body_ids[i];//model.GetBodyId(simple_rigid_body_names[i].c_str());
                    pos = CalcBodyToBaseCoordinates(model,Q,id,tmpVec,update);
                    update = false;
                    orient = CalcBodyWorldOrientation(model,Q,id,update);
                    quat = Quaternion::fromMatrix(orient);
                    augment_config_list(configs,index);
                    configs[index].first = config_names[i];

                    configs[index].second.set_position(pos[0],pos[1],pos[2]);
                    configs[index].second.set_orientation(quat.x(),quat.y(),quat.z(),quat.w());
                    if(std::isnan(quat.w()) || std::isnan(quat.x()) || std::isnan(quat.y()) || std::isnan(quat.z()) )
                    {
                        uni_convert_matrix_to_quaternion(orient,qx,qy,qz,qw);
                        configs[index].second.set_orientation(qx,qy,qz,qw);
                    }

                    index++;
                }
            }

            void unigripper_motoman_plant_t::update_derivative(state_t * const result)
            {

                // for(unsigned i=6;i<14; i++)
                // {
                //     state_memory.push_back(&Q[i]);
                // }
                // for(unsigned i=15;i<22; i++)
                // {
                //     state_memory.push_back(&Q[i]);
                // }
                // //extra degree of freedom
                // state_memory.push_back(&Q[14]);

                //update state 
                int index =0;
                for(unsigned i=6;i<14; i++)
                {
                    result->memory[index] = QDot[i];
                    index++;
                }
                for(unsigned i=15;i<22; i++)
                {
                    result->memory[index] = QDot[i];
                    index++;
                }
                result->memory[15] = QDot[14];
                result->memory[16] = _grasping_control/simulation::simulation_step;
            }


            void unigripper_motoman_plant_t::get_end_effector_configuration(config_t& effector_config)
            {
                double qw,qx,qy,qz;
                bool update = true;
                VectorNd tmpVec = VectorNd::Zero(3);
                VectorNd pos;
                Matrix3d orient;
                Quaternion quat;
                std::string ee = "vacuum_volume";
                unsigned int id = model.GetBodyId(ee.c_str());
                pos = CalcBodyToBaseCoordinates(model,Q,id,tmpVec,update); 
                update = false;
                orient = CalcBodyWorldOrientation(model,Q,id,update);
                quat = Quaternion::fromMatrix(orient);
                effector_config.set_position(pos[0], pos[1], pos[2]);
                effector_config.set_orientation(quat[0],quat[1],quat[2],quat[3]);
                if(std::isnan(quat.w()) || std::isnan(quat.x()) || std::isnan(quat.y()) || std::isnan(quat.z()) )
                {
                    uni_convert_matrix_to_quaternion(orient,qx,qy,qz,qw);
                    effector_config.set_orientation(qx,qy,qz,qw);
                }
            }

            bool unigripper_motoman_plant_t::IK_solver(const config_t& effector_config, space_point_t* computed_state, bool set_grasping, const space_point_t* seed_state, bool do_min)
            {
                double qx,qy,qz,qw;
                double x,y,z;
                effector_config.get_orientation().get(qx,qy,qz,qw);
                effector_config.get_position(x,y,z);
                KDL::Chain chain1;
                my_tree->getChain("base_link","vacuum_volume",chain1);
                KDL::JntArray q(chain1.getNrOfJoints());
                KDL::JntArray q_out(chain1.getNrOfJoints());
                KDL::JntArray q_min(chain1.getNrOfJoints());
                KDL::JntArray q_max(chain1.getNrOfJoints());


                std::vector< double > state_var;
                if( seed_state != NULL )
                    state_space->copy_point_to_vector( seed_state, state_var );
                else
                    state_space->copy_to_vector( state_var );
                q(0) = state_var[0];
                q(1) = state_var[1];
                q(2) = state_var[2];
                q(3) = state_var[3];
                q(4) = state_var[4];
                q(5) = state_var[5];
                q(6) = state_var[6];
                q(7) = state_var[7];
                q(8) = state_var[15];

                q_min(0) = state_space->get_bounds()[0]->get_lower_bound();
                q_min(1) = state_space->get_bounds()[1]->get_lower_bound();
                q_min(2) = state_space->get_bounds()[2]->get_lower_bound();
                q_min(3) = state_space->get_bounds()[3]->get_lower_bound();
                q_min(4) = state_space->get_bounds()[4]->get_lower_bound();
                q_min(5) = state_space->get_bounds()[5]->get_lower_bound();
                q_min(6) = state_space->get_bounds()[6]->get_lower_bound();
                q_min(7) = state_space->get_bounds()[7]->get_lower_bound();
                q_min(8) = state_space->get_bounds()[15]->get_lower_bound();

                q_max(0) = state_space->get_bounds()[0]->get_upper_bound();
                q_max(1) = state_space->get_bounds()[1]->get_upper_bound();
                q_max(2) = state_space->get_bounds()[2]->get_upper_bound();
                q_max(3) = state_space->get_bounds()[3]->get_upper_bound();
                q_max(4) = state_space->get_bounds()[4]->get_upper_bound();
                q_max(5) = state_space->get_bounds()[5]->get_upper_bound();
                q_max(6) = state_space->get_bounds()[6]->get_upper_bound();
                q_max(7) = state_space->get_bounds()[7]->get_upper_bound();
                q_max(8) = state_space->get_bounds()[15]->get_upper_bound();

                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                KDL::ChainIkSolverVel_pinv ik_solver_vel(chain1);
                KDL::ChainIkSolverPos_NR_JL ik_solver(chain1,q_min,q_max,fk_solver,ik_solver_vel,1000,1e-6);
                KDL::Frame F(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(x,y,z));
                bool ik_res = (ik_solver.CartToJnt(q,F,q_out)>=0);

                if(ik_res)
                {
                    std::vector<double> state_vec;
                    state_vec = state_var;
                    state_vec[0] = q_out(0);
                    state_vec[1] = q_out(1);
                    state_vec[2] = q_out(2);
                    state_vec[3] = q_out(3);
                    state_vec[4] = q_out(4);
                    state_vec[5] = q_out(5);
                    state_vec[6] = q_out(6);
                    state_vec[7] = q_out(7);
                    state_vec[15] = q_out(8);
                    state_vec[16] = set_grasping ? (double)GRIPPER_CLOSED : GRIPPER_OPEN;
                    state_space->copy_vector_to_point( state_vec, computed_state );
                }

                return ik_res;
            }

            bool unigripper_motoman_plant_t::is_grasping() const
            {
                return (_grasping == 1);
            }

            bool unigripper_motoman_plant_t::IK_steering( const config_t& start_config, const config_t& goal_config, sim::plan_t& result_plan, bool set_grasping )
            {
                //Begin by determining a distance between the start and goal configurations
                double distance = start_config.get_position().distance( goal_config.get_position() );
                //Set the number of interpolation steps
                double steps = std::ceil(distance / MAX_IK_STEP);
                //Clear out the plan we were given, we will be filling it in ourselves
                result_plan.clear();
                //Keep around an intermediate plan for systematically building up the solution
                plan_t intermediate_plan = result_plan;

                //Store the arm's current state as the previous state
                state_space->copy_to_point(prev_st);
                //So first, let's do the IK for the start configuration
                if( !IK_solver( start_config, prev_st, set_grasping, prev_st, true ) )
                {
                    PRX_DEBUG_COLOR("IK failed in Steering for the start config.", PRX_TEXT_RED);
                    return false;
                }

                //Storage for the interpolated configuration
                config_t interpolated_config;

                //Then, for each step
                for( double i=1; i<=steps; ++i )
                {
                    //Compute the next interpolated configuration
                    double t = PRX_MINIMUM( (i/steps), 1.0 );
                    interpolated_config.interpolate( start_config, goal_config, t );

                    //Compute Minimum IK for interpolated configuration (Pose, seed state, soln)
                    if( IK_solver( interpolated_config, inter_st, set_grasping, prev_st, true ) )
                    {
                        //Create a plan by steering between states and add it to the resulting plan
                        intermediate_plan.clear();
                        steering_function(prev_st, inter_st, intermediate_plan);
                        result_plan += intermediate_plan;
                        //And make sure we store this state as our prior state
                        state_space->copy_point(prev_st, inter_st);
                    }
                    //If the MinIK fails, then the steering is a failure, abort
                    else
                    {
                        //Clear out whatever plan we had started making
                        PRX_DEBUG_COLOR("IK failed in Steering", PRX_TEXT_BROWN);
                        result_plan.clear();
                        return false;
                    }
                }

                //Report our Success
                return true;
            }

        }
    }
}

