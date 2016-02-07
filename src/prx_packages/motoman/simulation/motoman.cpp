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

#include "simulation/motoman.hpp"
#include "prx/simulation/simulators/simulator.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/assign/list_of.hpp>
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/definitions/random.hpp"

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

PLUGINLIB_EXPORT_CLASS(prx::packages::motoman::motoman_plant_t, prx::sim::system_t)

#define IK_ITERATIONS 70

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

            void convert_matrix_to_quaternion(Matrix3d& mat, double& qx,double& qy,double& qz,double& qw)
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

            motoman_plant_t::motoman_plant_t() : derivative_function(boost::bind(&motoman_plant_t::update_derivative, this, _1))
            {
                integrator = NULL;
            }

            void motoman_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                // std::string filename = parameters::get_attribute_as<std::string>("urdf_file",reader,template_reader);
                root_offset = parameters::get_attribute_as<std::vector<double> >("root_offset",reader,template_reader);
                MAX_IK_STEP = parameters::get_attribute_as<double>("max_ik_step", reader, template_reader, 0.01);
                hand = parameters::get_attribute("hand", reader, template_reader, "left");
                robotiq_hand = parameters::get_attribute_as<bool>("robotiq", reader, template_reader, false);
                unigripper = parameters::get_attribute_as<bool>("unigripper", reader, template_reader, true);

                std::string input_file_name = "sda10f_complete_kinect.urdf";
                if( parameters::has_attribute( "urdf_file", reader, template_reader ) )
                {
                	input_file_name = parameters::get_attribute_as< std::string >( "urdf_file", reader, template_reader );
                }

                char* w = std::getenv("PRACSYS_PATH");
                std::string filename(w);
                if(robotiq_hand && unigripper)
                {
                    filename += ("/prx_planning/templates/motoman/");
                    filename += input_file_name;
                }
                if(!robotiq_hand && unigripper)
                    filename += ("/prx_planning/templates/motoman/sda10f_complete_kinect_reflex.urdf");
                if(robotiq_hand && !unigripper)
                    filename += ("/prx_planning/templates/motoman/motoman_robotiq.urdf");
                if(!robotiq_hand && !unigripper)
                    filename += ("this is not a good string");

                PRX_PRINT("Going to load file: " << filename, PRX_TEXT_MAGENTA);

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

                if( robotiq_hand )
                {
                    Q[23] =Q[27] = Q[30]= .3;
                }
                else
                {
                    Q[23] = Q[26] = Q[28] = 1.05;
                    Q[24] = Q[27] = Q[29] = 0.7;
                }

                if(unigripper)
                {
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
                        control_memory.push_back(&QDot[i]);
                    }
                    for(unsigned i=15;i<22; i++)
                    {
                        control_memory.push_back(&QDot[i]);
                    }
                    //extra degree of freedom
                    control_memory.push_back(&QDot[14]);
                    state_memory.push_back(&_grasping);
                    control_memory.push_back(&_grasping_control);

                    state_space = new space_t("Motoman_unigripper", state_memory);
                    input_control_space = new space_t("Motoman_unigripper", control_memory);
                }
                else
                {
                    for(unsigned i=6;i<22; i++)
                    {
                        state_memory.push_back(&Q[i]);
                    }
                    for(unsigned i=6;i<22; i++)
                    {
                        control_memory.push_back(&QDot[i]);
                    }
                    state_memory.push_back(&_grasping);
                    control_memory.push_back(&_grasping_control);

                    state_space = new space_t("Motoman", state_memory);
                    input_control_space = new space_t("Motoman", control_memory);

                }

                prev_st = state_space->alloc_point();
                inter_st = state_space->alloc_point();

                if( template_reader != NULL )
                {
//                    PRX_DEBUG_COLOR("Template integrator", PRX_TEXT_CYAN);
                    integrator = template_reader->initialize_from_loader< integrator_t > ("integrator", "prx_simulation");
                }
                else
                {
//                    PRX_DEBUG_COLOR("Normal integrator", PRX_TEXT_CYAN);
                    integrator = reader->initialize_from_loader< integrator_t > ("integrator", "prx_simulation");
                }
                integrator->init_space(state_space);

                //    PRX_WARN_S("Init for rigid body ");
                manipulator_plant_t::init(reader, template_reader);
                foreach(std::string name, config_names)
                {
                    simple_rigid_body_names.push_back(reverse_split_path(name).second);
                }

                //Alright first let's get a zero vector for every body that has a name
                for(unsigned i=0;i<simple_rigid_body_names.size();i++)
                {
                    config_offsets.push_back( VectorNd::Zero(3) );
                }
                                
                   parameter_reader_t::reader_map_t subsystem_map = parameters::get_map("config_offsets", reader, template_reader);

                foreach(const parameter_reader_t::reader_map_t::value_type key_value, subsystem_map)
                {
                    std::vector<double> offset = key_value.second->get_attribute_as<std::vector<double> >("offset");
                    VectorNd tmpVec = VectorNd::Zero(3);
                    tmpVec[0] = offset[0];
                    tmpVec[1] = offset[1];
                    tmpVec[2] = offset[2];
                    //Kinda dumb, but let's just search the body
                    for( unsigned i=0; i<simple_rigid_body_names.size(); ++i )
                    {
                        if( key_value.first == simple_rigid_body_names[i] )
                        {
                            config_offsets[i] = tmpVec;
                        }
                    }
                }

                UpdateKinematicsCustom(model, &Q, NULL, NULL);
                // foreach(std::string name, model.mBodyNameMap | boost::adaptors::map_keys)
                // {
                //     PRX_INFO_S(name);
                // }
                // foreach(std::string name, simple_rigid_body_names)
                // {
                //     PRX_INFO_S(name);
                // }
                // ForwardDynamics(model, Q, QDot, Tau, QDDot);
                
                for(unsigned i=0;i<simple_rigid_body_names.size();i++)
                {
                    unsigned int id = model.GetBodyId(simple_rigid_body_names[i].c_str());
                    body_ids.push_back(id);
//                    PRX_INFO_S(simple_rigid_body_names[i]<<" "<<id);
                }
                // foreach(std::string name, model.mBodyNameMap | boost::adaptors::map_keys)
                // {
                //     PRX_WARN_S(name);
                // }
                if(hand=="left")
                {
                    if(unigripper)
                        simple_ee_name = "head_sponge";
                    else
                        simple_ee_name = "arm_left_link_tool0";

                    simple_camera_name = "left_RGB_camera";
                }
                else
                {
                    if(robotiq_hand)
                        simple_ee_name = "arm_right_robotiq_virtual";
                    if(!robotiq_hand)
                        simple_ee_name = "arm_right_reflex_virtual";

                    simple_camera_name = "right_RGB_camera";
                }
                effector_name = pathname+"/"+simple_ee_name;
                
            }

            void motoman_plant_t::propagate(const double simulation_step)
            {
                integrator->integrate(derivative_function, simulation_step);
                UpdateKinematicsCustom(model, &Q, NULL, NULL);
            }

            void motoman_plant_t::steering_function(const sim::state_t* start, const sim::state_t* goal, sim::plan_t& result_plan)
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
                        max_time = test_time;
                }
                double steps = std::ceil(max_time / simulation::simulation_step);
                max_time = steps * simulation::simulation_step;
                result_plan.append_onto_back(max_time);
                std::vector<double> new_control;
                for(unsigned i=0;i<(unigripper?16:15); i++)
                {
                    double dist = goal->memory[i] - start->memory[i];
                    new_control.push_back(dist/max_time);
                }
                if( start->memory[(unigripper?16:15)] != goal->memory[(unigripper?16:15)] )
                {
                    PRX_PRINT( " start state :: " << state_space->print_point( start, 3 ), PRX_TEXT_BROWN );
                    PRX_PRINT( " start state :: " << state_space->print_point( goal, 3 ), PRX_TEXT_BROWN );
                    PRX_FATAL_S( " Steering error: start grasping state different than the goal one" );
                }
                new_control.push_back(goal->memory[(unigripper?16:15)]);
                input_control_space->set_from_vector(new_control, result_plan.back().control);
            }

            void motoman_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
            {
                bool update = true;
                VectorNd pos;
                Matrix3d orient;
                Quaternion quat;
                double qw,qx,qy,qz;
                for(unsigned i=0;i<simple_rigid_body_names.size();i++)
                {
                    unsigned int id = body_ids[i];//model.GetBodyId(simple_rigid_body_names[i].c_str());
                    pos = CalcBodyToBaseCoordinates(model,Q,id,config_offsets[i],update);
                    update = false;
                    orient = CalcBodyWorldOrientation(model,Q,id,update);
                    quat = Quaternion::fromMatrix(orient);
                    augment_config_list(configs,index);
                    configs[index].first = config_names[i];

                    configs[index].second.set_position(pos[0],pos[1],pos[2]);
                    configs[index].second.set_orientation(quat.x(),quat.y(),quat.z(),quat.w());
                    if(std::isnan(quat.w()) || std::isnan(quat.x()) || std::isnan(quat.y()) || std::isnan(quat.z()) )
                    {
                        convert_matrix_to_quaternion(orient,qx,qy,qz,qw);
                        configs[index].second.set_orientation(qx,qy,qz,qw);
                    }

                    index++;
                }
            }

            void motoman_plant_t::update_derivative(state_t * const result)
            {
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


            void motoman_plant_t::get_end_effector_configuration( config_t& effector_config )
            {
                double qw,qx,qy,qz;
                bool update = true;
                VectorNd tmpVec = VectorNd::Zero(3);
                VectorNd pos;
                Matrix3d orient;
                Quaternion quat;
                std::string ee = simple_ee_name;

                unsigned int id = model.GetBodyId(ee.c_str());
                pos = CalcBodyToBaseCoordinates(model,Q,id,tmpVec,update); 
                update = false;
                orient = CalcBodyWorldOrientation(model,Q,id,update);
                quat = Quaternion::fromMatrix(orient);
                effector_config.set_position(pos[0], pos[1], pos[2]);
                effector_config.set_orientation(quat[0],quat[1],quat[2],quat[3]);
                if(std::isnan(quat.w()) || std::isnan(quat.x()) || std::isnan(quat.y()) || std::isnan(quat.z()) )
                {
                    convert_matrix_to_quaternion(orient,qx,qy,qz,qw);
                    effector_config.set_orientation(qx,qy,qz,qw);
                }
                effector_config.normalize_orientation();
            }

            void motoman_plant_t::get_camera_configuration( config_t& effector_config )
            {
                double qw,qx,qy,qz;
                bool update = true;
                VectorNd tmpVec = VectorNd::Zero(3);
                VectorNd pos;
                Matrix3d orient;
                Quaternion quat;
                std::string ee = simple_camera_name;

                // PRX_PRINT("Simple camera name is: " << ee, PRX_TEXT_LIGHTGRAY);

                unsigned int id = model.GetBodyId(ee.c_str());
                pos = CalcBodyToBaseCoordinates(model,Q,id,tmpVec,update); 
                update = false;
                orient = CalcBodyWorldOrientation(model,Q,id,update);
                quat = Quaternion::fromMatrix(orient);
                effector_config.set_position(pos[0], pos[1], pos[2]);
                effector_config.set_orientation(quat[0],quat[1],quat[2],quat[3]);
                if( std::isnan(quat.w()) || std::isnan(quat.x()) || std::isnan(quat.y()) || std::isnan(quat.z()) )
                {
                    convert_matrix_to_quaternion(orient,qx,qy,qz,qw);
                    effector_config.set_orientation(qx,qy,qz,qw);
                }
            }

            bool motoman_plant_t::get_utility_IK(const config_t& effector_config, space_point_t* computed_state, bool set_grasping, const space_point_t* seed_state, bool do_min)
            {
//                PRX_PRINT( "Running IK for: " << hand+"_IR_camera and configuration " << effector_config, PRX_TEXT_BLUE );

                double qx,qy,qz,qw;
                double x,y,z;
                effector_config.get_orientation().get(qx,qy,qz,qw);
                effector_config.get_position(x,y,z);
                KDL::Chain chain1;
                my_tree->getChain("base_link",hand+"_RGB_camera",chain1);                
                KDL::JntArray q(chain1.getNrOfJoints());
                KDL::JntArray q_out(chain1.getNrOfJoints());
                KDL::JntArray q_min(chain1.getNrOfJoints());
                KDL::JntArray q_max(chain1.getNrOfJoints());                

                std::vector< double > state_var;
                if( seed_state != NULL )
                {
//                    PRX_PRINT( "I do have a seed: " << state_space->print_point( seed_state, 4 ) , PRX_TEXT_LIGHTGRAY );
                    state_space->copy_point_to_vector( seed_state, state_var );
                }
                else
                {
//                    PRX_PRINT( "I do not have a seed: ", PRX_TEXT_LIGHTGRAY );
                    state_space->copy_to_vector( state_var );
                }

                if(hand=="left")
                {
                    q(0) = state_var[0];
                    q(1) = state_var[1];
                    q(2) = state_var[2];
                    q(3) = state_var[3];
                    q(4) = state_var[4];
                    q(5) = state_var[5];
                    q(6) = state_var[6];
                    q_min(0) = -2;//state_space->get_bounds()[0]->get_lower_bound();
                    q_min(1) = state_space->get_bounds()[1]->get_lower_bound();
                    q_min(2) = state_space->get_bounds()[2]->get_lower_bound();
                    q_min(3) = state_space->get_bounds()[3]->get_lower_bound();
                    q_min(4) = state_space->get_bounds()[4]->get_lower_bound();
                    q_min(5) = state_space->get_bounds()[5]->get_lower_bound();
                    q_min(6) = state_space->get_bounds()[6]->get_lower_bound();
                    q_max(0) = .86;//state_space->get_bounds()[0]->get_upper_bound();
                    q_max(1) = state_space->get_bounds()[1]->get_upper_bound();
                    q_max(2) = state_space->get_bounds()[2]->get_upper_bound();
                    q_max(3) = state_space->get_bounds()[3]->get_upper_bound();
                    q_max(4) = state_space->get_bounds()[4]->get_upper_bound();
                    q_max(5) = state_space->get_bounds()[5]->get_upper_bound();
                    q_max(6) = state_space->get_bounds()[6]->get_upper_bound();
                }
                else
                {
//                    PRX_PRINT( "Right hand", PRX_TEXT_LIGHTGRAY );
                    q(0) = state_var[0];
                    q(1) = state_var[8];
                    q(2) = state_var[9];
                    q(3) = state_var[10];
                    q(4) = state_var[11];
                    q(5) = state_var[12];
                    q(6) = state_var[13];
                    q_min(0) = -.86;//state_space->get_bounds()[0]->get_lower_bound();
                    q_min(1) = state_space->get_bounds()[8]->get_lower_bound();
                    q_min(2) = state_space->get_bounds()[9]->get_lower_bound();
                    q_min(3) = state_space->get_bounds()[10]->get_lower_bound();
                    q_min(4) = state_space->get_bounds()[11]->get_lower_bound();
                    q_min(5) = state_space->get_bounds()[12]->get_lower_bound();
                    q_min(6) = state_space->get_bounds()[13]->get_lower_bound();
                    q_max(0) = 2;//state_space->get_bounds()[0]->get_upper_bound();
                    q_max(1) = state_space->get_bounds()[8]->get_upper_bound();
                    q_max(2) = state_space->get_bounds()[9]->get_upper_bound();
                    q_max(3) = state_space->get_bounds()[10]->get_upper_bound();
                    q_max(4) = state_space->get_bounds()[11]->get_upper_bound();
                    q_max(5) = state_space->get_bounds()[12]->get_upper_bound();
                    q_max(6) = state_space->get_bounds()[13]->get_upper_bound();
                }
                
                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                KDL::ChainIkSolverVel_pinv ik_solver_vel(chain1);
                KDL::ChainIkSolverPos_NR_JL ik_solver(chain1,q_min,q_max,fk_solver,ik_solver_vel,IK_ITERATIONS,1e-6);
                KDL::Frame F(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(x,y,z));
                bool ik_res = (ik_solver.CartToJnt(q,F,q_out)>=0);

//                PRX_PRINT( "ik_res at this point is: " << ik_res, PRX_TEXT_LIGHTGRAY );
                
                // if(!ik_res)
                // {
                int count=0;
                while(!ik_res && count<0)
                {
                    for(unsigned i=0 ; i< 7 ; i++)
                    {
                        q(i) = uniform_random(q_min(i),q_max(i));
                    }
                    count++;
                    ik_res = (ik_solver.CartToJnt(q,F,q_out)>=0);
                }
                // }
                
                if(ik_res)
                {
                    std::vector<double> state_vec = state_var;
                    if(hand=="left")
                    {
                        state_vec[0] = q_out(0);
                        state_vec[1] = q_out(1);
                        state_vec[2] = q_out(2);
                        state_vec[3] = q_out(3);
                        state_vec[4] = q_out(4);
                        state_vec[5] = q_out(5);
                        state_vec[6] = q_out(6);
                        state_vec[7] = 0;
                        state_vec[15] = 0;
                    }
                    else
                    {
//                        PRX_PRINT( "Right hand", PRX_TEXT_LIGHTGRAY );
                        state_vec[0] = q_out(0);
                        state_vec[8] = q_out(1);
                        state_vec[9] = q_out(2);
                        state_vec[10] = q_out(3);
                        state_vec[11] = q_out(4);
                        state_vec[12] = q_out(5);
                        state_vec[13] = q_out(6);  
                        state_vec[14] = 0;                      
                    }
                    state_vec[16] = ( set_grasping ? (double)GRIPPER_CLOSED : GRIPPER_OPEN );
//                    PRX_PRINT( "state_vec[16] " << state_vec[16], PRX_TEXT_BROWN );
                    state_space->copy_vector_to_point( state_vec, computed_state );
                }
                
                return ik_res;                
            }

            bool motoman_plant_t::IK_solver( const config_t& effector_config, space_point_t* computed_state, bool set_grasping, const space_point_t* seed_state, bool do_min )
            {
                double qx,qy,qz,qw;
                double x,y,z;
                effector_config.get_orientation().get(qx,qy,qz,qw);
                effector_config.get_position(x,y,z);
                KDL::Chain chain1;
                my_tree->getChain("base_link",simple_ee_name,chain1);
                KDL::JntArray q(chain1.getNrOfJoints());
                KDL::JntArray q_out(chain1.getNrOfJoints());
                KDL::JntArray q_min(chain1.getNrOfJoints());
                KDL::JntArray q_max(chain1.getNrOfJoints());

                std::vector< double > state_var;
                if( seed_state != NULL )
                {
                    // PRX_PRINT( "I do have a seed: " << state_space->print_point( seed_state, 4 ) , PRX_TEXT_LIGHTGRAY );
                    state_space->copy_point_to_vector( seed_state, state_var );
                }
                else
                {
//                    PRX_PRINT( "I do not have a seed: ", PRX_TEXT_LIGHTGRAY );
                    state_space->copy_to_vector( state_var );
                }
                
                if( hand=="left" && unigripper )
                {
                    // PRX_PRINT( "Left hand and UniGripper", PRX_TEXT_LIGHTGRAY );
                    q(0) = state_var[0];
                    q(1) = state_var[1];
                    q(2) = state_var[2];
                    q(3) = state_var[3];
                    q(4) = state_var[4];
                    q(5) = state_var[5];
                    q(6) = state_var[6];
                    q(7) = state_var[7];
                    q(8) = state_var[15];
                    q_min(0) = -2;//state_space->get_bounds()[0]->get_lower_bound();
                    q_min(1) = state_space->get_bounds()[1]->get_lower_bound();
                    q_min(2) = state_space->get_bounds()[2]->get_lower_bound();
                    q_min(3) = state_space->get_bounds()[3]->get_lower_bound();
                    q_min(4) = state_space->get_bounds()[4]->get_lower_bound();
                    q_min(5) = state_space->get_bounds()[5]->get_lower_bound();
                    q_min(6) = state_space->get_bounds()[6]->get_lower_bound();
                    q_min(7) = state_space->get_bounds()[7]->get_lower_bound();
                    q_min(8) = state_space->get_bounds()[15]->get_lower_bound();
                    
                    q_max(0) = .86;//state_space->get_bounds()[0]->get_upper_bound();
                    q_max(1) = state_space->get_bounds()[1]->get_upper_bound();
                    q_max(2) = state_space->get_bounds()[2]->get_upper_bound();
                    q_max(3) = state_space->get_bounds()[3]->get_upper_bound();
                    q_max(4) = state_space->get_bounds()[4]->get_upper_bound();
                    q_max(5) = state_space->get_bounds()[5]->get_upper_bound();
                    q_max(6) = state_space->get_bounds()[6]->get_upper_bound();
                    q_max(7) = state_space->get_bounds()[7]->get_upper_bound();
                    q_max(8) = state_space->get_bounds()[15]->get_upper_bound();
                }
                else if(hand=="left")
                {
                    // PRX_PRINT( "Left hand and Not UniGripper", PRX_TEXT_LIGHTGRAY );
                    q(0) = state_var[0];
                    q(1) = state_var[1];
                    q(2) = state_var[2];
                    q(3) = state_var[3];
                    q(4) = state_var[4];
                    q(5) = state_var[5];
                    q(6) = state_var[6];
                    q(7) = state_var[7];
                    q_min(0) = -2;//state_space->get_bounds()[0]->get_lower_bound();
                    q_min(1) = state_space->get_bounds()[1]->get_lower_bound();
                    q_min(2) = state_space->get_bounds()[2]->get_lower_bound();
                    q_min(3) = state_space->get_bounds()[3]->get_lower_bound();
                    q_min(4) = state_space->get_bounds()[4]->get_lower_bound();
                    q_min(5) = state_space->get_bounds()[5]->get_lower_bound();
                    q_min(6) = state_space->get_bounds()[6]->get_lower_bound();
                    q_min(7) = state_space->get_bounds()[7]->get_lower_bound();
                    
                    q_max(0) = .86;//state_space->get_bounds()[0]->get_upper_bound();
                    q_max(1) = state_space->get_bounds()[1]->get_upper_bound();
                    q_max(2) = state_space->get_bounds()[2]->get_upper_bound();
                    q_max(3) = state_space->get_bounds()[3]->get_upper_bound();
                    q_max(4) = state_space->get_bounds()[4]->get_upper_bound();
                    q_max(5) = state_space->get_bounds()[5]->get_upper_bound();
                    q_max(6) = state_space->get_bounds()[6]->get_upper_bound();
                    q_max(7) = state_space->get_bounds()[7]->get_upper_bound();
                }
                else
                {
//                    PRX_PRINT( "Right hand", PRX_TEXT_LIGHTGRAY );
                    q(0) = state_var[0];
                    q(1) = state_var[8];
                    q(2) = state_var[9];
                    q(3) = state_var[10];
                    q(4) = state_var[11];
                    q(5) = state_var[12];
                    q(6) = state_var[13];
                    q(7) = state_var[14];
                    q_min(0) = -.86;//state_space->get_bounds()[0]->get_lower_bound();
                    q_min(1) = state_space->get_bounds()[8]->get_lower_bound();
                    q_min(2) = state_space->get_bounds()[9]->get_lower_bound();
                    q_min(3) = state_space->get_bounds()[10]->get_lower_bound();
                    q_min(4) = state_space->get_bounds()[11]->get_lower_bound();
                    q_min(5) = state_space->get_bounds()[12]->get_lower_bound();
                    q_min(6) = state_space->get_bounds()[13]->get_lower_bound();
                    q_min(7) = state_space->get_bounds()[14]->get_lower_bound();
                    q_max(0) = 2;//state_space->get_bounds()[0]->get_upper_bound();
                    q_max(1) = state_space->get_bounds()[8]->get_upper_bound();
                    q_max(2) = state_space->get_bounds()[9]->get_upper_bound();
                    q_max(3) = state_space->get_bounds()[10]->get_upper_bound();
                    q_max(4) = state_space->get_bounds()[11]->get_upper_bound();
                    q_max(5) = state_space->get_bounds()[12]->get_upper_bound();
                    q_max(6) = state_space->get_bounds()[13]->get_upper_bound();
                    q_max(7) = state_space->get_bounds()[14]->get_upper_bound();
                }
                
                KDL::ChainFkSolverPos_recursive fk_solver(chain1);
                KDL::ChainIkSolverVel_pinv ik_solver_vel(chain1);
                KDL::ChainIkSolverPos_NR_JL ik_solver(chain1,q_min,q_max,fk_solver,ik_solver_vel,IK_ITERATIONS,1e-3);
                KDL::Frame F(KDL::Rotation::Quaternion(qx,qy,qz,qw),KDL::Vector(x,y,z));
                
                bool ik_res = ( ik_solver.CartToJnt( q, F, q_out ) >= 0 );

                // PRX_PRINT( "ik_res at this point is: " << ik_res, PRX_TEXT_LIGHTGRAY );
                
                // if(!ik_res)
                // {
                int count=0;
                while(!ik_res && count<0)
                {
                    for(unsigned i=0 ; i< (hand=="left" && unigripper ? 9 : 8) ; i++)
                    {
                        q(i) = uniform_random(q_min(i),q_max(i));
                    }
                    count++;
                    ik_res = (ik_solver.CartToJnt(q,F,q_out)>=0);
                }
                // }
                
                if(ik_res)
                {
                    std::vector<double> state_vec = state_var;
                    if(hand=="left" && unigripper)
                    {
                        // PRX_PRINT( "Left hand and UniGripper", PRX_TEXT_LIGHTGRAY );
                        state_vec[0] = q_out(0);
                        state_vec[1] = q_out(1);
                        state_vec[2] = q_out(2);
                        state_vec[3] = q_out(3);
                        state_vec[4] = q_out(4);
                        state_vec[5] = q_out(5);
                        state_vec[6] = q_out(6);
                        state_vec[7] = q_out(7);
                        state_vec[15] = q_out(8);
                    }
                    else if(hand=="left")
                    {
                        // PRX_PRINT( "Left hand", PRX_TEXT_LIGHTGRAY );
                        state_vec[0] = q_out(0);
                        state_vec[1] = q_out(1);
                        state_vec[2] = q_out(2);
                        state_vec[3] = q_out(3);
                        state_vec[4] = q_out(4);
                        state_vec[5] = q_out(5);
                        state_vec[6] = q_out(6);
                        state_vec[7] = q_out(7);
                    }
                    else
                    {
//                        PRX_PRINT( "Right hand", PRX_TEXT_LIGHTGRAY );
                        state_vec[0] = q_out(0);
                        state_vec[8] = q_out(1);
                        state_vec[9] = q_out(2);
                        state_vec[10] = q_out(3);
                        state_vec[11] = q_out(4);
                        state_vec[12] = q_out(5);
                        state_vec[13] = q_out(6);
                        state_vec[14] = q_out(7);                        
                    }
                    state_vec[16] = ( set_grasping ? (double)GRIPPER_CLOSED : GRIPPER_OPEN );
//                    PRX_PRINT( "state_vec[16] " << state_vec[16], PRX_TEXT_BROWN );
                    state_space->copy_vector_to_point( state_vec, computed_state );
                }
                
                return ik_res;

          //       config_t zero_configuration;
          //       config_t unit_forward_configuration;
          //       config_t unit_right_configuration;

          //       zero_configuration.set_position( 0, 0, 0 );
          //       zero_configuration.set_orientation( 0, 0, 0, 1 );
          //       zero_configuration.relative_to_global( effector_config );
             //    unit_forward_configuration.set_position( 0, 0, 0 );
             //    unit_forward_configuration.set_orientation( 0, 0, 0, 1 );
             //    unit_forward_configuration.relative_to_global( effector_config );
             //    unit_right_configuration.set_position( 0, 0, 0 );
             //    unit_right_configuration.set_orientation( 0, 0, 0, 1 );
             //    unit_right_configuration.relative_to_global( effector_config );

          //       //Get the robot state in vector form
          //       std::vector< double > state_var;
          //       if( seed_state != NULL )
          //           state_space->copy_point_to_vector( seed_state, state_var );
          //       else
          //           state_space->copy_to_vector( state_var );

          //       bool success = false;
                // std::vector< unsigned int > ids;
          //       unsigned int id = model.GetBodyId("arm_left_link_tool0");
                // ids.push_back(id);
                // ids.push_back(id);
                // ids.push_back(id);
          //       unsigned int id_base = model.GetBodyId("base_link");
                // ids.push_back(id_base);
                // ids.push_back(id_base);
                // ids.push_back(id_base);
                // // ids.push_back(id);
                // std::vector< Vector3d > body_points;
                // std::vector< Vector3d > target_pos;
                // VectorNd seed = VectorNd::Zero(model.dof_count);
                // for(unsigned i=0;i<15;i++)
                // {
                //     seed[i+6] = state_var[i];
                // }
                // VectorNd result = VectorNd::Zero(model.dof_count);
                // body_points.push_back(Vector3d::Zero());
                // body_points.push_back(Vector3d::Zero());
                // body_points.push_back(Vector3d::Zero());
                // body_points.push_back(Vector3d::Zero());
                // body_points.push_back(Vector3d::Zero());
                // body_points.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // target_pos.push_back(Vector3d::Zero());
                // zero_configuration.get_position(x,y,z);
                // target_pos[0][0] = x;
                // target_pos[0][1] = y;
                // target_pos[0][2] = z;
                // // unit_forward_configuration.get_position(x,y,z);
                // target_pos[1][0] = x;
                // target_pos[1][1] = y;
                // target_pos[1][2] = z;
                // // zero_configuration.get_position(x,y,z);
                // target_pos[2][0] = x;
                // target_pos[2][1] = y;
                // target_pos[2][2] = z;

                // target_pos[3][0] = 0;
                // target_pos[3][1] = 0;
                // target_pos[3][2] = 0;

                // target_pos[4][0] = 0;
                // target_pos[4][1] = 0;
                // target_pos[4][2] = 1;

                // target_pos[5][0] = 1;
                // target_pos[5][1] = 0;
                // target_pos[5][2] = 0;

                // //
                // body_points[0][0] = 0;
                // body_points[0][1] = 0;
                // body_points[0][2] = 0;

                // body_points[1][0] = 0;
                // body_points[1][1] = 0;
                // body_points[1][2] = 0;

                // body_points[2][0] = 0;
                // body_points[2][1] = 0;
                // body_points[2][2] = 0;

                // body_points[3][0] = 0;
                // body_points[3][1] = 0;
                // body_points[3][2] = 0;

                // body_points[4][0] = 0;
                // body_points[4][1] = 0;
                // body_points[4][2] = 1;

                // body_points[5][0] = 1;
                // body_points[5][1] = 0;
                // body_points[5][2] = 0;



                // success = InverseKinematics(model,seed,ids,body_points,target_pos,result,1.0e-24,0.01,10000);
                // PRX_INFO_S(success<<" "<<result);
                // // Q = result;


          //       //Check for failure
          //       if( !success )
          //           return false;

          //       //Copy over the IK solution
          //       std::vector<double> state_vec = boost::assign::list_of(result(6+0))(result(6+1))(result(6+2 ))(result(6+3 ))(result(6+4 ))(result(6+5 ))(result(6+6 ))(result(6+7))
          //                                                             (result(6+8))(result(6+9))(result(6+10))(result(6+11))(result(6+12))(result(6+13))(result(6+14));
          //       state_vec.push_back( set_grasping ? (double)GRIPPER_CLOSED : GRIPPER_OPEN );
          //       state_space->copy_vector_to_point( state_vec, computed_state );

          //       return true;
            }

            bool motoman_plant_t::is_grasping() const
            {
                return (_grasping == 1);
            }

            bool motoman_plant_t::IK_steering( const config_t& start_config, const config_t& goal_config, sim::plan_t& result_plan, bool set_grasping )
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

            bool motoman_plant_t::IK_steering_general( const config_t& start_config, const config_t& goal_config, sim::plan_t& result_plan, bool set_grasping, bool camera_link )
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
                if( ( camera_link && !get_utility_IK( start_config, prev_st, set_grasping, prev_st, true ) ) ||
                    (!camera_link && !IK_solver( start_config, prev_st, set_grasping, prev_st, true ) ) )
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
                    if( ( camera_link && get_utility_IK( interpolated_config, inter_st, set_grasping, prev_st, true ) ) ||
                        (!camera_link && IK_solver( interpolated_config, inter_st, set_grasping, prev_st, true ) ) )
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



