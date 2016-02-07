/**
 * @file rbdl_plant.cpp 
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

#include "prx/simulation/systems/plants/rbdl_plant.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/plan.hpp"

#include <pluginlib/class_loader.h>
#include <boost/bind.hpp>
#include <boost/assign/list_of.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(prx::sim::rbdl_plant_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
#ifdef RBDL_FOUND
    using namespace RigidBodyDynamics::Addons;
    using namespace RigidBodyDynamics::Math;
    using namespace RigidBodyDynamics;
#endif
    namespace sim
    {

        rbdl_plant_t::rbdl_plant_t() : integration_plant_t()
        {
        }

        void rbdl_plant_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
        {

#ifdef RBDL_FOUND
            std::string filename = parameters::get_attribute_as<std::string>("urdf_file",reader,template_reader);

            URDFReadFromFile(filename.c_str(), &model, false);
            model.gravity = Vector3d (0., 0., 0. );
            Q = VectorNd::Zero (model.dof_count);
            QDot = VectorNd::Zero (model.dof_count);
            Tau = VectorNd::Zero (model.dof_count);
            QDDot = VectorNd::Zero (model.dof_count);

            for(unsigned i=0;i<model.dof_count; i++)
            {
                state_memory.push_back(&Q[i]);
            }
            for(unsigned i=0;i<model.dof_count; i++)
            {
                state_memory.push_back(&QDot[i]);
            }
            for(unsigned i=0;i<model.dof_count; i++)
            {
                control_memory.push_back(&Tau[i]);
            }

            state_space = new space_t("Baxter", state_memory);
            input_control_space = new space_t("BaxterControl", control_memory);

            parameter_reader_t::reader_map_t subsystem_map = parameters::get_map("config_offsets", reader, template_reader);

            foreach(const parameter_reader_t::reader_map_t::value_type key_value, subsystem_map)
            {
                std::vector<double> offset = key_value.second->get_attribute_as<std::vector<double> >("offset");
                VectorNd tmpVec = VectorNd::Zero(3);
                tmpVec[0] = offset[0];
                tmpVec[1] = offset[1];
                tmpVec[2] = offset[2];
                config_offsets[key_value.first] = tmpVec;
            }

            //    PRX_WARN_S("Init for rigid body ");
            integration_plant_t::init(reader, template_reader);
            foreach(std::string name, config_names)
            {
                simple_rigid_body_names.push_back(reverse_split_path(name).second);
            }
            ForwardDynamics(model, Q, QDot, Tau, QDDot);

            unsigned int id = model.GetBodyId("left_paddle");
            Vector3d pos = CalcBodyToBaseCoordinates(model,Q,id,VectorNd::Zero(3));
            PRX_INFO_S("Start End Effector State: "<<pos.transpose());

#endif

        }

        void rbdl_plant_t::propagate(const double simulation_step)
        {
#ifdef RBDL_FOUND

            integration_plant_t::propagate(simulation_step);
            UpdateKinematics(model, Q, QDot, QDDot);
#endif

        }

        void rbdl_plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {

#ifdef RBDL_FOUND
            bool update = true;
            for(unsigned i=0;i<simple_rigid_body_names.size();i++)
            {
                unsigned int id = model.GetBodyId(simple_rigid_body_names[i].c_str());
                VectorNd pos = CalcBodyToBaseCoordinates(model,Q,id,config_offsets[simple_rigid_body_names[i]],update);
                update = false;
                Matrix3d orient = CalcBodyWorldOrientation(model,Q,id,update);
                Quaternion quat = Quaternion::fromMatrix(orient);
                augment_config_list(configs,index);
                configs[index].first = config_names[i];
                configs[index].second.set_position(pos[0],pos[1],pos[2]);
                configs[index].second.set_orientation(quat[0],quat[1],quat[2],quat[3]);
                index++;
            }
#endif
        }

        void rbdl_plant_t::update_derivative(state_t * const result)
        {

#ifdef RBDL_FOUND
            //update state 
            ForwardDynamics(model, Q, QDot, Tau, QDDot);
            for(unsigned i=0;i<model.dof_count; i++)
            {
                result->memory[i] = QDot[i];
            }
            for(unsigned i=0;i<model.dof_count; i++)
            {
                result->memory[i+model.dof_count] = QDDot[i] - .7*QDot[i];
            }
#endif
        }

    }
}