/**
 * @file obstacle.cpp 
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */


#include "prx/simulation/systems/obstacle.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/range/adaptor/map.hpp> //adaptors
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( prx::sim::obstacle_t, prx::sim::system_t)

namespace prx
{
    using namespace util;
    namespace sim
    {        

        obstacle_t::obstacle_t() { }

        obstacle_t::~obstacle_t() { }

        void obstacle_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {            
            std::string geom_name;
            root_config.zero();
            if(parameters::has_attribute("root_configuration",reader,template_reader))
            {
                parameters::initialize(&root_config,reader,"root_configuration",template_reader,"root_configuration");
            }

            foreach(const parameter_reader_t* r, parameters::get_list("geometries",reader,template_reader))
            {
                geom_name = pathname + "/" + r->get_attribute_as< std::string > ("name");
                geometries_names.push_back(geom_name);
                geometries[geom_name] = *(r->initialize_new<geometry_t > (std::string("collision_geometry")));
                // PRX_INFO_S(geom_name<<" "<<geometries[geom_name]);
                relative_configurations.push_back(config_list_element_t(geom_name, *(r->initialize_new<config_t > (std::string("config")))));
                configurations.push_back(relative_configurations.back());
                configurations.back().second.relative_to_global(root_config);
                if(comm::vis_comm!=NULL)
                {
                    comm::vis_comm->add_marker_to_array(geometries[geom_name],geom_name,relative_configurations.back().second);
                    PRX_INFO_S(relative_configurations.back().second.print());
                }
            }
        }

        const space_t* obstacle_t::get_state_space() const
        {
            throw invalid_operation_exception("obstacle_t::get_state_space()");
        }

        const space_t* obstacle_t::get_control_space() const
        {
            throw invalid_operation_exception("obstacle_t::get_control_space()");
        }

        void obstacle_t::propagate(const double simulation_step)
        {
            throw invalid_operation_exception("obstacle_t::propagate()");
        }

        void obstacle_t::compute_control()
        {
            throw invalid_operation_exception("obstacle_t::compute_control()");
        }

        void obstacle_t::update_phys_configs(config_list_t &configs,unsigned& index) const
        {
            foreach(config_list_element_t element, configurations)
            {
                if(configs.size()<=index)
                {
                    configs.push_back(element);
                }
                else
                {
                    configs[index] = element;
                }
                index++;
            }
        }

        void obstacle_t::update_phys_geoms(geom_map_t &geoms) const
        {
            foreach(std::string name, geometries | boost::adaptors::map_keys)
            geoms[name] = geometries[name];
        }
        
        void obstacle_t::get_sensed_geoms(geom_map_t &geoms) const
        {
            foreach(std::string name, geometries | boost::adaptors::map_keys)
            geoms[name] = geometries[name];
        }

        system_graph_t::directed_vertex_t obstacle_t::update_system_graph(system_graph_t& graph)
        {
            //TODO: Think if we want the obstacles to be in the graph and if they are plants.
            return graph.add_node(pathname, true);
        }

        void obstacle_t::set_param(const std::string& system_name, const std::string& parameter_name, const boost::any& value)
        {
            if( !system_name.empty() )
                throw invalid_path_exception(system_name);
            set_param(parameter_name, value);
        }

        void obstacle_t::verify() const
        {
            if( geometries.size() == 0 || configurations.size() == 0 )
                throw std::runtime_error("obstacle_t::verify() failed because either geometries or configurations are not initialized.");
        }

        std::vector<std::string>* obstacle_t::get_geometries_names()
        {
            return &geometries_names;
        }

        void obstacle_t::update_vis_info() const {
        }

        void obstacle_t::update_point_cloud(const std::string& parameter_name, const boost::any& value)
        {
            set_param(parameter_name,value);
        }

        void obstacle_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            std::string name;
            std::string member;
            boost::tie(name, member) = split_path(parameter_name);

            if( name == "configurations" )
            {
                std::string param;
                boost::tie(member, param) = split_path(member);
                for( unsigned i = 0; i < configurations.size(); i++ )
                {
                    if( member == configurations[i].first )
                        configurations[i].second.set_param(param, boost::any_cast< double >(value));
                }
            }
            else if( name == "geometries" )
            {
                geometries[ member ].set_params(boost::any_cast< std::vector<double>* >(value));
            }
            else
            {
                throw invalid_element_exception(name);
            }
        }

        void obstacle_t::update_root_configuration(const config_t& new_root)
        {
            root_config = new_root;

            unsigned size = configurations.size();
            for(unsigned i=0;i<size;i++)
            {
                configurations[i].second = relative_configurations[i].second;
                configurations[i].second.relative_to_global(root_config);
            }
        }

    }
}