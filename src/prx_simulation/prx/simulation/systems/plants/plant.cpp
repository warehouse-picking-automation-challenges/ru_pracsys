/**
 * @file plant.cpp
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

#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/simulation/communication/visualization_comm.hpp"

#include <boost/tuple/tuple.hpp> // boost::tie
#include <boost/range/adaptor/map.hpp> //adaptors

namespace prx
{
    using namespace util;
    namespace sim
    {

        plant_t::plant_t()
        {
            state_space = NULL;
            input_control_space = NULL;
        }

        plant_t::~plant_t()
        {
            if( state_space != NULL )
                delete state_space;
            if( input_control_space != NULL )
                delete input_control_space;
        }

        void plant_t::init(const parameter_reader_t * reader, const parameter_reader_t* template_reader)
        {
            if( reader->has_attribute("input_control_space") )
            {
                input_control_space->init(reader->get_child("input_control_space").get());
            }
            else if( template_reader != NULL )
            {
                input_control_space->init(template_reader->get_child("input_control_space").get());
            }
            if( reader->has_attribute("state_space") )
            {
                state_space->init(reader->get_child("state_space").get());
            }
            else if( template_reader != NULL )
            {
                state_space->init(template_reader->get_child("state_space").get());
            }

            if( parameters::has_attribute("initial_state", reader, template_reader) ) //   reader->has_element("initial_state") )
            {
                state_space->set_from_vector(parameters::get_attribute_as< std::vector<double> > ("initial_state", reader, template_reader));
            }
            else
            {
                state_space->zero();
            }
            if( parameters::has_attribute("initial_control", reader, template_reader) ) //   reader->has_element("initial_state") )
            {
                input_control_space->set_from_vector(parameters::get_attribute_as< std::vector<double> > ("initial_control", reader, template_reader));
            }
            else
            {
                input_control_space->zero();
            }

            if( parameters::has_attribute("root_geom", reader, template_reader) )
                root_geom = pathname + "/" + parameters::get_attribute_as< std::string > ("root_geom", reader, template_reader);
                /*tacos
                PRX_DEBUG_S("The root geometry is " << root_geom);
                */
            std::vector<const parameter_reader_t*> readers = parameters::get_list("geometries", reader, template_reader);
            std::string geom_name;

            foreach(const parameter_reader_t* r, readers)
            {
                geom_name = pathname + "/" + r->get_attribute_as< std::string > ("name");
                geometries[geom_name] = (*r->initialize_new<geometry_t > (std::string("collision_geometry")));
                if(comm::vis_comm!=NULL)
                    comm::vis_comm->add_marker_to_array(geometries[geom_name],geom_name);
                if( r->has_element("relative_configuration") )
                {
                    relative_config_map.push_back(config_list_element_t(geom_name, (*r->initialize_new<config_t > (std::string("relative_configuration")))));
                    //                    relative_config_map[geom_name] = (*r->initialize_new<config_t > (std::string("relative_configuration")));
                }
                config_names.push_back(geom_name);
            }

            int index = 0;
            /*tacos
             if( parameters::has_attribute("geometries", reader, template_reader) )
            {

                foreach(const parameter_reader_t* list_reader, parameters::get_list("geometries", reader, template_reader))
                {
                    index++;
                    std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                    if( r.size() != 2 )
                        PRX_FATAL_S("The pair " << index << " in white list of plant: " << pathname.c_str() << " is wrong. Has to be a systems with a list of systems.");

                    std::string name1 = r[0]->get_attribute("");

                    foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                    {
                        std::string name2 = systems_reader->get_attribute("");
                        interior_white_list.push_back(std::make_pair(pathname + "/" + name1, pathname + "/" + name2));
                    }
                }
            }
            */
            if( parameters::has_attribute("white_list", reader, template_reader) )
            {

                foreach(const parameter_reader_t* list_reader, parameters::get_list("white_list", reader, template_reader))
                {
                    index++;
                    std::vector<const parameter_reader_t*> r = list_reader->get_list("");

                    if( r.size() != 2 )
                        PRX_FATAL_S("The pair " << index << " in white list of plant: " << pathname.c_str() << " is wrong. Has to be a systems with a list of systems.");

                    std::string name1 = r[0]->get_attribute("");

                    foreach(const parameter_reader_t* systems_reader, r[1]->get_list(""))
                    {
                        std::string name2 = systems_reader->get_attribute("");
                        interior_white_list.push_back(std::make_pair(pathname + "/" + name1, pathname + "/" + name2));
                    }
                }
            }

        }

        const space_t* plant_t::get_state_space() const
        {
            return state_space;
        }

        const space_t* plant_t::get_control_space() const
        {
            return input_control_space;
        }

        void plant_t::compute_control() { }

        void plant_t::update_phys_configs(config_list_t& configs, unsigned& index) const
        {
            augment_config_list(configs,index);
            configs[index].first = root_geom;
            configs[index].second = root_config;
            index++;
            for( config_list_t::const_iterator config_iter = relative_config_map.begin(); config_iter != relative_config_map.end(); ++config_iter )
            {
                augment_config_list(configs,index);
                configs[index].first = config_iter->first;
                //////new
                configs[index].second = config_iter->second;
                configs[index].second.relative_to_global(root_config);
                index++;
                //////old
                // configs[index].second = root_config;
                // config_t tmp_config = config_iter->second;
                // //        vector_t vec(root_config.get_orientation().qv_rotation(tmp_config.get_position()));
                // tmp_config.set_position(vector_t(root_config.get_orientation().qv_rotation(tmp_config.get_position())));
                // configs[index].second += tmp_config;
                // index++;
            }
        }

        void plant_t::update_phys_geoms(geom_map_t& geoms) const
        {
            foreach(std::string name, geometries | boost::adaptors::map_keys)
            geoms[name] = geometries[name];
        }

        void plant_t::get_sensed_geoms(geom_map_t& geoms) const
        {
            foreach(std::string name, geometries | boost::adaptors::map_keys)
            geoms[name] = geometries[name];
        }

        void plant_t::set_param(const std::string& system_name, const std::string& parameter_name, const boost::any& value)
        {
            if( !system_name.empty() )
                throw invalid_path_exception(system_name);
            set_param(parameter_name, value);
        }

        system_graph_t::directed_vertex_t plant_t::update_system_graph(system_graph_t& graph)
        {
            //    PRX_DEBUG_S("The plant in the graph : " << pathname);
            system_graph_t::directed_vertex_t v = graph.add_node(pathname, true);
            return v;
        }

        void plant_t::update_collision_list(collision_list_t* white_list)
        {
            foreach(collision_pair_t p, interior_white_list)
            white_list->add_new_pair(p);
        }

        void plant_t::verify() const
        {
            if( state_space == NULL )
                throw invalid_system_exception("State space for the composite controller" + pathname + " is not initialized correctly.");
            else
                state_space->verify();
            if( input_control_space == NULL )
                throw invalid_system_exception("Control space for the composite controller" + pathname + " is not initialized correctly.");
            else
                input_control_space->verify();
        }

        void plant_t::update_vis_info() const { }

        void plant_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            std::string name;
            std::string member;
            boost::tie(name, member) = split_path(parameter_name);

            if( name == "geometries" )
            {
                geometries[ member ].set_params(boost::any_cast< std::vector<double>* >(value));
            }
            system_t::set_param(parameter_name, value);
        }

        std::vector<std::string>* plant_t::get_geometries_names()
        {
            return &config_names;
        }

        std::string plant_t::get_root_geom_name()
        {
            return root_geom;
        }

        void plant_t::append_contingency(plan_t& result_plan, double duration)
        {
            result_plan.link_control_space(input_control_space);
            result_plan.augment_plan(duration);
        }


    }
}

