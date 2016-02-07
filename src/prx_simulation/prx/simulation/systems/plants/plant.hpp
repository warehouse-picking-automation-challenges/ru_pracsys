/**
 * @file plant.hpp
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

#pragma once

#ifndef PLANT_HPP
#define	PLANT_HPP

#include "prx/simulation/systems/system.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"

namespace prx
{
    namespace sim
    {

        /**
         * A convenient base class for systems that do not have subsystems. Plants represent the physical plants
         * that will be controlled by controllers.
         * Such systems must have a concrete state and control as well as associated spaces.
         * Systems that inherit from this must only implement \ref system_t::propagate() and \ref system_t::update_configs.
         *
         * @brief <b> An entity with a state and a way to change that state with a control. </b>
         *
         * @author Andrew Dobson, Athanasios Krontiris
         */
        class plant_t : public system_t
        {

          public:

            plant_t();
            virtual ~plant_t();

            /** @copydoc system_t::init (const util::parameter_reader_t * , const util::parameter_reader_t* )*/
            virtual void init(const util::parameter_reader_t * reader, const util::parameter_reader_t* template_reader = NULL);

            /** @copydoc system_t::get_state_space() const*/
            virtual const util::space_t* get_state_space() const;

            /** @copydoc system_t::get_control_space() const*/
            virtual const util::space_t* get_control_space() const;

            /** @copydoc integration_plant_t::propagate(const double)*/
            virtual void propagate(const double simulation_step = 0) = 0;

            /**
             * The plants cannot compute controls. This function will overwrite this function with
             * empty body.
             *
             * @brief Empty.
             */
            virtual void compute_control();

            /** @copydoc system_t::update_phys_configs(util::config_list_t&, unsigned& index) const */
            virtual void update_phys_configs(util::config_list_t &configs, unsigned& index) const;

            /** @copydoc system_t::update_phys_geoms(util::geom_map_t &) const */
            virtual void update_phys_geoms(util::geom_map_t &geoms) const;

            /** @copydoc system_t::get_sensed_geoms(util::geom_map_t &) const */
            virtual void get_sensed_geoms(util::geom_map_t &geoms) const;

            /** @copydoc system_t::set_param( const std::string&, const std::string&, const boost::any& ) */
            virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value);

            /** @copydoc system_t::update_system_graph()*/
            virtual system_graph_t::directed_vertex_t update_system_graph(system_graph_t& graph);

            virtual void update_collision_list(collision_list_t* white_list);

            /**
             * Returns the names of the geometries that the plant has.
             *
             * @brief Returns the names of the geometries that the plant has.
             *
             * @param geom_names Stores and returns all the full path name of the geometries
             */
            virtual std::vector<std::string>* get_geometries_names(); // <- Needed?

            /**
             * Apologies for bloating the interface... deadline mode.
             */
            virtual std::string get_root_geom_name();

            /** @copydoc system_t::verify() */
            virtual void verify() const;


            /** @copydoc system_t::append_contingency(plan_t&,double)*/
            void append_contingency(plan_t& result_plan, double duration);

          protected:

            /** The state \ref util::space_t for the plant*/
            util::space_t* state_space;

            /** The control \ref util::space_t that the plant is expecting for its parent controller.*/
            util::space_t* input_control_space;

            /** A map with all the geometries that construct the plant.*/
            util::geom_map_t geometries;

            /** The names for doing configuration updates in the config map. */
            std::vector< std::string > config_names;
            /**
             * In some sense these are the joints between the different geometries of the plant.
             * You will have to multiply the configuration of the main body in order to find the
             * correct new configuration for the other parts of the plant, if the plant is constructed
             * from more that one geometry.
             */
            util::config_list_t relative_config_map;
            /**
             * A white list for collisions between parts of the plant. Mostly the parts of the plant
             * are in black list, which means they can collide each other, (eg. for a car that is constructed
             * by a box and 4 cylinders, as wheels, the cylinder can collide with the chassis). There are
             * cases, thought, that we want to check for collisions between parts of the plant, for example if
             * we have a two arm robot we do not want the arms to collide between them, so we need to check for
             * collisions between the two arms.
             */
            std::vector<collision_pair_t> interior_white_list;

            /**
             * The name of the root geometry. The one that we can set the state of the plant. Using the \relative_config_map
             * we can construct the rest of the plant, with all the geometries in the correct position.
             */
            std::string root_geom;
            /** The root configuration for the system.*/
            mutable util::config_t root_config;

            /** @copydoc system_t::update_vis_info() const */
            virtual void update_vis_info() const;

            /** @copydoc system_t::set_param(const std::string&, const boost::any& )*/
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

        };

    }
}

#endif

