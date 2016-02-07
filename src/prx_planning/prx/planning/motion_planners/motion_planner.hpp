/**
 * @file motion_planner.hpp 
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

#ifndef PRX_MOTION_PLANNER_HPP
#define PRX_MOTION_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/simulation/state.hpp"
#include "prx/planning/planner.hpp"

////Ah man... need to figure out a better way to do this
//#include "prx/planning/modules/stopping_criteria/pno_criterion.hpp"

#include <pluginlib/class_loader.h>

namespace prx
{
    namespace util
    {
        class parameter_reader_t;
        class space_t;
        class statistics_t;
    }
    
    namespace plan
    {

        class world_model_t;
        class motion_planning_query_t;
        class motion_planning_specification_t;

        class pno_criterion_t;
        class goal_criterion_t;

        /**
         * @anchor motion_planner_t
         * 
         * A class to encapsulate the internals of a motion planning algorithm such as 
         * RRT or PRM.
         * 
         * @brief <b> Abstract motion planner class. </b>
         *
         * @author Andrew Kimmel
         */
        class motion_planner_t : public planner_t
        {

          public:

            motion_planner_t();

            virtual ~motion_planner_t();

            /** 
             * @copydoc planner_t::init() 
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc planner_t::link_world_model()
             */
            virtual void link_world_model(world_model_t * const model);

            /**
             * @copydoc planner_t::link_specification(specification_t*)
             */
            virtual void link_specification(specification_t* new_spec);
            
            /**
             * @copydoc planner_t::link_query(query_t*)
             */
            virtual void link_query(query_t* new_query);

            /**
             * @copydoc planner_t::execute()
             */
            virtual bool execute();

            /** 
             * @brief Performs a single atomic step in the planning process for this algorithm.
             */
            virtual void step() = 0;

            /**
             * @brief Computes the cost of the current solution.  Requires that query is linked.
             * @return The cost of the current solution.
             */
            virtual double compute_cost();

            /**
             * @copydoc planner_t::set_param( const std::string&, const std::string&, const boost::any& )
             */
            virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value);

            /**
             * @copydoc planner_t::get_statistics()
             */
            virtual const util::statistics_t* get_statistics();

            /**
             * @brief Data structure serialization method.
             *
             * Most motion planners create some structure for planning purposes.  This
             * method serializes these structures for long-term storage.
             *
             * @return A flag indicating success or failure of the serialization process.
             */
            virtual bool serialize();
            /**
             * @brief Data structure deserialization method.
             *
             * Most motion planners create some structure for planning purposes.  This
             * method deserializes a stored structure and initializes the planner with
             * this information.
             *
             * @return A flag indicating success or failure of the deserialization process.
             */
            virtual bool deserialize();

            /**
             * @brief Reports whether this motion planner is able to serialize its structures.
             *
             * @return A flag indicating whether the motion planner can serialize.
             */
            bool can_serialize();
            /**
             * @brief Reports whether this motion planner is able to deserialize stored structures.
             *
             * @return A flag indicating whether the motion planner can deserialize.
             */
            bool can_deserialize();

            friend class pno_criterion_t;
            friend class goal_criterion_t;

          protected:

            /**
             * @copydoc planner_t::set_param( const std::string&, const boost::any& )
             */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

            /** @brief Pointer to the current problem specification */
            motion_planning_specification_t* input_specification;
            /** @brief Pointer to the current planning query */
            motion_planning_query_t* input_query;

            /** @brief The state space over which this motion planner is planning. */
            const util::space_t* state_space;
            /** @brief The control space over which this motion planner is planning. */
            const util::space_t* control_space;

            /** @brief A vector of states to be checked by a goal criterion */
            std::vector<sim::state_t*> states_to_check;

            /** @brief A flag indicating whether or not this planner is able to serialize its planning information. */
            bool serialize_flag;
            /** @brief A flag indicating whether or not this planner is able to deserialize its planning information. */
            bool deserialize_flag;
            /** @brief The file to which the planner will serialize its information. */
            std::string serialization_file;
            /** @brief The file from which the planner will deserialize its information.*/
            std::string deserialization_file;

            /** @brief The internal statistics of this motion planner. */
            util::statistics_t* statistics;
        };

    }
}

#endif 
