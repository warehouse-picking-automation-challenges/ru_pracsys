/**
 * @file task_planner.hpp 
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
#ifndef PRX_TASK_PLANNER_HPP
#define	PRX_TASK_PLANNER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/planner.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx
{
    namespace plan
    {

        class query_t;
        class specification_t;
        
        /**
         * @anchor task_planner_t
         * 
         * An abstract class to encapsulate the higher level logic of the planning 
         * operations. Manages motion planners and other task planners in a heirarchical
         * fashion.
         * 
         * @brief <b> Abstract task planner class. </b>
         */
        class task_planner_t : public planner_t
        {

          public:
            task_planner_t();
            virtual ~task_planner_t();

            /** 
             * @copydoc planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t* )
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc planner_t::link_world_model( world_model_t* const )
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
             * @copydoc planner_t::reset()
             */
            virtual void reset();

            /**
             * @copydoc planner_t::set_param( const std::string&, const std::string&, const boost::any& )
             */
            virtual void set_param(const std::string& path, const std::string& parameter_name, const boost::any& value);

          protected:

            /**
             * @copydoc planner_t::set_param( const std::string&, const boost::any& )
             */
            virtual void set_param(const std::string& parameter_name, const boost::any& value);

            /** @brief The set name for this task planner */
            std::vector<std::string> planner_names;
            /** @brief A map of all the planners this particular task planner is responsible for. */
            util::hash_t<std::string, planner_t* > planners;
            /** @brief A map of all the space names in which this task planner operates. */
            util::hash_t<std::string, std::string> space_names;
            /** @brief Pointer to the current problem specification */
            specification_t* input_specification;
            /** @brief Pointer to the current planning query */
            query_t* input_query;
            /** @brief A map storing all of the queries for the planners in this task planner. */
            util::hash_t<std::string, specification_t* > output_specifications;
            /** @brief A map storing all of the queries for the planners in this task planner. */
            util::hash_t<std::string, query_t* > output_queries;
            /** @brief The world model linked to this task planner. */
            world_model_t* model;
        };

    }
}

#endif

