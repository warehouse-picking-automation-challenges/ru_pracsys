/**
 * @file rrtc.hpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */
#pragma once

#ifndef PRX_RRTC_HPP
#define PRX_RRTC_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/planning/motion_planners/rrt/rrt.hpp"
#include "prx/utilities/goals/goal_state.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"

namespace prx
{
    namespace plan
    {

        /**
         * @brief <b> Randomly-exploring Random Tree </b>
         * @author Zakary Littlefield
         */
        class rrtc_t : public motion_planner_t
        {

          public:
            rrtc_t();
            virtual ~rrtc_t();

            /**
             * @copydoc motion_planner_t::init(const util::parameter_reader_t*, const util::parameter_reader_t*)
             */
            virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);

            /**
             * @copydoc motion_planner_t::reset()
             */
            virtual void reset();

            /**
             * @copydoc motion_planner_t::setup()
             */
            virtual void setup();

            /**
             * @copydoc motion_planner_t::link_query(query_t*)
             */
            virtual void link_query(query_t* new_query);

            /**
             * @copydoc motion_planner_t::step()
             */
            virtual void step();

            /**
             * @copydoc motion_planner_t::resolve_query()
             */
            virtual void resolve_query();

            /**
             * @copydoc motion_planner_t::succeeded() const
             */
            virtual bool succeeded() const;

            /**
             * @copydoc motion_planner_t::serialize()
             */
            virtual bool serialize();

            /**
             * @copydoc motion_planner_t::deserialize()
             */
            virtual bool deserialize();

            /**
             * @copydoc motion_planner_t::get_statistics()
             */
            virtual const util::statistics_t* get_statistics();

          protected:
            rrt_t* rrt[2];
            unsigned tree_a;
            unsigned tree_b;

            bool trees_met;

            util::goal_state_t* tree_goals[2];
            stopping_criteria_t* crits[2];

            virtual void update_vis_info() const;

        };

    }
}

#endif //PRX_RRTC_HPP
