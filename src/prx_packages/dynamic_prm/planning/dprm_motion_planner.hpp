/**
 * @file dprm_motion_planner.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Justin Cardoza, Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#ifndef PRX_DPRM_MOTION_PLANNER_HPP
#define PRX_DPRM_MOTION_PLANNER_HPP

#include "prx/planning/motion_planners/prm_star/prm_star.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include "prx/utilities/boost/boost_wrappers.hpp"
#include "prx/planning/world_model.hpp"
#include "dprm_astar.hpp"

#include <string>
#include <vector>



namespace prx
{
    namespace packages
    {
        namespace dynamic_prm
        {

            /**
             * @anchor dprm_motion_planner_t
             * 
             * A specialized version of PRM* which does its own collision checking 
             * for dealing with dynamic obstacles.
             * 
             * @brief <b> Version of PRM* for dealing with dynamic obstacles. </b>
             */
            class dprm_motion_planner_t : public plan::prm_star_t
            {

              public:
                dprm_motion_planner_t();
                virtual ~dprm_motion_planner_t();

                bool updateObstacles(util::hash_t<std::string, sim::plant_t*> & plants);

                virtual void init(const util::parameter_reader_t* reader, const util::parameter_reader_t* template_reader);
                virtual void setup();

                double distanceToSystem2(sim::state_t *state);

                /** 
                 * @copydoc prm_star_t::resolve_query()
                 */
                void resolve_query();

                /** 
                 * @copydoc motion_planner_t::execute()
                 */
                virtual bool execute();
                virtual void step();

                bool isGraphBuilt() const;
                void setPredictedTime(double t);
                void setWorldModel(plan::world_model_t *wm);

                void update_vis_info() const;


              protected:
                bool isColliding(util::space_point_t *plantState, plan::prm_star_edge_t *edge, double& edgeLength);


                std::vector<util::directed_edge_index_t> pathEdges;
                bool graphBuilt;
                sim::state_t *systemState;
                sim::plan_t previousPlan;
                dprm_astar_t astarSearch;
                double predictedTime;
                plan::world_model_t *worldModel;
            };
        }//namespace dynamic_prm
    }//namespace packages
}//namespace prx



#endif
