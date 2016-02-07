/**
 * @file simulation_comm.hpp 
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

#ifndef PRX_PLANNING_SIMULATION_COMM_HPP
#define	PRX_PLANNING_SIMULATION_COMM_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/planning/communication/plan_base_communication.hpp"
#include "prx_simulation/query_msg.h"
#include "prx_simulation/state_msg.h"
#include "prx_simulation/plant_locations_msg.h"

namespace prx
{
    namespace util
    {
        class space_t;
    }

    namespace sim
    {
        class plan_t;
    }

    namespace plan
    {

        class planning_application_t;

        using namespace prx_utilities;
        using namespace prx_simulation;

        namespace comm
        {

            /**
             * @anchor simulation_comm_t
             *
             * This class is responsible for receiving messages and service calls
             * specifically from the simulation node.
             *
             * @brief <b> Planning node's communication class for talking to the simulator. </b>
             * 
             * @author Andrew Kimmel
             */
            class simulation_comm_t : public plan_base_communication_t
            {

              private:
                /** @brief Subscribes to the ground truth topic */
                ros::Subscriber ground_truth_subscriber;

                //            /** @brief  */
                //            ros::Subscriber planning_query_subscriber;

                /** @brief Subscribes to the plant locations topic */
                ros::Subscriber plant_locations_subscriber;

              public:
                simulation_comm_t();
                
                virtual void link_application(planning_application_t* parent_app);

                //            /**
                //             * @brief Request ground truth information from the simulation node.
                //             *
                //             * This function sends a request to the simulation node for ground
                //             * truth information.  This service prompts the simulation to begin
                //             * broadcasting ground truth information.
                //             * 
                //             * @param time Timestamp for the service request.
                //             */
                //            void request_ground_truth( double time );


                /**
                 * @brief Callback tied to an update of ground truth.
                 *
                 * This callback updates the planning application with ground truth
                 * information received from the simulation node.
                 *
                 * @param msg The message with ground truth data.
                 */
                void update_ground_truth_callback(const state_msg& msg);

                /**
                 * @brief Plant location callback function.
                 *
                 * Queries the planning application for the location of the plants
                 * in the planning node.
                 *
                 * @param msg The message to be processed.
                 */
                void plant_locations_callback(const plant_locations_msg& msg);
                
                void set_simulator_state (const std::vector<double>& point);

            };
            /** @brief The global simulation comm instance pointer. */
            extern plan_base_communication_t* sim_comm;
        }

    }

}

#endif

