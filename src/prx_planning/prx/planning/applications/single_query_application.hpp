/**
 * @file single_query.hpp 
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

#include "prx/planning/applications/planning_application.hpp"
#include "prx/planning/modules/path_smoothers/path_smoother.hpp"
#include "prx/planning/queries/query.hpp"

#ifndef PRX_SINGLE_QUERY_APPLICATION_HPP
#define	PRX_SINGLE_QUERY_APPLICATION_HPP

namespace prx
{
    namespace plan
    {

        class criterion_t;

        /**
         * @anchor single_query_application_t
         *
         * This application processes single-query, open-loop motion planning.  The
         * application reads an input query, constructs a plan for the system, and
         * transmits the computed plan, then ceases execution.
         *
         * @brief <b> Planning application for single-shot planning tasks. </b>
         *
         * @author Andrew Kimmel
         */
        class single_query_application_t : public planning_application_t
        {

          public:
            single_query_application_t();
            virtual ~single_query_application_t();

            /**
             * @copydoc planning_application_t::init()
             */
            virtual void init(const util::parameter_reader_t* reader);

            /**
             * @copydoc planning_application_t::execute()
             *
             * This execute function will actually terminate once the planning is
             * completed.
             */
            virtual void execute();

          private:
            /** @brief The query which this single shot application is answering. */
            query_t* the_query;
            /** @brief The set of interruption criteria which might interrupt the planning. */
            std::vector<criterion_t*> interruption_criteria;
            /** @brief A flag indicating whether the planning is allowed to be interrupted. */
            bool interruptable;
            /** @brief Path smoother used by this application to refine solution paths. */
            path_smoother_t* path_smoother;
        };

    }
}

#endif

