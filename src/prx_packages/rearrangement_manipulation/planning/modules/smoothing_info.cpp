/**
 * @file smoothing_info.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Rahul Shome Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include "planning/modules/smoothing_info.hpp"
#include "prx/simulation/trajectory.hpp"
#include "prx/simulation/plan.hpp"
#include <fstream>
#include <sstream>
#include <set>

namespace prx
{
    using namespace util;
    using namespace sim;

    namespace packages
    {
        namespace rearrangement_manipulation
        {

            smoothing_info_t::smoothing_info_t() { }

            smoothing_info_t::smoothing_info_t(const util::space_t* control_space)
            {
                plan.link_control_space(control_space);
            }

            smoothing_info_t::~smoothing_info_t() { }

            bool smoothing_info_t::is_constrained_by(unsigned pose)
            {
                return from_pose == pose || to_pose == pose || constraints.count(pose) == 1;
            }

            std::string smoothing_info_t::print() const
            {
                std::stringstream output(std::stringstream::out);

                output << object_id << ") " << from_pose << " -> " << to_pose << " |P|:" << plan.size() << "  c:";

                foreach(unsigned i, constraints)
                {
                    output << i << " , ";
                }
                return output.str();
            }
        }
    }
}
