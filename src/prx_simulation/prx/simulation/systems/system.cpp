/**
 * @file system.cpp 
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

#include "prx/simulation/systems/system.hpp"
#include "prx/simulation/system_ptr.hpp"
#include "prx/utilities/spaces/space.hpp"

namespace prx
{
    using namespace util;
    namespace sim
    {


        pluginlib::ClassLoader<system_t> system_t::loader("prx_simulation", "prx::sim::system_t");

        system_t::system_t()
        {
            active = true;
            visualizable = true;
        }

        void system_t::add_system(const std::string& path, system_ptr_t system)
        {
            throw invalid_subsystem_tree_exception("Cannot add subsystem in this type of system");
        }

        void system_t::remove_system(const std::string& path)
        {
            throw invalid_subsystem_tree_exception("This system it does not have subsystem to remove");
        }

        void system_t::replace_system(const std::string& path, system_ptr_t system)
        {
            throw invalid_subsystem_tree_exception("This system it does not have subsystem to replace");
        }

        system_ptr_t system_t::get_system(const std::string& system_path) const
        {
            PRX_FATAL_S("Invalid path for get system");
            throw invalid_element_exception(system_path);
        }

        void system_t::steering_function(const state_t* start, const state_t* goal, plan_t& result_plan)
        {
            if(active)
            {
                PRX_FATAL_S("steering_function is not implemented for the system " << pathname.c_str());
            }
        }

        void system_t::append_contingency(plan_t& result_plan, double duration)
        {
            //    PRX_ASSERT(result_plan.length() >= duration);
            result_plan.augment_plan(duration);
        }

        void system_t::update_visualization_info() const
        {
            if( visualizable )
                update_vis_info();
        }

        bool system_t::is_active() const
        {
            return active;
        }

        void system_t::set_active(bool in_active, const std::string& path)
        {
            active = in_active;
        }

        void system_t::set_param(const std::string& parameter_name, const boost::any& value)
        {
            if( parameter_name == "visualizable" )
                visualizable = boost::any_cast< bool >(value);
            else if( parameter_name == "active" )
                active = boost::any_cast< bool >(value);
            else
                throw invalid_element_exception("The element " + parameter_name + " does not exist! ");
        }

        pluginlib::ClassLoader<system_t>& system_t::get_loader()
        {
            //    if(loader == NULL)
            //        loader = new pluginlib::ClassLoader<system_t>("prx_simulation","system_t");

            return loader;
        }

    }
}












