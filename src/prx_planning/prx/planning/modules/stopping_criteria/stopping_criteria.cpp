/**
 * @file criteria.cpp 
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


#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/modules/stopping_criteria/element/criterion.hpp"
#include "prx/planning/modules/stopping_criteria/element/goal_criterion.hpp"
#include "prx/planning/motion_planners/motion_planner.hpp"
#include "prx/utilities/statistics/statistics.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

namespace prx 
{ 
    using namespace util;
    namespace plan
    {

stopping_criteria_t::stopping_criteria_t()
{
    criteria_check_type = ALL_CRITERIA;
}

stopping_criteria_t::~stopping_criteria_t()
{
    
}

void stopping_criteria_t::init(const parameter_reader_t* reader,const parameter_reader_t* template_reader )
{
    // Assumed to have a reader of in namespace "criteria"
    criteria_check_type = (criteria_check_type_t) parameters::get_attribute_as<int>("criteria_check_type", reader, template_reader, ALL_CRITERIA);

    parameter_reader_t::reader_map_t stopping_criterion_map;
    if(reader->has_attribute("elements"))
        stopping_criterion_map = reader->get_map("elements");
    else
        stopping_criterion_map = template_reader->get_map("elements");

    foreach(const parameter_reader_t::reader_map_t::value_type key_value, stopping_criterion_map)
    {
        PRX_DEBUG_S("Creating criterion from namespace: "<<key_value.first);
        const parameter_reader_t* child_template_reader = NULL;  
        std::string type, template_name;

        if(key_value.second->has_attribute("template"))
        {
            template_name = key_value.second->get_attribute("template");
            child_template_reader = new parameter_reader_t(template_name);  
        }
        stopping_criteria.push_back(parameters::initialize_from_loader<criterion_t>( "prx_planning", key_value.second,"",child_template_reader,""));
        stopping_criteria.back()->set_type(stopping_criteria.back()->get_type() + "_" + int_to_str(stopping_criteria.size()));
        if(child_template_reader!=NULL)
            delete child_template_reader;
    }
}

bool stopping_criteria_t::satisfied()
{
    bool ret, check;
    satisfied_criterion.clear();
    
    
    // First check the stopping criteria for satisfaction
    switch (criteria_check_type)
    {
        case ALL_CRITERIA:
            ret = 1;
            for (unsigned i = 0; i < stopping_criteria.size(); i++)
            {
                check = stopping_criteria[i]->criterion_check();
                
                // We stored any satisfied criterion 
                if (check)
                    satisfied_criterion.push_back(stopping_criteria[i]);
                ret *= check;
            }
            break;
            
        case SINGLE_CRITERION:
            ret = 0;
            for (unsigned i = 0; i < stopping_criteria.size(); i++)
            {
                check = stopping_criteria[i]->criterion_check();
                
                // We stored any satisfied criterion 
                if (check)
                {
                    satisfied_criterion.push_back(stopping_criteria[i]);
                }
                ret += check;
            }
            break;
            
        default:
            PRX_FATAL_S ("Criteria check type "<<criteria_check_type<<" not supported in stopping criterion");
            break;
    }
    if (stopping_criteria.empty())
        ret = 0;
    // If the stopping criterion is satisfied, based on its type, an exception is thrown
    if (ret)
        throw stopping_criteria_t::stopping_criteria_satisfied(" Stopping criteria is satisfied ");
    
    // The first interruption criterion that is satisfied will throw the exception
    for (unsigned i = 0; i < interruption_criteria.size(); i++)
    {
        if (interruption_criteria[i]->criterion_check())
        {
            // Since an interruption has been met, clear the vector
            interruption_criteria.clear();

            // Throw an interruption
            throw stopping_criteria_t::interruption_criteria_satisfied(" Interruption criteria is satisfied");
        }
    }
    
    return false;
}

void stopping_criteria_t::get_statistics(std::vector<statistics_t*>& stats)
{
    for (unsigned i = 0; i < stopping_criteria.size(); i++)
    {
        stats.push_back(stopping_criteria[i]->compute_statistics());
    }
}

const std::vector<criterion_t*>& stopping_criteria_t::get_satisfied_criterion()
{
    return satisfied_criterion;
}

void stopping_criteria_t::reset()
{
    PRX_DEBUG_COLOR ("Stopping criteria reset", PRX_TEXT_BROWN);
    for (unsigned i = 0; i < stopping_criteria.size(); i++)
    {
        stopping_criteria[i]->reset();
    }

    for (unsigned i = 0; i < interruption_criteria.size(); i++)
    {
        interruption_criteria[i]->reset();
    }
}

void stopping_criteria_t::link_motion_planner( motion_planner_t* mp )
{
//    PRX_DEBUG_S ("linking motion planner");
    linked_motion_planner = mp;
    for (unsigned i = 0; i < stopping_criteria.size(); i++)
    {
        stopping_criteria[i]->link_motion_planner(mp);
    }
    for (unsigned i = 0; i < interruption_criteria.size(); i++)
    {
        interruption_criteria[i]->link_motion_planner(mp);
    }
}

void stopping_criteria_t::link_goal( goal_t* new_goal )
{
//    PRX_DEBUG_S ("linking goal");
    for (unsigned i = 0; i < stopping_criteria.size(); i++)
    {
        stopping_criteria[i]->link_goal(new_goal);
    }
    for (unsigned i = 0; i < interruption_criteria.size(); i++)
    {
        interruption_criteria[i]->link_goal(new_goal);
    }
}

void stopping_criteria_t::set_criteria_check_type(criteria_check_type_t new_type)
{
    criteria_check_type = new_type;
}

void stopping_criteria_t::link_interruption_criteria(const std::vector<criterion_t*>& interruption_set)
{
    PRX_DEBUG_COLOR ("Interruption criteria linked", PRX_TEXT_RED);
    foreach(criterion_t* crit, interruption_set)
    {
        interruption_criteria.push_back(crit);
    }
}

void stopping_criteria_t::add_interruption_criterion(criterion_t* new_criterion)
{
    PRX_DEBUG_COLOR ("Interruption criterion added", PRX_TEXT_RED);
    interruption_criteria.push_back(new_criterion);
}

void stopping_criteria_t::add_criterion(criterion_t* new_criterion)
{
    PRX_DEBUG_S("Stopping criterion added");
    stopping_criteria.push_back(new_criterion);
 
}

std::ostream& operator<<(std::ostream& output, const stopping_criteria_t& criterion)
{
    for (unsigned i = 0; i < criterion.stopping_criteria.size(); i++)
    {
        output << "\n-- Criterion: "<< criterion.stopping_criteria[i]->get_type() << " \n----" << (criterion.stopping_criteria[i]->print_statistics());
    }
//    for (unsigned i = 0; i < criterion.interruption_criteria.size(); i++)
//    {
//        output << "\n-- Interruption Criterion " << i << " \n----" << (criterion.(*interruption_criteria)[i]->print_statistics());
//    }   
    return output;
}


    }
}