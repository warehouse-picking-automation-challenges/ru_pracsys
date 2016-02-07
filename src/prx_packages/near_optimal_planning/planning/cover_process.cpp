/**
 * @file cover_process.cpp
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

#include "planning/cover_process.hpp"

#include "prx/utilities/definitions/sys_clock.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/goals/goal.hpp"
#include "prx/planning/modules/stopping_criteria/stopping_criteria.hpp"
#include "prx/planning/queries/motion_planning_query.hpp"
#include "prx/planning/communication/visualization_comm.hpp"

#include <pluginlib/class_list_macros.h>
#include <boost/assign/list_of.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/range/adaptor/map.hpp>

PLUGINLIB_EXPORT_CLASS( prx::packages::near_optimal_planning::coverage_process_t, prx::plan::planner_t)

namespace prx
{
    using namespace util;
    using namespace sim;
    
    namespace packages
    {
        namespace near_optimal_planning
        {
            
            coverage_process_t::coverage_process_t()
            {
            }
            
            coverage_process_t::~coverage_process_t()
            {
            }
            
            void coverage_process_t::init(const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                prm_star_t::init( reader, template_reader );
                ballrad = reader->get_attribute_as<double>("beta_naught");
            }
            
            void coverage_process_t::setup()
            {
                prm_star_t::setup();
                //Initialize the data structures
                std::vector< std::vector< double > > v_points;
                state_t* pt;
                
                // =-=-=-=
                // Barrier
                // =-=-=-=
//                //First leg...
//                v_points.push_back( boost::assign::list_of<double>(-4.5)(-4)(1) );
//                v_points.push_back( boost::assign::list_of<double>(-69.0/16.0)(-15.0/4.0)(11.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-66.0/16.0)(-14.0/4.0)(14.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-63.0/16.0)(-13.0/4.0)(17.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-60.0/16.0)(-12.0/4.0)(20.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-57.0/16.0)(-11.0/4.0)(23.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-54.0/16.0)(-10.0/4.0)(26.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-51.0/16.0)(-9.0/4.0)(29.0/8.0) );
//                v_points.push_back( boost::assign::list_of<double>(-3)(-2)(4) );
//                //In the first hole
//                v_points.push_back( boost::assign::list_of<double>(-2.5)(-2)(4) );
//                //Thrid leg...
//                v_points.push_back( boost::assign::list_of<double>(-2)(-2)(4) );
//                v_points.push_back( boost::assign::list_of<double>(-18.0/11.0)(-19.0/11.0)(47.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(-14.0/11.0)(-16.0/11.0)(50.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(-10.0/11.0)(-13.0/11.0)(53.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(-6.0/11.0)(-10.0/11.0)(56.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(-2.0/11.0)(-7.0/11.0)(59.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(2.0/11.0)(-4.0/11.0)(62.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(6.0/11.0)(-1.0/11.0)(65.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(10.0/11.0)(2.0/11.0)(68.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(14.0/11.0)(5.0/11.0)(71.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(18.0/11.0)(8.0/11.0)(74.0/11.0) );
//                v_points.push_back( boost::assign::list_of<double>(2)(1)(7) );
//                //In the second hole
//                v_points.push_back( boost::assign::list_of<double>(2.5)(1)(7) );
//                //Final leg...
//                v_points.push_back( boost::assign::list_of<double>(3)(1)(7) );
//                v_points.push_back( boost::assign::list_of<double>(51.0/16.0)(11.0/8.0)(29.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(54.0/16.0)(14.0/8.0)(30.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(57.0/16.0)(17.0/8.0)(31.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(60.0/16.0)(20.0/8.0)(32.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(63.0/16.0)(23.0/8.0)(33.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(66.0/16.0)(26.0/8.0)(34.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(69.0/16.0)(29.0/8.0)(35.0/4.0) );
//                v_points.push_back( boost::assign::list_of<double>(4.5)(4)(9) );
                
                // =-=-=-=-=
                // Maze
                // =-=-=-=-=
                // //First leg
                // v_points.push_back( boost::assign::list_of<double>(0)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-5)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-10)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-15)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-20)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-25)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-30)(-80) );
                // v_points.push_back( boost::assign::list_of<double>(-35)(-80) );
                // //First Corner
                // v_points.push_back( boost::assign::list_of<double>(-40)(-80) );
                // //Second Leg
                // v_points.push_back( boost::assign::list_of<double>(-40)(-75) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-70) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-65) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-60) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-55) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-50) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-45) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-40) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-35) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-30) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-25) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-20) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-15) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-10) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(-5) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(0) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(5) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(10) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(15) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(20) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(25) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(30) );
                // v_points.push_back( boost::assign::list_of<double>(-40)(35) );
                // //Second Corner
                // v_points.push_back( boost::assign::list_of<double>(-40)(40) );
                // //Third Leg
                // v_points.push_back( boost::assign::list_of<double>(-35)(40) );
                // v_points.push_back( boost::assign::list_of<double>(-30)(40) );
                // v_points.push_back( boost::assign::list_of<double>(-25)(40) );
                // v_points.push_back( boost::assign::list_of<double>(-20)(40) );
                // v_points.push_back( boost::assign::list_of<double>(-15)(40) );
                // v_points.push_back( boost::assign::list_of<double>(-10)(40) );
                // v_points.push_back( boost::assign::list_of<double>(-5)(40) );
                // //Third Corner
                // v_points.push_back( boost::assign::list_of<double>(0)(40) );
                // //Last Leg
                // v_points.push_back( boost::assign::list_of<double>(0)(35) );
                // v_points.push_back( boost::assign::list_of<double>(0)(30) );
                // v_points.push_back( boost::assign::list_of<double>(0)(25) );
                // v_points.push_back( boost::assign::list_of<double>(0)(20) );
                // v_points.push_back( boost::assign::list_of<double>(0)(15) );
                // v_points.push_back( boost::assign::list_of<double>(0)(10) );
                // v_points.push_back( boost::assign::list_of<double>(0)(5) );
                // v_points.push_back( boost::assign::list_of<double>(0)(0) );

                // =-=-=-=
                // Empty
                // =-=-=-=
                //Only leg
                v_points.push_back( boost::assign::list_of<double>(-10)(0) );
                v_points.push_back( boost::assign::list_of<double>(-9)(0) );
                v_points.push_back( boost::assign::list_of<double>(-8)(0) );
                v_points.push_back( boost::assign::list_of<double>(-7)(0) );
                v_points.push_back( boost::assign::list_of<double>(-6)(0) );
                v_points.push_back( boost::assign::list_of<double>(-5)(0) );
                v_points.push_back( boost::assign::list_of<double>(-4)(0) );
                v_points.push_back( boost::assign::list_of<double>(-3)(0) );
                v_points.push_back( boost::assign::list_of<double>(-2)(0) );
                v_points.push_back( boost::assign::list_of<double>(-1)(0) );
                v_points.push_back( boost::assign::list_of<double>(0)(0) );
                v_points.push_back( boost::assign::list_of<double>(1)(0) );
                v_points.push_back( boost::assign::list_of<double>(2)(0) );
                v_points.push_back( boost::assign::list_of<double>(3)(0) );
                v_points.push_back( boost::assign::list_of<double>(4)(0) );
                v_points.push_back( boost::assign::list_of<double>(5)(0) );
                v_points.push_back( boost::assign::list_of<double>(6)(0) );
                v_points.push_back( boost::assign::list_of<double>(7)(0) );
                v_points.push_back( boost::assign::list_of<double>(8)(0) );
                v_points.push_back( boost::assign::list_of<double>(9)(0) );
                v_points.push_back( boost::assign::list_of<double>(10)(0) );

                // =-=-=-=
                //  Torus
                // =-=-=-=
                //Only leg
                //v_points.push_back( boost::assign::list_of<double>(-1)(-1)(-1) );
                //v_points.push_back( boost::assign::list_of<double>(-0.6)(-0.6)(-0.6) );
                //v_points.push_back( boost::assign::list_of<double>(-0.2)(-0.2)(-0.2) );
                //v_points.push_back( boost::assign::list_of<double>(0.2)(0.2)(0.2) );
                //v_points.push_back( boost::assign::list_of<double>(0.6)(0.6)(0.6) );
                //v_points.push_back( boost::assign::list_of<double>(1)(1)(1) );

                firsts.resize( v_points.size() );
                for( unsigned i=0; i<v_points.size(); ++i )
                {
                    pt = state_space->alloc_point();
                    state_space->set_from_vector( v_points[i], pt );
                    centers.push_back( state_space->clone_point( pt ) );
                    firsts[i] = NULL;
                }
                
                //            std::ofstream o;
                //            o.open("/Users/chuples/Desktop/coverballs.yaml");
                //            o << "covering_set:\n";
                //            o << "    type: obstacle\n";
                //            o << "    geometries:\n";
                //            for( unsigned i=0; i<v_points.size(); ++i )
                //            {
                //                o << "    -\n";
                //                o << "        name: ball" << i << "\n";
                //                o << "        collision_geometry:\n";
                //                o << "            type: sphere\n";
                //                o << "            radius: 0.25\n";
                //                o << "            material: pink\n";
                //                o << "        config:\n";
                //                o << "            position: [" << v_points[i][0] << "," << v_points[i][1] << "," << v_points[i][2] << "]\n";
                //                o << "            orientation: [0,0,0,1]\n";
                //            }
                //            o.close();
                //            o.clear();
                //            
                //            PRX_LOG_ERROR("Done creating file");
            }
            
            void coverage_process_t::resolve_query()
            {
                if( last_solution_length < PRX_INFINITY )
                    return;
                
                if( succeeded() )
                {
                    last_solution_length = 0;
                    for( unsigned i=0; i<firsts.size()-1; ++i )
                    {
                        last_solution_length += state_space->distance( firsts[i], firsts[i+1] );
                    }
                }
            }
            
            bool coverage_process_t::succeeded()
            {
                for( unsigned i=0; i<firsts.size(); ++i )
                {
                    if( firsts[i] == NULL )
                        return false;
                }
                return true;
            }
            
            void coverage_process_t::link_node_to_neighbors(util::directed_vertex_index_t v, const std::vector< const util::abstract_node_t* >& neighbors)
            {
                for( unsigned i=0; i<firsts.size(); ++i )
                {
                    if( firsts[i] == NULL )
                    {
                        if( state_space->distance( graph[v]->point, centers[i] ) <= ballrad + PRX_DISTANCE_CHECK )
                        {
                            firsts[i] = graph[v]->point;
                        }
                    }
                }
            }

        }
    }
}




