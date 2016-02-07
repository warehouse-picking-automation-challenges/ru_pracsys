/**
 * @file space_grid.cpp
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

#include "space_grid.hpp"
#include "prx/utilities/definitions/random.hpp"

#define GAMMA 0

namespace prx
{
    using namespace util;
    using namespace sim;
    using namespace plan;
    namespace packages
    {
        namespace sparse_rrt
        {

// #ifdef OCTAVE_FOUND
//             octave_caller_t* space_grid_t::caller = NULL;
// #endif

            double POW(double base, int exponent)
            {
                if(exponent<0)
                    PRX_FATAL_S("POW error "<<exponent);
                double val=1;
                for(int i=0;i<exponent;i++)
                {
                    val*=base;
                }
                return val;
            }


            space_grid_t::space_grid_t(std::vector<bounds_t*>& in_bounds,std::vector<unsigned> in_divisions)
            {
// #ifdef OCTAVE_FOUND
//                 if(caller==NULL)
//                     caller = new octave_caller_t();
// #endif
                bounds = in_bounds;
                dim = bounds.size();
                divisions = in_divisions;
                multiplier.resize(dim);
                cell_size.resize(dim);
                temp_index.resize(dim);
                min_val.resize(dim);
                intersection.resize(dim);
                number_of_cells=1;
                for(unsigned i=0;i<dim;i++)
                {
                    multiplier[i]=1;
                }
                for(unsigned i=0;i<dim;i++)
                {
                    intersection[i] = new bounds_t();
                    min_val[i] = bounds[i]->get_lower_bound();
                    number_of_cells*=divisions[i];
                    cell_size[i] = (bounds[i]->get_upper_bound() - bounds[i]->get_lower_bound())/divisions[i];
                    for(unsigned j=i+1;j<dim;j++)
                    {
                        multiplier[i] *= divisions[j];
                    }
                }
                multiplier[dim-1] = 1;
                grid = new cell*[number_of_cells];
                xs = new double*[number_of_cells];
                for(unsigned i=0;i<number_of_cells;i++)
                {
                    // grid[i].node=NULL;
                    // grid[i].time=PRX_INFINITY;
                    // grid[i].overlap = 0;
                    grid[i] = NULL;
                    xs[i]= new double[dim];
                }
                default_cell.node=NULL;
                default_cell.time=PRX_INFINITY;
                default_cell.overlap = 0;
                total_occupied = 0;

                // PRX_ERROR_S("L1\tU1");
                // for(unsigned i=0;i<dim;i++)
                // {
                //     PRX_ERROR_S(bounds[i]->get_lower_bound()<<"\t"<<
                //                 bounds[i]->get_upper_bound());
                // }
                control_field.resize(number_of_cells);
                collision_check = false;
            }

            space_grid_t::~space_grid_t()
            {
                foreach(control_t* ctrl, control_field)
                {
                    if(ctrl!=NULL)
                        ctrl_space->free_point(ctrl);
                }
                for(unsigned i=0;i<number_of_cells;i++)
                {
                    if(grid[i]!=NULL)
                        delete grid[i];
                    delete xs[i];
                }
                delete xs;
                delete grid;
            }
            cell* space_grid_t::get(std::vector<unsigned>& position)
            {
                unsigned pos = 0;
                for(unsigned i=0;i<dim;i++)
                {
                    pos += multiplier[i]*position[i];
                }
                return grid[pos];
            }
            unsigned space_grid_t::get_index(std::vector<unsigned>& position)
            {
                unsigned pos = 0;
                for(unsigned i=0;i<dim;i++)
                {
                    pos += multiplier[i]*position[i];
                }
                return pos;
            }

            cell* space_grid_t::get(space_point_t* point)
            {
                for(unsigned i=0;i<dim;i++)
                {
                    temp_index[i] = (unsigned) (point->at(i)-min_val[i])/cell_size[i];
                    if(temp_index[i]==divisions[i])
                        temp_index[i]--;
                    else if(temp_index[i]>divisions[i])
                        temp_index[i]=0;
                }
                return get(temp_index);
            }

            unsigned space_grid_t::get_flat_index(space_point_t* point)
            {
                for(unsigned i=0;i<dim;i++)
                {
                    temp_index[i] = (unsigned) (point->at(i)-min_val[i])/cell_size[i];
                    if(temp_index[i]==divisions[i])
                        temp_index[i]--;
                    else if(temp_index[i]>divisions[i])
                        temp_index[i]=0;
                }
                return get_index(temp_index);
            }
            unsigned space_grid_t::get_flat_index(std::vector<double>& point)
            {
                for(unsigned i=0;i<dim;i++)
                {
                    temp_index[i] = (unsigned) (point[i]-min_val[i])/cell_size[i];
                    if(temp_index[i]==divisions[i])
                        temp_index[i]--;
                    else if(temp_index[i]>divisions[i])
                        temp_index[i]=0;
                }
                return get_index(temp_index);
            }



            void space_grid_t::give_validity_checker(validity_checker_t* check, space_point_t* point, bool checking)
            {
                checker = check;
                test_point = point;
                collision_check = checking;
            }
            void space_grid_t::recompute_grid()
            {
                // for(unsigned i=0;i<grid.size();i++)
                // {
                //     grid[i].node=NULL;
                //     grid[i].time=PRX_INFINITY;
                //     grid[i].overlap = 0;
                // }
                // total_occupied = 0;
                // add_node(node, control_space, simulation_step, num_sim_steps);
                double cost;
                for(unsigned i=0;i<number_of_cells;i++)
                {
                    if(grid[i]!=NULL)
                    {
                        cost = grid[i]->node->cost;
                        grid[i]->time += (cost - old_cost);
                    }
                }
                old_cost = cost;
            }

            void space_grid_t::add_node(reachable_sparse_rrt_node_t* node, const space_t* control_space, double simulation_step, unsigned num_sim_steps)
            {

                ctrl_space = control_space;
                grid_weight = 0;
                this->simulation_step = simulation_step;
                base_node = node;
                for(unsigned i=0;i<dim;i++)
                    point_bounds.push_back(new bounds_t(base_node->point->at(i),base_node->point->at(i)));
                //add this node into the grid

                double cost = node->cost;
                old_cost = cost;
                //multipliers for indexing
                std::vector<unsigned> this_index;
                this_index.resize(dim);
                double* divide_cell_size = new double[dim];
                for(unsigned i=0;i<dim;i++)
                {
                    this_index[i] = 0;
                    divide_cell_size[i] = (.5/cell_size[i]);

                }
                level_set = new double[number_of_cells];

                //initialize the level set
                bool* collide = new bool[number_of_cells];
                double* delta_phi = new double[dim];
                unsigned* up_indices = new unsigned[dim*number_of_cells];
                unsigned* down_indices = new unsigned[dim*number_of_cells];

                double* cs = new double[number_of_cells];
                double* ss = new double[number_of_cells];


                double max_vel = control_space->get_bounds()[0]->get_upper_bound();
                double min_w = control_space->get_bounds()[1]->get_lower_bound();
                double max_w = control_space->get_bounds()[1]->get_upper_bound();

                for(unsigned i=0;i<number_of_cells;i++)
                {
                    level_set[i] = 0;
                    control_field[i] = NULL;
                    for(unsigned j=0;j<dim;j++)
                    {
                        xs[i][j] = min_val[j] + cell_size[j]*this_index[j];
                        test_point->at(j) = xs[i][j];
                        if(j==2)
                        {
                            double val = fabs(xs[i][j] - node->point->at(j));
                            if(val > PRX_PI)
                                val=2*PRX_PI - val;
                            level_set[i] += POW(val,2);
                        }
                        else
                        {
                            level_set[i] += POW(xs[i][j] - node->point->at(j),2);
                        }
                    }
                    if(!collision_check || checker->is_valid(test_point))
                    {
                        level_set[i] = sqrt(level_set[i])-3*cell_size[0];
                        collide[i] = false;
                    }
                    else
                    {
                        level_set[i] = PRX_INFINITY;
                        collide[i] = true;
                    }
                    cs[i] = (cos(xs[i][2]));
                    ss[i] = (sin(xs[i][2]));
                    for(unsigned j=0;j<dim;j++)
                    {
                        if(this_index[j]==0)
                        {
                            this_index[j]+=1;
                            up_indices[i*dim+j] = (get_index(this_index));
                            down_indices[i*dim+j] = (number_of_cells+1);
                            this_index[j]-=1;
                        }
                        else if(this_index[j]==divisions[j]-1)
                        {
                            this_index[j]-=1;
                            up_indices[i*dim+j] = (number_of_cells+1);
                            down_indices[i*dim+j] = (get_index(this_index));
                            this_index[j]+=1;
                        }
                        else
                        {
                            this_index[j]+=1;
                            up_indices[i*dim+j] = (get_index(this_index));
                            this_index[j]-=2;
                            down_indices[i*dim+j] = (get_index(this_index));
                            this_index[j]+=1;
                        }
                    }
                    //increment
                    this_index[dim-1]++;
                    for(int j=dim-1;j>=0;j--)
                    {
                        if(this_index[j]==divisions[j])
                        {
                            this_index[j] = 0;
                            if(j!=0)
                            {
                                this_index[j-1]++;
                            }
                        }
                        else
                            break;
                    }
                }

                std::vector<unsigned> temp_random;
                double last_time=0;
                double time_counter = 0;
                while(time_counter<simulation_step*num_sim_steps)
                {
                    time_counter+=simulation_step;
                    temp_random.clear();
                    unsigned index_counter = 0;
                    bool update = false;
                    for(unsigned i=0;i<number_of_cells;i++)
                    {
                        double prev_value = level_set[i];
                        //compute delta_phi
                        for(unsigned j=0;j<dim;j++)
                        {
                            if(down_indices[index_counter] > number_of_cells)
                            {
                                if(!collide[up_indices[index_counter]])
                                {
                                    //test only the rightmost derivative
                                    delta_phi[j] = level_set[up_indices[index_counter]] - prev_value;
                                    delta_phi[j]*=2*divide_cell_size[j];
                                }
                                else
                                {
                                    delta_phi[j]=0;
                                }
                            }
                            else if(up_indices[index_counter] > number_of_cells)
                            {
                                if(!collide[down_indices[index_counter]])
                                {
                                    //test only the rightmost derivative
                                    delta_phi[j] = prev_value-level_set[down_indices[index_counter]];
                                    delta_phi[j]*=2*divide_cell_size[j];
                                }
                                else
                                {
                                    delta_phi[j]=0;
                                }
                            }
                            else
                            {
                                if(!collide[up_indices[index_counter]] && !collide[down_indices[index_counter]] )
                                {
                                    delta_phi[j] = (level_set[up_indices[index_counter]]-level_set[down_indices[index_counter]])*divide_cell_size[j];
                                }
                                else
                                {
                                    delta_phi[j]=0;
                                }
                            }
                            index_counter++;
                        }


////*****  System Specific Code *******/////////
                        double vel;
                        double w;
                        double p1p2 = (delta_phi[0]*cs[i]+delta_phi[1]*ss[i]);
                        vel = max_vel;
                        if(delta_phi[2]<0)
                            w =  min_w;
                        else if(delta_phi[2]>0)
                            w =  max_w;
                        else
                            w =  0;
                        double delta = simulation_step*(p1p2*vel+w*delta_phi[2]);
                        level_set[i] -= std::max(delta,0.0);

////*****  End System Specific Code *******/////////
                        if(grid[i] == NULL && prev_value > 0 && level_set[i] < 0)
                        {
                            grid[i] = new cell;
                            grid[i]->node = node;
                            grid[i]->time = cost+time_counter;//cost + last_time - (time_counter - last_time)*prev_value/(level_set[i] - prev_value);//
                            grid[i]->overlap = 1;
                            grid_weight += 1;
                            update = true;
                            max_time = grid[i]->time;
                            for(unsigned j=0;j<dim;j++)
                                point_bounds[j]->expand(xs[i][j]);
                            control_field[i] = control_space->alloc_point();
////*****  System Specific Code *******/////////
                            control_field[i]->at(0) = vel;
                            control_field[i]->at(1) = w;
////*****  End System Specific Code *******/////////
                            node->num_cells++;
                            total_occupied++;
                            temp_random.push_back(i);
                        }
                    }
                    if(update)
                        last_time = max_time;
                    if(temp_random.size()>0)
                        random_indices = temp_random;
                }

                delete up_indices;
                delete down_indices;
                delete cs;
                delete ss;
                delete delta_phi;
                delete collide;
                delete level_set;
            }

            bool space_grid_t::in_bounds(util::space_point_t* point)
            {
                for(unsigned i=0;i<bounds.size();i++)
                {
                    if(!bounds[i]->is_valid(point->memory[i]))
                        return false;
                }
                return true;
            }
            bool space_grid_t::in_bounds(std::vector<double>& point)
            {
                for(unsigned i=0;i<bounds.size();i++)
                {
                    if(!bounds[i]->is_valid(point[i]))
                        return false;
                }
                return true;
            }


            void space_grid_t::get_controls(space_point_t* goal_point, plan_t& plan)
            {
                unsigned index = get_flat_index(goal_point);
                get_controls(index,plan,goal_point);
            }

            void space_grid_t::random_selection(plan_t& plan)
            {
                int val;
                bool flag = false;
                double best_val = -99999;
                unsigned iter=0;
                do
                {
                    val = uniform_int_random(0,random_indices.size()-1);
                    flag = grid[random_indices[val]]->overlap==0;
                    if(flag)
                    {
                        random_indices[val] = random_indices.back();
                        random_indices.pop_back();
                    }
                    else
                    {
                        iter++;
                        best_val = (best_val<grid[random_indices[val]]->time?grid[random_indices[val]]->time:best_val);
                    }

                }
                while(iter<100);
                get_controls(random_indices[val],plan);
            }

            void space_grid_t::get_controls(unsigned index, plan_t& plan, space_point_t* goal_point)
            {
                unsigned original_index = index;
                plan.clear();

                plan.append_onto_front(grid[index]->time - base_node->cost);
                // plan.get_end_state()->at(0) = xs[index][0];
                // plan.get_end_state()->at(1) = xs[index][1];
                // plan.get_end_state()->at(2) = xs[index][2];
                control_t* control = control_field[index];
                ctrl_space->copy_point(plan[0].control,control);
                return;
                unsigned max_steps = ((grid[index]->time - base_node->cost)/simulation_step);
                PRX_ERROR_S(grid[index]->time<<" "<<base_node->cost<<" "<<max_steps);
                std::vector<double> point;
                point.resize(dim);
                std::vector<double> point2;
                point2.resize(dim);
                // point = xs[index];
                // point2 = xs[index];
                unsigned start_index = get_flat_index(base_node->point);
                unsigned iter=0;
                if(goal_point==NULL)
                {
                    for(unsigned i=0;i<dim;i++)
                        point[i] = xs[index][i];
                }

                // double w = 0;
                // if(xs[index][2]-xs[start_index][2] > PRX_ZERO_CHECK)
                //     w = 1;
                // else if(xs[index][2]-xs[start_index][2] < PRX_ZERO_CHECK)
                //     w = -1;
                while(index!=start_index && iter < max_steps)//for(unsigned i = 0; i<max_steps;i++)//
                {
                    // if(i==0 && control_field[index]==NULL)
                    //     PRX_LOG_ERROR("Started in bad cell!");
                    // PRX_ASSERT(index < grid.size());
                    iter++;
                    control_t* control = control_field[index];
                    bool repeat = false;
                    if(control==NULL)
                    {
                        repeat = true;
                        control = plan[0].control;
                    }
                    // control->at(1) = w;
                    plan.copy_onto_front(control,simulation_step);
////*****  System Specific Code *******/////////
                    point2[0] = point[0]-control->at(0)*cos(point[2])*simulation_step;
                    point2[1] = point[1]-control->at(0)*sin(point[2])*simulation_step;
                    point2[2] = point[2]-control->at(1)*simulation_step;
                    if(point2[2]>PRX_PI)
                        point2[2]-=2*PRX_PI;
                    else if(point2[2]<-PRX_PI)
                        point2[2]+=2*PRX_PI;
                    point[0] = point2[0];
                    point[1] = point2[1];
                    point[2] = point2[2];
////*****  End System Specific Code *******/////////
                    unsigned new_index = get_flat_index(point);
                    // PRX_ERROR_S(bounds[0]->get_lower_bound()<<" "<<bounds[1]->get_lower_bound()<<" "<<bounds[2]->get_lower_bound());
                    PRX_WARN_S(point[0]<<" "<<point[1]<<" "<<point[2]);
                    // PRX_ERROR_S(bounds[0]->get_upper_bound()<<" "<<bounds[1]->get_upper_bound()<<" "<<bounds[2]->get_upper_bound());


                    // if(!repeat)
                    //     for(unsigned j=0;j<dim;j++)
                    //         plan[0].control->at(j) = control->at(j);
                    // else
                    //     plan[0].duration+=simulation_step;
                    index = new_index;
                }
                PRX_ERROR_S(""<<plan.length());

                //testing
                if(goal_point==NULL)
                {
                    for(unsigned i=0;i<dim;i++)
                        point[i] = xs[start_index][i];
                }
                PRX_INFO_S("Started: "<<xs[start_index][0]<<" "<<xs[start_index][1]<<" "<<xs[start_index][2]);
                max_steps = ((plan.length())/simulation_step);
                for(unsigned i = 0; i<max_steps;i++)
                {
                    control_t* control = plan.get_control_at(simulation_step*i);
                    point2[0] = point[0]+control->at(0)*cos(point[2])*simulation_step;
                    point2[1] = point[1]+control->at(0)*sin(point[2])*simulation_step;
                    point2[2] = point[2]+control->at(1)*simulation_step;
                    if(point2[2]>PRX_PI)
                        point2[2]-=2*PRX_PI;
                    else if(point2[2]<-PRX_PI)
                        point2[2]+=2*PRX_PI;
                    point = point2;
                    // PRX_WARN_S("Applied: "<<ctrl_space->print_point(control));
                    PRX_INFO_S("Interim: "<<point[0]<<" "<<point[1]<<" "<<point[2]);
                }
                PRX_INFO_S("Reached: "<<point[0]<<" "<<point[1]<<" "<<point[2]);
                PRX_INFO_S("Should : "<<xs[original_index][0]<<" "<<xs[original_index][1]<<" "<<xs[original_index][2]);
                exit(0);

                // plan.append_onto_front(grid[index].time - base_node->cost);
                // plan[0].control->at(0)=175;
                // plan[0].control->at(1)=atan2(   xs[index][1] - base_node->point->at(1),
                //                                 xs[index][0] - base_node->point->at(0));
                return;
            }

            void space_grid_t::intersect(space_grid_t* other_grid)
            {
                PRX_ASSERT(this!=other_grid);
                //get the intersected region
                bounds::intersect(intersection,bounds,other_grid->bounds);
                for(unsigned i=0;i<dim;i++)
                {
                    if(intersection[i]->get_upper_bound() < intersection[i]->get_lower_bound())
                        return;
                }

                //temp_division (number of cells)
                std::vector<unsigned> temp_division;
                std::vector<unsigned> temp_multiplier;
                std::vector<unsigned> this_start;
                std::vector<unsigned> other_start;
                //multipliers for indexing
                temp_multiplier.resize(dim);
                std::vector<unsigned> this_index;
                std::vector<unsigned> other_index;
                std::vector<unsigned> this_min;
                std::vector<unsigned> this_max;
                std::vector<unsigned> other_max;
                std::vector<unsigned> other_min;


                this_index.resize(dim);
                other_index.resize(dim);
                this_min.resize(dim);
                this_max.resize(dim);
                other_max.resize(dim);
                other_min.resize(dim);

                //index bounds for this grid
                for(unsigned i=0;i<dim;i++)
                {
                    this_min[i] = (unsigned) (intersection[i]->get_lower_bound()-min_val[i])/cell_size[i];
                    this_max[i] = (unsigned) (intersection[i]->get_upper_bound()-min_val[i])/cell_size[i];
                    other_min[i] = (unsigned) (intersection[i]->get_lower_bound()-other_grid->min_val[i])/cell_size[i];
                    other_max[i] = (unsigned) (intersection[i]->get_upper_bound()-other_grid->min_val[i])/cell_size[i];
                }

                unsigned total = 1;
                unsigned other_total = 1;
                for(unsigned i=0;i<dim;i++)
                {
                    total *= this_max[i]-this_min[i];
                    other_total *= other_max[i]-other_min[i];
                }
                this_index = this_min;
                other_index = other_min;
                total = std::min(total,other_total);
//                if(total!=other_total)
//                    PRX_LOG_ERROR("DFSDFSDFSDFSDFSDFSDFDS %d %d",total,other_total);
                for(unsigned i=0;i<total;i++)
                {
                    //Comparing Cells
                    unsigned this_flat = get_index(this_index);
                    unsigned other_flat = other_grid->get_index(other_index);
                    cell* this_cell = grid[this_flat];
                    cell* other_cell = other_grid->grid[other_flat];
                    //compare the cells
                    if(this_cell != NULL && other_cell !=NULL )
                    {
                        if(this_cell->overlap>0 && other_cell->overlap>0)
                        {
                            if(this_cell->time < other_cell->time)
                            {
                                other_cell->node->num_cells--;
                                other_grid->total_occupied--;
                                // if(other_cell.overlap!=0)
                                other_grid->grid_weight -= POW(GAMMA,other_cell->overlap-1);
                                grid_weight -= POW(GAMMA,this_cell->overlap-1);
                                int temp_val = other_cell->overlap;
                                this_cell->overlap = temp_val+1;
                                other_cell->overlap = 0;
                                grid_weight += POW(GAMMA,this_cell->overlap-1);
                                delete other_grid->grid[other_flat];
                                other_grid->grid[other_flat] = NULL;
                                // other_cell->node = NULL;
                                // other_cell.time = PRX_INFINITY;
                            }
                            else
                            {
                                // PRX_WARN_S("1. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                this_cell->node->num_cells--;
                                // PRX_WARN_S("2. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                total_occupied--;
                                // PRX_WARN_S("3. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                grid_weight -= POW(GAMMA,this_cell->overlap-1);
                                // PRX_WARN_S("4. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                other_grid->grid_weight -= POW(GAMMA,other_cell->overlap-1);
                                // PRX_WARN_S("5. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                int temp_val = this_cell->overlap;
                                // PRX_WARN_S("6. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                other_cell->overlap = temp_val+1;
                                // PRX_WARN_S("7. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                this_cell->overlap = 0;
                                // PRX_WARN_S("8. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                other_grid->grid_weight += POW(GAMMA,other_cell->overlap-1);
                                // PRX_WARN_S("9. "<<this_cell.overlap<<" "<<other_cell.overlap);
                                // this_cell->node = NULL;
                                delete grid[this_flat];
                                grid[this_flat] = NULL;
                                // this_cell.time = PRX_INFINITY;
                            }
                        }
                    }

                    this_index[dim-1]++;
                    other_index[dim-1]++;
                    for(int j=dim-1;j>=0;j--)
                    {
                        if(this_index[j]==this_max[j])
                        {
                            this_index[j] = this_min[j];
                            other_index[j] = other_min[j];
                            if(j!=0)
                            {
                                this_index[j-1]++;
                                other_index[j-1]++;
                            }
                        }
                        else
                            break;
                    }
                }
            }
        }
    }
}
