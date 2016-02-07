/**
 * @file tree.cpp 
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

#include "prx/utilities/graph/tree.hpp"
#include "prx/utilities/spaces/space.hpp"
#include "tree.hpp"

namespace prx 
 { 
 namespace util 
 {

tree_node_t::tree_node_t() : abstract_node_t()
{
            parent = -1;
}

tree_node_t::~tree_node_t()
{
}


tree_vertex_index_t tree_node_t::get_parent() const
{
    return parent;
}

tree_vertex_index_t tree_node_t::get_index() const
{
    return index;
}

const std::list<tree_vertex_index_t>& tree_node_t::get_children() const
{
    return children;
}


tree_edge_t::tree_edge_t() : abstract_edge_t()
{
}

tree_edge_t::~tree_edge_t()
{
}

tree_edge_index_t tree_edge_t::get_index() const
{
    return index;
}

tree_vertex_index_t tree_edge_t::get_source() const
{
    return source;
}

tree_vertex_index_t tree_edge_t::get_target() const
{
    return target;
}


        
tree_t::tree_t()
{
    vertex_count = 0;
    edge_count = 0;
    max_count = 0;
    vertex_id_counter = 0;
    edge_id_counter = 0;
}

tree_t::~tree_t()
{
    foreach(tree_node_t* node, vertex_list)
    {
        delete node;
    }
    foreach(tree_edge_t* node, edge_list)
    {
        delete node;
    }
}

void tree_t::remove_vertex(tree_vertex_index_t v)
{
    //since we are maintaining a tree, can't remove this vertex unless it is a leaf (i.e. no children)
    PRX_ASSERT(v_index_map[v]->children.size()==0);
    
    v_index_map[v_index_map[v]->parent]->children.remove(v);
    tree_edge_index_t e = v_index_map[v]->parent_edge;
    
    //swap the elements with end of the list
    tree_node_t* temp_v = v_index_map[v];
    temp_v->prox_node = NULL;
    temp_v->index = 9999999;
    v_index_map[v] = NULL;
    vertex_iterator v_iterator = std::find(vertex_list.begin(),v_iter,temp_v);
    v_iter--;
    const_v_iter--;
    *v_iterator = *v_iter;
    *v_iter = temp_v;
    vertex_count--;
    
    tree_edge_t* temp_e = e_index_map[e];
    temp_e->index = 99999999;
    edge_iterator e_iterator = std::find(edge_list.begin(),e_iter,temp_e);
    e_index_map[e] = NULL;
    e_iter--;
    const_e_iter--;
    *e_iterator = *e_iter;
    *e_iter = temp_e;
    edge_count--;    
}

tree_node_t* tree_t::operator[](tree_vertex_index_t v) const
{
    return v_index_map[v];
}

void tree_t::clear()
{
    PRX_DEBUG_S("Clearing the tree...");
    
    foreach(tree_edge_t* e, edge_list)
    {
        e->source = e->target = 0;
    }
    foreach(tree_node_t* v, vertex_list)
    {
//        space->free_point(v->point);
        v->children.clear();
        v->parent = v->index;
    }
//    vertex_list.clear();
//    edge_list.clear();
    vertex_count = 0;
    edge_count = 0;
}

unsigned tree_t::num_vertices() const
{
    return vertex_count;
}
unsigned tree_t::num_edges() const
{
    return edge_count;
}

bool tree_t::is_leaf(tree_vertex_index_t v)
{
    return (v_index_map[v]->get_children().size() == 0);
}

} 
 }