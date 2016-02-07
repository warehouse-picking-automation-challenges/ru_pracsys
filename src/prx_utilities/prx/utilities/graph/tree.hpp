/**
 * @file tree.hpp 
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

#ifndef PRX_TREE_HPP
#define	PRX_TREE_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/graph/abstract_node.hpp"
#include "prx/utilities/graph/abstract_edge.hpp"
#include "prx/utilities/boost/hash.hpp"

namespace prx 
 { 
 namespace util 
 {

class tree_node_t;
class tree_edge_t;
class tree_t;
class space_t;

typedef unsigned int tree_vertex_index_t;
typedef unsigned int tree_edge_index_t;


/**
 * An implementation of a node for trees.
 * 
 * @brief <b> An implementation of a node for trees.</b>
 * 
 * @author Zakary Littlefield
 */
class tree_node_t : public abstract_node_t
{
    public:
        tree_node_t();
        ~tree_node_t();
        /**
         * @brief Gets the parent vertex of this vertex.
         * @return Parent vertex
         */
        tree_vertex_index_t get_parent() const;
        /**
         * @brief Gets this node's index.
         * @return The node index.
         */
        tree_vertex_index_t get_index() const;
        /**
         * @brief Gets the children vertices.
         * @return A list of children vertex indices.
         */
        const std::list<tree_vertex_index_t>& get_children() const;
    protected:
        /**
         * @brief Parent vertex index.
         */
        tree_vertex_index_t parent;

        /**
        * @brief Parent edge
        */
        tree_edge_index_t parent_edge;
        
        /**
         * @brief Index of this node.
         */
        tree_vertex_index_t index;
        
        /**
         * @brief Children vertices
         */
        std::list<tree_vertex_index_t> children;
        
    friend class tree_t;
};

/**
 * An implementation of an edge for trees.
 * 
 * @brief <b> An implementation of an edge for trees.</ b>
 * 
 * @author Zakary Littlefield
 */
class tree_edge_t : public abstract_edge_t
{
    public:
        tree_edge_t();
        ~tree_edge_t();
        /**
         * @brief Gets the index of this edge.
         * @return The edge index.
         */
        tree_edge_index_t get_index() const;
        /**
         * @brief Gets the source vertex of this edge. 
         * @return The source vertex.
         */
        tree_vertex_index_t get_source() const;
        /**
         * @brief Gets the target vertex of this edge. 
         * @return The target vertex.
         */
        tree_vertex_index_t get_target() const;
    protected:
        /**
         * @brief The source vertex of this edge.
         */
        tree_vertex_index_t source;
        /**
         * @brief The target vertex of this edge.
         */
        tree_vertex_index_t target;
        /**
         * @brief This edge's index.
         */
        tree_edge_index_t index;
        
    friend class tree_t;
};

/**
 * A simple implementation of a tree data structure. Allows for preallocated memory. Provides a fast 
 * way of representing trees for motion planning, sacrificing some of the functionality of boost graphs.
 * @brief  <b> A simple implementation of a tree data structure.</b>
 */      
class tree_t 
{
    public:
        
        typedef std::list<tree_node_t*>::const_iterator const_vertex_iterator;
        typedef std::list<tree_edge_t*>::const_iterator const_edge_iterator;
        typedef std::list<tree_node_t*>::iterator vertex_iterator;
        typedef std::list<tree_edge_t*>::iterator edge_iterator;
        
        tree_t();
        ~tree_t();
        
        /**
         * Preallocates memory for the tree. Since the relationship between nodes 
         * and edges is known, these data members can be allocated and used later.
         * 
         * @brief Preallocates memory for the tree.
         * @param new_size The amount of vertices to hold in the tree.
         */
        template<class node_type,class edge_type>
        void pre_alloc_memory(unsigned new_size)
        {
            unsigned old_size = max_count;
            if(old_size < new_size)
            {
                v_index_map.resize(new_size);
                e_index_map.resize(new_size);
                    PRX_INFO_S("inserting");
                for(unsigned i=old_size;i<new_size;i++)
                {
                    vertex_list.insert(vertex_list.end(),new node_type());
                    edge_list.insert(edge_list.end(),new edge_type());
                    v_iter = vertex_list.begin();
                    const_v_iter = vertex_list.begin();
                    std::advance(v_iter,old_size);
                    std::advance(const_v_iter,old_size);
                    e_iter = edge_list.begin();
                    const_e_iter = edge_list.begin();
                    std::advance(e_iter,old_size);
                    std::advance(const_e_iter,old_size);
                }
            }
            if(old_size==0)
            {
                v_iter = vertex_list.begin();
                e_iter = edge_list.begin();
                const_v_iter = vertex_list.begin();
                const_e_iter = edge_list.begin();
            }
            max_count = new_size;
        }
        
        /**
         * Adds a vertex onto the tree. Can be done without an edge. Assumes an edge will be made soon after.
         * @brief Adds a vertex onto the tree.
         * @return The vertex index of the new vertex.
         */
        template<class node_type,class edge_type>
        tree_vertex_index_t add_vertex()
        {
            if(vertex_count == max_count)
            {
                pre_alloc_memory<node_type,edge_type>(vertex_list.size()*2+1);
            }
            node_type* node = (node_type*)(*v_iter);
            node->index = vertex_id_counter;
            node->parent = vertex_id_counter;
            v_index_map[vertex_id_counter] = node;
            vertex_id_counter++;
            v_iter++;
            const_v_iter++;
            vertex_count++;
            return node->index;
        }
        
        /**
         * Adds a new edge onto the tree. Does not check for loops.
         * @brief Adds a new edge onto the tree.
         * @param from Vertex to start the edge from.
         * @param to Vertex to end the edge on.
         * @return The edge index of the new edge.
         */
        template<class edge_type>
        tree_edge_index_t add_edge(tree_vertex_index_t from, tree_vertex_index_t to)
        {

            PRX_ASSERT(v_index_map[to]->parent==to);

            if(edge_count == max_count)
            {
                PRX_FATAL_S("There are more edges than vertices in the tree. Never should happen.");
            }
            
            v_index_map[from]->children.insert(v_index_map[from]->children.begin(),to);
            v_index_map[to]->parent = from;
            
            edge_type* edge = (edge_type*)(*e_iter);
            edge->index = edge_id_counter;
            edge->source = from;
            edge->target = to;
            e_index_map[edge_id_counter] = edge;
            edge_count++;
            edge_id_counter++;
            e_iter++;
            const_e_iter++;
            v_index_map[to]->parent_edge = edge->index;
            // edge_map[v_index_map[from]][v_index_map[to]] = edge->index;
            
            return edge->index;
        }
        
        /**
         * Remove a vertex from the tree. Can only remove if the vertex is a leaf. 
         * @brief Removes a leaf vertex from the tree.
         * @param v The vertex to remove.
         */
        void remove_vertex(tree_vertex_index_t v);
        
        /**
         * Gets a vertex from the tree
         * @param v The index of the vertex to retrieve.
         * @return The vertex.
         */
        tree_node_t* operator[](tree_vertex_index_t v) const;
        
        /**
         * Gets an edge from the tree.
         * @param v1 The start vertex index.
         * @param v2 The end vertex index.
         * @return The edge between \c v1 and \c v2.
         */
        tree_edge_index_t edge(tree_vertex_index_t v1,tree_vertex_index_t v2) const
        {
            return v_index_map[v2]->parent_edge;
            // return edge_map[v_index_map[v1]][v_index_map[v2]];
        }
        
        /**
         * Gets a vertex from the tree as a certain type
         * @param v The index of the vertex to retrieve.
         * @return The vertex.
         */
        template<class node_type>
        node_type* get_vertex_as(tree_vertex_index_t i) const
        {
            return (node_type*)(v_index_map[i]);
        }
        
        /**
         * Gets an edge from the tree as a certain type
         * @param i The index of the edge to retrieve.
         * @return The edge.
         */
        template<class edge_type>
        edge_type* get_edge_as(tree_edge_index_t i) const
        {
            return (edge_type*)(e_index_map[i]);
        }
        
        /**
         * Gets an edge from the tree as a certain type.
         * @param i The start vertex index of the edge.
         * @param j The end vertex index of the edge.
         * @return The edge.
         */
        template<class edge_type>
        edge_type* get_edge_as(tree_vertex_index_t i, tree_vertex_index_t j) const
        {
            //return (edge_type*)(e_index_map[edge_map[v_index_map[i]][v_index_map[j]]]);
            return (edge_type*)(e_index_map[v_index_map[j]->parent_edge]);
        }
        
        /**
         * Marks the tree as clear. Doesn't free any memory.
         * @brief Marks the tree as clear.
         */
        void clear();
                
        /**
         * A helper function for iteration over the vertices of this tree.
         * @brief A helper function for iteration over the vertices of this tree.
         * @return The pair containing the start and end iterators for vertices
         */
        std::pair<const_vertex_iterator,const_vertex_iterator> vertices() const
        {   
            return std::pair<const_vertex_iterator,const_vertex_iterator>(vertex_list.begin(),const_v_iter);
        }
        
        
        /**
         * A helper function for iteration over the edges of this tree.
         * @brief A helper function for iteration over the edges of this tree.
         * @return The pair containing the start and end iterators for edges
         */
        std::pair<const_edge_iterator,const_edge_iterator> edges() const
        {            
            return std::pair<const_edge_iterator,const_edge_iterator>(edge_list.begin(),const_e_iter);
        }
        
        /**
         * Gets the number of vertices in the tree.
         * @brief Gets the number of vertices in the tree.
         * @return The number of vertices.
         */
        unsigned num_vertices() const;
        
        /**
         * Gets the number of edges in the tree.
         * @brief Gets the number of edges in the tree.
         * @return The number of edges.
         */
        unsigned num_edges() const;
        
        /**
         * Tells if a vertex is a leaf or not.
         * @brief Tells if a vertex is a leaf or not.
         * @param v The vertex to test.
         * @return Whether the vertex is a leaf or not.
         */
        bool is_leaf(tree_vertex_index_t v);
        
        /**
         * Moves a subtree under another parent vertex.
         * @brief Moves a subtree to another part of the tree.
         * @param root The root of the subtree to move.
         * @param new_parent The new parent of \c root.
         */
        void transplant(tree_vertex_index_t root, tree_vertex_index_t new_parent)
        {         
            //move root to be a subtree of new_parent
            tree_vertex_index_t old_from;
            old_from = v_index_map[root]->parent;
            // tree_edge_t* edge = e_index_map[(edge_map[v_index_map[old_from]][v_index_map[root]])];
            tree_edge_t* edge = e_index_map[v_index_map[root]->parent_edge];
            v_index_map[old_from]->children.remove(root);
            
            v_index_map[root]->parent = new_parent;         
            v_index_map[new_parent]->children.insert(v_index_map[new_parent]->children.end(),root);
            edge->source = new_parent;
            
            // edge_map[v_index_map[new_parent]][v_index_map[root]] = edge->index;
            v_index_map[root]->parent_edge = edge->index;
        }
                
        
    protected:
        
        /**
         * @brief A list of vertex containers.
         */
        std::list<tree_node_t*> vertex_list;
        /**
         * @brief A list of edge containers.
         */
        std::list<tree_edge_t*> edge_list;
        /**
         * @brief The marker for the end of the vertex list.
         */
        vertex_iterator v_iter;
        /**
         * @brief The marker for the end of the edge list.
         */
        edge_iterator e_iter;
        /**
         * @brief A const version of \c v_iter
         */
        const_vertex_iterator const_v_iter;
        /**
         * @brief A const version of \c e_iter
         */
        const_edge_iterator const_e_iter;
        /**
         * @brief The number of vertices in the tree.
         */
        unsigned vertex_count;
        /**
         * @brief The number of edges in the tree.
         */
        unsigned edge_count;
        /**
         * @brief The maximum number of vertices that can be stored.
         */
        unsigned max_count;
        /**
         * @brief Current counter of vertex indices. Will update when adding nodes.
         */
        unsigned vertex_id_counter;
        /**
         * @brief Current counter of edge indices. Will update when adding edges.
         */
        unsigned edge_id_counter;
        /**
         * @brief Map from vertex indices to actual vertices.
         */
        std::vector<tree_node_t*> v_index_map;
        /**
         * @brief Map from edge indices to actual edges.
         */
        std::vector<tree_edge_t*> e_index_map;
        /**
         * @brief A map of maps that represent the connection between two vertices.
         */
        // hash_t<tree_node_t*, hash_t<tree_node_t*, tree_edge_index_t> > edge_map;
};
        
} 
 }
        
#endif