/**
 * @file pqp_collision_checker.hpp
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

#ifndef PRX_PQP_COLLISION_CHECKER_HPP
#define	PRX_PQP_COLLISION_CHECKER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/simulation/collision_checking/collision_checker.hpp"

#ifdef __APPLE__
#include <PQP.h>
#else
#include <PQP.h>
#endif

namespace prx
{
    namespace sim
    {

        /**
         * A struct to maintain information for the pqp model. Can update the translation and
         * rotation matrices for the correct position and orientation of the model.
         *
         * @brief <b> A struct to maintain information for the pqp model. </b>
         *
         * @author Athanasios Krontiris, Kostas Bekris
         */
        typedef struct pqp_info_t
        {

            /**
             * The PQP model.
             * @brief The PQP model.
             */
            PQP_Model* model;
            /**
             * The pointer for the \ref system_t.
             * @brief The pointer for the \ref system_t.
             */
            system_t* system;
            /**
             * The name of the system.
             * @brief The name of the system.
             */
            std::string name;

            /**
             * The vector that keeps the translation of the model on the axis.
             * @brief The vector that keeps the translation of the model on the axis.
             */
            double translation_matrix[3];

            /**
             * The matrix for the rotation information so as to rotate the model on the axis.
             * @brief The matrix for the rotation information.
             */
            double rotation_matrix[3][3];


            /**
             * As a basic broadphase algorithm, find that sphere that encloses the model,
             * then perform basic distance checks to determine if narrow phase collision
             * checking is needed.
             */
            double broadphase_radius;

            pqp_info_t(system_t * p) : pos(3)
            {
                system = p;
                model = new PQP_Model();
                broadphase_radius = PRX_INFINITY;
                pos.resize(3);
            }

            ~pqp_info_t()
            {
                delete model;
            }

            /**
             * Sets the new position to the translation vector.
             *
             * @brief Sets the new position to the translation matrix.
             *
             * @param pos The new position of the system.
             */
            void update_translation_matrix(const util::vector_t & pos)
            {
                update_translation_matrix(pos[0], pos[1], pos[2]);
            }

            /**
             * Sets the new position to the translation vector, given the coordinates
             * separately and not as one vector.
             *
             * @brief Sets the new position to the translation vector.
             *
             * @param x The x coordinate.
             * @param y The y coordinate.
             * @param z The z coordinate.
             */
            void update_translation_matrix(double x, double y, double z)
            {
                translation_matrix[0] = x;
                translation_matrix[1] = y;
                translation_matrix[2] = z;
            }

            /**
             * Updates the rotation matrix based on the given quaternion. The quaternion \c quat
             * represents the current orientation of the system.
             *
             * @brief Updates the rotation matrix based on the given quaternion.
             *
             * @param quat The current orientation of the system.
             */
            void update_rotation_matrix(const util::quaternion_t & quat)
            {
                update_rotation_matrix(quat.get_x(), quat.get_y(), quat.get_z(), quat.get_w());
            }

            /**
             * Updates the rotation matrix based on the given quaternion. Instead of
             * one \ref util::quaternion_t this functions gets as arguments the different
             * values of the quaternion.
             *
             * @brief Updates the rotation matrix based on the given quaternion.
             *
             * @param qx The x coordinate of the quaternion.
             * @param qy The y coordinate of the quaternion.
             * @param qz The z coordinate of the quaternion.
             * @param qw The w coordinate of the quaternion.
             */
            void update_rotation_matrix(double qx, double qy, double qz, double qw)
            {
                rotation_matrix[0][0] = 1 - 2 * qy * qy - 2 * qz*qz;
                rotation_matrix[0][1] = 2 * qx * qy - 2 * qz*qw;
                rotation_matrix[0][2] = 2 * qx * qz + 2 * qy*qw;
                rotation_matrix[1][0] = 2 * qx * qy + 2 * qz*qw;
                rotation_matrix[1][1] = 1 - 2 * qx * qx - 2 * qz*qz;
                rotation_matrix[1][2] = 2 * qy * qz - 2 * qx*qw;
                rotation_matrix[2][0] = 2 * qx * qz - 2 * qy*qw;
                rotation_matrix[2][1] = 2 * qy * qz + 2 * qx*qw;
                rotation_matrix[2][2] = 1 - 2 * qx * qx - 2 * qy*qy;
            }

            /**
             * Updates both translation vector and rotation matrix based on the given
             * configuration of the system.
             *
             * @brief Updates both translation vector and rotation matrix.
             *
             * @param config The current configuration of the system.
             */
            void update_matrices(const util::config_t & config)
            {
                config.get(pos, q);
                update_translation_matrix(pos);
                update_rotation_matrix(q);
            }

            /**
             * Flips the rotation matrix.
             *
             * @brief Flips the rotation matrix.
             */
            void flip_rotation_matrix()
            {
                rotation_matrix[0][2] = -rotation_matrix[0][2];
                rotation_matrix[2][0] = -rotation_matrix[2][0];
            }

            /**
             * A helper function to print the information from the translation and rotation
             * matrices.
             *
             * @brief Prints the information from the translation and rotation matrices.
             *
             * @return A string with all the information ready to be printed.
             */
            std::string print() const
            {
                std::stringstream out(std::stringstream::out);

                out << "position: x:" << translation_matrix[0] << "  y:" << translation_matrix[1] << "  z:" << translation_matrix[2] << std::endl;
                out << "Rotation: [" << std::endl;
                out << rotation_matrix[0][0] << " , " << rotation_matrix[0][1] << " , " << rotation_matrix[0][2] << std::endl;
                out << rotation_matrix[1][0] << " , " << rotation_matrix[1][1] << " , " << rotation_matrix[1][2] << std::endl;
                out << rotation_matrix[2][0] << " , " << rotation_matrix[2][1] << " , " << rotation_matrix[2][2] << std::endl;
                out << "]" << std::endl;

                return out.str();
            }

          private:
            /**
             * The position of the model.
             * @brief The position of the model.
             */
            util::vector_t pos;
            /**
             * The quaternion of the model.
             * @brief The quaternion of the model.
             */
            util::quaternion_t q;

        } translation_matrix_t;

        /**
         * An implementation of a collision checker. This version will use the pqp library in order to check
         * if two systems are in collision or not.
         *
         * @brief <b> Uses the PQP library to check for collisions. </b>
         *
         * @author Athanasios Krontiris, Kostas Bekris
         */
        class pqp_collision_checker_t : public collision_checker_t
        {

          public:

            friend std::ostream& operator<<(std::ostream& output, const pqp_collision_checker_t& checker);

            pqp_collision_checker_t();
            virtual ~pqp_collision_checker_t();

            /**
             * Sets the new configuration of the system.
             *
             * @brief Sets the new configuration of the system.
             *
             * @param name The full slash-delimited path for the system to set the new configuration.
             * @param config The new configuration for the system.
             */
            void set_configuration(const std::string& name, const util::config_t& config);

            /**
             * Adds a body to a system to be checked for collisions.
             *
             * @brief Adds a body to a system to be checked for collisions.
             *
             * @param name The name of the rigid body to be added.
             * @param geometry The geometry of the rigid body to be added.
             * @param plant The pointer of the plant that the body belongs.
             * @param is_obstacle If the body belongs to an obstacle or not.
             */
            void add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle = false);

            /**
             * Removes a rigid body from the collision checker such as it will not spend time to
             * check for collisions for this body.
             *
             * @brief Removes a rigid body from the collision checker.
             *
             * @param name The name of the rigid body that needs to be removed from the collision list.
             */
            void remove_body(const std::string& name);

            /**
             * Sets the white list of pairs of systems to check for collisions.
             *
             * @brief Sets the white list of pairs of systems to check for collisions.
             *
             * @list The new white list of pairs that need to be checked for collisions.
             */
            void link_collision_list(collision_list_t* list);

            /**
             * Checks if there is at least one collision between the systems.
             *
             * @brief Checks if there is at least one collision between the systems.
             *
             * @return True if there is at least one collision. \n
             *         False if there is no collision during this simulation step.
             */
            bool in_collision();

            /**
             * Checks if there is at least one collision between the systems in
             * the provided collision list.
             *
             * @brief Checks for collisions in the provided collision list.
             *
             * @return True if there is a collision in the list. \n
             *         False if there is no such collision in the list.
             */
            virtual bool in_collision( collision_list_t* list );

            /**
             * Checks to make sure all of the systems are close to the obstacles of the
             * scene.
             *
             * @brief Checks if there is any near-collision.
             *
             * @param eps The clearance to use for near checking.
             * @return True if it finds a near-collision. \n
             *         False if there is no near-collision between the systems.
             */
            virtual bool near_collision( double eps );


            /**
             * Get the distance between two systems.
             *
             * @brief Get the distance between two systems.
             *
             * @return The distance
             */
            virtual double get_clearance();

            /**
             * Checks for collisions between the systems. Will return a list with all the
             * collided bodies during this simulation step.
             *
             * @brief Will return a list with all the collided bodies.
             *
             * @return The list with the pairs for all the systems that they are in collision.
             */
            collision_list_t* colliding_bodies();

            /**
             * Returns a list with all the pairs of bodes which are also in the
             * provided input list.
             */
            collision_list_t* colliding_bodies( collision_list_t* list );

            /**
             * Checks for near-collisions between the systems, returning a list of
             * all pairs of bodies which are within eps distance of each other.
             *
             * @brief Get a list of nearly-colliding bodies.
             *
             * @param eps The clearance to do near-checking with.
             * @return A list of nearly-colliding bodies.
             */
            virtual collision_list_t* near_colliding_bodies( double eps );

          protected:

            /**
             * A map between the name of the systems and the pqp models.
             * @brief A map between the name of the systems and the pqp models.
             */
            std::map<std::string, pqp_info_t*> models_map;

            /**
             * The iterator for the \c models_map.
             * @brief The iterator for the \c models_map.
             */
            std::map<std::string, pqp_info_t*>::iterator models_map_iter;

            /**
             * The list of the systems that will be checked for collisions.
             * @brief The list of the systems that will be checked for collisions.
             */
            std::vector<std::pair<pqp_info_t*, pqp_info_t*> > body_cache;


          private:

            /**
             * Using information from the given parameters, creates a pqp model that is
             * mandatory for the pqp library to checkes for collisions.
             *
             * @brief Creates a pqp model given some information.
             *
             * @param name The name of the system.
             * @param geometry The geometry of the system.
             */
            void create_model(const std::string& name, const util::geometry_t& geometry);

            /**
             * Checks if the two systems are in collision or not.
             *
             * @brief Checks if the two systems are in collision or not.
             *
             * @param name1 The pqp information for the first system.
             * @param name2 The pqp information for the second system.
             *
             * @return True if the systems are in collision. \n
             *         False if they are not in collision.
             */
            bool in_collision(pqp_info_t* name1, pqp_info_t* name2);

            /**
             * Checks if the two systems are in collision or not.
             *
             * @brief Checks if the two systems are in collision or not.
             *
             * @param name1 The full slash-delimited path for the first system.
             * @param name2 The full slash-delimited path for the second system.
             *
             * @return True if the systems are in collision. \n
             *         False if they are not in collision.
             */
            bool in_collision(const std::string& name1, const std::string& name2);

            /**
             * Internal near_collision check call.
             *
             * @brief Checks if the two systems are near-collision or not.
             *
             * @param name1 The pqp information for the first system.
             * @param name2 The pqp information for the second system.
             *
             * @return True if the systems are near-collision. \n
             *         False if they are not near-collision.
             */
            bool near_collision( double eps, pqp_info_t* info1, pqp_info_t* info2 );

            /**
             * Internal distance query.
             *
             * @brief Checks the distance between two systems.
             *
             * @param min_so_far The minimum distance found so far. Used for optimization.
             * @param info1 The pqp information for the first system.
             * @param info2 The pqp information for the second system.
             *
             * @return The distance.
             */
            double get_clearance(double min_so_far, pqp_info_t* info1, pqp_info_t* info2 );
        };

        std::ostream& operator<<(std::ostream& output, const pqp_collision_checker_t& checker);


    }
}

#endif

