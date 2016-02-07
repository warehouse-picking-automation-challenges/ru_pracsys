/**
 * @file collision_checker.hpp
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

#ifndef PRX_COLLISION_CHECKER_HPP
#define	PRX_COLLISION_CHECKER_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/boost/hash.hpp"
#include "prx/utilities/math/3d_geometry/geometry.hpp"
#include "prx/simulation/collision_checking/collision_list.hpp"
#include "prx/simulation/collision_checking/vector_collision_list.hpp"
#include "prx/simulation/systems/plants/plant.hpp"
#include <pluginlib/class_loader.h>

namespace prx
{
    namespace sim
    {

        /**
         * Collision checker is the module that will check for collisions, during the simulation.
         * The simulator will have a collision checker
         *
         * @brief <b> Checks for collisions, during the simulation </b>
         *
         * @author Athanasios Krontiris
         */
        class collision_checker_t
        {

          public:

            collision_checker_t();

            virtual ~collision_checker_t(){ }

            /**
             * Initializes the collision_checker from the given parameters.
             *
             * @brief Initializes from the given parameters.
             *
             * @param reader A \ref util::parameter_reader_t with a dictionary of parameters for this \ref collision_checker_t.
             */
            virtual void init(const util::parameter_reader_t * const reader)
            {
                PRX_INFO_S("Initializes collision checker ...");
            }

            /**
             * Sets the collision list (white list) of pairs of systems that we want to check for collisions.
             *
             * @brief Sets the collision list.
             *
             * @param list Is the list that contains
             */
            virtual void link_collision_list(collision_list_t* list);

            /**
             * Sets the configurations of the systems that we want to check for collisions.
             * In the \c map the configurations are mapped by the full pathname of the system (rigid body).
             *
             * @brief Sets the new configurations of the systems.
             *
             * @param map Is the map between the bodies names and the configurations.
             */
            virtual void set_configurations(const util::config_list_t& map);

            /**
             * Sets the configuration of the system, located at the \c name, that we want to
             * check for collisions.
             *
             * @brief Sets the new configuration for a system.
             *
             * @param name The full slash-delimited path name for the system that we want to
             * update the configuration.
             * @param config The new configuration of that system.
             */
            virtual void set_configuration(const std::string& name, const util::config_t& config) = 0;

            /**
             * Adds the geometries of the bodies in the collision checker.
             * In the map the geometries are mapped by the full path of the rigid bodies of the system.
             * In the bodies the systems have the entire path to the plant.
             *
             * @brief Adds the geometries of the bodies in the collision checker.
             *
             * @param map Is the map between the bodies names and the geometries of the body.
             * @param bodies Is the hash map between plants and names of the plants, so as to know
             * which plant the body belongs.
             */
            virtual void add_bodies(const util::geom_map_t& map, util::hash_t<std::string, plant_t*>& bodies);

            /**
             * Adds a new rigid body in the collision checker.
             *
             * @brief Adds a new rigid body in the collision checker.
             *
             * @param name The name of the new rigid body that we want to add.
             * @param geometry The geometry for the new rigid body.
             * @param plant The plant that the rigid body belongs.
             * @param is_obstacle True if it is obstacle \n False otherwise.
             */
            virtual void add_body(const std::string& name, const util::geometry_t& geometry, system_t* plant, bool is_obstacle = false) = 0;

            /**
             * Removes the rigid body from the collision checker.
             *
             * @brief Removes the rigid body from the collision checker.
             *
             * @param name The name of the rigid body to remove.
             */
            virtual void remove_body(const std::string& name) = 0;

            /**
             * Checks if there is at least one collision between the systems in the
             * white list.
             *
             * @brief Checks if there is any collision.
             *
             * @return True if it founds a collision. \n
             *         False if there is no collision between the systems,
             */
            virtual bool in_collision() = 0;

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
             * Sets the configurations of the systems to check for collisions.
             * Sets the white list of pairs of systems to check for collisions.
             * Checks if there is at least one collision between the systems in the
             * white list.
             *
             * @brief Checks for any collision in the given list.
             *
             * @param map The new configurations for the systems to be checked for collisions.
             * @param list The white list of the systems to check for collisions.
             * @return True if it founds a collision. \n
             *         False if there is no collision between the systems in the white list
             */
            bool in_collision(const util::config_list_t& map, collision_list_t* list);

            /**
             * Sets the updated configurations of the systems to check for collisions.
             * Checks if there is at least one collision between the systems in the
             * white list.
             *
             * @brief Checks for any collision after sets the new configurations of the systems.
             *
             * @param map The new configurations for the systems to be checked for collisions.
             *
             * @return True if it founds a collision. \n
             *         False if there is no collision between the systems in the white list
             */
            bool in_collision(const util::config_list_t& map);

            /**
             * Get the distance between two systems.
             *
             * @brief Get the distance between two systems.
             *
             * @return The distance
             */
            virtual double get_clearance();

            /**
             * Sets the configurations of the systems to check for distances.
             * Sets the white list of pairs of systems to check for distances.
             * Gets the minimum distance among them.
             *
             * @brief Gets the minimum distance.
             *
             * @param map The new configurations for the systems.
             * @param list The white list of the systems.
             * @return The minimum distance.
             */
            double get_clearance(const util::config_list_t& map, collision_list_t* list);

            /**
             * Sets the updated configurations of the systems to check for collisions.
             * Gets the minimum distance among them.
             *
             * @brief Gets the minimum distance.
             * @param map The new configurations for the systems.
             *
             * @return The minimum distance.
             */
            double get_clearance(const util::config_list_t& map);

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
             * Sets the configurations of the systems to check for near-collisions.
             * Sets the white list of pairs of systems to check for near-collisions.
             * Checks to make sure all of the systems are close to the obstacles of the
             * scene.
             *
             * @brief Checks for any near-collision in the given list.
             *
             * @param eps The clearance to use for near checking.
             * @param map The new configurations for the systems to be checked for collisions.
             * @param list The white list of the systems to check for collisions.
             * @return True if it finds a near-collision. \n
             *         False if there is no near-collision between the systems.
             */
            bool near_collision( double eps, const util::config_list_t& map, collision_list_t* list);

            /**
             * Sets the updated configurations of the systems to check for collisions.
             * Checks if there is at least one collision between the systems in the
             * white list.
             *
             * @brief Checks for near-collision after setting the configurations.
             *
             * @param eps The clearance to use for near checking.
             * @param map The new configurations for the systems to be checked for collisions.
             *
             * @return True if it finds a near-collision. \n
             *         False if there is no near-collision between the systems.
             */
            bool near_collision( double eps, const util::config_list_t& map);

            /**
             * Returns a list with all the pairs of the bodies that were collided during the
             * last simulation step.
             *
             * @brief Returns all the pairs of collided bodies.
             *
             * @return A list with all pairs of systems in collision
             */
            virtual collision_list_t* colliding_bodies() = 0;

            /**
             * Returns a list with all the pairs of bodes which are also in the
             * provided input list.
             */
            virtual collision_list_t* colliding_bodies( collision_list_t* list );

            /**
             * First sets the new configurations for the systems and the new collision list.
             * Then, returns a list with all the pairs of the bodies that were collided during
             * the last simulation step.
             *
             * @brief Returns all the pairs of the collided bodies.
             *
             * @param map The new configurations for the systems to be checked for collisions.
             * @param list The white list of the systems to check for collisions.
             * @return All the pairs of the collided bodies.
             */
            collision_list_t* colliding_bodies(const util::config_list_t& map, collision_list_t* list);

            /**
             * First sets the new configurations for the systems.
             * Then, returns a list with all the pairs of the bodies that were collided during
             * the last simulation step.
             *
             * @brief Returns all the pairs of the collided bodies.
             *
             * @param map The new configurations for the systems to be checked for collisions.
             * @return All the pairs of the collided bodies.
             */
            collision_list_t* colliding_bodies(const util::config_list_t& map);

            /**
             * Returns a list with all the pairs of the bodies that were collided during the
             * last simulation step.
             *
             * @brief Returns all the pairs of collided bodies.
             *
             * @param eps The clearanc value to check near-collisions for.
             *
             * @return A list with all pairs of systems in collision
             */
            virtual collision_list_t* near_colliding_bodies( double eps );

            /**
             * First sets the new configurations for the systems and the new collision list.
             * Then, returns a list with all the pairs of the bodies that were collided during
             * the last simulation step.
             *
             * @brief Returns all the pairs of the collided bodies.
             *
             * @param map The new configurations for the systems to be checked for collisions.
             * @param list The white list of the systems to check for collisions.
             * @param eps The clearanc value to check near-collisions for.
             * @return All the pairs of the collided bodies.
             */
            collision_list_t* near_colliding_bodies(const util::config_list_t& map, collision_list_t* list, double eps);

            /**
             * First sets the new configurations for the systems.
             * Then, returns a list with all the pairs of the bodies that were collided during
             * the last simulation step.
             *
             * @brief Returns all the pairs of the collided bodies.
             *
             * @param map The new configurations for the systems to be checked for collisions.
             * @param eps The clearanc value to check near-collisions for.
             * @return All the pairs of the collided bodies.
             */
            collision_list_t* near_colliding_bodies(const util::config_list_t& map, double eps);

            /**
             * Gets the collision list.
             *
             * @brief Gets the collision list.
             *
             * @return The collision list.
             */
            collision_list_t* get_collision_list();

            /**
             * It returns a pluginlib class loader for the \ref collision_checker_t.
             *
             * @brief It returns a plugin lib class loader for the \ref collision_checker_t.
             *
             * @return The pluginlib loader for the \ref collision_checker_t.
             */
            static pluginlib::ClassLoader<collision_checker_t>& get_loader();

            /**
             * Will be thrown if the white list is asking from the collision checker to check
             * for two systems that it does not contains.
             *
             * @brief Thrown if the system pair does not exist in the collision list.
             */
            class invalid_system_pair : public std::runtime_error
            {

              public:

                invalid_system_pair(const std::string& msg) : std::runtime_error("Invalid system pair: \"" + msg + "\"."){ };
            };


          protected:
            /**
             * The stored collision list. Is the list that maintains the pairs of the systems that
             * will be checked for collisions.
             *
             * @brief The collision list.
             */
            collision_list_t* collision_list;

            /**
             * Maintains the pairs of the bodies that collided during the last simulation step.
             *
             * @brief Maintains the pairs of the bodies that collided during the last simulation step.
             */
            vector_collision_list_t* colliding_bodies_list;

          private:
            /**
             * The pluginlib loader for the \ref collision_checker_t.
             *
             * @brief The pluginlib loader for the \ref collision_checker_t.
             */
            static pluginlib::ClassLoader<collision_checker_t> loader;
        };

        inline bool collision_checker_t::in_collision(const util::config_list_t& map, collision_list_t* list)
        {
            link_collision_list(list);
            set_configurations(map);
            return in_collision();
        }

        inline bool collision_checker_t::in_collision(const util::config_list_t& map)
        {
            set_configurations(map);
            return in_collision();
        }

        inline bool collision_checker_t::near_collision( double eps )
        {
            PRX_WARN_S("Near-Collision checking not implemented for this collision checker.  Returning true.");
            return true;
        }

        inline bool collision_checker_t::near_collision( double eps, const util::config_list_t& map, collision_list_t* list)
        {
            link_collision_list(list);
            set_configurations(map);
            return near_collision( eps );
        }

        inline bool collision_checker_t::near_collision( double eps, const util::config_list_t& map)
        {
            set_configurations(map);
            return near_collision( eps );
        }

        inline double collision_checker_t::get_clearance()
        {
            PRX_WARN_S("Get clearance not implemented for this collision checker.  Returning distance of zero.");
            return 0;
        }

        inline double collision_checker_t::get_clearance(const util::config_list_t& map, collision_list_t* list)
        {
            link_collision_list(list);
            set_configurations(map);
            return get_clearance();
        }

        inline double collision_checker_t::get_clearance(const util::config_list_t& map)
        {
            set_configurations(map);
            return get_clearance();
        }


        inline collision_list_t* collision_checker_t::colliding_bodies(const util::config_list_t& map, collision_list_t* list)
        {
            link_collision_list(list);
            set_configurations(map);
            return colliding_bodies();
        }

        inline collision_list_t* collision_checker_t::colliding_bodies(const util::config_list_t& map)
        {
            set_configurations(map);
            return colliding_bodies();
        }

        inline collision_list_t* collision_checker_t::near_colliding_bodies( double eps )
        {
            PRX_WARN_S("Near-Collision checking not implemented for this ocllision checker.  Returning NULL.");
            return NULL;
        }

        inline collision_list_t* collision_checker_t::near_colliding_bodies(const util::config_list_t& map, collision_list_t* list, double eps)
        {
            link_collision_list(list);
            set_configurations(map);
            return near_colliding_bodies( eps );
        }

        inline collision_list_t* collision_checker_t::near_colliding_bodies(const util::config_list_t& map, double eps)
        {
            set_configurations(map);
            return near_colliding_bodies( eps );
        }

        inline collision_list_t* collision_checker_t::get_collision_list()
        {
            return collision_list;
        }

    }
}

#endif

