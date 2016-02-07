/**
 * @file parameter_reader.hpp 
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

#ifndef PRACSYS_PARAMETER_READER_HPP
#define	PRACSYS_PARAMETER_READER_HPP

#include "prx/utilities/definitions/defs.hpp"

#include <boost/function.hpp>
#include <boost/lambda/construct.hpp> // boost::new_prt
#include <boost/noncopyable.hpp>
#include <boost/scoped_ptr.hpp>
#include <memory> // std::auto_ptr
#include <sstream>
#include <typeinfo>
#include <vector>
#include <string>
#include <ros/param.h>

#include <pluginlib/class_loader.h>

namespace prx 
{ 
    namespace util 
    {
        std::istream& operator>>( std::istream& input, std::vector<double>& v );
        std::istream& operator>>( std::istream& input, std::vector<std::string>& v );


        /**
         * The parameter reader is responsible for retrieving attributes from
         * the input .yaml files and .launch files.  Many classes in PRACSYS
         * use the parameter reader for initialization.
         * @brief <b> Responsible for retrieving attributes from input files </b>
         *
         * @authors James Marble, Zakary Littlefield
         */

        class parameter_reader_t : boost::noncopyable
        {

        public:

            parameter_reader_t(const std::string& path);

            virtual ~parameter_reader_t() { };

            /**
             * Initializes an object by calling its \c init() function
             *
             * @brief Initializes an object
             * @param a An object with an init function that accepts a reader
             * @param name The name of the child element to pass to the init function
             */
            template <class T>
            void initialize(T* a, const std::string& name) const
            {
                if(name!="")
                {
                    const boost::scoped_ptr<const parameter_reader_t> subreader(get_child(name));
                    a->init(subreader.get());
                }
                else
                {
                    a->init(this);
                }
            }
            
            /**
             * Creates a new object and initializes it.
             * 
             * @brief Creates a new object and initializes it.
             * @param name The name of the child element to pass to the init function
             * @return a new object of the desired class initialized with the child
             */
            template <class T>
            T* initialize_new(const std::string& name,
                      boost::function<T* () > generator = boost::lambda::new_ptr<T>()) const
            {
                T* a = generator();
                initialize(a, name);
                return a;
            }

            /**
             * Creates a new object of type plugin but it does not initialize it.
             * 
             * @brief Creates a new object of type plugin but does not initialize it
             * @param name The name of the child element.
             * @param node_name The name of the node that the element is registered under.
             * 
             * @return A memory allocated, uninitialized object. 
             */
            template<class T>
            T* create_from_loader(const std::string name, const std::string node_name) const
            {
                T* a;
                std::string plugin_type_name;
                if(name!="")
                {
                    std::auto_ptr<const parameter_reader_t> reader = get_child(name);
                    plugin_type_name = reader->get_attribute("type");
                }
                else
                {
                    plugin_type_name = this->get_attribute("type");
                }
                try
                {
                    pluginlib::ClassLoader<T>& loader = T::get_loader(); 
                    a = loader.createUnmanagedInstance(node_name + "/" + plugin_type_name);
                }
                catch( pluginlib::PluginlibException& ex )
                {
                    PRX_FATAL_S("Could not load " << name.c_str() << " as type in reader " << trace().c_str() << " : " << ex.what());
                }

                PRX_ASSERT(a != NULL);
                return a;
            }

            /**
             * Creates a new object of type plugin and initializes it.
             * 
             * @brief Creates a new object of type plugin and initializes it.
             * @param name The name of the child element.
             * @param node_name The name of the node that the element is registered under.
             * 
             * @return A memory allocated, initialized object.
             */
            template<class T>
            T* initialize_from_loader(const std::string name, const std::string node_name) const
            {
                std::string plugin_type_name;
                T* a = this->create_from_loader<T>(name,node_name);
                PRX_ASSERT(a != NULL);
                initialize(a,name);
                return a;
            }


            /**
             * Create a reader from subtree of this one.
             *
             * @brief Create a reader from subtree of this one.
             * @param name the name of the child element
             * @return a reader that acts as if the named child element is the root
             */
            virtual std::auto_ptr<const parameter_reader_t> get_child(const std::string& name) const;

            /**
             * Create readers from all the subtrees of this one with the same name.
             *
             * @brief Create readers from all the subtrees of this one with the same name.
             * @param name the name of the child elements
             * @return a list of readers
             */
            virtual std::vector<const parameter_reader_t*> get_list(const std::string& name) const;

            typedef std::map<const std::string, const parameter_reader_t*> reader_map_t;

            /**
             * Return all key/value pairs for a given path.
             *
             * @brief Return all key/value pairs for a given path.
             * @param name the name of the child map
             * @return a list of readers
             */
            virtual reader_map_t get_map(const std::string& name) const;

            /**
             * Checks if a child element exists.
             *
             * @brief Checks if a child element exists.
             * @param name element name to check for.
             * @return true if there is a subelement with the given name.
             */
            bool has_element(const std::string& name) const
            {
                return has_attribute(name);
            }

            /**
             * Checks if an attribute exists.
             *
             * @brief Checks if an attribute exists.
             * @param name attribute name to check for.
             * @return true if there is an attribute with the given name.
             */
            bool has_attribute(const std::string& name) const;

            /**
             * Retrieves a string from the reader with the name /c name
             * 
             * @brief Retrieves a string from the reader with the name /c name
             * @param name The name of the string attribute to retrieve
             * @return The retrieved string
             */
            const std::string get_attribute(const std::string& name) const;

            /**
             * Retrieves a string from the reader with the name /c name. 
             * 
             * If the attribute does not exist in the reader, the default value is returned
             * 
             * @brief Retrieves a string from the reader with the name /c name. Allows default values.
             * @param name The name of the string attribute to retrieve
             * @param default_val The default value to return if the attribute does not exist
             * @return The retrieved string
             */
            const std::string get_attribute(const std::string& name, const std::string& default_val) const;


            /**
             * Retrieves a templated type from the reader with the name /c name
             * 
             * @brief Retrieves a templated type from the reader with the name /c name
             * @param name The name of the template attribute to retrieve
             * @return  The retrieved template value
             */
            template<typename T>
            T get_attribute_as(const std::string& name) const
            {
                // Get the attribute as a string.
                const std::string attribute = get_attribute(name);

                // Put it in an istream.
                std::istringstream iss(attribute);

                // Try to push it in the T.
                T result;
                if( !(iss >> result) )
                {
                    const std::string typeName = typeid (T).name();
                    PRX_FATAL_S("Can't parse \"" << attribute << "\" as " << typeName);
                }

                if( iss.peek() != std::istream::traits_type::eof() )
                {
                    std::string extra;
                    iss >> extra;
                    const std::string typeName = typeid (T).name();
                    PRX_FATAL_S("Extra characters \"" << extra
                                << "\" when parsing \"" << attribute
                                << "\" as " << typeName);
                }
                return result;
            }


            /**
             * Retrieves a templated type from the reader with the name /c name
             * 
             * If the attribute does not exist in the reader, the default value is returned
             * 
             * @brief Retrieves a templated type from the reader with the name /c name
             * @param name The name of the template attribute to retrieve
             * @param default_val The default value to return if the attribute does not exist
             * @return  The retrieved template value
             */
            template<typename T>
            T get_attribute_as(const std::string& name, const T& default_val) const
            {
                if(has_attribute(name))
                    return get_attribute_as<T>(name);
                return default_val;
            }

            /**
             * Traces the current position of the reader in the parameter tree
             * 
             * @brief Traces the current position of the reader in the parameter tree
             * @return a "stack trace" of the current position in the parameter tree.
             */
            virtual const std::string trace() const;
            
            /**
             * Exports the loaded parameters into an xml string.
             *
             * @brief Exports the loaded parameters into an xml string.
             *
             * @return A string containing all the parameters.
             */
             inline std::string toXML()
             {
                return root.toXml();
             }

        protected:

            /**
             * Retrieves the string indexed by /c key in the reader
             * 
             * @brief Retrieves the string indexed by /c key in the reader
             * @param key The name of the string to retrieve
             * @return The data as a string
             */
            virtual const std::string get_value(const std::string& key) const;

            /**
             * Checks if the key exists in the reader
             * 
             * @brief Checks if the key exists in the reader
             * @param key The key to check for existence
             * @return True if the key exists in the reader, false otherwise
             */
            virtual bool has_key(const std::string& key) const;


            /**
             * Parameter readers may be nested. For example, a reader for the path
             * "planner/rrt" can be split into the reader "planner" and the subreader "rrt".
             * 
             * @brief Attempts to retrieve the subreader
             * @param name The name of the subreader
             * @return A pointer to the parameter reader of the subpath
             */
            virtual const parameter_reader_t* get_subreader(const std::string& name) const;


            /**
             * Retrieves all subreaders
             * 
             * @brief Retrieves all subreaders
             * @param name The path of the subreader
             * @return A vector containing all subreaders
             */
            virtual std::vector<const parameter_reader_t*> get_subreaders(const std::string& name) const;


            /**
             * Retrieves all subreaders into a map indexed by path
             * 
             * @brief Retrieves all subreaders into a map indexed by path
             * @return A map of the subreaders indexed by path
             */
            virtual std::map<const std::string, const parameter_reader_t*> get_subreaders_map() const;

        private:

            parameter_reader_t(XmlRpc::XmlRpcValue root, const std::string& path);

            /** @brief The root path of the parameter reader */
            mutable XmlRpc::XmlRpcValue root;

            /** @brief The current path of the parameter reader */
            std::string path;

        };

        namespace parameters
        {

            std::string get_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader, const std::string& default_val);
            std::string get_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader);


            template<typename T>
            T get_attribute_as(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader, const T& default_val)
            {
                PRX_ASSERT( reader != NULL );

                if( reader->has_attribute(name) )
                    return reader->get_attribute_as<T>(name, default_val);
                if( template_reader != NULL )
                    return template_reader->get_attribute_as<T>(name, default_val);

                return default_val;
            }

            template<typename T>
            T get_attribute_as(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_ASSERT( reader != NULL );

                if( reader->has_attribute(name) )
                    return reader->get_attribute_as<T>(name);
                if( template_reader != NULL )
                    return template_reader->get_attribute_as<T>(name);

                T result = T();
                return result;
            }

            bool has_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader);

            template<class T>
            T* create_from_loader(const std::string node_name, const parameter_reader_t* reader, const std::string reader_name, const parameter_reader_t* template_reader, const std::string template_name="")
            {
                std::string reader_element_name;
                std::string template_element_name;
                if(reader_name!="")
                {
                    reader_element_name = reader_name+"/type";
                }
                else
                {
                    reader_element_name = "type";                    
                }
                if( reader->has_attribute(reader_element_name) )
                {
                    return reader->create_from_loader<T>(reader_name,node_name);
                }
                if( template_reader != NULL )
                {
                    if(template_name!="")
                    {
                        template_element_name = template_name+"/type";
                    }
                    else
                    {
                        template_element_name = "type";                    
                    }
                    if( template_reader->has_attribute(template_element_name) )
                    {
                        return template_reader->create_from_loader<T>(template_name,node_name);
                    }
                }
                PRX_FATAL_S("Failed to create from loader, most likely no type attribute defined.");
                return NULL;
            }

            template<class T>
            T* initialize_from_loader(const std::string node_name, const parameter_reader_t* reader, const std::string reader_name, const parameter_reader_t* template_reader, const std::string template_name="")
            {
                T* a = create_from_loader<T>(node_name,reader,reader_name,template_reader,template_name);
                PRX_ASSERT(a != NULL);
                std::auto_ptr<const parameter_reader_t> reader_init_hold,template_init_hold;
                const parameter_reader_t* reader_init=NULL;
                const parameter_reader_t* template_init=NULL;
                if(template_reader!=NULL)
                {
                    if(template_name!="")
                    {
                        template_init_hold = template_reader->get_child(template_name);
                        template_init = template_init_hold.get();
                    }
                    else
                    {
                        template_init = template_reader;
                    }
                }
                
                if(reader_name!="")
                {
                    reader_init_hold = reader->get_child(reader_name);
                    reader_init = reader_init_hold.get();
                }
                else
                {
                    reader_init = reader;
                }
                
                a->init(reader_init, template_init);
                return a;
            }

            template<class T>
            void initialize(T* a, const parameter_reader_t* reader, const std::string reader_name, const parameter_reader_t* template_reader, const std::string template_name="")
            {
                PRX_ASSERT(a != NULL);
                std::auto_ptr<const parameter_reader_t> reader_init_hold,template_init_hold;
                const parameter_reader_t* reader_init=NULL;
                const parameter_reader_t* template_init=NULL;
                if(template_reader!=NULL)
                {
                    if(template_name!="")
                    {
                        template_init_hold = template_reader->get_child(template_name);
                        template_init = template_init_hold.get();
                    }
                    else
                    {
                        template_init = template_reader;
                    }
                }
                
                if(reader_name!="")
                {
                    reader_init_hold = reader->get_child(reader_name);
                    reader_init = reader_init_hold.get();
                }
                else
                {
                    reader_init = reader;
                }
                
                a->init(reader_init, template_init);
//                return a;
            }

            std::vector<const parameter_reader_t*> get_list(const std::string list_name, const parameter_reader_t* reader, const parameter_reader_t* template_reader);
            
            parameter_reader_t::reader_map_t get_map(const std::string map_name, const parameter_reader_t* reader, const parameter_reader_t* template_reader);

        }
    }
}


#endif	
