/**
 * @file parameter_reader.cpp 
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

#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"

#include <ros/param.h>
#include <XmlRpc.h>
#include <deque>
#include <boost/tuple/tuple.hpp> // boost::tie

namespace prx
{
    namespace util
    {


        parameter_reader_t* global_reader;

        std::string get_xmlrpc_as_string(XmlRpc::XmlRpcValue& input);

        bool parameter_reader_t::has_attribute(const std::string& path) const
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( subpath.empty() ) // No slash found
            {
                return has_key(path);
            }
            else
            {
                bool value = false;
                const parameter_reader_t* subreader = get_subreader(name);
                value = subreader->has_attribute(subpath);
                delete subreader;
                return value;
            }
        }

        const std::string parameter_reader_t::get_attribute(const std::string& path) const
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( subpath.empty() ) // No slash found
            {
                return get_value(path);
            }
            else
            {
                if( !has_element(name) )
                    PRX_WARN_S("Can't follow path \"" << path << "\" from " << trace());
                std::string ret_val;
                const parameter_reader_t* subreader = get_subreader(name);
                ret_val = subreader->get_attribute(subpath);
                delete subreader;
                return ret_val;
            }
        }

        const std::string parameter_reader_t::get_attribute(const std::string& name, const std::string& default_val) const
        {
            if( has_attribute(name) )
                return get_attribute(name);

            return default_val;
        }

        std::auto_ptr<const parameter_reader_t> parameter_reader_t::get_child(const std::string& path) const
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( path.empty() )
            {
                PRX_FATAL_S("Calling get child with empty path");
            }

            else if( subpath.empty() ) // No slash found
            {
                return std::auto_ptr<const parameter_reader_t > (get_subreader(path));
            }
            else
            {
                if( !has_element(name) )
                    PRX_WARN_S("Can't follow path \"" << path << "\" from " << trace());

                const parameter_reader_t* subreader = get_subreader(name);
                std::auto_ptr<const parameter_reader_t> child_reader = subreader->get_child(subpath);
                delete subreader;
                return child_reader;
            }
        }

        std::vector<const parameter_reader_t*> parameter_reader_t::get_list(const std::string& path) const
        {
            std::string name;
            std::string subpath;
            boost::tie(name, subpath) = split_path(path);

            if( subpath.empty() ) // No slash found
            {
                return get_subreaders(path);
            }
            else
            {
                if( !has_element(name) )
                    PRX_WARN_S("Can't follow path \"" << path << "\" from " << trace());

                const parameter_reader_t* subreader = get_subreader(name);
                std::vector<const parameter_reader_t*> subreaders = subreader->get_list(subpath);
                delete subreader;
                return subreaders;
            }
        }

        std::map<const std::string, const parameter_reader_t*> parameter_reader_t::get_map(const std::string& path) const
        {
            return get_child(path)->get_subreaders_map();
        }

        parameter_reader_t::parameter_reader_t(const std::string& path) : path(path)
        {

            if(global_reader==NULL)
            {
                XmlRpc::XmlRpcValue global_root;
                if( !ros::param::get("", global_root)  )
                {
                    PRX_FATAL_S("Can't create global parameter_reader_t");
                }
                global_reader = new parameter_reader_t(global_root,"");
            }

            char last = this->path[this->path.length() - 1];
            if( last == '/' )
            {
                this->path = this->path.substr(0,this->path.length()-1);
            }
            last = this->path[0];
            if( last == '/' )
            {
                this->path = this->path.substr(1,this->path.length()-1);
            }
            std::auto_ptr<const parameter_reader_t> subreader = global_reader->get_child(this->path);

            root = XmlRpc::XmlRpcValue(subreader->root);
            // if( !ros::param::get(path, root) )
            //     PRX_FATAL_S("Can't create root parameter_reader_t at " << path);

            if( root.getType() != XmlRpc::XmlRpcValue::TypeStruct )
                PRX_FATAL_S("Subreader constructed from nondictionary at " << path);
        }

        parameter_reader_t::parameter_reader_t(XmlRpc::XmlRpcValue root, const std::string& path)
        : root(root), path(path)
        {
        }

        const std::string parameter_reader_t::trace() const
        {
            return path;
        }

        const std::string parameter_reader_t::get_value(const std::string& key) const
        {
            std::string value;
            try
            {
                if( root.getType() != XmlRpc::XmlRpcValue::TypeStruct )
                {
                    if( key.empty() )
                        value = get_xmlrpc_as_string(root);
                    else
                        PRX_WARN_S("Expected dictionary in " << trace() << " for key " << key);
                }
                else
                {
                    if( has_key(key) )
                        value = get_xmlrpc_as_string(root[key]);
                    else
                        PRX_WARN_S("Expected key " << key << " in " << trace());
                }
            }
            catch( XmlRpc::XmlRpcException e )
            {
                PRX_FATAL_S("Exception (" << e.getMessage() << ")"
                            << " reading ROS parameter server value: "
                            << path << '/' << key);
            }

            return value;
        }

        bool parameter_reader_t::has_key(const std::string& key) const
        {
            bool result = false;

            if( root.getType() != XmlRpc::XmlRpcValue::TypeStruct )
            {
                if( key.empty() )
                    result = true;
                // else
                //     PRX_WARN_S("Expected dictionary \"" << trace() << "\" when looking for key " << key);
            }
            else
                result = root.hasMember(key);

            return result;
        }

        const parameter_reader_t* parameter_reader_t::get_subreader(const std::string& key) const
        {
            // if( !has_key(key) )
            //     PRX_WARN_S("No element with name \"" << path <<", "<< key << '"');

            XmlRpc::XmlRpcValue sub_reader;
            try
            {
                sub_reader = root[key];
            }
            catch( XmlRpc::XmlRpcException e )
            {
                PRX_FATAL_S("Exception (" << e.getMessage() << ")"
                            << " reading ROS parameter server list: " << path <<" "<< key);
            }

            if(key[key.length()-1]!='/')
                return new parameter_reader_t(sub_reader, path + key + '/');
            return new parameter_reader_t(sub_reader, path + key);
        }

        std::vector<const parameter_reader_t*> parameter_reader_t::get_subreaders(const std::string& key) const
        {
            // if( !has_key(key) )
            //     PRX_WARN_S("No element with name " << path << " " << key);

            std::vector<const parameter_reader_t*> sub_readers;

            XmlRpc::XmlRpcValue sub_trees;
            try
            {
                if( key.empty() )
                    sub_trees = root;
                else
                    sub_trees = root[key];
            }
            catch( XmlRpc::XmlRpcException e )
            {
                PRX_FATAL_S("Exception (" << e.getMessage() << ")"
                            << " reading ROS parameter server list: " << path << key);
            }
            if( sub_trees.getType() != XmlRpc::XmlRpcValue::TypeArray )
                PRX_FATAL_S("Element with name \"" << path << "\"" << key << "\" is not a list.");

            sub_readers.reserve(sub_trees.size());
            for( int i = 0; i < sub_trees.size(); ++i )
            {
                XmlRpc::XmlRpcValue& sub_tree = sub_trees[i];
                sub_readers.push_back(new parameter_reader_t(sub_tree, path + key + '/'));
            }
            return sub_readers;
        }

        /**
         * A subclass of XmlRpc::XmlRpcValue that exposes keys of the map.
         */
        class MyXmlRpcValue : public XmlRpc::XmlRpcValue
        {

          public:

            const ValueStruct& get_map() const
            {
                return *_value.asStruct;
            }
        };

        std::map<const std::string, const parameter_reader_t*> parameter_reader_t::get_subreaders_map() const
        {
            std::map<const std::string, const parameter_reader_t*> sub_readers_map;

            if( root.getType() != XmlRpc::XmlRpcValue::TypeStruct )
                PRX_FATAL_S("Element with name " << path << " is not a map.");

            const MyXmlRpcValue& my_rpc_value = static_cast<const MyXmlRpcValue&>(root);
            const XmlRpc::XmlRpcValue::ValueStruct& value_map = my_rpc_value.get_map();
            typedef XmlRpc::XmlRpcValue::ValueStruct::value_type key_value_t;

            foreach(key_value_t key_value, value_map)
            {
                const std::string& key = key_value.first;
                const XmlRpc::XmlRpcValue& value = key_value.second;
                sub_readers_map[key] = new parameter_reader_t(value, path + key + '/');
            }

            return sub_readers_map;
        }

        std::istream& operator>>(std::istream& input, std::vector<double>& v)
        {

            while( input )
            {
                double r;
                input >> r;
                if( input )
                    v.push_back(r);
            }

            input.clear();

            return input;
        }

        std::istream& operator>>(std::istream& input, std::vector<std::string>& v)
        {

            while( input )
            {
                std::string r;
                input >> r;
                if( input )
                    v.push_back(r);
            }

            input.clear();

            return input;
        }

        std::string get_xmlrpc_as_string(XmlRpc::XmlRpcValue& input)
        {
            std::string value;
            switch( input.getType() )
            {
                case XmlRpc::XmlRpcValue::TypeBoolean:
                    value = boost::lexical_cast<std::string > (static_cast<bool>(input));
                    break;
                case XmlRpc::XmlRpcValue::TypeInt:
                    value = boost::lexical_cast<std::string > (static_cast<int>(input));
                    break;
                case XmlRpc::XmlRpcValue::TypeDouble:
                    value = boost::lexical_cast<std::string > (static_cast<double>(input));
                    break;
                case XmlRpc::XmlRpcValue::TypeString:
                    value = (std::string)input;
                    break;
                case XmlRpc::XmlRpcValue::TypeDateTime:
                    throw XmlRpc::XmlRpcException("Can't handle XmlRpc::XmlRpcValue::TypeDateTime");
                case XmlRpc::XmlRpcValue::TypeBase64:
                    throw XmlRpc::XmlRpcException("Can't handle XmlRpc::XmlRpcValue::TypeBase64");
                case XmlRpc::XmlRpcValue::TypeArray:
                    value.append(get_xmlrpc_as_string(input[0]));
                    for( int i = 1; i < input.size(); ++i )
                    {
                        value.push_back(' ');
                        value.append(get_xmlrpc_as_string(input[i]));
                    }
                    break;
                case XmlRpc::XmlRpcValue::TypeStruct:
                    throw XmlRpc::XmlRpcException("Can't handle XmlRpc::XmlRpcValue::TypeStruct");
                case XmlRpc::XmlRpcValue::TypeInvalid:
                    throw XmlRpc::XmlRpcException("Invalid XmlRpcValue type.");
            }

            return value;
        }

        namespace parameters
        {

            std::string get_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader, const std::string& default_val)
            {
                return get_attribute_as<std::string > (name, reader, template_reader, default_val);
            }

            std::string get_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                return get_attribute_as<std::string > (name, reader, template_reader);
            }

            bool has_attribute(const std::string& name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                PRX_ASSERT(reader != NULL);
                bool has = false;
                if( reader->has_attribute(name) )
                    has = true;
                else if( template_reader != NULL && template_reader->has_attribute(name) )
                    has = true;
                return has;
            }

            std::vector<const parameter_reader_t*> get_list(const std::string list_name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                if( reader->has_attribute(list_name) )
                    return reader->get_list(list_name);
                else if( template_reader != NULL && template_reader->has_attribute(list_name) )
                    return template_reader->get_list(list_name);

                return std::vector<const parameter_reader_t*>();

            }

            parameter_reader_t::reader_map_t get_map(const std::string map_name, const parameter_reader_t* reader, const parameter_reader_t* template_reader)
            {
                if( reader->has_attribute(map_name) )
                    return reader->get_map(map_name);
                else if( template_reader != NULL && template_reader->has_attribute(map_name) )
                    return template_reader->get_map(map_name);

                return parameter_reader_t::reader_map_t();

            }
        }
    }
}
