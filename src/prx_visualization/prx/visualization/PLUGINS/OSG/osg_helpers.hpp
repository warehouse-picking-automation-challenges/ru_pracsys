/**
 * @file osg_helpers.hpp 
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

#ifndef OSG_HELPERS_HPP
#define	OSG_HELPERS_HPP

#include "prx/utilities/definitions/defs.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"
#include "prx/visualization/PLUGINS/OSG/osg_basic.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"

namespace prx 
{ 
    namespace vis 
    {

/**
 * Converts a util::vector_t to an osg::Vec3
 * 
 * @brief Converts a util::vector_t to an osg::Vec3
 * @param vec The util::vector_t to be converted
 * @return The converted osg::Vec3
 */
inline osg::Vec3 toVec3(const util::vector_t& vec) 
{
    PRX_ASSERT(vec.get_dim() == 3);
    return osg::Vec3(vec[0], vec[1], vec[2]);
}


/**
 * Converts a std::vector to an osg::Vec3
 * 
 * @brief Converts a std::vector to an osg::Vec3
 * @param vec The std::vector to be converted
 * @return The converted osg::Vec3
 */
inline osg::Vec3 toVec3(const std::vector<double>& vec) 
{
    PRX_ASSERT(vec.size() == 3);
    return osg::Vec3(vec[0], vec[1], vec[2]);
}

/**
 * Converts a util::vector_t to an osg::Vec4
 * 
 * @brief Converts a util::vector_t to an osg::Vec4
 * @param vec The util::vector_t to be converted
 * @return The converted osg::Vec4
 */
inline osg::Vec4 toVec4(const util::vector_t& vec) 
{
    PRX_ASSERT(vec.get_dim() == 4);
    return osg::Vec4(vec[0], vec[1], vec[2], vec[3]);
}

/**
 * Converts a std::vector to an osg::Vec4
 * 
 * @brief Converts a std::vector to an osg::Vec4
 * @param vec The std::vector to be converted
 * @return The converted osg::Vec4
 */
inline osg::Vec4 toVec4(const std::vector<double>& vec) 
{
    PRX_ASSERT(vec.size() == 4);
    return osg::Vec4(vec[0], vec[1], vec[2], vec[3]);
}

/**
 * Converts a util::quaternion_t to an osg::Vec4
 * 
 * @brief Converts a util::quaternion_t to an osg::Vec4
 * @param quat The util::quaternion_t to be converted
 * @return The converted osg::Vec4
 */
inline osg::Vec4 toVec4(const util::quaternion_t& quat) 
{
    double x, y, z, w;
    quat.get(x, y, z, w);
    return osg::Vec4(x, y, z, w);
}

/**
 * Converts an osg::Vec3 into a util::vector_t
 * 
 * @brief Converts an osg::Vec3 into a util::vector_t
 * @param vec The osg::Vec3 to be converted
 * @return The converted util::vector_t
 */
inline util::vector_t fromVec3(const osg::Vec3& vec) 
{
    return util::vector_t(vec[0], vec[1], vec[2]);
}

/**
 * Converts an osg::Vec3 into a util::vector_t
 * 
 * @brief Converts an osg::Vec3 into a util::vector_t
 * @param vec The osg::Vec3 to be converted
 * @return The converted util::vector_t
 */
inline util::vector_t fromVec3d(const osg::Vec3d& vec) 
{
    return util::vector_t(vec[0], vec[1], vec[2]);
}

/**
 * Converts an osg::Quat into a util::quaternion_t
 * 
 * @brief Converts an osg::Quat into a util::quaternion_t
 * @param quat The osg::Quat to be converted
 * @return The converted util::quaternion_t
 */
inline util::quaternion_t fromOSGQuat(const osg::Quat& quat) 
{
    return util::quaternion_t(quat[0], quat[1], quat[2], quat[3]);
}

/**
 * Converts an double pointer into an osg::Quat
 * 
 * @brief Converts an double pointer into an osg::Quat
 * @param quat The double pointer to be converted
 * @return The converted osg::Quat
 */
inline osg::Quat toOSGQuat(const double* quat) 
{
    return osg::Quat(quat[0], quat[1], quat[2], quat[3]);
}

inline osg::Quat toOSGQuat(const util::quaternion_t& quat) 
{
    return osg::Quat(quat[0], quat[1], quat[2], quat[3]);
}

/**
 * Reads in common input attributes for a util::vector_t.
 * 
 * Initializes a util::vector_t using these attributes.
 * 
 * @brief Reads in common input attributes for a util::vector_t
 * @param reader The parameter reader
 * @param path The path of the util::vector_t in the input file
 * @return An initialized util::vector_t
 */
inline util::vector_t xml_vector_attr(const util::parameter_reader_t* reader, const std::string& path)
{
    return util::vector_t(
        reader->get_attribute_as<double>(path + "/x"),
        reader->get_attribute_as<double>(path + "/y"),
        reader->get_attribute_as<double>(path + "/z"));
}

/**
 * Reads in common input attributes for a util::quaternion_t.
 * 
 * Initializes a util::quaternion_t using these attributes.
 * 
 * @brief Reads in common input attributes for a util::quaternion_t
 * @param reader The parameter reader
 * @param path The path of the util::quaternion_t in the input file
 * @return An initialized util::quaternion_t
 */
inline util::quaternion_t xml_quat_attr(const util::parameter_reader_t* reader, const std::string& path)
{
    return util::quaternion_t(
        reader->get_attribute_as<double>(path + "/x"),
        reader->get_attribute_as<double>(path + "/y"),
        reader->get_attribute_as<double>(path + "/z"),
        reader->get_attribute_as<double>(path + "/w"));
}

/**
 * Reads in common input attributes for a util::vector_t. Typically
 * used to read in color vectors.
 * 
 * Initializes a util::vector_t using these attributes.
 * 
 * @brief Reads in common input attributes for a util::vector_t
 * @param reader The parameter reader
 * @param path The path of the util::vector_t in the input file
 * @return An initialized util::vector_t
 */
inline util::vector_t xml_color_attr(const util::parameter_reader_t *reader, const std::string& path)
{
    return util::vector_t(
        reader->get_attribute_as<double>(path + "/r"),
        reader->get_attribute_as<double>(path + "/g"),
        reader->get_attribute_as<double>(path + "/b"),
        reader->get_attribute_as<double>(path + "/a"));
}

    }
 }


#endif	/* OSG_HELPERS_HPP */
