/**
 * @file defs.hpp 
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

#ifndef PRACSYS_DEFINITIONS_HPP
#define PRACSYS_DEFINITIONS_HPP 

//=============================================================================
//= Standard Library Inclusions
//=============================================================================
#include <cmath>
#include <cstdio>
#include <string>
#include <sstream>
#include <iosfwd>
#include <vector>
#include <stdexcept>

// Fix deprecation warnings when boost uses backward/hash_set etc...
#include <boost/config.hpp>
# if defined(__GLIBCXX__) && __GLIBCXX__ > 20100000 // GCC > 4.4
#  define BOOST_NO_HASH
# endif

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

//begin namespace
#define PRX_START   namespace prx{
//end namespace
#define PRX_FINISH  }

//=============================================================================
//= Arithmetic Checking values
//=============================================================================

/// A definition of the infinite value
#define PRX_INFINITY    99999999

/// A zero limit definition.  All positive values less than this are assumed zero.
#define PRX_ZERO_CHECK      1e-6
#define PRX_DISTANCE_CHECK  1e-4

#ifdef NO_ROS
#define PRX_PI 3.1415926535897932385
#else
#define PRX_PI M_PI
#endif
#define PRX_2PI 2.0*PRX_PI
#define PRX_HALF_PI PRX_PI/2.0

// Threading defines
#define PRX_MAX_THREADS 8

// Visualization flags
#define PRX_VIS_TEMPORARY_REMOVE 0
#define PRX_VIS_TEMPORARY_SHOW 1
#define PRX_VIS_REMOVE 2
#define PRX_VIS_SHOW 3

//=============================================================================
//= Keyboard Values
//=============================================================================
enum KEYS
{
    PRX_KEY_LEFT                = 0xFF51,
    PRX_KEY_UP                  = 0xFF52,
    PRX_KEY_RIGHT               = 0xFF53,
    PRX_KEY_DOWN                = 0xFF54,
    PRX_KEY_RETURN              = 0xFF0D,
    PRX_KEY_PAUSE               = 0xFF0C,
    PRX_KEY_CTRL                = 0xFFE3,
    PRX_KEY_TAB                 = 0xFF09,
    PRX_KEY_0                   = 0x30,
    PRX_KEY_1                   = 0x31,
    PRX_KEY_2                   = 0x32,
    PRX_KEY_3                   = 0x33,
    PRX_KEY_LEFT_BRACKET        = 0x5B,
    PRX_KEY_RIGHT_BRACKET       = 0x5D,
    PRX_KEY_PAGE_UP             = 0xFF55,
    PRX_KEY_PAGE_DOWN           = 0xFF56,
    PRX_KEY_DELETE              = 0xFFFF,
    PRX_KEY_HOME                = 0xFF50,
    PRX_KEY_PLUS                = 0x43,
    PRX_KEY_LOWER_I             = 0x69,
    PRX_KEY_LOWER_J             = 0x6A,
    PRX_KEY_LOWER_K             = 0x6B,
    PRX_KEY_LOWER_L             = 0x6C,
    PRX_KEY_UPPER_H             = 0x48,
    PRX_KEY_UPPER_U             = 0x55,
    PRX_KEY_UPPER_J             = 0x4A,
    PRX_KEY_UPPER_K             = 0x4B,
    PRX_KEY_UPPER_I             = 0x49,
    PRX_KEY_UPPER_O             = 0x4F,
    PRX_KEY_UPPER_P             = 0x50,
    PRX_KEY_UPPER_L             = 0x4C,
    PRX_KEY_UPPER_COLON         = 0x3A,
    PRX_KEY_UPPER_DBL_QUT       = 0x22,
    PRX_KEY_UPPER_N             = 0x4E,
    PRX_KEY_UPPER_M             = 0x4D,
    PRX_KEY_UPPER_LESS          = 0x3C,
    PRX_KEY_UPPER_GRT           = 0x3E,
    PRX_KEY_UPPER_QU_MARK       = 0x3F,
    PRX_KEY_UPPER_CURLY_LEFT    = 0x7B,
    PRX_KEY_UPPER_CURLY_RIGHT   = 0x7D
};

//=============================================================================
//= Text Colors
//=============================================================================
enum TEXT_COLOR
{
    PRX_TEXT_BLACK = 30,
    PRX_TEXT_RED = 31,
    PRX_TEXT_GREEN = 32,
    PRX_TEXT_BROWN = 33,
    PRX_TEXT_BLUE = 34,
    PRX_TEXT_MAGENTA = 35,
    PRX_TEXT_CYAN = 36,
    PRX_TEXT_LIGHTGRAY = 37
};

//=============================================================================
//= Min/Max functions for two input values
//=============================================================================
#define PRX_MINIMUM(a,b)     (((a)<(b)) ? (a) : (b)) ///< Computing the minimum of two numbers.
#define PRX_MAXIMUM(a,b)     (((a)<(b)) ? (b) : (a)) ///< Computing the maximum of two numbers.
#define PRX_SIGN(a)          (((a)<(0)) ? (-1.0) : (1.0)) ///< Computing the sign of the number

namespace prx 
 { 
 namespace util 
 {

//=============================================================================
//= Global variables: useful for printing in debug mode
//=============================================================================
/// For parallel applications, the id of the process.  The server or single processes get an id of -1.
extern int         process_id;
/// Stores the value of the environment variable PRACSYS_PATH
extern std::string path_prefix;
/// Stores the value of the environment variable PRACSYS_MODELS_PATH
extern std::string models_path_prefix;
class parameter_reader_t;
extern parameter_reader_t* global_reader;
} 
 }

//=============================================================================
//= Debugging Macros
//=============================================================================

#ifndef NO_ROS //If we have ROS

#ifndef NDEBUG //If we are in Debugging mode
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG

#else //If we're in Release mode
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_INFO

#endif

#include <ros/console.h>
#include <ros/assert.h>
#include <ros/ros.h>

#define PRX_DEBUG_S( stream ) ROS_DEBUG_STREAM( stream )
#define PRX_INFO_S( stream ) ROS_INFO_STREAM( stream )
#define PRX_WARN_S( stream ) ROS_WARN_STREAM( stream )
#define PRX_ERROR_S( stream ) ROS_ERROR_STREAM( stream )
#define PRX_FATAL_S( stream )\
{\
    ROS_FATAL_STREAM( stream );\
    std::ostringstream message;\
    message << "Fatal error at: " << __FILE__ << " - " << __LINE__;\
	throw std::runtime_error(message.str());\
}



#define PRX_ASSERT( cond ) ROS_ASSERT( cond )
#define PRX_ASSERT_MSG(cond, ... ) ROS_ASSERT_MSG(cond, ##__VA_ARGS__ )

#ifndef NDEBUG
#define PRX_DEBUG_POINT( stream )\
{\
    std::string PRX_DEBUG_COLOR______ = "\033[0;35m";\
    std::string NODE_NAME____________ = ros::this_node::getName();\
    std::string FILE_________________ = boost::lexical_cast<std::string>(__FILE__);\
    const size_t last_slash__________ = FILE_________________.rfind('/');\
    std::string FILE_NAME____________ = FILE_________________.substr(last_slash__________ + 1, FILE_________________.npos);\
    ( std::cout << PRX_DEBUG_COLOR______ <<"[DEBUG_POINT][" << NODE_NAME____________ << "][" << FILE_NAME____________ << "::" << __FUNCTION__ << "][" << __LINE__ << "]:    " << stream << "\n" );\
    ROS_ISSUE_BREAK( );\
}
#else
#define PRX_DEBUG_POINT( stream ){}
#endif

#include <boost/lexical_cast.hpp>
#ifndef NDEBUG
#define PRX_DEBUG_COLOR( stream, stream_color )\
{\
    std::string PRX_DEBUG_COLOR______ = "\033[0;" + boost::lexical_cast<std::string>(stream_color)+"m";\
    std::string NODE_NAME____________ = ros::this_node::getName();\
    std::string FILE_________________ = boost::lexical_cast<std::string>(__FILE__);\
    const size_t last_slash__________ = FILE_________________.rfind('/');\
    std::string FILE_NAME____________ = FILE_________________.substr(last_slash__________ + 1, FILE_________________.npos);\
    ( std::cout << PRX_DEBUG_COLOR______ <<"[DEBUG_COLOR][" << NODE_NAME____________ << "][" << FILE_NAME____________ << "::" << __FUNCTION__ << "][" << __LINE__ << "]:    " << stream << "\n" );\
}
#else
#define PRX_DEBUG_COLOR( stream, stream_color ){}
#endif

#define PRX_PRINT( stream, stream_color )\
{\
    std::string PRX_COLOR____ = "\033[0;" + boost::lexical_cast<std::string>(stream_color)+"m";\
    std::string NODE_NAME____ = ros::this_node::getName();\
    std::string FILE_________ = boost::lexical_cast<std::string>(__FILE__);\
    const size_t last_slash__ = FILE_________.rfind('/');\
    std::string FILE_NAME____ = FILE_________.substr(last_slash__ + 1, FILE_________.npos);\
    ( std::cout << PRX_COLOR____ <<"[PRINT][" << NODE_NAME____ << "][" << FILE_NAME____ << "::" << __FUNCTION__ << "][" << __LINE__ << "]:    " << stream << "\n" );\
}

#define PRX_PRINT_S( stream )\
{\
    std::string PRX_COLOR____ = "\033[0;" + boost::lexical_cast<std::string>(PRX_TEXT_LIGHTGRAY)+"m";\
    std::string NODE_NAME____ = ros::this_node::getName();\
    std::string FILE_________ = boost::lexical_cast<std::string>(__FILE__);\
    const size_t last_slash__ = FILE_________.rfind('/');\
    std::string FILE_NAME____ = FILE_________.substr(last_slash__ + 1, FILE_________.npos);\
    ( std::cout << PRX_COLOR____ <<"[PRINT][" << NODE_NAME____ << "][" << FILE_NAME____ << "::" << __FUNCTION__ << "][" << __LINE__ << "]:    " << stream << "\n" );\
}

#else //If we do NOT have ROS

#define PRX_DEBUG_S( stream ) ( std::cout << "DEBUG: " << stream )
#define PRX_INFO_S( stream )  ( std::cout << "INFO: " << stream )
#define PRX_WARN_S( stream )  ( std::cout << "WARN: " << stream )
#define PRX_ERROR_S( stream ) ( std::cout << "ERROR: " << stream )
#define PRX_FATAL_S( stream )
{\
    std::cout << "FATAL: " << stream;
    std::ostringstream message;\
    message << "Fatal error: " << __FILE__ << " - " << __LINE__;\
    fflush(stdout);\
    throw std::runtime_error(message.str());\
}
#define PRX_ASSERT_MSG(cond, ... ) ((void)0)
 


#ifndef NDEBUG //If we are in Debugging mode


/**
*
* Tests the given value for failure.  If the arguments doesn't evaluate True,
* program execution ceases and throws an exception.  Throwing an uncaught
* exception instead of exiting directly will show a stack trace and
* allows for checking if an assertion was triggered during a unit test.
*
* @brief If the given argument fails, execution ceases and the program exits. 
* @param assert_value The value to check for failure.
*/
#include <stdexcept>
#define PRX_ASSERT( assert_value )\
{\
    if( ( assert_value ) == false )\
    {\
        std::ostringstream message;\
        message << "Assertion error: " << __FILE__ << " - " << __LINE__;\
        fflush(stdout);\
	throw std::runtime_error(message.str());\
    }\
}

#else //If we are in Release mode

#define PRX_ASSERT( ... )               ((void)0)
#endif //NDEBUG

#endif //NO_ROS

template <class ResultType, class SourceType>
ResultType checked_cast(SourceType* item)
{
    PRX_ASSERT(item != NULL);
    ResultType cast_item = dynamic_cast<ResultType>(item);
    PRX_ASSERT(cast_item != NULL);
    return cast_item;
}

template <class ResultType, class SourceType>
ResultType checked_cast(SourceType& item)
{
    ResultType cast_item = dynamic_cast<ResultType>(item);
    return cast_item;
}

 #include <boost/foreach.hpp>
 #include <boost/version.hpp>

 namespace boost {

 #if BOOST_VERSION != 104900
 namespace BOOST_FOREACH = foreach;
 #endif
 
 } // namespace boost
 
// #define foreach BOOST_FOREACH

#define foreach BOOST_FOREACH
#define reverse_foreach BOOST_REVERSE_FOREACH

#ifdef __GNUC__
#define VARIABLE_IS_NOT_USED __attribute__ ((unused))
#else
#define VARIABLE_IS_NOT_USED
#endif

#endif // PRACSYS_DEFINITIONS_HPP
