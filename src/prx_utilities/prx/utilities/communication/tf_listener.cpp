/**
 * @file tf_listener.cpp 
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

#include "prx/utilities/communication/tf_listener.hpp"
#include "prx/utilities/math/configurations/config.hpp"

namespace prx
{
    namespace util
    {

        void tf_listener_t::lookup(const std::string& name, config_t& config) const
        {
            tf::StampedTransform transform;

            try
            {
                tf_listener.lookupTransform("map", name, ros::Time(0), transform);
                //        PRX_INFO_S("listener while looking up transformation for correct:" << name);        

            }
            catch( tf::TransformException ex )
            {

                //        PRX_WARN_S("Exception while looking up transformation for " << name << std::endl
                //                   << ex.what());
                //        PRX_LOG_ERROR("Exception while looking up transformation for ");
            }

            const tf::Vector3& pos = transform.getOrigin();
            config.set_position(pos.x(), pos.y(), pos.z());

            const tf::Quaternion& rot = transform.getRotation();

            //    PRX_INFO_S("tfROT:" << rot.getX() << "," << rot.getY()<< "," << rot.getZ()<< "," << rot.getW());
            config.set_xyzw_orientation(rot.getX(), rot.getY(), rot.getZ(), rot.getW());

            //    PRX_ERROR_S("---------------------------------------------------------------------");
            //    PRX_ERROR_S("--------------------------------------------------------------------");
            //    PRX_ERROR_S("--------------------------------------------------------------------");
            //    PRX_WARN_S("listener2 name:" << name);
            //    config.print();
            //    PRX_ERROR_S("--------------------------------------------------------------------");
            //    PRX_ERROR_S("--------------------------------------------------------------------");
            //    PRX_ERROR_S("--------------------------------------------------------------------");
        }

    }
}
