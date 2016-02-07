/**
 * @file tf_broadcaster.cpp 
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


#include "prx/utilities/communication/tf_broadcaster.hpp"
#include "prx/utilities/math/configurations/config.hpp"
#include "prx/utilities/math/configurations/vector.hpp"
#include "prx/utilities/math/configurations/quaternion.hpp"

namespace prx
{
    namespace util
    {

        void tf_broadcaster_t::broadcast_configs()
        {
            //send all the transforms
            br.sendTransform(transforms);
            //    if(transforms.size() > 5)
            //    {
            //        PRX_ERROR_S("transforms size: " << transforms.size());
            //        foreach(tf::StampedTransform transform, transforms)
            //        {
            //
            //            const tf::Quaternion& rat = transform.getRotation();   
            //            PRX_WARN_S(transform.child_frame_id_);        
            //            PRX_ERROR_S("ITER_RAT:" << rat.getX() << "," << rat.getY()<< "," << rat.getZ()<< "," << rat.getW());
            //        }
            //    }
            transforms.clear();
        }

        void tf_broadcaster_t::queue_config(const config_t& config, const std::string& name)
        {
            tf::Transform transform;

            vector_t pos;
            quaternion_t rot;
            config.get(pos, rot);
            //    PRX_WARN_S("TF name:" << name);
            //    config.print();
            //    PRX_WARN_S(rot.get_x() << "," << rot.get_y() << "," <<  rot.get_z() << "," <<  rot.get_w());
            transform.setOrigin(tf::Vector3(pos[0], pos[1], pos[2]));

            transform.setRotation(tf::Quaternion(rot.get_x(), rot.get_y(), rot.get_z(), rot.get_w()));

            transforms.push_back(tf::StampedTransform(transform, ros::Time::now(), "/base_link", name));

            //    const tf::Quaternion& rat = transforms.back().getRotation();   
            //    PRX_INFO_S("tftoRAT:" << rat.getX() << "," << rat.getY()<< "," << rat.getZ()<< "," << rat.getW());
        }

    }
}
