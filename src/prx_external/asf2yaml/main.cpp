/**
 * @file asf2yaml.cpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2012, Board of Regents, NSHE, obo UNR
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * @authors Athanasios Krontiris, Andrew Dobson, Andrew Kimmel, Zakary Littlefield, James Marble, Kostas Bekris 
 * 
 * Email: pracsys@googlegroups.com
 */

#include <cstdlib>
#include <iostream>
#include <fstream>

#include "headers/Library.hpp"
#include "headers/Skeleton.hpp"
#include "headers/ReadSkeleton.hpp"
#include "headers/pose_utils.hpp"


/*
 * 
 */
int main(int argc, char** argv)
{
    Library::Skeleton skeleton;
    std::string full_data_path = "/Users/krontir/Research/pracsys/prx_input/experiments/skeleton_animation/CMU/";
    std::string infile;
    std::string infile_path = "../../pracsys/prx_input/experiments/skeleton_animation/CMU/";
    std::string outfile = "../../pracsys/prx_input/systems/pqp/skeleton/";
    std::string outfile_animator = "../../pracsys/prx_input/controllers/animators/mc_animator.yaml";
    
    int pose_size = 62;
    
    char the_path[256];

    getcwd(the_path, 255);
    printf("Current path : %s\n",the_path);
    
    if(argc <= 2)
    {
        printf("Give input file : ");
        std::cin >> infile;
    }
    else
        infile = argv[1];
    
    outfile = outfile + "skeleton.yaml";       
    infile = infile + ".asf";       
    
//    if(argc <= 3)
//    {
//        printf("Give output file : ");
//        std::cin >> outfile;
//    }
//    else
//        outfile = argv[2];

    Library::init(infile_path);
    if (Library::motion_count() == 0) {
        std::cerr << "------------ERROR-------------" << std::endl;
        std::cerr << "Could not find any motions to browse in directory '" << infile_path << "'.\nPlease either place amc files and associated asf here, or specify a different\n directory on on the command line." << std::endl;
        return 1;
    }
    
    Library::Motion const &motion = Library::motion(0);
                    
    float time = 0;
    time = fmodf(0 * 0.017 + time, motion.length());

    unsigned int frame = (unsigned int)(time / motion.skeleton->timestep);
    if (frame >= motion.frames()) 
        frame = motion.frames() - 1;


    Character::WorldBones wbones;
    Character::Pose pose;
    motion.get_pose(0,pose);
    Character::get_world_bones(pose,wbones);

    std::cout << "Motion has orientations for : " << pose.bone_orientations.size() << "  bones" << std::endl;
    std::cout << "wbones has  : " << wbones.tips.size() << "  bones" << std::endl;
           
    
    std::ofstream animator_file(outfile_animator.c_str());
    animator_file << "type: mc_animator" << std::endl;
    animator_file << "data_file: " << full_data_path << std::endl;
    animator_file.close();
    
    std::ofstream file(outfile.c_str());    
    
    file << "type: skeleton" << std::endl;
//    file << "file_name: " << full_data_path << std::endl;
    file << "asf_file: " << full_data_path << infile << std::endl;
    file << "state_space:" << std::endl;       
    file << "  min: [-1000, -1000, -1000, -1, -1, -1, -1]" << std::endl;
    file << "  max: [1000, 1000, 1000, 1, 1, 1, 1]" << std::endl;
    file << "control_space:" << std::endl;
    
    std::string min_ctrl_bounds, max_ctrl_bounds;
    for(int i=0; i<pose_size-1; i++)
    {
        min_ctrl_bounds = min_ctrl_bounds + "-3.14, ";
        max_ctrl_bounds = max_ctrl_bounds + "3.14, ";
    }
    min_ctrl_bounds = min_ctrl_bounds + "-3.14";
    max_ctrl_bounds = max_ctrl_bounds + "3.14";
        
    file << "  min: [" << min_ctrl_bounds << "]" << std::endl;
    file << "  max: [" << max_ctrl_bounds << "]" << std::endl;
    file << "pose_size : " << pose_size << std::endl;
    
    std::string asf_in_file = infile_path + infile;
    if(ReadSkeleton(asf_in_file,skeleton))
    {                       
        
        file << "skeleton_size : " << skeleton.bones.size() << std::endl;
        file << "root_geom: root" << std::endl;    
        file << "geometries:" << std::endl;
        file << "  -" << std::endl;

        //root
        file << "    name: root" << std::endl;
        file << "    dof: XYZxyz" << std::endl;
        file << "    parent: -1" << std::endl;
        file << "    direction: [0, 0, 0]" << std::endl;
        file << "    global_to_local: [0, 0, 0, 1]" << std::endl;    
        file << "    collision_geometry:" << std::endl;
        file << "      type: sphere" << std::endl;
        file << "      radius: 0.1" << std::endl;
        file << "      material: green" << std::endl;
        file << "    configuration:" << std::endl;
        file << "      position: [" << pose.root_position[0] << "," << pose.root_position[1] << "," << pose.root_position[2] << "]"  << std::endl;
        file << "      orientation: [" << pose.root_orientation[0] << "," << pose.root_orientation[1] << "," << pose.root_orientation[2] <<  "," << pose.root_orientation[3] <<"]"  << std::endl;
            
            
        Quatd rotx = rotation( M_PI * 0.5, make_vector(1.0, 0.0, 0.0) );
        Quatf roty = rotation( (float)M_PI, make_vector(0.0f, 1.0f, 0.0f) );
        Quatf rotz = rotation( -(float)M_PI * 0.5f, make_vector(0.0f, 0.0f, 1.0f) );
        Quatd orient, gtl;        
        Vector3f pos;
        Vector3f zero;
        zero[0]=0;
        zero[1]=0;
        zero[2]=0;
        std::string order;
        std::string after;
        
        for(size_t i=0; i<skeleton.bones.size(); ++i)
        {
            
            orient = wbones.orientations[i];
            pos = wbones.bases[i];
                        
            file << "  -" << std::endl;
            file << "    name: " << skeleton.bones[i].name << std::endl;            
            
            order.clear();
            after.clear();
            for( unsigned int o = 0; o<skeleton.bones[i].dof.size(); ++o)
            {
                std::cout << skeleton.bones[i].dof.size() << "   dof:" << skeleton.bones[i].dof[o] << std::endl;
                if(skeleton.bones[i].dof[o] == 'z')    
                    order = order + "z";
                else if(skeleton.bones[i].dof[o] == 'y')
                    order = order + "y";
                else
                    order = order + "x";
            }
            
            if(!after.empty())            
                order = order + after;            
            
            if(!order.empty())
                file << "    dof: " << order << std::endl;
            
            file << "    parent: " << skeleton.bones[i].parent << std::endl;
            file << "    direction: [" << skeleton.bones[i].direction[0] << ", " << skeleton.bones[i].direction[1] << ", " << skeleton.bones[i].direction[2] << "]" <<std::endl;
            gtl = skeleton.bones[i].global_to_local;
            file << "    global_to_local: [" << gtl[0] << ", " << gtl[1] << ", " << gtl[2] << ", " << gtl[3] << "]" <<std::endl;
            file << "    length: " << skeleton.bones[i].length << std::endl;
            file << "    frame_offset: " << skeleton.bones[i].frame_offset << std::endl;
            file << "    collision_geometry:" << std::endl;
            file << "      type: cylinder" << std::endl;
            file << "      radius: " << skeleton.bones[i].radius << std::endl;
            file << "      height: " << skeleton.bones[i].length << std::endl;
            file << "      material: blue" << std::endl;

        }
        std::cout << "root position : " << pose.root_position << std::endl;
        std::cout << "root orientation : " << pose.root_orientation << std::endl;
    }
    else
        std::cout << "No skeleton have read";

    
    file.close();
    
    
    
        
    return 0;
}

