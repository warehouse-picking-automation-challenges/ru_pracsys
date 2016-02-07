/*
 * IKFast Demo
 *
 * Shows how to calculate FK from joint angles.
 * Calculates IK from rotation-translation matrix, or translation-quaternion pose.
 * Performance timing tests.
 *
 * Run the program to view command line parameters.
 *
 *
 * To compile, run:
 * g++ -lstdc++ -llapack -o compute ikfastdemo.cpp -lrt
 * (need to link with 'rt' for gettime(), it must come after the source file name)
 *
 *
 * Tested with Ubuntu 11.10 (Oneiric)
 * IKFast54 from OpenRAVE 0.6.0
 * IKFast56/61 from OpenRave 0.8.2
 *
 * Author: David Butterworth, KAIST
 *         Based on code by Rosen Diankov
 * Date: November 2012
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ikfastdemo.hpp"
#include "time.h"
#include <stdlib.h>
#include <math.h>
#include "prx/utilities/math/configurations/config.hpp"

namespace prx
{
    using namespace util;
    namespace packages
    {
        namespace baxter
        {

            kinematic_model_t::kinematic_model_t()
            {
                double unit[]={1,0,0,0,1,0,0,0,1};
                static_inv_tf_rot = new double[9];
                static_tf_rot = new double[9];
                for( unsigned i=0; i<9; ++i )
                {
                    static_tf_rot[i] = unit[i];
                    static_inv_tf_rot[i] = unit[i];
                }

                static_inv_tf_t = new double[3];
                static_tf_t = new double[3];
                static_inv_tf_t[0] = static_inv_tf_t[1] = static_inv_tf_t[2] = static_tf_t[0] = static_tf_t[1] = static_tf_t[2] = 0;
            }

            kinematic_model_t::~kinematic_model_t()
            {
                delete[] static_tf_rot;
                delete[] static_inv_tf_rot;
                delete[] static_tf_t;
                delete[] static_inv_tf_t;
            }

            void kinematic_model_t::Display_FK(IkReal* eetrans, IkReal* eerot, double x, double y, double z, double* fk_state) const
            {

                eetrans[0] += (x * eerot[0])+(y * eerot[1]) + z * (eerot[2]);
                eetrans[1] += (x * eerot[3])+(y * eerot[4]) + z * (eerot[5]);
                eetrans[2] += (x * eerot[6])+(y * eerot[7]) + z * (eerot[8]);

                // Display equivalent Euler angles
                float yaw;
                float pitch;
                float roll;
                if( eerot[5] > 0.998 || eerot[5] < -0.998 )
                {
                    //There is a singularity
                    yaw = IKatan2(-eerot[6], eerot[0]);
                    pitch = 0;
                }
                else
                {
                    yaw = IKatan2(eerot[2], eerot[8]);
                    pitch = IKatan2(eerot[3], eerot[4]);
                }
                roll = IKasin(eerot[5]);

                // Convert rotation matrix to quaternion (Daisuke Miyazaki)
                float q0 = (eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
                float q1 = (eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
                float q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
                float q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
                if( q0 < 0.0f ) q0 = 0.0f;
                if( q1 < 0.0f ) q1 = 0.0f;
                if( q2 < 0.0f ) q2 = 0.0f;
                if( q3 < 0.0f ) q3 = 0.0f;
                q0 = sqrt(q0);
                q1 = sqrt(q1);
                q2 = sqrt(q2);
                q3 = sqrt(q3);
                if( q0 >= q1 && q0 >= q2 && q0 >= q3 )
                {
                    q0 *= +1.0f;
                    q1 *= SIGN(eerot[7] - eerot[5]);
                    q2 *= SIGN(eerot[2] - eerot[6]);
                    q3 *= SIGN(eerot[3] - eerot[1]);
                }
                else if( q1 >= q0 && q1 >= q2 && q1 >= q3 )
                {
                    q0 *= SIGN(eerot[7] - eerot[5]);
                    q1 *= +1.0f;
                    q2 *= SIGN(eerot[3] + eerot[1]);
                    q3 *= SIGN(eerot[2] + eerot[6]);
                }
                else if( q2 >= q0 && q2 >= q1 && q2 >= q3 )
                {
                    q0 *= SIGN(eerot[2] - eerot[6]);
                    q1 *= SIGN(eerot[3] + eerot[1]);
                    q2 *= +1.0f;
                    q3 *= SIGN(eerot[7] + eerot[5]);
                }
                else if( q3 >= q0 && q3 >= q1 && q3 >= q2 )
                {
                    q0 *= SIGN(eerot[3] - eerot[1]);
                    q1 *= SIGN(eerot[6] + eerot[2]);
                    q2 *= SIGN(eerot[7] + eerot[5]);
                    q3 *= +1.0f;
                }
                float r = NORM(q0, q1, q2, q3);
                q0 /= r;
                q1 /= r;
                q2 /= r;
                q3 /= r;

                fk_state[0] = eetrans[0];
                fk_state[1] = eetrans[1];
                fk_state[2] = eetrans[2];
                fk_state[3] = q1;
                fk_state[4] = q2;
                fk_state[5] = q3;
                fk_state[6] = q0;

            }

            bool kinematic_model_t::ik_for_position(const config_t& pose, const std::vector< double >& joints, double* soln) const
            {
                IKREAL_TYPE eerot[9], eetrans[3], eerot_[9], eetrans_[9];

                unsigned int num_of_joints = GetNumJoints();
                unsigned int num_free_parameters = GetNumFreeParameters();

                IkSolutionList<IKREAL_TYPE> solutions;
                std::vector<IKREAL_TYPE> vfree(num_free_parameters);

                //Grab the position of the pose
                eetrans_[0] = pose.get_position()[0];
                eetrans_[1] = pose.get_position()[1];
                eetrans_[2] = pose.get_position()[2];

                //Grab the quaternion of the pose
                double qx = pose.get_orientation()[0];
                double qy = pose.get_orientation()[1];
                double qz = pose.get_orientation()[2];
                double qw = pose.get_orientation()[3];

                const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
                qw *= n;
                qx *= n;
                qy *= n;
                qz *= n;
                eerot_[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz*qz;
                eerot_[1] = 2.0f * qx * qy - 2.0f * qz*qw;
                eerot_[2] = 2.0f * qx * qz + 2.0f * qy*qw;
                eerot_[3] = 2.0f * qx * qy + 2.0f * qz*qw;
                eerot_[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz*qz;
                eerot_[5] = 2.0f * qy * qz - 2.0f * qx*qw;
                eerot_[6] = 2.0f * qx * qz - 2.0f * qy*qw;
                eerot_[7] = 2.0f * qy * qz + 2.0f * qx*qw;
                eerot_[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy*qy;

                transform_matrix_multiply(static_inv_tf_t, static_inv_tf_rot, eetrans_, eerot_, eetrans, eerot);

                double* fk = new double[7];
                Inverse_Transform_FK(eetrans, eerot);
                Display_FK(eetrans, eerot, 0, 0, 0, fk);

                IkReal* pfree = new IkReal[1];

                //Shouldn't this be our bounds on the arm?
                IkReal min_bounds[] = {-1.70167993878, -2.147, -3.05417993878, -0.05, -3.059, -1.57079632679, -3.059};
                IkReal max_bounds[] = {1.70167993878, 1.047, 3.05417993878, 2.618, 3.059, 2.094, 3.059};
                int counter = 0;

                bool bSuccess;
                do
                {
                    double sampling = 3.5 * rand() / RAND_MAX - 1.5;
                    pfree[0] = sampling;
                    bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);

                    if( bSuccess )
                    {
                        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

                        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
                        double min_distance = 1000000;
                        int min_index = 0;
                        bSuccess = false;
                        for( std::size_t i = 0; i < num_of_solutions; ++i )
                        {

                            double distance = 0;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            bool within_bounds = true;
                            for( int i = 0; i < 7 && within_bounds == true; ++i )
                            {
                                if( solvalues[i] <= min_bounds[i] || solvalues[i] >= max_bounds[i] )
                                    within_bounds = false;
                            }

                            if( within_bounds )
                            {
                                bSuccess = true;

                                for( std::size_t j = 0; j < solvalues.size(); ++j )
                                {
                                    distance += ((solvalues[j] - joints[j])*(solvalues[j] - joints[j]));
                                }
                                if( distance < min_distance )
                                {
                                    min_distance = distance;
                                    min_index = (int)(i);

                                }
                            }
                        }

                        if( bSuccess )
                        {
                            int i = min_index;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            for( std::size_t j = 0; j < solvalues.size(); ++j )
                            {
                                soln[j] = solvalues[j];
                            }
                        }
                    }
                    counter++;
                }
                while( !bSuccess && counter < 50 );

                delete fk;

                return bSuccess;
            }

            bool kinematic_model_t::min_ik_for_position(const config_t& pose, const std::vector< double >& joints, double* soln) const
            {
                IKREAL_TYPE eerot[9], eetrans[3], eerot_[9], eetrans_[3];

                unsigned int num_of_joints = GetNumJoints();
                unsigned int num_free_parameters = GetNumFreeParameters();

                IkSolutionList<IKREAL_TYPE> solutions;

                std::vector<IKREAL_TYPE> vfree(num_free_parameters);

                //Grab the position of the pose
                eetrans_[0] = pose.get_position()[0];
                eetrans_[1] = pose.get_position()[1];
                eetrans_[2] = pose.get_position()[2];

                //Grab the quaternion of the pose
                double qx = pose.get_orientation()[0];
                double qy = pose.get_orientation()[1];
                double qz = pose.get_orientation()[2];
                double qw = pose.get_orientation()[3];


                const double n = 1.0f / sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
                qw *= n;
                qx *= n;
                qy *= n;
                qz *= n;
                eerot_[0] = 1.0f - 2.0f * qy * qy - 2.0f * qz*qz;
                eerot_[1] = 2.0f * qx * qy - 2.0f * qz*qw;
                eerot_[2] = 2.0f * qx * qz + 2.0f * qy*qw;
                eerot_[3] = 2.0f * qx * qy + 2.0f * qz*qw;
                eerot_[4] = 1.0f - 2.0f * qx * qx - 2.0f * qz*qz;
                eerot_[5] = 2.0f * qy * qz - 2.0f * qx*qw;
                eerot_[6] = 2.0f * qx * qz - 2.0f * qy*qw;
                eerot_[7] = 2.0f * qy * qz + 2.0f * qx*qw;
                eerot_[8] = 1.0f - 2.0f * qx * qx - 2.0f * qy*qy;

                transform_matrix_multiply(static_inv_tf_t, static_inv_tf_rot, eetrans_, eerot_, eetrans, eerot);

                double* fk = new double[7];
                Inverse_Transform_FK(eetrans, eerot);
                Display_FK(eetrans, eerot, 0, 0, 0, fk);

                IkReal* pfree = new IkReal[1];

                IkReal min_bounds[] = {-1.70167993878, -2.147, -3.05417993878, -0.05, -3.059, -1.57079632679, -3.059};
                IkReal max_bounds[] = {1.70167993878, 1.047, 3.05417993878, 2.618, 3.059, 2.094, 3.059};

                int counter = 0;

                bool bSuccess;
                bool min_bSuccess = false;

                double min_pfree = -10;
                double min_distance_pfree = 1000000;

                counter = 0;
                do
                {
                    double sampling = joints[5]-(counter * 0.001);
                    pfree[0] = sampling;
                    bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);

                    if( bSuccess )
                    {
                        min_bSuccess = true;
                        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

                        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
                        double min_distance = 1000000;
                        int min_index = 0;
                        bSuccess = false;
                        for( std::size_t i = 0; i < num_of_solutions; ++i )
                        {
                            double distance = 0;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            bool within_bounds = true;
                            for( int i = 0; i < 7 && within_bounds == true; ++i )
                            {
                                if( solvalues[i] <= min_bounds[i] || solvalues[i] >= max_bounds[i] )
                                    within_bounds = false;
                            }

                            if( within_bounds )
                            {
                                bSuccess = true;

                                for( std::size_t j = 0; j < solvalues.size(); ++j )
                                {
                                    distance += ((solvalues[j] - joints[j])*(solvalues[j] - joints[j]));
                                }
                                if( distance < min_distance )
                                {
                                    min_distance = distance;
                                    min_index = (int)(i);

                                }
                                if( distance < min_distance_pfree )
                                {
                                    min_distance_pfree = distance;
                                    min_pfree = sampling;
                                    for( std::size_t j = 0; j < solvalues.size(); ++j )
                                    {
                                        soln[j] = solvalues[j];
                                    }
                                }
                            }
                        }
                    }

                    sampling = joints[5]+(counter * 0.001);
                    pfree[0] = sampling;
                    bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);

                    if( bSuccess )
                    {
                        min_bSuccess = true;
                        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

                        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
                        double min_distance = 1000000;
                        int min_index = 0;
                        bSuccess = false;
                        for( std::size_t i = 0; i < num_of_solutions; ++i )
                        {

                            double distance = 0;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            bool within_bounds = true;
                            for( int i = 0; i < 7 && within_bounds == true; ++i )
                            {
                                if( solvalues[i] <= min_bounds[i] || solvalues[i] >= max_bounds[i] )
                                    within_bounds = false;
                            }

                            if( within_bounds )
                            {
                                bSuccess = true;

                                for( std::size_t j = 0; j < solvalues.size(); ++j )
                                {
                                    distance += ((solvalues[j] - joints[j])*(solvalues[j] - joints[j]));
                                }
                                if( distance < min_distance )
                                {
                                    min_distance = distance;
                                    min_index = (int)(i);

                                }
                                if( distance < min_distance_pfree )
                                {
                                    min_distance_pfree = distance;
                                    min_pfree = sampling;
                                    for( std::size_t j = 0; j < solvalues.size(); ++j )
                                    {
                                        soln[j] = solvalues[j];
                                    }
                                }
                            }
                        }
                    }
                    counter++;
                }
                while( counter < 1000 );

                if( !min_bSuccess )
                {
                    for( int i = 0; i < 7; i++ )
                    {
                        soln[i] = -10;

                    }
                    delete fk;
                    return min_bSuccess;
                }



                do
                {
                    double sampling = -1.57 + (counter * 0.01);
                    pfree[0] = sampling;
                    bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);

                    if( bSuccess )
                    {
                        min_bSuccess = true;
                        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

                        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
                        double min_distance = 1000000;
                        int min_index = 0;
                        bSuccess = false;
                        for( std::size_t i = 0; i < num_of_solutions; ++i )
                        {

                            double distance = 0;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            bool within_bounds = true;
                            for( int i = 0; i < 7 && within_bounds == true; ++i )
                            {
                                if( solvalues[i] <= min_bounds[i] || solvalues[i] >= max_bounds[i] )
                                    within_bounds = false;
                            }

                            if( within_bounds )
                            {
                                bSuccess = true;

                                for( std::size_t j = 0; j < solvalues.size(); ++j )
                                {
                                    distance += ((solvalues[j] - joints[j])*(solvalues[j] - joints[j]));
                                }
                                if( distance < min_distance )
                                {
                                    min_distance = distance;
                                    min_index = (int)(i);

                                }
                                if( distance < min_distance_pfree )
                                {
                                    min_distance_pfree = distance;
                                    min_pfree = sampling;
                                    for( std::size_t j = 0; j < solvalues.size(); ++j )
                                    {
                                        soln[j] = solvalues[j];
                                    }
                                }
                            }
                        }
                    }
                    counter++;
                }
                while( counter < 358 );

                counter = 0;
                do
                {
                    double sampling = joints[5]-(counter * 0.0001);
                    pfree[0] = sampling;
                    bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);

                    if( bSuccess )
                    {
                        min_bSuccess = true;
                        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

                        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
                        double min_distance = 1000000;
                        int min_index = 0;
                        bSuccess = false;
                        for( std::size_t i = 0; i < num_of_solutions; ++i )
                        {

                            double distance = 0;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            bool within_bounds = true;
                            for( int i = 0; i < 7 && within_bounds == true; ++i )
                            {
                                if( solvalues[i] <= min_bounds[i] || solvalues[i] >= max_bounds[i] )
                                    within_bounds = false;
                            }

                            if( within_bounds )
                            {
                                bSuccess = true;

                                for( std::size_t j = 0; j < solvalues.size(); ++j )
                                {
                                    distance += ((solvalues[j] - joints[j])*(solvalues[j] - joints[j]));
                                }
                                if( distance < min_distance )
                                {
                                    min_distance = distance;
                                    min_index = (int)(i);

                                }
                                if( distance < min_distance_pfree )
                                {
                                    min_distance_pfree = distance;
                                    min_pfree = sampling;
                                    for( std::size_t j = 0; j < solvalues.size(); ++j )
                                    {
                                        soln[j] = solvalues[j];
                                    }
                                }
                            }
                        }
                    }
                    counter++;
                }
                while( counter < 100 );

                counter = 0;
                do
                {
                    double sampling = joints[5]+(counter * 0.0001);
                    pfree[0] = sampling;
                    bSuccess = ComputeIk(eetrans, eerot, pfree, solutions);

                    if( bSuccess )
                    {
                        min_bSuccess = true;
                        unsigned int num_of_solutions = (int)solutions.GetNumSolutions();

                        std::vector<IKREAL_TYPE> solvalues(num_of_joints);
                        double min_distance = 1000000;
                        int min_index = 0;
                        bSuccess = false;
                        for( std::size_t i = 0; i < num_of_solutions; ++i )
                        {

                            double distance = 0;
                            const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                            int this_sol_free_params = (int)sol.GetFree().size();

                            std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
                            sol.GetSolution(&solvalues[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);

                            bool within_bounds = true;
                            for( int i = 0; i < 7 && within_bounds == true; ++i )
                            {
                                if( solvalues[i] <= min_bounds[i] || solvalues[i] >= max_bounds[i] )
                                    within_bounds = false;
                            }

                            if( within_bounds )
                            {
                                bSuccess = true;

                                for( std::size_t j = 0; j < solvalues.size(); ++j )
                                {
                                    distance += ((solvalues[j] - joints[j])*(solvalues[j] - joints[j]));
                                }
                                if( distance < min_distance )
                                {
                                    min_distance = distance;
                                    min_index = (int)(i);

                                }

                                if( distance < min_distance_pfree )
                                {
                                    min_distance_pfree = distance;
                                    min_pfree = sampling;

                                    for( std::size_t j = 0; j < solvalues.size(); ++j )
                                    {
                                        soln[j] = solvalues[j];
                                    }
                                }
                            }
                        }

                    }
                    counter++;
                }
                while( counter < 100 );

                delete fk;
                return min_bSuccess;


            }

            void kinematic_model_t::transform_matrix_multiply(double* t1, double* r1, double* t2, double* r2, double* res_t, double* res_rot) const
            {
                int mstart, estart;

                for( int i = 0; i < 9; ++i )
                {
                    mstart = floor(i / 3)*3;
                    estart = i % 3;
                    res_rot[i] = r1[mstart] * r2[estart] + r1[mstart + 1] * r2[estart + 3] + r1[mstart + 2] * r2[estart + 6];
                }

                res_t[0] = r1[0] * t2[0] + r1[1] * t2[1] + r1[2] * t2[2] + t1[0];
                res_t[1] = r1[3] * t2[0] + r1[4] * t2[1] + r1[5] * t2[2] + t1[1];
                res_t[2] = r1[6] * t2[0] + r1[7] * t2[1] + r1[8] * t2[2] + t1[2];
            }

            void kinematic_model_t::Transform_FK(IkReal* t, IkReal* e) const
            {
                IkReal* mount;
                IkReal* res_t = new IkReal[3];
                IkReal* res = new IkReal[9];

                IkReal x, y, z, t0, t1, t2;

                if(hand==LEFT)
                {
                    x = IkReal(0.024093114373300);
                    y = IkReal(0.219974876072000);
                    z = IkReal(0.108827793167000);
                    IkReal mount2[] = {0.709794066175522, -0.704408539497704, -0.000996546401591, 0.704405199486540, 0.709784485664986, 0.004393045141974, -0.002387165337311, -0.003820129841025, 0.999989853973354};
                    mount=mount2;
                }
                else if(hand==RIGHT)
                {
                    x = IkReal(0.025333750040600);
                    y = IkReal(-0.219561017038000);
                    z = IkReal(0.107942754913000);
                    IkReal mount2[] = {0.706823930392534, 0.707342413920389, 0.008163387392936, -0.707217241158897, 0.706859409151833, -0.013912206910484, -0.015611061208269, 0.004060192458359, 0.999869896339095};
                    mount=mount2;
                }
                int mstart, estart;

                for( int i = 0; i < 9; ++i )
                {
                    mstart = floor(i / 3)*3;
                    estart = i % 3;
                    res[i] = mount[mstart] * e[estart] + mount[mstart + 1] * e[estart + 3] + mount[mstart + 2] * e[estart + 6];
                }

                for( int i = 0; i < 9; ++i )
                {
                    e[i] = res[i];
                }

                t0 = mount[0] * t[0] + mount[1] * t[1] + mount[2] * t[2] + x;
                t1 = mount[3] * t[0] + mount[4] * t[1] + mount[5] * t[2] + y;
                t2 = mount[6] * t[0] + mount[7] * t[1] + mount[8] * t[2] + z;
                t[0] = t0;
                t[1] = t1;
                t[2] = t2;

                transform_matrix_multiply(static_tf_t, static_tf_rot, t, e, res_t, res);

                for( int i = 0; i < 9; ++i )
                {
                    e[i] = res[i];
                }

                t[0]=res_t[0];
                t[1]=res_t[1];
                t[2]=res_t[2];

                delete res;
                delete res_t;
            }

            void kinematic_model_t::Inverse_Transform_FK(IkReal* t, IkReal* e) const
            {
                IkReal* mount;
                IkReal x, y, z, t0, t1, t2;
                if( hand == LEFT )
                {
                    x = IkReal(-0.171792806143796);
                    y = IkReal(-0.138747622464126);
                    z = IkReal(-0.109769038651563);

                    IkReal mount2[] = {0.709794066175521, 0.704405199486539, -0.002387165337311, -0.704408539497704, 0.709784485664987, -0.003820129841025, -0.000996546401591, 0.004393045141974, 0.999989853973354};
                    mount = mount2;
                }
                else if( hand == RIGHT )
                {
                    x = IkReal(-0.171498736556999);
                    y = IkReal(0.136840866509450);
                    z = IkReal(-0.111190098679624);
                    IkReal mount2[] = { 0.706823930392534,   -0.707217241158897,  -0.0156110612082700,
                                        0.707342413920389,   0.706859409151833,   0.00406019245835896,
                                        0.00816338739293532, -0.0139122069104834, 0.999869896339095 };
                    mount = mount2;
                }

                int mstart, estart;
                IkReal* res = new IkReal[9];

                for( int i = 0; i < 9; ++i )
                {
                    mstart = floor(i / 3)*3;
                    estart = i % 3;
                    res[i] = mount[mstart] * e[estart] + mount[mstart + 1] * e[estart + 3] + mount[mstart + 2] * e[estart + 6];
                }

                for( int i = 0; i < 9; ++i )
                {
                    e[i] = res[i];
                }

                t0 = mount[0] * t[0] + mount[1] * t[1] + mount[2] * t[2] + x;
                t1 = mount[3] * t[0] + mount[4] * t[1] + mount[5] * t[2] + y;
                t2 = mount[6] * t[0] + mount[7] * t[1] + mount[8] * t[2] + z;
                t[0] = t0;
                t[1] = t1;
                t[2] = t2;
                delete res;
            }

            void kinematic_model_t::fk_for_link(std::string link, double* joints_, double gripper_, double* fk_state, double x, double y, double z) const
            {
                unsigned int num_of_joints = GetNumJoints();
                IKREAL_TYPE joints[num_of_joints];
                IKREAL_TYPE eerot[9], eetrans[3];

                for( unsigned int i = 0; i < num_of_joints; i++ )
                {
                    joints[i] = joints_[i];
                }

                if( link == "left_gripper" )
                {
                    ComputeW2Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0.13855, fk_state);
                }
                else if( link == "left_gripper_left_finger" )
                {
                    ComputeW2Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    if( gripper_ == 0 )
                        Display_FK(eetrans, eerot, 0, 0.044, 0.20105, fk_state);
                    else
                        Display_FK(eetrans, eerot, 0, 0.04, 0.20105, fk_state);
                        // Display_FK(eetrans, eerot, 0, 0.034, 0.20105, fk_state);
                }
                else if( link == "left_gripper_right_finger" )
                {
                    ComputeW2Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    if( gripper_ == 0 )
                        Display_FK(eetrans, eerot, 0, -0.044, 0.20105, fk_state);
                    else
                        Display_FK(eetrans, eerot, 0, -0.04, 0.20105, fk_state);
                        // Display_FK(eetrans, eerot, 0, -0.034, 0.20105, fk_state);
                }
                else if( link == "left_upper_shoulder" )
                {
                    ComputeS0Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0.1361, fk_state);
                }
                else if( link == "left_lower_shoulder" )
                {
                    ComputeS1Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0, fk_state);
                }
                else if( link == "left_upper_elbow" )
                {
                    ComputeE0Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, .08, fk_state);
                }
                else if( link == "left_lower_elbow" )
                {
                    ComputeE1Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0, fk_state);
                }
                else if( link == "left_upper_forearm" )
                {
                    ComputeW0Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, .08, fk_state);
                }
                else if( link == "left_lower_forearm" )
                {
                    ComputeW1Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0, fk_state);
                }
                else if( link == "left_wrist" )
                {
                    ComputeW2Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, -0.010, fk_state);
                }
                else if( link == "left_hand" )
                {
                    ComputeW2Fk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0.08605, fk_state);
                }
                else if( link == "torso" )
                {
                    fk_state[0] = 0;
                    fk_state[1] = 0;
                    fk_state[2] = 0;
                    fk_state[3] = 0;
                    fk_state[4] = 0;
                    fk_state[5] = 0;
                    fk_state[6] = 1;

                }
                else if( link == "pedestal" )
                {
                    fk_state[0] = -0.04;
                    fk_state[1] = 0;
                    fk_state[2] = -0.7750;
                    fk_state[3] = 0;
                    fk_state[4] = 0;
                    fk_state[5] = 0;
                    fk_state[6] = 1;
                }
                else if( link == "ee" )
                {
                    ComputeFk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0, fk_state);
                }
                else if( link == "end_effector" )
                {
                    ComputeFk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, 0, 0, 0.05, fk_state);
                }
                else if( link == "offset" )
                {
                    ComputeFk(joints, eetrans, eerot);
                    Transform_FK(eetrans, eerot);
                    Display_FK(eetrans, eerot, x, y, z, fk_state);
                }


            }

            float kinematic_model_t::SIGN(float x) const
            {
                return (x >= 0.0f) ? +1.0f : -1.0f;
            }

            float kinematic_model_t::NORM(float a, float b, float c, float d) const
            {
                return sqrt(a * a + b * b + c * c + d * d);
            }
        }
    }
}
