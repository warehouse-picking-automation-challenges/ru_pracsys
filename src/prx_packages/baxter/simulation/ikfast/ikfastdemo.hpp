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

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

/*
Set which IKFast version you are using
The API calls are slightly different for versions > 54
*/

#define IK_VERSION 61
#include "baxter_left.hpp"

//#define IK_VERSION 56
//#include "ikfast56.Transform6D.0_1_2_3_4_5.cpp"

//#define IK_VERSION 54
//#include "output_ikfast54.cpp"

//----------------------------------------------------------------------------//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include<math.h>
#include <prx/utilities/definitions/defs.hpp>

//How about we put this class in a namespace, eh?

namespace prx
{
    namespace util
    {
        class config_t;
    }

    namespace packages
    {
        namespace baxter
        {
            class kinematic_model_t
            {
              public:
                kinematic_model_t();
                ~kinematic_model_t();
                enum hand_state{LEFT,RIGHT};
                float SIGN(float x) const;
                float NORM(float a, float b, float c, float d) const;
                void Transform_FK(IkReal* e, IkReal* t) const;
                void Inverse_Transform_FK(IkReal* e, IkReal* t) const;
                void Display_FK(IkReal* eetrans,IkReal* eerot, double x, double y, double z, double* fk_state) const;
                void fk_for_link(std::string link, double* joints_, double gripper_, double* fk_state, double x=0, double y=0, double z=0) const;
                bool ik_for_position( const util::config_t& pose, const std::vector< double >& joints, double* solution) const;
                bool min_ik_for_position( const util::config_t& pose, const std::vector< double >& joints, double* soln) const;
                #if IK_VERSION > 54
                #define IKREAL_TYPE IkReal // for IKFast 56,61
                #else
                #define IKREAL_TYPE IKReal // for IKFast 54
                #endif

                hand_state hand;
                double* static_tf_t;
                double* static_tf_rot;
                double* static_inv_tf_t;
                double* static_inv_tf_rot;
                void set_hand_state(hand_state hs)
                {
                    hand=hs;
                }
                void set_static_transform(double x, double y, double z, double* rot)
                {
                    static_tf_t[0]=x;
                    static_tf_t[1]=y;
                    static_tf_t[2]=z;
                    static_tf_rot[0] = rot[0];
                    static_tf_rot[1] = rot[1];
                    static_tf_rot[2] = rot[2];
                    static_tf_rot[3] = rot[3];
                    static_tf_rot[4] = rot[4];
                    static_tf_rot[5] = rot[5];
                    static_tf_rot[6] = rot[6];
                    static_tf_rot[7] = rot[7];
                    static_tf_rot[8] = rot[8];
                    PRX_PRINT("SETTER tf translation "<<static_tf_t[0]<<", "<<static_tf_t[1]<<", "<<static_tf_t[2],PRX_TEXT_LIGHTGRAY);
                    PRX_PRINT("SETTER tf rotation "<<static_tf_rot[0]<<", "<<static_tf_rot[1]<<", "<<static_tf_rot[2]<<", "<<static_tf_rot[3]<<", "<<static_tf_rot[4]<<", "<<static_tf_rot[5]<<", "<<static_tf_rot[6]<<", "<<static_tf_rot[7]<<", "<<static_tf_rot[8],PRX_TEXT_LIGHTGRAY);

                }

                void set_static_inverse_transform(double x, double y, double z, double* rot)
                {
                    static_inv_tf_t[0]=x;
                    static_inv_tf_t[1]=y;
                    static_inv_tf_t[2]=z;
                    static_inv_tf_rot[0] = rot[0];
                    static_inv_tf_rot[1] = rot[1];
                    static_inv_tf_rot[2] = rot[2];
                    static_inv_tf_rot[3] = rot[3];
                    static_inv_tf_rot[4] = rot[4];
                    static_inv_tf_rot[5] = rot[5];
                    static_inv_tf_rot[6] = rot[6];
                    static_inv_tf_rot[7] = rot[7];
                    static_inv_tf_rot[8] = rot[8];
                    PRX_PRINT("SETTER inv tf translation "<<static_inv_tf_t[0]<<", "<<static_inv_tf_t[1]<<", "<<static_inv_tf_t[2],PRX_TEXT_LIGHTGRAY);
                    PRX_PRINT("SETTER inv tf rotation "<<static_inv_tf_rot[0]<<", "<<static_inv_tf_rot[1]<<", "<<static_inv_tf_rot[2]<<", "<<static_inv_tf_rot[3]<<", "<<static_inv_tf_rot[4]<<", "<<static_inv_tf_rot[5]<<", "<<static_inv_tf_rot[6]<<", "<<static_inv_tf_rot[7]<<", "<<static_inv_tf_rot[8],PRX_TEXT_LIGHTGRAY);

                }
                void transform_matrix_multiply(double* t1, double* r1, double* t2, double* r2, double* res_t, double* res_rot) const;
            };
        }
    }
}


