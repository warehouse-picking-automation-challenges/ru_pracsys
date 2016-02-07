#
# Copyright (c) 2014, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
##############################################################################
# Description
##############################################################################

"""
.. module:: lazy_mat2image_publisher
   :synopsis: Converts and publishes cv images in a lazy way..

Combines several cells that ensure no work is done in converting/publishing
if there are no subscribers.
----

"""

##############################################################################
# Imports
##############################################################################

import ecto
import ecto_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

##############################################################################
# Blackbox
##############################################################################


class LazyMat2ImagePublisher(ecto.BlackBox):
    """
    Encapsulates the cells necessary to transfer a bag's image pair output
    on publishers into our eco-system through to a format usable by
    the dslam beachhead.
    """
    @staticmethod
    def declare_direct_params(_p):
        """
        Configure non-forwarded, blackbox specific params here. These get
        passed to the ensuing declare_xxx calls below.
        """
        pass

    @staticmethod
    def declare_cells(_p):
        """
        Implement the virtual function from the base class
        Only cells from which something is forwarded have to be declared
        """
        cells = {}
        cells['throttle'] = ecto.TrueEveryN("Throttle", n=1)
        cells['mat2image'] = ecto_ros.Mat2Image("Mat2Image")
        cells['subscribed_mat2image'] = ecto.If("Subscribed Mat2Image", input_tendril_name="has_subscribers", cell=cells['mat2image'])
        cells['throttled_mat2image'] = ecto.If("Throttled Mat2Image", input_tendril_name="throttle", cell=cells['subscribed_mat2image'])
        cells['publisher'] = ecto_sensor_msgs.Publisher_Image("Publisher", topic_name="/images/candidate_beans", queue_size=2, latched=True)
        cells['throttled_publisher'] = ecto.If("Throttled Publisher", input_tendril_name="throttle", cell=cells['publisher'])
        subscribers_input, subscribers_output = \
            ecto.EntangledPair(value=cells['throttled_mat2image'].inputs.at('has_subscribers'), source_name="Has Subscribers Input", sink_name="Has Subscribers Output")
        cells['subscribers_input'] = subscribers_input
        cells['subscribers_output'] = subscribers_output
        return cells

    @staticmethod
    def declare_forwards(_p):
        """
        Implement the virtual function from the base class
        """
        p = {
             'throttle': [Forward('n', new_key='every_n')],
             'mat2image': 'all',
             'publisher': 'all',
            }

        i = {
             'mat2image': 'all'
            }

        o = {
             'throttled_publisher': 'all'  # has_subscribers
            }

        return (p, i, o)

    def connections(self, _p):
        return [
            self.subscribers_input[:] >> self.throttled_mat2image['has_subscribers'],
            self.throttle['flag'] >> self.throttled_mat2image['throttle'],
            self.throttled_mat2image['image'] >> self.throttled_publisher['input'],
            self.throttle['flag'] >> self.throttled_publisher['throttle'],
            self.throttled_publisher['has_subscribers'] >> self.subscribers_output[:],
        ]
