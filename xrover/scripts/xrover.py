#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Tope Akinsipe.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## xRover node


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf import transformations as trans
from decimal import Decimal
import serial
import struct

# actual measured values
DIST_COVERED_PER_TICK = 0.635 / 94  # total_circ / numb_ticks
WHEEL_RADIUS = 0.10106
WHEEL_BASE = 0.494

SR_DATA_SZ = 12
US_TO_S = 1000000
# at a tick spacing above this, consider the motor stopped
MOTOR_STOP_TICK_SPA = 50000


class XRover(object):
    def __init__(self):
        self.speed_l = 0
        self.speed_r = 0
        self.v1_dir = 1
        self.v2_dir = 1
        self.tick_sp_1 = 0
        self.tick_sp_2 = 0

        # serial port to wheels
        self.port = serial.Serial('/dev/ttyACM1', 115200, timeout=0.5)

    def cmd_vel_callback(self, data):
        vx = data.linear.x
        vth = data.angular.z

        # calculate the linear velocity required by each wheel
        v2 = ((2*vx) - (vth*WHEEL_BASE)) / 2
        v1 = (2*vx) - v2

        self.v2_dir = 1 if v2 >= 0 else 0
        self.v1_dir = 1 if v1 >= 0 else 0

        # Set the tick spacing. If the velocity is zero, don't try to calculate tick spacing
        self.tick_sp_1 = 0 if v1 == 0 else round(abs((DIST_COVERED_PER_TICK * US_TO_S)/v1))
        self.tick_sp_2 = 0 if v2 == 0 else round(abs((DIST_COVERED_PER_TICK * US_TO_S)/v2))

        self.tick_sp_1 = self.tick_sp_1 if self.tick_sp_1 < MOTOR_STOP_TICK_SPA else 0
        self.tick_sp_2 = self.tick_sp_2 if self.tick_sp_2 < MOTOR_STOP_TICK_SPA else 0
        rospy.loginfo(rospy.get_caller_id() + ' sent to wheels v1_dir %d, v2_dir %d, tick_spa1 %d, tick_spa2 %d ',
                      self.v1_dir, self.v2_dir, self.tick_sp_1, self.tick_sp_2)

        # send the speed converted to time between ticks
        self.port.write(struct.pack('>HHII', self.v1_dir, self.v2_dir, self.tick_sp_1, self.tick_sp_2))

    def get_motor_speed(self):
        while 1:
            # read the data on space between ticks
            serial_data = self.port.read(SR_DATA_SZ)
            if serial_data:
                data = struct.unpack('>HHII', serial_data)
                rospy.loginfo('cmd1 %d, cmd2 %d, tick_spa1 %d, tick_spa2 %d', data[0], data[1], data[2], data[3])
                #print data
                self.speed_l = DIST_COVERED_PER_TICK / (float(data[2]) / US_TO_S)
                self.speed_r = DIST_COVERED_PER_TICK / (float(data[3]) / US_TO_S)

                # for now assume it is going in the direction we asked it to go
                self.speed_l = self.speed_l if self.v1_dir else (-1 * self.speed_l)
                self.speed_r = self.speed_r if self.v2_dir else (-1 * self.speed_r)

                xrover_x_speed = (self.speed_l + self.speed_r) / 2
                xrover_theta_speed = (self.speed_l - self.speed_r) / WHEEL_BASE

                # TODO: estimate dist
                # xrover_x_dist = (self.dist_l + self.dist_r) / 2
                # xrover_theta_dist = (self.dist_l - self.dist_r) / WHEEL_BASE

                rospy.loginfo(rospy.get_caller_id() + " x_speed %f theta_speed %f", xrover_x_speed, xrover_theta_speed)


def start():
    rospy.init_node('xrover')
    xrover = XRover()
    rospy.Subscriber("/cmd_vel_mux/input/teleop", Twist, xrover.cmd_vel_callback)
    xrover.get_motor_speed()

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


if __name__ == '__main__':
    start()
