#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Printing xx status: ", data.data)

def limo_status_client_node():
    
    get_status = 0
    pub = rospy.Publisher('request', String, queue_size=10)
    rospy.init_node('limo_status_client_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz every 1 second request new status
    
    #rospy.Subscriber('response', String, callback)
    
    while not rospy.is_shutdown():
        #send get_status request 0 to 4
        if (get_status == 0): # /limo_status/vehicle_status
            print("Sending get_status value: {}" .format(get_status))
            rospy.loginfo("0") #needs to be string
            pub.publish("0")

        elif (get_status == 1): # /limo_status/control_mode
            print("Sending get_status value: {}" .format(get_status))
            rospy.loginfo("1")
            pub.publish("1")

        elif (get_status == 2): # /limo_status/battery_voltage
            print("Sending get_status value: {}" .format(get_status))
            rospy.loginfo("2")
            pub.publish("2")

        elif (get_status == 3): # /limo_status/error_code
            print("Sending get_status value: {}" .format(get_status))
            rospy.loginfo("3")
            pub.publish("3")

        elif (get_status == 4): # /limo_status/motion_mode
            print("Sending get_status value: {}" .format(get_status))
            rospy.loginfo("4")
            pub.publish("4")
            
        rospy.Subscriber('response', String, callback)

        get_status += 1 
        if get_status == 5:
            get_status = 0

        rate.sleep()


if __name__ == '__main__':
    try:
        limo_status_client_node()
    except rospy.ROSInterruptException:
        pass
    