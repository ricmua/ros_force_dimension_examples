""" ROS2 implementations of the examples provided by the Force Dimension 
    haptics and robotics SDK 
"""

# Copyright 2023 Carnegie Mellon Neuromechatronics Lab
# 
# https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html
#
# Contact: a.whit nml@whit.contact
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.


# Import message types.
from geometry_msgs.msg      import Point   as position_msg
from geometry_msgs.msg      import Vector3 as velocity_msg
from geometry_msgs.msg      import Vector3 as force_msg
from example_interfaces.msg import Int32   as button_msg
from example_interfaces.msg import String  as console_msg


# Define a default Quality-of-Service profile for the local ROS2 system.
qos_default = history_depth = 10
