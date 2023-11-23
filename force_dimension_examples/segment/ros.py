""" A ROS2 interface for interacting with the Force Dimension ROS2 node.
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

# Create a ROS2 node to act as a client for the Force Dimension node.
def initialize_node():
    
    # Initialize a ROS2 node.
    from rclpy.node import Node
    node = Node('segment', namespace='robot')
    
    # Declare parameter: Guidance spring stiffness in [N/m]
    node.declare_parameter('spring.stiffness', 2000.0)
    
    # Declare parameter: Guidance spring damping in [N/(m/s)]
    node.declare_parameter('spring.damping', 20.0)
    
    # Set the default state.
    node.state = dict(position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0), 
                      button=0, console='')
    
    # Define shorthand for translating ROS2 messages into state updates.
    subscribe = node.create_subscription
    message_to_xyz = lambda m: (m.x, m.y, m.z)
    message_to_scalar = lambda m: m.data
    update_state = lambda k,v: node.state.update({k: v})
    
    # Subscribe to receive position feedback.
    subscribe(msg_type=position_msg, 
              topic='feedback/position', 
              callback=lambda m: update_state('position', message_to_xyz(m)),
              qos_profile=qos_default)
    
    # Subscribe to receive velocity feedback.
    subscribe(msg_type=velocity_msg, 
              topic='feedback/velocity', 
              callback=lambda m: update_state('velocity', message_to_xyz(m)),
              qos_profile=qos_default)
    
    # Subscribe to receive button feedback.
    subscribe(msg_type=button_msg, 
              topic='feedback/button', 
              callback=lambda m: update_state('button', message_to_scalar(m)),
              qos_profile=qos_default)
    
    # Subscribe to receive console feedback.
    subscribe(msg_type=console_msg, 
              topic='feedback/console', 
              callback=lambda m: update_state('console', message_to_scalar(m)),
              qos_profile=qos_default)
    
    # Initialize a publisher for force commands.
    node.force_publisher = node.create_publisher(msg_type=force_msg,
                                                 topic='command/force',
                                                 qos_profile=qos_default)
    
    return node
    
  
