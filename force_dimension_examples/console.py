""" Graphical user interface (GUI) for translating key strikes into published 
    ROS2 messages, for the purpose of interacting with Force Dimension robot 
    examples.
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


# Imports.
import tkinter
import rclpy
from rclpy.node import Node
from force_dimension_examples import console_msg
from force_dimension_examples import qos_default


# Entry point.
def entry_point(args=None):
    """ Entry point for a ROS2 console input node. """
    
    # Initialize the ROS2 interface.
    rclpy.init()
    
    # Create a ROS2 node.
    node = Node('console', namespace='robot')
    
    # Initialize a logger.
    log = lambda m: node.get_logger().info(m)
    
    # Create a publisher for publishing key strikes to the ROS2 graph.
    publisher = node.create_publisher(msg_type=console_msg,
                                      topic='feedback/console',
                                      qos_profile=qos_default)
    
    # Create a tkinter instance.
    tkroot = tkinter.Tk()
    
    # Add the instructions to the GUI window.
    lines = ['Key strikes are published to the ROS2 graph ', 
             'via the /robot/feedback/console topic.'
             'Press "q", or close the window, to quit']
    text = tkinter.Label(master=tkroot, text='\n'.join(lines))
    text.pack()
    
    # Define a callback for key presses.
    def on_key_press(event):
        
        # Log the event.
        log(f'Key press: {event.char}')
        
        # Publish a message.
        message = console_msg(data=event.char)
        publisher.publish(message)
        
        # Pass control to the ROS2 executor for processing.
        rclpy.spin_once(node, timeout_sec=0.005)
        
        # Terminate, if requested.
        if message.data == 'q':
            log('Terminating')
            tkroot.destroy()
            exit()
    
    # Bind key press events to the local callback.
    tkroot.bind('<KeyPress>', on_key_press)
    
    # Pass control to the GUI main loop.
    # Alternatively, `self.root.update` might be called from a callback.
    tkroot.mainloop()
    
    # Clean up.
    rclpy.shutdown()
    
  

