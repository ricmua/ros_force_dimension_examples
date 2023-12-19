""" Graphical user interface (GUI) for interacting with the `segment` example 
    ROS2 node.
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
from rclpy.node import Node
from force_dimension_examples.segment import console_msg
from force_dimension_examples.segment import qos_default


# ROS2 node and tkinter GUI for relaying keyboard events to the example node.
class Window(Node):
    """ A ROS2 node and tkinter GUI for relaying keyboard events to the 
        `segment` example node for Force Dimension robots.
    """
    
    def __init__(self, *args, node_name='gui', namespace='robot', **kwargs):
        """ Constructor """
        
        # Invoke the superclass constructor.
        super().__init__(*args, node_name=node_name, 
                                namespace=namespace, **kwargs)
        
        # Create a publisher for sending console messages to the example node.
        self.console_publisher \
            = self.create_publisher(msg_type=console_msg,
                                    topic='feedback/console',
                                    qos_profile=qos_default)
        
        # Create a tkinter instance.
        self.root = tkinter.Tk()
        
        # Add the instructions to the GUI window.
        lines = ['Press "p" to define the segment extremities',
                 'Press "c" to clear the current segment',
                 'Press "q" to quit']
        self.text = tkinter.Label(master=self.root, text='\n'.join(lines))
        self.text.pack()
        
        # Bind key press events to the local callback.
        self.root.bind('<KeyPress>', self.on_key_press)
        
        # Pass control to the GUI main loop.
        # Alternatively, `self.root.update` might be called from a callback.
        self.root.mainloop()
        
    def log(self, message): self.get_logger().info(message)
    
    def on_key_press(self, event):
        message = console_msg(data=event.char)
        self.log(f'Key press: {message.data}')
        self.console_publisher.publish(message=message)
        if message.data == 'q':
            self.log('Terminating')
            self.root.destroy()
            exit()
    
  


