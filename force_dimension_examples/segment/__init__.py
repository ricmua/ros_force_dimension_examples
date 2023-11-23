""" This example illustrates how to constrain a Force Dimension haptic device 
    on an arbitrary segment defined using the user button on the haptic device 
    end-effector.


The constraint force model parameters are defined and documented in the haptic 
loop and can be adjusted to modify the behavior of the application.
"""

# Derived from segment.cpp in the Force Dimension SDK.
# Original source code Copyright (C) 2001-2023 Force Dimension, Switzerland.
# All Rights Reserved.
#
# Force Dimension SDK 3.16.1
# https://www.forcedimension.com/software/sdk

# Adapted to Python and ROS2 by a.whit (nml@whit.contact)
# Carnegie Mellon University Neuromechatronics Lab
# https://www.meche.engineering.cmu.edu/faculty/neuromechatronics-lab.html


# Imports.
from force_dimension_examples.segment import ros
from force_dimension_examples.segment import app


# Entry point.
def entry_point(args=None):
    """
    """
    
    # Initialize the ROS2 interface.
    import rclpy
    rclpy.init()
    
    # Create a ROS2 node.
    node = ros.initialize_node()
    
    # Override the implementation of the segment application, to ensure that 
    # parameter values are provided by the ROS2 node, and that applied forces 
    # are published by the ROS2 node.
    class App(app.SegmentApp):
        
        def apply_force(self, x=0.0, y=0.0, z=0.0):
            message = ros.force_msg(x=x, y=y, z=z)
            node.force_publisher.publish(message)
            
        def get_parameter(self, key): return node.get_parameter(key).value
    
    # Map logging functionality to the ROS2 log.
    log = node.get_logger().info
    
    # Create an instance of the segment application.
    application = App(state=node.state, log=log)
    
    # In order to update the state of the system at a regular interval, create 
    # another subscription to robot position messages. The application update 
    # function is invoked each time the (high-frequency) position messages are 
    # received.
    node.create_subscription(msg_type=ros.position_msg, 
                             topic='feedback/position',
                             callback=application.update, 
                             qos_profile=ros.qos_default)
    
    # Pass control to the ROS2 system until the node is terminated.
    try: rclpy.spin(node)
    except KeyboardInterrupt as e: pass
    #finally: rclpy.shutdown()
    


