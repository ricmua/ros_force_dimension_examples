""" This example illustrates how to constrain a Force Dimension haptic device 
    on an arbitrary segment defined using the user button on the haptic device 
    end-effector. This module closely follows the exact implementation found in 
    the Force Dimension SDK.

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


#
def projectPointOnSegment(point, A, B):
    """ This function computes the projection of a `point` onto a segment 
        defined by two points `A` and `B`.
    """
    
    # Compute Ap vector.
    Ap = (point_n - A_n for (point_n, A_n) in zip(point, A))
    
    # Compute AB segment vector.
    AB = (B_n - A_n for (B_n, A_n) in zip(B, A))
    
    # Compute the segment norm, and return the current input point if it is too 
    # small.
    from math import sqrt
    norm = sqrt(sum(AB_n**2 for AB_n in AB))
    if norm <= 1e-6: return point

    # Compute the segment direction unit vector.
    direction = (AB_n / norm for AB_n in AB)
    
    # Compute the projection ratio.
    projectionRatio = sum(Ap_n * direction_n 
                          for (AP_n, direction_n) in zip(AP, direction))
    
    # Compute the point projection on the segment.
    if projectionRatio < 0.0: projection = A
    elif projectionRatio > norm: projection = B
    else: projection = (A_n + projectionRatio * direction_n 
                        for (A_n, direction_n) in zip(A, direction))
    
    # Return the result.
    return projection
    
  

















def projectForceOnDirection(force, A, B):
    """ This function computes the projection of a `force` onto a direction 
        defined by two points `A` and `B`.
    """
    
    # Compute AB segment vector.
    AB = numpy.asarray(B) - A
    
    # Compute the segment norm, and return the current input point if it is too 
    # small.
    norm = numpy.linalg.norm(AB)
    if norm <= 1e-6: return force
    
    # Compute the segment direction unit vector.
    direction = AB / norm
    
    # Compute the projection ratio.
    projectionRatio = force @ direction
    
    # Compute the force projection on the segment.
    projectedForce = projectionRatio * direction
    
    # Return the result.
    return projectedForce
    
  

#from collections import deque
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from example_interfaces.msg import Int32

def entry_point(args=None):
    """
    """
    
    # Initialize the ROS2 interface.
    rclpy.init()
    
    # Initialize a ROS2 node.
    node = Node('segment', namespace='robot')
    
    # Set up the logging functionality.
    log = node.get_logger().info
    
    # Display version information.
    log('Segment Constraint Example 3.16.1')
    log('Copyright (C) 2001-2023 Force Dimension')
    log('All Rights Reserved.')
    log('Translated to Python and ROS2 by a.whit')
    log('(Neuromechatronics Lab, Carnegie Mellon University')
    
    # Declare parameter: Guidance spring stiffness in [N/m]
    node.declare_parameter('spring.stiffness', 2000.0)
    
    # Declare parameter: Guidance spring damping in [N/(m/s)]
    node.declare_parameter('spring.damping', 20.0)
    
    ## Declare parameter: line segment endpoints
    #node.declare_parameter('segment.points', [])
    
    # Subscribe to receive position, velocity, and button feedback.
    state = dict(position=Point(), velocity=Vector3(), button=Int32())
    update_state_callback = lambda t: lambda m: state.update({t: m})
    history_depth = 10
    node.create_subscription(msg_type=Point, 
                             topic='feedback/position', 
                             callback=update_state_callback('position'),
                             qos_profile=history_depth)
    node.create_subscription(msg_type=Vector3, 
                             topic='feedback/velocity', 
                             callback=update_state_callback('velocity'),
                             qos_profile=history_depth)
    node.create_subscription(msg_type=Int32, 
                             topic='feedback/button', 
                             callback=update_state_callback('button'),
                             qos_profile=history_depth)
    
    # Initialize a publisher for force commands.
    force_publisher = node.create_publisher(msg_type=Vector3,
                                            topic='command/force',
                                            qos_profile=history_depth)
    
    # See Issue 13: https://github.com/ricmua/ros_force_dimension/issues/13     
    
    #
    A = (0.0, 0.0, 0.0)
    B = (0.0, 0.0, 0.0)
    previous_button = 0
    
    # Subscribe to receive position feedback a second time.
    def update(msg):
        
        nonlocal previous_button
        nonlocal A
        nonlocal B
        
        # Initialize variables to represent the current state.
        message_to_tuple = lambda m: (m.x, m.y, m.z)
        current_position = message_to_tuple(state['position'])
        current_velocity = message_to_tuple(state['velocity'])
        current_button   = state['button'].data
        
        ## If the button is released, then update the segment points.
        #if previous_button and not current_button: 
        #    B = message_to_tuple(state['position']) if not any(B) and any(A) else B
        #    A = message_to_tuple(state['position']) if not any(A) else A
        #    log(f'Point A: {A} -> Point B: {B}')
        # 
        ## If the button is held down, then clear the segment points.
        #if current_button and previous_button: 
        #    A = (0.0, 0.0, 0.0)
        #    B = (0.0, 0.0, 0.0)
        #    log(f'Segment points cleared')
        
        if (previous_button == 1) and not current_button: 
            A = message_to_tuple(state['position'])
            log(f'Point A: {A}')
        if (previous_button == 2) and not current_button: 
            B = message_to_tuple(state['position'])
            log(f'Point B: {B}')
        if (previous_button == 4) and not current_button: 
            A = (0.0, 0.0, 0.0)
            B = (0.0, 0.0, 0.0)
            log(f'Segment points cleared')
        
        # Compute spring-damper guidance force.
        def compute_force(position, velocity, A, B):
            
            # Retrieve  the parameters.
            Kp = node.get_parameter('spring.stiffness').value
            Kv = node.get_parameter('spring.damping').value
            
            # Compute the projection of the device position onto the segment.
            projection = projectPointOnSegment(position, A, B)
            
            # Compute the guidance force, modeled as a spring-damper system 
            # that pulls the device towards its projection on the constraint 
            # segment.
            force  = Kp * (numpy.asarray(projection) - position)
            force -= Kv * numpy.asarray(velocity)
            
            # Project the guidance force onto the vector defined by the device 
            # position and its projection; this removes all unwanted force 
            # components (e.g. damping along the "free" direction).
            force = projectForceOnDirection(force, position, projection)
            
            # Return the result.
            return force
        
        # Apply force.
        force = compute_force(current_position, current_velocity, A, B)
        force = force if any(A) and any(B) else (0.0, 0.0, 0.0)
        #force = (0.0, 0.0, 0.0)
        message = Vector3(**dict(zip(('x', 'y', 'z'), force)))
        force_publisher.publish(message)
        
        # Record the previous value of the user button, for the next sample.
        previous_button = current_button
        
    # Tie updates to the high-frequency position updates by creating a second 
    # subscription for the appropriate topic.
    node.create_subscription(Point, 'feedback/position',
                             callback=update, qos_profile=10)
    
    #
    try: rclpy.spin(node)
    except KeyboardInterrupt as e: pass
    #finally: rclpy.shutdown()
    


