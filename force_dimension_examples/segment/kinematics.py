""" Computing kinematics projections to implement a line segment-constraint 
    application for Force Dimension haptic device.
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
import numpy


# Project a point onto a line segment.
def projectPointOnSegment(point, A, B):
    """ This function computes the projection of a `point` onto a segment 
        defined by two points `A` and `B`.
    """
    
    # Compute Ap vector.
    Ap = numpy.asarray(point) - A
    
    # Compute AB segment vector.
    AB = numpy.asarray(B) - A
    
    # Compute the segment norm, and return the current input point if it is too 
    # small.
    norm = numpy.linalg.norm(AB)
    if norm <= 1e-6: return point

    # Compute the segment direction unit vector.
    direction = AB / norm
    
    # Compute the projection ratio.
    projectionRatio = Ap @ direction
    
    # Compute the point projection on the segment.
    if projectionRatio < 0.0: projection = A
    elif projectionRatio > norm: projection = B
    else: projection = A + projectionRatio * direction
    
    # Return the result.
    return projection
    
  

# Project a force vector onto a direction vector.
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
    
  

# Compute spring-damper guidance force.
def compute_force(position, velocity, A, B, Kp=2000.0, Kv=20.0):
    
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
    
  

