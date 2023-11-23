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
from force_dimension_examples.segment import dynamics


#
class SegmentApp:
    
    def __init__(self, state, log=lambda m: print(m)):
        
        # Maintain a reference to the argument.
        self.state = state
        
        # Initialize default parameter values.
        self.A = (0.0, 0.0, 0.0)
        self.B = (0.0, 0.0, 0.0)
        self.previous_button = 0
        
        # Set up the logging functionality.
        self.log = log
        
        # Initialize default parameters.
        self._parameters = {'spring.stiffness': 2000.0, 'spring.damping': 20.0}
        
        # Display version information.
        log('Segment Constraint Example 3.16.1')
        log('Copyright (C) 2001-2023 Force Dimension')
        log('All Rights Reserved.')
        log('Translated to Python and ROS2 by a.whit')
        log('(Neuromechatronics Lab, Carnegie Mellon University')
        
        # Display instructions.
        log('press BUTTON or publish "p" to define the segment extremities')
        log('                publish "c" to clear the current segment')
        log('                "CTRL-C" to quit')
    
    def apply_force(self, x=0.0, y=0.0, z=0.0): 
        self.log(f'Applied force: ({x}, {y}, {z})')
    
    def get_parameter(self, key): return self._parameters[key]
    
    def process_input(self):
        
        # Initialize local variables to represent the current state.
        position = self.state['position']
        velocity = self.state['velocity']
        button   = self.state['button']
        console  = self.state['console']
        
        # Define shorthand for conditions / states.
        button_released = self.previous_button and not button
        
        # If the addition of a point has been requested, then assign a point 
        # for each of the first two times that a button is pressed and 
        # released. The first time that the button is pressed, then current 
        # position of the robot is assigned to point A. The second time the 
        # button is pressed, the current position is assigned to point B.
        if console == 'p':
            if button_released:
              if not any(self.A):
                  self.A = position
                  self.log(f'Set point A: {self.A}')
              elif not any(self.B):
                  self.B = position
                  self.state['console'] = ''
                  self.log(f'Set point B: {self.B}')
        
        # Clear the line segment sample points, if requested.
        if console == 'c':
            self.A = (0.0, 0.0, 0.0)
            self.B = (0.0, 0.0, 0.0)
            self.state['console'] = ''
            self.log('Cleared line segment')
        
        # See Issue 13: https://github.com/ricmua/ros_force_dimension/issues/13
        
    def process_forces(self):
        
        # Initialize local variables to represent the current state.
        position = self.state['position']
        velocity = self.state['velocity']
        
        # Retrieve the dynamics parameters.
        Kp = self.get_parameter('spring.stiffness')
        Kv = self.get_parameter('spring.damping')
        
        # Apply force.
        A = self.A
        B = self.B
        force = dynamics.compute_force(position, velocity, A, B, Kp, Kv)
        force = force if any(A) and any(B) else (0.0, 0.0, 0.0)
        self.apply_force(*force)
    
    def update(self, msg):
        
        # Process user input from the console and robot buttons.
        self.process_input()
        
        # Compute and apply any forces that might be appropriate.
        self.process_forces()
        
        # Record the previous value of the user button, for the next sample.
        button = self.state['button']
        self.previous_button = button
        
        

