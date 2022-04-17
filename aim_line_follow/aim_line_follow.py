# AIM Line Follow Example Code - Python
# 
# This example code is meant to act as an example for simple line-following control
# in NXP Cup Summer Camp and NXP AIM Challenge.
#
# The code is heavily commented to help students and professors understand the code.
#
# Questions? Email landon.haugh@nxp.com
#
# Written by: Landon Haugh (landon.haugh@nxp.com)
#             Aditya Vashista (aditya.vashista@nxp.com)

# ROS2 library imports
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

# ROS2 message imports
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from nxp_cup_interfaces.msg import PixyVector

# Standard Python3 imports
from time import sleep
from datetime import datetime
import numpy as np

# Line follower Node class
# Functions:
#   __init__()        - Initialize subscribers, publishers, and parameters
#   get_num_vectors() - Return the number of lines detected from the simulated Pixy camera
#   listener_callback() - Function that runs when a PixyVector ROS msg is received
class LineFollow(Node):

    # Initalize function - this is run at the instantiation of the LineFollow Class
    def __init__(self):

        # Initialize ROS2 node with name 'aim_line_follow'
        super().__init__('aim_line_follow')

        # Create parameters
        # Start delay - amount of time to wait before starting
        start_delay_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Seconds to delay before starting.')

        # Camera topic descriptor - Namespacing with camera topic
        camera_vector_topic_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description='Namespaceing with camera topic.')
        
        # Linear Velocity - Parameter to change linear velocity scalar
        linear_velocity_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Linear velocity for vehicle motion (m/s).')

        # Angular Velocity - Parameter to change angular velocity scalar
        angular_velocity_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Angular velocity for vehicle motion (rad/s).')

        # Single line steer scale - scalar to adjust angular velocity when only one line is found
        single_line_steer_scale_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description='Single found line steer scaling.')
        
        # Declare parameters defined above with names and default values
        self.declare_parameter("start_delay", 15.0, 
            start_delay_descriptor)
        
        self.declare_parameter("camera_vector_topic", "/cupcar0/PixyVector", 
            camera_vector_topic_descriptor)
        
        self.declare_parameter("linear_velocity", 1.25, 
            linear_velocity_descriptor)

        self.declare_parameter("angular_velocity", 1.5, 
            angular_velocity_descriptor)

        self.declare_parameter("single_line_steer_scale", 0.5, 
            single_line_steer_scale_descriptor)

        # Pull parameters defined in ROS2 and apply them to class variables
        self.start_delay = float(self.get_parameter("start_delay").value)

        self.camera_vector_topic = str(self.get_parameter("camera_vector_topic").value)

        self.linear_velocity = float(self.get_parameter("linear_velocity").value)

        self.angular_velocity = float(self.get_parameter("angular_velocity").value)

        self.single_line_steer_scale = float(self.get_parameter("single_line_steer_scale").value)

        # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        # Subscribers - subscribe to PixyVector topic at 10Hz
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)


        # Publishers - Create publisher object so we can apply speed and steer to the vehicle
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Class variables for Speed, steer, and twist messages to package into cmd_vel topic
        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

    # get_num_vectors function - return number of vectors found
    def get_num_vectors(self, msg):

        # Initialize variable for return
        num_vectors = 0

        # Check to see if m0 line is all 0s. If so, 0 lines found. If not, add 1 to num_vectors
        if(not(msg.m0_x0 == 0 and msg.m0_x1 == 0 and msg.m0_y0 == 0 and msg.m0_y1 == 0)):
            num_vectors = num_vectors + 1
        # Check to see if m1 line is all 0s. If so, 1 lines found. If not, add 1 to num_vectors
        if(not(msg.m1_x0 == 0 and msg.m1_x1 == 0 and msg.m1_y0 == 0 and msg.m1_y1 == 0)):
            num_vectors = num_vectors + 1
        # return number of vectors found.
        return num_vectors

    def publish_controls(self, speed, steer):
        # Set speed and steer values
        # Vector3 format has x, y, z
        # Speed is in X direction, steer is in Z rotation
        self.speed_vector.x = float(speed)
        self.steer_vector.z = float(steer)

        # Set linear and angular values of Twist() msg to speed and steer Vector3
        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector

        # Publish our Twist() to the cmd_vel topic
        self.cmd_vel_publisher.publish(self.cmd_vel)


    # listener_callback function
    # Self driving algorithm code goes here.
    # The 'msg' argument contains a PixyVector message with line information.
    def listener_callback(self, msg):

        # Current time for timestamping cmd_vel msg.
        current_time = datetime.now().timestamp() 
        
        # Frame width and height of the PixyVector frame
        frame_width = 78
        frame_height = 51

        # Center of Pixy frame on the X axis.
        window_center = (frame_width / 2)

        # Misc variables
        x = 0
        y = 0
        steer = 0
        speed = 0

        # Get number of vectors in the PixyVector frame
        num_vectors = self.get_num_vectors(msg)

        # If num_vectors is 0...
        if(num_vectors == 0):  
            # Stop the car.
            speed = 0
            steer = 0

        # If one vector is found...
        if(num_vectors == 1):

            # Find x/y values normalized to frame width and height 
            if(msg.m0_x1 > msg.m0_x0):
                x = (msg.m0_x1 - msg.m0_x0) / frame_width
                y = (msg.m0_y1 - msg.m0_y0) / frame_height
            else:
                x = (msg.m0_x0 - msg.m0_x1) / frame_width
                y = (msg.m0_y0 - msg.m0_y1) / frame_height

            # Use slope of line to determine steering angle
            if(msg.m0_x0 != msg.m0_x1 and y != 0):
                steer = (-self.angular_velocity) * (x / y) * self.single_line_steer_scale
            else:
                steer = 0

            # Set speed to linear velocity parameter
            speed = self.linear_velocity

        # If two vectors are found..
        if(num_vectors == 2):

            # Find average of both top X values 
            m_x1 = (msg.m0_x1 + msg.m1_x1) / 2

            # Find distance from center of frame and use as steering value
            steer = self.angular_velocity*(m_x1 - window_center) / frame_width
            
            # Set speed to linear_velocity paramater
            speed = self.linear_velocity
        
        self.publish_controls(speed, steer)

# Main function
def main(args=None):

    # Initialize rclpy library
    rclpy.init(args=args)

    # Create LineFollow class
    line_follow = LineFollow()

    # Allow LineFollow class to spin (schedule listener_callback on msg recv)
    rclpy.spin(line_follow)

    # Once it dies, destroy it
    line_follow.destroy_node()

    # Shutdown rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
