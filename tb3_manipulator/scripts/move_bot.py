import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool

class DifferentialDriveController:
    def __init__(self, left_joint_name, right_joint_name):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Store joint names
        self.left_joint_name = left_joint_name
        self.right_joint_name = right_joint_name

        # Define control gains (adjust as needed)
        self.kp = 0.1  # Proportional gain
        self.ki = 0.01  # Integral gain
        self.kd = 0.05  # Derivative gain

        # Initialize PID variables
        self.left_integral_error = 0.0
        self.right_integral_error = 0.0
        self.prev_left_error = 0.0
        self.prev_right_error = 0.0

    def control_joints(self, left_theta_desired, right_theta_desired):
        # Define the rate at which the while loop will run (in Hz)
        rate = rospy.Rate(5)  # Adjust this rate as needed

        while not rospy.is_shutdown():
            # Get current joint states
            joint_states = rospy.wait_for_message('/joint_states', JointState)

            # Extract left and right joint angles
            left_theta_current = joint_states.position[joint_states.name.index(self.left_joint_name)]
            right_theta_current = joint_states.position[joint_states.name.index(self.right_joint_name)]

            # Calculate the error in joint angles
            left_error = left_theta_desired - left_theta_current
            right_error = right_theta_desired - right_theta_current

            # Calculate control commands using PID controller
            self.left_integral_error += left_error
            self.right_integral_error += right_error

            left_cmd_vel = self.kp * left_error + self.ki * self.left_integral_error + self.kd * (left_error - self.prev_left_error)
            right_cmd_vel = self.kp * right_error + self.ki * self.right_integral_error + self.kd * (right_error - self.prev_right_error)

            self.prev_left_error = left_error
            self.prev_right_error = right_error

            # Create a Twist message for cmd_vel
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = (left_cmd_vel + right_cmd_vel) / 2.0
            cmd_vel_msg.angular.z = (-left_cmd_vel + right_cmd_vel) / 2.0

            # Publish cmd_vel
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # Check if both joint angles are close to the desired values
            epsilon = 0.05  # Tolerance for angle error
            if abs(left_error) < epsilon and abs(right_error) < epsilon:
                rospy.loginfo("Reached desired joint angles.")
                break

            rate.sleep()
        return True
    
    def move(self, thetas):
        try:
            rospy.loginfo('Got wheel data')
            self.control_joints(thetas.data[0], thetas.data[1])
            rospy.loginfo('wheel moved, go ahead')
        except (IndexError, ValueError):
            rospy.loginfo('some error in move_bot.py -> DifferentialDriveController -> move()')
            
if __name__ == '__main__':
    try:
        # Initialize ROS node and publishers
        rospy.init_node('differential_drive_controller')
        rospy.loginfo('Node Initiated')
        # Specify the desired joint angles (in radians)
        # left_theta_desired = 0.0  # Adjust as needed
        # right_theta_desired = 0.0  # Adjust as needed
        controller = DifferentialDriveController('wheel_left_joint', 'wheel_right_joint')
        # controller.control_joints(left_theta_desired, right_theta_desired)
        sub = rospy.Subscriber('/manipulator/wheel_theta', Float64MultiArray, controller.move)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
