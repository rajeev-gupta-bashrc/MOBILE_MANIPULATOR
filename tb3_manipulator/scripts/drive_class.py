import math, rospy
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist


class Diff_drive():
    def __init__(self, wheel_radii, wheel_separation):
        self.wheel_radii = wheel_radii
        self.wheel_separation = wheel_separation
        self.posn = [0.0, 0.0, 0.0]
        self.cmd_vel = [0.0, 0.0, 0.0]
        self.max_linear_vel = 0.10          #m/s
        self.max_angular_vel = 1            #rad/s
        self.forward_vel = self.max_linear_vel
        self.angular_vel = self.max_angular_vel
        self.prev_ang_err = 0
        self.angular_vel_integral = 0.0
        self.kp = 0.3
        self.kd = 0.5
        self.ki = 0.0
        
        self.namespace = ''
        self.link_state_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_callback)
        self.base_name = 'base_footprint'
        self.index = None
        self.cmd_pub = rospy.Publisher(self.namespace + '/cmd_vel', Twist, queue_size=10)
        
        
        
    def set_forwaard_vel(self, vel):
        self.forward_vel = vel
        
    def move(self, cmd_vector):
        xi = cmd_vector[0]
        yi = cmd_vector[1]
        th = math.atan2(yi, xi) 
        alpha = th
        err_th1 = (alpha-self.posn[2])
        self.cmd_vel[2] = (err_th1 * self.kp + (err_th1 - self.prev_ang_err) * self.kd + self.angular_vel_integral * self.ki) * self.angular_vel 
        self.cmd_vel[0] = self.angular_vel * 0.1
        # update algo data
        self.prev_ang_err = err_th1
        self.angular_vel_integral += err_th1
        # print(alpha, err_th1)
        data = Twist()
        data.linear.x = self.cmd_vel[0]
        data.angular.z = self.cmd_vel[2]
        self.cmd_pub.publish(data)
        return self.cmd_vel
    
    def link_callback(self, msg):
        if self.index is None:
            self.index = msg.name.index(self.namespace + '::' + self.base_name)
        _x = msg.pose[self.index].orientation.x
        _y = msg.pose[self.index].orientation.y
        _z = msg.pose[self.index].orientation.z
        _w = msg.pose[self.index].orientation.w
        rpy = euler_from_quaternion([_x, _y, _z, _w])
        self.posn[0] = msg.pose[self.index].position.x
        self.posn[1] = msg.pose[self.index].position.y
        self.posn[2] = rpy[2]
        
    def stop(self):
        data = Twist()
        self.cmd_pub.publish(data)