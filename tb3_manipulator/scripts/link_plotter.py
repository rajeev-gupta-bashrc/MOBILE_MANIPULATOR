#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point

class LinkPositionPlotter:
    def __init__(self):
        rospy.init_node('link_position_plotter', anonymous=True)
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        self.link_positions = None
        self.link_trajectory = []

    def link_states_callback(self, data):
        # Find the index of the desired link (e.g., 'link_name') in the message
        try:
            link_index = data.name.index('tb3_manipulator::link5')
            self.link_positions = data.pose[link_index].position
            self.link_trajectory.append(self.link_positions)  # Append current position to trajectory
        except ValueError:
            rospy.logwarn("Desired link not found in LinkStates message")

    def plot_link_positions(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        while not rospy.is_shutdown():
            if self.link_positions is not None:
                # Extract x, y, z coordinates
                x, y, z = self.link_positions.x, self.link_positions.y, self.link_positions.z

                # Plot the link position in 3D
                ax.cla()
                ax.scatter(x, y, z, c='b', marker='o', label='Current Position')

                # Plot the trajectory
                if self.link_trajectory:
                    trajectory_points = zip(*[(p.x, p.y, p.z) for p in self.link_trajectory])
                    ax.plot(*trajectory_points, c='r', label='Trajectory')

                ax.set_xlabel('X-axis')
                ax.set_ylabel('Y-axis')
                ax.set_zlabel('Z-axis')
                ax.set_title('Link Position and Trajectory in 3D')
                # for circle data1
                # ax.set_xlim(0.24, 0.32)
                # ax.set_ylim(0.02, 0.12)
                # ax.set_zlim(0.14, 0.17)
                
                # for line data1
                # ax.set_xlim(-0.1, 0.1)
                # ax.set_ylim(0.2, 0.4)
                # ax.set_zlim(0.4, 0.6)
                ax.legend()
                plt.pause(0.1)

if __name__ == '__main__':
    try:
        link_plotter = LinkPositionPlotter()
        link_plotter.plot_link_positions()
    except rospy.ROSInterruptException:
        pass
