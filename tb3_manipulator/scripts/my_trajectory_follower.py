import sys
import os
package_directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(package_directory)


from path_planner import Path_Planner
from drive_class import Diff_drive
import rospy 

mini_bot = None
def shutdown_callback():
    if mini_bot is not None:
        mini_bot.stop()
    
if __name__ == '__main__':
    rospy.init_node('my_tracjectory_follower_node', anonymous=True)
    mini_bot = Diff_drive(0.287, 0.033)
    mini_bot.namespace = 'tb3_manipulator'
    navigator = Path_Planner(-0.1, -0.1, 2, 3)
    navigator.set_vehicle(mini_bot)
    # To move the bot to a point---->
    navigator.move_to(3, 1)
    
    # To follow a trajectory ------->
    # navigator.function = "0"
    rospy.on_shutdown(shutdown_callback)
    try:
        navigator.follow()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.loginfo('Stopping bot :(')
        mini_bot.stop()
    rospy.spin()