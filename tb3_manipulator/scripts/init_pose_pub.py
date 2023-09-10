#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Float64

rospy.init_node('MM_Init_Pose', anonymous=True)
joint_command_pubs = [rospy.Publisher(joint_topic, Float64, queue_size=10) for joint_topic in [f"/manipulator/joint{i+1}_position/command" for i in range(5)]]
rate1 = rospy.Rate(10)
# init_pose = [16.9252, 20.3118, -9.3825, 13.0726, 12.2653]

## Init pose for Line ----------->
# init_pose = [80.3174568052801,
# 88.1114008107389,
# -47.8176334479377,
# 59.8647882648705,
# 49.2868846759355
# ]

## Init pose for circle ---------->
init_pose = [0.0,
20.0585,
-7.8419,
12.1782,
11.7269
]
while not rospy.is_shutdown(): 
    for i in range(5):
        joint_command_pubs[i].publish(Float64(init_pose[i]*math.pi/180))
        rospy.loginfo('init_pose published :)')
    rate1.sleep()