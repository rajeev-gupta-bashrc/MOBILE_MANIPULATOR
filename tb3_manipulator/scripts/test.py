#!/usr/bin/env python

import rospy
import numpy as np, math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float64MultiArray, Float64

rospy.init_node('MM_Line_Node', anonymous=True)
mm_command_pub = rospy.Publisher('mm_command', Float64MultiArray, queue_size=10)
joint_command_pubs = [rospy.Publisher(joint_topic, Float64, queue_size=10) for joint_topic in [f"/manipulator/joint{i+1}_position/command" for i in range(5)]]
cmd = Float64MultiArray(data=[0, 0, 0, 0, 0])
rate = rospy.Rate(1)
# Degree to radian conversion
d2r=math.pi/180
# Radian to degree conversion
r2d=180/math.pi
# Sampling period
delta_t =  0.05
# Mobile plotform parameters declaration
L1=0.2      
x=0           
y=0        
dt=0.01        
v=0.5
# manipulators parameters assignment
l1=0.033    
l2=0.155      
l3=0.135      
d1=0.37     
d2=0.175     
x0=0.1        
y0=0.1             
b1=0.03       
b2=0.04              
b3=0.03        
b4=0.04              
b5=0.1       
b6=0.1 
b7=0.03
r_1=0.1      
L=0.1         
h=0.1   
phi0=0
# Theta varies from 0 t0 360 degrees
thet = list(range(33))
xf =[00]
yf =[00]
zf =[00]
xf2 =[00]
yf2 =[00]
zf2 =[00]
xfa1 =[00]
yfa1 =[00]
zfa1 =[00]
xfa2 =[00]
yfa2 =[00]
zfa2 =[00]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
XF = ax.plot(xf, yf, zf, linewidth=5, color='blue')
XF2 = ax.plot(xf2, yf2, zf2, linewidth=5, color='blue')
XF1 = ax.plot(xfa1, yfa1, zfa1, linewidth=5, color='blue')
XF2 = ax.plot(xfa2, yfa2, zfa2, linewidth=5, color='blue')
XJ = ax.plot([0], [0], [0], linewidth=5, color='red', marker='o', markersize=5, markerfacecolor='red')

ax.set_xlim([-0.2, 0.5])
ax.set_ylim([-0.2, 0.5])
ax.set_zlim([-0.2, 0.5])
ax.grid()

qf1 = np.array([[75 * d2r], [0 * d2r], [0 * d2r], [0 * d2r], [0 * d2r]])
# only the three F/E angles alone
t1 = qf1[0, 0]
t2 = qf1[1, 0]
t3 = qf1[2, 0]
t4 = qf1[3, 0]
t5 = qf1[4, 0]
t6 = 0*d2r
t7 = -90*d2r
t8 = -180*d2r
t9 =  -90*d2r
t10 = 90*d2r
t11 = 0*d2r
t12 = -90*d2r

# Forward kinematic equations
xfa=l1*math.cos (t1) + b1*math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) + b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) + math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5))) - math.sin (t7)*(math.sin (t1)*math.sin (t5)*math.sin (t6) - math.cos (t5)*math.cos (t6)*math.sin (t1) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + b1*math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) - b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) + math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5))) + math.cos (t7)*(math.sin (t1)*math.sin (t5)*math.sin (t6) - math.cos (t5)*math.cos (t6)*math.sin (t1) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + l2*math.cos (t1)*math.cos (t2) + b2*math.sin (t2 + t3 + t4)*math.cos (t1) + d2*math.sin (t2 + t3 + t4)*math.cos (t1) + l3*math.cos (t1)*math.cos (t2)*math.cos (t3) - l3*math.cos (t1)*math.sin (t2)*math.sin (t3)       
yfa=l1*math.sin (t1) - b1*math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1))) + math.sin (t7)*(math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t1)*math.sin (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) - b1*math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1))) - math.cos (t7)*(math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t1)*math.sin (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) + l2*math.cos (t2)*math.sin (t1) + b2*math.sin (t2 + t3 + t4)*math.sin (t1) + d2*math.sin (t2 + t3 + t4)*math.sin (t1) + l3*math.cos (t2)*math.cos (t3)*math.sin (t1) - l3*math.sin (t1)*math.sin (t2)*math.sin (t3)
zfa=d1 + l3*math.sin (t2 + t3) + (b1*math.sin (t2 + t3 + t4 - t5 - t6))/2 + l2*math.sin (t2) + (b7*math.sin (t2 + t3 + t4 + t5 + t6 + t7 + t12))/2 + (b1*math.sin (t2 + t3 + t4 + t5 + t6))/2 - (b7*math.sin (t5 - t3 - t4 - t2 + t6 + t7 + t12))/2 - b2*math.cos (t2 + t3 + t4) - d2*math.cos (t2 + t3 + t4)

t=0
tf=1
# Joint Limitsa
q1min = -169*d2r      
q1max = 169*d2r
q2min = -65*d2r       
q2max = 90*d2r
q3min = -150*d2r      
q3max = 146*d2r
q4min = -102.5*d2r    
q4max = 102.5*d2r
q5min = -167.5*d2r    
q5max = 167.5*d2r
# Average of Joint limits
q1_middle = (q1min + q1max)/2     
q2_middle = (q2min + q2max)/2
q3_middle = (q3min + q3max)/2     
q4_middle = (q4min + q4max)/2
q5_middle = (q5min + q5max)/2     

# plt.gca().set_next_plot('replace')
xpos=[]
ypos=[]
zpos=[]
xdes = []
ydes = []
zdes = []
xdot = []
ydot = []
zdot = []
E_sqrt = []
for i in range(len(thet)):
    rate.sleep()
    t1 = qf1[0, 0]
    t2 = qf1[1, 0] 
    t3 = qf1[2, 0]
    t4 = qf1[3, 0]
    t5 = qf1[4, 0]
    joint_ang=[t1, t2, t3, t4, t5]
    xpos.append(0)
    ypos.append(0)
    zpos.append(0)
    xpos[i]=l1*math.cos (t1) + b1*math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) + b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) + math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5))) - math.sin (t7)*(math.sin (t1)*math.sin (t5)*math.sin (t6) - math.cos (t5)*math.cos (t6)*math.sin (t1) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + b1*math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) - b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) + math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5))) + math.cos (t7)*(math.sin (t1)*math.sin (t5)*math.sin (t6) - math.cos (t5)*math.cos (t6)*math.sin (t1) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + l2*math.cos (t1)*math.cos (t2) + b2*math.sin (t2 + t3 + t4)*math.cos (t1) + d2*math.sin (t2 + t3 + t4)*math.cos (t1) + l3*math.cos (t1)*math.cos (t2)*math.cos (t3) - l3*math.cos (t1)*math.sin (t2)*math.sin (t3)
    ypos[i]=l1*math.sin (t1) - b1*math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1))) + math.sin (t7)*(math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t1)*math.sin (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) - b1*math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1))) - math.cos (t7)*(math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t1)*math.sin (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) + l2*math.cos (t2)*math.sin (t1) + b2*math.sin (t2 + t3 + t4)*math.sin (t1) + d2*math.sin (t2 + t3 + t4)*math.sin (t1) + l3*math.cos (t2)*math.cos (t3)*math.sin (t1) - l3*math.sin (t1)*math.sin (t2)*math.sin (t3)
    zpos[i]=d1 + l3*math.sin (t2 + t3) + (b1*math.sin (t2 + t3 + t4 - t5 - t6))/2 + l2*math.sin (t2) + (b7*math.sin (t2 + t3 + t4 + t5 + t6 + t7 + t12))/2 + (b1*math.sin (t2 + t3 + t4 + t5 + t6))/2 - (b7*math.sin (t5 - t3 - t4 - t2 + t6 + t7 + t12))/2 - b2*math.cos (t2 + t3 + t4) - d2*math.cos (t2 + t3 + t4)  
    
    xf10=xpos[i]
    yf10=ypos[i]
    zf10=zpos[i]
     
    xd = [xf10-0.1*t, yf10, zf10]
    
    xdes.append(0)
    ydes.append(0)
    zdes.append(0)
    xdes[i] = xd[0]
    ydes[i] = xd[1]
    zdes[i] = xd[2]
    xdot.append(0)
    ydot.append(0)
    zdot.append(0)
    if i==1:
        xdot[i] = 0
        ydot[i] = 0
        zdot[i] = 0
    else:
        xdot[i] =(xdes[i]-xdes[i-1])/delta_t
        ydot[i] =(ydes[i]-ydes[i-1])/delta_t
        zdot[i] =(zdes[i]-zdes[i-1])/delta_t
    xdes[i]
    ydes[i]
    zdes[i]
    xdotis = [xdot[i], ydot[i], zdot[i]]

    ## Jacobian Matrix
    Jaco =np.array([[b1*math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)) - l1*math.sin (t1) + b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1))) + math.sin (t7)*(math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t1)*math.sin (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) + b1*math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) - b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.cos (t1)*math.cos (t5) + math.cos (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)) + math.cos (t6)*(math.cos (t1)*math.sin (t5) - math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1))) - math.cos (t7)*(math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t1)*math.sin (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) - l2*math.cos (t2)*math.sin (t1) - b2*math.sin (t2 + t3 + t4)*math.sin (t1) - d2*math.sin (t2 + t3 + t4)*math.sin (t1) - l3*math.cos (t2)*math.cos (t3)*math.sin (t1) + l3*math.sin (t1)*math.sin (t2)*math.sin (t3), b2*math.cos (t2 + t3 + t4)*math.cos (t1) - l2*math.cos (t1)*math.sin (t2) + d2*math.cos (t2 + t3 + t4)*math.cos (t1) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6)) - math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + b7*math.sin (t12)*(math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5)) + math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6))) - l3*math.cos (t1)*math.cos (t2)*math.sin (t3) - l3*math.cos (t1)*math.cos (t3)*math.sin (t2) - b1*math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) + b1*math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6), b2*math.cos (t2 + t3 + t4)*math.cos (t1) + d2*math.cos (t2 + t3 + t4)*math.cos (t1) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6)) - math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + b7*math.sin (t12)*(math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5)) + math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6))) - l3*math.cos (t1)*math.cos (t2)*math.sin (t3) - l3*math.cos (t1)*math.cos (t3)*math.sin (t2) - b1*math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) + b1*math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6), b2*math.cos (t2 + t3 + t4)*math.cos (t1) + d2*math.cos (t2 + t3 + t4)*math.cos (t1) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6)) - math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + b7*math.sin (t12)*(math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5)) + math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6))) - b1*math.sin (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) + b1*math.sin (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6), b1*math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) - math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5))) + math.sin (t7)*(math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t6)*math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6))) - b1*math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) - math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5))) - math.cos (t7)*(math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t6)*math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6)))], \
        [l1*math.cos (t1) + b1*math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) + b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) + math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5))) - math.sin (t7)*(math.sin (t1)*math.sin (t5)*math.sin (t6) - math.cos (t5)*math.cos (t6)*math.sin (t1) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + b1*math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) - b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) + math.cos (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5))) + math.cos (t7)*(math.sin (t1)*math.sin (t5)*math.sin (t6) - math.cos (t5)*math.cos (t6)*math.sin (t1) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.sin (t6) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t6)*math.sin (t5))) + l2*math.cos (t1)*math.cos (t2) + b2*math.sin (t2 + t3 + t4)*math.cos (t1) + d2*math.sin (t2 + t3 + t4)*math.cos (t1) + l3*math.cos (t1)*math.cos (t2)*math.cos (t3) - l3*math.cos (t1)*math.sin (t2)*math.sin (t3), b7*math.cos (t12)*(math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5)) - math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) - math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6))) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) - math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6)) + math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) - l2*math.sin (t1)*math.sin (t2) + b2*math.cos (t2 + t3 + t4)*math.sin (t1) + d2*math.cos (t2 + t3 + t4)*math.sin (t1) - l3*math.cos (t2)*math.sin (t1)*math.sin (t3) - l3*math.cos (t3)*math.sin (t1)*math.sin (t2) - b1*math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) + b1*math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6), b7*math.cos (t12)*(math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5)) - math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) - math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6))) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) - math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6)) + math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) + b2*math.cos (t2 + t3 + t4)*math.sin (t1) + d2*math.cos (t2 + t3 + t4)*math.sin (t1) - l3*math.cos (t2)*math.sin (t1)*math.sin (t3) - l3*math.cos (t3)*math.sin (t1)*math.sin (t2) - b1*math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) + b1*math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6), b7*math.cos (t12)*(math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5)) - math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) - math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6))) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) - math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6)) + math.cos (t7)*(math.sin (t2 + t3 + t4)*math.cos (t5)*math.sin (t1)*math.sin (t6) + math.sin (t2 + t3 + t4)*math.cos (t6)*math.sin (t1)*math.sin (t5))) + b2*math.cos (t2 + t3 + t4)*math.sin (t1) + d2*math.cos (t2 + t3 + t4)*math.sin (t1) - b1*math.sin (t2 + t3 + t4)*math.cos (t5)*math.cos (t6)*math.sin (t1) + b1*math.sin (t2 + t3 + t4)*math.sin (t1)*math.sin (t5)*math.sin (t6), b1*math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) - math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5))) + math.sin (t7)*(math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t6)*math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6))) - b1*math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) - math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5))) - math.cos (t7)*(math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t6)*math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6)))], \
        [0, l3*math.cos (t2 + t3) + (b1*math.cos (t2 + t3 + t4 - t5 - t6))/2 + l2*math.cos (t2) + (b7*math.cos (t2 + t3 + t4 + t5 + t6 + t7 + t12))/2 + (b1*math.cos (t2 + t3 + t4 + t5 + t6))/2 + (b7*math.cos (t5 - t3 - t4 - t2 + t6 + t7 + t12))/2 + b2*math.sin (t2 + t3 + t4) + d2*math.sin (t2 + t3 + t4), l3*math.cos (t2 + t3) + (b1*math.cos (t2 + t3 + t4 - t5 - t6))/2 + (b7*math.cos (t2 + t3 + t4 + t5 + t6 + t7 + t12))/2 + (b1*math.cos (t2 + t3 + t4 + t5 + t6))/2 + (b7*math.cos (t5 - t3 - t4 - t2 + t6 + t7 + t12))/2 + b2*math.sin (t2 + t3 + t4) + d2*math.sin (t2 + t3 + t4), (b1*math.cos (t2 + t3 + t4 - t5 - t6))/2 + (b7*math.cos (t2 + t3 + t4 + t5 + t6 + t7 + t12))/2 + (b1*math.cos (t2 + t3 + t4 + t5 + t6))/2 + (b7*math.cos (t5 - t3 - t4 - t2 + t6 + t7 + t12))/2 + b2*math.sin (t2 + t3 + t4) + d2*math.sin (t2 + t3 + t4), b1*math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)) - b7*math.cos (t12)*(math.cos (t7)*(math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) - math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5))) + math.sin (t7)*(math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t6)*math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6))) - b1*math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) + b7*math.sin (t12)*(math.sin (t7)*(math.sin (t6)*(math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)) - math.cos (t6)*(math.cos (t5)*math.sin (t1) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5))) - math.cos (t7)*(math.cos (t5)*math.sin (t1)*math.sin (t6) + math.cos (t6)*math.sin (t1)*math.sin (t5) + math.cos (t2 + t3 + t4)*math.cos (t1)*math.cos (t5)*math.cos (t6) - math.cos (t2 + t3 + t4)*math.cos (t1)*math.sin (t5)*math.sin (t6)))]])

    JT = np.transpose(Jaco)
    ## Detremining constant arbitrary vector using
    Pone = -(2*q1_middle - 2*qf1[0, 0])/(3*(q1min - q1max)**2)
    p1 =  -(2*qf1[0, 0] - 2*q1_middle)/(6*(q1min - q1max)**2)
    
    Ptwo=-(2*q2_middle - 2*qf1[1, 0])/(3*(q2min - q2max)**2)
    p2 =-(2*qf1[1, 0] - 2*q2_middle)/(6*(q2min - q2max)**2)
    
    Pthree = -(2*q3_middle - 2*qf1[2, 0])/(3*(q3min - q3max)**2)
    p3=-(2*qf1[2, 0] - 2*q3_middle)/(6*(q3min - q3max)**2)
    
    pfour=-(2*q4_middle - 2*qf1[3, 0])/(3*(q4min - q4max)**2)
    p4=-(2*qf1[3, 0] - 2*q4_middle)/(6*(q4min - q4max)**2)
    
    Pfive = -(2*q5_middle - 2*qf1[4, 0])/(3*(q5min - q5max)**2)
    p5 = -(2*qf1[4, 0] - 2*q5_middle)/(6*(q5min - q5max)**2)
    Eps = np.array([p1, p2, p3, p4, p5]).reshape(-1, 1)
    
    # print(Jaco.shape)
    Jinv = np.linalg.pinv(Jaco)
    
    dpb = np.array(xdotis).reshape(-1, 1) 
    
    #qt1dot = Jinv*dpb + (eye(3) - Jinv*Jaco)*Eps
    e = np.array([[xdes[i]-xf10], [ydes[i]-yf10], [zdes[i]-zf10]])
    E_sqrt.append(0)
    E_sqrt[i] = math.sqrt((xdes[i]-xf10)**2+(ydes[i]-yf10)**2+(zdes[i]-zf10)**2)
    RMSE=math.sqrt(np.mean(np.array(E_sqrt)))
    #K = 200
    #Kp=200
    K = 0
    Kp=10
    qt1dot = Jinv@(dpb+K*e) + (np.eye(5)-Jinv@Jaco)@(Kp*Eps) 
    qf1 = qf1 + qt1dot*delta_t
    # print(qf1)
    for i in range(5):
        cmd.data[i] = qf1[i, 0]
        joint_command_pubs[i].publish(Float64(data=cmd.data[i]))
    mm_command_pub.publish(cmd)
    
# plt.show()