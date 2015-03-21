#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import dynamic_reconfigure.client

b=[]# class cstruct:
#     x = 0.0
#     y = 0.0
#     z = 0.0
    

# calc_pose = [cstruct() for i in range(4)]

def callback(data):
    x=[0,0,0,0]
    y=[0,0,0,0]
    z=[0,0,0,0]
    j=0
    for j in range(len(data.markers)):
         i=data.markers[j].id
         x[i]=data.markers[j].pose.pose.position.x
         y[i]=data.markers[j].pose.pose.position.y
         z[i]=data.markers[j].pose.pose.position.z
         #        print calc_pose[i].x
    
    client = dynamic_reconfigure.client.Client('cropbox')
    params= {'min_x': min(x),'max_x': max(x),'min_y': min(y),'max_y': max(y),'min_z': min(z),'max_z': max(z)}
    config = client.update_configuration(params)         
   
    
    
    
def listener():

    rospy.init_node('dynamic_cropbox', anonymous=True)

    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)

   
    rospy.spin()

if __name__ == '__main__':
    listener()
