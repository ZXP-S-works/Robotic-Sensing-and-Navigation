#!/usr/bin/env python

'''
Subscribes to the /rtk_gps_node topic
Converts the gps fixes to UTM
Publishes the UTM fixes to the topic /utm_fix as standard ROS nav_msgs/Odometry.msg
Save the UTM x,y and z under the geometry_msgs/Points x, y and z. Leave everything else as 0
'''

import rospy
import utm
import rosbag
import time
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

pub = rospy.Publisher('/utm_fix', Odometry, queue_size=10)
time_str = time.strftime('%Y-%m-%d-%H:%M:%S')
bag_name = 'gps_bag' + time_str + '.bag'
bag = rosbag.Bag(bag_name, 'w')

def callback(data):
    rospy.loginfo('converting...')
    utm_x, utm_y, _, _ = utm.from_latlon(data.latitude, data.longitude)
    utm_odom = Odometry()
    utm_odom.pose.pose.position.x = utm_x
    utm_odom.pose.pose.position.y = utm_y
    utm_odom.pose.pose.position.z = data.altitude
    print(utm_odom.pose.pose)
    bag.write('utm', utm_odom.pose.pose)

    rospy.loginfo('utm_odom publishing', utm_x, utm_y)
    pub.publish(utm_odom)

    point = Point()
    point.x = utm_x
    point.y = utm_y
    point.z = data.altitude
    bag.write('Points', point)
    print(point)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gps2utm', anonymous=True)
    rospy.Subscriber('rtk_fix', NavSatFix, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()