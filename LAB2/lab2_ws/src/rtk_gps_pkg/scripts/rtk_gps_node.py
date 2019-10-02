#!/usr/bin/env python

import serial # Import serial module
import rospy
from sensor_msgs.msg import NavSatFix

class SerialReader(object):
    def __init__(self, portName, baudRate):
        rospy.loginfo('GPS: Initialization')
        print('aaa')
        self.port = serial.Serial(portName, baudRate, timeout=1.)  # 4800-N-8-1
        self.packet = NavSatFix()


    def readloop(self):
        pub = rospy.Publisher('rtk_fix', NavSatFix, queue_size=10)
        rospy.init_node('serial_reader', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            line = self.port.readline()
            vals = [x for x in line.split(',')]
            print(vals[0] == '$GNGGA')
            print(vals[0])
            # vals = ['$GPGGA',241260.60,1111.11,'N',1111.11,'S',1,22,1.1,1.1,'M',1.1,'M',1.1,23*23]

            if vals[0] == '$GNGGA':
                lat = (int((float(vals[2])/100))+((float(vals[2])%100)/60))		#for boston N
                lon = -(int((float(vals[4])/100))+((float(vals[4])%100)/60))	#for boston W
                self.packet.latitude = lat
                self.packet.longitude = lon
                self.packet.altitude = float(vals[9])
                print('publish', lat, lon, vals[9])
                rospy.loginfo('GPS publishing')
                pub.publish(self.packet)
                rate.sleep()

if __name__ == '__main__':
    portName = rospy.get_param('port_name', '/dev/ttyACM0')     # param set in your launch file, will use
                                                                # 'default_value' if parameter not found
    baudRate = rospy.get_param('baud_rate', '115200')   # param set in your launch file, will use
                                                        # 'default_value' if parameter not found
    print(baudRate)
    print(portName)
    mySr = SerialReader(portName, baudRate)
    mySr.readloop()