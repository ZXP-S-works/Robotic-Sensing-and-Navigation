#!/usr/bin/env python
# -*- coding: utf-8 -*-
# for GPS

import sys
import lcm
import utm
import time
import serial
import exlcm


class Gps(object):
    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 4800, timeout=1.)  # 4800-N-8-1
        self.lcm = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
        self.packet = exlcm.gps()
        print('GPS: Initialization')

    def readloop(self):
        while True:
            line = self.port.readline()
#            try:
            vals = [x for x in line.split(',')]
            print(vals[0])
            if vals[0] == '$GPGGA':
                for i in [1, 2, 4, 9]:
                	print(vals[i], 'end')
                while (vals[2] == ''):
                	print('wait for gps signal')
                	time.sleep(2)
                self.packet.time = float(vals[1])
                self.packet.lat = float(vals[2])
                self.packet.lon = float(vals[4])
                self.packet.alt = float(vals[9])

		for i in vals:
			print(i)
	        lat = (int((float(vals[2])/100))+((float(vals[2])%100)/60))
		lon = -(int((float(vals[4])/100))+((float(vals[4])%100)/60))
		print('in gpgga')
                # (EASTING, NORTHING, ZONE NUMBER, ZONE LETTER)
                self.packet.utm_x, self.packet.utm_y, _, _ \
                = utm.from_latlon(lat, lon)
		print(self.packet.utm_x)
		print(type(self.packet.time))
                print(self.packet.time)
                self.lcm.publish("GPS", self.packet.encode())
                print('(' + line +')')
 #           except:
 #               print('GPS ERROR (' + line + ')')
        
if __name__ == "__main__":
# $GPGGA,241260.60,1111.11,N,1111.11,S,1,22,1.1,1.1,M,1.1,M,1.1,23*23
    if len(sys.argv) != 2:
        print("Usage: %s <serial_port>\n" % sys.argv[0])
        sys.exit(0)
    my_gps = Gps(sys.argv[1])
    my_gps.readloop()
