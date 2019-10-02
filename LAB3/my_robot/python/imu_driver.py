#!/usr/bin/env python
# -*- coding: utf-8 -*-
# for IMU

import sys
import lcm
import utm
import time
import serial
import exlcm


class Imu(object):
    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 115200, timeout=1.)  # 115200-N-8-1
        self.lcm = lcm.LCM('udpm://239.255.76.67:7667?ttl=1')
        self.packet = exlcm.imu()
        print('IMU: Initialization')

    def readloop(self):
        while True:
            line = self.port.readline()
            vals = [x for x in line.split(',')]
            if vals[0] == '$VNYMR':
                while (vals[2] == ''):
                	print('wait for imu signal')
                	time.sleep(2)
                # print(vals[12])
                # for i in vals[12]:
                #     print(i)
                # print('end')
                vals[12] = vals[12][:-5]
                print(vals[12])
                self.packet.yaw = float(vals[1])
                self.packet.pitch = float(vals[2])
                self.packet.roll = float(vals[3])
                self.packet.magnetic_x = float(vals[4])
                self.packet.magnetic_y = float(vals[5])
                self.packet.magnetic_z = float(vals[6])
                self.packet.acceleration_x = float(vals[7])
                self.packet.acceleration_y = float(vals[8])
                self.packet.acceleration_z = float(vals[9])
                self.packet.angular_rate_x = float(vals[10])
                self.packet.angular_rate_y = float(vals[11])
                self.packet.angular_rate_z = float(vals[12])
                for i in vals:
                    print(i)
                print('imu publishing')
                self.lcm.publish("IMU", self.packet.encode())

        
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: %s <serial_port>\n" % sys.argv[0])
        sys.exit(0)
    my_imu = Imu(sys.argv[1])
    my_imu.readloop()
