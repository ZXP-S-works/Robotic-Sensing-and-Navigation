#/usr/bin/env bash
sudo chmod 666 /dev/ttyUSB0
#sudo chmod 666 /dev/ttyACM1
#export CLASSPATH=$PWD
export CLASSPATH=../lcm-spy/my_types.jar
#lcm-logger --auto-split-hours=0.5 -s ./data/lcm-log-%F-%T &

lcm-logger -s ../log/lcm-log-%F-%T &
lcm-spy &

#./gps_driver.py /dev/ttyUSB0 &
./imu_driver.py /dev/ttyUSB0

kill %1 #%2