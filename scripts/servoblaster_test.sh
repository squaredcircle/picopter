# Example of how to use servoblaster to control the copter
# Run this as root.
#
# Syntax: echo p1-PIN=(50-250 or %) > /dev/servoblaster
# The levels range from 50-250, but we find that 100-200 works roughly covering max-min
#
# Pin Mapping:
#	11: Channel A 		Left/right or Up/Down
#	12: NA
#	13: Elevator		Short Pulse
#	14: No Map
#	15: Rudder			Rotate
#	16: Gimble
#	

cd ~/PiBits/ServoBlaster/user
./servod

echo p1-15=100 > /dev/servoblaster
echo p1-11=50% > /dev/servoblaster
