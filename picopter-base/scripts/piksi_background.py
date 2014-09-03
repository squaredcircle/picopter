import sys
sys.path.insert(0, '/home/baxter/Desktop/piksi_firmware/scripts')

import sbp_piksi as sbp
import serial_link
import time

def pos_callback(data):
  p = sbp.PosLLH(data)
  #print("Time: " + str(p.tow) + "\tLattitude: " + str(p.lat) + "\tLongitude:" + str(p.lon) )
  gps_data_string = str(p.tow//100) + " " + str(p.lat)  + " " + str(p.lon)  + " " + str(p.lat)  + " " + str(p.v_accuracy)  + " " + str(p.n_sats) + "\n"
  print(gps_data_string)
  f = open('piksi_data.txt', 'w+')
  f.write(gps_data_string)
  f.close()
  


try:
  f = open('piksi_data.txt', 'w+')
  f.close()
  
except:
  print("Could not open file\n")
  exit()

try:
  link = serial_link.SerialLink(port="/dev/ttyUSB0")
except:
  print("No device found\n")
  exit()
  
link.add_callback(sbp.SBP_POS_LLH, pos_callback)
print("Reading data from piksi\n")

try:
  while True:
    time.sleep(60)
except:
  link.close()
  print("Exiting\n")
