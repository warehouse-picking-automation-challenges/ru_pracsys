from Device import Controller

import time
import math
 
servo = Controller()
servo.setAccel(5, 255)

servo.setTarget(5, 4500)
time.sleep(0.5)
servo.setTarget(5, 7000)
time.sleep(0.5)
servo.setTarget(5, 6100)

# for j in range(0, 100):
#     for i in range(-180, 180):
#         position = 6100 + int(math.cos(i * (math.pi / 180)) * 1800)
#         print "Pedro j = ", j, "  i = ", i, "  postion = ", position
#         servo.setTarget(5, position)  #set servo to move to center position
#         time.sleep(0.005)
#     
# servo.close
