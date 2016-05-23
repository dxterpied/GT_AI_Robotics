#!/home/robot

from ev3 import *
import time

m = Motor(OUTPUT_A)

m.run_forever(duty_cycle_sp = 100)

print "hello sucker!"


time.sleep(1)
m.stop()