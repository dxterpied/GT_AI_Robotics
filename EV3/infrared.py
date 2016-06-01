#!/usr/bin/python2

import ev3dev.ev3 as ev3

m1 = ev3.LargeMotor('outA')
m1.run_timed(time_sp=3000, duty_cycle_sp=75)

m2 = ev3.LargeMotor('outA')
m2.run_timed(time_sp=3000, duty_cycle_sp=75)
