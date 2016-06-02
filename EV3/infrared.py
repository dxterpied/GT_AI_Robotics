#!/usr/bin/python2

import ev3dev.ev3 as ev3
from time   import sleep
from random import choice, randint



# Connect infrared and touch sensors.
ir = ev3.InfraredSensor(); assert ir.connected
#ts = TouchSensor();    assert ts.connected
# Put the infrared sensor into proximity mode.
ir.mode = 'IR-PROX'

m1 = ev3.LargeMotor('outA')
#m1.run_timed(time_sp=3000, duty_cycle_sp=75)

m2 = ev3.LargeMotor('outB')
#m2.run_timed(time_sp=3000, duty_cycle_sp=75)

# sample at:
#https://github.com/rhempel/ev3dev-lang-python/blob/develop/demo/auto-drive.py
# dennis.demidov@gmail.com

"""
    Start both motors. `run-direct` command will allow to vary motor
    performance on the fly by adjusting `duty_cycle_sp` attribute.
    """

m1.run_direct()
m2.run_direct()

    # for m in motors:
    #     m.run_direct()



# Stop both motors and reverse for 1.5 seconds.
    # `run-timed` command will return immediately, so we will have to wait
    # until both motors are stopped before continuing.
    #for m in motors:
    #    m.stop(stop_command='brake')
    #    m.run_timed(duty_cycle_sp=-50, time_sp=1500)

# When motor is stopped, its `state` attribute returns empty list.
    # Wait until both motors are stopped:
    # while any(m.state for m in motors):
    #     sleep(0.1)

# Run the robot
while True:

    # if ts.value():
    #     # We bumped an obstacle.
    #     # Back away, turn and go in other direction.
    #     backup()
    #     turn()
    #     start()

    # Infrared sensor in proximity mode will measure distance to the closest
    # object in front of it.
    distance = ir.value()

    if distance < 60:
        # Sound backup alarm.
        ev3.Sound.tone([(1000, 500, 500)] * 3)
        m1.stop()
        m2.stop()
        break

        # Path is clear, run at full speed.
        #dc = 90
    #else:
        # Obstacle ahead, slow down.
        #dc = 40

    #for m in motors:
    #    m.duty_cycle_sp = dc

    sleep(0.1)

exit()