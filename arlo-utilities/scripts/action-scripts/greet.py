#!/usr/bin/env python


import freenect
import time

ctx = freenect.init()
device = freenect.open_device(ctx, 0)
freenect.set_tilt_degs(device, 25)
time.sleep(0.8)
freenect.set_tilt_degs(device, 5)
time.sleep(0.8)
freenect.set_tilt_degs(device, 25)
time.sleep(0.8)
freenect.set_tilt_degs(device, 5)
time.sleep(0.8)
freenect.set_tilt_degs(device, 25)
time.sleep(0.8)
freenect.set_tilt_degs(device, 5)
