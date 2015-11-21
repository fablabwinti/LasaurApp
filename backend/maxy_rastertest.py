#!/usr/bin/env python
import driveboard as b
import sys, os, time, random
import numpy as np
from PIL import Image

thislocation = os.path.dirname(os.path.realpath(__file__))

# the usable speed limit seems to be around 15000
# (at about 20000 and higher the firmware sometimes seems to lock up)
feedrate = 8000 # mm/min
travel_feedrate = feedrate

# for final cut
cut_feedrate = 1200 # mm/min
cut_intensity = 70 # percent

# serial limit at 8000mm/min: 21 ppmm (calcuated assuming zero bytes for move commands)
#ppmm = 20.0 # @ 8000mm/min still quite okay!
#ppmm = 40.0 # @ 8000mm/min results in slowdown (baudrate limit) but raster pulses looks okay
#ppmm = 0.03
ppmm = 8.00
ppmm_y = ppmm

testno = 1
if testno == 0:
    OUTDATED
elif testno == 1:
    # test with black/white
    pulse_duration = 2 # laser IRQs (32us each)
    #filename = os.path.join(thislocation, 'testjobs', 'lasersaur_photo_dithered.png')
    filename = os.path.join(thislocation, 'testjobs', 'raster-test.png')
    img = Image.open(filename)
    img = img.convert('L') # grayscale
    img = np.array(img)
    img = 255-img # invert
    img[img<127] = 0
    img[img>127] = pulse_duration
elif testno == 2:
    OUTDATED
    # test with different gray levels
    pulse_duration = 10 # laser IRQs (32us each)
    filename = os.path.join(thislocation, 'testjobs', 'raster-test.png')
    img = Image.open(filename)
    img = img.convert('L') # grayscale
    img = np.array(img)
    print img, img.mean(), img.max()
    img = 255-img # invert
    img = img * (pulse_duration/255.0) + 0.5

img = img.astype('uint8')
print img, img.mean(), img.max()
#assert img.max() <= pulse_duration

w, h = img.shape[1] / ppmm, img.shape[0] / ppmm_y
spacing = 5.0 # mm
leadin = True

execute_job = True

if leadin:
    # firmware constant
    ACCELERATION = 1800000.0 # mm/min^2, divide by (60*60) to get mm/sec^2
    accel_dist = 0.5 * feedrate**2 / ACCELERATION  * 1.1
else:
    accel_dist = 0.0

def info():
    box_w = w + 2*spacing + 2*accel_dist
    box_h = h + 2*spacing
    print 'image size (pixels)', img.shape
    print 'image size (mm) %.1f, %.1f' % (w, h)
    print 'piece size (mm) %.1f, %.1f' % (box_w, box_h)
    print 'pixel size (mm) %.3f' % (1.0/ppmm)
    pulse_duration_us = pulse_duration * b.SHORTEST_PULSE_SECONDS / 1e-6
    print 'pulse duration %dus' % pulse_duration_us
    pixel_duration_us = 1.0/ppmm / (feedrate/60.0) / 1e-6
    print 'pixel duration %dus' % pixel_duration_us
    print 'black pixel laser power %.1f%%' % (100*pulse_duration_us/pixel_duration_us)
    print 'raster duration %.1f minutes (without acceleration)' % (2*len(img.flat)*pixel_duration_us * 1e-6 / 60)
    print 'acceleration distance %.1fmm' % (accel_dist)
    
def run():
    #b.stop()
    #b.unstop()
    b.homing()
    print 'queuing job...'

    b.air_on()
    for lineno, y in enumerate(np.arange(0.0, h, 1.0/ppmm_y)):
        y += spacing
        x = spacing
        b.feedrate(travel_feedrate)
        b.intensity(0)
        b.move(x, y)
        if accel_dist > 0:
            x += accel_dist
            b.move(x, y)
        b.feedrate(feedrate)
        N = int(w * ppmm)
        x += N/ppmm
        data = img[lineno]
        b.raster_move(x, y, data)
        if accel_dist > 0:
            x += accel_dist
            b.move(x, y)
        
    # cut it out
    x0 = accel_dist
    x1 = accel_dist + 2*spacing + w
    y0 = 0
    y1 = h + 2*spacing
    b.move(x0, y0)
    b.feedrate(cut_feedrate)
    b.intensity(cut_intensity)
    b.move(x1, y0)
    b.move(x1, y1)
    b.move(x0, y1)
    b.move(x0, y0)
    b.move(0,0)
    b.air_off()
    b.intensity(0)
    print 'done, waiting for execution...'

    # wait for job to finish
    time.sleep(2.0)
    while True:
        s = b.status()
        print s
        if s['stops']:
            print s
            raise RuntimeError
        if s['ready']:
            break
        time.sleep(2)
    print s

    print 'ready.'

info()
if not execute_job:
    sys.exit(0)
try:
    print 'starting in 3 seconds...'
    time.sleep(3)
    b.connect()
    assert b.connected()
    print b.status()
    run()
finally:
    b.close()



