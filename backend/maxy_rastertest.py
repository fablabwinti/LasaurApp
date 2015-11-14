#!/usr/bin/env python
import driveboard as b
import sys, os, time, random
import numpy as np
from PIL import Image

thislocation = os.path.dirname(os.path.realpath(__file__))

feedrate = 4000 # mm/min
travel_feedrate = feedrate

# for final cut
cut_feedrate = 1200 # mm/min
cut_intensity = 70 # percent

ppmm = 10.0
ppmm_y = ppmm

testno = 0
if testno == 0:
    pulse_duration = 50 # laser IRQs (32us each)
    # test individual pulse lengths
    H = 30
    W = 400
    img = np.zeros((H, W), dtype='uint8')

    ppmm_y = 1.0/3.5
    for p in range(H):
        pulse_density = np.linspace(0, .25, W)
        line = p * np.diff(np.cumsum(pulse_density).astype(int))
        img[p,0] = pulse_duration # marker
        img[p,1:] = line
        #print img[p,:]

elif testno == 1:
    # test with black/white GIMP-dithered image
    pulse_duration = 5 # laser IRQs (32us each)
    filename = os.path.join(thislocation, 'testjobs', 'lasersaur_photo_dithered.png')
    img = Image.open(filename)
    img = img.convert('L') # grayscale
    img = np.array(img)
    img[img>0] = pulse_duration
elif testno == 2:
    # test with different gray levels
    pulse_duration = 10 # laser IRQs (32us each)
    filename = os.path.join(thislocation, 'testjobs', 'lasersaur_photo.png')
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
spacing = 10.0 # mm
leadin = True

execute_job = True


if leadin:
    # firmware constant
    ACCELERATION = 1800000.0 # mm/min^2, divide by (60*60) to get mm/sec^2
    accel_dist = 0.5 * feedrate**2 / ACCELERATION  * 1.1
else:
    accel_dist = 0.0

box_w = w + 2*spacing + 2*accel_dist
box_h = h + 2*spacing

def info():
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
    N = 60*4
    print 'queuing job...'
    for lineno, y in enumerate(np.arange(0.0, h, 1/ppmm_y)):
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
        x_end = x + N/ppmm
        #data = N*[0, 4, 4, 8]
        #data = ''.join([chr(i) for i in data[:N]])
        data = img[lineno]
        b.raster_line(x, y, x_end, y, data)
        x = x_end
        if accel_dist > 0:
            x += accel_dist
            b.move(x, y)
        
    # cut it out
    b.move(0, 0)
    b.feedrate(cut_feedrate)
    b.intensity(cut_intensity)
    b.move(box_w, 0)
    b.move(box_w, box_h)
    b.move(0, box_h)
    b.move(0, 0)
    b.intensity(0)
    print 'done, waiting for execution...'

    # wait for job to finish
    time.sleep(2.0)
    while True:
        s = b.status()
        #print s['feedrate'], s['intensity']
        #print s['pulses_per_mm'], s['pulse_duration']
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
    b.connect()
    assert b.connected()
    print b.status()
    run()
finally:
    b.close()



