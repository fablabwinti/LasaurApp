#!/usr/bin/env python3
# Super Awesome LasaurGrbl python flash script.
#
# Copyright (c) 2011 Nortd Labs
# Open Source by the terms of the Gnu Public License (GPL3) or higher.

import os, sys, time, subprocess


thislocation = os.path.dirname(os.path.realpath(__file__))
resources_dir = os.path.abspath(os.path.join(thislocation, '..'))

class FlashFailed(Exception):
    pass

def run(s):
    print(s)
    status, output = subprocess.getstatusoutput(s)
    print(output)
    if status != 0:
        raise FlashFailed(s + '\n' + output)

def flash_upload(conf, firmware_name='LasaurGrbl'):
    FIRMWARE = os.path.join(resources_dir, "firmware", firmware_name + '.hex')

    # honor src/config.user.h if exists
    if os.path.exists(os.path.join(resources_dir, "firmware", 'src', 'config.user.h')):
        name, ext = os.path.splitext(FIRMWARE)
        FIRMWARE = name + '_user' + ext
        print("INFO: using %s" % FIRMWARE)
        if not os.path.exists(FIRMWARE):
            print("ERROR: first build 'config.user.h'-based firmware")

    assert os.path.exists(FIRMWARE), "ERROR: invalid firmware path"

    if not (conf['board'] == 'beaglebone' or conf['board'] == 'raspberrypi'):
        DEVICE = "atmega328p"
        PROGRAMMER = "arduino"
        BITRATE = conf['baudrate_avrdude']

        if sys.platform == "darwin":  # OSX
            AVRDUDEAPP    = os.path.join(resources_dir, "firmware/tools_osx/avrdude")
            AVRDUDECONFIG = os.path.join(resources_dir, "firmware/tools_osx/avrdude.conf")

        elif sys.platform == "win32": # Windows
            AVRDUDEAPP    = os.path.join(resources_dir, "firmware", "tools_win", "avrdude")
            AVRDUDECONFIG = os.path.join(resources_dir, "firmware", "tools_win", "avrdude.conf")

        elif sys.platform == "linux" or sys.platform == "linux2":  #Linux
            AVRDUDEAPP    = os.path.join(resources_dir, "/usr/bin/avrdude")
            AVRDUDECONFIG = os.path.join(resources_dir, "/etc/avrdude.conf")

        # call avrdude, returns 0 on success
        run('"%(dude)s" -c %(programmer)s -b %(bps)s -P %(port)s -p %(device)s -C "%(dudeconf)s" -Uflash:w:"%(firmware)s":i'
            % {'dude':AVRDUDEAPP, 'programmer':PROGRAMMER, 'bps':BITRATE, 'port':conf['serial_port'], 'device':DEVICE, 'dudeconf':AVRDUDECONFIG, 'firmware':FIRMWARE})

        # PROGRAMMER = "avrisp"  # old way, required pressing the reset button
        # os.system('%(dude)s -c %(programmer)s -b %(bps)s -P %(port)s -p %(device)s -C %(dudeconf)s -B 10 -F -U flash:w:%(firmware)s:i'
        #     % {'dude':AVRDUDEAPP, 'programmer':PROGRAMMER, 'bps':BITRATE, 'port':conf['serial_port], 'device':DEVICE, 'dudeconf':AVRDUDECONFIG, 'firmware':FIRMWARE})

        # fuse setting taken over from Makefile for reference
        #os.system('%(dude)s -U hfuse:w:0xd2:m -U lfuse:w:0xff:m' % {'dude':AVRDUDEAPP})

    elif conf['board'] == 'beaglebone' or conf['board'] == 'raspberrypi':
        # Make sure you have avrdude installed:
        # beaglebone:
        # opkg install libreadline5_5.2-r8.9_armv4.ipk
        # opkg install avrdude_5.10-r1.9_armv7a.ipk
        # get the packages from http://www.angstrom-distribution.org/repo/
        # raspberrypi:
        # sudo apt-get install avrdude

        AVRDUDEAPP    = "avrdude"
        AVRDUDECONFIG = "/etc/avrdude.conf"
        DEVICE = "atmega328p"
        PROGRAMMER = "arduino"    # use this for bootloader
        SERIAL_OPTION = '-P %(port)s' % {'port':conf['serial_port']}
        BITRATE = conf['baudrate_avrdude'] or "115200"

        command = ('"%(dude)s" -c %(programmer)s -b %(bps)s %(serial_option)s -p %(device)s -C "%(dudeconf)s" -Uflash:w:"%(product)s":i' %
                  {'dude':AVRDUDEAPP, 'programmer':PROGRAMMER, 'bps':BITRATE, 'serial_option':SERIAL_OPTION, 'device':DEVICE, 'dudeconf':AVRDUDECONFIG, 'product':FIRMWARE})

        ### Trigger the atmega328's reset pin to invoke bootloader

        if conf['board'] == 'beaglebone':
            print("Flashing from BeagleBone ...")
            # The reset pin is connected to GPIO2_7 (2*32+7 = 71).
            # Setting it to low triggers a reset.
            # echo 71 > /sys/class/gpio/export
            try:
                fw = open("/sys/class/gpio/export", "w")
                fw.write("%d" % (71))
                fw.close()
                fwb = open("/sys/class/gpio/export", "w")
                fwb.write("%d" % (73))
                fwb.close()
            except IOError:
                # probably already exported
                pass
            # set the gpio pin to output
            # echo out > /sys/class/gpio/gpio71/direction
            fw = open("/sys/class/gpio/gpio71/direction", "w")
            fw.write("out")
            fw.close()
            fwb = open("/sys/class/gpio/gpio73/direction", "w")
            fwb.write("out")
            fwb.close()
            # set the gpio pin low -> high
            # echo 1 > /sys/class/gpio/gpio71/value
            fw = open("/sys/class/gpio/gpio71/value", "w")
            fw.write("0")
            fw.flush()
            fwb = open("/sys/class/gpio/gpio73/value", "w")
            fwb.write("0")
            fwb.flush()
            time.sleep(0.5)
            fw.write("1")
            fw.flush()
            fw.close()
            fwb.write("1")
            fwb.flush()
            fwb.close()
            time.sleep(0.1)
        elif conf['board'] == 'raspberrypi':
            print("Flashing from Raspberry Pi ...")
            import _thread
            import RPi.GPIO as GPIO
            def trigger_reset():
                GPIO.setmode(GPIO.BCM)  # use chip pin number
                pinReset = 2
                GPIO.setup(pinReset, GPIO.OUT)
                GPIO.output(pinReset, GPIO.LOW)
                time.sleep(0.8)
                GPIO.output(pinReset, GPIO.HIGH)
                time.sleep(0.1)
            _thread.start_new_thread(trigger_reset, ())
            # GPIO.setmode(GPIO.BCM)  # use chip pin number
            # pinReset = 2
            # # GPIO.setup(pinReset, GPIO.OUT)
            # GPIO.output(pinReset, GPIO.LOW)
            # time.sleep(0.5)
            # GPIO.output(pinReset, GPIO.HIGH)
            # time.sleep(0.1)

        run(command)


def reset_atmega(conf):
    print("Resetting Atmega ...")
    if conf['board'] == 'beaglebone':
        try:
            fw = open("/sys/class/gpio/export", "w")
            fw.write("%d" % (71))
            fw.close()
            fwb = open("/sys/class/gpio/export", "w")
            fwb.write("%d" % (73))
            fwb.close()
        except IOError:
            pass
        fw = open("/sys/class/gpio/gpio71/direction", "w")
        fw.write("out")
        fw.close()
        fwb = open("/sys/class/gpio/gpio73/direction", "w")
        fwb.write("out")
        fwb.close()
        fw = open("/sys/class/gpio/gpio71/value", "w")
        fw.write("0")
        fw.flush()
        fwb = open("/sys/class/gpio/gpio73/value", "w")
        fwb.write("0")
        fwb.flush()
        time.sleep(0.2)
        fw.write("1")
        fw.flush()
        fw.close()
        fwb.write("1")
        fwb.flush()
        fwb.close()
    elif conf['board'] == 'raspberrypi':
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)  # use chip pin number
        pinReset = 2
        GPIO.setup(pinReset, GPIO.OUT)
        GPIO.output(pinReset, GPIO.LOW)
        time.sleep(0.2)
        GPIO.output(pinReset, GPIO.HIGH)
    else:
        raise NotImplementedError("forced reset only possible on beaglebone and raspberrypi")


def usb_reset_hack(conf):
    # Hack to reset usb (possibly linux-only), read flash with avrdude
    # This solves a problem on my dev machine where the serial connection
    # fails after replugging the usb arduino. It seems strictly related
    # to the USB stack on the Linux dev machine (possibly also on OSX or Win).
    # Note: This should be irrelevant on the Lasersaur/BBB.
    run("avrdude -p atmega328p -P "+conf['serial_port']+" -c arduino -U flash:r:flash.bin:r -q -q")


if __name__ == '__main__':
    import argparse, configparser
    parser = argparse.ArgumentParser(description='flash driveboard firmware')
    parser.add_argument('configfile', metavar='configfile.ini',
                        help='port and gpio config file (e.g. beaglebone.ini)')
    args = parser.parse_args()
    conf = configparser.ConfigParser()
    conf.read(args.configfile)
    try:
        flash_upload(conf['driveboard'])
    except FlashFailed:
        sys.exit(1)
