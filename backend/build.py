#!/usr/bin/env python3
# Super Awesome LasaurGrbl python flash script.
#
# Copyright (c) 2011 Nortd Labs
# Open Source by the terms of the Gnu Public License (GPL3) or higher.

import os, sys, subprocess, shutil

# Make sure you have the Arduino IDE installed (we've tested this on 022 and newer).
# While build.py does not use the IDE directly it makes use of its tool chain.
# On Linux all you need is the "arduino-core" package.
# Please verify the following locations are correct for you platform:

if sys.platform == "darwin":  # OSX
    AVRDUDEAPP    = "/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avrdude"
    AVRGCCAPP     = "/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avr-gcc"
    AVROBJCOPYAPP = "/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avr-objcopy"
    AVRSIZEAPP    = "/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avr-size"
    AVROBJDUMPAPP = "/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avr-objdump"
    AVRDUDECONFIG = "/Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/etc/avrdude.conf"

elif sys.platform == "win32": # Windows
    AVRDUDEAPP    = "C:\\arduino\\hardware\\tools\\avr\\bin\\avrdude"
    AVRGCCAPP     = "C:\\arduino\\hardware\\tools\\avr\\bin\\avr-gcc"
    AVROBJCOPYAPP = "C:\\arduino\\hardware\\tools\\avr\\bin\\avr-objcopy"
    AVRSIZEAPP    = "C:\\arduino\\hardware\\tools\\avr\\bin\\avr-size"
    AVROBJDUMPAPP = "C:\\arduino\\hardware\\tools\\avr\\bin\\avr-objdump"
    AVRDUDECONFIG = "C:\\arduino\\hardware\\tools\\avr\\etc\\avrdude.conf"

elif sys.platform == "linux" or sys.platform == "linux2":  #Linux
    AVRDUDEAPP    = "avrdude"
    AVRGCCAPP     = "avr-gcc"
    AVROBJCOPYAPP = "avr-objcopy"
    AVRSIZEAPP    = "avr-size"
    AVROBJDUMPAPP = "avr-objdump"
    AVRDUDECONFIG = "/etc/avrdude.conf"



# =============================================================================
# No need to edit anything below this line

class BuildFailed(Exception):
    pass

def run(s):
    print(s)
    status, output = subprocess.getstatusoutput(s)
    print(output)
    if status != 0:
        raise BuildFailed(s + '\n' + output)


def build_firmware(firmware_name="LasaurGrbl"):
    """Build the firmware and name it firmware_name.hex.
    The source code is assumed to be in ../firmware/src/
    And the hex file will be placed in ../firmware/
    Returns 0 on success.
    """

    thislocation = os.path.dirname(os.path.realpath(__file__))
    firmware_dir = os.path.join(thislocation, '..', 'firmware')
    source_dir = os.path.join(firmware_dir, 'src')

    cwd_temp = os.getcwd()
    os.chdir(source_dir)

    # honor src/config.user.h if exists
    if os.path.exists('config.user.h'):
        shutil.copy('config.h', 'config.orig.h')
        shutil.copy('config.user.h', 'config.h')
        firmware_name = firmware_name + '_user'
        print("INFO: using config.user.h")

    DEVICE = "atmega328p"
    CLOCK = "16000000"
    BUILDNAME = firmware_name
    OBJECTS  = ["main", "serial", "protocol", "planner", "sense_control", "laser", "stepper"]

    COMPILE = AVRGCCAPP + " -Wall -O2 -flto -DF_CPU=" + CLOCK + " -mmcu=" + DEVICE + " -I. -ffunction-sections" + " --std=c99"

    for fileobj in OBJECTS:
        run('%(compile)s -c %(obj)s.c -o %(obj)s.o' % {'compile': COMPILE, 'obj':fileobj})

    run('%(compile)s -o main.elf %(alldoto)s  -lm' % {'compile': COMPILE, 'alldoto':".o ".join(OBJECTS)+'.o'})

    run('%(objcopy)s -j .text -j .data -O ihex main.elf %(product)s.hex' % {'objcopy': AVROBJCOPYAPP, 'obj':fileobj, 'product':BUILDNAME})

    # command = '%(size)s *.hex *.elf *.o' % {'size':AVRSIZEAPP}
    run('%(size)s --mcu=%(mcu)s --format=avr *.elf' % {'size':AVRSIZEAPP, 'mcu':DEVICE})

    # run('%(objdump)s -t -j .bss main.elf' % {'objdump':AVROBJDUMPAPP})

    try:
        # honor src/config.user.h if exists
        if os.path.exists('config.user.h'):
            shutil.move('config.orig.h', 'config.h')
        ## clean after upload
        print("Cleaning up build files.")
        for fileobj in OBJECTS:
            f = '%s.o' % (fileobj)
            if os.path.isfile(f):
                os.remove(f)
        if os.path.isfile('main.elf'):
            os.remove('main.elf')

        ## move firmware hex file
        firmware_src = firmware_name+'.hex'
        firmware_dst = os.path.join(firmware_dir, firmware_src)
        print("Moving firmware to standard location", firmware_dst)
        shutil.move(firmware_src, firmware_dst)
    finally:
        #restore previous cwd
        os.chdir(cwd_temp)


if __name__ == '__main__':
    try:
        build_firmware()
    except BuildFailed:
        sys.exit(1)

