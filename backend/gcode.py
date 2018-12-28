import driveboard
import pulseraster
import json
import re
import base64

class DriveboardGcode:
    version = '# LasaurGrbl2 (pulseraster)'

    def __init__(self, serial_port, baudrate):
        self.driveboard = driveboard.Driveboard(serial_port, baudrate)

        # modal state of the gcode protocol (last used parameter)
        self.relative = False
        # defaults only used of never set by gcode
        self.feedrate = 6000
        self.seekrate = 1500

    def connect(self):
        self.driveboard.connect()

    def disconnect(self, reason):
        self.driveboard.disconnect(reason)

    def is_connected(self):
        return self.driveboard.is_connected()

    def get_status(self):
        return self.driveboard.get_status()

    def special_line(self, line):
        # those commands work even when disconnected:
        if line == '!' or line == '!stop':
            # instant stop
            self.driveboard.send_command('CMD_STOP')
            return 'ok'
        elif line == '~' or line == '!resume':
            # recover from all stop conditions
            error = self.driveboard.connect()
            if error: return 'error:' + error
            self.driveboard.send_command('CMD_RESUME')
            self.driveboard.unpause()
            return 'ok'
        elif line == '!pause':
            self.driveboard.pause()
            return 'ok'
        elif line == '!unpause':
            self.driveboard.unpause()
            return 'ok'
        else:
            return 'error:invalid command'

    def gcode_line(self, line):
        line = line.split(';')[0].strip()  # remove gcode comments
        if not line:
            return ''

        if not self.driveboard.is_connected():
            return 'error:' + self.driveboard.get_disconnect_reason()

        if self.driveboard.fw_stopped:
            # firmware is discarding all queue commands, purging the current jobdata
            # so don't waste time parsing it
            return 'ok'

        args = {}

        # parse and remove raster data
        if ' D' in line:
            try:
                line, data = line.split(' D')
                data = base64.b64decode(data)
                args['D'] = data
            except:
                return 'error: invalid base64 encoded data in gcode %r' % line

        # extract gcode parameters
        parts = re.split(r'([A-Z])', line)
        if parts[0] != '' or len(parts) < 3:
            return 'error:unknown gcode %r' % line
        try:
            # this way, we support both G00 and G0
            cmd = (parts[1] + str(int(parts[2]))).strip()
        except ValueError:
            return 'error:could not parse int in %r' % line
        parts = parts[3:]

        while parts:
            letter = parts.pop(0).strip()
            try:
                value = float(parts.pop(0))
            except ValueError:
                return 'error:could not parse float in %r' % line
            args[letter] = value

        # result of parsing
        params = []
        command = None
        raster_data = None
        intensity_value = None

        if cmd in ('G0', 'G1', 'G7'):
            # move (G0: without lasing; G1: with lasing)
            if 'X' in args: params.append(('PARAM_TARGET_X', args.pop('X')))
            if 'Y' in args: params.append(('PARAM_TARGET_Y', args.pop('Y')))
            if 'Z' in args: params.append(('PARAM_TARGET_Z', args.pop('Z')))

            if cmd == 'G0':
                self.seekrate = args.pop('F', self.seekrate)
                params.append(('PARAM_FEEDRATE', self.seekrate))
            else:
                self.feedrate = args.pop('F', self.feedrate)
                params.append(('PARAM_FEEDRATE', self.feedrate))

            if cmd == 'G0':
                command = 'CMD_LINE_SEEK'
            elif cmd == 'G1':
                command = 'CMD_LINE_BURN'
                intensity_value = args.pop('S', None)
            elif cmd == 'G7':
                command = 'CMD_LINE_RASTER'
                if args.pop('V') != 1:
                    return 'error:G7 command of unknown version'
                raster_data = args.pop('D')
                if not raster_data:
                    return 'error:G7 command without raster data'
                if len(raster_data) > driveboard.RASTER_BYTES_MAX:
                    return 'error:G7 command only implemented for at most %d bytes of raster data' % driveboard.RASTER_BYTES_MAX
                params.append(('PARAM_RASTER_BYTES', len(raster_data)))

        elif cmd == 'G90':
            command = 'CMD_REF_ABSOLUTE'
            self.relative = False
        elif cmd == 'G91':
            command = 'CMD_REF_RELATIVE'
            self.relative = True
        elif cmd == 'M80': command = 'CMD_AIR_ENABLE'
        elif cmd == 'M81': command = 'CMD_AIR_DISABLE'
        elif cmd == 'M82': command = 'CMD_AUX1_ENABLE'
        elif cmd == 'M83': command = 'CMD_AUX1_DISABLE'
        elif cmd == 'M84': command = 'CMD_AUX2_ENABLE'
        elif cmd == 'M85': command = 'CMD_AUX2_DISABLE'
        elif cmd == 'G10':
            p = args.pop('P', None)
            if p == 0:  # set table offset (G54)
                which = 'TABLE'  # (never used by current frontend)
            elif p == 1:  # set custom offset (G55)
                which = 'CUSTOM'
            else:
                return 'error:set_offset G10 requires P0 or P1 parameter'

            l = args.pop('L', None)
            if l == 20:  # L20 - set to current location
                command = 'CMD_SET_OFFSET_' + which
            elif l != 2:
                return 'error:set_offset G10 requires L2 or L20 parameter'
            else:  # L2 - set to value
                if 'X' in args: params.append(('PARAM_OFF' + which + '_X', args.pop('X')))
                if 'Y' in args: params.append(('PARAM_OFF' + which + '_Y', args.pop('Y')))
                if 'Z' in args: params.append(('PARAM_OFF' + which + '_Z', args.pop('Z')))
                command = None  # sending the parameters is enough
        elif cmd == 'G54':
            command = 'CMD_SEL_OFFSET_TABLE'
        elif cmd == 'G55':
            command = 'CMD_SEL_OFFSET_CUSTOM'
        elif cmd == 'G30':
            command = 'CMD_HOMING'
        elif cmd[0] == 'S':
            # set intensity
            intensity_value = line[1:]
        else:
            return 'error:unknown gcode command %r' % line
        if args:
            return 'error:unknown arguments %r in gcode line %r' % (args, line)

        if intensity_value is not None:
            try:
                intensity = float(intensity_value)
            except ValueError:
                return 'error:invalid intensity %r' % line
            if intensity < 0 or intensity > 255:
                return 'error:intensity out of range (0-255) %r' % line
            frequency, duration = pulseraster.intensity2pulse(intensity)
            params.append(('PARAM_PULSE_FREQUENCY', frequency))
            params.append(('PARAM_PULSE_DURATION', duration))

        # execute

        for name, value in params:
            self.driveboard.send_param(name, value)
        if command:
            self.driveboard.send_command(command)
        if raster_data:
            self.driveboard.send_raster_data(raster_data)
        return 'ok'
