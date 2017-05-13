"""raw serial communication with the driveboard

Implements the firmware binary serial protocol, tracks the firmware
command buffer, reports error and status.
"""

import os
import time
import ast
from collections import OrderedDict
import struct
import serial
import logging
from tornado.ioloop import IOLoop, PeriodicCallback

# firmware constants, need to match device firmware
# (maybe they should be reported by the firmware's superstatus)

TX_CHUNK_SIZE = 16  # number of bytes written to the device in one go
FIRMBUF_SIZE = 255-1  # the firmware sacrifices one byte to distinguish full from empty
RASTER_BYTES_MAX = 60
PULSE_SECONDS = 31.875e-6  # see laser.c
MINIMUM_PULSE_TICKS = 3  # unit: PULSE_SECONDS
MAXIMUM_PULSE_TICKS = 127  # unit: PULSE_SECONDS
ACCELERATION = 1800000.0  # mm/min^2, divide by (60*60) to get mm/sec^2

# "import" firmware protocol constants
markers_tx = {}
markers_rx = {}
name_to_marker = {}
def import_firmware_constants():
    def parse_headerfile():
        file_dir = os.path.dirname(__file__)
        for line in open(file_dir + '/../firmware/src/protocol.h'):
            if line.startswith('#define'):
                parts = line.split(maxsplit=2)
                if len(parts) == 3:
                    define, name, value = parts
                    assert value[0] == "'", line
                    value = ast.literal_eval('b'+value)[0]
                    yield name, value

    for name, value in parse_headerfile():
        prefix = name.split('_')[0]
        if prefix in ['CMD', 'PARAM']:
            markers_tx[value] = name
            name_to_marker[name] = value
        elif prefix in ['INFO', 'STOPERROR', 'STATUS']:
            markers_rx[value] = name
            name_to_marker[name] = value
    globals().update(name_to_marker)

import_firmware_constants()


class Driveboard:
    def __init__(self, serial_port, baudrate):
        self.serial_port = serial_port
        self.baudrate = baudrate

        self.io_loop = IOLoop.current()
        self.serial_write_queue = bytearray()
        self.device = None

        self.read_hist = bytearray()
        self.pdata = []

        self.firmbuf_used = 0
        self.firmbuf_queue = bytearray()
        self.paused = False
        self.jobsize = 0

        self.disconnect_reason = None
        self.firmver = None

        # always request status (while connected) every 100ms
        self.last_status_request = 0.0
        self.last_status_report = 0.0
        self.status_raw = OrderedDict()  # preserve knowledge which STOPERROR_* was first
        polling_interval = 100  # milliseconds
        PeriodicCallback(self._status_timer_cb, polling_interval).start()

        # for stop/resume timing
        self.fw_stopped = False
        self.fw_resuming = False

        # initialize self.status
        self._update_status()

    def reset_protocol(self):
        self.serial_write_queue.clear()
        self.read_hist.clear()
        self.firmbuf_queue.clear()
        self.firmbuf_used = 0
        self.pdata = []
        self.firmver = None
        self.send_command('CMD_RESET_PROTOCOL')
        self.send_command('CMD_SUPERSTATUS')

    def connect(self):
        if self.device is not None:
            return ''
        try:
            logging.info('opening serial port %r baudrate %s', self.serial_port, self.baudrate)
            self.device = serial.Serial(self.serial_port, self.baudrate)
            self.device.timeout = 0
            self.device.write_timeout = 0
            self.device.nonblocking()
            self.protocol_errors = 0
            self.io_loop.add_handler(self.device, self._serial_event, IOLoop.READ)
        except serial.SerialException as e:
            self.disconnect_reason = str(e)
            logging.error(e)
            self.device = None
            return str(e)

        self.reset_protocol()
        self.greeting_timeout = self.io_loop.call_later(2.0, self._on_greeting_timeout)
        self.disconnect_reason = None
        return ''

    def disconnect(self, reason):
        logging.error(reason)
        self.disconnect_reason = reason
        if self.device is None:
            return
        logging.info('disconnecting from serial port %r', self.serial_port)
        self.io_loop.remove_handler(self.device)
        self.device.close()
        self.device = None

    def is_connected(self):
        return bool(self.device)

    def get_disconnect_reason(self):
        return self.disconnect_reason or 'disconnected'

    def pause(self):
        self.paused = True

    def unpause(self):
        self.paused = False
        # start sending again
        self._send_fwbuf()

    def get_status(self):
        if self.last_status_report < time.time() - 0.5:
            # no firmware status updates (e.g. disconnected)
            # update local status without firmware information
            self._update_status()

        return self.status

    def _serial_event(self, fd, events):
        if events & IOLoop.READ:
            try:
                self._serial_read()
            except serial.SerialException:
                self.disconnect('could not read from serial port')
                raise
        if events & IOLoop.WRITE:
            self._serial_write()

    def _serial_write(self, data=b''):
        if not self.device:
            logging.warning('write ignored (device closed)')
            return

        if False:  # debug
            def pretty(i):
                if i < 128:
                    return i
                else:
                    return ord('^')
            data_pretty = bytes(pretty(i) for i in data)
            logging.info('_serial_write %r', data_pretty)

        for b in data:
            # by protocol send twice
            self.serial_write_queue.append(b)
            self.serial_write_queue.append(b)
        if self.serial_write_queue:
            n = self.device.write(self.serial_write_queue)

            # del self.serial_write_queue[:n]
            #
            # workaround for Python 3.4.2 bug
            # (bytearray del[] bug http://bugs.python.org/issue23985)
            self.serial_write_queue = self.serial_write_queue[n:]

            if self.serial_write_queue:
                # usually never reached (because fw_buffer < serial_buffer)
                #logging.warning('%d bytes still waiting in queue after tx', len(queue))
                self.io_loop.update_handler(self.device, IOLoop.READ | IOLoop.WRITE)
        else:
            self.io_loop.update_handler(self.device, IOLoop.READ)

    def _serial_read(self):
        data = self.device.read(2000)
        if not data:
            raise RuntimeError('no read data - maybe serial port was closed?')

        # for error diagnostics
        self.read_hist.extend(data)
        # del self.read_hist[:-80]
        #
        # workaround for Python 3.4.2 bug
        # (bytearray del[] bug http://bugs.python.org/issue23985)
        # self.read_hist.extend(data)
        self.read_hist = self.read_hist[-80:]

        print_hist = False

        for byte in data:
            name = markers_rx.get(byte, repr(byte))
            if byte < 32:  # flow
                if byte == CMD_CHUNK_PROCESSED:
                    self.firmbuf_used -= TX_CHUNK_SIZE
                    #logging.debug('chunk processed, firmbuf_used %d/%d', self.firmbuf_used, FIRMBUF_SIZE)
                    self._send_fwbuf(b'')
                    if self.firmbuf_used < 0:
                        logging.error('firmware buffer tracking too low (%d)', self.firmbuf_used)
                elif byte == STATUS_END:
                    self._on_status_end()
                else:
                    logging.warning('unhandled rx %r %s', bytes([byte]), name)
            elif 31 < byte < 91:  # stop error markers, info flags
                self.status_raw[byte] = True
            elif 96 < byte < 123:  # parameter
                if len(self.pdata) == 4:
                    value = (( (self.pdata[3]-128)*2097152
                             + (self.pdata[2]-128)*16384
                             + (self.pdata[1]-128)*128
                             + (self.pdata[0]-128))- 134217728)/1000.0
                    self.pdata = []
                    self._on_parameter(byte, value)
                else:
                    logging.error('not enough parameter data %s %d', name, len(self.pdata))
                    print_hist = True
            elif byte > 127:  # data
                if len(self.pdata) < 4:
                    self.pdata.append(byte)
                else:
                    logging.error("invalid data (rx history %s)")
                    self.pdata = []
                    print_hist = True
            else:
                logging.fatal('received invalid byte %d (firmware should never send this byte) (rx history %s)', byte, self.read_hist)
                print_hist = True

        if print_hist:
            hist = self.read_hist.decode('unicode-escape')
            logging.error('Last 80 bytes from firmware: %r', hist)

    def _on_parameter(self, byte, value):
        if byte == INFO_STARTUP_GREETING:
            self._on_startup_greeting(value)
        elif byte == INFO_VERSION:
            # superstatus, only received once
            self.firmver = value / 100.0
        else:
            self.status_raw[byte] = value

    def _on_status_end(self):
        self._update_status(self.status_raw)
        self.status_raw.clear()

        if self.fw_resuming:
            # The firmware may report "stopped" status once more after
            # we sent CMD_RESUME if it hasn't received CMD_RESUME yet.
            #
            # We do not want to discard any commands directly after
            # CMD_RESUME (e.g. CMD_HOMING) from the queue.
            self.fw_resuming = False
        else:
            self.fw_stopped = bool(self.status['stops'])

        if self.fw_stopped:
            # We discard our queue; the firmware will do the same while stopped.
            #
            # If we did not discard it, it could take several seconds
            # until the whole job is fully sent over the serial port
            # and discarded by the firmware. The user might press
            # "resume" while old commands are still being sent.
            self.firmbuf_queue.clear()

    def _update_status(self, status_raw={}):
        if status_raw:
            # new firmware status is available
            self.last_status_report = time.time()

        # Estimate "percent firmware buffer used"
        #
        # We don't know the exact state of the firmware buffer because
        # we receive a confirmation only every TX_CHUNK_SIZE bytes. But
        # for GUI purpose a "percent" display should show zero when idle.
        firmbuf_used_for_sure = max(0, self.firmbuf_used - TX_CHUNK_SIZE)
        firmbuf_percent = 100.0 * float(firmbuf_used_for_sure) / (FIRMBUF_SIZE - TX_CHUNK_SIZE)

        # Estimate "percent job done" (percent of bytes, not execution time)
        bytes_waiting = firmbuf_used_for_sure + len(self.firmbuf_queue) + len(self.serial_write_queue)
        if bytes_waiting == 0:
            job_percent = 100.0
            self.jobsize = 0
        else:
            self.jobsize = max(bytes_waiting, self.jobsize)
            job_percent = 100.0 * (1.0 - float(bytes_waiting) / self.jobsize)

        r = status_raw
        self.status = {
            # 'appver':conf['version'],
            'firmver': self.firmver,
            'ready': r.pop(INFO_IDLE_YES, False) and not self.firmbuf_queue,
            'paused': self.paused,
            'serial_connected': bool(self.device),
            # 'progress': TODO, # if self.job_size == 0: self._status['progress'] = 1.0 else: self._status['progress'] = round(SerialLoop.tx_pos/float(SerialLoop.job_size),3)
            'queue': {
                'firmbuf': self.firmbuf_used,
                'firmbuf_percent': round(firmbuf_percent, 2),
                'backend': len(self.firmbuf_queue) + len(self.serial_write_queue),
                'job_percent': round(job_percent, 2)
                },
            'pos': {
                'x': r.pop(INFO_POS_X, 0.0),
                'y': r.pop(INFO_POS_Y, 0.0),
                'z': r.pop(INFO_POS_Z, 0.0)
                },
            'underruns': r.pop(INFO_BUFFER_UNDERRUN, 0.0),
            'stackclear': r.pop(INFO_STACK_CLEARANCE, 999999.0),
            'delayed_microsteps': r.pop(INFO_DELAYED_MICROSTEPS, 0.0),

            'stops': [],  # list of active stop errors, first one first
            'error_report': '',  # either empty, or description of the current problem

            'info': {
                'door_open': r.pop(INFO_DOOR_OPEN, False),
                'chiller_off': r.pop(INFO_CHILLER_OFF, False)
                },
            # 'offset': [0.0, 0.0, 0.0],  # todo: super-status only; should track changes?
        }

        # process everything that was not pop()ed above
        for byte, value in r.items():
            name = markers_rx.get(byte, repr(byte))
            if name.startswith('STOPERROR_') and byte != STOPERROR_OK:
                reason = name.split('STOPERROR_')[1].lower()
                # e.g. limit_hit_x1 limit_hit_x2 limit_hit_y1 limit_hit_y2 serial_stop_request
                # see protocol.h for the full list
                # note: it is significant which STOPERROR_* was reported first (using OrderedDict)
                self.status['stops'].append(reason)
            else:
                logging.warning('unhandled marker_rx %r value %r', name, value)

        # check if driveboard has crashed or disconnected
        status_report_missing = False
        if self.device and self.last_status_report < time.time() - 0.5:
            # Because importing a large job (SVG or DXF) blocks the backend
            # we only report this error if we requested the status recently.
            if self.last_status_request < time.time() - 0.5:
                status_report_missing = True

        # generate summary error report
        report = ''
        if not self.device:
            report = 'disconnected from serial port'
            if self.disconnect_reason:
                report += ' - ' + self.disconnect_reason
        elif status_report_missing:
            report = 'last status update from driveboard is too old'
        elif self.status['stops']:
            stops = self.status['stops']
            report = 'stopped - ' + stops[0]
            if len(stops) > 1:
                report += ' (and also ' + ' '.join(stops[1:]) + ')'
        self.status['error_report'] = report

        previous_report = getattr(self, 'previous_error_report', '')
        if previous_report != report:
            self.previous_error_report = report
            logging.warning('error_report changed: %r --> %r', previous_report, report)

        # TODO maybe: push notifications to websocket right away

    def send_command(self, cmd):
        cmd = name_to_marker[cmd]
        if cmd < 32:
            # controls chars, handled directly in the serial ISR (not buffered)
            if cmd == CMD_RESUME:
                self.fw_stopped = False
                self.fw_resuming = True
                if 'rx_buffer_overflow' in self.status['stops'] or \
                   'transmission_error' in self.status['stops']:
                    # need to fix the buffer tracking first
                    self.reset_protocol()
            self._serial_write(bytes([cmd]))
        else:
            self._send_fwbuf(bytes([cmd]))

    def send_param(self, param, val):
        param = name_to_marker[param]
        # num to be [-134217.728, 134217.727], [-2**27, 2**27-1]
        # three decimals are retained
        num = int(round(((val+134217.728)*1000)))
        data = struct.pack(
            'BBBBB',
            (num&127)+128,
            ((num&(127<<7))>>7)+128,
            ((num&(127<<14))>>14)+128,
            ((num&(127<<21))>>21)+128,
            param)
        self._send_fwbuf(data)

    def send_raster_data(self, data):
        # data = numpy.frombuffer(data, 'uint8')
        # data.clip(0, 127)
        # data += 128
        # self._send_fwbuf(data.tobytes())
        data = bytearray(data)
        for i in range(len(data)):
            if data[i] > 127:
                data[i] = 127
            else:
                data[i] = data[i] + 128
        self._send_fwbuf(data)

    def _send_fwbuf(self, data=b''):
        if self.fw_stopped:
            # while stopped, the firmware will discard all queued
            # bytes anyway; keep the queues empty for clean resume
            return
        self.firmbuf_queue += data
        if self.paused:
            return

        # transfer firmbuf_queue to the driveboard (if space is available)
        available = FIRMBUF_SIZE - self.firmbuf_used

        if available > 0 and self.firmbuf_queue:
            out = self.firmbuf_queue[:available]
            # del self.firmbuf_queue[:available]
            #
            # workaround for Python 3.4.2 bug
            # (bytearray del[] bug http://bugs.python.org/issue23985)
            self.firmbuf_queue = self.firmbuf_queue[available:]
            self.firmbuf_used += len(out)
            self._serial_write(out)

    def _status_timer_cb(self):
        if self.device:
            self.send_command('CMD_STATUS')
            self.last_status_request = time.time()

    def _on_startup_greeting(self, value):
        if abs(value - 200.456) < 0.001:
            if self.greeting_timeout is not None:
                self.io_loop.remove_timeout(self.greeting_timeout)
                self.greeting_timeout = None
                logging.info('got firmware startup greeting')
                if self.firmbuf_used != 0:
                    self.disconnect('Startup error, firmbuf_used should remain empty during startup!')
            else:
                self.disconnect('Got firmware startup greeting. Unexpected firmware reset!')
        else:
            self.disconnect('Got invalid firmware startup greeting: %r' % repr(value))

    def _on_greeting_timeout(self):
        self.greeting_timeout = None
        if self.read_hist.startswith(b'# LasaurGrbl '):
            self.disconnect('Old LasaurGrbl firmware detected. Please flash the new one.')
        elif self.last_status_report < time.time() - 0.5:
            self.disconnect('No firmware startup greeting and no response to status request. (Wrong firmware?) Bytes received: %r' % self.read_hist)
        else:
            logging.info('Got no startup greeting, but firmware is responding to status requests.')
            if self.firmver:
                logging.info('Firmware version: %s', self.firmver)
            else:
                self.disconnect('Firmware did not report its version! Incompatible firmware?')
