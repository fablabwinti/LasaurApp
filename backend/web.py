#!/usr/bin/env python3
import logging
import tornado.options
import tornado.web
import tornado.websocket
import tornado.tcpserver
#from tornado.escape import json_encode, json_decode
from tornado.ioloop import PeriodicCallback
from tornado import gen

import build
import flash


class GcodeTCPServer(tornado.tcpserver.TCPServer):
    """TCP server with a line oriented gcode protocol
    """
    def __init__(self, board, **args):
        self.board = board
        super(GcodeTCPServer, self).__init__(**args)

    @gen.coroutine
    def handle_stream(self, stream, address):
        logging.info('incoming "gcode over tcp" connection from %r', address)
        try:
            stream.write((self.board.version + '\n').encode('utf-8'))
            while True:
                line = yield stream.read_until(b'\n')
                line = line.decode('utf-8', 'ignore').strip()
                if line:
                    resp = self.board.gcode_line(line) + '\n'
                    if resp.startswith('error:'):
                        logging.warning(resp[6:])
                    stream.write(resp.encode('utf-8'))
        except tornado.iostream.StreamClosedError:
            logging.info('closed "gcode over tcp" by client %r', address)

class FirmwareHandler(tornado.web.RequestHandler):
    """HTTP Build and flash API
    """
    def initialize(self, board, conf):
        self.board = board
        self.conf = conf
    def post(self, action):
        if action == 'build':
            try:
                firmware_name = "LasaurGrbl_from_src"
                build.build_firmware(firmware_name)
            except build.BuildFailed as e:
                self.write(str(e))
                self.set_status(500, 'Build Failed')
        elif action == 'flash' or action == 'flash_release':
            if action == 'flash':
                firmware_name = "LasaurGrbl_from_src"
            elif action == 'flash_release':
                firmware_name = "LasaurGrbl"
            try:
                self.board.disconnect('flashing new firmware')
                flash.flash_upload(self.conf['driveboard'], firmware_name)
                self.board.connect()
            except flash.FlashFailed as e:
                self.write(str(e))
                self.set_status(500, 'Flash Failed')
        elif action == 'reset':
            try:
                self.board.disconnect('reset requested')
                flash.reset_atmega(self.conf['driveboard'])
                self.board.connect()
            except flash.FlashFailed as e:
                self.write(str(e))
                self.set_status(500, 'Reset Failed')
        else:
            self.set_status(501, 'not implemented')

class StatusHandler(tornado.web.RequestHandler):
    """HTTP status requests
    """
    def initialize(self, board):
        self.board = board

    def get(self):
        self.write(self.board.get_status())

class StatusWebsocket(tornado.websocket.WebSocketHandler):
    """Websocket for status updates (to avoid HTTP GET polling)
    """
    clients = set()
    started = False

    def initialize(self, board):
        if not StatusWebsocket.started:
            def status_timer_cb():
                status = board.get_status()
                for client in StatusWebsocket.clients:
                    client.new_status(status)
            polling_interval = 200  # milliseconds
            PeriodicCallback(status_timer_cb, polling_interval).start()
            StatusWebsocket.started = True

    def open(self):
        StatusWebsocket.clients.add(self)

    def on_close(self):
        StatusWebsocket.clients.remove(self)

    def new_status(self, status):
        # we always send it (even if not changed) to allow the client
        # to detect the lack of messages with a simple timeout
        self.write_message(status)

    def check_origin(self, origin):
        return True  # anyone may listen to status changes

class ConfigHandler(tornado.web.RequestHandler):
    def initialize(self, board, conf):
        self.board = board
        self.conf = conf

    def get(self):
        c = self.conf
        res = dict(
            baudrate=c.get('driveboard', 'baudrate'),
            serial_port=c.get('driveboard', 'serial_port'),
            original_port=c.get('original', 'network_port'),
            )
        self.write(res)

class GcodeHandler(tornado.web.RequestHandler):
    def initialize(self, board):
        self.board = board

    def post(self):
        gcode = self.request.body
        # note: some code duplication with GcodeTCPServer
        for line in gcode.split(b'\n'):
            line = line.decode('utf-8', 'ignore').strip()
            if line:
                resp = self.board.gcode_line(line) + '\n'
                if resp.startswith('error:'):
                    errors = True
                    logging.warning(resp[6:])
                # TODO: Not sure if this non-json response is useful for javascript.
                #       Should probably also return HTTP error if disconnected, etc.
                self.write(resp.encode('utf-8'))

class WSHandler(tornado.websocket.WebSocketHandler):
    clients = set()

    def initialize(self, board):
        self.board = board

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        logging.info('The port-gate is open!')
        WSHandler.clients.add(self)

    def on_close(self):
        logging.info('The port-gate is closed.')
        WSHandler.clients.remove(self)

    def on_message(self, message):
        logging.info("got message %r", message)
        for line in message.split('\n'):
            if line:
                self.on_gcode(line)

    def on_gcode(self, line):
        print('executing gcode: %r' % line)
        resp = self.board.gcode_line(line)
        if resp.startswith('error:'):
            logging.warning(resp[6:])

    def check_origin(self, origin):
        # TODO: this is bad; we don't really want javascript from
        # random websites to use our machine?
        # In addition, we also should require authentication.
        return True
