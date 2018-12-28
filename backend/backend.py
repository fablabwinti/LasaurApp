#!/usr/bin/env python3
import logging
import tornado.options
import tornado.web
from tornado.ioloop import IOLoop
import os.path
import sys
import subprocess
import atexit
import argparse
import configparser

import hardware_init
from gcode import DriveboardGcode
import web

class Application(tornado.web.Application):
    def __init__(self, conf, board):
        handlers = [
            (r"/gcode", web.GcodeHandler, dict(board=board)),
            (r"/status", web.StatusHandler, dict(board=board)),
            (r"/ws/status", web.StatusWebsocket, dict(board=board)),
            (r"/firmware/(build|flash|flash_release|reset)", web.FirmwareHandler, dict(board=board, conf=conf)),
            (r"/config", web.ConfigHandler, dict(board=board, conf=conf)),
            (r"/(.*)", tornado.web.StaticFileHandler, {
                "path": "../frontend/admin",
                "default_filename": "index.html"
            }),
        ]
        settings = dict(
            # cookie_secret="__TODO:_GENERATE_YOUR_OWN_RANDOM_VALUE_HERE__",  #TODO: check if we need this (besides auth); requires html changes
            xsrf_cookies=False,  # TODO: should be true, make template, etc.
            debug=conf['backend'].getboolean('debug', False),
            serve_traceback=True,  # always serve tracebacks
            autoreload=False,  # avoid multiple calls to start_old_backend()
        )
        super(Application, self).__init__(handlers, **settings)

def start_old_backend(conf):
    orig_app = os.path.join(os.path.split(__file__)[0],
                            'original', 'original_app.py')
    cmd = ['python3', orig_app]
    if conf.getboolean('public', False):
        cmd.append('--public')
    if conf.getboolean('debug', False):
        cmd.append('--debug')
    cmd.extend(['--network-port', str(conf.getint('network_port', 4444))])
    logging.info('starting original backend:' + ' '.join(cmd))
    p = subprocess.Popen(cmd)
    atexit.register(p.terminate)


def main():
    parser = argparse.ArgumentParser(description='Lasersaur backend server')
    parser.add_argument('configfile', metavar='configfile.ini',
                        help='port and gpio config file (e.g. beaglebone.ini)')
    args = parser.parse_args()
    conf = configparser.ConfigParser()
    conf.read(args.configfile)

    hardware_init.init(conf['driveboard']['board'])

    tornado.options.parse_command_line()
    io_loop = IOLoop.current()

    board = DriveboardGcode(
        conf['driveboard']['serial_port'],
        conf['driveboard'].getint('baudrate'))
    board.connect()

    app = Application(conf, board)

    start_old_backend(conf['original'])

    public = conf.getboolean('backend', 'public')
    port = conf.getint('backend', 'network_port')
    if public:
        addr = ''
    else:
        addr = '127.0.0.1'
    app.listen(port, addr)

    io_loop.start()

if __name__ == "__main__":
    if not os.path.exists('../frontend/admin'):
        print('Directory ../frontend/admin does not exist! Running from the wrong directory?')
        sys.exit(1)
    elif not os.path.exists('../frontend/admin/bower_components'):
        print('You need to download the frontent dependencies!')
        print('Please run "bower install" in the ../frontend/admin/ directory.')
        sys.exit(1)
    main()
