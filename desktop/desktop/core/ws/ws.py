#!/usr/bin/python3
import json
import socket
from time import time
from PyQt5.QtCore import QThread, pyqtSignal
import os
import sys
import errno

from desktop.core.ws.data import Data


class Ws(QThread):
    HOST = '192.168.43.100'
    PORT = 80
    data = []
    start_logging = False
    log_file_name: str
    linear_speed = 0.2
    angular_speed = 0.6
    robot_position = pyqtSignal(Data)

    def add_data(self, x, y):
        self.data.append({'x': float(x), 'y': float(y)})

    def init_connection(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((self.HOST, self.PORT))

    def send_data(self):
        self.log_file_name = self.get_script_path()
        try:
            self.client.send((json.dumps({
                'type': 0,
                'data': self.data,
            })+'\n').encode())
            self.start_logging = True
        except ConnectionResetError:
            self.init_connection()

    def send_custom_data(self, msg):
        self.start_logging = False
        self.log_file_name = self.get_script_path()
        try:
            self.client.send((json.dumps(msg)+'\n').encode())
            self.start_logging = True
        except Exception as e:
            self.init_connection()

    def send_movement(self, direction):
        angular = {
            'turn_right': -self.angular_speed,
            'turn_left': self.angular_speed,
            'forward': 0,
            'backward': 0,
            'stop':0,
        }
        linear = {
            'turn_right': 0,
            'turn_left': 0,
            'forward': self.linear_speed,
            'backward': -self.linear_speed,
            'stop':0
        }
        payload = {
            'type': 1,
            'data': {
                'lin_speed': linear.get(direction),
                'ang_speed': angular.get(direction),
            }
        }
        self.client.send((json.dumps(payload)+'\n').encode())

    def send_reset(self):
        self.start_logging = False
        self.client.send((json.dumps({'type':2})+'\n').encode())

    def get_script_path(self):
        path = os.path.dirname(os.path.realpath(
            sys.argv[0]))+f'/log/{time()}.txt'
        if not os.path.exists(os.path.dirname(path)):
            try:
                os.makedirs(os.path.dirname(path))
            except OSError as exc:
                if exc.errno != errno.EEXIST:
                    raise
        return path

    def stop_logging(self):
        self.start_logging = False

    def run(self) -> None:
        print(self.get_script_path())
        self.init_connection()
        while True:
            msg = self.client.recv(150)
            msg = msg.decode('utf-8')
            try:
                self.robot_position.emit(Data(msg))
            except:
                pass
            if self.start_logging:
                print(msg)
                f = open(self.log_file_name, 'a')
                f.write(msg)
                f.close()