#!/usr/bin/python3
import json
import socket
from time import time
from PyQt5.QtCore import QThread, pyqtSignal
import os
import sys
import errno
class Ws(QThread):
    HOST = '192.168.43.100'
    PORT = 80
    data = []
    start_logging = False
    log_file_name:str

    robot_position = pyqtSignal(str)

    def add_data(self, x, y):
        self.data.append({'x':float(x), 'y': float(y)})
    
    def init_connection(self):
        self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.client.connect((self.HOST,self.PORT))
        
    def send_data(self):
        self.log_file_name = self.get_script_path()
        try:
            self.client.send((json.dumps(self.data)+'\n').encode())
            self.start_logging = True
        except ConnectionResetError:
            self.init_connection()

    def get_script_path(self):
        path = os.path.dirname(os.path.realpath(sys.argv[0]))+f'/log/{time()}.txt'
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
            if self.start_logging:
                msg = self.client.recv(1000)
                msg = msg.decode('utf-8')
                self.robot_position.emit(msg)
                f = open(self.log_file_name, 'a')
                f.write(f'{time()},'+msg+'\n')
                f.close()