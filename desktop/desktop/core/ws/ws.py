#!/usr/bin/python3
import json
import socket
HOST = '192.168.43.100'
PORT = 80
# client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
# client.connect((HOST,PORT))
# client.send('data'.encode())
# while True:
#     msg = client.recv(8)
#     msg = msg.decode('utf-8')

from PyQt5.QtCore import QThread


class Ws(QThread):
    data = []

    def add_data(self, x, y):
        self.data.append({'x':float(x), 'y': float(y)})
    
    def init_connection(self):
        self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.client.connect((HOST,PORT))
        

    def send_data(self):
        try:
            self.client.send((json.dumps(self.data)+'\n').encode())
        except ConnectionResetError:
            self.init_connection()
    
    def run(self) -> None:
        print('RUN method')
        self.init_connection()
        while True:
            msg = self.client.recv(1000)
            msg = msg.decode('utf-8')
            print(msg)