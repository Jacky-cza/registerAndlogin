import socket
import json
import struct
import re


class Agv:
    def __init__(self):
        self.ip = "192.168.1.2"
        self.state_port = 19204
        self.control_port = 19205
        self.nav_port = 19206
        self.other_port = 19210

    def query_robot_pose(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5)
        pose = {}
        packet = [0x5A, 0x01, 0x11, 0x22, 0x00, 0x00, 0x00, 0x00, 0x03, 0xEC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        # send
        s.connect((self.ip, self.state_port))
        s.send(bytearray(packet))
        # recv and regular check
        recv_data = bytes()
        regu = b'\\x5A\\x01\\x11\\x22.{4}\\x2A\\xFC\\x03\\xec\\x00{4}'
        while True:
            recv_data += s.recv(256)
            m = re.search(regu, recv_data)
            if m:
                barray = recv_data[m.start() + 4: m.start() + 8]
                size = struct.unpack(">i", barray)[0]

                if len(recv_data) >= (size + 16):
                    recv_data = recv_data[m.end():m.end() + size]
                    break

        json_obj = json.loads(recv_data.decode(encoding='utf-8'))
        pose['x'] = json_obj['x']
        pose['y'] = json_obj['y']
        pose['theta'] = json_obj['angle']
        pose['station'] = json_obj['current_station']
        s.close()
        return pose['station']

    def get_navigation_state(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.ip, self.state_port))
        packet = [0x5A, 0x01, 0x55, 0x66, 0x00, 0x00, 0x00, 0x00]
        packet = packet + [0x03, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        s.send(bytearray(packet))
        recv_data = bytes()
        regu = b'\\x5A\\x01\\x55\\x66.{4}\\x2B\\x0C\\x03\\xFC\\x00{4}'
        while True:
            recv_data += s.recv(256)
            m = re.search(regu, recv_data)
            if m:
                barray = recv_data[m.start() + 4: m.start() + 8]
                size = struct.unpack(">i", barray)[0]

                if len(recv_data) >= (size + 16):
                    recv_data = recv_data[m.end():m.end() + size]
                    break
        json_obj = json.loads(recv_data.decode(encoding='utf-8'))
        s.close()
        return json_obj["task_status"]

    def navigation(self, target, method=None):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.ip, self.nav_port))
        packet = [0x5A, 0x01, 0x33, 0x44]
        j = dict()
        j['id'] = target
        j['method'] = method
        str_tmp = json.dumps(j)
        packet = packet + list(self.trans(len(str_tmp))) \
                 + [0x0B, 0xEB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                 + list(bytearray(str_tmp, encoding='utf-8'))
        s.send(bytearray(packet))
        recv_data = bytes()
        regu = b'\\x5A\\x01\\x33\\x44.{4}\\x32\\xFB\\x0B\\xEB\\x00{4}'
        while True:
            recv_data += s.recv(256)
            m = re.search(regu, recv_data)
            if m:
                barray = recv_data[m.start() + 4: m.start() + 8]
                size = struct.unpack(">i", barray)[0]
                if len(recv_data) >= (size + 16):
                    recv_data = recv_data[m.end():m.end() + size]
                    break
        json_obj = json.loads(recv_data.decode(encoding='utf-8'))
        s.close()

    def cancle_navigation(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.ip, self.nav_port))
        packet = [0x5A, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00]
        packet += [0x0B, 0xBB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            s.send(bytearray(packet))
        except Exception as e:
            return False
        recv_data = bytes()
        regu = b'\\x5A\\x01\\x01\\x02.{4}\\x32\\xCB\\x0B\\xBB\\x00{4}'
        try:
            while True:
                recv_data += s.recv(256)
                m = re.search(regu, recv_data)
                if m:
                    barray = recv_data[m.start() + 4: m.start() + 8]
                    size = struct.unpack(">i", barray)[0]
                    if len(recv_data) >= (size + 16):
                        recv_data = recv_data[m.end():m.end() + size]
                        break
        except Exception as e:
            return False
        try:
            jsonObj = json.loads(recv_data.decode(encoding='utf-8'))
        except Exception as e:
            pass
        return True

    def confirm_location(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self.ip, self.control_port))
        packet = [0x5A, 0x01, 0x22, 0x44, 0x00, 0x00, 0x00, 0x00]
        packet = packet + [0x07, 0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        try:
            s.send(bytearray(packet))
        except Exception as e:
            return False
        recv_data = bytes()
        regu = b'\\x5A\\x01\\x22\\x44.{4}\\x2E\\xE3\\x07\\xD3\\x00{4}'
        try:
            while True:
                recv_data += s.recv(256)
                m = re.search(regu, recv_data)
                if m:
                    barray = recv_data[m.start() + 4: m.start() + 8]
                    size = struct.unpack(">i", barray)[0]
                    if len(recv_data) >= (size + 16):
                        recv_data = recv_data[m.end():m.end() + size]
                        break
            return True
        except Exception as e:
            pass

    def reLocation(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(10)
        s.connect((self.ip, self.control_port))
        packet = [0x5A, 0x01, 0x11, 0x33]
        j = dict()
        j['x'] = 0
        j['y'] = 0
        j['angle'] = 0
        j['home'] = True
        str_tmp = json.dumps(j)
        packet = packet + list(self.trans(len(str_tmp))) + [0x07, 0xD2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00] \
                 + list(bytearray(str_tmp, encoding='utf-8'))
        try:
            s.send(bytes(packet))
        except Exception as e:
            return False
        recv_data = bytes()
        regu = b'\\x5A\\x01\\x11\\x33.{4}\\x2E\\xE2\\x07\\xD2\\x00{4}'
        try:
            while True:
                recv_data += s.recv(256)
                m = re.search(regu, recv_data)
                if m:
                    # size = int.from_bytes(recvData[m.start() + 4: m.start() + 8], byteorder='big')
                    barray = recv_data[m.start() + 4: m.start() + 8]
                    size = struct.unpack(">i", barray)[0]
                    if len(recv_data) >= (size + 16):
                        recv_data = recv_data[m.end():m.end() + size]
                        break
        except Exception as e:
            pass
        try:
            json_obj = json.loads(recv_data.decode(encoding='utf-8'))
        except Exception as e:
            pass

    def trans(self, num):
        length = 4
        return ("%%0%dx" % (length << 1) % num).decode("hex")[-length:]
