import serial
from threading import Thread, Event
import time

class Position():
    def __init__(self, px, py, pz, qf) -> None:
        self.px = px
        self.py = py
        self.pz = pz
        self.qf = qf

class Anchor():
    def __init__(self, addr, dst, qf, px, py, pz, pqf) -> None:
        self.addr = addr
        self.dst = dst
        self.qf = qf
        self.px = px
        self.py = py
        self.pz = pz
        self.pqf = pqf

class dwm1001():
    def __init__(self, path: str) -> None:
        self.s = serial.Serial(path, 115200)
        self.ekf = Event()
        self.update_rate = .1
        self.pos: Position = Position(0, 0, 0, 0)
        self.anchs: list[Anchor] = []
        Thread(target=self.__listen_thread__).start()
        time.sleep(self.update_rate)

    def __listen_thread__(self):
        while not self.ekf.is_set():
            self.s.write(bytearray([0x0C, 0X00]))

            # header
            buf = self.s.read(3)
            t = int.from_bytes([buf[0]], byteorder='little')
            l = int.from_bytes([buf[1]], byteorder='little')
            v = int.from_bytes([buf[2]], byteorder='little')

            # position
            buf = self.s.read(2)
            t = int.from_bytes([buf[0]], byteorder='little')
            l = int.from_bytes([buf[1]], byteorder='little')
            v = self.s.read(l)

            pos_x = round(int.from_bytes(v[0:4], byteorder='little', signed=True) * .001, 3)
            pos_y = round(int.from_bytes(v[4:8], byteorder='little', signed=True) * .001, 3)
            pos_z = round(int.from_bytes(v[8:12], byteorder='little', signed=True) * .001, 3)
            pos_qf = int.from_bytes([v[12]], byteorder='little')
            self.pos = Position(pos_x, pos_y, pos_z, pos_qf)

            # distances header
            buf = self.s.read(3)
            t = int.from_bytes([buf[0]], byteorder='little')
            l = int.from_bytes([buf[1]], byteorder='little')
            vl = int.from_bytes([buf[2]], byteorder='little')

            # distances
            self.anchs = []
            for i in range(vl):
                buf = self.s.read(20)
                addr = ''.join(format(byte, '02x') for byte in buf[1::-1])
                dst = int.from_bytes(buf[2:6], byteorder='little') * .001
                qf = int.from_bytes(buf[6:7], byteorder='little')
                buf = buf[7:20]
                pos_x = round(int.from_bytes(buf[0:4], byteorder='little', signed=True) * .001, 3)
                pos_y = round(int.from_bytes(buf[4:8], byteorder='little', signed=True) * .001, 3)
                pos_z = round(int.from_bytes(buf[8:12], byteorder='little', signed=True) * .001, 3)
                pos_qf = int.from_bytes([buf[12]], byteorder='little')
                self.anchs.append(Anchor(addr, dst, qf, pos_x, pos_y, pos_z, pos_qf))
            
            time.sleep(self.update_rate)

    def position(self) -> Position:
        return self.pos
    
    def anchors(self):
        return self.anchs

    def close(self):
        self.ekf.set()