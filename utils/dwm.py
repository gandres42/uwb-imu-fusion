import serial
from multiprocessing import Process, Lock, Value, Array, Event
import time
import ctypes

class _CPosition(ctypes.Structure):
    _fields_ = [
        ("px", ctypes.c_double),
        ("py", ctypes.c_double),
        ("pz", ctypes.c_double),
        ("qf", ctypes.c_int)
    ]

class _CAnchor(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_char_p),
        ("dst", ctypes.c_double),
        ("qf", ctypes.c_int),
        ("px", ctypes.c_double),
        ("py", ctypes.c_double),
        ("pz", ctypes.c_double),
        ("pqf", ctypes.c_double),
    ]

class Position:
    def __init__(self, px, py, pz, qf) -> None:
        self.px = px
        self.py = py
        self.pz = pz
        self.qf = qf

class Anchor:
    def __init__(self, dst, qf, px, py, pz, pqf) -> None:
        self.dst = dst
        self.qf = qf
        self.px = px
        self.py = py
        self.pz = pz
        self.pqf = pqf

class dwm1001():
    def __init__(self, path: str) -> None:
        # open serial port, clear buffer
        self.s = serial.Serial(path, 115200, timeout=0.05)
        tmp_buf = None
        while tmp_buf != b'':
            tmp_buf = self.s.read(1)
        
        self.ekf = Event()
        self.update_rate = Value('d', .1)
        self.pos = Value(_CPosition)
        self.anch_count = Value('i', 0)
        self.anchs = Array(_CAnchor, 4)
        self.pos_lock = Lock()
        self.anch_lock = Lock()

        Process(target=self.__listen_thread__).start()
        time.sleep(self.update_rate.value)

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

            pos_x = int.from_bytes(v[0:4], byteorder='little', signed=True) * .001
            pos_y = int.from_bytes(v[4:8], byteorder='little', signed=True) * .001
            pos_z = int.from_bytes(v[8:12], byteorder='little', signed=True) * .001
            pos_qf = int.from_bytes([v[12]], byteorder='little')
            with self.pos_lock:
                self.pos.px = pos_x
                self.pos.py = pos_y
                self.pos.pz = pos_z
                self.pos.qf = pos_qf

            # distances header
            buf = self.s.read(3)
            t = int.from_bytes([buf[0]], byteorder='little')
            l = int.from_bytes([buf[1]], byteorder='little')
            vl = int.from_bytes([buf[2]], byteorder='little')

            # distances
            with self.anch_lock:
                self.anch_count.value = vl
                for i in range(vl):
                    buf = self.s.read(20)
                    addr = ''.join(format(byte, '02x') for byte in buf[1::-1])
                    dst = int.from_bytes(buf[2:6], byteorder='little') * .001
                    qf = int.from_bytes(buf[6:7], byteorder='little')
                    buf = buf[7:20]
                    pos_x = int.from_bytes(buf[0:4], byteorder='little', signed=True) * .001
                    pos_y = int.from_bytes(buf[4:8], byteorder='little', signed=True) * .001
                    pos_z = int.from_bytes(buf[8:12], byteorder='little', signed=True) * .001
                    pos_qf = int.from_bytes([buf[12]], byteorder='little')
                    self.anchs[i].dst = dst
                    self.anchs[i].qf = qf
                    self.anchs[i].px = pos_x
                    self.anchs[i].py = pos_y
                    self.anchs[i].pz = pos_z
            
            time.sleep(self.update_rate.value)

    def position(self) -> Position:
        with self.pos_lock:
            p = Position(self.pos.px, self.pos.py, self.pos.pz, self.pos.qf)
        return p

    
    def anchors(self) -> list[Anchor]:
        a = []
        with self.anch_lock:
            for i in range(self.anch_count.value):
                a.append(Anchor(
                    self.anchs[i].dst, 
                    self.anchs[i].qf, 
                    self.anchs[i].px, 
                    self.anchs[i].py, 
                    self.anchs[i].pz, 
                    self.anchs[i].pqf)
                )
        return a

    def close(self):
        self.ekf.set()