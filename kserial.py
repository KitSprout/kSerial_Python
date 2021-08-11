#        __            ____
#       / /__ _  __   / __/                      __  
#      / //_/(_)/ /_ / /  ___   ____ ___  __ __ / /_ 
#     / ,<  / // __/_\ \ / _ \ / __// _ \/ // // __/ 
#    /_/|_|/_/ \__//___// .__//_/   \___/\_,_/ \__/  
#                      /_/   github.com/KitSprout    
#   
#   @file    kserial.py
#   @author  KitSprout
#   @brief   kserial packet format :
#            byte 1   : header 'K' (75)       [HK]
#            byte 2   : header 'S' (83)       [HS]
#            byte 3   : data type (4-bit)     [TP]
#            byte 4   : data bytes (12-bit)   [LN]
#            byte 5   : parameter 1           [P1]
#            byte 6   : parameter 2           [P2]
#            byte 7   : checksum              [CK]
#             ...
#            byte L-1 : data                  [DN]
#            byte L   : finish '\r' (13)      [ER]

import time
import struct
import sys
import glob
import serial
import numpy as np

class kserial:

    typesize = {
        'uint8'  :  1, 'uint16' :  2, 'uint32' :  4, 'uint64' :  8,
        'int8'   :  1, 'int16'  :  2, 'int32'  :  4, 'int64'  :  8,
        'half'   :  2, 'float'  :  4, 'double' :  8,
        'R0'     :  1, 'R1'     :  1, 'R2'     :  1, 'R3'     :  1, 'R4'     :  1,
        0 : 1,  1 : 2,  2 : 4,  3 : 8,
        4 : 1,  5 : 2,  6 : 4,  7 : 8,
        9 : 2, 10 : 4, 11 : 8,
        8 : 1, 12 : 1, 13 : 1, 14 : 1,  15: 1,
    }
    typeconv = {
        'uint8'  :  0, 'uint16' :  1, 'uint32' :  2, 'uint64' :  3,
        'int8'   :  4, 'int16'  :  5, 'int32'  :  6, 'int64'  :  7,
        'half'   :  9, 'float'  : 10, 'double' : 11,
        'R0'     :  8, 'R1'     : 12, 'R2'     : 13, 'R3'     : 14, 'R4'     : 15,
        0: 'uint8',  1: 'uint16', 2: 'uint32', 3: 'uint64',
        4: 'int8',   5: 'int16',  6: 'int32',  7: 'int64',
        9: 'half',  10: 'float', 11: 'double',
        8: 'R0',    12: 'R1',    13: 'R2',    14: 'R3',     15: 'R4'
    }
    typestructconv = {
        'uint8'  : 'B', 'uint16' : 'H', 'uint32' : 'I', 'uint64' : 'Q',
        'int8'   : 'b', 'int16'  : 'h', 'int32'  : 'i', 'int64'  : 'q',
        'half'   : 'e', 'float'  : 'f', 'double' : 'd',
        'R0'     : 'B', 'R1'     : 'B', 'R2'     : 'B', 'R3'     : 'B', 'R4'     : 'B',
        0 : 'B',  1 : 'H',  2 : 'I',  3 : 'Q',
        4 : 'b',  5 : 'h',  6 : 'i',  7 : 'q',
        9 : 'e', 10 : 'f', 11 : 'd',
        8 : 'B', 12 : 'B', 13 : 'B', 14 : 'B',  15: 'B',
    }
    typestr = {
        'uint8'  : 'U8',  'uint16' : 'U16', 'uint32' : 'U32', 'uint64' : 'U64',
        'int8'   : 'I8',  'int16'  : 'I16', 'int32'  : 'I32', 'int64'  : 'I64',
        'half'   : 'F16', 'float'  : 'F32', 'double' : 'F64',
        'R0'     : 'R0',  'R1'     : 'R1',  'R2'     : 'R2',  'R3'     : 'R3',  'R4'     : 'R4',
    }

    KSCMD_R0_DEVICE_ID          = 0xD0
    KSCMD_R0_DEVICE_BAUDRATE    = 0xD1
    KSCMD_R0_DEVICE_RATE        = 0xD2
    KSCMD_R0_DEVICE_MDOE        = 0xD3
    KSCMD_R0_DEVICE_GET         = 0xE3
    KSCMD_R2_TWI_SCAN_DEVICE    = 0xA1
    KSCMD_R2_TWI_SCAN_REGISTER  = 0xA2

    def __init__(self, port="", baudrate=115200):
        self.s = serial.Serial()
        if len(port) == 0:
            portlist = self.get_portlist()
            if len(portlist) > 0:
                self.port = portlist[0]
            else:
                print('not available com port')
                self.port = "COM0"
        else:
            self.port = port
        self.baudrate = baudrate
        self.buf = bytes()
        self.runtime = 0

    def open(self):
        self.s.port = self.port
        self.s.baudrate = self.baudrate
        self.s.open()
        self.clear()

    def close(self):
        self.s.close()

    def is_open(self):
        return self.s.is_open

    def get_bytes_available(self):
        return self.s.in_waiting

    def clear(self):
        self.s.reset_input_buffer()

    def write(self, data):
        self.s.write(data)

    def read(self, lens):
        return self.s.read(lens)

    def get_portlist(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        portlist = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                portlist.append(port)
            except (OSError, serial.SerialException):
                pass
        return portlist

    def find(self, packet, val):
        return [i for i, v in enumerate(packet) if v == val]

    def delay(self, second):
        time.sleep(second)

    def getinfo(self, packet, Ki):
        Td  = packet[Ki + 2]
        Ld  = packet[Ki + 3]
        P1d = packet[Ki + 4]
        P2d = packet[Ki + 5]
        CKd = packet[Ki + 6]
        return Td, Ld, P1d, P2d, CKd

    # param = bytes(2), ktype = string, data = int or list
    def pack(self, param = [], ktype = 'R0', data = []):
        # get data lens
        if data != []:
            if type(data) is int:
                lens = 1
                pdata = bytearray(struct.pack(self.typestructconv[ktype], data))
            else:
                lens = len(data)
                pdata = bytearray(struct.pack(self.typestructconv[ktype], data[0]))
                for i in range(1, lens):
                    pdata += bytearray(struct.pack(self.typestructconv[ktype], data[i]))
        else:
            lens = 0
        # pack
        packet_bytes = self.typesize[ktype] * lens
        packet_type = self.typeconv[ktype] * 16 + int(packet_bytes / 256)
        packet = bytes(b'KS')               # header 'KS'
        packet += bytes([packet_type])      # data type
        packet += bytes([packet_bytes])     # data bytes
        if param != []:
            packet += bytes(param)          # parameter 1, 2
        else:
            packet += bytes(2)
        checksum = (packet[2] + packet[3] + packet[4] + packet[5]) % 256
        packet += bytes([checksum])         # checksum
        if packet_bytes != 0:               # data ...
            packet += pdata
        packet += bytes(b'\r')              # finish '\r'
        info = packet_bytes, packet_type, param[0], param[1], checksum
        return info, packet

    # pkinfo, pkdata, idx = ks.unpack(packet)
    # pkinfo(1) : data length (bytes)
    # pkinfo(2) : data type
    # pkinfo(3) : parameter 1
    # pkinfo(4) : parameter 2
    # pkinfo(5) : checksum
    def unpack(self, packet = []):
        info = ([],)
        data = ([],)
        index = 0
        # empty data
        if packet == []:
            return info, data, index
        # conv to array
        packet = np.array(list(packet))
        packet_bytes = len(packet)
        # check 'KS'
        Ki = np.where(packet == 75)[0]
        Si = np.where(packet == 83)[0]
        Ki = np.array(sorted(list(set(Ki) & set(Si - 1))))
        if Ki.size == 0:
            return info, data, index
        # check last packet
        if Ki[-1] + 8 > packet_bytes:
            Ki = np.delete(Ki, -1)
            if Ki.size == 0:
                return info, data, index
        # get packet info and checksum
        Td, Ld, P1d, P2d, CKd = self.getinfo(packet, Ki)
        # check checksum
        checksum = (Td + Ld + P1d + P2d) % 256
        rm = np.where((CKd == checksum) != True)[0]
        if rm.size != 0:
            Ki = np.delete(Ki, rm)
            if Ki.size != 0:
                Td, Ld, P1d, P2d, CKd = self.getinfo(packet, Ki)
            else:
                return info, data, index
        # get type, lens, parameter
        packet_type = np.trunc(Td / 16).astype(int)
        packet_lens = Ld + (Td % 16) * 256
        # check last packet
        if Ki[-1] + packet_lens[-1] + 8 > packet_bytes:
            Ki = np.delete(Ki, -1)
            if Ki.size != 0:
                Td, Ld, P1d, P2d, CKd = self.getinfo(packet, Ki)
                packet_type = np.trunc(Td / 16).astype(int)
                packet_lens = Ld + (Td % 16) * 256
            else:
                return info, data, index
        # check '/r'
        Ed = packet[Ki + packet_lens + 7]
        rm = np.where((Ed == 13) != True)[0]
        if rm.size != 0:
            Ki = np.delete(Ki, rm)
            if Ki.size != 0:
                Td, Ld, P1d, P2d, CKd = self.getinfo(packet, Ki)
                packet_type = np.trunc(Td / 16).astype(int)
                packet_lens = Ld + (Td % 16) * 256
            else:
                return info, data, index
        # get packet data
        Ki = np.array(Ki)
        cnt = 0
        data = ()
        for i in Ki + 7:
            Dd = packet[i:i+packet_lens[cnt]]
            cnt = cnt + 1
            data += (list(Dd),)
        info = list(packet_lens), list(packet_type), list(P1d), list(P2d), list(CKd)
        index = Ki[-1] + packet_lens[-1] + 8
        return info, data, index

    # pkinfo, pkdata, pkcount = ks.send(param, ktype, data)
    def send(self, param = [], ktype = 'R0', data = []):
        pkinfo, pkdata = self.pack(param, ktype, data)
        pkcount = len(pkdata)
        if pkcount:
            self.write(pkdata)
        return pkinfo, pkdata, pkcount

    # pkinfo, pkdata, pkcount = ks.recv()
    # pkinfo(1) : data length (bytes)
    # pkinfo(2) : data type
    # pkinfo(3) : parameter 1
    # pkinfo(4) : parameter 2
    # pkinfo(5) : checksum
    # pkinfo(6) : data length
    def recv(self):
        pkinfo = ([],)
        pkdata = ([],)
        pkcount = 0
        nbyte = self.get_bytes_available()
        if nbyte == 0:
            return pkinfo, pkdata, pkcount
        pkinfo, pkdata, idx = self.unpack(self.read(nbyte))
        if pkinfo[0] == []:
            return pkinfo, pkdata, pkcount
        pkcount = len(pkdata)
        pkdatalens = []
        pk = ()
        for i in range(0, pkcount):
            if pkdata[i] != []:
                typesize = self.typesize[pkinfo[1][i]]
                lens = int(pkinfo[0][i] / typesize)
                pkdatalens.append(lens)
                pd = []
                for k in range(0, lens):
                    pd.append(struct.unpack(self.typestructconv[pkinfo[1][i]], bytes(pkdata[i][0:typesize]))[0])
                    del pkdata[i][0:typesize]
            else:
                pd = []
            pkinfo[1][i] = self.typeconv[pkinfo[1][i]]
            pk += (pd,)
        # add packet data lens to info
        pkinfo += (pkdatalens,)
        pkdata = pk
        return pkinfo, pkdata, pkcount

    # pkinfo, pkdata, pkcount = ks.continuous()
    # pkinfo(1) : data length (bytes)
    # pkinfo(2) : data type
    # pkinfo(3) : parameter 1
    # pkinfo(4) : parameter 2
    # pkinfo(5) : checksum
    # pkinfo(6) : data length
    def continuous(self, ts = True, record = False):
        pkinfo = ([],)
        pkdata = ([],)
        pkdt = []
        pkcount = 0
        nbyte = self.get_bytes_available()
        if nbyte == 0:
            return pkinfo, pkdata, pkcount, pkdt
        # add to buffer
        rd = self.read(nbyte)
        self.buf += rd
        # unpack
        pkinfo, pkdata, idx = self.unpack(self.buf)
        if pkinfo[0] != []:
            self.buf = self.buf[idx:]
        else:
            return pkinfo, pkdata, pkcount, pkdt
        pkcount = len(pkdata)
        pkdatalens = []
        pk = ()
        for i in range(0, pkcount):
            if pkdata[i] != []:
                typesize = self.typesize[pkinfo[1][i]]
                lens = int(pkinfo[0][i] / typesize)
                pkdatalens.append(lens)
                pd = []
                for k in range(0, lens):
                    pd.append(struct.unpack(self.typestructconv[pkinfo[1][i]], bytes(pkdata[i][0:typesize]))[0])
                    del pkdata[i][0:typesize]
            else:
                pd = []
            pkinfo[1][i] = self.typeconv[pkinfo[1][i]]
            pk += (pd,)
            if ts == True:
                now = pk[i][0] + pk[i][1] * 1e-3
            else:
                now = time.time()
            dt = now - self.runtime
            self.runtime = now
            pkdt.append(dt)
        # add packet data lens to info
        pkinfo += (pkdatalens,)
        pkdata = pk
        return pkinfo, pkdata, pkcount, pkdt

    # TODO
    # pkinfo, pkdata, pkcount = ks.data(mode)
    # mode = get, reset

    # TODO
    # pkinfo, pkdata, pkcount = ks.observer(param, ktype, ack, second)

    # TODO
    # getPacketLostRate

    # pkinfo, pkdata, pkcount = ks.command(param, ktype, ack, second)
    def command(self, param = [], ktype = 'R0', ack = False, second = 0.05):
        pkinfo, pkdata = self.pack(param, ktype)
        pkcount = len(pkdata)
        if ack:
            self.clear()
        if pkcount:
            self.write(pkdata)
            if ack:
                self.delay(second)
                pkinfo, pkdata, pkcount = self.recv()
        return pkinfo, pkdata, pkcount

    # deviceid = ks.check()
    def check(self):
        pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_ID, 0], 'R0', True)
        deviceid = pkinfo[3][0] * 256 + pkinfo[2][0]
        return deviceid

    # baudrate = ks.target_baudrate()
    # ks.target_baudrate(115200)
    # ks.target_baudrate(115200, True)
    def target_baudrate(self, baudrate = -1, resetting = False):
        if baudrate < 0:
            return self.target_get(self.KSCMD_R0_DEVICE_BAUDRATE)
        pkinfo, pkdata, pkcount = self.send([self.KSCMD_R0_DEVICE_BAUDRATE, 4], 'R0', struct.pack('I', int(baudrate)))
        # resetting baudrate
        if resetting == True:
            self.close()
            self.baudrate = baudrate
            self.delay(0.1)
            self.open()
        self.delay(0.1)
        return pkinfo, pkdata, pkcount

    # rate = ks.target_rate()
    # ks.target_rate(100)
    def target_rate(self, rate = -1, second = 0):
        if rate < 0:
            return self.target_get(self.KSCMD_R0_DEVICE_RATE)
        pkinfo, pkdata, pkcount = self.send([self.KSCMD_R0_DEVICE_RATE, 4], 'R0', struct.pack('I', int(rate)))
        self.delay(second)
        return pkinfo, pkdata, pkcount

    # mode = ks.target_mode()
    # ks.target_mode(mode)
    def target_mode(self, mode = -1, second = 0):
        if mode < 0:
            return self.target_get(self.KSCMD_R0_DEVICE_MDOE)
        pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_MDOE, mode], 'R0', False)
        self.delay(second)
        return pkinfo, pkdata, pkcount

    # deviceid = ks.target_get(self.KSCMD_R0_DEVICE_ID)
    # baudrate = ks.target_get(self.KSCMD_R0_DEVICE_BAUDRATE)
    # rate     = ks.target_get(self.KSCMD_R0_DEVICE_RATE)
    # mode     = ks.target_get(self.KSCMD_R0_DEVICE_MDOE)
    def target_get(self, item, second = 0.05):
        pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_GET, item], 'R0', True)
        value = struct.unpack('I', bytes(pkdata[0]))[0];
        return value

    # value = ks.i2c_read(address, reg, lens, show, second)
    def i2c_read(self, address, reg, lens = 1, show = False, second = 0.05):
        pkinfo, pkdata = self.pack([(address << 1) + 1, reg], 'R1', lens)
        self.clear()
        self.write(pkdata)
        self.delay(second)
        pkinfo, pkdata, pkcount = self.recv()
        if pkcount == 0 or pkinfo[0] == []:
            print('  >> i2c read device error ... no ack\n')
            return []
        return pkdata[0]

    # value = ks.i2c_write(address, reg, data, show, second)
    def i2c_write(self, address, reg, data = [], show = False, second = 0.05):
        pkinfo, pkdata = self.pack([(address << 1) + 0, reg], 'R1', data)
        self.write(pkdata)
        return pkdata

    # address = ks.i2c_scan(show, second)
    def i2c_scan(self, show = False, second = 0.05):
        pkinfo, pkdata = self.pack([self.KSCMD_R2_TWI_SCAN_DEVICE, 0], 'R2')
        self.clear()
        self.write(pkdata)
        self.delay(second)
        pkinfo, pkdata, pkcount = self.recv()
        if pkcount == 0 or pkinfo[0] == []:
            print('  >> i2c secn device error ... no ack\n')
            return [0xFF]
        if self.KSCMD_R2_TWI_SCAN_DEVICE != pkinfo[2][0]:
            print('  >> i2c secn device error ... wrong format\n')
            return [0xFF]
        address = pkdata[0]
        if show:
            devnum = len(address)
            s = f'  >> found {devnum} device ...'
            for i in range(devnum):
                s += f' {address[i]:02X}'
            print(s)
        return address

    # reg = ks.i2c_register(address, show, second)
    def i2c_register(self, address, show = False, second = 0.1):
        pkinfo, pkdata = self.pack([self.KSCMD_R2_TWI_SCAN_REGISTER, address << 1], 'R2')
        self.clear()
        self.write(pkdata)
        self.delay(second)
        pkinfo, pkdata, pkcount = self.recv()
        if pkcount == 0 or pkinfo[0] == []:
            print('  >> i2c secn register error ... no ack\n')
            return ([],)
        if self.KSCMD_R2_TWI_SCAN_REGISTER != pkinfo[2][0]:
            print('  >> i2c secn register error ... wrong format\n')
            return ([],)
        reg = pkdata[0]
        if show:
            idx = 0
            s  = f'  >> i2c secn address (0x{address:02X})\n\n'
            s += '       0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n'
            for i in range(0, 256, 16):
                s += f'  {i:02X}:'
                s += f' {reg[i+0]:02X} {reg[i+1]:02X} {reg[i+ 2]:02X} {reg[i+ 3]:02X} {reg[i+ 4]:02X} {reg[i+ 5]:02X} {reg[i+ 6]:02X} {reg[i+ 7]:02X}'
                s += f' {reg[i+8]:02X} {reg[i+9]:02X} {reg[i+10]:02X} {reg[i+11]:02X} {reg[i+12]:02X} {reg[i+13]:02X} {reg[i+14]:02X} {reg[i+15]:02X}\n'
            print(s)
        return reg

    # Need to install pyqtgraph and PySide6
    # pip install pyqtgraph
    # pip install PySide6
    class oscilloscope():
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore
        class style():
            def __init__(self, index, color='', scale=1.0, text=''):
                self.index = index
                self.color = color
                self.scale = scale
                self.text = text
        def __init__(self, kserial, posx=100, posy=100, width=640, height=360):
            self.ks = kserial
            self.app = self.pg.mkQApp('kSerial')
            self.plt = self.pg.plot()
            self.plt.setWindowTitle('kSerial Oscilloscope')
            self.plt.setGeometry(posx, posy, width, height)
            self.plt.showGrid(x=True, y=True, alpha=0.2)
            self.plt.enableAutoRange()
            self.ts = 0
        def update(self):
            pkinfo, pkdata, pkcount, pkdt = self.ks.continuous()
            if pkcount != 0:
                # record data
                # TODO: save all recv data
                # len(pkcount)
                for i in range(self.num):
                    self.raw[i][:-1] = self.raw[i][1:]
                    self.raw[i][-1] = pkdata[-1][i]
                # update curve
                text = ''
                for i in range(len(self.index)):
                    curvedata = self.raw[self.index[i]] * self.scale[i]
                    self.curve[i].setData(curvedata)
                    text += f' {self.curvetext[i]}: {curvedata[-1]: 10.4f}'
                # TODO: setPos()
                # udpate title
                tt = time.time()
                dt = tt - self.ts
                if dt > 0.05:
                    self.ts = tt
                    self.plt.setTitle(text)
                # update
                self.app.processEvents()
        def run(self, xlabel='samples', ylabel='', width=800, yrange=[], style=[style(0)]):
            self.plt.setLabel('bottom', xlabel)
            self.plt.setLabel('left', ylabel)
            if len(yrange) == 2:
                self.plt.setRange(xRange=[0, width], yRange=yrange)
            self.width = width
            # get packet size
            self.ks.target_mode(1)
            timeout, pkcount = 0, 0
            while pkcount == 0 and timeout < 5.0:
                pkinfo, pkdata, pkcount, pkdt = self.ks.continuous()
                timeout += 0.01
                time.sleep(0.01)
            # create buffer
            self.num = len(pkdata[0])
            self.raw = np.zeros([self.num, self.width])
            # create curve
            self.index = []
            self.pen = []
            self.curve = []
            self.scale = []
            self.curvetext = []
            for i in range(len(style)):
                self.index.append(style[i].index)
                self.pen.append(self.pg.mkPen(style[i].color, name=style[i].text))
                self.curve.append(self.plt.plot(pen=self.pen[i]))
                self.scale.append(style[i].scale)
                self.curvetext.append(style[i].text)
            # continuous mode
            self.ks.target_mode(1)
            # set timer and start
            timer = self.QtCore.QTimer()
            timer.timeout.connect(self.update)
            timer.start(0)
            self.pg.exec()
            # self.pg.show()
            self.ks.target_mode(0)
            return self.raw

# test
if __name__ == '__main__':
    print('')
    print('---- kserial test start ----')
    ks = kserial(baudrate=115200)
    ks.open()
    # check connection
    deviceid = ks.check()
    print(f'>> device check: {deviceid:X}')
    ksosc = ks.oscilloscope(ks)
    raw = ksosc.run(
        xlabel='samples',
        ylabel='m/s^2',
        yrange=[-40, 40],
        style=[
            ksosc.style(index=5, color='r', scale=9.81/8192, text='ax'),
            ksosc.style(index=6, color='g', scale=9.81/8192, text='ay'),
            ksosc.style(index=7, color='b', scale=9.81/8192, text='az')
        ]
    )
    # time.sleep(5)
    print('---- kserial test stop  ----')
    ks.close()
