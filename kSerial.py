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

    def __init__(self, port="COM3", baudrate=115200):
        self.s = serial.Serial()
        self.port = port
        self.baudrate = baudrate
        self.buf = bytes()

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
    def continuous(self, record = False):
        pkinfo = ([],)
        pkdata = ([],)
        pkcount = 0
        nbyte = self.get_bytes_available()
        if nbyte == 0:
            return pkinfo, pkdata, pkcount
        # add to buffer
        rd = self.read(nbyte)
        self.buf += rd
        # unpack
        pkinfo, pkdata, idx = self.unpack(self.buf)
        if pkinfo[0] != []:
            self.buf = self.buf[idx:]
        else:
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

    # TODO
    # pkinfo, pkdata, pkcount = ks.data(mode)
    # mode = get, reset

    # TODO
    # pkinfo, pkdata, pkcount = ks.observer(param, ktype, ack, second)

    # % [freq, tims] = s.packetObserver();        ->  calculate freq and run time
    # function varargout = packetObserver( s, varargin )
    #     s.tick.count = s.tick.count + s.recv.count;
    #     if (s.tick.timeunit == 0) || (s.tick.timeunit >= size(s.ks.data, 1))
    #         % use system clock to calculate
    #         caltype = 'system';
    #         dt = toc;
    #         freq = s.tick.count / dt;
    #         tims = s.tick.time + dt;
    #         if dt >= s.tick.period
    #             s.tick.count = 0;
    #             s.tick.time = tims;
    #             if s.tick.state > (1.5 / s.tick.period)
    #                 s.tick.freq = (1 - s.tick.alpha) * s.tick.freq + s.tick.alpha * freq;
    #             else
    #                 s.tick.freq = freq;
    #                 s.tick.state = s.tick.state + 1;
    #             end
    #             % reset time tick
    #             tic;
    #         end
    #     else
    #         % use packet sec/msc to calculate
    #         caltype = 'packet';
    #         tims = s.ks.data(s.tick.timeindex(1), end) + s.ks.data(s.tick.timeindex(2), end) * s.tick.timeunit;
    #         dt = tims - s.tick.time;
    #         freq = s.tick.count / dt;
    #         if dt >= s.tick.period
    #             s.tick.count = 0;
    #             s.tick.time = tims;
    #             if s.tick.state > (1.0 / s.tick.period)
    #                 s.tick.freq = (1 - s.tick.alpha) * s.tick.freq + s.tick.alpha * freq;
    #             else
    #                 s.tick.freq = freq;
    #                 s.tick.state = s.tick.state + 1;
    #             end
    #         end
    #     end
    #     freq = round(s.tick.freq);
    #     msc = fix((tims - fix(tims)) * 1000 + 1e-5);
    #     sec = mod(fix(tims), 60);
    #     min = fix(fix(tims) / 60);
    #     varargout = {freq, [tims, min, sec, msc], caltype};
    # end

    # TODO
    # getPacketLostRate

    # % [rate, lost, dc] = s.getLostRate();       ->  check lost packet
    # function varargout = getPacketLostRate( s, varargin )
    #     if (s.tick.timeunit == 0) || (s.tick.timeunit >= size(s.ks.data, 1))
    #         % use parameter bytes to check lost packet
    #         caltype = 'parameter';
    #         count = zeros(1, s.ks.lens);
    #         for i = 1 : s.ks.lens
    #             count(i) = s.getParameter(i, 'uint16');
    #         end
    #         dc = count(2:end) - count(1:end-1);
    #         dc(dc < 0) = dc(dc < 0) + 65536;
    #         lost = size(find(dc ~= 1), 2);
    #     else
    #         % use packet sec/msc to check lost packet
    #         caltype = 'packet';
    #         tc = fix(s.ks.data(s.tick.timeindex(1), :) / s.tick.timeunit) + s.ks.data(s.tick.timeindex(2), :);
    #         dc = tc(2 : end) - tc(1 : end - 1);
    #         res = find(dc > round(1000 / s.tick.freq));
    #         lost = size(res, 2);
    #     end
    #     rate = lost / (s.ks.lens - 1);
    #     varargout = { rate, lost, dc, caltype};
    # end

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

    # TODO
    # ks.set_baudrate(115200)
    # def set_baudrate(self, baudrate = 115200):
    #     pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_BAUDRATE, mode], 'R0', False)
    #     return pkinfo, pkdata, pkcount

    # TODO
    # ks.set_rate(100)
    # def set_rate(self, rate = 100):
    #     pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_RATE, mode], 'R0', False)
    #     return pkinfo, pkdata, pkcount

    # ks.set_mode(mode)
    def set_mode(self, mode = 0):
        pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_MDOE, mode], 'R0', False)
        return pkinfo, pkdata, pkcount

    # TODO
    # deviceid   = ks.get_value(self.KSCMD_R0_DEVICE_ID)
    # baudrate   = ks.get_value(self.KSCMD_R0_DEVICE_BAUDRATE)
    # updaterate = ks.get_value(self.KSCMD_R0_DEVICE_RATE)
    # devicemode = ks.get_value(self.KSCMD_R0_DEVICE_MDOE)
    # def get_value(self, item = 0, second = 0.05)):
    #     pkinfo, pkdata, pkcount = self.command([self.KSCMD_R0_DEVICE_GET, item], 'R0', True)
    #     return pkinfo, pkdata, pkcount

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

    # device = ks.i2c_scan(show, second)
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
        dev = pkdata[0]
        if show:
            s = f'  >> found {pkcount} device ...'
            for i in range(pkcount):
                s += f' {dev[i]:02X}'
            print(s)
        return dev

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
