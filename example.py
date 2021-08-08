import sys
import glob
import serial
from kserial import kserial


def port_list():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def i2cdevice(ks):
    # i2c scan device
    address = ks.i2c_scan(True)
    # i2c scan device reg
    reg = ks.i2c_register(address[0], True)


def continuous(ks):
    # mode
    ks.target_rate(100)
    ks.target_mode(1)
    # continuous read
    total = 0
    while total < 50:
        pkinfo, pkdata, pkcount, pkdt = ks.continuous()
        if pkcount != 0:
            total = total + pkcount
            pktype = pkinfo[1][-1]
            pklens = pkinfo[5][-1]
            pkparm = [pkinfo[2][-1], pkinfo[3][-1]]
            freq = 1 / pkdt[-1]
            s = f'[{total:6d}][{pkcount:3d}][{ks.typestr[pktype]}][{pkparm[0]:02X}:{pkparm[1]:02X}][{freq:4.0f}Hz]'
            for j in range(pklens):
                s += f' {pkdata[-1][j]:6d}'
            print(s)
    ks.target_mode(0)


# main process

ports = port_list()
if len(ports) < 1:
    print('not available com port')
    exit()

default_comport = ports[0]
default_baudrate = 115200

print(f'\n  >> open com port {default_comport}, baudrate {default_baudrate} bps')
ks = kserial(default_comport, default_baudrate)
ks.open()

# device id
deviceid = ks.check()
print(f'  >> device check: {deviceid:X}')

i2cdevice(ks)
continuous(ks)

ks.close()
