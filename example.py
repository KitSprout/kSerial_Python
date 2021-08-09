import sys
import glob
import serial
from kserial import kserial

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


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


def plotupdate(i, ks, lines, lines_value, line_label, width, ys, line_num):
    pkinfo, pkdata, pkcount, pkdt = ks.continuous()
    if pkcount != 0:
        for i in range(line_num):
            for k in range(pkcount):
                y_data = pkdata[k][i+5] / 8192.0 * 9.8
                ys[i].append(y_data)
            ys[i] = ys[i][-width:]
            lines[i].set_ydata(ys[i])
            lines_value[i].set_text(f'[{line_label[i]}] {y_data:9.5f} m/s^2')
    return lines[0], lines[1], lines[2]


def plotsingle(ks):
    # kserial mode
    ks.target_rate(100)
    ks.target_mode(1)
    # Parameters
    width = 100
    y_range = [-40, 40]
    line_num = 3
    line_label = ['X', 'Y', 'Z']
    line_style = ['r', 'g', 'b']
    # create figure
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim([0, width])
    ax.set_ylim(y_range)
    # create line
    xs = list(range(0, width))
    ys = []
    lines = []
    lines_value = []
    for i in range(line_num):
        ys.append([0]*width)
        lines.append(ax.plot(xs, ys[i], line_style[i], label=line_label[i])[0])
        lines_value.append(ax.text(0.82, 0.95-i*0.05, '', transform=ax.transAxes))
    len(lines)
    len(ys)
    plt.title('title')
    plt.xlabel('samples')
    plt.ylabel('data')
    ani = animation.FuncAnimation(fig, plotupdate, fargs=(ks, lines, lines_value, line_label, width, ys, line_num,),
        interval=0.001)
    # ani = animation.FuncAnimation(fig, plotupdate, fargs=(ks, lines, lines_value, line_label, width, ys, line_num,),
    #     interval=0.001, blit=True)
    plt.show()


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
plotsingle(ks)

ks.close()
