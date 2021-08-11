from kserial import kserial


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
# ks = kserial(port='COM3', baudrate=115200)
ks = kserial(baudrate=115200)   # auto select comport
ks.open()

# device id
deviceid = ks.check()
print(f'  >> device check: {deviceid:X}')

i2cdevice(ks)
continuous(ks)

ksosc = ks.oscilloscope(ks)
raw = ksosc.run(
    # xlabel='samples',
    # ylabel='m/s^2',
    # yrange=[-40, 40],
    style=[
        ksosc.style(index=5, color='r', scale=9.81/8192, text='ax'),
        ksosc.style(index=6, color='g', scale=9.81/8192, text='ay'),
        ksosc.style(index=7, color='b', scale=9.81/8192, text='az')
    ]
)

ks.close()
