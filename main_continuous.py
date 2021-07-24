from kserial import kserial

ks = kserial('COM3', 115200)
ks.open()

# device id
deviceid = ks.check()
print(f'  >> device check: {deviceid:X}')

# mode
ks.set_mode(1)

# continuous read
total = 0
while total < 1000:
    pkinfo, pkdata, pkcount = ks.continuous()
    if pkcount != 0:
        total = total + pkcount
        pktype = pkinfo[1][-1]
        pklens = pkinfo[5][-1]
        pkparm = [pkinfo[2][-1], pkinfo[3][-1]]
        freq = 100
        s = f'[{total:6d}][{pkcount:3d}][{ks.typestr[pktype]}][{pkparm[0]:02X}:{pkparm[1]:02X}][{freq:4d}Hz]'
        for j in range(pklens):
            s += f' {pkdata[-1][j]:6d}'
        print(s)
ks.set_mode(0)

ks.close()
