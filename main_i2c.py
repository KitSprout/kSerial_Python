from kserial import kserial

ks = kserial('COM3', 115200)
ks.open()

# device id check
deviceid = ks.check()
print(f'  >> device check: {deviceid:X}')

# i2c scan device
address = ks.i2c_scan(True)

# i2c scan device reg
for i in range(len(address)):
    reg = ks.i2c_register(address[i], True)

ks.close()
