import struct
import binascii
import serial
from msgdev import PeriodTimer

# data = [90, 80, 70, 50]

fst = struct.Struct('!2B')
line = ''


def crc8(x):
    poly = 0x84
    crc = 0x0
    for byte in x:
        crc = crc ^ ord(byte)
        for i in range(8):
            last = (0xFF & crc) & 1
            crc = (0xff & crc) >> 1
            if last == 1:
                crc = crc ^ poly
    return crc & 0xFF


def send_data(data):
    header = '\xff'
    tmp = fst.pack(data[0], data[1])
    crc_code = struct.pack('!B', crc8(tmp))
    # print binascii.hexlify(tmp), type(tmp)
    tmp = header + tmp
    tmp = tmp + crc_code
    print binascii.hexlify(tmp)
    return tmp


def read_data(ser):
    global line
    l = ser.readline()
    if l == '':
        print 2
        return
    line = line + l
    if line.endswith('\n'):
        print line
        line = ''


def main():
    Arduino_ser = serial.Serial(port='COM9', baudrate=9600, bytesize=8, 
        parity = 'N', stopbits=1, timeout=1)
    t = PeriodTimer(0.1)
    t.start()
    data = 0
    while True:
        with t:
            # data += 1
            # if data > 254:
            #     data = 0
            data = [10, 130]
            Arduino_ser.write(send_data(data))
            print data
            read_data(Arduino_ser)

if __name__ == '__main__':
    main()
    # send_data(20)
