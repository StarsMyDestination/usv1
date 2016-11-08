import struct
import binascii
import serial
from msgdev import PeriodTimer

# data = [90, 80, 70, 50]

fst = struct.Struct('!4H')
line = ''


def crc16(x):
    poly = 0x8408
    crc = 0x0
    for byte in x:
        crc = crc ^ ord(byte)
        for i in range(8):
            last = (0xFFFF & crc) & 1
            crc = (0xffff & crc) >> 1
            if last == 1:
                crc = crc ^ poly
    return crc & 0xFFFF


def send_data(data):
    header = '\xff\x01'
    tmp = fst.pack(data, data+1, data+2, data+3)
    crc_code = struct.pack('!H', crc16(tmp))
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
    Arduino_ser = serial.Serial(port='COM3', baudrate=9600, bytesize=8, 
        parity = 'N', stopbits=1, timeout=1)
    t = PeriodTimer(0.1)
    t.start()
    data = 0
    while True:
        with t:
            data += 1
            if data > 360:
                data = 0
            Arduino_ser.write(send_data(data))
            print data, data+1, data+2, data+3
            read_data(Arduino_ser)

if __name__ == '__main__':
    main()
    # send_data(20)
