import struct
import binascii
from msgdev import PeriodTimer
import socket

# data = [90, 80, 70, 50]

fst = struct.Struct('!2B')
host = "192.168.44.100"
port = 9000


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


def dataSend(data):
    header = '\xff'
    tmp = fst.pack(data[0], data[1])
    crc_code = struct.pack('!B', crc8(tmp))
    # print binascii.hexlify(tmp), type(tmp)
    tmp = header + tmp
    tmp = tmp + crc_code
    print binascii.hexlify(tmp)
    return tmp


def dataRead(s):
    tmp = s.recv(1024)
    if tmp == '':
        print 2
        return
    if tmp.endswith('\n'):
        print tmp
        tmp = tmp.rstrip('\n')
        ps = tmp.split(',')
        tmp = ""
        if len(ps) != 7:
            return
        else:
            motorMicroSec = int(ps[0])
            rudderAng = int(ps[1])
            voltage = float(ps[2])
            current = float(ps[3])
            power = float(ps[4])
            arduinoReadMark = bool(ps[5])
            autoFlag = bool(ps[6])
            return([motorMicroSec, rudderAng, voltage, current, power,
                    arduinoReadMark, autoFlag])


def main():
    arduinoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    arduinoSocket.connect((host, port))
    # arduinoSocket.setblocking(0)
    t = PeriodTimer(0.1)
    t.start()
    count = 0
    num1 = 105
    num2 = 50
    while True:
        try:
            with t:
                count += 1
                num1 += 1
                num2 += 1
                if num1 > 120:
                    num1 = 100
                if num2 > 130:
                    num2 = 50
                print count
                data = [num1, num2]
                arduinoSocket.send(dataSend(data))
                print data
                print dataRead(arduinoSocket)
        except(Exception, KeyboardInterrupt):
            arduinoSocket.close()


if __name__ == '__main__':
    main()
