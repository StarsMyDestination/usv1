import struct
import binascii
from msgdev import MsgDevice, PeriodTimer
import socket
import math


fst = struct.Struct('!3B')
line = ''
host = "192.168.3.99"
# host = "192.168.4.1"
# host = "192.168.3.99"
port = 9000
# arduinoUrl = "socket://192.168.44.100"

msgSubConnect = 'tcp://127.0.0.1:5555'
msgPubBind = 'tcp://0.0.0.0:6666'

dataNum = 12


def constraint(num, lowerLimit, upperLimit):
    if num < lowerLimit:
        num = lowerLimit
    if num > upperLimit:
        num = upperLimit
    return num


def myMap(num, l1, u1, l2, u2):
    k = (u2 - l2) / (u1 - l1)
    tmp = l2 + k * (num - l1)
    return tmp


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
    tmp = fst.pack(data[0], data[1], data[2])
    crc_code = struct.pack('!B', crc8(tmp))
    # print binascii.hexlify(tmp), type(tmp)
    tmp = header + tmp
    tmp = tmp + crc_code
    print binascii.hexlify(tmp)
    return tmp


def dataRead(s):
    tmp = s.recv(1024)
    if tmp.startswith('#') and tmp.endswith('\n'):
        tmp = tmp.rstrip('\n')
        ps = tmp.split(',')
        tmp = ""
        if len(ps) != dataNum:
            print 'invalid length'
            print tmp
            return None
        else:
            motorMicroSec = int(ps[1])
            motorMaxMicroSec = int(ps[2])
            rudderAng = int(ps[3])
            voltage = float(ps[4])
            current = float(ps[5])
            power = float(ps[6])
            arduinoReadMark = int(ps[7])
            maxPower = float(ps[8])
            powerIsLimited = int(ps[9])
            autoFlag = int(ps[10])
            power_f = float(ps[11])
            return([motorMicroSec, motorMaxMicroSec, rudderAng, voltage,
                    current, power, arduinoReadMark, maxPower,
                    powerIsLimited, autoFlag, power_f])
    else:
        print 'invalid data'
        return None


def pubToVeristand(dev, data):
    dev.pub_set1('motorMicroSec', data[0])
    dev.pub_set1('motorMaxMicroSec', data[1])
    dev.pub_set1('rudderAng', data[2])
    dev.pub_set1('voltage', data[3])
    dev.pub_set1('current', data[4])
    dev.pub_set1('power', data[5])
    dev.pub_set1('arduinoReadMark', data[6])
    dev.pub_set1('maxPower', data[7])
    dev.pub_set1('powerIsLimited', data[8])
    dev.pub_set1('autoFlag', data[9])
    dev.pub_set1('power_f', data[10])


def subFromVeristand(dev):
    motorSpeed = dev.sub_get1('motorSpeed')
    rudderAng = dev.sub_get1('rudderAng')
    maxPower = dev.sub_get1('maxPower')
    if math.isnan(motorSpeed) or math.isnan(rudderAng):
        return ([100, 90, 0])
    motorSpeed = myMap(motorSpeed, -100, 100, 0, 200)
    motorSpeed = constraint(motorSpeed, 0, 200)
    motorSpeed = int(motorSpeed)
    rudderAng = myMap(rudderAng, -40, 40, 50, 130)
    rudderAng = constraint(rudderAng, 50, 130)
    rudderAng = int(rudderAng)
    maxPower = myMap(maxPower, 0, 20, 0, 255)
    maxPower = constraint(maxPower, 0, 254)
    maxPower = int(maxPower)
    return([motorSpeed, rudderAng, maxPower])


def main():
    arduinoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, )
    dev = MsgDevice()
    dev.open()
    dev.pub_bind(msgPubBind)
    dev.sub_connect(msgSubConnect)
    dev.sub_add_url('motorSpeed')
    dev.sub_add_url('rudderAng')
    dev.sub_add_url('maxPower')

    t = PeriodTimer(0.1)
    t.start()
    try:
        arduinoSocket.connect((host, port))
        while True:
            with t:
                # data = subFromVeristand(dev)
                data = [100, 90, 0]
                print data
                try:
                    arduinoSocket.send(dataSend(data))
                    dataFromArduino = dataRead(arduinoSocket)
                    print dataFromArduino
                except(socket.error):
                    arduinoSocket.close()
                    arduinoSocket.connect((host, port))
                print
                if dataFromArduino:
                    pubToVeristand(dev, dataFromArduino)
    except Exception, e:
        print Exception, ':', e
        raise
        # print 333333333
    finally:
        # print 2222222
        dev.close()
        arduinoSocket.close()


if __name__ == '__main__':
    main()
