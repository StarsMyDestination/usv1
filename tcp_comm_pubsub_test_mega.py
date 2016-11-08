import struct
import binascii
from msgdev import MsgDevice, PeriodTimer
import socket
import math


fst = struct.Struct('!9B')
line = ''
host = "192.168.3.100"
# host = "192.168.4.1"
# host = "192.168.3.99"
port = 9000
# arduinoUrl = "socket://192.168.44.100"

msgSubConnect = 'tcp://127.0.0.1:7777'
msgPubBind = 'tcp://0.0.0.0:8888'


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
    tmp = fst.pack(data[0], data[1], data[2], data[3], data[4],
                   data[5], data[6], data[7], data[8])
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
        if len(ps) != 17:
            print 'invalid length'
            return
        else:
            motorMicroSec = int(ps[1])
            motorMaxMicroSec = int(ps[2])
            rudder0Ang = int(ps[3])
            rudder1Ang = int(ps[4])
            rudder2Ang = int(ps[5])
            rudder3Ang = int(ps[6])
            rudder4Ang = int(ps[7])
            rudder5Ang = int(ps[8])
            rudder6Ang = int(ps[9])
            voltage = float(ps[10])
            current = float(ps[11])
            power = float(ps[12])
            arduinoReadMark = int(ps[13])
            maxPower = float(ps[14])
            powerIsLimited = int(ps[15])
            autoFlag = int(ps[16])
            return([motorMicroSec, motorMaxMicroSec, rudder0Ang, rudder1Ang,
                    rudder2Ang, rudder3Ang, rudder4Ang, rudder5Ang, rudder6Ang,
                    voltage, current, power, arduinoReadMark,
                    maxPower, powerIsLimited, autoFlag])
    else:
        return None


def pubToVeristand(dev, data):
    dev.pub_set1('motorMicroSec', data[0])
    dev.pub_set1('motorMaxMicroSec', data[1])
    dev.pub_set1('rudder0Ang', data[2])
    dev.pub_set1('rudder1Ang', data[3])
    dev.pub_set1('rudder2Ang', data[4])
    dev.pub_set1('rudder3Ang', data[5])
    dev.pub_set1('rudder4Ang', data[6])
    dev.pub_set1('rudder5Ang', data[7])
    dev.pub_set1('rudder6Ang', data[8])
    dev.pub_set1('voltage', data[9])
    dev.pub_set1('current', data[10])
    dev.pub_set1('power', data[11])
    dev.pub_set1('arduinoReadMark', data[12])
    dev.pub_set1('maxPower', data[13])
    dev.pub_set1('powerIsLimited', data[14])
    dev.pub_set1('autoFlag', data[15])


def subFromVeristand(dev):
    motorSpeed = dev.sub_get1('motorSpeed')
    maxPower = dev.sub_get1('maxPower')
    rudderAngList = []
    for i in range(7):
        rudderAngList.append(dev.sub_get1('rudder' + str(i) + 'Ang'))
    # print rudderAngList
    if math.isnan(motorSpeed):
        return ([100, 0, 90, 90, 90, 90, 90, 90, 90])
    motorSpeed = myMap(motorSpeed, -100, 100, 0, 200)
    motorSpeed = constraint(motorSpeed, 0, 200)
    motorSpeed = int(motorSpeed)
    rudderAngList2 = []
    for rudderAng in rudderAngList:
        rudderAng = myMap(rudderAng, -40, 40, 50, 130)
        rudderAng = constraint(rudderAng, 50, 130)
        rudderAng = int(rudderAng)
        rudderAngList2.append(rudderAng)
    # print rudderAngList2
    maxPower = myMap(maxPower, 0, 20, 0, 255)
    maxPower = constraint(maxPower, 0, 254)
    maxPower = int(maxPower)
    return([motorSpeed, maxPower] + rudderAngList2)


def main():
    arduinoSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM, )
    dev = MsgDevice()
    dev.open()
    dev.pub_bind(msgPubBind)
    dev.sub_connect(msgSubConnect)
    dev.sub_add_url('motorSpeed')
    dev.sub_add_url('maxPower')
    for i in range(7):
        dev.sub_add_url('rudder' + str(i) + 'Ang')

    t = PeriodTimer(0.1)
    t.start()
    try:
        arduinoSocket.connect((host, port))
        while True:
            with t:
                data = subFromVeristand(dev)
                print data
                arduinoSocket.send(dataSend(data))
                dataFromArduino = dataRead(arduinoSocket)
                print dataFromArduino
                print
                if dataFromArduino:
                    pubToVeristand(dev, dataFromArduino)
    except Exception, e:
        print Exception, ':', e
        raise
    finally:
        dev.close()
        arduinoSocket.close()


if __name__ == '__main__':
    main()
