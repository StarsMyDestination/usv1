#!/usr/bin/env python
import zmq
import serial
from msgdev import MsgDevice, PeriodTimer
import logging
import struct

Serial_on = True
Local_test = False
Data_show = False

# encoder offset; when calibrate, set offset to 0, then get the zero position value as the offset
dogvane_offset = 111.6
sail_offset = 79.1
# voltage ratio
voltage_ratio = 25.394
T = 0.1  # Loop period 0.1 or 0.5

MSG_SUB_CONNECTS = ['tcp://127.0.0.1:55001', 'tcp://192.168.1.62:55001',  # for dev
                    'tcp://127.0.0.1:55005', 'tcp://192.168.1.62:55005']  # for jsdev
MSG_PUB_BINDS = 'tcp://0.0.0.0:55002'
Arduino_port_addr = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_854393133303516032C1-if00'
# Arduino_port_addr = 'COM7'


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


def sign(x):
    if x > 0:
        return 1
    elif x == 0:
        return 0
    else:
        return -1


class Servo():
    def __init__(self, prefix, limit, offset, dev, jsdev, tmdev):
        self.prefix = prefix
        self.limit = limit
        self.offset = offset
        self.dev = dev  # receive the data from Veristand
        self.jsdev = jsdev  # receive the data from Joystick
        self.tmdev = tmdev  # receive the data from Transmitter
        self.r_data = 0  # the data received
        self.c_data = 0  # the data computed to Arduino
        self.dev.sub_add_url(prefix)
        self.jsdev.sub_add_url('js.'+prefix)
        self.tmdev.sub_add_url(prefix)

    def update(self, Autoctrl_flag, Transmitter_flag):
        self.receive_data(Autoctrl_flag, Transmitter_flag)
        self.compute_pwm()

    def receive_data(self, Autoctrl_flag, Transmitter_flag):
        if Transmitter_flag == 1:
            r_data = self.tmdev.sub_get1(self.prefix)
        else:
            if Autoctrl_flag == 1:
                r_data = self.dev.sub_get1(self.prefix)
            else:
                r_data = self.jsdev.sub_get1('js.'+self.prefix)
                # send joystick data to veristand
                self.dev.pub_set1('js.'+self.prefix, r_data)
        self.r_data = r_data
        # print self.prefix, self.r_data

    def compute_pwm(self):
        if self.prefix == 'Motor_speed':
            # the speed of motor uses percentage to describe
            # -100% means maximum reverse speed, 100% means maximum speed
            c_data = int(90 + self.r_data*self.limit)
        else:
            if abs(self.r_data) > self.limit:
                self.r_data = sign(self.r_data)*self.limit
            if self.prefix == 'Rudder_ang':
                # right rudder is positive
                c_data = int(90 - self.r_data + self.offset)
            # right sail is positive
            # the valid input for sailservo is from 45 to 145
            elif self.prefix == 'Mainsail_ang':
                c_data = int(45 + (0.0024*self.r_data*self.r_data + 0.5298*abs(self.r_data) - 1.106) + self.offset)
                if c_data < 45:
                    c_data = 45
                # c_data = int(45 + self.r_data + self.offset)
            elif self.prefix == 'Foresail_ang':
                c_data = int(45 + 0.3*abs(self.r_data) + self.offset)
        self.c_data = c_data


class Arduino_READ():
    def __init__(self, ser_port, dev):
        self.logger = console_logger('Arduino to MIO')
        self.ser = ser_port
        self.dev = dev
        self.line = ''
        self.dogvane = 0
        self.mainsail = 0
        self.voltage = 0
        self.arduino_receive_flag = 0

    def update(self):
        l = self.ser.readline()
        if l == '':
            self.logger.info('Read Nothing! Reconnecting---')
            # self.ser.close()
            # self.ser.open()
            return
        self.line = self.line + l
        if self.line.endswith('\n'):
            self.parse_line()
            self.line = ''

    def parse_line(self):
        self.line = self.line.rstrip('\r\n')
        ps = self.line.split(',')
        # print 'Arduino_READ raw data', ps
        if len(ps) != 4:
            return
        # self.dogvane = int(ps[0])
        # self.mainsail = int(ps[1])
        # self.voltage = int(ps[2])
        # self.arduino_receive_flag = int(ps[3])
        try:
            self.dogvane = _Encoder(float(ps[0]), dogvane_offset)
            self.mainsail = _Encoder(float(ps[1]), sail_offset)
            self.voltage = round(float(ps[2])*voltage_ratio, 2)
            self.arduino_receive_flag = float(ps[3])

            self.upload()
        except(ValueError):
            pass

    def upload(self):
        self.dev.pub_set1('encoder_dogvane', self.dogvane)
        self.dev.pub_set1('encoder_mainsail', self.mainsail)
        self.dev.pub_set1('voltage', self.voltage)
        self.dev.pub_set1('arduino_receive_flag', self.arduino_receive_flag)


def _Encoder(encoder_deg, offset=0):
    # if the arduino not get the encoder data, then output 0
    if encoder_deg == 0:
        return 0
    encoder_deg = encoder_deg - offset
    if encoder_deg > 180:
        encoder_deg = encoder_deg - 360
    elif encoder_deg < -180:
        encoder_deg = encoder_deg + 360
    output = round(encoder_deg, 2)
    return output


def console_logger(name):
    # create logger
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    # create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # add formatter to ch
    ch.setFormatter(formatter)
    # add ch to logger
    logger.addHandler(ch)
    return logger


class MIO_TRANS():
    def __init__(self):
        self.dev_connect_flag = 0
        self.datashow_count = 0

        self.logger = console_logger('MIO-5251')
        self.dev_init()
        self.ser_init()
        self.Servo_init()

        self.fst = struct.Struct('!4H')

    def dev_init(self):
        self.jsdev = MsgDevice()
        self.dev = MsgDevice()
        self.tmdev = MsgDevice()
        self.jsdev.open()
        self.dev.open()
        self.tmdev.open()
        try:
            self.tmdev.sub_connect('tcp://127.0.0.1:55006')
            self.logger.info('tmdev sub connect tcp://127.0.0.1:55006')
            if Local_test is True:
                self.dev.sub_connect(MSG_SUB_CONNECTS[0])
                self.logger.info('dev sub connect '+MSG_SUB_CONNECTS[0])
                self.dev.pub_bind(MSG_PUB_BINDS)
                self.logger.info('dev pub bind '+MSG_PUB_BINDS)
                self.jsdev.sub_connect(MSG_SUB_CONNECTS[2])
                self.logger.info('jsdev sub connect '+MSG_SUB_CONNECTS[2])
            else:
                self.dev.sub_connect(MSG_SUB_CONNECTS[1])
                self.logger.info('dev sub connect '+MSG_SUB_CONNECTS[1])
                self.dev.pub_bind(MSG_PUB_BINDS)
                self.logger.info('dev pub bind '+MSG_PUB_BINDS)
                self.jsdev.sub_connect(MSG_SUB_CONNECTS[3])
                self.logger.info('jsdev sub connect '+MSG_SUB_CONNECTS[3])
            self.jsdev.sub_add_url('js.Autoctrl')
            self.tmdev.sub_add_url('Transmitter_flag')
        except(zmq.error.ZMQError):
            self.dev.close()
            self.jsdev.close()
            self.logger.info('Address already in use')
            raise

    def ser_init(self):
        if not Serial_on:
            return
        try:
            self.Arduino_ser = serial.Serial(Arduino_port_addr,
                                             baudrate=9600, timeout=1)
            self.logger.info(self.Arduino_ser.portstr+' open successfully!')
            self.Arduino_ser.flushInput()
            self.Arduino_read = Arduino_READ(self.Arduino_ser, self.dev)
        except(serial.serialutil.SerialException, OSError):
            self.dev.close()
            self.jsdev.close()
            self.logger.info('could not open port: '+Arduino_port_addr)
            raise

    def Servo_init(self):
        self.motor = Servo('Motor_speed', 30, 0, self.dev, self.jsdev, self.tmdev)
        self.rudder = Servo('Rudder_ang', 40, 4, self.dev, self.jsdev, self.tmdev)
        self.mainsail = Servo('Mainsail_ang', 80, 0, self.dev, self.jsdev, self.tmdev)
        self.foresail = Servo('Foresail_ang', 70, 0, self.dev, self.jsdev, self.tmdev)

    def update(self):
        self.MIO_pub_sub()
        self.Arduino_update()
        if not Data_show:
            return
        self.DataInfoShow()

    def MIO_pub_sub(self):
        self.js_Autoctrl = int(self.jsdev.sub_get1('js.Autoctrl'))
        self.dev.pub_set1('js.Autoctrl', self.js_Autoctrl)
        self.dev_connect_flag = not self.dev_connect_flag
        self.dev.pub_set1('dev_connect_flag', self.dev_connect_flag)
        self.Transmitter_flag = self.tmdev.sub_get1('Transmitter_flag')
        self.motor.update(self.js_Autoctrl, self.Transmitter_flag)
        self.rudder.update(self.js_Autoctrl, self.Transmitter_flag)
        self.mainsail.update(self.js_Autoctrl, self.Transmitter_flag)
        self.foresail.update(self.js_Autoctrl, self.Transmitter_flag)

        global Data_show
        if self.Transmitter_flag == 1:
            Data_show = False

    def Arduino_update(self):
        if not Serial_on:
            return
        self.Arduino_read.update()
        self.Arduino_write()

    def Arduino_write(self):
        header = '\xff\x01'
        tmp = self.fst.pack(self.motor.c_data, self.rudder.c_data,
                            self.mainsail.c_data, self.foresail.c_data)
        crc_code = struct.pack('!H', crc16(tmp))
        tmp = header + tmp
        tmp = tmp + crc_code
        self.Arduino_ser.write(tmp)

    def DataInfoShow(self):
        self.datashow_count += 1
        if self.datashow_count < 15:
            return
        self.datashow_count = 0

        self.logger.info('''MIO received data from bank:
            Autoctrl, Motor, Rudder, Mainsail, Foresail: '''
            + str([self.js_Autoctrl, self.motor.r_data, self.rudder.r_data, self.mainsail.r_data, self.foresail.r_data]))

        if not Serial_on:
            return
        self.logger.info('''Arduino to MIO data:)
            dogvane, mainsail, voltage, arduino_receive_flag: '''
            + str([self.Arduino_read.dogvane, self.Arduino_read.mainsail, self.Arduino_read.voltage, self.Arduino_read.arduino_receive_flag]))

        self.logger.info('''MIO to Arduino data: )
            motor_pwm, rudder_pwm, mainsail_pwm, foresail_pwm: '''
            + str([self.motor.c_data, self.rudder.c_data, self.mainsail.c_data, self.foresail.c_data]))
        print


def main():
    mio = MIO_TRANS()
    t = PeriodTimer(T)
    t.start()
    try:
        while True:
            with t:
                mio.update()
    except(Exception, KeyboardInterrupt):
        mio.dev.close()
        mio.jsdev.close()
        mio.tmdev.close()
        if Serial_on:
            mio.Arduino_ser.close()
        raise


if __name__ == "__main__":
    main()
