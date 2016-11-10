#include <Servo.h>
#include <FlexiTimer2.h>
#include <EEPROM.h>
//servo configure
Servo motor, rudder0, rudder1, rudder2, rudder3, rudder4, rudder5, rudder6;
int motorMicroSec = 1500, motorMicroSecOld = 1500, motorMaxMicroSec = 1500;
int rudder0Ang = 140, rudder0AngOld = 140;
int rudder1Ang = 140, rudder1AngOld = 140;
int rudder2Ang = 140, rudder2AngOld = 140;
int rudder3Ang = 140, rudder3AngOld = 140;
int rudder4Ang = 140, rudder4AngOld = 140;
int rudder5Ang = 140, rudder5AngOld = 140;
int rudder6Ang = 140, rudder6AngOld = 140;
//data read
const int dataNum = 9; //motorData 7rudderData maxPower
int serial_out_count = 0; //used for counting the times that serial communication error
boolean readMark = 0; //used for serialread flag
//read data servoData,motorData
int motorData = 0, motorDataOld = 100;
int rudder0Data = 0, rudder0DataOld = 90;
int rudder1Data = 0, rudder1DataOld = 90;
int rudder2Data = 0, rudder2DataOld = 90;
int rudder3Data = 0, rudder3DataOld = 90;
int rudder4Data = 0, rudder4DataOld = 90;
int rudder5Data = 0, rudder5DataOld = 90;
int rudder6Data = 0, rudder6DataOld = 90;
//remote control pins
const int throPin = 4, ruddPin = 5, gearPin = 6;
//auto control flag
boolean autoFlag = 1;
//loop count
int count = 0;
// remote controller duration
int durGear; // used for control type switch (control type: auto or manual)
//voltage current meter
int voltagePin = 0;
int currentPin = 1;
const float voltageRef = 4.965; // Arduino uno voltage reference /v
float voltage, current, power, averagePower = 0; //units are /v /A /w respectively
float tmpVoltage[2], tmpCurrent[2] = {0, 0};
// power limit
// float powerUpperLimit = 4; // power upper limit 10 w
boolean powerIsLimited = 0;
//EEPROM
int addr = 0; //one byte(0-255) to store maxpower
int maxPowerVal; //maxPower 0~255 represent 0~20w
float maxPower;
int powerUpTo = 20; //20w
//velocity limit
int rudderVelLimit = 4;
int rudderLimit = 30;
// rudder offset
int rudderOffset = 7;

void setup() {
  motor.attach(9, 1100, 1900);  // attaches the servo on pin 9 to the servo object
  rudder0.attach(22, 1100, 1900);
  rudder1.attach(23, 1100, 1900);
  rudder2.attach(24, 1100, 1900);
  rudder3.attach(25, 1100, 1900);
  rudder4.attach(26, 1100, 1900);
  rudder5.attach(27, 1100, 1900);
  rudder6.attach(28, 1100, 1900);
  pinMode(throPin, INPUT);
  pinMode(ruddPin, INPUT);
  pinMode(gearPin, INPUT);
  motor.writeMicroseconds(motorMicroSec);
  rudder0.write(rudder0Ang);
  rudder1.write(rudder1Ang);
  rudder2.write(rudder2Ang);
  rudder3.write(rudder3Ang);
  rudder4.write(rudder4Ang);
  rudder5.write(rudder5Ang);
  rudder6.write(rudder6Ang);
  Serial.begin(115200);
  while (!Serial);
  durGear = pulseIn(gearPin, HIGH);
  delay(1000);
  maxPowerVal = EEPROM.read(addr);
  maxPower = maxPowerVal / 255.0 * powerUpTo;
  maxPower = 10;
  FlexiTimer2::set(100, flash);
  FlexiTimer2::start();
}


unsigned int calcCRC(int data[9])
{
  unsigned int poly = 0x84;
  unsigned int crc = 0;
  int carry;
  int i_bits;
  for (int j = 0; j < dataNum; j++)
  {
    crc = crc ^ data[j];
    for (i_bits = 0; i_bits < 8; i_bits++)
    {
      carry = crc & 1;
      crc = crc >> 1;
      if (carry)
      {
        crc = crc ^ poly;
      }
    }
  }
  return crc;
}


void serialRead() {
  int serialdata[dataNum + 1];
  boolean header_find_flag = false;
  // while (Serial.available() > (2 * (dataNum + 2) - 1)) Serial.read(); //clear the buffer(transfer 11bytes data, 21bytes enough for read)
  while (Serial.available()) {
    if ((int)Serial.read() == 255) {
      header_find_flag = true;
      break;
    }
  }
  if (header_find_flag == true && Serial.available() > dataNum) {
    for (int i = 0; i < (dataNum + 1); i++) {
      serialdata[i] = (int)Serial.read();
    }
    int Data[dataNum];
    for (int i = 0; i < dataNum; i++) {
      Data[i] = serialdata[i];
    }
    unsigned int crcnum = serialdata[dataNum];
    unsigned int crchecknum = calcCRC(Data);
    if (crcnum == crchecknum) {
      readMark = 1;
      motorData = motorDataOld = Data[0];
      rudder0Data = rudder0DataOld = Data[2];
      rudder1Data = rudder1DataOld = Data[3];
      rudder2Data = rudder2DataOld = Data[4];
      rudder3Data = rudder3DataOld = Data[5];
      rudder4Data = rudder4DataOld = Data[6];
      rudder5Data = rudder5DataOld = Data[7];
      rudder6Data = rudder6DataOld = Data[8];
      int maxPowerRead = Data[1];
      if (maxPowerRead != maxPowerVal && maxPowerRead != 0) {
        maxPowerVal = maxPowerRead;
        EEPROM.write(addr, maxPowerVal);
        maxPower = maxPowerVal / 255.0 * powerUpTo;
      }
      serial_out_count = 0;
    }
  }
  else {
    serial_out_count ++;
    readMark = 0;
    if (serial_out_count > 10) {  //timeout=10*0.1=1s
      motorData = 100;
      rudder0Data = 90;
      rudder1Data = 90;
      rudder2Data = 90;
      rudder3Data = 90;
      rudder4Data = 90;
      rudder5Data = 90;
      rudder6Data = 90;
    }
    else {
      motorData = motorDataOld;
      rudder0Data = rudder0DataOld;
      rudder1Data = rudder1DataOld;
      rudder2Data = rudder2DataOld;
      rudder3Data = rudder3DataOld;
      rudder4Data = rudder4DataOld;
      rudder5Data = rudder5DataOld;
      rudder6Data = rudder6DataOld;
    }
  }
}


void signalSelection() {
  if (autoFlag == 1) {
    motorMicroSec = map(motorData, 0, 200, 1400, 1600);
    rudder0Ang = map(rudder0Data, 50, 130, 100, 180);
    rudder1Ang = map(rudder1Data, 50, 130, 100, 180);
    rudder2Ang = map(rudder2Data, 50, 130, 100, 180);
    rudder3Ang = map(rudder3Data, 50, 130, 100, 180);
    rudder4Ang = map(rudder4Data, 50, 130, 100, 180);
    rudder5Ang = map(rudder5Data, 50, 130, 100, 180);
    rudder6Ang = map(rudder6Data, 50, 130, 100, 180);
  }
  else {
    motorMicroSec = pulseIn(throPin, HIGH);
    motorMicroSec = map(motorMicroSec, 1100, 1900, 1400, 1600);
    int durRudd = pulseIn(ruddPin, HIGH);
    rudder0Ang = map(durRudd, 1100, 1900, 100, 180);
    rudder1Ang = rudder2Ang = rudder3Ang = rudder4Ang = rudder5Ang = rudder6Ang = rudder0Ang;
  }
}


void veloLimit() {
  if (abs(motorMicroSecOld - 1500) < 24 && abs(motorMicroSec - 1500) > 24) {
    if (motorMicroSec > 1500) motorMicroSec = 1524;
    else motorMicroSec = 1476;
  }
  else {
    motorMicroSec = constrain(motorMicroSec, motorMicroSecOld - 4, motorMicroSecOld + 4);
  }
  rudder0Ang = constrain(rudder0Ang, rudder0AngOld - rudderVelLimit, rudder0AngOld + rudderVelLimit); //limit up to 50 deg/s
  rudder1Ang = constrain(rudder1Ang, rudder1AngOld - rudderVelLimit, rudder1AngOld + rudderVelLimit);
  rudder2Ang = constrain(rudder2Ang, rudder2AngOld - rudderVelLimit, rudder2AngOld + rudderVelLimit);
  rudder3Ang = constrain(rudder3Ang, rudder3AngOld - rudderVelLimit, rudder3AngOld + rudderVelLimit);
  rudder4Ang = constrain(rudder4Ang, rudder4AngOld - rudderVelLimit, rudder4AngOld + rudderVelLimit);
  rudder5Ang = constrain(rudder5Ang, rudder5AngOld - rudderVelLimit, rudder5AngOld + rudderVelLimit);
  rudder6Ang = constrain(rudder6Ang, rudder6AngOld - rudderVelLimit, rudder6AngOld + rudderVelLimit);
}


void voltageCurrentMeter() {
  tmpVoltage[0]  = tmpVoltage[1];
  tmpVoltage[1] = voltage;
  tmpCurrent[0] = tmpCurrent[1];
  tmpCurrent[1] = current;
  long analogVoltage = 0;
  long analogCurrent = 0;
  int count = 0;
  int countEnd = 30;
  while (count < countEnd) { //read voltage and current 10 times.analoRead() cost 0.1ms per time, s.t.2ms in this loop)
    count ++;
    analogVoltage += analogRead(voltagePin);
    analogCurrent += analogRead(currentPin);
  }
  analogVoltage = analogVoltage / countEnd;
  analogCurrent = analogCurrent / countEnd;
  voltage = analogVoltage / 1024.0 * 5 * voltageRef; //compute real voltage value
  current = abs(analogCurrent / 1024.0 - 0.5) * voltageRef / 0.185; //0A output mid voltage; 0.185V/A
  voltage = 0.2 * tmpVoltage[0] + 0.3 * tmpVoltage[1] + 0.5 * voltage; // weighted average
  current = 0.2 * tmpCurrent[0] + 0.3 * tmpCurrent[1] + 0.5 * current;
  float powerOld = power;
  power = voltage * current;
  averagePower = (powerOld + power) / 2;
}


void powerLimit() {
  float powerOverflow = (averagePower - maxPower) / maxPower;
  if (powerOverflow <= 0) {
    motorMaxMicroSec = motorMicroSecOld;
    powerIsLimited = 0;
  }
  else {
    float k = 0.1 * powerOverflow;
    if (motorMicroSec > motorMicroSecOld && motorMicroSecOld > 1500) {
      motorMicroSec = motorMaxMicroSec;
    }
    else if (motorMicroSec < motorMicroSecOld && motorMicroSecOld < 1500) {
      motorMicroSec = motorMaxMicroSec;
    }
    motorMicroSec = (int)(motorMicroSec - k * (motorMicroSec - 1500)); //limit motor voltage signal; close up to 1500
    powerIsLimited = 1;
  }
}


void servoCtrl() {
  motorMicroSecOld = motorMicroSec = constrain(motorMicroSec, 1100, 1900);
  motor.writeMicroseconds(motorMicroSec);
  rudder0AngOld = rudder0Ang = constrain(rudder0Ang, 140 - rudderLimit, 140 + rudderLimit);
  int rudder0AngModify = rudder0Ang + rudderOffset;
  rudder0.write(rudder0AngModify);
  rudder1AngOld = rudder1Ang = constrain(rudder1Ang, 140 - rudderLimit, 140 + rudderLimit);
  rudder1.write(rudder1Ang);
  rudder2AngOld = rudder2Ang = constrain(rudder2Ang, 140 - rudderLimit, 140 + rudderLimit);
  rudder2.write(rudder2Ang);
  rudder3AngOld = rudder3Ang = constrain(rudder3Ang, 140 - rudderLimit, 140 + rudderLimit);
  rudder3.write(rudder3Ang);
  rudder4AngOld = rudder4Ang = constrain(rudder4Ang, 140 - rudderLimit, 140 + rudderLimit);
  rudder4.write(rudder4Ang);
  rudder5AngOld = rudder5Ang = constrain(rudder5Ang, 140 - rudderLimit, 140 + rudderLimit);
  rudder5.write(rudder5Ang);
  rudder6AngOld = rudder6Ang = constrain(rudder6Ang, 140 - rudderLimit, 140 + rudderLimit);
  rudder6.write(rudder6Ang);
}


void dataSend() {
  Serial.print("#");
  Serial.print(",");
  Serial.print(motorMicroSec);
  Serial.print(",");
  Serial.print(motorMaxMicroSec);
  Serial.print(",");
  Serial.print(rudder0Ang - 140);
  Serial.print(",");
  Serial.print(rudder1Ang - 140);
  Serial.print(",");
  Serial.print(rudder2Ang - 140);
  Serial.print(",");
  Serial.print(rudder3Ang - 140);
  Serial.print(",");
  Serial.print(rudder4Ang - 140);
  Serial.print(",");
  Serial.print(rudder5Ang - 140);
  Serial.print(",");
  Serial.print(rudder6Ang - 140);
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.print(power);
  Serial.print(",");
  Serial.print(readMark);
  Serial.print(",");
  Serial.print(maxPower);
  Serial.print(",");
  Serial.print(powerIsLimited);
  Serial.print(",");
  Serial.println(autoFlag);
}


void flash() {
  voltageCurrentMeter();
  serialRead();
  count ++;
  if (count == 10) { //read the gearPin every 10 intervals
    durGear = pulseIn(gearPin, HIGH, 20000);
    count = 0;
  }
  if (durGear < 1950 && durGear > 1050) {
    if (durGear < 1500)
      autoFlag = 0;
    else
      autoFlag = 1;
  }
  signalSelection();
  veloLimit();
  // powerLimit();
  servoCtrl();
  dataSend();
}


void loop() {

}
