#include <Servo.h>
#include <MsTimer2.h>
#include <EEPROM.h>
//servo configure
Servo motor, rudder;
int motorMicroSec = 1500, motorMicroSecOld = 1500, motorMaxMicroSec = 1500, rudderAng = 140, rudderAngOld = 140;
//data read
const int dataNum = 3; //motorData rudderData maxPower
int serial_out_count = 0; //used for counting the times that serial communication error
boolean readMark = 0; //used for serialread flag
//read data servoData,motorData
int motorData, rudderData = 0;
int motorDataOld = 100, rudderDataOld = 90;
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
float voltage, current, power, averagePower; //units are /v /A /w respectively
float tmpVoltage[2], tmpCurrent[2] = {0, 0};
// power limit
// float powerUpperLimit = 4; // power upper limit 10 w
boolean powerIsLimited = 0;
//EEPROM
int addr = 0; //one byte(0-255) to store maxpower
int maxPowerVal; //maxPower 0~255 represent 0~20w
float maxPower;
int powerUpTo = 20; //20w
// velocity limit
int rudderVelLimit = 4;
int rudderLimit = 35;

void setup() {
  motor.attach(9, 1100, 1900);  // attaches the servo on pin 9 to the servo object
  rudder.attach(10, 1100, 1900);
  pinMode(throPin, INPUT);
  pinMode(ruddPin, INPUT);
  pinMode(gearPin, INPUT);
  motor.writeMicroseconds(motorMicroSec);
  rudder.write(rudderAng);
  Serial.begin(115200);
  while (!Serial);
  durGear = pulseIn(gearPin, HIGH);
  delay(1000);
  maxPowerVal = EEPROM.read(addr);
  maxPower = maxPowerVal / 255.0 * powerUpTo;
  MsTimer2::set(100, flash);
  MsTimer2::start();
}


unsigned int calcCRC(int data[3])
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
  while (Serial.available() > (2 * (dataNum + 2) - 1)) Serial.read(); //clear the buffer(transfer 4bytes data, 7bytes enough for read)
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
      rudderData = rudderDataOld = Data[1];
      int maxPowerRead = Data[2];
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
    if (serial_out_count > 10) {  //timeout=10*0.2=2s
      motorData = 100;
      rudderData = 90;
    }
    else {
      motorData = motorDataOld;
      rudderData = rudderDataOld;
    }
  }
}


void signalSelection() {
  if (autoFlag == 1) {
    motorMicroSec = map(motorData, 0, 200, 1400, 1600);
    rudderAng = map(rudderData, 50, 130, 100, 180);
  }
  else {
    motorMicroSec = pulseIn(throPin, HIGH);
    motorMicroSec = map(motorMicroSec, 1100, 1900, 1400, 1600);
    int durRudd = pulseIn(ruddPin, HIGH);
    rudderAng = map(durRudd, 1100, 1900, 100, 180);
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

  rudderAng = constrain(rudderAng, rudderAngOld - rudderVelLimit, rudderAngOld + rudderVelLimit); //limit up to 50 deg/s
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
  averagePower = 0.3 * powerOld + 0.7 * power;
}


float k = 0.2, Q_w = 0.0005, Q_v = 0.0068;
float P = Q_v;
float power_f = 0;
void powerKF() {
  int N = abs(motorMicroSec - 1500);
  int O = abs(motorMicroSecOld - 1500);
  int u = N - O;
  if (O <= 24 && N <= 24) // avoid computing deadzone
    u = 0;
  else if (O <= 24 && N > 24)
    u = N - 24;
  else if (O > 24 && N <= 24)
    u = 24 - O;
  float power1P = power_f + k * u; // 1 step prediction
  float epsilon = power - power1P;
  float PP = P + Q_w;
  float Q_ep = PP + Q_v;
  float K = PP / Q_ep;
  P = (1 - K) * PP;
  power_f = power1P + K * epsilon;
}


void powerLimit() {
  float powerOverflow = (averagePower - maxPower) / maxPower;
  if (powerOverflow <= 0) {
    motorMaxMicroSec = motorMicroSecOld;
    powerIsLimited = 0;
  }
  else {
    float k = 0.05 * powerOverflow;
    if (motorMicroSec > motorMicroSecOld && motorMicroSecOld > 1500) {
      motorMicroSec = motorMaxMicroSec;
    }
    else if (motorMicroSec < motorMicroSecOld && motorMicroSecOld < 1500) {
      motorMicroSec = motorMaxMicroSec;
    }
    else {
      motorMicroSec = (int)(motorMicroSec - k * (motorMicroSec - 1500)); //limit motor voltage signal; close up to 1500
      powerIsLimited = 1;
    }
  }
}


void servoCtrl() {
  motorMicroSecOld = motorMicroSec = constrain(motorMicroSec, 1100, 1900);
  rudderAngOld = rudderAng = constrain(rudderAng, 140 - rudderLimit, 140 + rudderLimit);
  motor.writeMicroseconds(motorMicroSec);
  int rudderAngReverse = map(rudderAng, 100, 180, 180, 100); //reverse the rudder signal
  rudder.write(rudderAngReverse);
}


void dataSend() {
  Serial.print("#");
  Serial.print(",");
  Serial.print(motorMicroSec);
  Serial.print(",");
  Serial.print(motorMaxMicroSec);
  Serial.print(",");
  Serial.print(rudderAng - 140);
  Serial.print(",");
  Serial.print(voltage);
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.print(averagePower);
  Serial.print(",");
  Serial.print(readMark);
  Serial.print(",");
  Serial.print(maxPower);
  Serial.print(",");
  Serial.print(powerIsLimited);
  Serial.print(",");
  Serial.print(autoFlag);
  Serial.print(",");
  Serial.println(power_f);
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
  powerLimit();
  servoCtrl();
  powerKF();
  dataSend();
}


void loop() {

}
