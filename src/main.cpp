#include "Arduino.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#include <AutoPID.h>

#define MSP_RC 105
#define MSP_ATTITUDE 108
#define OUTPUT_MIN -90
#define OUTPUT_MAX 90
#define PitchKP .775
#define PitchKI 14.712
#define PitchKD 2770
#define RollKP .679
#define RollKI 12.323
#define RollKD 2220
#define SERVO_ROLL PB0
#define SERVO_PITCH PB1
#define midpos 1500
//#define alpha .98758465
//#define alpha .984528833
#define alpha .1
//#define alpha .969529086

double oldpitch, oldroll, fpitch, froll, setPointPitch, setPointRoll, outputPitch, outputRoll;
int mpitch, mroll;
unsigned long long lastTempUpdate, lastsamp;
AutoPID PIDpitch(&fpitch, &setPointPitch, &outputPitch, OUTPUT_MIN, OUTPUT_MAX, PitchKP, PitchKI, PitchKD);
AutoPID PIDroll(&froll, &setPointRoll, &outputRoll, OUTPUT_MIN, OUTPUT_MAX, RollKP, RollKI, RollKD);
MPU6050 mpu(Wire);
Servo pitchservo;
Servo rollservo;
HardwareSerial fc(USART3);

int i = 0;
const int interv = 5;
unsigned long timer = 0;
unsigned long previousMillis = 0;
const long dly = 15;
int mspaux1, mspaux2;
//double alpha = 0.969529086;

void updatefeedback() {
  mpitch = int(mpu.getAngleY());
  mroll = int(mpu.getAngleX());
  fpitch = (alpha * oldpitch) + ((1 - alpha) * mpitch);
  froll = (alpha * oldroll) + ((1 - alpha) * mroll);
  oldpitch = fpitch;
  oldroll = froll;
}

void sendMSP(uint8_t cmd, uint8_t *data, uint8_t n_bytes) {
  uint8_t checksum = 0;
  fc.write((byte *) "$M<", 3);
  fc.write(n_bytes);
  checksum ^= n_bytes;
  fc.write(cmd);
  checksum ^= cmd;
  fc.write(checksum);
}

void readDataMSP() {
  int16_t aux1;
  int16_t aux2;
  byte count = 0;
  while (fc.available()) {
    count += 1;
    byte c = fc.read();
    switch (count) {
      case 14:
        aux1 = c;
        break;
      case 15:
        aux1 <<= 8;
        aux1 += c;
        aux1 = (aux1 & 0xFF00) >> 8 | (aux1 & 0x00FF) << 8; // Reverse the order of bytes
        break;
      case 16:
        aux2 += c;
        break;
      case 17:
        aux2 <<= 8;
        aux2 += c;
        aux2 = (aux2 & 0xFF00) >> 8 | (aux2 & 0x00FF) << 8; // Reverse the order of bytes
        break;
    }
  }
  if (aux1 >=1000 && aux1 <= 2000) mspaux1 = aux1;
  if (aux2 >=1000 && aux2 <= 2000) mspaux2 = aux2;
}

void setup() {
  fc.begin(115200);
  Serial.begin(57600);
  pitchservo.attach(SERVO_PITCH, 1000, 2000);
  rollservo.attach(SERVO_ROLL, 1000, 2000);
  pitchservo.writeMicroseconds(midpos);
  rollservo.writeMicroseconds(midpos);
  Serial.print(alpha);
  delay(1000);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  PIDpitch.setBangBang(0);
  PIDpitch.setTimeStep(interv);
  PIDroll.setBangBang(0);
  PIDroll.setTimeStep(interv);
}

void loop() {
  mpu.update();
  uint8_t datad = 0;
  uint8_t *data = &datad;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= dly) {
    previousMillis = currentMillis;
//    sendMSP(MSP_ATTITUDE, data, 0);
    sendMSP(MSP_RC, data, 0);
    readDataMSP();
//    debugserial();
  }
  updatefeedback();
  if (mspaux1 >= 1800) setPointPitch = map(mspaux2,1000,2000,-90,90);
  else setPointPitch = 0;
  setPointRoll = 0;
  PIDpitch.run();
  PIDroll.run();
  pitchservo.writeMicroseconds(map(outputPitch, 90, -90, 1000, 2000));
  rollservo.writeMicroseconds(map(outputRoll, -90, 90, 1000, 2000));
}
