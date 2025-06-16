#include <SoftwareSerial.h>
#include <Arduino.h>

SoftwareSerial servoSerial(9, 11); // RX, TX

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define LOBOT_SERVO_FRAME_HEADER     0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE  1
#define LOBOT_SERVO_ID_WRITE         13
#define LOBOT_SERVO_POS_READ         28
#define LOBOT_SERVO_TORQUE_OFF       29
#define LOBOT_SERVO_TORQUE_ON        28

byte LobotCheckSum(byte buf[]) {
  uint16_t temp = 0;
  for (byte i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  return (byte)(~temp);
}

void LobotSerialServoMove(SoftwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time) {
  byte buf[10];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = LOBOT_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = LobotCheckSum(buf);
  SerialX.write(buf, 10);
}

void LobotSerialServoSetID(SoftwareSerial &SerialX, uint8_t oldID, uint8_t newID) {
  byte buf[7];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = LobotCheckSum(buf);
  SerialX.write(buf, 7);
}

void LobotSerialServoTorqueOn(SoftwareSerial &SerialX, uint8_t id) {
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TORQUE_ON;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

void LobotSerialServoTorqueOff(SoftwareSerial &SerialX, uint8_t id) {
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_TORQUE_OFF;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);
}

int LobotSerialServoReadPosition(SoftwareSerial &SerialX, uint8_t id) {
  byte buf[6];
  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = LOBOT_SERVO_POS_READ;
  buf[5] = LobotCheckSum(buf);
  SerialX.write(buf, 6);

  unsigned long startTime = millis();
  while (SerialX.available() < 8) {
    if (millis() - startTime > 100) return -1; // timeout
  }

  byte header1 = SerialX.read(); // 0x55
  byte header2 = SerialX.read(); // 0x55
  byte ID = SerialX.read();
  byte len = SerialX.read();
  byte cmd = SerialX.read();
  byte posL = SerialX.read();
  byte posH = SerialX.read();
  byte checksum = SerialX.read();

  int position = ((int)posH << 8) | posL;
  return position;
}

void setup() {
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  servoSerial.begin(115200);
  delay(1000);

  // تحريك السيرفوهات الأساسية للموضع 0


  Serial.println("🚀 بدأ البرنامج. تحريك السيرفو إلى 0 ثم 90 ثم 180");
}

void loop() {
  for(int i =1 ;i<=5;i++){
    Serial.println(i);
    LobotSerialServoMove(servoSerial, i,random(0, 800), 500);
     delay(2500);
  }
       
  }
  
  if (Serial.available()) {
    char received = Serial.read();
    if (received == '1') {
           printPosition(1);
    }else  if (received == '2') {
           printPosition(2);
    }else  if (received == '3') {
           printPosition(3);
    }else  if (received == '4') {
           printPosition(4);
    }else  if (received == '5') {
           printPosition(5);
    }
  }
}

// دالة طباعة موضع السيرفو
void printPosition(uint8_t id) {
  int pos = LobotSerialServoReadPosition(servoSerial, id);
  if (pos != -1) {
    Serial.print("📍 الموضع الحالي للسيرفو ID ");
    Serial.print(id);
    Serial.print(" = ");
    Serial.println(pos);
  } else {
    Serial.println("❌ فشل في قراءة الموضع (Timeout)");
  }
}
