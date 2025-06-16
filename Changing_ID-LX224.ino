#include <SoftwareSerial.h>

// تعريف الماكروات
#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define LOBOT_SERVO_FRAME_HEADER 0x55
#define LOBOT_SERVO_ID_WRITE 13

// تعريف المنفذ التسلسلي الجديد
SoftwareSerial servoSerial(9, 11); // RX, TX

// حساب Checksum
byte LobotCheckSum(byte buf[]) {
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  return (byte)temp;
}

// دالة تغيير ID السيرفو
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

void setup() {
  Serial.begin(115200);      // لمراقبة السيريال من الكمبيوتر
  servoSerial.begin(115200); // للسيرفو
  pinMode(13, OUTPUT);
  delay(1000);
  Serial.println("🚀 Starting servo ID change...");
}

void loop() {
  delay(500);
  digitalWrite(13, HIGH);
  Serial.println("🔧 Changing ID from 2 to 1...");
  LobotSerialServoSetID(servoSerial, 2, 1);  // من ID 1 إلى 2
  delay(500);
  digitalWrite(13, LOW);
}
