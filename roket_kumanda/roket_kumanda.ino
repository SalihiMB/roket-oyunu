/*
  ROKET KAÇIŞI - Arduino Uno R3 + MPU6050 kumanda
  ------------------------------------------------
  Bağlantılar:
    MPU6050 VCC -> 5V
    MPU6050 GND -> GND
    MPU6050 SDA -> A4
    MPU6050 SCL -> A5
    Yeşil LED : D8 -> Led kısa bacak 8 nuamaralı pin
                LED kısa bacak -> GND

  Arduino açılınca LED bir kez yanıp söner (LED testi), sonra
  yaklaşık 2 saniye jiroskop kalibrasyonu yapılır.
  Bu sırada breadboard'u masada sabit tut!

  Bilgisayara gönderilen satır:  X(roll),Y(pitch),Z(yaw),g
  Bilgisayardan gelen komutlar:
    'W' -> kazanan! LED 3 sn yanıp söner, sonra yanık kalır
    'L' -> LED yak
    'O' -> LED söndür
    'Z' -> Z açısını (yaw) sıfırla
    'C' -> jiroskobu yeniden kalibre et
*/

#include <Wire.h>
#include <math.h>

const uint8_t MPU_ADDR = 0x68;
const uint8_t LED_PIN  = 8;
const float   ALPHA    = 0.96;   // jiroskop / ivmeölçer karışım oranı

float gxOff = 0, gyOff = 0, gzOff = 0;
float roll = 0, pitch = 0, yaw = 0;
unsigned long lastMicros = 0, lastSend = 0;

// LED: 0 = kapalı, 1 = açık, 2 = kutlama (yanıp söner, sonra açık kalır)
uint8_t ledMode = 0;
unsigned long ledStart = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t read16() {
  uint8_t h = Wire.read();
  uint8_t l = Wire.read();
  return (int16_t)((h << 8) | l);
}

bool readMPU(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  ax = read16(); ay = read16(); az = read16();
  read16();                       // sıcaklık (kullanmıyoruz)
  gx = read16(); gy = read16(); gz = read16();
  return true;
}

void calibrate() {
  Serial.println(F("#CAL"));
  long sx = 0, sy = 0, sz = 0;
  float sr = 0, sp = 0;
  int n = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 500; i++) {
    if (readMPU(ax, ay, az, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      float x = ax / 16384.0, y = ay / 16384.0, z = az / 16384.0;
      sr += atan2(y, z) * RAD_TO_DEG;
      sp += atan2(-x, sqrt(y * y + z * z)) * RAD_TO_DEG;
      n++;
    }
    delay(3);
  }
  if (n > 0) {
    gxOff = sx / (float)n;
    gyOff = sy / (float)n;
    gzOff = sz / (float)n;
    roll  = sr / n;
    pitch = sp / n;
  }
  yaw = 0;
  lastMicros = micros();
  Serial.println(F("#OK"));
}

void updateLed() {
  if (ledMode == 0) {
    digitalWrite(LED_PIN, LOW);
  } else if (ledMode == 1) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    unsigned long e = millis() - ledStart;
    if (e < 3000) digitalWrite(LED_PIN, ((e / 150) % 2 == 0) ? HIGH : LOW);
    else ledMode = 1;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  // MPU6050 bağlı mı?
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    while (true) {
      Serial.println(F("#HATA MPU6050 bulunamadi - kablolari kontrol et"));
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(900);
    }
  }

  writeReg(0x6B, 0x00);  // sensörü uyandır
  writeReg(0x1A, 0x03);  // titreşim filtresi (~44 Hz)
  writeReg(0x1B, 0x00);  // jiroskop ±250 °/s
  writeReg(0x1C, 0x00);  // ivmeölçer ±2g
  delay(100);

  // LED testi
  digitalWrite(LED_PIN, HIGH); delay(300); digitalWrite(LED_PIN, LOW);

  calibrate();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  if (readMPU(ax, ay, az, gx, gy, gz)) {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0;
    lastMicros = now;
    if (dt > 0.1) dt = 0.1;

    float x = ax / 16384.0, y = ay / 16384.0, z = az / 16384.0;   // g
    float gxs = (gx - gxOff) / 131.0;                              // °/s
    float gys = (gy - gyOff) / 131.0;
    float gzs = (gz - gzOff) / 131.0;

    float accRoll  = atan2(y, z) * RAD_TO_DEG;
    float accPitch = atan2(-x, sqrt(y * y + z * z)) * RAD_TO_DEG;

    // Complementary filter: jiroskop hızlı, ivmeölçer kaymayı düzeltir
    roll  = ALPHA * (roll  + gxs * dt) + (1 - ALPHA) * accRoll;
    pitch = ALPHA * (pitch + gys * dt) + (1 - ALPHA) * accPitch;
    yaw  += gzs * dt;   // Z ekseni sadece jiroskoptan -> zamanla kayar

    float g = sqrt(x * x + y * y + z * z);   // sallama algılamak için

    if (millis() - lastSend >= 20) {          // saniyede 50 veri
      lastSend = millis();
      Serial.print(roll, 1);  Serial.print(',');
      Serial.print(pitch, 1); Serial.print(',');
      Serial.print(yaw, 1);   Serial.print(',');
      Serial.println(g, 2);
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'W')      { ledMode = 2; ledStart = millis(); }
    else if (c == 'L') { ledMode = 1; }
    else if (c == 'O') { ledMode = 0; }
    else if (c == 'Z') { yaw = 0; }
    else if (c == 'C') { calibrate(); }
  }

  updateLed();
}
