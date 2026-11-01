#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

// ---------- BLE ----------
#define DEVICE_NAME  "ESP32 Safety Monitor"
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ---------- Pins ----------
#define TRIG_PIN      5
#define ECHO_PIN      18   // use a voltage divider (HC-SR04 echo is 5V)
#define WATER_PIN     27
#define BUZZER_PIN    26
#define VIBRATOR_PIN  25   // vibrator 1, drive through a transistor
#define VIBRATOR2_PIN 33   // vibrator 2, drive through its own transistor
#define MPU_SDA       21
#define MPU_SCL       22

// ---------- Settings ----------
#define DIST_THRESHOLD_CM   25
#define VIBRATOR_TIME_MS    3000
#define WATER_ACTIVE_STATE  HIGH   // change to LOW for active-low modules
#define GYRO_LIMIT_DPS      60.0
#define ACCEL_LIMIT_G       0.35
#define NOTIFY_EVERY_MS     200

const uint8_t MPU_ADDR = 0x68;

BLEServer* bleServer = nullptr;
BLECharacteristic* dataChar = nullptr;
bool clientConnected = false;
bool wasConnected = false;

// ---------- Latest readings ----------
long  distanceCm = -1;
bool  objectNear = false, waterDetected = false, unstable = false;
bool  buzzerOn = false, vibratorOn = false, lastObjectNear = false;
float gyroMag = 0, accMag = 1;
unsigned long vibratorStart = 0, lastNotify = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*)    { clientConnected = true;  Serial.println("Client connected"); }
  void onDisconnect(BLEServer*) { clientConnected = false; Serial.println("Client disconnected"); }
};

long readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}

void mpuInit() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

bool readMpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (Wire.available() < 14) return true;   // no response = unstable

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  float axg = ax / 16384.0, ayg = ay / 16384.0, azg = az / 16384.0;
  float gxd = gx / 131.0,   gyd = gy / 131.0,   gzd = gz / 131.0;
  accMag  = sqrt(axg * axg + ayg * ayg + azg * azg);
  gyroMag = sqrt(gxd * gxd + gyd * gyd + gzd * gzd);
  return (fabs(accMag - 1.0) > ACCEL_LIMIT_G) || (gyroMag > GYRO_LIMIT_DPS);
}

// 9-byte packet: flags, distance(int16), vibratorLeft(uint16), gyro x10(uint16), accel x100(uint16)
void sendPacket() {
  uint16_t left = 0;
  if (vibratorOn) {
    unsigned long used = millis() - vibratorStart;
    left = used < VIBRATOR_TIME_MS ? VIBRATOR_TIME_MS - used : 0;
  }
  uint8_t flags = (objectNear ? 1 : 0) | (waterDetected ? 2 : 0) | (unstable ? 4 : 0) |
                  (buzzerOn ? 8 : 0) | (vibratorOn ? 16 : 0);
  int16_t  d = (int16_t)distanceCm;
  uint16_t g = (uint16_t)constrain(gyroMag * 10, 0, 65535);
  uint16_t a = (uint16_t)constrain(accMag * 100, 0, 65535);

  uint8_t p[9];
  p[0] = flags;
  p[1] = d & 0xFF;     p[2] = (d >> 8) & 0xFF;
  p[3] = left & 0xFF;  p[4] = (left >> 8) & 0xFF;
  p[5] = g & 0xFF;     p[6] = (g >> 8) & 0xFF;
  p[7] = a & 0xFF;     p[8] = (a >> 8) & 0xFF;

  dataChar->setValue(p, 9);
  dataChar->notify();
}

void setVibrators(bool on) {
  digitalWrite(VIBRATOR_PIN,  on ? HIGH : LOW);
  digitalWrite(VIBRATOR2_PIN, on ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(WATER_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VIBRATOR_PIN, OUTPUT);
  pinMode(VIBRATOR2_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  setVibrators(false);
  mpuInit();

  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);
  dataChar = service->createCharacteristic(CHAR_UUID,
               BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  dataChar->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Advertising as: " DEVICE_NAME);
  Serial.print("Bluetooth address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());
}

void loop() {
  distanceCm    = readDistanceCm();
  objectNear    = (distanceCm > 0 && distanceCm <= DIST_THRESHOLD_CM);
  waterDetected = (digitalRead(WATER_PIN) == WATER_ACTIVE_STATE);
  unstable      = readMpu();   // still sent to the dashboard, does not drive the buzzer

  // Buzzer: ON if object within 25 cm OR water detected, otherwise OFF
  buzzerOn = objectNear || waterDetected;
  digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);

  // Both vibrators: ON for 3 s when the ultrasonic detects an object
  if (objectNear && !lastObjectNear) {
    vibratorOn = true;
    vibratorStart = millis();
    setVibrators(true);
  }
  lastObjectNear = objectNear;

  if (vibratorOn && millis() - vibratorStart >= VIBRATOR_TIME_MS) {
    vibratorOn = false;
    setVibrators(false);
  }

  // Send readings to the dashboard
  if (clientConnected && millis() - lastNotify >= NOTIFY_EVERY_MS) {
    lastNotify = millis();
    sendPacket();
  }

  // Start advertising again after the phone/computer disconnects
  if (!clientConnected && wasConnected) {
    delay(300);
    BLEDevice::startAdvertising();
  }
  wasConnected = clientConnected;

  delay(50);
}