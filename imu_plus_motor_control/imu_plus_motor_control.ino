#include <Arduino_FreeRTOS.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// === MPU6050 ===
MPU6050 mpu;
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];
Quaternion q;
VectorInt16 aa, gg, aaWorld, ggWorld;
VectorFloat gravity;

#define EARTH_GRAVITY_MS2 9.80665
#define DEG_TO_RAD        0.017453292519943295769236907684886

// === L298N Motor Driver Pins ===
#define enA 5
#define in1 6
#define in2 7
#define in3 8
#define in4 9
#define enB 10

// === Function Prototypes ===
void TaskIMU(void *pvParameters);
void TaskMotorControl(void *pvParameters);

void setup() {
  // Motor setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // I2C + Serial
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  // FreeRTOS Tasks
  xTaskCreate(TaskIMU, "IMU", 256, NULL, 1, NULL);
  xTaskCreate(TaskMotorControl, "Motor", 256, NULL, 1, NULL);
}

void loop() {
  // Nothing here with FreeRTOS
}

void TaskIMU(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    if (DMPReady && mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetAccel(&aa, FIFOBuffer);
      mpu.dmpGetGyro(&gg, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
      mpu.dmpConvertToWorldFrame(&ggWorld, &gg, &q);

      float gx = ggWorld.x * mpu.get_gyro_resolution() * DEG_TO_RAD;
      float gy = ggWorld.y * mpu.get_gyro_resolution() * DEG_TO_RAD;
      float gz = ggWorld.z * mpu.get_gyro_resolution() * DEG_TO_RAD;

      float ax = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
      float ay = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
      float az = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;

      Serial.print("IMU: ");
      Serial.print(q.w); Serial.print(",");
      Serial.print(q.x); Serial.print(",");
      Serial.print(q.y); Serial.print(",");
      Serial.print(q.z); Serial.print(",");
      Serial.print(gx); Serial.print(",");
      Serial.print(gy); Serial.print(",");
      Serial.print(gz); Serial.print(",");
      Serial.print(ax); Serial.print(",");
      Serial.print(ay); Serial.print(",");
      Serial.println(az);
    }
    vTaskDelay(0.1 / portTICK_PERIOD_MS); // 50Hz update rate
  }
}

void TaskMotorControl(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    if (Serial.available()) {
      char command = Serial.read();
      switch (command) {
        case 'w':
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          analogWrite(enA, 127);
          analogWrite(enB, 191);
          break;
        case 's':
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          digitalWrite(in4, LOW);
          digitalWrite(enA, LOW);
          digitalWrite(enB, LOW);
          break;
        case 'a':
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          analogWrite(enA, 127);
          analogWrite(enB, 191);
          break;
        case 'd':
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enA, 127);
          analogWrite(enB, 191);
          break;
        case 'b':
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enA, 127);
          analogWrite(enB, 191);
          break;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); // check input every 200ms
  }
}
