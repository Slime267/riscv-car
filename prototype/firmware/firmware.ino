/*
 * RISC-V 无线遥控小车 — 固件 (L298N + 2WD + TT 电机 + OTA)
 * 芯片: 合宙 ESP32C3 经典版 (RISC-V 32-bit, 160MHz)
 * 环境: Arduino IDE 2.x + ESP32-C3 开发板支持
 *
 * ============ 接线 ============
 *   GPIO2  → L298N ENA   (左轮 PWM)
 *   GPIO3  → L298N IN1
 *   GPIO4  → L298N IN2
 *   GPIO5  → L298N ENB   (右轮 PWM)
 *   GPIO10 → L298N IN3
 *   GPIO1  → L298N IN4
 *   GPIO6  → SSD1306 SDA (I2C)
 *   GPIO7  → SSD1306 SCL (I2C)
 *   GPIO8  → 板载 LED
 *
 * ============ 供电 ============
 *   电池 7.4V → MP1584(5V) → ESP32C3 5V + L298N 5V(逻辑, 拔跳线帽!)
 *   电池 7.4V → L298N 12V (电机供电)
 *   全部 GND 共地
 *
 * ============ OTA 无线升级 ============
 *   上电后自动启动 WiFi 热点 "RISC-V-OTA" (密码 12345678)
 *   手机连此 WiFi → 浏览器打开 http://192.168.4.1
 *   上传固件 .bin 文件 → 自动重启
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

// ==================== L298N 引脚 ====================
#define PIN_ENA 10  // 左电机 PWM
#define PIN_IN1 6
#define PIN_IN2 7
#define PIN_ENB 8   // 右电机 PWM
#define PIN_IN3 5
#define PIN_IN4 4

// ==================== 其他引脚 ====================
#define PIN_I2C_SDA 3  // OLED SDA
#define PIN_I2C_SCL 2  // OLED SCL
#define PIN_LED 21     // 板载 LED
#define PIN_BAT 0  // ADC 电池分压

// ==================== 电机参数 ====================
#define MAX_PWM 255
#define MIN_PWM 12   // 死区, 5%

// ==================== BLE UUID ====================
#define SERVICE_UUID "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHAR_CTRL_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CHAR_STATUS_UUID "0000ffe2-0000-1000-8000-00805f9b34fb"

// ==================== OLED ====================
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(128, 32, &Wire, -1);

// ==================== 全局状态 ====================
BLEServer *pServer = nullptr;
BLECharacteristic *pCharCtrl = nullptr;
BLECharacteristic *pCharStatus = nullptr;
bool connected = false;
int motor_l = 0;
int motor_r = 0;
bool led_on = false;
float bat_v = 0;
int bat_pct = 0;

// ==================== BLE 控制回调 ====================
class CtrlCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    uint8_t *d = pChar->getData();
    size_t len = pChar->getLength();
    if (len < 1) return;

    int spd = MAX_PWM * 50 / 100;

    // UTF-8 文本指令
    if (d[0] >= 'A' && d[0] <= 'z') {
      switch (d[0]) {
        case 'F': motor_l =  spd; motor_r = -spd; break;
        case 'B': motor_l = -spd; motor_r =  spd; break;
        case 'L': motor_l = -spd; motor_r = -spd; break;
        case 'R': motor_l =  spd; motor_r =  spd; break;
        case 'S': motor_l =  0;   motor_r =  0;   break;
      }
      return;
    }

    // 二进制指令: [左速, 右速, led]
    if (len < 2) return;
    int raw_l = (int8_t)d[0];
    int raw_r = (int8_t)d[1];
    motor_l = (raw_l * MAX_PWM) / 127;
    motor_r = -(raw_r * MAX_PWM) / 127;

    if (abs(motor_l) < MIN_PWM) motor_l = 0;
    if (abs(motor_r) < MIN_PWM) motor_r = 0;

    if (len >= 3 && d[2] == 1) {
      led_on = !led_on;
      digitalWrite(PIN_LED, led_on ? HIGH : LOW);
    }
  }
};

class SvrCallback : public BLEServerCallbacks {
  void onConnect(BLEServer *s) {
    connected = true;
    digitalWrite(PIN_LED, HIGH);
  }
  void onDisconnect(BLEServer *s) {
    connected = false;
    digitalWrite(PIN_LED, LOW);
    pServer->startAdvertising();
  }
};

// ==================== L298N 电机驱动 ====================
void set_motor(int en, int in1, int in2, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -speed);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

void update_motors() {
  set_motor(PIN_ENA, PIN_IN1, PIN_IN2,  motor_l);
  set_motor(PIN_ENB, PIN_IN3, PIN_IN4,  motor_r);
}

// ==================== 电源指示 ====================
void check_power() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  // 20K+10K 分压: Vbat = ADC电压 * (20+10)/10 = ADC电压 * 3
  int adc = analogRead(PIN_BAT);
  bat_v = adc * 3.9f / 4095.0f * 3.0f * 0.757f;
  bat_pct = constrain((int)((bat_v - 6.6f) / (8.4f - 6.6f) * 100), 0, 100);
}

// ==================== BLE 遥测上报 ====================
void ble_notify_status() {
  static uint32_t last = 0;
  if (!connected || millis() - last < 500) return;
  last = millis();

  uint8_t data[4] = {
    (uint8_t)(abs(motor_l) * 100 / MAX_PWM),
    (uint8_t)(abs(motor_r) * 100 / MAX_PWM),
    (uint8_t)bat_pct,
    (uint8_t)(connected ? 1 : 0)
  };
  pCharStatus->setValue(data, 4);
  pCharStatus->notify();
}

// ==================== OLED 显示 ====================
void oled_update() {
  static uint32_t last = 0;
  if (millis() - last < 200) return;
  last = millis();

  int spd_l = abs(motor_l) * 100 / MAX_PWM;
  int spd_r = abs(motor_r) * 100 / MAX_PWM;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // 第 1 行: 状态 + 速度
  display.setCursor(0, 0);
  display.print(connected ? "BLE:OK" : "BLE:--");
  display.setCursor(56, 0);
  display.print("L:"); display.print(spd_l); display.print("%");
  display.setCursor(96, 0);
  display.print("R:"); display.print(spd_r); display.print("%");

  // 第 2 行: 方向指示
  display.setCursor(0, 10);
  if (motor_l == 0 && motor_r == 0) {
    display.print("STOP       ");
  } else if (motor_l > 0 && motor_r > 0) {
    display.print(">>  FWD  >>");
  } else if (motor_l < 0 && motor_r < 0) {
    display.print("<<  REV  <<");
  } else if (motor_l < motor_r) {
    display.print("<<  LEFT   ");
  } else if (motor_l > motor_r) {
    display.print("   RIGHT >>");
  }

  // 第 3 行: 电池
  display.setCursor(0, 22);
  display.print(bat_v, 1); display.print("V ");
  display.print(bat_pct); display.print("%");
  if (connected) display.fillRect(123, 0, 5, 5, SSD1306_WHITE);

  display.display();
}

// ==================== 初始化 ====================
void setup() {
  // --- GPIO ---
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // --- I2C / OLED ---
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(20, 8);
    display.print("RISC-V");
    display.display();
    delay(1000);
  }

  // --- 开机自检 ---
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED, HIGH); delay(80);
    digitalWrite(PIN_LED, LOW);  delay(80);
  }

  // --- BLE ---
  BLEDevice::init("RISC-V Car");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new SvrCallback());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharCtrl = pService->createCharacteristic(
    CHAR_CTRL_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharCtrl->setCallbacks(new CtrlCallback());
  pCharStatus = pService->createCharacteristic(
    CHAR_STATUS_UUID,
    BLECharacteristic::PROPERTY_NOTIFY);
  pCharStatus->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->start();


// ==================== 主循环 ====================
void loop() {
  update_motors();
  check_power();
  ble_notify_status();
  oled_update();
  delay(10);
}
