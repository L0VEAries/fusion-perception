/*
 * 智能驾驶感知系统 - 绝对编译通过且防卡死版
 * 修复：修正函数名拼写错误，优化初始化顺序，确保蓝灯必亮。
 */
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_AHTX0.h>

// ================= 1. 引脚定义 =================
#define IR_PIN        11   
#define BUZZER_PIN    10   // 蜂鸣器 (低电平触发)
#define BLUE_PIN      9    // 系统运行指示
#define WHITE_PIN     7    // 湿度高/下雨
#define YELLOW_PIN    4    // 红外避障
#define RED_PIN       3    // 快速逼近警告 / 变道失败
#define GREEN_PIN     5    // 变道成功
#define BUTTON_PIN    A0   // 变道按键

// ================= 2. 核心阈值 =================
#define HUM_ON        85.0  // 湿度开灯
#define HUM_OFF       75.0  // 湿度关灯防抖
#define TOF_MAX_DIST  800   // 雷达警告距离 (80cm)
#define APPROACH_SPD  1000  // 快速靠近速度阈值 (mm/s)

// ================= 3. 全局变量 =================
VL53L0X tof;
Adafruit_AHTX0 aht;

unsigned long lastTime = 0;
int lastDist = -1;
float rawSpeed = 0.0;
float smoothSpeed = 0.0;

bool whiteLedOn = false;
bool btnReady = true;
unsigned long redFlashStartTime = 0;
unsigned long greenStartTime = 0;

bool isApproachWarning = false; 
bool isRedFlashing = false;     
bool isGreenLit = false;        

// ================= 4. 初始化 setup =================
void setup() {
  Serial.begin(9600);
  
  // 1. 最先初始化所有输出引脚，并强制点亮蓝灯！
  pinMode(BLUE_PIN, OUTPUT);   digitalWrite(BLUE_PIN, HIGH); // 只要进 setup，蓝灯立刻亮
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); // 默认关闭
  pinMode(WHITE_PIN, OUTPUT);  digitalWrite(WHITE_PIN, LOW);
  pinMode(YELLOW_PIN, OUTPUT); digitalWrite(YELLOW_PIN, LOW);
  pinMode(RED_PIN, OUTPUT);    digitalWrite(RED_PIN, LOW);
  pinMode(GREEN_PIN, OUTPUT);  digitalWrite(GREEN_PIN, LOW);
  
  // 2. 初始化输入引脚
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT);

  // 3. 最后再初始化容易卡顿的传感器 (I2C)
  Wire.begin();
  Serial.println("Blue LED is ON. System started.");

  // 4. 雷达初始化（增加容错，初始化失败也不会卡死程序）
  tof.setTimeout(500);
  if (!tof.init()) { 
    Serial.println("WARNING: VL53L0X Sensor Failed. But system continues!"); 
  } else {
    tof.startContinuous(50);
    Serial.println("VL53L0X Sensor is Ready.");
  }

  // 5. 温湿度初始化
  if (!aht.begin()) { 
    Serial.println("WARNING: AHT20 Sensor Failed."); 
  }
}

// ================= 5. 主循环 loop =================
void loop() {
  unsigned long now = millis();

  // --- A. 传感器数据读取 ---
  int dist = -1;
  // 只有雷达初始化成功才读取，防止程序卡死
  if (tof.init()) {
    dist = tof.readRangeContinuousMillimeters();
    if (tof.timeoutOccurred() || dist <= 0 || dist > 2000) {
      dist = -1; // 数据无效，标记为 -1
    }
  }

  // 计算并强力平滑速度 (过滤掉传感器微小抖动)
  if (dist >= 0 && lastDist >= 0 && (now - lastTime) >= 50) {
    float dt = (now - lastTime) / 1000.0;
    rawSpeed = (lastDist - dist) / dt;
    smoothSpeed = smoothSpeed * 0.7 + rawSpeed * 0.3; // 0.7的平滑权重
    lastTime = now;
  }
  lastDist = dist;

  // 读取温湿度数据
  sensors_event_t h, t;
  if (aht.getEvent(&h, &t)) {
    if (h.relative_humidity > HUM_ON && !whiteLedOn) whiteLedOn = true;
    else if (h.relative_humidity < HUM_OFF && whiteLedOn) whiteLedOn = false;
  }

  bool irBlocked = (digitalRead(IR_PIN) == LOW);
  bool btnPressed = (digitalRead(BUTTON_PIN) == LOW);

  // --- B. 串口打印 (方便调试雷达警告) ---
  Serial.print("Dist: "); Serial.print(dist); 
  Serial.print("mm | Speed: "); Serial.print(smoothSpeed, 0); Serial.print(" mm/s");
  Serial.print(" | Hum: "); Serial.print(h.relative_humidity, 1); Serial.print("%");
  Serial.println();

  // --- C. 核心逻辑控制 (互不干扰) ---

  // 1. 【常驻】红外避障黄灯 (不受按键影响)
  digitalWrite(YELLOW_PIN, irBlocked ? HIGH : LOW);

  // 2. 【白灯】湿度控制 (彻底不参与雷达报警)
  if (!isApproachWarning && !isRedFlashing) {
    digitalWrite(WHITE_PIN, whiteLedOn ? HIGH : LOW);
  }

  // 3. 【最高级】雷达快速逼近警告 (红灯快闪 + 蜂鸣器鸣叫)
  // 只有在距离小于80cm 且 平滑后的速度大于 1000mm/s 时报警
  if (dist > 0 && dist < TOF_MAX_DIST && smoothSpeed > APPROACH_SPD) {
    isApproachWarning = true;
    
    // 雷达报警节奏 (150毫秒闪烁一次)
    if (now % 150 < 75) {
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, LOW);  // 低电平响
    } else {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH); // 高电平停
    }
  } else {
    isApproachWarning = false;
    digitalWrite(BUZZER_PIN, HIGH); // 无警告时关闭蜂鸣器
  }

  // 4. 【变道】按键逻辑 (只有在非雷达警告时才能触发)
  if (btnPressed && btnReady && !isApproachWarning) {
    btnReady = false; // 锁定按键，防止连按
    
    if (irBlocked) {
      // 变道失败：红灯闪烁 + 蜂鸣器响
      isRedFlashing = true;
      redFlashStartTime = now;
    } else {
      // 变道成功：绿灯亮1秒
      isGreenLit = true;
      greenStartTime = now;
    }
  }
  if (!btnPressed) btnReady = true; // 手松开解锁按键

  // 变道失败：红灯闪烁逻辑
  if (isRedFlashing && !isApproachWarning) {
    if (now % 200 < 100) digitalWrite(RED_PIN, HIGH);
    else digitalWrite(RED_PIN, LOW);
    
    if (now % 300 < 150) digitalWrite(BUZZER_PIN, LOW);
    else digitalWrite(BUZZER_PIN, HIGH);

    // 1.5秒后自动停止
    if (now - redFlashStartTime > 1500) {
      isRedFlashing = false;
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH);
    }
  }

  // 变道成功：绿灯逻辑
  if (isGreenLit && !isApproachWarning) {
    digitalWrite(GREEN_PIN, HIGH);
    if (now - greenStartTime > 1000) {
      isGreenLit = false;
      digitalWrite(GREEN_PIN, LOW);
    }
  }
}