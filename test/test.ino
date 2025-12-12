#include <Servo.h>

// === 引脚定义 (用户校正) ===
const int CH_RIGHT_IN = 2;   // RX 右通道 PWM 输入（Pin 2）
const int CH_LEFT_IN  = 3;   // RX 左通道 PWM 输入（Pin 3）

const int ESC_RIGHT_OUT = 9;   // 右侧电调(ESC)输出
const int ESC_LEFT_OUT  = 10;  // 左侧电调(ESC)输出

// === 时序参数 ===
const unsigned long FAILSAFE_MS      = 200;    // 故障保护超时 200ms

// === RX 有效范围 (参考 original.ino) ===
const int RX_VALID_MIN = 950;   // <950 视为断开或故障
const int RX_VALID_MAX = 2000;  // >2000 视为断开或故障
const int RC_MID       = 1500;  // 中间值
const int DEADBAND_US  = 25;    // 中位死区 ±25µs

// === ESC 范围 ===
const int ESC_MIN = 1100;
const int ESC_MID = 1500;
const int ESC_MAX = 1900;

unsigned long lastUpdateL = 0, lastUpdateR = 0;
unsigned long loopCount = 0;

// === 中断捕获相关 ===
volatile unsigned long rRiseMicros = 0, rPulseMicros = 0;
volatile unsigned long lRiseMicros = 0, lPulseMicros = 0;

// 简单滤波（滑动平均）
int avgR = ESC_MID;
int avgL = ESC_MID;
const int FILTER_ALPHA = 20; // 百分比（越大越跟随输入）
const int MAX_STEP_US   = 10; // 每循环最大变化（软启动斜坡限制）

// === PWM 档位分级设置 ===
const bool ENABLE_GEAR_MODE = true;  // 是否启用档位模式（false=连续模式）
const int NUM_GEARS = 9;             // 档位数量（含中位档）

// 档位定义：9档模式（后退4档 | 中位 | 前进4档）- 每档间隔100µs
const int GEAR_VALUES[NUM_GEARS] = {
  1100,  // 档位1：后退最高速 (-400µs)
  1200,  // 档位2：后退高速   (-300µs)
  1300,  // 档位3：后退中速   (-200µs)
  1400,  // 档位4：后退低速   (-100µs)
  1500,  // 档位5：中位停止   (0µs)
  1600,  // 档位6：前进低速   (+100µs)
  1700,  // 档位7：前进中速   (+200µs)
  1800,  // 档位8：前进高速   (+300µs)
  1900   // 档位9：前进最高速 (+400µs)
};

// 档位切换阈值（输入信号范围划分）
const int GEAR_THRESHOLDS[NUM_GEARS - 1] = {
  1050,  // < 1050: 档位1
  1150,  // 1050-1150: 档位2
  1250,  // 1150-1250: 档位3
  1350,  // 1250-1350: 档位4
  1450,  // 1350-1450: 档位5 (中位)
  1550,  // 1450-1550: 档位5, 1550-1650: 档位6
  1650,  // 1650-1750: 档位7
  1750   // 1750-1850: 档位8, > 1850: 档位9
};

// 右通道上升沿/下降沿捕获
void onRightChange() {
  int level = digitalRead(CH_RIGHT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    rRiseMicros = now;
  } else {
    unsigned long width = now - rRiseMicros;
    // 仅接受合理范围的脉宽，避免噪声
    if (width >= 800 && width <= 2200) {
      rPulseMicros = width;
    }
  }
}

// 左通道上升沿/下降沿捕获
void onLeftChange() {
  int level = digitalRead(CH_LEFT_IN);
  unsigned long now = micros();
  if (level == HIGH) {
    lRiseMicros = now;
  } else {
    unsigned long width = now - lRiseMicros;
    if (width >= 800 && width <= 2200) {
      lPulseMicros = width;
    }
  }
}

Servo escL, escR;

// 将输入信号映射到档位
int mapToGear(unsigned long inUs) {
  if (inUs == 0) return ESC_MID; // 超时/无脉冲
  if ((int)inUs < RX_VALID_MIN || (int)inUs > RX_VALID_MAX) return ESC_MID; // 非法范围
  if (abs((int)inUs - RC_MID) <= DEADBAND_US) return ESC_MID; // 中位保持
  
  // 根据输入值选择档位（9档）
  for (int i = 0; i < NUM_GEARS - 1; i++) {
    if (inUs < GEAR_THRESHOLDS[i]) {
      return GEAR_VALUES[i];
    }
  }
  return GEAR_VALUES[NUM_GEARS - 1];  // 最高档位
}

// 规则映射: 950..2000 → 1100..1900；超出范围/超时返回 1500
int mapLinearToEsc(unsigned long inUs) {
  if (inUs == 0) return ESC_MID; // 超时/无脉冲
  if ((int)inUs < RX_VALID_MIN || (int)inUs > RX_VALID_MAX) return ESC_MID; // 非法范围
  if (abs((int)inUs - RC_MID) <= DEADBAND_US) return ESC_MID; // 中位保持

  // 线性映射，并夹紧到 ESC 范围
  long out = map((long)inUs, RX_VALID_MIN, RX_VALID_MAX, ESC_MIN, ESC_MAX);
  if (out < ESC_MIN) out = ESC_MIN;
  if (out > ESC_MAX) out = ESC_MAX;
  return (int)out;
}

// 统一映射函数：根据模式选择连续或档位映射
int mapToEsc(unsigned long inUs) {
  if (ENABLE_GEAR_MODE) {
    return mapToGear(inUs);
  } else {
    return mapLinearToEsc(inUs);
  }
}

void setup() {
  pinMode(CH_LEFT_IN,  INPUT);
  pinMode(CH_RIGHT_IN, INPUT);

  Serial.begin(115200);
  while (!Serial) {}

  // 启用中断捕获（UNO 上 2/3 支持外部中断）
  attachInterrupt(digitalPinToInterrupt(CH_RIGHT_IN), onRightChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH_LEFT_IN),  onLeftChange,  CHANGE);

  // 附加 ESC 输出并进行 1500µs 初始化
  escR.attach(ESC_RIGHT_OUT);
  escL.attach(ESC_LEFT_OUT);
  escR.writeMicroseconds(ESC_MID);
  escL.writeMicroseconds(ESC_MID);
  delay(2000); // 让 ESC 进入安全中位

  Serial.println("========================================");
  Serial.println("PWM 信号测试 - Pin 2(右) 和 Pin 3(左)");
  Serial.print("控制模式: ");
  Serial.println(ENABLE_GEAR_MODE ? "档位模式 (9档, 100µs间隔)" : "连续模式");
  Serial.println("========================================");
  Serial.println("格式: Loop | 右(Pin2)_PWM | 右_状态 | 左(Pin3)_PWM | 左_状态 | 距上次更新时间 | outL|outR");
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  loopCount++;

  // === 读取中断捕获到的脉宽（非阻塞）===
  noInterrupts();
  unsigned long inR = rPulseMicros;
  unsigned long inL = lPulseMicros;
  interrupts();

  // 记录最后更新时间
  if (inL >= RX_VALID_MIN && inL <= RX_VALID_MAX) lastUpdateL = now;
  if (inR >= RX_VALID_MIN && inR <= RX_VALID_MAX) lastUpdateR = now;

  // === 将输入脉冲映射为 ESC 输出 ===
  int outL = mapToEsc(inL);
  int outR = mapToEsc(inR);

  // 简单低通滤波，避免抖动导致电调断续
  int filtL = (avgL * (100 - FILTER_ALPHA) + outL * FILTER_ALPHA) / 100;
  int filtR = (avgR * (100 - FILTER_ALPHA) + outR * FILTER_ALPHA) / 100;

  // 软启动斜坡：限制每次更新的最大变化量
  int deltaL = filtL - avgL;
  if (deltaL >  MAX_STEP_US) deltaL =  MAX_STEP_US;
  if (deltaL < -MAX_STEP_US) deltaL = -MAX_STEP_US;
  avgL += deltaL;

  int deltaR = filtR - avgR;
  if (deltaR >  MAX_STEP_US) deltaR =  MAX_STEP_US;
  if (deltaR < -MAX_STEP_US) deltaR = -MAX_STEP_US;
  avgR += deltaR;

  // 备份故障保护：超时则置中
  if (now - lastUpdateL > FAILSAFE_MS) avgL = ESC_MID;
  if (now - lastUpdateR > FAILSAFE_MS) avgR = ESC_MID;

  // 驱动 ESC
  escR.writeMicroseconds(avgR);
  escL.writeMicroseconds(avgL);

  // === 输出详细测试结果 ===
  Serial.print("[");
  Serial.print(loopCount);
  Serial.print("] ");

  // Pin 2 → 右通道
  Serial.print("右(Pin2): ");
  Serial.print(inR);
  Serial.print("μs ");
  if (inR == 0) {
    Serial.print("(超时) ");
  } else if (inR < RX_VALID_MIN) {
    Serial.print("(过低) ");
  } else if (inR > RX_VALID_MAX) {
    Serial.print("(过高) ");
  } else if (abs((int)inR - RC_MID) <= 25) {
    Serial.print("(中位) ");
  } else if (inR < RC_MID) {
    Serial.print("(后退) ");
  } else {
    Serial.print("(前进) ");
  }

  // Pin 3 → 左通道
  Serial.print("| 左(Pin3): ");
  Serial.print(inL);
  Serial.print("μs ");
  if (inL == 0) {
    Serial.print("(超时) ");
  } else if (inL < RX_VALID_MIN) {
    Serial.print("(过低) ");
  } else if (inL > RX_VALID_MAX) {
    Serial.print("(过高) ");
  } else if (abs((int)inL - RC_MID) <= 25) {
    Serial.print("(中位) ");
  } else if (inL < RC_MID) {
    Serial.print("(后退) ");
  } else {
    Serial.print("(前进) ");
  }

  // 显示距离上次有效更新的时间
  Serial.print("| 距上次更新: L=");
  Serial.print(now - lastUpdateL);
  Serial.print("ms R=");
  Serial.print(now - lastUpdateR);
  Serial.print("ms");

  // 故障保护检测
  if (now - lastUpdateL > FAILSAFE_MS) {
    Serial.print(" [左通道故障保护]");
  }
  if (now - lastUpdateR > FAILSAFE_MS) {
    Serial.print(" [右通道故障保护]");
  }

  // 打印 ESC 输出对比
  Serial.print(" | outL|outR=");
  Serial.print(avgL);
  Serial.print('|');
  Serial.print(avgR);

  Serial.println();

  delay(5);  // 参考 original.ino 使用 5ms 延迟
}

// 仅每秒打印一次状态
void loop() {
  unsigned long now = millis();
  loopCount++;

  // === 读取中断捕获到的脉宽（非阻塞）===
  noInterrupts();
  unsigned long inR = rPulseMicros;
  unsigned long inL = lPulseMicros;
  interrupts();

  // 记录最后更新时间
  if (inL >= RX_VALID_MIN && inL <= RX_VALID_MAX) lastUpdateL = now;
  if (inR >= RX_VALID_MIN && inR <= RX_VALID_MAX) lastUpdateR = now;

  // === 将输入脉冲映射为 ESC 输出 ===
  int outL = mapToEsc(inL);
  int outR = mapToEsc(inR);

  // 简单低通滤波，避免抖动导致电调断续
  int filtL = (avgL * (100 - FILTER_ALPHA) + outL * FILTER_ALPHA) / 100;
  int filtR = (avgR * (100 - FILTER_ALPHA) + outR * FILTER_ALPHA) / 100;

  // 软启动斜坡：限制每次更新的最大变化量
  int deltaL = filtL - avgL;
  if (deltaL >  MAX_STEP_US) deltaL =  MAX_STEP_US;
  if (deltaL < -MAX_STEP_US) deltaL = -MAX_STEP_US;
  avgL += deltaL;

  int deltaR = filtR - avgR;
  if (deltaR >  MAX_STEP_US) deltaR =  MAX_STEP_US;
  if (deltaR < -MAX_STEP_US) deltaR = -MAX_STEP_US;
  avgR += deltaR;

  // 备份故障保护：超时则置中
  if (now - lastUpdateL > FAILSAFE_MS) avgL = ESC_MID;
  if (now - lastUpdateR > FAILSAFE_MS) avgR = ESC_MID;

  // 驱动 ESC
  escR.writeMicroseconds(avgR);
  escL.writeMicroseconds(avgL);

  // === 每秒输出一次详细测试结果 ===
  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs >= 1000) {
    lastPrintMs = now;
    
    Serial.print("[");
    Serial.print(loopCount);
    Serial.print("] ");

    // Pin 2 → 右通道
    Serial.print("右(Pin2): ");
    Serial.print(inR);
    Serial.print("μs ");
    if (inR == 0) {
      Serial.print("(超时) ");
    } else if (inR < RX_VALID_MIN) {
      Serial.print("(过低) ");
    } else if (inR > RX_VALID_MAX) {
      Serial.print("(过高) ");
    } else if (abs((int)inR - RC_MID) <= 25) {
      Serial.print("(中位) ");
    } else if (inR < RC_MID) {
      Serial.print("(后退) ");
    } else {
      Serial.print("(前进) ");
    }

    // Pin 3 → 左通道
    Serial.print("| 左(Pin3): ");
    Serial.print(inL);
    Serial.print("μs ");
    if (inL == 0) {
      Serial.print("(超时) ");
    } else if (inL < RX_VALID_MIN) {
      Serial.print("(过低) ");
    } else if (inL > RX_VALID_MAX) {
      Serial.print("(过高) ");
    } else if (abs((int)inL - RC_MID) <= 25) {
      Serial.print("(中位) ");
    } else if (inL < RC_MID) {
      Serial.print("(后退) ");
    } else {
      Serial.print("(前进) ");
    }

    // 显示距离上次有效更新的时间
    Serial.print("| 距上次更新: L=");
    Serial.print(now - lastUpdateL);
    Serial.print("ms R=");
    Serial.print(now - lastUpdateR);
    Serial.print("ms");

    // 故障保护检测
    if (now - lastUpdateL > FAILSAFE_MS) {
      Serial.print(" [左通道故障保护]");
    }
    if (now - lastUpdateR > FAILSAFE_MS) {
      Serial.print(" [右通道故障保护]");
    }

    // 打印 ESC 输出对比
    Serial.print(" | outL|outR=");
    Serial.print(avgL);
    Serial.print('|');
    Serial.print(avgR);

    Serial.println();
  }

  delay(5);  // 参考 original.ino 使用 5ms 延迟
}
