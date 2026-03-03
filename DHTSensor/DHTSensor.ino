#include "DHT.h"

// 定义 DHT 传感器引脚和类型
#define DHTPIN_12 12
#define DHTPIN_13 13
#define DHTTYPE DHT22   // DHT 22 (AM2302)

// 创建两个 DHT 实例
DHT dht12(DHTPIN_12, DHTTYPE);
DHT dht13(DHTPIN_13, DHTTYPE);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // 等待串口连接
  }

  Serial.println("DHT22 传感器测试 - 引脚 12 和 13");
  Serial.println("================================");

  // 启动两个传感器
  dht12.begin();
  dht13.begin();

  Serial.println("传感器已初始化，开始读取数据...\n");
}

void loop() {
  // 等待 2 秒（DHT22 读取间隔至少 2 秒）
  delay(2000);

  Serial.println("---------- 读取结果 ----------");

  // 读取引脚 12 的传感器
  readSensor(dht12, 12);

  // 读取引脚 13 的传感器
  readSensor(dht13, 13);

  Serial.println("");
}

void readSensor(DHT &dht, int pin) {
  // 读取湿度
  float h = dht.readHumidity();
  // 读取温度（摄氏度）
  float t = dht.readTemperature();

  // 检查是否读取失败
  if (isnan(h) || isnan(t)) {
    Serial.print("引脚 ");
    Serial.print(pin);
    Serial.println(": ❌ 读取失败 - 可能未连接 DHT22 传感器");
  } else {
    Serial.print("引脚 ");
    Serial.print(pin);
    Serial.print(": ✓ 已连接 - 温度: ");
    Serial.print(t);
    Serial.print("°C, 湿度: ");
    Serial.print(h);
    Serial.println("%");
  }
}
