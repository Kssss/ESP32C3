#include <HardwareSerial.h>

// 硬件配置
HardwareSerial BMSerial(0); // RX=GPIO20, TX=GPIO21

// 协议常量
#define FRAME_HEADER_1 0xA5
#define FRAME_HEADER_2 0x5A
#define FRAME_LENGTH 0x5D
#define COMMAND_CODE 0x82
#define START_ADDRESS 0x1000
#define BAUDRATE 2400

// 数据偏移量（相对于数据区的偏移）
#define OFFSET_VOLTAGE 0
#define OFFSET_CURRENT 2
#define OFFSET_SOC 6
#define OFFSET_TEMPERATURE 12

// 完整的帧长度
#define TOTAL_FRAME_LENGTH (2 + 1 + 1 + 2 + 90 ) // 头2 + 长度1 + 命令1 + 地址2 + 数据90 + 校验1

// 电池数据结构体
struct BatteryData {
  float voltage, current, temperature;
  float minTemp, maxTemp, avgTemp;
  float power;
  uint8_t soc;
  String status;
  unsigned long validFrames, invalidFrames, totalSamples, lastUpdate;
};

BatteryData batteryData;
float temperatureSum = 0.0;

void setup() {
  Serial.begin(115200);
  BMSerial.begin(BAUDRATE, SERIAL_8N1, 20, 21);
  
  initBatteryData();
  
  Serial.println("\n🔋 ESP32-C3 电池监控系统 (精简版)");
  Serial.println("📊 仅串口输出，无网络功能");
  Serial.println("=================================");
}

void initBatteryData() {
  memset(&batteryData, 0, sizeof(batteryData));
  batteryData.minTemp = 100.0;
  batteryData.maxTemp = -100.0;
  temperatureSum = 0.0;
}

void loop() {
  if (receiveAndParseFrame()) {
    displaySerialData();
  }
  delay(100); // 适当延迟
}

bool receiveAndParseFrame() {
  static uint8_t frameBuffer[TOTAL_FRAME_LENGTH];
  static int bufferIndex = 0;
  
  while (BMSerial.available() > 0) {
    uint8_t byte = BMSerial.read();
    
    // 寻找帧头
    if (bufferIndex == 0 && byte == FRAME_HEADER_1) {
      frameBuffer[bufferIndex++] = byte;
    } 
    else if (bufferIndex == 1) {
      if (byte == FRAME_HEADER_2) {
        frameBuffer[bufferIndex++] = byte;
      } else {
        bufferIndex = 0;
        batteryData.invalidFrames++;
      }
    }
    else if (bufferIndex >= 2) {
      frameBuffer[bufferIndex++] = byte;
      
      // 检查帧长度字段
      if (bufferIndex == 3 && frameBuffer[2] != FRAME_LENGTH) {
        bufferIndex = 0;
        batteryData.invalidFrames++;
        return false;
      }
      
      // 检查命令码字段
      if (bufferIndex == 4 && frameBuffer[3] != COMMAND_CODE) {
        bufferIndex = 0;
        batteryData.invalidFrames++;
        return false;
      }
      
      // 完整帧接收完成
      if (bufferIndex >= TOTAL_FRAME_LENGTH) {
        bool isValid = validateFrame(frameBuffer);
        if (isValid) {
          processFrameData(frameBuffer);
          batteryData.validFrames++;
        } else {
          batteryData.invalidFrames++;
        }
        bufferIndex = 0;
        return isValid;
      }
    }
  }
  return false;
}

bool validateFrame(uint8_t* frame) {
  return true;
  // // 计算校验和（从帧头到数据结束，不包括校验和本身）
  // uint8_t checksum = 0;
  // for (int i = 0; i < TOTAL_FRAME_LENGTH - 1; i++) {
  //   checksum += frame[i];
  // }
  
  // // 校验和是最后1个字节
  // return checksum == frame[TOTAL_FRAME_LENGTH - 1];
}

void processFrameData(uint8_t* frame) {
  // 数据区从第6字节开始（跳过：头2 + 长度1 + 命令1 + 地址2）
  uint8_t* data = &frame[6];
  
  // 解析电压 (UINT16, 单位10mV)
  batteryData.voltage = ((data[OFFSET_VOLTAGE] << 8) | data[OFFSET_VOLTAGE + 1]) * 0.01f;
  
  // 解析电流 (INT16, 单位0.1A)
  int16_t rawCurrent = (data[OFFSET_CURRENT] << 8) | data[OFFSET_CURRENT + 1];
  batteryData.current = rawCurrent * 0.1f;
  
  // 解析电量 (UINT16, 单位%)
  batteryData.soc = (data[OFFSET_SOC] << 8) | data[OFFSET_SOC + 1];
  
  // 解析温度
  batteryData.temperature = (data[OFFSET_TEMPERATURE] << 8) | data[OFFSET_TEMPERATURE + 1];
  
  // 更新统计信息
  updateStats();
  batteryData.lastUpdate = millis();
}

void updateStats() {
  batteryData.minTemp = min(batteryData.minTemp, batteryData.temperature);
  batteryData.maxTemp = max(batteryData.maxTemp, batteryData.temperature);
  
  temperatureSum += batteryData.temperature;
  batteryData.totalSamples++;
  batteryData.avgTemp = temperatureSum / batteryData.totalSamples;
  
  batteryData.power = batteryData.voltage * batteryData.current;
  batteryData.status = (batteryData.current > 0.1) ? "放电" : 
                      (batteryData.current < -0.1) ? "充电" : "静置";
}

void displaySerialData() {
  static unsigned long lastDisplay = 0;
  unsigned long currentTime = millis();
  
  // 每500ms显示一次，避免串口输出过于频繁
  if (currentTime - lastDisplay >= 500) {
    Serial.printf("⚡ 电压:%.1fV 电流:%.1fA 电量:%d%% 温度:%.1f°C 功率:%.1fW %s\n",
      batteryData.voltage, batteryData.current, batteryData.soc, 
      batteryData.temperature, batteryData.power, batteryData.status.c_str());
    
    // 每10次显示一次统计信息
    static int displayCount = 0;
    if (++displayCount >= 10) {
      displayCount = 0;
      Serial.printf("📊 统计: 有效帧:%lu 无效帧:%lu 采样数:%lu 成功率:%.1f%%\n",
        batteryData.validFrames, batteryData.invalidFrames, batteryData.totalSamples,
        (batteryData.validFrames * 100.0) / (batteryData.validFrames + batteryData.invalidFrames));
      Serial.println("---------------------------------");
    }
    
    lastDisplay = currentTime;
  }
}

// 串口命令处理函数（可选）
void handleSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "reset") {
      resetStats();
      Serial.println("📊 统计已重置");
    }
    else if (command == "status") {
      displayDetailedStatus();
    }
    else if (command == "help") {
      Serial.println("可用命令:");
      Serial.println("reset - 重置统计信息");
      Serial.println("status - 显示详细状态");
      Serial.println("help - 显示帮助信息");
    }
  }
}

void resetStats() {
  batteryData.minTemp = 100.0;
  batteryData.maxTemp = -100.0;
  batteryData.avgTemp = 0.0;
  temperatureSum = 0.0;
  batteryData.totalSamples = 0;
  batteryData.validFrames = 0;
  batteryData.invalidFrames = 0;
}

void displayDetailedStatus() {
  Serial.println("\n=== 电池详细状态 ===");
  Serial.printf("电压: %.2f V\n", batteryData.voltage);
  Serial.printf("电流: %.2f A\n", batteryData.current);
  Serial.printf("电量: %d %%\n", batteryData.soc);
  Serial.printf("温度: %.1f °C\n", batteryData.temperature);
  Serial.printf("功率: %.1f W\n", batteryData.power);
  Serial.printf("状态: %s\n", batteryData.status.c_str());
  Serial.printf("温度范围: %.1f ~ %.1f °C (平均: %.1f °C)\n", 
    batteryData.minTemp, batteryData.maxTemp, batteryData.avgTemp);
  Serial.printf("数据统计: 有效帧:%lu 无效帧:%lu 成功率:%.1f%%\n",
    batteryData.validFrames, batteryData.invalidFrames,
    (batteryData.validFrames * 100.0) / (batteryData.validFrames + batteryData.invalidFrames));
  Serial.println("==================\n");
}