/*
 * ESP32-S3 音频采集 + BLE接收器 集成版本
 * 
 * 功能说明:
 * 1. 保持原有的OPPO手表BLE广播接收功能
 * 2. 保持原有的LED控制功能
 * 3. 新增INMP441麦克风音频采集功能
 * 4. 支持串口命令控制音频采集开关
 * 
 * 硬件连接:
 * - LED: 板载LED (LED_BUILTIN)
 * - INMP441麦克风:
 *   VCC → 3.3V
 *   GND → GND
 *   SCK → GPIO 5 (I2S时钟)
 *   WS → GPIO 4 (I2S字选择)
 *   SD → GPIO 6 (I2S数据)
 *   L/R → GND (左声道)
 * 
 * 作者: ESP32-S3开发团队
 * 版本: v2.0 (集成音频采集功能)
 * 日期: 2024
 */

#include "BLEDevice.h"
#include "BLEScan.h"
#include "BLEAdvertisedDevice.h"
#include "driver/i2s.h"
#include <WiFi.h>

// ==================== 硬件配置 ====================
// LED配置
#define LED_PIN LED_BUILTIN  // 使用板载LED

// I2S音频配置
#define I2S_PORT I2S_NUM_0
#define I2S_SCK_PIN 5       // 时钟引脚
#define I2S_WS_PIN 4        // 字选择引脚
#define I2S_SD_PIN 6        // 数据引脚
#define SAMPLE_RATE 16000   // 采样率 16kHz
#define SAMPLE_BITS 16      // 位深度 16位
#define BUFFER_SIZE 1024    // 缓冲区大小

// ==================== BLE扫描配置 ====================
#define SCAN_TIME 5          // 扫描时间(秒)
#define SCAN_INTERVAL 1349   // 扫描间隔
#define SCAN_WINDOW 449      // 扫描窗口

// ==================== 协议定义 ====================
// OPPO手表BLE广播协议
#define COMPANY_ID 0xFFFF    // 自定义厂商ID
#define PROTOCOL_VERSION 0x01 // 协议版本

// 命令定义 (与OPPO手表应用保持一致)
#define CMD_ON 0x01          // 开启LED
#define CMD_OFF 0x02         // 关闭LED
#define CMD_TOGGLE 0x03      // 切换LED状态

// ==================== 全局变量 ====================
// BLE相关
BLEScan* pBLEScan;
bool ledState = false;
unsigned long lastCommandTime = 0;  // 上次命令时间
const unsigned long COMMAND_COOLDOWN = 500;  // 命令冷却时间(ms)

// 音频相关
bool audioEnabled = false;
bool audioInitialized = false;
int32_t audioBuffer[BUFFER_SIZE];
unsigned long lastAudioStats = 0;
unsigned long audioSampleCount = 0;
float audioRMS = 0.0;
int audioZeroCrossings = 0;
int audioClippingCount = 0;

// ==================== LED控制函数 ====================
void initLED() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  ledState = false;
  Serial.println("LED初始化完成 - 引脚: " + String(LED_PIN));
  Serial.flush();
}

void setLED(bool state) {
  digitalWrite(LED_PIN, state ? HIGH : LOW);
  ledState = state;
  Serial.println("LED状态: " + String(state ? "开启" : "关闭"));
  Serial.flush();
}

void toggleLED() {
  ledState = !ledState;
  setLED(ledState);
}

// ==================== I2S音频初始化 ====================
bool initI2S() {
  if (audioInitialized) {
    Serial.println("I2S已经初始化");
    return true;
  }
  
  Serial.println("初始化I2S音频接口...");
  
  // I2S配置
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  
  // I2S引脚配置
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };
  
  // 安装I2S驱动
  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    Serial.println("I2S驱动安装失败: " + String(result));
    return false;
  }
  
  // 设置I2S引脚
  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK) {
    Serial.println("I2S引脚设置失败: " + String(result));
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  
  // 启动I2S
  result = i2s_start(I2S_PORT);
  if (result != ESP_OK) {
    Serial.println("I2S启动失败: " + String(result));
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
  
  audioInitialized = true;
  Serial.println("I2S音频接口初始化成功");
  Serial.println("采样率: " + String(SAMPLE_RATE) + " Hz");
  Serial.println("位深度: " + String(SAMPLE_BITS) + " 位");
  Serial.println("缓冲区大小: " + String(BUFFER_SIZE));
  Serial.flush();
  
  return true;
}

void deinitI2S() {
  if (!audioInitialized) {
    return;
  }
  
  i2s_stop(I2S_PORT);
  i2s_driver_uninstall(I2S_PORT);
  audioInitialized = false;
  audioEnabled = false;
  
  Serial.println("I2S音频接口已关闭");
  Serial.flush();
}

// ==================== 音频数据处理 ====================
void processAudioData() {
  if (!audioEnabled || !audioInitialized) {
    return;
  }
  
  size_t bytesRead = 0;
  esp_err_t result = i2s_read(I2S_PORT, audioBuffer, sizeof(audioBuffer), &bytesRead, 10);
  
  if (result != ESP_OK || bytesRead == 0) {
    return;
  }
  
  int samplesRead = bytesRead / sizeof(int32_t);
  audioSampleCount += samplesRead;
  
  // 音频质量分析
  analyzeAudioQuality(audioBuffer, samplesRead);
  
  // 每5秒输出一次音频统计信息
  unsigned long currentTime = millis();
  if (currentTime - lastAudioStats >= 5000) {
    lastAudioStats = currentTime;
    printAudioStats();
  }
}

void analyzeAudioQuality(int32_t* buffer, int sampleCount) {
  if (sampleCount == 0) return;
  
  float sum = 0.0;
  int zeroCrossings = 0;
  int clipping = 0;
  int32_t prevSample = 0;
  
  for (int i = 0; i < sampleCount; i++) {
    // 转换为16位样本
    int16_t sample = (int16_t)(buffer[i] >> 16);
    
    // RMS计算
    sum += (float)sample * sample;
    
    // 零交叉检测
    if (i > 0) {
      int16_t prevSample16 = (int16_t)(prevSample >> 16);
      if ((prevSample16 >= 0 && sample < 0) || (prevSample16 < 0 && sample >= 0)) {
        zeroCrossings++;
      }
    }
    
    // 削波检测
    if (abs(sample) > 30000) {
      clipping++;
    }
    
    prevSample = buffer[i];
  }
  
  // 更新统计信息
  audioRMS = sqrt(sum / sampleCount);
  audioZeroCrossings += zeroCrossings;
  audioClippingCount += clipping;
}

void printAudioStats() {
  Serial.println("\n=== 音频统计信息 ===");
  Serial.println("采样数量: " + String(audioSampleCount));
  Serial.println("RMS幅度: " + String(audioRMS, 2));
  Serial.println("零交叉次数: " + String(audioZeroCrossings));
  Serial.println("削波次数: " + String(audioClippingCount));
  
  // 音频质量评估
  if (audioRMS < 100) {
    Serial.println("音频状态: 静音或信号过弱");
  } else if (audioRMS > 10000) {
    Serial.println("音频状态: 信号过强，可能削波");
  } else {
    Serial.println("音频状态: 正常");
  }
  
  if (audioClippingCount > 0) {
    Serial.println("警告: 检测到音频削波，请降低输入音量");
  }
  
  Serial.println("==================\n");
  Serial.flush();
  
  // 重置计数器
  audioZeroCrossings = 0;
  audioClippingCount = 0;
}

// ==================== BLE广播处理 ====================
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // 添加设备发现日志 (调试用，可选)
    // Serial.println("[DEBUG] 发现BLE设备: " + advertisedDevice.getAddress().toString());
    
    // 检查是否有厂商数据
    if (!advertisedDevice.haveManufacturerData()) {
      return;
    }
    
    String manufacturerData = advertisedDevice.getManufacturerData();
    
    // 检查数据长度 (至少需要5字节: 2字节厂商ID + 3字节协议数据)
    if (manufacturerData.length() < 5) {
      return;
    }
    
    // 解析厂商ID (小端序)
    uint16_t companyId = (uint8_t)manufacturerData[1] << 8 | (uint8_t)manufacturerData[0];
    
    // 检查是否为目标厂商ID
    if (companyId != COMPANY_ID) {
      return;
    }
    
    // 解析协议数据: [厂商ID(2字节)] [版本(1字节)] [命令(1字节)] [序列号(1字节)]
    uint8_t version = (uint8_t)manufacturerData[2];
    uint8_t command = (uint8_t)manufacturerData[3];
    uint8_t sequence = (uint8_t)manufacturerData[4];
    
    // 检查协议版本
    if (version != PROTOCOL_VERSION) {
      Serial.println("协议版本不匹配: " + String(version));
      Serial.flush();
      return;
    }
    
    // 防止重复处理命令
    unsigned long currentTime = millis();
    if (currentTime - lastCommandTime < COMMAND_COOLDOWN) {
      return;
    }
    lastCommandTime = currentTime;
    
    // 打印接收到的命令信息
    Serial.println("\n=== 接收到OPPO手表命令 ===");
    Serial.println("设备地址: " + advertisedDevice.getAddress().toString());
    Serial.println("信号强度: " + String(advertisedDevice.getRSSI()) + " dBm");
    Serial.println("厂商ID: 0x" + String(companyId, HEX));
    Serial.println("协议版本: 0x" + String(version, HEX));
    Serial.println("命令: 0x" + String(command, HEX));
    Serial.println("序列号: " + String(sequence));
    Serial.flush();
    
    // 执行命令
    processCommand(command);
  }
};

// ==================== 命令处理函数 ====================
void processCommand(uint8_t command) {
  switch (command) {
    case CMD_ON:
      Serial.println("执行命令: 开启LED");
      Serial.flush();
      setLED(true);
      break;
      
    case CMD_OFF:
      Serial.println("执行命令: 关闭LED");
      Serial.flush();
      setLED(false);
      break;
      
    case CMD_TOGGLE:
      Serial.println("执行命令: 切换LED状态");
      Serial.flush();
      toggleLED();
      break;
      
    default:
      Serial.println("未知命令: 0x" + String(command, HEX));
      Serial.flush();
      break;
  }
  
  Serial.println("命令执行完成\n");
  Serial.flush();
}

// ==================== 蓝牙初始化 ====================
void initBLE() {
  Serial.println("初始化BLE设备...");
  Serial.flush();
  
  BLEDevice::init("ESP32-S3-OPPO-Audio-Receiver");
  
  // 创建BLE扫描器
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);  // 主动扫描
  pBLEScan->setInterval(SCAN_INTERVAL);
  pBLEScan->setWindow(SCAN_WINDOW);
  
  Serial.println("BLE扫描器初始化完成");
  Serial.println("扫描间隔: " + String(SCAN_INTERVAL) + "ms");
  Serial.println("扫描窗口: " + String(SCAN_WINDOW) + "ms");
  Serial.println("目标厂商ID: 0x" + String(COMPANY_ID, HEX));
  Serial.println("协议版本: 0x" + String(PROTOCOL_VERSION, HEX));
  Serial.flush();
}

// ==================== 串口命令处理 ====================
void processSerialCommand() {
  if (!Serial.available()) {
    return;
  }
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  
  Serial.println("\n收到串口命令: " + command);
  
  if (command == "audio on" || command == "audio start") {
    if (!audioInitialized) {
      if (initI2S()) {
        audioEnabled = true;
        Serial.println("音频采集已开启");
      } else {
        Serial.println("音频采集开启失败");
      }
    } else {
      audioEnabled = true;
      Serial.println("音频采集已开启");
    }
  }
  else if (command == "audio off" || command == "audio stop") {
    audioEnabled = false;
    Serial.println("音频采集已关闭");
  }
  else if (command == "audio status") {
    Serial.println("音频状态:");
    Serial.println("- 初始化: " + String(audioInitialized ? "是" : "否"));
    Serial.println("- 采集中: " + String(audioEnabled ? "是" : "否"));
    if (audioInitialized) {
      Serial.println("- 采样率: " + String(SAMPLE_RATE) + " Hz");
      Serial.println("- 位深度: " + String(SAMPLE_BITS) + " 位");
    }
  }
  else if (command == "audio test") {
    if (audioInitialized && audioEnabled) {
      printAudioStats();
    } else {
      Serial.println("音频采集未启用，请先执行 'audio on'");
    }
  }
  else if (command == "led on") {
    setLED(true);
  }
  else if (command == "led off") {
    setLED(false);
  }
  else if (command == "led toggle") {
    toggleLED();
  }
  else if (command == "status") {
    printSystemInfo();
  }
  else if (command == "help") {
    printHelp();
  }
  else {
    Serial.println("未知命令: " + command);
    Serial.println("输入 'help' 查看可用命令");
  }
  
  Serial.flush();
}

void printHelp() {
  Serial.println("\n=== 可用命令 ===");
  Serial.println("音频控制:");
  Serial.println("  audio on/start  - 开启音频采集");
  Serial.println("  audio off/stop  - 关闭音频采集");
  Serial.println("  audio status    - 查看音频状态");
  Serial.println("  audio test      - 显示音频统计");
  Serial.println("LED控制:");
  Serial.println("  led on          - 开启LED");
  Serial.println("  led off         - 关闭LED");
  Serial.println("  led toggle      - 切换LED状态");
  Serial.println("系统信息:");
  Serial.println("  status          - 显示系统信息");
  Serial.println("  help            - 显示此帮助");
  Serial.println("================\n");
}

// ==================== 系统状态监控 ====================
void printSystemInfo() {
  Serial.println("\n=== ESP32-S3 音频+BLE 系统信息 ===");
  Serial.println("芯片型号: " + String(ESP.getChipModel()));
  Serial.println("芯片版本: " + String(ESP.getChipRevision()));
  Serial.println("CPU频率: " + String(ESP.getCpuFreqMHz()) + " MHz");
  Serial.println("Flash大小: " + String(ESP.getFlashChipSize() / (1024 * 1024)) + " MB");
  Serial.println("PSRAM大小: " + String(ESP.getPsramSize() / (1024 * 1024)) + " MB");
  Serial.println("可用内存: " + String(ESP.getFreeHeap() / 1024) + " KB");
  Serial.println("LED引脚: " + String(LED_PIN));
  Serial.println("I2S引脚: SCK=" + String(I2S_SCK_PIN) + ", WS=" + String(I2S_WS_PIN) + ", SD=" + String(I2S_SD_PIN));
  Serial.println("音频初始化: " + String(audioInitialized ? "是" : "否"));
  Serial.println("音频采集: " + String(audioEnabled ? "是" : "否"));
  Serial.println("目标厂商ID: 0x" + String(COMPANY_ID, HEX));
  Serial.println("协议版本: 0x" + String(PROTOCOL_VERSION, HEX));
  Serial.println("==============================\n");
  Serial.flush();
}

void printScanStatus() {
  static unsigned long lastPrint = 0;
  unsigned long currentTime = millis();
  
  // 每10秒打印一次扫描状态
  if (currentTime - lastPrint >= 10000) {
    lastPrint = currentTime;
    String audioStatus = audioEnabled ? "采集中" : "已停止";
    Serial.println("[" + String(currentTime / 1000) + "s] BLE扫描中... LED: " + 
                   String(ledState ? "开启" : "关闭") + ", 音频: " + audioStatus);
    Serial.flush();
  }
}

// ==================== 串口初始化增强 ====================
void initSerial() {
  Serial.begin(115200);
  delay(2000);
  
  unsigned long startTime = millis();
  while (!Serial && (millis() - startTime < 10000)) {
    delay(100);
  }
  
  Serial.flush();
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32-S3 音频+BLE集成系统");
  Serial.println("版本: v2.0");
  Serial.println("功能: BLE接收 + 音频采集 + LED控制");
  Serial.println("波特率: 115200");
  Serial.println("时间戳: " + String(millis()) + "ms");
  Serial.println("========================================");
  Serial.flush();
  delay(500);
}

// ==================== 主程序 ====================
void setup() {
  // 增强的串口初始化
  initSerial();
  
  Serial.println("\n\n=== ESP32-S3 音频+BLE集成系统启动 ===");
  Serial.flush();
  
  // 打印系统信息
  printSystemInfo();
  
  // 初始化LED
  initLED();
  
  // 初始化BLE
  initBLE();
  
  Serial.println("系统初始化完成！");
  Serial.println("\n功能说明:");
  Serial.println("1. BLE扫描: 自动接收OPPO手表广播并控制LED");
  Serial.println("2. 音频采集: 输入 'audio on' 开启INMP441麦克风采集");
  Serial.println("3. 串口控制: 输入 'help' 查看所有可用命令");
  Serial.println("\n开始运行...\n");
  Serial.flush();
}

void loop() {
  // 处理串口命令
  processSerialCommand();
  
  // 处理音频数据
  processAudioData();
  
  // 循环状态监控
  static unsigned long lastLoopPrint = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastLoopPrint >= 60000) {
    lastLoopPrint = currentTime;
    Serial.println("\n[DEBUG] 系统运行正常 - 时间: " + String(currentTime / 1000) + "s");
    Serial.println("[DEBUG] 可用内存: " + String(ESP.getFreeHeap() / 1024) + " KB");
    if (audioEnabled) {
      Serial.println("[DEBUG] 音频采样数: " + String(audioSampleCount));
    }
    Serial.flush();
  }
  
  // BLE扫描
  BLEScanResults* foundDevices = pBLEScan->start(SCAN_TIME, false);
  
  // 打印扫描状态
  printScanStatus();
  
  // 清理扫描结果
  pBLEScan->clearResults();
  
  // 短暂延迟
  delay(100);
}

// ==================== 错误处理和调试 ====================
/*
 * 集成版本调试说明:
 * 
 * 1. 硬件连接验证:
 *    - 确保INMP441麦克风按照引脚图正确连接
 *    - 检查LED引脚配置是否正确
 *    - 验证3.3V和GND连接稳定
 * 
 * 2. 功能测试步骤:
 *    a) 上传代码后打开串口监视器 (115200波特率)
 *    b) 输入 'status' 查看系统状态
 *    c) 输入 'audio on' 开启音频采集
 *    d) 对着麦克风说话，观察音频统计信息
 *    e) 在OPPO手表上触发BLE广播，观察LED控制
 * 
 * 3. 串口命令测试:
 *    - 'help' - 查看所有可用命令
 *    - 'audio on/off' - 控制音频采集
 *    - 'audio status' - 查看音频状态
 *    - 'led on/off/toggle' - 控制LED
 * 
 * 4. 性能优化:
 *    - 音频和BLE功能独立运行，互不干扰
 *    - 使用非阻塞I2S读取，保证BLE扫描正常
 *    - 内存使用优化，适合长时间运行
 * 
 * 5. 常见问题:
 *    - 如果音频采集失败，检查I2S引脚连接
 *    - 如果BLE扫描异常，检查天线连接
 *    - 如果串口输出异常，检查波特率设置
 * 
 * 6. 扩展功能:
 *    - 可添加音频数据WiFi传输
 *    - 可集成语音识别算法
 *    - 可添加音频文件存储功能
 */