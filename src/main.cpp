#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <time.h>
#include <esp_task_wdt.h>
#include "chinese_font.h"

// ==================== 显示屏 (GC9A01 240x240 SPI) ====================
// CS=G5, SCK=G13, DC=G15, MOSI=G32 (详见User_Setup.h)
TFT_eSPI tft = TFT_eSPI();
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
// I2C 保留给传感器（ADXL345、MAX30102、触摸芯片）
#define I2C_SDA 21
#define I2C_SCL 22

// ==================== WonderEcho AI 语音模块 (Wire1 I2C) ====================
// SDA->G25, SCL->G26, VCC->3.3V, GND->GND
#define ASR_SDA 25
#define ASR_SCL 26
#define ASR_I2C_ADDR 0x34
#define ASR_RESULT_REG 100
#define ASR_SPEAK_REG 110
#define ASR_TYPE_COMMAND 0x00
#define ASR_TYPE_ANNOUNCE 0xFF

// ==================== 外设引脚 ====================
#define BUZZER_PIN 17
#define VIBRATE_PIN 27
#define LIGHT_PIN 4      // G4 (LED灯)
#define CANCEL_BUTTON 33
#define TOUCH_PIN 16     // G16 (触摸传感器DO引脚)

// ==================== SFD01 毫米波摔倒检测雷达 (UART) ====================
// SFD01 RXD->G19(ESP32 TX), TXD->G25(ESP32 RX), VCC->3.3V, GND->GND
#define SFD_RX 33    // G33 (原CANCEL_BUTTON已禁用，改用于SFD RX)
#define SFD_TX 19
#define SFD_BAUD 115200

// ==================== GPS模块 (HardwareSerial) ====================
// GPS模块: VCC->3.3V, GND->GND, TX->G18, RX->G2
#define GPS_RX 2
#define GPS_TX 18
#define GPS_BAUD 9600
#define FW_VERSION "2.5.0"           // 固件版本，变更后开机播报"烧录成功"
HardwareSerial gpsSerial(2); // UART2

// ==================== I2C超时保护 ====================
#define I2C_TIMEOUT_MS 100  // I2C操作超时时间（毫秒）
static unsigned long i2cStartTime = 0;
static bool i2cTimeoutEnabled = true;

// I2C超时检查函数
static bool i2cCheckTimeout() {
    if (!i2cTimeoutEnabled) return true;
    return (millis() - i2cStartTime < I2C_TIMEOUT_MS);
}

// 带超时的I2C请求
static int i2cRequestWithTimeout(TwoWire &wire, uint8_t addr, uint8_t len) {
    i2cStartTime = millis();
    int result = wire.requestFrom(addr, len);
    if (!i2cCheckTimeout() && result == 0) {
        Serial.println("[I2C] Timeout!");
        return 0;
    }
    return result;
}

// ==================== ADXL345 三轴加速度计 (硬件 I2C) ====================
// SDA->G21, SCL->G22, SDO->GND(地址0x53)
#define ADXL_ADDR_0 0x53
#define ADXL_ADDR_1 0x1D

// ==================== 4G通话模块触发 ====================
// 4G模块使用短接触发：INT1=G12, INT2=G14 同时拉低触发
// 需要同时拉低两个引脚才能触发拨打号码
#define TRIGGER_OUT1_PIN 12  // INT1
#define TRIGGER_OUT2_PIN 14  // INT2

// ==================== MAX30102 血氧模块 (硬件 I2C) ====================
// SDA->G21, SCL->G22, INT->G34
#define MAX30102_INT 34 // G34
#define MAX30102_ADDR 0x57

// ==================== IT7259 电容触摸屏 (I2C) ====================
// 与ADXL345/MAX30102共用I2C总线 (G21=SDA, G22=SCL)
#define CTP_ADDR 0x38           // IT7259 7-bit I2C地址
#define CTP_POINT_BUFFER 0xE0   // 触摸点数据寄存器
#define CTP_POINT_FLAG 0x08     // 触摸点标志位
#define CTP_GESTURE_FLAG 0x80   // 手势标志位
#define CTP_GESTURE_TAP 0x20    // 单击手势
#define CTP_GESTURE_DOUBLE_TAP 0x23  // 双击手势
#define CTP_GESTURE_FLICK 0x22  // 滑动手势
// 触摸状态
bool ctpAvailable = false;      // 触摸屏是否可用
uint16_t ctpX = 0, ctpY = 0;   // 触摸坐标
bool ctpPressed = false;       // 当前是否触摸中
bool ctpGesture = false;       // 是否检测到手势
uint8_t ctpGestureType = 0;    // 手势类型
unsigned long ctpLastTapTime = 0;  // 上次点击时间
int ctpTapCount = 0;           // 连击计数

// ==================== 语音命令分组表 ====================
static const uint8_t VOICE_ALARM_TRIGGER[] = {0x81, 0x82, 0x83, 0x8B, 0x8C, 0x8D, 0x8E, 0x8F};
static const uint8_t VOICE_ALARM_TRIGGER_COUNT = 8;
static const uint8_t VOICE_ALARM_CANCEL[] = {0x09, 0x84, 0x86, 0x85, 0x88};
static const uint8_t VOICE_ALARM_CANCEL_COUNT = 5;
static const uint8_t VOICE_LIGHT_ON[] = {0x12, 0x80};
static const uint8_t VOICE_LIGHT_ON_COUNT = 2;
static const uint8_t VOICE_LIGHT_OFF[] = {0x13};
static const uint8_t VOICE_LIGHT_OFF_COUNT = 1;
static const uint8_t VOICE_BUZZER_ON[] = {0x87};
static const uint8_t VOICE_BUZZER_ON_COUNT = 1;
static const uint8_t VOICE_BUZZER_OFF[] = {0x89};
static const uint8_t VOICE_BUZZER_OFF_COUNT = 1;

// ==================== 常量 ====================
#define VOICE_DEBOUNCE_MS 500
#define ALARM_COUNTDOWN 10
#define LONG_PRESS_TIME 2000
#define BUZZER_BEEP_DURATION 300   // 蜂鸣器响0.3秒
#define VIBRATE_BEEP_DURATION 300  // 震动0.3秒(与蜂鸣器同步)
#define BUZZER_OFF_DURATION 700    // 蜂鸣器停0.7秒(周期=1秒)
#define SOS_BEEP_DURATION 250      // SOS: 蜂鸣器响0.25秒
#define SOS_OFF_DURATION 250       // SOS: 蜂鸣器停0.25秒 (周期=0.5秒, 1秒响2次)
#define SOS_MAX_DURATION 30000     // SOS最多持续30秒，之后自动停止防止过载
#define LED_PWM_CHANNEL 0          // LED PWM通道
#define LED_PWM_FREQ 1000          // PWM频率 1kHz
#define LED_PWM_RES 8              // 8位分辨率 (0-255)
#define LED_BRIGHTNESS 25          // LED亮度(0-255)，降低功耗防止电压跌落
#define MIN_DISPLAY_INTERVAL 300   // 最小显示刷新间隔(ms)，防止闪烁
#define BOOT_STABILIZE_TIME 5000
#define CANCEL_COOLDOWN_TIME 3000
#define DOUBLE_CLICK_WINDOW 1500 // 双击间隔 1.5秒（原800ms太短）
#define FALL_COOLDOWN_MS 10000
#define ADXL_WINDOW_SIZE 20
#define AP_SSID "FallAlarm-Config"
#define AP_PASSWORD ""

// ==================== 全局对象 ====================
// tft 已在文件头部声明 (TFT_eSPI)
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;
Preferences preferences;
HardwareSerial sfdSerial(1);

// ==================== 屏幕颜色定义 ====================
#define BG_COLOR TFT_BLACK          // 背景色：黑色
#define TEXT_COLOR TFT_WHITE        // 主文字：白色
#define ACCENT_COLOR 0x07E0        // 强调色：绿色 (0x07E0)
#define ALERT_COLOR TFT_RED        // 警告色：红色
#define OK_COLOR 0x07E0           // 成功色：绿色

// ==================== 前向声明 ====================
static void drawCnChar(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t color);
static int cnCharIndex(char c1, char c2, char c3);
static void drawMixedString(const char *str, int y, uint16_t color, uint8_t asciiFont);
static void drawMixedStringAt(const char *str, int x, int y, uint16_t color, uint8_t asciiFont);
static void drawMixedStringBg(const char *str, int y, uint16_t color, uint16_t bgColor, uint8_t asciiFont);
static void drawDivider(int y);

// ==================== 系统状态 ====================
bool alarmActive = false;
int countdown = ALARM_COUNTDOWN;
int batteryLevel = 100;
unsigned long lastAlarmTime = 0;
unsigned long lastSecondTime = 0;
unsigned long lastDisplayTime = 0;
bool systemError = false;
int currentSecond = 0, currentMinute = 0, currentHour = 0;
bool systemReady = false;
bool buzzerActive = false;
unsigned long buzzerStartTime = 0;
bool manualBuzzerOn = false;
bool cancelCooldown = false;
unsigned long cancelCooldownStartTime = 0;
bool melodyActive = false;
int melodyStep = 0;
unsigned long melodyTimer = 0;
unsigned long voiceCmdDisplayTime = 0;
unsigned long bootTime = 0;
// 倒计时蜂鸣器相位控制 (1秒ON, 0.4秒OFF)
bool countdownBuzzerOn = true;
unsigned long countdownPhaseStart = 0;
unsigned long vibrateStartTime = 0;  // 震动开始时间(用于控制震动时长)
unsigned long alarmTriggerTime = 0;  // 报警触发时间(用于触摸防抖，防止双击报警后残余触摸误取消)

// GPS
bool gpsFixed = false;                  // GPS是否定位成功
float gpsLat = 0.0, gpsLon = 0.0;       // GPS经纬度
String gpsTime = "";                    // GPS时间
int gpsSatellites = 0;                  // GPS卫星数量
float gpsAltitude = 0.0;                // GPS海拔高度
float gpsSpeed = 0.0;                   // GPS速度(km/h)
String gpsLastNMEA = "";                // 最后接收的NMEA语句（调试用）
unsigned long gpsLastUpdate = 0;        // GPS最后更新时间

// WiFi
bool wifiConfigured = false;
bool inConfigMode = false;
unsigned long configStartTime = 0;  // 配置模式开始时间，用于20秒超时判断
bool configResetDisplay = false;  // 重置配置页面绘制状态
String savedSSID = "";
unsigned long wifiCheckTime = 0;
bool timeSyncAttempted = false;
bool reconnecting = false;
unsigned long reconnectStartTime = 0;
bool showingCancelFeedback = false;
unsigned long cancelFeedbackStartTime = 0;
bool startupPending = false;
unsigned long startupStartTime = 0;

// 4G通话触发（GPIO触发拨号模块）
bool callTriggered = false;
bool callCompleted = false;  // 标记电话触发完成（3秒脉冲结束）
unsigned long callCompletedTime = 0;  // 记录电话完成时间，用于延迟启动SOS旋律
unsigned long callTriggerTime = 0;
const unsigned long CALL_TRIGGER_DURATION = 3000; // 拉低3秒触发（尝试更长触发时间）
bool callTriggerLocked = false;  // 4G触发锁定标志，防止重启时重复触发
unsigned long callTriggerLockTime = 0;  // 锁定时间
bool callTriggerAttempted = false;  // 每次报警周期只尝试触发一次4G
int callTriggerCount = 0;  // 调用计数器，防止同一周期内多次触发
unsigned long lastCallTriggerDoneTime = 0;  // 上次触发完成时间，用于最小间隔保护

// MAX30102 血氧数据
int spo2Value = 0;
int hrValue = 0;
bool spo2Ready = false;
unsigned long lastValidSpo2Time = 0;  // 初始化为0，15秒无数据才清零
bool max30102Present = false;
unsigned long max30102LastUpdate = 0;

// 摔倒检测
bool sfdFallDetected = false;
unsigned long sfdFallTime = 0;
int sfdFallConfidence = 0;
bool adxlFreeFall = false;
bool adxlImpact = false;
float adxlAccelMag = 0;
float adxlAccelMagPeak = 0;
unsigned long adxlImpactTime = 0;
unsigned long lastFallTrigger = 0;
float adxlMagWindow[ADXL_WINDOW_SIZE];
int adxlMagIdx = 0;
bool adxlWindowFull = false;

// SFD01 UART
#define SFD_FRAME_BUF_SIZE 64
static uint8_t sfdFrameBuf[SFD_FRAME_BUF_SIZE];
static uint8_t sfdFrameIdx = 0;
static unsigned long sfdLastByteTime = 0;
static bool sfdHasPerson = false;
static bool sfdHasFall = false;
static int sfdBreathRate = 0;

// 显示
enum DisplayState
{
    NORMAL,
    COUNTDOWN,
    HELP,
    CANCELLED,
    ERROR,
    CONFIG
};
DisplayState currentDisplayState = NORMAL;
bool displayNeedsUpdate = true;

// ==================== 函数声明 ====================
void initDisplay();
void updateDisplay();
void triggerAlarm();
void updateAlarm();
void cancelAlarm();
void updateTime();
void handleTouch();
void ctpHandleTouch();
void startAPMode();
void handleRoot();
void handleSave();
void handleScan();
void buildScanJson(int n);
void checkScanResults();
void checkServerHealth();
void loadWiFiConfig();
void syncTime();
void checkWiFiConnection();
void triggerPhoneCall();
void checkCallTriggerTimeout();
void voiceSpeak(uint8_t type, uint8_t id);
void handleVoiceCommand();
void setLight(bool on);
void setVibrate(bool on);
static bool cmdInGroup(uint8_t cmd, const uint8_t group[], uint8_t count);
void processFallDetection();
void parseGPS();
void sfdParseFrame();
bool adxlInit();
bool adxlReadAccel(float &ax, float &ay, float &az);
void adxlProcessData();
void sfdProcessData();
bool max30102Init();
void max30102Read();

// ==================== 灯光控制 ====================
void setLight(bool on)
{
    if (on) {
        ledcWrite(LED_PWM_CHANNEL, LED_BRIGHTNESS);  // PWM限流，降低功耗
    } else {
        ledcWrite(LED_PWM_CHANNEL, 0);
    }
}

// ==================== 蜂鸣器控制 ====================
void setBuzzer(bool on)
{
    digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

// ==================== 振动马达控制 ====================
void setVibrate(bool on) { digitalWrite(VIBRATE_PIN, on ? HIGH : LOW); }

// ==================== ADXL345 硬件 I2C (Wire, SDA=G21, SCL=G22) ====================
static uint8_t adxlCurrentAddr = 0;

bool adxlWriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(adxlCurrentAddr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool adxlReadRegs(uint8_t reg, uint8_t len, uint8_t *buf)
{
    Wire.beginTransmission(adxlCurrentAddr);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0)
        return false;
    int read = Wire.requestFrom((uint8_t)adxlCurrentAddr, len);
    if (read != len)
        return false;
    for (int i = 0; i < len; i++)
        buf[i] = Wire.read();
    return true;
}

// 调试用: 扫描I2C总线上的所有设备
static void adxlI2cScan()
{
    Serial.print("[I2C] Scanning G21=SDA, G22=SCL:");
    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x6F; addr++)
    {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf(" 0x%02X", addr);
            found++;
        }
    }
    if (found == 0)
        Serial.println(" none found - check wiring!");
    else
        Serial.printf(" (%d devices)\n", found);
}

static bool adxlTryAddress(uint8_t addr)
{
    // 先唤醒ADXL345 - 写POWER_CTL进入测量模式
    Wire.beginTransmission(addr);
    Wire.write(0x2D); // POWER_CTL寄存器
    Wire.write(0x08); // measure mode
    Wire.endTransmission(true);
    delay(20); // 等待模块启动

    // 读取DEVID
    Wire.beginTransmission(addr);
    Wire.write(0x00); // DEVID寄存器
    if (Wire.endTransmission(true) != 0)
        return false;

    if (Wire.requestFrom(addr, (uint8_t)1) != 1)
        return false;

    uint8_t devid = Wire.read();
    if (devid == 0xE5)
    {
        adxlCurrentAddr = addr;
        Serial.printf("[ADXL] DEVID=0x%02X OK at addr 0x%02X\n", devid, addr);
        return true;
    }
    // 有些ADXL345克隆版返回不同的DEVID，只要能通信就接受
    Serial.printf("[ADXL] addr=0x%02X DEVID=0x%02X (expected 0xE5, accepting clone)\n", addr, devid);
    adxlCurrentAddr = addr;
    return true;
}

bool adxlInit()
{
    // 自动检测地址: 先试0x53(SDO=GND), 再试0x1D(SDO=VCC)
    if (adxlTryAddress(ADXL_ADDR_0))
    {
        Serial.println("[ADXL] Address detected: 0x53 (SDO=GND)");
    }
    else if (adxlTryAddress(ADXL_ADDR_1))
    {
        Serial.println("[ADXL] Address detected: 0x1D (SDO=VCC)");
    }
    else
    {
        // 只在常规地址失败时才扫描，避免每次启动都扫全总线
        adxlI2cScan();
        Serial.println("[ADXL] FAILED - not found at 0x53 or 0x1D, check G21/G22 wiring");
        return false;
    }
    adxlWriteReg(0x31, 0x0B); // +-16g
    adxlWriteReg(0x2C, 0x0A); // 100Hz
    adxlWriteReg(0x2D, 0x08); // measure mode
    adxlWriteReg(0x28, 0x07); // FF threshold 0.5g
    adxlWriteReg(0x29, 0x14); // FF time 20ms
    memset(adxlMagWindow, 0, sizeof(adxlMagWindow));
    adxlMagIdx = 0;
    adxlWindowFull = false;
    Serial.println("[ADXL] Initialized OK (+-16g, 100Hz)");
    return true;
}

bool adxlReadAccel(float &ax, float &ay, float &az)
{
    uint8_t buf[6];
    if (!adxlReadRegs(0x32, 6, buf))
        return false;
    int16_t rawX = (int16_t)(buf[1] << 8) | buf[0];
    int16_t rawY = (int16_t)(buf[3] << 8) | buf[2];
    int16_t rawZ = (int16_t)(buf[5] << 8) | buf[4];
    ax = rawX * 0.0039f;
    ay = rawY * 0.0039f;
    az = rawZ * 0.0039f;
    return true;
}

void adxlProcessData()
{
    float ax, ay, az;
    if (!adxlReadAccel(ax, ay, az))
        return;
    adxlAccelMag = sqrt(ax * ax + ay * ay + az * az);
    adxlMagWindow[adxlMagIdx] = adxlAccelMag;
    adxlMagIdx = (adxlMagIdx + 1) % ADXL_WINDOW_SIZE;
    if (adxlMagIdx == 0)
        adxlWindowFull = true;
    int count = adxlWindowFull ? ADXL_WINDOW_SIZE : adxlMagIdx;
    adxlAccelMagPeak = 0;
    for (int i = 0; i < count; i++)
        if (adxlMagWindow[i] > adxlAccelMagPeak)
            adxlAccelMagPeak = adxlMagWindow[i];
    adxlFreeFall = (adxlAccelMag < 0.3f && adxlAccelMag >= 0);
    static bool wasHigh = false;
    if (adxlAccelMag > 3.0f && !wasHigh)
    {
        adxlImpact = true;
        adxlImpactTime = millis();
        wasHigh = true;
    }
    else if (adxlAccelMag <= 3.0f)
        wasHigh = false;
    if (adxlImpact && millis() - adxlImpactTime > 2000)
        adxlImpact = false;
}

// ==================== MAX30102 血氧模块驱动 (硬件 I2C, 与OLED/ADXL共享) ====================

static bool max30102WriteReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static uint8_t max30102ReadReg(uint8_t reg)
{
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(true) != 0)
        return 0;
    if (Wire.requestFrom(MAX30102_ADDR, 1) != 1)
        return 0;
    return Wire.read();
}

static bool max30102ReadFifo(uint32_t &red, uint32_t &ir)
{
    // 先检查FIFO中是否有可用数据
    uint8_t wrPtr, rdPtr;
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(0x04); // FIFO_WR_PTR
    if (Wire.endTransmission(true) != 0)
    {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 5000) { lastErr = millis(); Serial.println("[MAX30102] I2C err: read WR_PTR"); }
        return false;
    }
    if (Wire.requestFrom(MAX30102_ADDR, 1) != 1)
    {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 5000) { lastErr = millis(); Serial.println("[MAX30102] I2C err: request WR_PTR"); }
        return false;
    }
    wrPtr = Wire.read();

    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(0x06); // FIFO_RD_PTR
    if (Wire.endTransmission(true) != 0)
    {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 5000) { lastErr = millis(); Serial.println("[MAX30102] I2C err: read RD_PTR"); }
        return false;
    }
    if (Wire.requestFrom(MAX30102_ADDR, 1) != 1)
    {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 5000) { lastErr = millis(); Serial.println("[MAX30102] I2C err: request RD_PTR"); }
        return false;
    }
    rdPtr = Wire.read();

    int avail = (int)wrPtr - (int)rdPtr;
    if (avail < 0) avail += 32;
    if (avail <= 0) return false; // FIFO为空

    // 读取FIFO数据寄存器 (MAX30102 datasheet: FIFO_DATA = 0x07)
    Wire.beginTransmission(MAX30102_ADDR);
    Wire.write(0x07);
    if (Wire.endTransmission(true) != 0)
    {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 5000) { lastErr = millis(); Serial.println("[MAX30102] I2C err: set FIFO_DATA addr"); }
        return false;
    }
    if (Wire.requestFrom(MAX30102_ADDR, 6) != 6)
    {
        static unsigned long lastErr = 0;
        if (millis() - lastErr > 5000) { lastErr = millis(); Serial.println("[MAX30102] I2C err: read FIFO_DATA"); }
        return false;
    }
    uint8_t d[6];
    for (uint8_t i = 0; i < 6; i++)
        d[i] = Wire.read();
    red = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
    ir  = ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5];
    red &= 0x3FFFF; // 掩码到18-bit（MAX30102 ADC分辨率）
    ir  &= 0x3FFFF;
    return true;
}

// SpO2 FIFO和简易计算
// 增大缓冲区到200样本(2秒@100Hz)，提高心率分辨率
#define MAX30102_FSIZE 200
static uint32_t maxIrBuf[MAX30102_FSIZE], maxRedBuf[MAX30102_FSIZE];
static int maxBufIdx = 0, maxBufReady = 0;
// 心率滑动平均滤波器 (最近8次有效值)
#define HR_AVG_SIZE 8
static int hrHistory[HR_AVG_SIZE] = {0};
static int hrHistoryIdx = 0;
static int hrHistoryCount = 0;

bool max30102Init()
{
    Serial.print("[MAX30102] Checking...");
    Wire.beginTransmission(MAX30102_ADDR);
    if (Wire.endTransmission() != 0)
    {
        Serial.println(" not found on I2C bus");
        return false;
    }

    uint8_t partId = max30102ReadReg(0xFF);
    uint8_t revId = max30102ReadReg(0xFE);
    Serial.printf(" PART=0x%02X REV=0x%02X\n", partId, revId);

    // 放宽检测：MAX30102正品为0x15，但部分克隆芯片返回不同值
    // 只要能I2C通信，就尝试初始化（PART_ID仅为参考）
    if (partId == 0xFF || partId == 0x00)
    {
        // 完全未响应或总线错误，放弃
        Serial.println("[MAX30102] PART_ID invalid, sensor not responding");
        return false;
    }
    if (partId != 0x15)
    {
        Serial.printf("[MAX30102] PART_ID=0x%02X (expected 0x15), trying clone mode\n", partId);
    }

    // 复位（Mode=0x40，会同时复位FIFO指针）
    max30102WriteReg(0x09, 0x40); // Mode: reset bit
    delay(200);                   // 等待200ms确保复位完成

    // 复位后显式清空FIFO指针
    max30102WriteReg(0x04, 0x00); // FIFO_WR_PTR归0
    max30102WriteReg(0x06, 0x00); // FIFO_RD_PTR归0
    max30102WriteReg(0x05, 0x00); // OVF计数器清0

    // FIFO Config (0x08): almost full=15, FIFO_ROLLOVER=1（满后覆盖旧数据）
    max30102WriteReg(0x08, 0x1F);

    // Mode Config (0x09): SpO2 mode (RED + IR)
    max30102WriteReg(0x09, 0x03); // SpO2 mode

    // SpO2 Config (0x0A): ADC=16384nA, SR=100Hz, LED_PW=411us
    // ADC量程从2048nA提高到16384nA（8倍），防止50mA LED反射光饱和ADC
    max30102WriteReg(0x0A, 0x67);

    // LED电流设置（同时兼容正品MAX30102和克隆MAX30105模块）
    // 正品MAX30102: RED=0x0B, IR=0x0C
    // 克隆(MAX30105): RED=0x0C, IR=0x0D
    // 三个都写入以兼容两类模块
    // LED电流=0x3F=25mA，平衡信号强度和ADC饱和问题
    max30102WriteReg(0x0B, 0x3F); // RED LED (正品MAX30102)
    max30102WriteReg(0x0C, 0x3F); // IR LED (正品MAX30102) / RED LED (克隆MAX30105)
    max30102WriteReg(0x0D, 0x3F); // IR LED (克隆MAX30105，正品MAX30102无此寄存器，写入无害)
    delay(10);

    // 显式配置 Multi-LED Mode Control（确保Slot分配正确）
    // Slot1=RED(0x01), Slot2=IR(0x02), Slot3/4=disabled(0x00)
    max30102WriteReg(0x11, 0x21); // SLOT2=IR(0x02<<4), SLOT1=RED(0x01)
    max30102WriteReg(0x12, 0x00); // SLOT3/4 disabled
    delay(5);

    // 等待传感器填充FIFO
    delay(500);

    maxBufIdx = 0;
    maxBufReady = 0;
    memset(maxIrBuf, 0, sizeof(maxIrBuf));
    memset(maxRedBuf, 0, sizeof(maxRedBuf));

    // 验证配置是否正确写入
    uint8_t modeReg = max30102ReadReg(0x09);
    uint8_t spo2Reg = max30102ReadReg(0x0A);
    uint8_t led1RegA = max30102ReadReg(0x0B); // RED PA (正品MAX30102)
    uint8_t led1RegB = max30102ReadReg(0x0C); // IR PA (正品) / RED PA (克隆)
    uint8_t led2RegB = max30102ReadReg(0x0D); // IR PA (克隆)
    Serial.printf("[MAX30102] Verify: MODE=0x%02X SPO2=0x%02X 0x0B=0x%02X 0x0C=0x%02X 0x0D=0x%02X\n",
                  modeReg, spo2Reg, led1RegA, led1RegB, led2RegB);

    // 调试：读取FIFO指针寄存器
    uint8_t wrPtr = max30102ReadReg(0x04); // FIFO_WR_PTR
    uint8_t rdPtr = max30102ReadReg(0x06); // FIFO_RD_PTR
    uint8_t ovfCnt = max30102ReadReg(0x05); // OVF_COUNTER
    Serial.printf("[MAX30102] FIFO: WR=0x%02X RD=0x%02X OVF=0x%02X\n", wrPtr, rdPtr, ovfCnt);

    // 清空FIFO旧数据
    uint32_t dummyRed = 0, dummyIR = 0;
    uint8_t emptyReads = 0;
    Serial.print("[MAX30102] Draining FIFO...");
    uint8_t readsNeeded = ((wrPtr - rdPtr + 32) % 32) + 32;
    for (int i = 0; i < readsNeeded; i++)
    {
        Wire.beginTransmission(MAX30102_ADDR);
        Wire.write(0x07);
        if (Wire.endTransmission(true) == 0 && Wire.requestFrom(MAX30102_ADDR, 6) == 6)
        {
            uint8_t d[6];
            for (int j = 0; j < 6; j++)
                d[j] = Wire.read();
            emptyReads++;
        }
    }
    // 读取最终的指针状态
    rdPtr = max30102ReadReg(0x06);
    wrPtr = max30102ReadReg(0x04);
    Serial.printf(" %d reads, RD=0x%02X WR=0x%02X\n", emptyReads, rdPtr, wrPtr);

    max30102Present = true;
    Serial.println("[MAX30102] Initialized OK (SpO2 mode, 100Hz)");
    return true;
}

void max30102Read()
{
    if (!max30102Present)
    {
        // 定期检查传感器是否重新上线
        static unsigned long lastCheck = 0;
        unsigned long now = millis();
        if (now - lastCheck > 10000)
        {
            lastCheck = now;
            Wire.beginTransmission(MAX30102_ADDR);
            if (Wire.endTransmission() == 0)
            {
                Serial.println("[MAX30102] Detected again, re-initializing...");
                max30102Init();
            }
        }
        return;
    }
    unsigned long now = millis();
    if (now - max30102LastUpdate < 200)
        return; // 200ms采集周期
    max30102LastUpdate = now;

    // 一次性读取FIFO中所有可用样本
    uint32_t red, ir;
    int samplesRead = 0;
    while (max30102ReadFifo(red, ir) && samplesRead < 32)
    {
        maxIrBuf[maxBufIdx] = ir;
        maxRedBuf[maxBufIdx] = red;
        maxBufIdx = (maxBufIdx + 1) % MAX30102_FSIZE;
        if (maxBufIdx == 0)
            maxBufReady = 1;
        samplesRead++;
    }

    // 调试打印（每10秒）
    static unsigned long lastRawPrint = 0;
    if (now - lastRawPrint > 10000)
    {
        lastRawPrint = now;
        uint8_t w = 0, r = 0;
        Wire.beginTransmission(MAX30102_ADDR); Wire.write(0x04);
        if (Wire.endTransmission(true) == 0 && Wire.requestFrom(MAX30102_ADDR, 1) == 1) w = Wire.read();
        Wire.beginTransmission(MAX30102_ADDR); Wire.write(0x06);
        if (Wire.endTransmission(true) == 0 && Wire.requestFrom(MAX30102_ADDR, 1) == 1) r = Wire.read();
        Serial.printf("[MAX30102] FIFO read %d samples, bufIdx=%d ready=%d WR=0x%02X RD=0x%02X\n",
                      samplesRead, maxBufIdx, maxBufReady, w, r);
        if (samplesRead > 0)
            Serial.printf("[MAX30102] Last RAW: RED=%lu IR=%lu\n", red, ir);
    }

    // 缓冲区未满则等待
    if (!maxBufReady)
        return;

    // 卡死/无手指检测：全0=传感器故障，全饱和(>200000)=无手指，均需重置
    bool allSame = true;
    uint32_t firstVal = maxIrBuf[0];
    for (int i = 1; i < MAX30102_FSIZE; i++)
    {
        if (maxIrBuf[i] != firstVal)
        {
            allSame = false;
            break;
        }
    }
    if (allSame)
    {
        if (firstVal == 0)
        {
            Serial.println("[MAX30102] All samples zero, resetting buffer...");
        }
        else if (firstVal > 200000)
        {
            Serial.println("[MAX30102] No finger detected (saturation), resetting...");
        }
        else
        {
            // 其他不变值也重置
            allSame = false;
        }
        if (allSame)
        {
            maxBufReady = 0;
            maxBufIdx = 0;
            spo2Ready = false;
            hrValue = 0;
            return;
        }
    }

    // === SpO2 / HR 计算 ===
    float irDC = 0, redDC = 0;
    float irAC = 0, redAC = 0;
    uint32_t irMax = 0, redMax = 0, irMin = 0xFFFFFFFF, redMin = 0xFFFFFFFF;

    // 跳过前20个样本（启动不稳定期）
    int startIdx = min(20, MAX30102_FSIZE - 1);
    int sampleCount = MAX30102_FSIZE - startIdx;
    float irSum = 0, redSum = 0;
    for (int i = startIdx; i < MAX30102_FSIZE; i++)
    {
        irSum += maxIrBuf[i];
        redSum += maxRedBuf[i];
        if (maxIrBuf[i] > irMax) irMax = maxIrBuf[i];
        if (maxIrBuf[i] < irMin) irMin = maxIrBuf[i];
        if (maxRedBuf[i] > redMax) redMax = maxRedBuf[i];
        if (maxRedBuf[i] < redMin) redMin = maxRedBuf[i];
    }
    irDC = irSum / sampleCount;
    redDC = redSum / sampleCount;
    irAC = irMax - irMin;
    redAC = redMax - redMin;

    // 通道互换检测（克隆模块常见问题）
    static bool channelSwapped = false;
    if (irDC > 0 && redDC > 0 && (float)irDC / (float)redDC < 0.15f)
    {
        if (!channelSwapped)
        {
            Serial.println("[MAX30102] Channels swapped, auto-correcting...");
            channelSwapped = true;
        }
        float tmp = irDC; irDC = redDC; redDC = tmp;
        tmp = irAC; irAC = redAC; redAC = tmp;
    }

    // 信号质量检查（使用百分比判断AC波动）
    float irACRatio = irDC > 0 ? irAC / irDC : 0;
    float redACRatio = redDC > 0 ? redAC / redDC : 0;

    static unsigned long lastDCPrint = 0;
    if (now - lastDCPrint > 5000)
    {
        lastDCPrint = now;
        Serial.printf("[MAX30102] DC: IR=%.0f RED=%.0f  AC: IR=%.0f(%.4f) RED=%.0f(%.4f)\n",
                      irDC, redDC, irAC, irACRatio, redAC, redACRatio);
    }

    // === 无手指检测 ===
    // 无手指时：环境光直接照射，DC值会很高（>150000）
    // 有手指时：手指吸收大部分光，DC值通常在20000-150000范围
    // 注意：只有真正确定无手指时才重置缓冲区
    if (irDC > 150000 && redDC > 150000)
    {
        // 两个通道都高，说明没有手指在传感器上
        if (lastValidSpo2Time > 0 && now - lastValidSpo2Time > 30000) { spo2Ready = false; hrValue = 0; }
        maxBufReady = 0; maxBufIdx = 0;
        return;
    }

    // DC值检查：有手指时DC通常降至3000-150000
    if (irDC < 2000 || redDC < 2000 || irDC > 200000 || redDC > 200000)
    {
        if (lastValidSpo2Time > 0 && now - lastValidSpo2Time > 30000) { spo2Ready = false; hrValue = 0; }
        maxBufReady = 0; maxBufIdx = 0;
        return;
    }

    // AC信号太弱（无脉搏波动），无法计算SpO2
    // 降低阈值到0.00001，适配手指静止不动的持续监测场景
    if (irACRatio < 0.00001f || redACRatio < 0.00001f)
    {
        if (lastValidSpo2Time > 0 && now - lastValidSpo2Time > 30000) { spo2Ready = false; hrValue = 0; }
        // 重置缓冲区，让新鲜数据进入
        maxBufReady = 0;
        maxBufIdx = 0;
        return;
    }

    // R值计算
    float r = (redACRatio > 0 && irACRatio > 0) ? (redACRatio / irACRatio) : 0;

    // 放宽R值范围，允许更多有效数据
    if (r < 0.2f || r > 4.0f)
    {
        if (lastValidSpo2Time > 0 && now - lastValidSpo2Time > 30000) { spo2Ready = false; hrValue = 0; }
        // 重置缓冲区，让新鲜数据进入
        maxBufReady = 0;
        maxBufIdx = 0;
        return;
    }

    // SpO2经验公式（基于MAX30102典型校准）
    spo2Value = (int)(110.0f - 25.0f * r);
    if (spo2Value > 100) spo2Value = 100;
    if (spo2Value < 70) spo2Value = 70;
    spo2Ready = true;
    lastValidSpo2Time = now;

    // 心率: 通过IR AC信号的峰值检测 + 峰间间隔法
    float ac[MAX30102_FSIZE];
    for (int i = 0; i < MAX30102_FSIZE; i++)
        ac[i] = (float)maxIrBuf[i] - irDC;

    // 5点移动平均滤波，抑制高频噪声
    float filtered[MAX30102_FSIZE];
    for (int i = 0; i < MAX30102_FSIZE; i++) {
        if (i < 2 || i >= MAX30102_FSIZE - 2) {
            filtered[i] = ac[i];
        } else {
            filtered[i] = (ac[i-2] + ac[i-1] + ac[i] + ac[i+1] + ac[i+2]) / 5.0f;
        }
    }

    // 峰值检测：找局部极大值
    float peakThreshold = irAC * 0.15f; // 峰值至少为AC幅度的15%
    if (peakThreshold < 30) peakThreshold = 30; // 最小阈值30

    int peakPositions[20]; // 最多记录20个峰值位置
    int peakCount = 0;
    int lastPeakIdx = -25; // 最小间距25样本(250ms)，对应最高心率240BPM

    for (int i = 3; i < MAX30102_FSIZE - 3; i++) {
        if (filtered[i] > filtered[i-1] && filtered[i] > filtered[i-2] &&
            filtered[i] > filtered[i+1] && filtered[i] > filtered[i+2] &&
            filtered[i] > peakThreshold &&
            (i - lastPeakIdx) >= 25 && peakCount < 20) {
            peakPositions[peakCount++] = i;
            lastPeakIdx = i;
        }
    }

    // 峰间间隔法计算心率：用相邻峰值间距计算瞬时心率，再取平均
    if (peakCount >= 2) {
        float totalInterval = 0;
        int intervalCount = 0;
        for (int i = 1; i < peakCount; i++) {
            float interval = (float)(peakPositions[i] - peakPositions[i-1]) / 100.0f; // 秒
            if (interval > 0.25f && interval < 2.0f) { // 有效间隔: 30-240 BPM
                totalInterval += interval;
                intervalCount++;
            }
        }
        if (intervalCount > 0) {
            float avgInterval = totalInterval / intervalCount;
            int rawHR = (int)(60.0f / avgInterval);
            if (rawHR >= 30 && rawHR <= 220) {
                // 滑动平均滤波
                hrHistory[hrHistoryIdx] = rawHR;
                hrHistoryIdx = (hrHistoryIdx + 1) % HR_AVG_SIZE;
                if (hrHistoryCount < HR_AVG_SIZE) hrHistoryCount++;
                
                int sum = 0;
                for (int i = 0; i < hrHistoryCount; i++) sum += hrHistory[i];
                hrValue = sum / hrHistoryCount;
            } else {
                hrValue = 0;
            }
        } else {
            hrValue = 0;
        }
    } else if (peakCount == 1) {
        // 只有一个峰值，用整个窗口估算
        float duration = (float)sampleCount / 100.0f;
        int rawHR = (int)(60.0f / duration);
        if (rawHR >= 30 && rawHR <= 220) {
            hrHistory[hrHistoryIdx] = rawHR;
            hrHistoryIdx = (hrHistoryIdx + 1) % HR_AVG_SIZE;
            if (hrHistoryCount < HR_AVG_SIZE) hrHistoryCount++;
            
            int sum = 0;
            for (int i = 0; i < hrHistoryCount; i++) sum += hrHistory[i];
            hrValue = sum / hrHistoryCount;
        } else {
            hrValue = 0;
        }
    } else {
        hrValue = 0;
    }

    // 计算完成后使用滑动窗口：保留后50%数据，只需100个新样本即可再次计算
    // 这样更新率约1秒一次，更流畅
    const int KEEP_SAMPLES = MAX30102_FSIZE / 2;  // 保留100个样本
    if (maxBufIdx >= KEEP_SAMPLES) {
        // 移动数据：将后50个样本移到缓冲区开头
        for (int i = 0; i < KEEP_SAMPLES; i++) {
            maxIrBuf[i] = maxIrBuf[maxBufIdx - KEEP_SAMPLES + i];
            maxRedBuf[i] = maxRedBuf[maxBufIdx - KEEP_SAMPLES + i];
        }
        maxBufIdx = KEEP_SAMPLES;  // 下次写入位置
        maxBufReady = 0;  // 需要再填充50个样本
    } else {
        // 缓冲区数据不足，完全重置
        maxBufReady = 0;
        maxBufIdx = 0;
    }

    // 新数据可用，触发显示更新
    displayNeedsUpdate = true;

    static unsigned long lastResultPrint = 0;
    if (now - lastResultPrint > 5000)
    {
        lastResultPrint = now;
        Serial.printf("[MAX30102] SpO2=%d%% HR=%d R=%.3f samples=%d peaks=%d\n",
                      spo2Value, hrValue, r, sampleCount, peakCount);
    }
}

// ==================== GPS模块解析 ====================
void parseGPS()
{
    static String nmea = "";
    static unsigned long lastRawPrint = 0;
    static unsigned long rawByteCount = 0;
    static unsigned long lastNmeaMsg = 0;
    unsigned long now = millis();
    
    // 限制每次解析的数据量，避免阻塞太久
    int maxBytesPerLoop = 100;
    int bytesProcessed = 0;
    
    while (gpsSerial.available() && bytesProcessed < maxBytesPerLoop)
    {
        char c = gpsSerial.read();
        rawByteCount++;
        bytesProcessed++;
        
        // 记录收到数据的时间
        lastNmeaMsg = now;
        
        if (c == '\n')
        {
            nmea.trim();
            gpsLastNMEA = nmea;
            
            // 打印原始NMEA语句（调试用，每10秒最多打印一次）
            if (now - lastRawPrint > 10000 && nmea.length() > 3)
            {
                lastRawPrint = now;
                Serial.printf("[GPS RAW] %s\n", nmea.c_str());
            }
            
            // 解析GPRMC/GNRMC语句获取位置和时间
            if (nmea.startsWith("$GPRMC") || nmea.startsWith("$GNRMC"))
            {
                // 格式: $GPRMC,hhmmss.ss,A,ddmm.mmmm,N,dddmm.mmmm,E,f,ss,ddmmyy,mm*hh
                int comma[15];
                int p = 0;
                for (int i = 0; i < nmea.length() && p < 14; i++)
                {
                    if (nmea.charAt(i) == ',')
                        comma[p++] = i;
                }
                if (p < 10 || nmea.length() <= comma[2] + 1) return;
                
                char status = nmea.charAt(comma[2] + 1);
                if (status == 'A') // A=有效定位
                {
                    gpsFixed = true;
                    gpsLastUpdate = now;
                    
                    // 纬度 ddmm.mmmm -> 度
                    if (p >= 5 && comma[3] + 1 < nmea.length() && comma[4] < nmea.length()) {
                        String latStr = nmea.substring(comma[3] + 1, comma[4]);
                        String latDir = nmea.substring(comma[4] + 1, comma[4] + 2);
                        if (latStr.length() >= 7)
                        {
                            int latDeg = latStr.substring(0, 2).toInt();
                            float latMin = latStr.substring(2).toFloat();
                            gpsLat = latDeg + latMin / 60.0;
                            if (latDir == "S")
                                gpsLat = -gpsLat;
                        }
                    }
                    // 经度 dddmm.mmmm -> 度
                    if (p >= 7 && comma[5] + 1 < nmea.length() && comma[6] < nmea.length()) {
                        String lonStr = nmea.substring(comma[5] + 1, comma[6]);
                        String lonDir = nmea.substring(comma[6] + 1, comma[6] + 2);
                        if (lonStr.length() >= 8)
                        {
                            int lonDeg = lonStr.substring(0, 3).toInt();
                            float lonMin = lonStr.substring(3).toFloat();
                            gpsLon = lonDeg + lonMin / 60.0;
                            if (lonDir == "W")
                                gpsLon = -gpsLon;
                        }
                    }
                    // 速度 (节 -> km/h)
                    if (p >= 9 && comma[7] + 1 < comma[8] && comma[8] < nmea.length())
                    {
                        float speedKnots = nmea.substring(comma[7] + 1, comma[8]).toFloat();
                        gpsSpeed = speedKnots * 1.852; // 1节 = 1.852 km/h
                    }
                    // 时间 - 修复：添加 p >= 2 检查
                    if (p >= 2 && comma[1] + 9 <= nmea.length()) {
                        String timeStr = nmea.substring(comma[1] + 1, comma[1] + 9);
                        if (timeStr.length() >= 6)
                        {
                            gpsTime = timeStr.substring(0, 2) + ":" +
                                      timeStr.substring(2, 4) + ":" +
                                      timeStr.substring(4, 6);
                        }
                    }
                    
                    // 串口输出GPS位置（每5秒一次）
                    static unsigned long lastSerialPrint = 0;
                    if (now - lastSerialPrint >= 5000)
                    {
                        lastSerialPrint = now;
                        Serial.printf("[GPS] Position: LAT=%.6f LON=%.6f ALT=%.1fm SAT=%d SPD=%.1fkm/h TIME=%s\n",
                                      gpsLat, gpsLon, gpsAltitude, gpsSatellites, gpsSpeed, gpsTime.c_str());
                    }
                }
                else
                {
                    if (gpsFixed)
                    {
                        gpsFixed = false;
                        Serial.println("[GPS] Lost fix");
                    }
                }
            }
            // 解析GPGGA/GNGGA语句获取卫星数量和海拔
            else if (nmea.startsWith("$GPGGA") || nmea.startsWith("$GNGGA"))
            {
                // 格式: $GPGGA,hhmmss.ss,ddmm.mmmm,N,dddmm.mmmm,E,q,ss,hdop,alt,M,geoid,M,age,ref*hh
                int comma[15];
                int p = 0;
                for (int i = 0; i < nmea.length() && p < 14; i++)
                {
                    if (nmea.charAt(i) == ',')
                        comma[p++] = i;
                }
                if (p >= 9)
                {
                    // 卫星数量 (字段7)
                    if (p >= 8 && comma[6] + 1 < comma[7] && comma[7] < nmea.length())
                        gpsSatellites = nmea.substring(comma[6] + 1, comma[7]).toInt();
                    // 海拔高度 (字段9) - 修复：添加 p >= 10 检查
                    if (p >= 10 && comma[8] + 1 < comma[9] && comma[9] < nmea.length())
                        gpsAltitude = nmea.substring(comma[8] + 1, comma[9]).toFloat();
                }
            }
            nmea = "";
        }
        else if (c != '\r')
        {
            if (nmea.length() < 200) // 限制缓冲区大小
                nmea += c;
        }
    }
    
    // GPS诊断：如果5秒内未收到任何NMEA数据，输出警告
    if (now - lastNmeaMsg > 5000 && lastNmeaMsg > 0 && now - lastRawPrint > 10000)
    {
        lastRawPrint = now;
        Serial.printf("[GPS] No NMEA data for %lu seconds! Total raw bytes: %lu. Check GPS wiring (RX=%d, TX=%d)\n",
                      (now - lastNmeaMsg) / 1000, rawByteCount, GPS_RX, GPS_TX);
    }
    
    // GPS超时检测：30秒无更新则标记为未定位
    if (gpsFixed && (now - gpsLastUpdate > 30000))
    {
        gpsFixed = false;
        Serial.println("[GPS] Timeout - lost fix");
    }
}

// ==================== SFD01 毫米波雷达驱动 ====================
void sfdInit()
{
    sfdSerial.begin(SFD_BAUD, SERIAL_8N1, SFD_RX, SFD_TX);
    sfdFrameIdx = 0;
    sfdLastByteTime = 0;
    Serial.println("[SFD] UART initialized (115200, RX=G33, TX=G19)");
}

static int sfdFindPattern(const uint8_t *buf, int len, const uint8_t *pat, int plen)
{
    // 防止下溢：如果模式长度大于缓冲区，直接返回-1
    if (plen > len || plen <= 0 || len <= 0) return -1;
    
    for (int i = 0; i <= len - plen; i++)
    {
        bool match = true;
        for (int j = 0; j < plen; j++)
            if (buf[i + j] != pat[j])
            {
                match = false;
                break;
            }
        if (match)
            return i;
    }
    return -1;
}

void sfdProcessData()
{
    unsigned long now = millis();
    while (sfdSerial.available())
    {
        uint8_t c = sfdSerial.read();
        sfdLastByteTime = now;
        if (sfdFrameIdx < SFD_FRAME_BUF_SIZE)
            sfdFrameBuf[sfdFrameIdx++] = c;
    }
    if (sfdFrameIdx > 0 && (now - sfdLastByteTime > 50))
    {
        sfdParseFrame();
        sfdFrameIdx = 0;
    }
    if (sfdFrameIdx >= SFD_FRAME_BUF_SIZE - 1)
    {
        sfdParseFrame();
        sfdFrameIdx = 0;
    }
}

void sfdParseFrame()
{
    if (sfdFrameIdx < 4)
        return;

    // 格式1: 0xAA 0xAA ...
    int pos = sfdFindPattern(sfdFrameBuf, sfdFrameIdx, (const uint8_t *)"\xAA\xAA", 2);
    if (pos >= 0 && pos + 5 < SFD_FRAME_BUF_SIZE)  // 确保 pos+4 不越界
    {
        uint8_t st = sfdFrameBuf[pos + 4];
        sfdHasFall = (st & 0x01);
        sfdHasPerson = (st & 0x02);
        if (sfdHasFall)
            Serial.println("[SFD] FALL detected");
    }

    // 格式2: 0x53 0x59 ...
    pos = sfdFindPattern(sfdFrameBuf, sfdFrameIdx, (const uint8_t *)"\x53\x59", 2);
    if (pos >= 0 && pos + 4 < SFD_FRAME_BUF_SIZE)  // 确保 pos+3 不越界
    {
        uint8_t st = sfdFrameBuf[pos + 3];
        sfdHasFall = (st & 0x01);
        sfdHasPerson = (st & 0x02);
        if (sfdHasFall)
            Serial.println("[SFD] FALL detected");
    }

    // 格式3: ASCII "FALL"
    for (int i = 0; i <= sfdFrameIdx - 4 && i + 3 < SFD_FRAME_BUF_SIZE; i++)
    {
        if (memcmp(&sfdFrameBuf[i], "FALL", 4) == 0 || memcmp(&sfdFrameBuf[i], "fall", 4) == 0)
        {
            sfdHasFall = true;
            sfdHasPerson = true;
            Serial.println("[SFD] FALL (ASCII)");
            break;
        }
    }
}

// ==================== 双模传感器融合摔倒检测 ====================
void processFallDetection()
{
    unsigned long now = millis();
    
    // 开机稳定期检查：60秒内不进行摔倒检测，防止传感器未稳定时误触发
    if (now - bootTime < 60000) {
        return;
    }
    
    if (now - lastFallTrigger < FALL_COOLDOWN_MS)
        return;
    if (alarmActive)
        return;

    // 策略1: SFD01 直接检测 (置信度85%)
    if (sfdHasFall)
    {
        Serial.println("[FALL] SFD01 radar");
        triggerAlarm();
        lastFallTrigger = now;
        sfdHasFall = false;
        return;
    }

    // 策略2: ADXL345 自由落体→撞击序列检测 (置信度75%)
    // 真实摔倒: 先失重(自由落体) → 再撞击(高加速度)
    static bool ffFlag = false;
    static unsigned long ffFlagTime = 0;

    if (adxlFreeFall && !adxlImpact)
    {
        if (!ffFlag)
        {
            ffFlag = true;
            ffFlagTime = now;
        }
    }
    if (ffFlag && adxlImpact && (now - ffFlagTime < 1000))
    {
        Serial.printf("[FALL] ADXL: FF+Impact (peak=%.1fg)\n", adxlAccelMagPeak);
        triggerAlarm();
        lastFallTrigger = now;
        ffFlag = false;
        adxlImpact = false;
        adxlFreeFall = false;
        return;
    }
    if (ffFlag && (now - ffFlagTime > 3000))
    {
        ffFlag = false; // 超时,取消自由落体标记
    }

    // 策略3: ADXL345 高冲击+姿态变化 (置信度60%)
    if (adxlWindowFull && adxlAccelMagPeak > 5.0f)
    {
        bool postStill = true;
        int idx = (adxlMagIdx - 5 + ADXL_WINDOW_SIZE) % ADXL_WINDOW_SIZE;
        for (int i = 0; i < 5; i++)
        {
            float v = adxlMagWindow[(idx + i) % ADXL_WINDOW_SIZE];
            if (v < 0.5f || v > 2.5f)
            {
                postStill = false;
                break;
            }
        }
        if (postStill)
        {
            Serial.printf("[FALL] ADXL: High impact (peak=%.1fg)\n", adxlAccelMagPeak);
            triggerAlarm();
            lastFallTrigger = now;
            adxlImpact = false;
            adxlFreeFall = false;
            return;
        }
    }
}

// 触发拨打号码1（SOS）
// 确保产生下降沿：先拉高再拉低
// 使用状态机避免阻塞delay
static enum CallTriggerState { CT_INIT, CT_HIGH, CT_LOW, CT_DONE } callTriggerStage = CT_INIT;
static unsigned long callTriggerStageTime = 0;

void triggerPhoneCall()
{
    // 双重保护：只在状态机空闲时才能触发
    // 先检查状态机，即使callTriggered还没设置也能防止重复触发
    if (callTriggerStage != CT_INIT) {
        Serial.println("[CALL] State machine busy, skipping");
        return;
    }
    
    // 防止重复触发：电话已完成则不再触发
    if (callCompleted) {
        Serial.println("[CALL] Already completed, skipping");
        return;
    }
    
    // 防止重启时重复触发：锁定期间不触发
    if (callTriggerLocked) {
        Serial.println("[CALL] Locked, skipping");
        return;
    }
    
    // 防止重复触发：callTriggered为true时不触发
    if (callTriggered) {
        Serial.println("[CALL] Already triggered, skipping");
        return;
    }
    
    // 调用计数保护：每个报警周期只允许触发一次
    callTriggerCount++;
    if (callTriggerCount > 1) {
        Serial.printf("[CALL] BLOCKED - already triggered %d times this cycle\n", callTriggerCount);
        return;
    }

    Serial.println("\n========== SOS TRIGGERING ==========");
    callTriggered = true;
    callTriggerTime = millis();
    callTriggerStage = CT_HIGH;  // 立即设置状态，防止重入
    callTriggerStageTime = millis();
    
    // 锁定4G触发，防止重启时重复触发（锁定60秒，确保一次报警周期内只触发一次）
    callTriggerLocked = true;
    callTriggerLockTime = millis();

    // 确保引脚为 HIGH（已在setup中初始化为 OUTPUT HIGH，此处防御性设置）
    digitalWrite(TRIGGER_OUT1_PIN, HIGH);
    digitalWrite(TRIGGER_OUT2_PIN, HIGH);
    Serial.printf("[CALL] Trigger duration: %d ms\n", CALL_TRIGGER_DURATION);
}

// 非阻塞处理拨号触发时序（替代delay(50)）
void updateCallTriggerStateMachine()
{
    if (!callTriggered)
        return;

    unsigned long now = millis();

    switch (callTriggerStage)
    {
    case CT_HIGH:
        if (now - callTriggerStageTime >= 50)
        {
            // 同时拉低INT1和INT2产生短路触发
            digitalWrite(TRIGGER_OUT1_PIN, LOW);
            digitalWrite(TRIGGER_OUT2_PIN, LOW);
            callTriggerStage = CT_LOW;
            callTriggerStageTime = now;
            Serial.println("========== SOS TRIGGER END ==========\n");
        }
        break;
    case CT_LOW:
    case CT_DONE:
        // 由 checkCallTriggerTimeout 负责恢复HIGH
        break;
    case CT_INIT:
        // 异常状态，重置
        callTriggerStage = CT_HIGH;
        callTriggerStageTime = now;
        break;
    }
}

// 检查并结束触发信号
void checkCallTriggerTimeout()
{
    if (callTriggered && millis() - callTriggerTime > CALL_TRIGGER_DURATION)
    {
        // 同时恢复INT1和INT2为HIGH，结束短路触发
        digitalWrite(TRIGGER_OUT1_PIN, HIGH);
        digitalWrite(TRIGGER_OUT2_PIN, HIGH);
        callTriggered = false;
        callCompleted = true;  // 标记电话触发已完成
        callCompletedTime = millis();  // 记录完成时间，用于延迟启动SOS旋律
        lastCallTriggerDoneTime = millis();  // 记录完成时间，用于最小间隔保护
        callTriggerStage = CT_INIT;  // 重置状态机
        Serial.println("[CALL] Trigger signal ended (INT1=INT2=HIGH)");
        // 通知显示更新
        displayNeedsUpdate = true;
    }
    
    // 自动释放4G触发锁定（120秒后），允许下次报警时重新触发
    // 延长到120秒，确保传感器完全稳定
    if (callTriggerLocked && millis() - callTriggerLockTime > 120000)
    {
        callTriggerLocked = false;
        Serial.println("[CALL] Lock auto-released after 120s");
    }
}

// ==================== WonderEcho 语音控制 ====================
void voiceSpeak(uint8_t type, uint8_t id)
{
    Wire1.beginTransmission(ASR_I2C_ADDR);
    Wire1.write(ASR_SPEAK_REG);
    Wire1.write(type);
    Wire1.write(id);
    Wire1.endTransmission();
}

static bool cmdInGroup(uint8_t cmd, const uint8_t group[], uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
        if (group[i] == cmd)
            return true;
    return false;
}

void handleVoiceCommand()
{
    static unsigned long lastPoll = 0, lastOkLog = 0, lastCmdTime = 0;
    static uint8_t lastCmd = 0;
    unsigned long now = millis();
    if (now - lastPoll < 150)
        return;
    lastPoll = now;

    Wire1.beginTransmission(ASR_I2C_ADDR);
    Wire1.write(ASR_RESULT_REG);
    if (Wire1.endTransmission() != 0)
    {
        if (now - lastOkLog > 5000)
        {
            lastOkLog = now;
            Serial.println("[VOICE] I2C no-ack");
        }
        return;
    }
    if (Wire1.requestFrom((uint8_t)ASR_I2C_ADDR, (uint8_t)1) != 1)
        return;
    uint8_t cmd = Wire1.read();
    if (cmd == 0)
    {
        if (now - lastOkLog > 5000)
        {
            lastOkLog = now;
            Serial.println("[VOICE] I2C OK");
        }
        return;
    }

    // 清零防止重复读取 (修复回复两次的BUG)
    Wire1.beginTransmission(ASR_I2C_ADDR);
    Wire1.write(ASR_RESULT_REG);
    Wire1.write((uint8_t)0);
    Wire1.endTransmission();

    if (cmd == lastCmd && (now - lastCmdTime) < VOICE_DEBOUNCE_MS)
        return;
    lastCmd = cmd;
    lastCmdTime = now;
    voiceCmdDisplayTime = now;

    Serial.printf("[VOICE] cmd=0x%02X -> ", cmd);

    if (cmdInGroup(cmd, VOICE_ALARM_TRIGGER, VOICE_ALARM_TRIGGER_COUNT))
    {
        if (!alarmActive)
        {
            triggerAlarm();
            Serial.println("TRIGGER");
        }
        else
        {
            Serial.println("IGNORED (already active)");
        }
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmdInGroup(cmd, VOICE_ALARM_CANCEL, VOICE_ALARM_CANCEL_COUNT))
    {
        if (alarmActive)
        {
            cancelAlarm();
            Serial.println("CANCEL");
        }
        else
        {
            Serial.println("IGNORED (no alarm)");
        }
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmdInGroup(cmd, VOICE_LIGHT_ON, VOICE_LIGHT_ON_COUNT))
    {
        setLight(true);
        Serial.println("LIGHT ON");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmdInGroup(cmd, VOICE_LIGHT_OFF, VOICE_LIGHT_OFF_COUNT))
    {
        setLight(false);
        Serial.println("LIGHT OFF");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmdInGroup(cmd, VOICE_BUZZER_ON, VOICE_BUZZER_ON_COUNT))
    {
        if (!alarmActive)
        {
            manualBuzzerOn = true;
            setBuzzer(true);
        }
        Serial.println(alarmActive ? "BUZZER ON (ignored, alarm active)" : "BUZZER ON");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmdInGroup(cmd, VOICE_BUZZER_OFF, VOICE_BUZZER_OFF_COUNT))
    {
        if (alarmActive)
        {
            cancelAlarm();
        }
        else
        {
            manualBuzzerOn = false;
            setBuzzer(false);
        }
        Serial.println("BUZZER OFF");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmd == 0x1A)
    {
        Serial.println("HELLO");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmd == 0x04)
    {
        Serial.println("VOL+");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmd == 0x05)
    {
        Serial.println("VOL-");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    if (cmd == 0x8A)
    {
        triggerPhoneCall();
        Serial.println("TEST SOS");
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
    Serial.printf("[VOICE] UNKNOWN cmd=0x%02X\n", cmd);
    // 对任何未匹配的高位命令(>=0x80)自动触发报警作为安全兜底
    if (cmd >= 0x80)
    {
        if (!alarmActive)
            triggerAlarm();
        voiceSpeak(ASR_TYPE_COMMAND, cmd);
        return;
    }
}

// ==================== WiFi & WebServer ====================
// HTML页面存储在PROGMEM中，节省RAM防止内存崩溃
const char htmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0,user-scalable=no">
<title>FallAlarm WiFi</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px;color:#c9d1d9}
.card{background:#161b22;border:1px solid #30363d;border-radius:16px;padding:28px 24px;width:100%;max-width:400px}
h1{text-align:center;color:#58a6ff;font-size:22px;margin-bottom:6px;letter-spacing:1px}
.sub{text-align:center;color:#8b949e;font-size:13px;margin-bottom:24px}
.row{margin-bottom:16px}
.row label{display:block;margin-bottom:6px;color:#c9d1d9;font-size:14px;font-weight:500}
.row input,.row select{width:100%;padding:12px 14px;border:1px solid #30363d;border-radius:8px;font-size:16px;background:#0d1117;color:#c9d1d9;outline:none;-webkit-appearance:none}
.row input:focus,.row select:focus{border-color:#58a6ff;box-shadow:0 0 0 3px rgba(88,166,255,0.15)}
.row select{appearance:none;cursor:pointer}
.btn-row{display:flex;gap:12px;margin-top:8px}
.btn{flex:1;padding:14px;border:none;border-radius:8px;font-size:16px;font-weight:600;cursor:pointer;-webkit-tap-highlight-color:transparent}
.btn-scan{background:#21262d;color:#58a6ff;border:1px solid #30363d}
.btn-save{background:#238636;color:#fff}
.btn-save:disabled{opacity:0.5;cursor:not-allowed}
.status{margin-top:18px;padding:14px;border-radius:8px;text-align:center;font-size:14px;display:none}
.status.info{background:rgba(88,166,255,0.12);color:#58a6ff;border:1px solid rgba(88,166,255,0.25);display:block}
.status.ok{background:rgba(46,160,67,0.12);color:#3fb950;border:1px solid rgba(46,160,67,0.25);display:block}
.status.err{background:rgba(248,81,73,0.12);color:#f85149;border:1px solid rgba(248,81,73,0.25);display:block}
.ver{text-align:center;color:#484f58;font-size:11px;margin-top:18px}
</style></head>
<body>
<div class="card">
<h1>FallAlarm Config</h1>
<p class="sub">192.168.4.1</p>
<div class="row"><label>WiFi 名称 (SSID)</label>
<select id="ssidSelect" onchange="selChange()">
<option value="">-- 手动输入或点击扫描 --</option>
</select>
<input type="text" id="ssid" placeholder="输入WiFi名称" style="margin-top:8px"></div>
<div class="row"><label>WiFi 密码</label>
<input type="text" id="pw" placeholder="输入WiFi密码"></div>
<div class="btn-row">
<button class="btn btn-scan" onclick="scanWiFi()" id="scanBtn">扫描WiFi</button>
<button class="btn btn-save" onclick="saveWiFi()" id="saveBtn">保存配置</button>
</div>
<div id="status" class="status"></div>
<div class="ver">Elder Fall Prevention System</div>
</div>
<script>
function setStatus(msg,type){
  var s=document.getElementById('status');
  s.innerHTML=msg;
  s.className='status '+type;
}
function selChange(){
  var v=document.getElementById('ssidSelect').value;
  document.getElementById('ssid').value=v;
}
function scanWiFi(){
  var btn=document.getElementById('scanBtn');
  var sel=document.getElementById('ssidSelect');
  btn.disabled=true;
  btn.textContent='扫描中...';
  setStatus('正在扫描WiFi网络...','info');
  var xhr=new XMLHttpRequest();
  xhr.open('GET','/scan',true);
  xhr.onreadystatechange=function(){
    if(xhr.readyState===4){
      if(xhr.status===200){
        try{
          var data=JSON.parse(xhr.responseText);
          sel.innerHTML='<option value="">-- 选择WiFi ('+data.length+'个) --</option>';
          data.sort(function(a,b){return b.rssi-a.rssi});
          for(var i=0;i<data.length;i++){
            var n=data[i];
            var sig=n.rssi>-50?'[强]':n.rssi>-70?'[中]':'[弱]';
            var enc=n.enc?'':' (开放)';
            var opt=document.createElement('option');
            opt.value=n.ssid;
            opt.textContent=sig+' '+n.ssid+enc;
            sel.appendChild(opt);
          }
          btn.disabled=false;
          btn.textContent='扫描WiFi';
          setStatus('扫描完成, 找到'+data.length+'个网络','info');
          setTimeout(function(){var s=document.getElementById('status');if(s)s.style.display='none'},3000);
        }catch(e){
          btn.disabled=false;
          btn.textContent='扫描WiFi';
          setStatus('扫描失败,请重试','err');
        }
      }else{
        btn.disabled=false;
        btn.textContent='扫描WiFi';
        setStatus('扫描失败,请重试','err');
      }
    }
  };
  xhr.onerror=function(){
    btn.disabled=false;
    btn.textContent='扫描WiFi';
    setStatus('扫描失败,请重试','err');
  };
  xhr.send();
}
function saveWiFi(){
  var ssid=document.getElementById('ssid').value.trim();
  var pw=document.getElementById('pw').value;
  var btn=document.getElementById('saveBtn');
  if(!ssid){setStatus('请输入WiFi名称','err');return}
  btn.disabled=true;
  btn.textContent='保存中...';
  setStatus('正在保存配置...','info');
  var xhr=new XMLHttpRequest();
  xhr.open('GET','/save?ssid='+encodeURIComponent(ssid)+'&password='+encodeURIComponent(pw),true);
  xhr.onreadystatechange=function(){
    if(xhr.readyState===4){
      if(xhr.status===200 && xhr.responseText==='success'){
        setStatus('WiFi已保存! 设备正在连接网络...','ok');
        btn.textContent='已保存';
        btn.disabled=true;
      }else{
        btn.disabled=false;
        btn.textContent='保存配置';
        setStatus('保存失败,请重试','err');
      }
    }
  };
  xhr.onerror=function(){
    btn.disabled=false;
    btn.textContent='保存配置';
    setStatus('网络错误,请重试','err');
  };
  xhr.send();
}
</script></body></html>
)=====";

const char *ntpServer1 = "ntp.aliyun.com", *ntpServer2 = "ntp.tencent.com", *ntpServer3 = "cn.pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;

void startAPMode()
{
    configStartTime = millis();  // 初始化配置模式开始时间
    
    Serial.println("[AP] Starting AP+STA mode...");
    
    // 先停止可能存在的旧服务，防止端口冲突
    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    delay(200);
    
    // 使用AP+STA双模式，允许同时提供AP热点和连接STA网络
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    
    // 启动AP热点 - 增加最大连接数到8，防止连接被拒绝
    bool apOk = WiFi.softAP(AP_SSID, NULL, 1, 0, 8);
    Serial.printf("[AP] softAP result: %s\n", apOk ? "OK" : "FAILED");
    
    if (!apOk) {
        Serial.println("[AP] ERROR: Failed to start AP!");
        return;
    }
    
    delay(200);  // 等待AP启动稳定
    
    // 设置发射功率
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    
    // 打印AP信息
    IPAddress apIP = WiFi.softAPIP();
    Serial.printf("[AP] SSID: %s\n", AP_SSID);
    Serial.printf("[AP] IP: %s\n", apIP.toString().c_str());
    Serial.printf("[AP] Channel: 1\n");
    Serial.printf("[AP] Max connections: 8\n");
    
    // 启动DNS服务器（Captive Portal）
    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.println("[DNS] Started");
    
    // 注册路由
    server.on("/", handleRoot);
    server.on("/save", handleSave);
    server.on("/scan", handleScan);
    
    // Captive Portal 检测URL
    server.on("/generate_204", []() { server.send(204, "text/plain", ""); });
    server.on("/hotspot-detect.html", []() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", ""); });
    server.on("/success.txt", []() { server.send(200, "text/plain", "success"); });
    server.on("/canonical.html", []() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", ""); });
    server.on("/ncsi.txt", []() { server.send(200, "text/plain", "Microsoft NCSI"); });
    server.on("/connecttest.txt", []() { server.send(200, "text/plain", "Redirect to 192.168.4.1"); });
    server.on("/redirect", []() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", ""); });
    server.on("/fwlink", []() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", ""); });
    server.on("/library/test/success.html", []() { server.send(200, "text/html", "<html><body>Success</body></html>"); });
    
    // 所有未匹配的请求返回404，避免无限重定向循环
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not Found");
    });
    
    // 启动WebServer
    server.begin();
    Serial.println("[AP] WebServer started on port 80");
    Serial.println("[AP] Ready for connections!");
    
    displayNeedsUpdate = true;
}

// WebServer健康检查：如果服务器崩溃则自动重启
unsigned long lastServerCheck = 0;
bool serverHealthy = true;
#define SERVER_CHECK_INTERVAL 30000  // 30秒检查一次

void checkServerHealth()
{
    if (millis() - lastServerCheck < SERVER_CHECK_INTERVAL) return;
    lastServerCheck = millis();
    
    // 检查AP是否还在运行
    if (inConfigMode && WiFi.getMode() != WIFI_AP_STA) {
        Serial.println("[HEALTH] AP mode lost, restarting...");
        // 先停止现有服务，避免端口冲突
        server.stop();
        dnsServer.stop();
        delay(100);
        
        WiFi.mode(WIFI_AP_STA);
        delay(100);
        WiFi.softAP(AP_SSID, NULL, 1, 0, 8);  // 最大8个连接（与startAPMode一致）
        delay(200);
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        
        // 重新启动服务
        server.begin();
        dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
        Serial.println("[HEALTH] AP restarted successfully");
    }
    
    // 检查DNS服务器是否运行
    if (inConfigMode) {
        dnsServer.processNextRequest();
    }
}
void handleRoot() {
    // 从PROGMEM读取HTML页面，通过server.send()发送
    // 不能直接用WiFiClient写HTTP头，WebServer框架已经处理了HTTP协议
    size_t len = strlen_P(htmlPage);
    char* buf = (char*)malloc(len + 1);
    if (!buf) {
        server.send(500, "text/plain", "Out of memory");
        return;
    }
    memcpy_P(buf, htmlPage, len);
    buf[len] = '\0';
    server.send(200, "text/html; charset=utf-8", buf);
    free(buf);
}
// WiFi扫描状态机（异步，防止阻塞导致服务器无响应）
volatile bool wifiScanInProgress = false;
volatile int wifiScanResultCount = 0;
unsigned long wifiScanStartTime = 0;
String wifiScanJson = "";  // 缓存扫描结果JSON
#define WIFI_SCAN_TIMEOUT 10000  // 10秒超时

void handleScan()
{
    // 如果扫描正在进行，返回"扫描中"状态
    if (wifiScanInProgress) {
        int result = WiFi.scanComplete();
        if (result >= 0) {
            // 扫描完成，构建JSON
            wifiScanInProgress = false;
            wifiScanResultCount = result;
            buildScanJson(result);
            server.send(200, "application/json", wifiScanJson);
            Serial.printf("[SCAN] Scan completed, found %d networks\n", result);
        } else if (result == WIFI_SCAN_FAILED) {
            wifiScanInProgress = false;
            server.send(200, "application/json", "[]");
            Serial.println("[SCAN] Scan failed");
        } else {
            // 仍在扫描
            server.send(200, "application/json", "{\"scanning\":true}");
        }
        return;
    }
    
    // 启动新的异步扫描
    WiFi.scanDelete();
    int ret = WiFi.scanNetworks(true, true);  // async=true, show_hidden=true
    if (ret == WIFI_SCAN_RUNNING) {
        wifiScanInProgress = true;
        wifiScanStartTime = millis();
        server.send(200, "application/json", "{\"scanning\":true}");
        Serial.println("[SCAN] Async scan started");
    } else if (ret == WIFI_SCAN_FAILED) {
        server.send(200, "application/json", "[]");
        Serial.println("[SCAN] Scan failed immediately");
    } else if (ret >= 0) {
        // 立即有结果（罕见）
        buildScanJson(ret);
        server.send(200, "application/json", wifiScanJson);
        Serial.printf("[SCAN] Found %d networks immediately\n", ret);
    }
}

// 构建扫描结果JSON
void buildScanJson(int n)
{
    wifiScanJson = "[";
    for (int i = 0; i < n; i++) {
        if (i > 0) wifiScanJson += ",";
        String ssid = WiFi.SSID(i);
        ssid.replace("\"", "\\\"");
        ssid.replace("\\", "\\\\");
        wifiScanJson += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + 
                ",\"enc\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN ? "true" : "false") + "}";
    }
    wifiScanJson += "]";
    WiFi.scanDelete();
}

// 检查异步扫描结果（在主循环中调用）
void checkScanResults()
{
    if (!wifiScanInProgress) return;
    
    int result = WiFi.scanComplete();
    if (result >= 0) {
        // 扫描完成
        wifiScanInProgress = false;
        wifiScanResultCount = result;
        buildScanJson(result);
        Serial.printf("[SCAN] Async scan done, found %d networks\n", result);
    } else if (result == WIFI_SCAN_FAILED) {
        wifiScanInProgress = false;
        wifiScanResultCount = 0;
        Serial.println("[SCAN] Async scan failed");
    }
    // 超时保护
    else if (millis() - wifiScanStartTime > WIFI_SCAN_TIMEOUT) {
        WiFi.scanDelete();
        wifiScanInProgress = false;
        wifiScanResultCount = 0;
        Serial.println("[SCAN] Async scan timeout");
    }
}
void handleSave()
{
    String ssid = server.arg("ssid"), pw = server.arg("password");
    if (ssid.length() > 0)
    {
        preferences.putString("ssid", ssid);
        preferences.putString("password", pw);
        // 标记WiFi已配置
        wifiConfigured = true;
        savedSSID = ssid;
        
        // 立即回复客户端，然后在后台连接WiFi
        server.send(200, "text/plain", "success");
        
        // 延迟100ms确保响应发送完成后再开始连接
        delay(100);
        
        // 后台连接WiFi（不阻塞HTTP响应）
        WiFi.disconnect(false);  // 不断开射频，只断开连接
        WiFi.mode(WIFI_AP_STA);  // 保持AP+STA模式
        WiFi.begin(ssid.c_str(), pw.c_str());
        Serial.printf("[WIFI] Saved config: %s, connecting in background\n", ssid.c_str());
        
        // 通知配置页面重绘
        configResetDisplay = true;
        displayNeedsUpdate = true;
    }
    else
        server.send(400, "text/plain", "error");
}

void loadWiFiConfig()
{
    String s = preferences.getString("ssid", "");
    savedSSID = s;
    if (s.length() > 0)
        wifiConfigured = true;
    // 加载保存的时间（用于离线模式）
    currentHour = preferences.getInt("hour", -1);
    currentMinute = preferences.getInt("minute", -1);
    if (currentHour < 0) {
        // 首次启动或无保存时间，使用默认值
        currentHour = 12;
        currentMinute = 0;
    }
}
void syncTime()
{
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
    Serial.print("Syncing time...");
    
    // 非阻塞时间同步：只尝试1次，失败则使用默认时间
    struct tm ti;
    if (getLocalTime(&ti)) {
        currentHour = ti.tm_hour;
        currentMinute = ti.tm_min;
        currentSecond = ti.tm_sec;
        Serial.printf(" OK: %02d:%02d:%02d\n", currentHour, currentMinute, currentSecond);
        // 同步成功后保存时间到Preferences（持久化）
        preferences.putInt("hour", currentHour);
        preferences.putInt("minute", currentMinute);
    } else {
        // 同步失败，使用保存的时间或默认值
        Serial.println(" failed, using saved/default time");
        if (currentHour < 0 || currentMinute < 0) {
            currentHour = 12;
            currentMinute = 0;
            currentSecond = 0;
        }
    }
    timeSyncAttempted = true;
}
// WiFi连接失败或无SSID时保持STA模式，不进入离线模式
// 系统始终尝试连接已配置的WiFi

void checkWiFiConnection()
{
    // 配置模式下不干预WiFi，由配置模式自行管理连接
    if (inConfigMode) {
        wifiCheckTime = millis();
        return;
    }
    
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!timeSyncAttempted)
            syncTime();
        wifiCheckTime = millis();
        reconnecting = false;
        return;
    }
    if (reconnecting)
    {
        if (millis() - reconnectStartTime >= 6000) {
            reconnecting = false;
            wifiCheckTime = millis();  // 重置计时器，防止立即重试
        }
        return;
    }
    String s = preferences.getString("ssid", "");
    if (s.length() == 0) {
        wifiCheckTime = millis();
        return;
    }
    
    if (millis() - wifiCheckTime > 60000)
    {
        wifiCheckTime = millis();
        String p = preferences.getString("password", "");
        WiFi.setTxPower(WIFI_POWER_19_5dBm);  // 修复：用正常功率，之前2dBm太低连不上
        WiFi.disconnect(false);
        WiFi.begin(s.c_str(), p.c_str());
        reconnecting = true;
        reconnectStartTime = millis();
    }
}
// connectToWiFi() 已删除 - 使用 checkWiFiConnection() 替代
// 原函数包含 WiFi.mode(WIFI_STA) 会杀死AP热点，导致配网失败

// ==================== 显示 ====================
// GC9A01 240x240 圆形屏，使用 TFT_eSPI 驱动
// 字体大小参考: FONT1=8px, FONT2=16px, FONT4=26px, FONT7=48px, FONT8=65px

void initDisplay()
{
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(OK_COLOR, TFT_BLACK);
    tft.setTextFont(2);
    tft.drawCentreString("Elder Fall", 120, 80, 2);
    tft.drawCentreString("Prevention System", 120, 100, 2);
    tft.setTextColor(0x7BEF, TFT_BLACK);
    tft.drawCentreString("启动中...", 120, 130, 2);
    tft.setTextColor(0x528A, TFT_BLACK);
    tft.drawCentreString("v" FW_VERSION, 120, 160, 2);
    delay(800);
    tft.fillScreen(TFT_BLACK);
    Serial.println("[TFT GC9A01] OK");
}

// 圆形屏安全绘制辅助：居中显示文本，确保在圆形可见区域内
static void safeCentreString(const char *str, int y, int font)
{
    tft.drawCentreString(str, 120, y, font);
}
static void safeCentreString(const String &s, int y, int font)
{
    tft.drawCentreString(s.c_str(), 120, y, font);
}

// 中文字体渲染函数（前向声明）
static void drawCnChar(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t color);
static int cnCharIndex(char c1, char c2, char c3);
static void drawMixedString(const char *str, int y, uint16_t color, uint8_t asciiFont);

// 在指定位置绘制一个中文字符 (16x16像素)
static void drawCnChar(uint16_t x, uint16_t y, const uint8_t *bitmap, uint16_t color)
{
    for (int row = 0; row < CN_FONT_SIZE; row++) {
        for (int col = 0; col < CN_FONT_SIZE; col++) {
            if (pgm_read_byte(&bitmap[row * 2 + col / 8]) & (1 << (7 - (col % 8)))) {
                tft.drawPixel(x + col, y + row, color);
            }
        }
    }
}

// 查找字符在中文字体表中的索引
static int cnCharIndex(char c1, char c2, char c3)
{
    // UTF-8 中文字符编码: 3字节 E0-EF, 80-BF, 80-BF
    for (int i = 0; i < CN_FONT_COUNT; i++) {
        char fc1 = pgm_read_byte(&CN_FONT_CHARS[i * 3]);
        char fc2 = pgm_read_byte(&CN_FONT_CHARS[i * 3 + 1]);
        char fc3 = pgm_read_byte(&CN_FONT_CHARS[i * 3 + 2]);
        if (fc1 == c1 && fc2 == c2 && fc3 == c3) return i;
    }
    return -1;
}

// 绘制混合中英文的居中字符串 (16px中文字符 + ASCII字符)
// asciiFont: TFT_eSPI字体编号 (1=8px, 2=16px, 4=26px, ...)
static void drawMixedString(const char *str, int y, uint16_t color, uint8_t asciiFont)
{
    int totalWidth = 0;
    const char *p = str;
    int asciiCharWidth = (asciiFont == 1) ? 6 : (asciiFont == 2) ? 10 : 8;
    int asciiSpacing = 1;
    int cnSpacing = 1;
    
    // 第一遍：计算总宽度
    while (*p) {
        if ((uint8_t)*p >= 0xE0 && *(p+1) && (uint8_t)*(p+1) >= 0x80) {
            totalWidth += CN_FONT_SIZE + cnSpacing;
            p += 3;
        } else {
            totalWidth += asciiCharWidth + asciiSpacing;
            p++;
        }
    }
    
    // 从中心开始绘制
    int x = 120 - totalWidth / 2;
    p = str;
    tft.setTextColor(color, BG_COLOR);
    
    while (*p) {
        if ((uint8_t)*p >= 0xE0 && *(p+1) && (uint8_t)*(p+1) >= 0x80) {
            // 中文字符 (16x16 位图)
            int idx = cnCharIndex(*p, *(p+1), *(p+2));
            if (idx >= 0) {
                drawCnChar(x, y, (const uint8_t*)pgm_read_ptr(&CN_FONT_BITMAPS[idx]), color);
            }
            x += CN_FONT_SIZE + cnSpacing;
            p += 3;
        } else {
            // ASCII字符
            char buf[2] = {*p, 0};
            tft.setTextFont(asciiFont);
            // 垂直居中：ASCII字符相对于16px中文基线居中
            int asciiY = y + (CN_FONT_SIZE - (asciiFont == 1 ? 8 : (asciiFont == 2 ? 16 : 26))) / 2;
            tft.drawString(buf, x, asciiY, asciiFont);
            x += asciiCharWidth + asciiSpacing;
            p++;
        }
    }
}

// 在指定x位置绘制混合中英文字符串
static void drawMixedStringAt(const char *str, int x, int y, uint16_t color, uint8_t asciiFont)
{
    const char *p = str;
    int asciiCharWidth = (asciiFont == 1) ? 6 : (asciiFont == 2) ? 10 : 8;
    int asciiSpacing = 1;
    int cnSpacing = 1;
    
    tft.setTextColor(color, BG_COLOR);
    
    while (*p) {
        if ((uint8_t)*p >= 0xE0 && *(p+1) && (uint8_t)*(p+1) >= 0x80) {
            int idx = cnCharIndex(*p, *(p+1), *(p+2));
            if (idx >= 0) {
                drawCnChar(x, y, (const uint8_t*)pgm_read_ptr(&CN_FONT_BITMAPS[idx]), color);
            }
            x += CN_FONT_SIZE + cnSpacing;
            p += 3;
        } else {
            char buf[2] = {*p, 0};
            tft.setTextFont(asciiFont);
            int asciiY = y + (CN_FONT_SIZE - (asciiFont == 1 ? 8 : (asciiFont == 2 ? 16 : 26))) / 2;
            tft.drawString(buf, x, asciiY, asciiFont);
            x += asciiCharWidth + asciiSpacing;
            p++;
        }
    }
}

// 绘制混合中英文居中字符串（带指定背景色）
static void drawMixedStringBg(const char *str, int y, uint16_t color, uint16_t bgColor, uint8_t asciiFont)
{
    int totalWidth = 0;
    const char *p = str;
    int asciiCharWidth = (asciiFont == 1) ? 6 : (asciiFont == 2) ? 10 : 8;
    int asciiSpacing = 1;
    int cnSpacing = 1;
    
    while (*p) {
        if ((uint8_t)*p >= 0xE0 && *(p+1) && (uint8_t)*(p+1) >= 0x80) {
            totalWidth += CN_FONT_SIZE + cnSpacing;
            p += 3;
        } else {
            totalWidth += asciiCharWidth + asciiSpacing;
            p++;
        }
    }
    
    int x = 120 - totalWidth / 2;
    p = str;
    tft.setTextColor(color, bgColor);
    
    while (*p) {
        if ((uint8_t)*p >= 0xE0 && *(p+1) && (uint8_t)*(p+1) >= 0x80) {
            int idx = cnCharIndex(*p, *(p+1), *(p+2));
            if (idx >= 0) {
                // 用背景色填充字符区域
                tft.fillRect(x, y, CN_FONT_SIZE, CN_FONT_SIZE, bgColor);
                drawCnChar(x, y, (const uint8_t*)pgm_read_ptr(&CN_FONT_BITMAPS[idx]), color);
            }
            x += CN_FONT_SIZE + cnSpacing;
            p += 3;
        } else {
            char buf[2] = {*p, 0};
            tft.setTextFont(asciiFont);
            int asciiY = y + (CN_FONT_SIZE - (asciiFont == 1 ? 8 : (asciiFont == 2 ? 16 : 26))) / 2;
            tft.drawString(buf, x, asciiY, asciiFont);
            x += asciiCharWidth + asciiSpacing;
            p++;
        }
    }
}

// 辅助函数：绘制分隔线
static void drawDivider(int y)
{
    tft.drawFastHLine(10, y, SCREEN_WIDTH - 20, 0x18E3); // 暗色线
}

void updateDisplay()
{
    unsigned long now = millis();  // 在函数内部定义now，供GPS刷新逻辑使用
    
    // 统一管理的静态状态标记
    static DisplayState lastState = (DisplayState)-1;
    static int lastCountdown = -1;
    static bool countdownFullRedraw = true;
    static bool helpDrawn = false;
    static bool cancelDrawn = false;
    static bool errorDrawn = false;
    static int lastMinute = -1;
    static bool lastGpsFixedState = false;
    static int lastGpsPrintMinute = -1;
    
    // WiFi配置模式 - 开机配网提示页面（圆形屏幕，持续重绘防止干扰）
    // 注意：此段代码在updateDisplay()中，只做状态检查和返回，实际绘制在loop中处理
    if (inConfigMode)
    {
        static unsigned long configDrawTime = 0;
        static bool prevWifiConnected = false;
        bool wifiConnected = (WiFi.status() == WL_CONNECTED);
        
        // 如果 STA 已连接到路由器，延迟后自动退出配置模式进入主页面
        if (wifiConnected) {
            if (configDrawTime == 0) {
                configDrawTime = millis();
            }
            if (millis() - configDrawTime > 1500) {
                inConfigMode = false;
                systemReady = true;
                currentDisplayState = NORMAL;
                displayNeedsUpdate = true;
                configDrawTime = 0;
                prevWifiConnected = false;
                lastState = CONFIG;  // 强制fullRedraw=true，确保NORMAL页面全量重绘
                server.stop();
                dnsServer.stop();
                delay(10);
                WiFi.softAPdisconnect(true);
                delay(50);
                WiFi.mode(WIFI_STA);
                delay(20);
                if (!timeSyncAttempted) {
                    syncTime();
                }
                Serial.println("[WIFI] Connected! Entering main display.");
                return;
            }
            
            // WiFi状态变化时重置
            if (wifiConnected != prevWifiConnected) {
                prevWifiConnected = wifiConnected;
                configDrawTime = millis();
            }
            
            // 绘制连接成功画面
            tft.fillScreen(BG_COLOR);
            
            // 连接成功图标
            tft.setTextColor(OK_COLOR, BG_COLOR);
            tft.setTextFont(6);
            tft.drawCentreString("OK", 120, 40, 6);
            tft.setTextFont(1);
            
            // 状态文字
            drawMixedString("WiFi已连接", 100, OK_COLOR, 2);
            drawMixedString(savedSSID.c_str(), 130, TEXT_COLOR, 1);
            drawMixedString("启动监测", 158, ACCENT_COLOR, 2);
            
            // 处理Web请求和触摸
            server.handleClient();
            dnsServer.processNextRequest();
            handleTouch();
            if (ctpAvailable) {
                ctpHandleTouch();
            }
            return;
        }
        
        // 未连接WiFi，显示配网提示（持续重绘防止干扰）
        // 使用全局 configStartTime，已在 startAPMode() 中初始化
        
        static unsigned long lastPromptDraw = 0;
        if (millis() - lastPromptDraw > 500) {
            lastPromptDraw = millis();
            tft.fillScreen(BG_COLOR);
            
            // 计算剩余时间
            unsigned long elapsed = millis() - configStartTime;
            int remaining = 20 - (elapsed / 1000);
            if (remaining < 0) remaining = 0;
            
            // 20秒超时判断：超时后显示未配置界面，但后台仍尝试连接已保存的WiFi
            bool showUnconfigured = (!wifiConfigured) || (elapsed > 20000);
            
            if (!showUnconfigured) {
                // 20秒内，已保存配置，正在自动连接
                drawMixedString("WiFi配置", 20, ACCENT_COLOR, 2);
                drawDivider(42);
                drawMixedString("连接中", 65, TFT_YELLOW, 2);
                drawMixedString(savedSSID.c_str(), 95, TEXT_COLOR, 1);
                
                // 显示倒计时
                char countdownStr[16];
                snprintf(countdownStr, sizeof(countdownStr), "%ds", remaining);
                tft.setTextColor(TFT_CYAN, BG_COLOR);
                tft.setTextFont(4);
                tft.drawCentreString(countdownStr, 120, 125, 4);
                tft.setTextFont(1);
                
                drawMixedString("重配WiFi", 165, 0x7BEF, 1);
                drawMixedString("192.168.4.1", 185, 0x7BEF, 1);
            } else {
                // 未配置 或 20秒超时，显示配网引导
                drawMixedString("WiFi配置", 20, ACCENT_COLOR, 2);
                drawDivider(42);
                drawMixedString("连接热点", 65, TEXT_COLOR, 1);
                // AP名称用大字体突出显示
                tft.setTextColor(TFT_YELLOW, BG_COLOR);
                tft.setTextFont(2);
                tft.drawCentreString(AP_SSID, 120, 88, 2);
                tft.setTextFont(1);
                drawDivider(118);
                drawMixedString("访问 192.168.4.1", 140, 0x7BEF, 1);
                drawMixedString("请连接WiFi", 165, TFT_YELLOW, 2);
            }
        }
        
        // 处理Web请求和触摸
        server.handleClient();
        dnsServer.processNextRequest();  // 处理Captive Portal DNS请求
        handleTouch();
        if (ctpAvailable) {
            ctpHandleTouch();
        }
        
        // 配置模式下不自动连接WiFi，保持AP热点稳定
        // 自动连接由checkWiFiConnection()在非配置模式下处理
        
        esp_task_wdt_reset();
        delay(10);
        return;
    }

    // 状态切换时重置标记（不在这里更新lastState，让各case自行处理）
    if (currentDisplayState != lastState)
    {
        countdownFullRedraw = true;
        helpDrawn = false;
        cancelDrawn = false;
        errorDrawn = false;
        lastCountdown = -1;
    }

    switch (currentDisplayState)
    {
    case NORMAL:
    {
        bool fullRedraw = (lastState != NORMAL);
        if (fullRedraw) {
            tft.fillScreen(BG_COLOR);
            lastGpsFixedState = gpsFixed;
            lastGpsPrintMinute = -1;
            lastMinute = -1;
            lastCountdown = -1;  // 重置倒计时显示状态
        }
        lastState = NORMAL;

        // 顶部状态栏 - 中文显示，WiFi状态变化时刷新
        {
            static bool lastWifiState = false;
            static String lastSSIDShown = "";
            bool wifiConnected = (WiFi.status() == WL_CONNECTED);
            if (fullRedraw || wifiConnected != lastWifiState || savedSSID != lastSSIDShown) {
                lastWifiState = wifiConnected;
                lastSSIDShown = savedSSID;
                tft.fillRect(0, 0, 240, 22, 0x1082);  // 清除整个状态栏区域
                if (wifiConnected && savedSSID.length() > 0) {
                    tft.setTextColor(OK_COLOR, 0x1082);
                    tft.setTextFont(1);
                    String cnLabel = "已连接:";
                    // 计算中文标签宽度后绘制
                    int cnW = 0;
                    for (int i = 0; i < 3; i++) { // 3个中文字符(UTF-8 3字节)
                        cnW += 16 + 1; // 16px + 1px间距
                    }
                    int x = 120 - (cnW + (savedSSID.length() > 10 ? 10 : savedSSID.length()) * 7) / 2;
                    drawMixedStringAt("已连接:", x, 4, OK_COLOR, 1);
                    x += cnW;
                    tft.setTextColor(OK_COLOR, 0x1082);
                    tft.drawString(savedSSID.substring(0, savedSSID.length() > 10 ? 10 : savedSSID.length()), x, 4, 1);
                } else {
                    drawMixedStringBg("WiFi离线", 4, 0x7BEF, 0x1082, 1);
                }
            }
        }

        // 时间 - 大字体居中
        {
            tft.setTextColor(0xFFE0, BG_COLOR);
            tft.setTextFont(6);
            char t[10];
            sprintf(t, "%02d:%02d", currentHour, currentMinute);
            if (currentMinute != lastMinute || fullRedraw) {
                lastMinute = currentMinute;
                tft.fillRect(0, 28, 240, 48, BG_COLOR);  // 清除整个时间区域
                tft.drawCentreString(t, 120, 30, 6);
            }
        }

        // 日期
        if (fullRedraw) {
            tft.drawFastHLine(40, 78, 160, 0x18E3);
            struct tm ti;
            if (getLocalTime(&ti)) {
                char dateBuf[16];
                sprintf(dateBuf, "%02d/%02d", ti.tm_mon+1, ti.tm_mday);
                tft.setTextColor(0x7BEF, BG_COLOR);
                tft.drawCentreString(dateBuf, 120, 82, 2);
            }
        }

        // 健康数据 - 居中紧凑，中文标签
        {
            tft.fillRect(0, 100, 240, 24, BG_COLOR);  // 清除健康数据区域
            drawMixedStringAt("血氧", 25, 106, TFT_RED, 1);
            char spo2Str[8];
            sprintf(spo2Str, spo2Ready ? "%d%%" : "--", spo2Value);
            tft.setTextColor(TEXT_COLOR, BG_COLOR);
            tft.drawString(spo2Str, 80, 106, 1);
            
            drawMixedStringAt("心率", 135, 106, TFT_RED, 1);
            char hrStr[8];
            sprintf(hrStr, hrValue > 0 ? "%d" : "--", hrValue);
            tft.setTextColor(TEXT_COLOR, BG_COLOR);
            tft.drawString(hrStr, 180, 106, 1);
        }

        // GPS 位置信息 - 居中（状态切换时强制重绘，防止COUNTDOWN残留文字覆盖）
        {
            bool needsRefresh = fullRedraw || (gpsFixed != lastGpsFixedState) || 
                               (gpsFixed && currentMinute != lastGpsPrintMinute);
            if (needsRefresh) {
                lastGpsFixedState = gpsFixed;
                lastGpsPrintMinute = currentMinute;
                tft.fillRect(0, 125, 240, 42, BG_COLOR);  // 清除整个GPS区域（扩大清除范围）
                if (gpsFixed) {
                    tft.drawFastHLine(40, 128, 160, 0x18E3);
                    tft.setTextColor(OK_COLOR, BG_COLOR);
                    char coord[64];
                    sprintf(coord, "%.5f,%.5f", gpsLat, gpsLon);
                    tft.drawCentreString(coord, 120, 134, 1);
                    char info[32];
                    sprintf(info, "S:%d A:%.0fm", gpsSatellites, gpsAltitude);
                    tft.drawCentreString(info, 120, 148, 1);
                    
                    // 串口输出GPS数据
                    Serial.printf("[GPS] LAT=%.6f LON=%.6f ALT=%.1fm SAT=%d\n",
                                  gpsLat, gpsLon, gpsAltitude, gpsSatellites);
                } else {
                    tft.drawFastHLine(40, 128, 160, 0x18E3);
                    tft.setTextColor(0xCE59, BG_COLOR);
                    tft.drawCentreString("GPS定位中...", 120, 138, 1);
                }
            }
        }

        // 底部状态 - 居中
        if (fullRedraw) {
            tft.fillRect(0, 165, 240, 30, BG_COLOR);  // 清除底部区域
            tft.drawFastHLine(40, 168, 160, 0x18E3);
            drawMixedString("系统监测中", 175, TEXT_COLOR, 1);
            drawMixedString("触摸两次求助", 196, 0x7BEF, 1);
        }
        break;
    }
    case COUNTDOWN:
    {
        lastState = COUNTDOWN;
        if (countdownFullRedraw)
        {
            countdownFullRedraw = false;
            lastCountdown = countdown;
            
            tft.fillScreen(BG_COLOR);
            tft.fillRect(0, 0, 240, 22, 0x4000);
            tft.setTextColor(ALERT_COLOR, 0x4000);
            tft.drawCentreString("FALL DETECTED!", 120, 4, 1);
            
            // 倒计时大数字
            tft.setTextColor(ALERT_COLOR, BG_COLOR);
            tft.setTextFont(7);
            char s[8];
            sprintf(s, "%d", countdown);
            tft.fillRect(0, 30, 240, 52, BG_COLOR);
            tft.drawCentreString(s, 120, 32, 7);
            
            // 提示
            tft.fillRect(0, 88, 240, 50, BG_COLOR);
            drawMixedString("秒后拨打求助电话", 100, ACCENT_COLOR, 1);
            drawMixedString("触摸两次可取消", 125, TEXT_COLOR, 1);
            
            // GPS
            if (gpsFixed) {
                tft.drawFastHLine(40, 155, 160, 0x18E3);
                tft.setTextColor(OK_COLOR, BG_COLOR);
                char gpsInfo[64];
                sprintf(gpsInfo, "%.5f,%.5f", gpsLat, gpsLon);
                tft.drawCentreString(gpsInfo, 120, 165, 1);
            }
            
            Serial.printf("[报警] 倒计时: %d秒\n", countdown);
        }
        else if (countdown != lastCountdown)
        {
            // 仅刷新倒计时数字，不重绘整屏（消除闪烁）
            lastCountdown = countdown;
            tft.setTextColor(ALERT_COLOR, BG_COLOR);
            tft.setTextFont(7);
            tft.fillRect(0, 30, 240, 52, BG_COLOR);
            char s[8];
            sprintf(s, "%d", countdown);
            tft.drawCentreString(s, 120, 32, 7);
            Serial.printf("[报警] 倒计时: %d秒\n", countdown);
        }
        break;
    }
    case HELP:
    {
        // prevCallCompleted只在HELP状态开始时初始化为callCompleted的当前值
        static bool prevCallCompleted = false;
        static bool initialized = false;
        static unsigned long helpDisplayStartTime = 0;
        static bool displayedSuccess = false;
        
        if (!initialized || lastState != HELP) {
            prevCallCompleted = callCompleted;
            initialized = true;
            helpDisplayStartTime = millis();
            displayedSuccess = false;
        }
        
        // 当状态刚进入HELP或callCompleted状态改变时，需要重绘HELP屏幕
        // 或者当4G完成后2秒到了需要更新显示时
        unsigned long timeSinceCallCompleted = callCompleted ? (millis() - callCompletedTime) : 0;
        bool needRedraw = (lastState != HELP) || (callCompleted != prevCallCompleted);
        
        // 4G完成后2秒，更新显示为"求救成功等待救援"
        if (callCompleted && timeSinceCallCompleted >= 2000 && !displayedSuccess) {
            needRedraw = true;
            displayedSuccess = true;
        }
        
        if (needRedraw) {
            prevCallCompleted = callCompleted;
            tft.fillScreen(BG_COLOR);
            tft.fillRect(0, 0, 240, 22, ALERT_COLOR);
            tft.setTextColor(TFT_WHITE, ALERT_COLOR);
            tft.drawCentreString("SOS HELP!", 120, 4, 1);
            // SOS大字
            tft.fillRect(0, 30, 240, 56, BG_COLOR);
            tft.setTextColor(ALERT_COLOR, BG_COLOR);
            tft.setTextFont(6);
            tft.drawCentreString("SOS", 120, 32, 6);
            tft.setTextFont(1);
            
            // 状态文本：根据callCompleted状态和延迟显示
            tft.fillRect(0, 90, 240, 60, BG_COLOR);
            
            if (callCompleted && timeSinceCallCompleted >= 2000) {
                // 4G完成后2秒显示"求救成功等待救援"
                drawMixedString("求救成功", 105, ACCENT_COLOR, 2);
                tft.fillRect(0, 125, 240, 30, BG_COLOR);
                drawMixedString("等待救援", 135, ACCENT_COLOR, 2);
            } else if (callCompleted) {
                // 4G刚完成，等待2秒
                drawMixedString("信号发送中...", 105, ACCENT_COLOR, 2);
                tft.fillRect(0, 125, 240, 30, BG_COLOR);
                drawMixedString("请稍候", 135, ACCENT_COLOR, 2);
            } else {
                // 倒计时结束后显示"正在求救SOS"
                drawMixedString("正在求救SOS", 105, ACCENT_COLOR, 2);
                tft.fillRect(0, 125, 240, 30, BG_COLOR);
                drawMixedString("发送求助信号", 135, ACCENT_COLOR, 2);
            }
            
            // GPS
            if (gpsFixed) {
                tft.drawFastHLine(40, 135, 160, 0x18E3);
                tft.setTextColor(OK_COLOR, BG_COLOR);
                char gpsInfo[64];
                sprintf(gpsInfo, "%.5f,%.5f", gpsLat, gpsLon);
                tft.drawCentreString(gpsInfo, 120, 145, 1);
            }
            // 底部分隔线
            tft.drawFastHLine(40, 175, 160, 0x18E3);
            tft.setTextColor(0x7BEF, BG_COLOR);
            tft.drawCentreString("S:卫星 A:海拔", 120, 185, 1);
        }
        lastState = HELP;
        break;
    }
    case CANCELLED:
    {
        lastState = CANCELLED;
        if (!cancelDrawn)
        {
            cancelDrawn = true;
            helpDrawn = false;
            countdownFullRedraw = true;
            callCompleted = false;
            tft.fillScreen(BG_COLOR);
            tft.fillRect(0, 0, 240, 22, 0x1082);
            tft.setTextColor(OK_COLOR, 0x1082);
            tft.drawCentreString("CANCELLED", 120, 4, 1);
            tft.fillRect(0, 50, 240, 60, BG_COLOR);
            drawMixedString("返回监测模式...", 65, OK_COLOR, 2);
            tft.fillRect(0, 120, 240, 40, BG_COLOR);
            drawMixedString("3秒后返回", 130, 0x7BEF, 2);
        }
        break;
    }
    case ERROR:
    {
        lastState = ERROR;
        if (!errorDrawn)
        {
            errorDrawn = true;
            tft.fillScreen(BG_COLOR);
            tft.fillRect(0, 0, 240, 22, ALERT_COLOR);
            tft.setTextColor(TFT_WHITE, ALERT_COLOR);
            tft.drawCentreString("SYSTEM ERROR", 120, 4, 1);
            tft.fillRect(0, 60, 240, 60, BG_COLOR);
            tft.setTextColor(ALERT_COLOR, BG_COLOR);
            tft.setTextFont(6);
            tft.drawCentreString("!", 120, 62, 6);
            tft.setTextFont(1);
            drawMixedString("请重启设备", 140, TEXT_COLOR, 2);
        }
        break;
    }
    }
}

// ==================== IT7259 电容触摸屏驱动 ====================
// 读取IT7259触摸数据 (14字节)
// 返回: true=有触摸数据, false=无触摸
bool ctpReadData()
{
    if (!ctpAvailable) return false;
    
    uint8_t buf[14];
    memset(buf, 0, 14);
    
    i2cTimeoutEnabled = true;
    i2cStartTime = millis();
    
    Wire.beginTransmission(CTP_ADDR);
    Wire.write(CTP_POINT_BUFFER);
    if (Wire.endTransmission(false) != 0) {
        ctpAvailable = false;
        return false;
    }
    
    if (Wire.requestFrom((uint8_t)CTP_ADDR, (uint8_t)14) != 14) {
        return false;
    }
    
    for (int i = 0; i < 14; i++) {
        buf[i] = Wire.read();
    }
    
    // 检查触摸点
    if (buf[0] & CTP_POINT_FLAG) {
        ctpX = (uint16_t)buf[2];  // X坐标
        ctpY = (uint16_t)buf[4];  // Y坐标
        ctpPressed = true;
        return true;
    }
    
    // 检查手势（不设置ctpPressed，避免手势重复触发点击计数）
    if (buf[0] & CTP_GESTURE_FLAG) {
        ctpGesture = true;
        ctpGestureType = buf[1];
        // 手势仅记录，不标记为触摸点，由触摸点检测点击/双击
        // 注释掉以下代码，避免手势导致额外点击：
        // if (buf[1] == CTP_GESTURE_TAP || buf[1] == CTP_GESTURE_DOUBLE_TAP) {
        //     ctpPressed = true;
        //     ctpX = (uint16_t)buf[2];
        //     ctpY = (uint16_t)buf[4];
        // }
        return true;
    }
    
    ctpPressed = false;
    ctpGesture = false;
    return false;
}

// 初始化IT7259触摸屏
bool ctpInit()
{
    Wire.beginTransmission(CTP_ADDR);
    if (Wire.endTransmission() == 0) {
        ctpAvailable = true;
        Serial.printf("[TOUCH] IT7259 found at 0x%02X\n", CTP_ADDR);
        return true;
    }
    Serial.println("[TOUCH] IT7259 not found");
    return false;
}

// 处理电容触摸屏的点击/双击事件
void ctpHandleTouch()
{
    static bool prevPressed = false;
    static unsigned long pressStartTime = 0;
    static bool tapHandled = false;
    unsigned long now = millis();
    
    if (!ctpAvailable) return;
    
    if (ctpPressed) {
        if (!prevPressed) {
            // 新触摸按下
            pressStartTime = now;
            tapHandled = false;
        }
        prevPressed = true;
    } else {
        if (prevPressed && !tapHandled) {
            // 触摸释放 - 判断点击类型
            unsigned long duration = now - pressStartTime;
            
            // 长按检测 (2秒重启) - 在所有状态下都允许
            if (duration >= LONG_PRESS_TIME) {
                tapHandled = true;
                Serial.println("[CTP] Long press (2s) detected - restarting...");
                delay(50);
                ESP.restart();
            }
            // 有效短按：8ms防抖，短于2秒
            else if (duration > 8 && duration < LONG_PRESS_TIME) {
                // 报警触发后0.05秒内忽略触摸，防止残余触摸误取消
                if (alarmActive && alarmTriggerTime > 0 && (now - alarmTriggerTime < 50)) {
                    Serial.println("[CTP] Touch ignored (debounce 0.05s after alarm trigger)");
                    tapHandled = true;
                } else {
                    ctpTapCount++;
                    ctpLastTapTime = now;

                    // 双击检测
                    if (ctpTapCount >= 2) {
                        ctpTapCount = 0;
                        tapHandled = true;
                        
                        if (alarmActive) {
                            // 报警状态下（包括COUNTDOWN和HELP）都允许双击取消
                            cancelAlarm();
                            Serial.println("[CTP] Double-tap: cancel alarm");
                        } else if (!cancelCooldown) {
                            triggerAlarm();
                            Serial.println("[CTP] Double-tap: trigger alarm");
                        }
                    }
                }
            }
        }
        prevPressed = false;
    }
    
    // 双击窗口：0.8秒内无第二次点击，重置计数器
    if (ctpTapCount > 0 && (now - ctpLastTapTime > 800)) {
        ctpTapCount = 0;
    }
}

// ==================== 触摸 & 按钮 ====================
// 支持三种触摸方式（自动检测）：
// 1. I2C触摸控制器：CST816S (0x15) / FT6336 (0x38) - GC9A01圆形屏常见配置
// 2. GPIO电容触摸按键：TTP223模块 (INPUT_PULLUP, LOW=触摸)
// 系统启动时自动检测I2C触摸控制器，找不到则回退到GPIO触摸按键

enum TouchType { TOUCH_NONE, TOUCH_I2C_CST816S, TOUCH_I2C_FT6336, TOUCH_GPIO };
static TouchType touchType = TOUCH_NONE;
static uint16_t touchX = 0, touchY = 0;
static bool touchPressed = false;

// I2C触摸控制器初始化与检测
void initTouchController()
{
    // 如果 ctpInit() 已在 setup() 中检测到 IT7259，直接使用
    // 避免重复 I2C 探测破坏 IT7259 的配置状态
    if (ctpAvailable) {
        touchType = TOUCH_I2C_FT6336;  // 复用FT6336类型标记
        Serial.println("[TOUCH] IT7259 already initialized by ctpInit, skip probe");
        return;
    }
    
    // 尝试检测CST816S (地址0x15)
    Wire.beginTransmission(0x15);
    if (Wire.endTransmission() == 0)
    {
        // 读取芯片ID确认
        Wire.beginTransmission(0x15);
        Wire.write(0xA3); // CST816S 芯片ID寄存器
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)0x15, (uint8_t)1) == 1)
        {
            uint8_t chipId = Wire.read();
            if (chipId == 0xB5 || chipId == 0x81)
            {
                touchType = TOUCH_I2C_CST816S;
                Serial.printf("[TOUCH] CST816S detected (chip=0x%02X)\n", chipId);
                // 配置CST816S: 设置中断模式为低电平有效
                Wire.beginTransmission(0x15);
                Wire.write(0xFE);
                Wire.write(0x01); // 休眠使能
                Wire.endTransmission();
                Wire.beginTransmission(0x15);
                Wire.write(0xED);
                Wire.write(0x01); // 自动唤醒
                Wire.endTransmission();
                return;
            }
        }
    }

    // 尝试检测FT6336 / FT6206 (地址0x38)
    Wire.beginTransmission(0x38);
    if (Wire.endTransmission() == 0)
    {
        Wire.beginTransmission(0x38);
        Wire.write(0xA1); // FT6x36 芯片ID寄存器
        Wire.endTransmission(false);
        if (Wire.requestFrom((uint8_t)0x38, (uint8_t)1) == 1)
        {
            uint8_t chipId = Wire.read();
            if (chipId == 0x11 || chipId == 0x34 || chipId == 0x64)
            {
                touchType = TOUCH_I2C_FT6336;
                Serial.printf("[TOUCH] FT6336/FT6206 detected (chip=0x%02X)\n", chipId);
                // 配置FT6336: 设置触摸模式
                Wire.beginTransmission(0x38);
                Wire.write(0x00);
                Wire.write(0x01); // 设备模式: 工作模式
                Wire.endTransmission();
                return;
            }
        }
    }

    // 尝试检测IT7259 (地址0x38，与FT6336相同但协议不同)
    Wire.beginTransmission(CTP_ADDR);
    if (Wire.endTransmission() == 0)
    {
        // IT7259响应，检查是否是真正的IT7259
        // 读取点数据缓冲区
        uint8_t buf[14];
        Wire.beginTransmission(CTP_ADDR);
        Wire.write(CTP_POINT_BUFFER);
        Wire.endTransmission(false);
        if (Wire.requestFrom(CTP_ADDR, (uint8_t)14) == 14)
        {
            for (int i = 0; i < 14; i++) buf[i] = Wire.read();
            // IT7259点缓冲寄存器0xE0的响应格式不同于FT6336
            // 如果响应正常，设置IT7259类型
            ctpAvailable = true;
            touchType = TOUCH_I2C_FT6336; // 复用FT6336类型，但ctpAvailable标记为真
            Serial.println("[TOUCH] IT7259 detected at 0x38");
            return;
        }
    }

    // 未检测到I2C触摸控制器，使用GPIO触摸按键
    touchType = TOUCH_GPIO;
    Serial.println("[TOUCH] No I2C touch controller, using GPIO touch button (G16)");
}

// 读取CST816S触摸数据
static bool readCST816S()
{
    Wire.beginTransmission(0x15);
    Wire.write(0x01); // 触摸点数寄存器
    if (Wire.endTransmission(true) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)0x15, (uint8_t)5) != 5)
        return false;
    
    uint8_t touchPoints = Wire.read(); // 触摸点数
    if (touchPoints == 0)
        return false;
    
    // X坐标高位和低位
    uint8_t xH = Wire.read();
    uint8_t xL = Wire.read();
    uint8_t yH = Wire.read();
    uint8_t yL = Wire.read();
    
    touchX = ((uint16_t)(xH & 0x0F) << 8) | xL;
    touchY = ((uint16_t)(yH & 0x0F) << 8) | yL;
    
    // 坐标映射到屏幕 (CST816S原生240x240)
    if (touchX >= 240) touchX = 239;
    if (touchY >= 240) touchY = 239;
    
    return true;
}

// 读取FT6336触摸数据
static bool readFT6336()
{
    Wire.beginTransmission(0x38);
    Wire.write(0x02); // 触摸点数寄存器
    if (Wire.endTransmission(true) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)0x38, (uint8_t)5) != 5)
        return false;
    
    uint8_t status = Wire.read(); // 状态
    uint8_t touchPoints = status & 0x0F;
    if (touchPoints == 0)
        return false;
    
    // 第一个触摸点的X和Y
    uint8_t xH = Wire.read();
    uint8_t xL = Wire.read();
    uint8_t yH = Wire.read();
    uint8_t yL = Wire.read();
    
    touchX = ((uint16_t)(xH & 0x0F) << 8) | xL;
    touchY = ((uint16_t)(yH & 0x0F) << 8) | yL;
    
    if (touchX >= 240) touchX = 239;
    if (touchY >= 240) touchY = 239;
    
    return true;
}

// 主触摸处理函数
void handleTouch()
{
    unsigned long now = millis();
    if (now - bootTime < 2000)
        return;
    
    static bool touchInitialized = false;
    if (!touchInitialized)
    {
        initTouchController();
        touchInitialized = true;
        return;
    }
    
    if (touchType == TOUCH_I2C_CST816S || touchType == TOUCH_I2C_FT6336)
    {
        // I2C触摸控制器
        static unsigned long lastRead = 0;
        if (now - lastRead < 50)
            return; // 50ms采样率
        lastRead = now;
        
        bool prevPressed = touchPressed;
        if (touchType == TOUCH_I2C_CST816S)
            touchPressed = readCST816S();
        else if (ctpAvailable)
            touchPressed = ctpReadData();  // 使用IT7259
        else
            touchPressed = readFT6336();    // 标准FT6336

        // 触摸事件状态机
        static unsigned long touchStartTime = 0;
        static bool shortPressHandled = false;
        static bool longPressHandled = false;
        
        if (touchPressed)
        {
            if (!prevPressed)
            {
                // 新触摸开始
                touchStartTime = now;
                shortPressHandled = false;
                longPressHandled = false;
            }
            
            // 长按检测 (2秒重启) - 在HELP和COUNTDOWN状态下禁用
            if (!longPressHandled && (now - touchStartTime >= LONG_PRESS_TIME))
            {
                longPressHandled = true;
                if (currentDisplayState != HELP && currentDisplayState != COUNTDOWN)
                {
                    Serial.println("[TOUCH] Long press detected - restarting...");
                    delay(50);
                    ESP.restart();
                }
                else
                {
                    Serial.printf("[TOUCH] Long press ignored (state=%d)\n", currentDisplayState);
                }
            }
        }
        else if (prevPressed && !shortPressHandled)
        {
            // 触摸释放 - 处理短按
            unsigned long duration = now - touchStartTime;
            if (duration < LONG_PRESS_TIME)
            {
                shortPressHandled = true;
                if (ctpAvailable)
                {
                    // IT7259电容触摸：短按报警/取消由CTP处理器统一管理
                    // 此处不处理，避免双击时第二下误触发cancelAlarm()
                }
                else if (alarmActive)
                {
                    // 报警期间短按不取消，仅允许双击取消
                    Serial.println("[TOUCH] Short press ignored (alarm active, use double-tap to cancel)");
                }
                else if (!cancelCooldown)
                {
                    triggerAlarm();
                }
            }
        }
    } // I2C触摸控制器块结束
    
    // IT7259 CTP 点击处理已由 ctpHandleTouch() 独立处理，此处不再重复

    // ===== GPIO16 TTP223 触摸传感器（仅在无CTP时启用，避免与电容屏冲突） =====
    if (!ctpAvailable) {
        // TTP223默认模式：HIGH=触摸, LOW=未触摸。双击切换报警/取消，长按(>2s)重启
        // 使用INPUT_PULLDOWN: LOW=未触摸, HIGH=触摸，消除浮空干扰
        // 多重采样滤波：快速连续读取5次，防止蜂鸣器/震动EMI干扰导致误触发
        // 不使用delayMicroseconds，避免阻塞I2C总线导致传感器失效
        int rawRead = digitalRead(TOUCH_PIN);
        static int stableSt = LOW;
        int sampleCount = 0;
        for (int i = 0; i < 5; i++) {
            if (digitalRead(TOUCH_PIN) == rawRead) sampleCount++;
        }
        // 需要至少4/5次一致才认为读数有效，不一致则保持上次稳定值
        int st = (sampleCount >= 4) ? rawRead : stableSt;
        stableSt = st;
        static int last = LOW;  // 初始为未触摸状态
        static unsigned long touchDownTime = 0;
        static unsigned long lastTapTime = 0;  // 0.5秒防抖
        static int clickCount = 0;
        static unsigned long lastClickTime = 0;

        // 边沿检测：只在状态变化时处理
        if (st != last)
        {
            if (st == HIGH)
            {
                // 触摸按下 (HIGH)
                touchDownTime = now;
            }
            else
            {
                // 触摸释放 (LOW)
                unsigned long pressDuration = now - touchDownTime;
                
                // 长按检测 (2秒重启) - 在所有状态下都允许
                if (pressDuration >= LONG_PRESS_TIME)
                {
                    Serial.println("[TTP223] Long press (2s) detected - restarting...");
                    delay(50);
                    ESP.restart();
                    clickCount = 0;
                    return;
                }
                // 有效短按：15ms防抖，短于2秒，且0.15秒内无重复
                else if (pressDuration > 15 && pressDuration < LONG_PRESS_TIME)
                {
                    // 0.15秒防抖：防止EMI干扰导致的误触发
                    if (now - lastTapTime < 150) {
                        Serial.println("[TTP223] Ignored (0.15s debounce)");
                    } else {
                        lastTapTime = now;
                        clickCount++;
                        lastClickTime = now;

                        // 双击检测
                        if (clickCount >= 2)
                        {
                            clickCount = 0;
                            
                            // 报警触发后0.25秒内忽略触摸，防止残余触摸误取消
                            if (alarmActive && alarmTriggerTime > 0 && (now - alarmTriggerTime < 250))
                            {
                                Serial.println("[TTP223] Touch ignored (debounce 0.25s after alarm trigger)");
                            }
                            else if (alarmActive)
                            {
                                // 报警状态下（包括COUNTDOWN和HELP）都允许双击取消
                                cancelAlarm();
                                Serial.println("[TTP223] Double-tap: cancel alarm");
                            }
                            else if (!cancelCooldown && !alarmActive)
                            {
                                triggerAlarm();
                                Serial.println("[TTP223] Double-tap: trigger alarm");
                            }
                            return;  // 处理完成
                        }
                    }
                }
            }
            last = st;
        }

        // 双击超时重置：1.5秒内无第二次点击则清零
        if (clickCount > 0 && (now - lastClickTime > 1500))
        {
            clickCount = 0;
        }
    }
}

// ==================== 报警状态机 ====================
void triggerAlarm()
{
    if (alarmActive) return; // 已经在报警中，防止重复触发
    alarmActive = true;
    countdown = ALARM_COUNTDOWN;
    lastAlarmTime = millis();
    alarmTriggerTime = millis();  // 记录触发时间，用于触摸防抖（2秒内忽略触摸取消）
    currentDisplayState = COUNTDOWN;
    displayNeedsUpdate = true;
    // 倒计时期间只启动蜂鸣器+LED，不启动震动（震动电流大，避免brownout）
    // 震动留到SOS旋律阶段才激活
    setBuzzer(true);
    buzzerActive = true;
    buzzerStartTime = millis();
    delay(50);                // 50ms延迟让蜂鸣器电流稳定
    setLight(true);
    // 不启动震动，震动留到HELP状态的SOS旋律
    melodyActive = false;
    melodyStep = 0;
    callCompleted = false;  // 重置电话完成标记
    // 不重置 callTriggerAttempted，确保每个报警周期只触发一次4G
    // 不重置 callTriggerCount 和 lastCallTriggerDoneTime，保持最小间隔保护
    cancelCooldown = false;  // 重置取消冷却，防止误返回NORMAL
    showingCancelFeedback = false;  // 重置取消反馈标记
    voiceCmdDisplayTime = 0; // 清除语音命令显示超时，立即显示报警界面
    // 重置倒计时蜂鸣器相位状态
    countdownBuzzerOn = true;
    countdownPhaseStart = millis();
}
void updateAlarm()
{
    unsigned long now = millis();
    // 使用 += 1000 避免累积时间偏移，确保每秒精确倒计时
    if (now - lastAlarmTime >= 1000)
    {
        lastAlarmTime += 1000;
        // 防止长时间运行后的漂移：如果偏差超过200ms则重新对齐
        if (now - lastAlarmTime > 200) lastAlarmTime = now;
        
        if (countdown > 0)
        {
            countdown--;
            displayNeedsUpdate = true;
            // 不再强制重置蜂鸣器相位，让时序自然运行
            // 蜂鸣器/震动/LED的时序由主循环独立控制
        }
        
        // 修复：将 else if 改为 if，确保 countdown == 0 时能触发
        if (countdown == 0)
        {
            // 触发SOS只执行一次
            countdown = -1;
            Serial.println("!!! HELP !!!");
            currentDisplayState = HELP;
            displayNeedsUpdate = true;
            // 先关闭所有输出，避免4G模块触发时电流叠加导致brownout重启
            setBuzzer(false);
            setLight(false);
            setVibrate(false);
            countdownBuzzerOn = false;
            // 直接触发4G，移除 callTriggerAttempted 检查
            // triggerPhoneCall() 内部已有防重复触发机制
            triggerPhoneCall();
            // SOS联动模式将在电话发送完成后启动（callCompleted=true时）
        }
        // countdown < 0 时不做任何事，保持SOS状态
    }
}
void cancelAlarm()
{
    alarmActive = false;
    countdown = ALARM_COUNTDOWN;
    setBuzzer(false);
    setLight(false);
    setVibrate(false);
    buzzerActive = false;
    melodyActive = false;
    melodyStep = 0;
    callCompleted = false;  // 重置电话完成标记
    callTriggerAttempted = false;  // 重置4G触发尝试标记
    callTriggerCount = 0;  // 重置调用计数器，允许下次报警时触发
    callTriggerLocked = false;  // 立即释放锁定，允许下次报警时触发
    manualBuzzerOn = false;
    voiceCmdDisplayTime = 0;  // 清除语音命令显示超时，立即显示取消界面
    // 如果正在触发拨号，结束触发并重置状态机
    if (callTriggered)
    {
        digitalWrite(TRIGGER_OUT1_PIN, HIGH);
        digitalWrite(TRIGGER_OUT2_PIN, HIGH);
        callTriggered = false;
        callTriggerStage = CT_INIT;  // 重置拨号触发状态机
    }
    cancelCooldown = true;
    cancelCooldownStartTime = millis();
    showingCancelFeedback = true;
    cancelFeedbackStartTime = millis();
    currentDisplayState = CANCELLED;
    displayNeedsUpdate = true;
    Serial.println("--- ALARM CANCELLED ---");
}

// ==================== 时间 ====================
void updateTime()
{
    unsigned long now = millis();
    if (now - lastSecondTime >= 1000)
    {
        lastSecondTime = now;
        struct tm ti;
        if (getLocalTime(&ti))
        {
            int prevMinute = currentMinute;
            currentHour = ti.tm_hour;
            currentMinute = ti.tm_min;
            currentSecond = ti.tm_sec;
            if (currentMinute != prevMinute)
                displayNeedsUpdate = true;  // 仅在分钟变化时刷新显示
            return;
        }
        if (++currentSecond >= 60)
        {
            currentSecond = 0;
            if (++currentMinute >= 60)
            {
                currentMinute = 0;
                if (++currentHour >= 24)
                    currentHour = 0;
            }
            displayNeedsUpdate = true;  // 分钟变化时刷新显示
        }
    }
}

// ==================== SETUP ====================
void setup()
{
    // ===== 开机电源稳定延迟 =====
    delay(500);               // 等待电源稳定，避免欠压触发brownout重启
    Serial.begin(115200);
    delay(200);               // 等待串口稳定
    Serial.println("\n=== Elder Fall Alarm v2.0 (SFD01+ADXL345) ===\n");

    // 强制重置所有状态，确保开机干净
    alarmActive = false;
    cancelCooldown = false;
    melodyActive = false;
    melodyStep = 0;
    buzzerActive = false;
    manualBuzzerOn = false;
    showingCancelFeedback = false;
    startupPending = false;
    currentDisplayState = NORMAL;
    displayNeedsUpdate = true;
    systemReady = false;
    systemError = false;
    callTriggered = false;
    callTriggerStage = CT_INIT;  // 重置4G触发状态机
    callTriggerLocked = false;   // 不锁定，允许立即触发（测试模式）
    callTriggerLockTime = 0;
    callTriggerAttempted = false;
    callTriggerCount = 0;  // 初始化调用计数
    lastCallTriggerDoneTime = 0;
    sfdHasFall = false;
    sfdHasPerson = false;
    adxlFreeFall = false;
    adxlImpact = false;

    // ===== 最先初始化4G触发引脚，防止启动误触发 =====
    // 同时初始化INT1=G12和INT2=G14两个引脚
    // 重要：先配置pinMode为OUTPUT，确保引脚状态可控，再设置电平
    pinMode(TRIGGER_OUT1_PIN, OUTPUT);
    pinMode(TRIGGER_OUT2_PIN, OUTPUT);
    digitalWrite(TRIGGER_OUT1_PIN, HIGH);  // 确保HIGH（不触发）
    digitalWrite(TRIGGER_OUT2_PIN, HIGH);  // 确保HIGH（不触发）
    Serial.println("[4G] Trigger pins initialized (INT1=G12, INT2=G14, HIGH=idle)");

    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("Reset reason: ");
    switch (rr)
    {
    case ESP_RST_POWERON:    Serial.println("Power-on"); break;
    case ESP_RST_BROWNOUT:   Serial.println("BROWNOUT!!! Check power supply"); break;
    case ESP_RST_SW:         Serial.println("Software (ESP.restart())"); break;
    case ESP_RST_PANIC:      Serial.println("PANIC!!! Software exception/crash"); break;
    case ESP_RST_INT_WDT:    Serial.println("Interrupt watchdog"); break;
    case ESP_RST_TASK_WDT:   Serial.println("Task watchdog"); break;
    case ESP_RST_WDT:        Serial.println("Watchdog"); break;
    case ESP_RST_DEEPSLEEP:  Serial.println("Deep sleep"); break;
    case ESP_RST_SDIO:       Serial.println("SDIO"); break;
    default:                 Serial.printf("0x%X\n", (int)rr); break;
    }
    esp_task_wdt_init(15, true);
    esp_task_wdt_reset();

    Wire.begin(I2C_SDA, I2C_SCL);
    // 使能内部上拉电阻（某些模块无板载上拉时需要）
    pinMode(I2C_SDA, INPUT_PULLUP);
    pinMode(I2C_SCL, INPUT_PULLUP);
    Wire.setClock(50000); // 降低到50kHz提高稳定性
    delay(100); // 等待I2C总线稳定
    Wire1.begin(ASR_SDA, ASR_SCL);
    Wire1.setClock(100000); // 提升到100kHz（原10kHz过慢）
    Serial.print("Wire1 I2C (ASR) devices:");
    for (uint8_t a = 0x08; a < 0x78; a++)
    {
        Wire1.beginTransmission(a);
        if (Wire1.endTransmission() == 0)
            Serial.printf(" 0x%02X", a);
    }
    Serial.println();

    preferences.begin("fallalarm", false);

    // ASR语音模块初始化：发送唤醒命令
    Wire1.beginTransmission(ASR_I2C_ADDR);
    Wire1.write(ASR_RESULT_REG);
    Wire1.write((uint8_t)0);
    Wire1.endTransmission();
    delay(50);

    // 检查固件版本：若与上次存储版本不同，说明是新烧录 → 播报"烧录成功"
    String storedVer = preferences.getString("fw_ver", "");
    bool isNewUpload = (storedVer != FW_VERSION);
    if (isNewUpload) {
        preferences.putString("fw_ver", FW_VERSION);
        Wire1.beginTransmission(ASR_I2C_ADDR);
        Wire1.write(ASR_SPEAK_REG);
        Wire1.write(ASR_TYPE_COMMAND);
        Wire1.write(0x02); // 播报ID=2（烧录成功）
        Wire1.endTransmission();
        delay(1500); // 等待"烧录成功"播完
        Serial.println("[ASR] New firmware detected, playing 'Flash OK'");
    }

    // 播报开机欢迎语"你好宿主，欢迎使用本系统"
    Wire1.beginTransmission(ASR_I2C_ADDR);
    Wire1.write(ASR_SPEAK_REG);
    Wire1.write(ASR_TYPE_COMMAND);
    Wire1.write(0x01); // 播报ID=1（你好宿主，欢迎使用本系统）
    Wire1.endTransmission();
    Serial.printf("[ASR] Initialized (addr=0x%02X, Wire1 clock=%d)\n", ASR_I2C_ADDR, 100000);

    // 注意：Wire I2C设备扫描已移至adxlInit中，避免重复扫描拖慢启动

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(LIGHT_PIN, OUTPUT);
    // LED改用PWM控制，降低功耗减少电压跌落
    ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RES);
    ledcAttachPin(LIGHT_PIN, LED_PWM_CHANNEL);
    ledcWrite(LED_PWM_CHANNEL, 0);
    digitalWrite(VIBRATE_PIN, LOW);
    pinMode(VIBRATE_PIN, OUTPUT);
    // CANCEL_BUTTON(G33)与SFD_RX(G33)冲突，已禁用物理按钮
    pinMode(TOUCH_PIN, INPUT_PULLDOWN); // TTP223: LOW=未触摸, HIGH=触摸, 加下拉防止浮空
    bootTime = millis();
    setBuzzer(false);
    setLight(false);
    initDisplay();
    displayNeedsUpdate = true;
    // 初始化电容触摸屏 IT7259
    ctpInit();
    // 立即显示当前界面，避免启动时黑屏
    updateDisplay();

    Serial.println("ADXL345 init...");
    if (!adxlInit())
        Serial.println("[ADXL] FAILED - check G21/G22 wiring");
    esp_task_wdt_reset();  // 喂狗

    Serial.println("MAX30102 init...");
    if (!max30102Init())
        Serial.println("[MAX30102] FAILED - check G21/G22 wiring");
    esp_task_wdt_reset();

    // MAX30102 INT引脚 (G34仅支持输入，不支持上拉)
    pinMode(MAX30102_INT, INPUT);

    Serial.println("SFD01 init...");
    sfdInit();
    esp_task_wdt_reset();

    // GPS模块初始化
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    esp_task_wdt_reset();
    Serial.printf("[GPS] UART initialized (9600, RX=G%d, TX=G%d)\n", GPS_RX, GPS_TX);

    Serial.println("[VIBRATE] Pin initialized (G27)");

    loadWiFiConfig();
    
    // 如果WiFi已配置，先尝试自动连接，而不是直接进入配置模式
    if (wifiConfigured) {
        Serial.println("[WiFi] Saved config found, attempting auto-connect...");
        inConfigMode = false;
        currentDisplayState = NORMAL;
        
        // 启动AP+STA双模式，允许同时提供AP热点和连接STA网络
        WiFi.mode(WIFI_AP_STA);
        delay(100);
        
        // 启动AP热点（用于配网备用）
        WiFi.softAP(AP_SSID, NULL, 1, 0, 8);
        delay(200);
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        
        // 启动DNS和WebServer（备用）
        IPAddress apIP = WiFi.softAPIP();
        dnsServer.start(DNS_PORT, "*", apIP);
        server.on("/", handleRoot);
        server.on("/save", handleSave);
        server.on("/scan", handleScan);
        server.on("/generate_204", []() { server.send(204, "text/plain", ""); });
        server.on("/hotspot-detect.html", []() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", ""); });
        server.on("/success.txt", []() { server.send(200, "text/plain", "success"); });
        server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
        server.begin();
        
        // 尝试连接已保存的WiFi
        String ssid = preferences.getString("ssid", "");
        String password = preferences.getString("password", "");
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        WiFi.begin(ssid.c_str(), password.c_str());
        
        Serial.printf("[WiFi] Connecting to %s...\n", ssid.c_str());
        
        // 等待10秒看是否连接成功
        int retryCount = 0;
        while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
            delay(500);
            Serial.print(".");
            retryCount++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n[WiFi] Auto-connect SUCCESS!");
            Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
            // 连接成功，进入正常模式
            currentDisplayState = NORMAL;
        } else {
            Serial.println("\n[WiFi] Auto-connect FAILED, entering config mode...");
            // 连接失败，进入配置模式
            inConfigMode = true;
            currentDisplayState = CONFIG;
            startAPMode();
        }
    } else {
        // 未配置WiFi，进入配置模式
        Serial.println("[WiFi] No saved config, entering config mode...");
        inConfigMode = true;
        currentDisplayState = CONFIG;
        startAPMode();
    }

    Serial.println("Ready.");
}

// ==================== LOOP ====================
void loop()
{
    // 每次循环开始立即喂看门狗，防止超时重启
    esp_task_wdt_reset();

    unsigned long now = millis();

    // 开机稳定期过后，释放4G触发锁定，允许正常报警触发
    // 延长到120秒，确保传感器完全稳定
    if (callTriggerLocked && (now - bootTime > 120000)) {
        callTriggerLocked = false;
        Serial.println("[4G] Boot lock released after 120s, 4G trigger enabled");
    }

    // 检查异步WiFi扫描结果（非阻塞）
    checkScanResults();
    
    // WebServer健康检查（配置模式下）
    checkServerHealth();

    // 串口命令：发送 v <id> 测试语音ID（例如 v 5 播报ID=5的语音）
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.startsWith("v ")) {
            int id = cmd.substring(2).toInt();
            if (id >= 0 && id <= 255) {
                Wire1.beginTransmission(ASR_I2C_ADDR);
                Wire1.write(ASR_SPEAK_REG);
                Wire1.write(ASR_TYPE_COMMAND);
                Wire1.write((uint8_t)id);
                Wire1.endTransmission();
                Serial.printf("[TEST] Speaking voice ID=%d (0x%02X)\n", id, id);
            }
        }
        else if (cmd == "list") {
            Serial.println("=== Testing voice IDs 0x00~0x10 ===");
            for (int i = 0; i <= 0x10; i++) {
                Wire1.beginTransmission(ASR_I2C_ADDR);
                Wire1.write(ASR_SPEAK_REG);
                Wire1.write(ASR_TYPE_COMMAND);
                Wire1.write((uint8_t)i);
                Wire1.endTransmission();
                Serial.printf("  ID=%d (0x%02X)\n", i, i);
                delay(2000);
                esp_task_wdt_reset(); // 延长期间喂狗
            }
            Serial.println("=== Done ===");
        }
        else if (cmd == "gps") {
            // GPS测试命令：显示当前GPS状态
            Serial.printf("[GPS] Status: Fixed=%d LAT=%.6f LON=%.6f SAT=%d ALT=%.1f SPD=%.1f TIME=%s\n",
                          gpsFixed, gpsLat, gpsLon, gpsSatellites, gpsAltitude, gpsSpeed, gpsTime.c_str());
        }
        else if (cmd == "fall") {
            // 测试报警触发
            Serial.println("[TEST] Triggering alarm test...");
            alarmActive = true;
            countdown = ALARM_COUNTDOWN;
            displayNeedsUpdate = true;
            melodyActive = false;
            melodyStep = 0;
            callCompleted = false;
            callTriggerAttempted = false;
            callTriggerCount = 0;
            callTriggerLocked = false;
            callTriggered = false;
            callTriggerStage = CT_INIT;
            currentDisplayState = COUNTDOWN;
        }
    }

    // ===== 倒计时期间处理 =====
    if (alarmActive && currentDisplayState == COUNTDOWN)
    {
        // 倒计时期间处理核心任务
        handleVoiceCommand();
        
        // 正常处理触摸（双击取消，长按重启）
        handleTouch();
        // COUNTDOWN状态下也需要调用ctpHandleTouch()处理双击取消
        if (ctpAvailable) {
            ctpHandleTouch();
        }
        
        // 更新报警倒计时
        updateAlarm();
        // 如果updateAlarm()已将状态切换到HELP（倒计时结束），
        // 立即跳过蜂鸣器控制，避免将已关闭的输出重新打开导致电流尖峰重启
        if (currentDisplayState != COUNTDOWN) {
            delay(10);
            return;
        }
        
        // 5. 检查拨号触发状态
        updateCallTriggerStateMachine();
        checkCallTriggerTimeout();
        
        // 6. 更新显示（已优化为只在数值变化时重绘）
        if (displayNeedsUpdate)
        {
            updateDisplay();
            displayNeedsUpdate = false;
        }
        
        // 7. 蜂鸣器/灯光/震动控制 (倒计时期间蜂鸣器+LED+震动同步)

        if (countdownBuzzerOn)
        {
            // 响铃阶段：蜂鸣器+LED+震动同时响0.3秒
            if (now - countdownPhaseStart >= BUZZER_BEEP_DURATION)
            {
                setBuzzer(false);
                setVibrate(false);
                delay(10);
                setLight(false);
                countdownBuzzerOn = false;
                countdownPhaseStart = now;
            }
        }
        else
        {
            // 静音阶段：0.7秒后进入响铃
            if (now - countdownPhaseStart >= BUZZER_OFF_DURATION)
            {
                // 分步开启：蜂鸣器 -> 震动 -> 灯光，每步10ms
                setBuzzer(true);
                delay(10);
                setVibrate(true);
                delay(10);
                setLight(true);
                countdownBuzzerOn = true;
                countdownPhaseStart = now;
            }
        }
        
        // 再次喂狗确保安全
        esp_task_wdt_reset();
        
        delay(10); // 短暂延迟，避免CPU占用过高
        return; // 跳过其他任务处理
    }

    // ===== 配置模式下：updateDisplay()会处理所有配置页面逻辑（绘制+WebServer+WiFi连接+return）=====
    if (inConfigMode)
    {
        updateDisplay();  // 配置页面的所有逻辑都在updateDisplay()中处理，包括return
        // 冗余调用：确保即使updateDisplay()内部未调用handleClient()，也能处理HTTP请求
        server.handleClient();
        return;           // 必须return，否则正常显示代码会覆盖配置页面
    }

    // ===== 正常状态下的任务处理 =====
    handleVoiceCommand();
    parseGPS();        // GPS数据解析（已优化为限制每次解析字节数）
    server.handleClient();

    if (systemError)
    {
        if (currentDisplayState != ERROR)
        {
            currentDisplayState = ERROR;
            updateDisplay();
        }
        return;
    }
    if (showingCancelFeedback && !alarmActive)
    {
        if (now - cancelFeedbackStartTime >= 1500)
        {
            showingCancelFeedback = false;
            currentDisplayState = NORMAL;
            displayNeedsUpdate = true;
            updateDisplay();
        }
    }
    if (wifiConfigured)
        checkWiFiConnection();

    // CANCEL_BUTTON (G33) 已禁用 - G33浮空导致误触发

    if (!alarmActive && !cancelCooldown)
    {
        // 传感器数据处理（非报警状态）
        // 使用时间间隔控制，避免过于频繁的I2C通信
        static unsigned long lastSensorUpdate = 0;
        if (now - lastSensorUpdate >= 50) // 50ms间隔
        {
            lastSensorUpdate = now;
            adxlProcessData();
            sfdProcessData();
            
            // MAX30102读取频率降低（200ms间隔已在函数内部实现）
            max30102Read();
            
            if (now - bootTime >= BOOT_STABILIZE_TIME)
                processFallDetection();
        }
    }
    if (cancelCooldown && (now - cancelCooldownStartTime > CANCEL_COOLDOWN_TIME))
    {
        cancelCooldown = false;
        // 取消冷却结束后，自动返回NORMAL监测界面
        if (currentDisplayState == CANCELLED)
        {
            currentDisplayState = NORMAL;
            displayNeedsUpdate = true;
            Serial.println("[DISPLAY] Returning to NORMAL mode after alarm cancel");
        }
    }

    // 检查拨号触发超时
    updateCallTriggerStateMachine();
    checkCallTriggerTimeout();
    handleTouch();
    // CTP触摸处理：ctpReadData() 在 handleTouch() 中读取数据，ctpHandleTouch() 独立处理点击逻辑
    // COUNTDOWN期间不会执行到这里（有early return），避免EMI噪声误触发
    if (ctpAvailable) {
        ctpHandleTouch();
    }
    
    if (alarmActive)
        updateAlarm();

    // HELP状态下的旋律播放
    // 电话拨打完成后延迟8秒再启动SOS蜂鸣，避免与4G模块同时工作
    // SOS模式：1秒响2次（250ms on/250ms off），急促短促
    // 最多持续30秒，之后自动停止防止过载重启
    static bool sosStartedLogged = false;
    static unsigned long sosStartTime = 0;
    static unsigned long sosMelodyStartTime = 0;  // SOS旋律开始时间（用于限时）
    
    // 修复：HELP状态下也要处理4G触发状态机
    if (currentDisplayState == HELP) {
        updateCallTriggerStateMachine();
        checkCallTriggerTimeout();
    }
    
    if (currentDisplayState == HELP && !melodyActive && callCompleted)
    {
        if (sosStartTime == 0) {
            sosStartTime = millis();
            Serial.println("[SOS] Waiting for 4G module to stabilize...");
        }
        unsigned long elapsed = millis() - sosStartTime;
        // 8秒后启动SOS蜂鸣（给4G模块足够时间完成拨号）
        if (elapsed >= 8000 && !melodyActive)
        {
            // 分步启动，避免电流尖峰：先蜂鸣器 -> 50ms -> 震动 -> 50ms -> LED
            setBuzzer(true);
            delay(50);
            setVibrate(true);
            delay(50);
            setLight(true);
            melodyActive = true;
            melodyStep = 0;
            melodyTimer = now;
            sosMelodyStartTime = now;
            sosStartTime = 0;
            displayNeedsUpdate = true;
            Serial.println("[SOS] Melody started (2 beeps/sec, max 30s)");
            sosStartedLogged = true;
        }
    }
    if (melodyActive)
    {
        // SOS: 1秒响2次 = 250ms ON / 250ms OFF
        // 震动与蜂鸣器同步，LED常亮
        // 30秒后自动停止，防止过载导致重启
        if (now - sosMelodyStartTime >= SOS_MAX_DURATION) {
            // 30秒时限到，停止SOS并重置所有报警状态
            melodyActive = false;
            melodyStep = 0;
            setBuzzer(false);
            setVibrate(false);
            setLight(false);
            
            // 重置报警状态，允许下次触发
            alarmActive = false;
            countdown = ALARM_COUNTDOWN;
            callCompleted = false;
            callTriggerAttempted = false;
            callTriggerCount = 0;
            callTriggerLocked = false;
            if (callTriggered) {
                digitalWrite(TRIGGER_OUT1_PIN, HIGH);
                digitalWrite(TRIGGER_OUT2_PIN, HIGH);
                callTriggered = false;
                callTriggerStage = CT_INIT;
            }
            
            currentDisplayState = NORMAL;
            displayNeedsUpdate = true;
            Serial.println("[SOS] Time limit reached (30s), alarm reset");
        }
        else if (now - melodyTimer >= SOS_BEEP_DURATION)
        {
            melodyTimer = now;
            melodyStep = (melodyStep + 1) % 2;  // 0=ON, 1=OFF
            if (melodyStep == 0)
            {
                // ON阶段：蜂鸣器+震动同步，LED常亮
                setBuzzer(true);
                setVibrate(true);
                setLight(true);
            }
            else
            {
                // OFF阶段：蜂鸣器+震动关闭，LED保持常亮
                setBuzzer(false);
                setVibrate(false);
            }
        }
    }
    if (currentDisplayState != HELP && melodyActive)
    {
        melodyActive = false;
        melodyStep = 0;
        setBuzzer(false);
        setVibrate(false);
        setLight(false);
        sosMelodyStartTime = 0;
    }

    updateTime();
    
    // 显示更新优化：减少刷新频率，避免闪烁
    static unsigned long lastDisplayDrawTime = 0;
    if (displayNeedsUpdate)
    {
        if (voiceCmdDisplayTime > 0 && now - voiceCmdDisplayTime < 3000)
            return;
        voiceCmdDisplayTime = 0;
        // 最小刷新间隔保护：300ms内不重复绘制
        if (now - lastDisplayDrawTime >= MIN_DISPLAY_INTERVAL) {
            lastDisplayDrawTime = now;
            lastDisplayTime = now;
            updateDisplay();
            displayNeedsUpdate = false;
        }
    }
    else if (currentDisplayState == NORMAL && now - lastDisplayTime >= 2000)
    {
        // 正常状态下每2秒刷新一次（减少闪烁）
        lastDisplayDrawTime = now;
        lastDisplayTime = now;
        updateDisplay();
    }
    
    // 循环结束再次喂狗
    esp_task_wdt_reset();
    
    delay(10);
}