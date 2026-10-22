# 老人防摔智能报警系统（Elder Fall Monitor）

基于 ESP32 的可穿戴/桌面式跌倒监测终端：毫米波雷达与加速度传感器双路识别跌倒，检测到异常后声、光、震动同步告警并自动拨打预设号码，同时上报 GPS 位置与心率血氧数据。设备自带 WiFi 配置热点，手机连上热点即可完成联网设置，无需安装 App。

## 功能

- 毫米波雷达（SFD01）人体姿态监测，跌倒识别
- 三轴加速度（ADXL345）异常动作辅助判定，降低误报
- 心率血氧（MAX30102）实时采集
- 跌倒后自动 SOS：蜂鸣器 + LED + 震动马达，并拨打预设号码
- GPS 定位与位置上报
- 1.28 英寸圆屏（GC9A01，240×240）本地状态显示，内置中文字库
- WiFi 配置热点（Captive Portal）：扫码/连接热点后在网页里选 WiFi、填密码即可配网
- 断网自动重连、WebServer 崩溃自动重启

## 硬件

| 部件 | 型号 | 接口 |
| --- | --- | --- |
| 主控 | ESP32 Dev Module | — |
| 显示屏 | GC9A01 240×240 圆屏 | SPI |
| 毫米波雷达 | SFD01 | UART |
| 加速度计 | ADXL345 | I2C |
| 心率血氧 | MAX30102 | I2C |
| 定位 | GPS 模块 | UART |
| 报警 | 蜂鸣器 / LED / 震动马达 | GPIO |

## 目录结构

```
platformio.ini        PlatformIO 工程配置（esp32dev + TFT_eSPI）
src/main.cpp          主程序：传感器采集、跌倒判定、告警、配网、WebServer
src/chinese_font.h    屏幕用中文字库点阵
gen_chinese_font.py   字库生成脚本
docs/产品使用手册.md   产品说明、操作指南与常见问题
include/ lib/ test/   PlatformIO 标准目录
```

## 编译与烧录

```bash
pip install platformio
pio run                 # 编译
pio run -t upload       # 烧录（默认 COM 口，按实际修改）
pio device monitor -b 115200
```

依赖库由 `platformio.ini` 的 `lib_deps` 自动安装（TFT_eSPI）。TFT_eSPI 的引脚配置需按实际接线在库的 `User_Setup.h` 中设置。

## 配网

1. 首次上电或长按配网键，设备进入热点模式，热点名 `FallAlarm-Config`。
2. 手机连接该热点，浏览器打开 `192.168.4.1`。
3. 选择 WiFi、填写密码并保存，设备自动重启并连接网络。

## 说明

本项目为课程/竞赛作品，用于跌倒监测与告警演示，不作为医疗器械使用，不能替代专业监护与急救。
