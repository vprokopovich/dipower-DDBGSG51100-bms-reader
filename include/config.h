// Настройки устройства. Свои значения удобнее задать в include/secrets.h
// (скопируйте secrets.example.h) — тогда этот файл трогать не нужно.
#pragma once

#if __has_include("secrets.h")
#include "secrets.h"
#endif

// ------------------------------------------------------------------ Wi-Fi
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_PASSWORD"
#endif
// Имя в сети: http://bms-monitor.local/
#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "bms-monitor"
#endif
// Если за 30 с не удалось подключиться к Wi-Fi, поднимается своя точка доступа
// с этим именем (страница будет на http://192.168.4.1/).
#ifndef FALLBACK_AP_PASSWORD
#define FALLBACK_AP_PASSWORD "bms-monitor"  // минимум 8 символов
#endif
// Пароль для обновления прошивки по воздуху (ArduinoOTA). Пустой = OTA выключено.
#ifndef OTA_PASSWORD
#define OTA_PASSWORD ""
#endif

// ------------------------------------------------------------------ MQTT (необязательно)
// Пустой MQTT_HOST = MQTT выключен, остаётся только веб-страница и /api/telemetry.
#ifndef MQTT_HOST
#define MQTT_HOST ""
#endif
#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif
#ifndef MQTT_USER
#define MQTT_USER ""
#endif
#ifndef MQTT_PASSWORD
#define MQTT_PASSWORD ""
#endif
// Телеметрия: <база>/telemetry (JSON), состояние: <база>/status (online/offline)
#ifndef MQTT_BASE_TOPIC
#define MQTT_BASE_TOPIC "bms/dipower"
#endif

// ------------------------------------------------------------------ BMS
// Адрес батареи = положение DIP-переключателей (одиночная батарея: 1).
// Если по этому адресу ответа нет, прошивка сама переберёт адреса 0..15.
#ifndef BMS_ADDRESS
#define BMS_ADDRESS 1
#endif
// Период опроса, мс
#ifndef POLL_INTERVAL_MS
#define POLL_INTERVAL_MS 5000
#endif
// Ток меньше этого порога (А) считается "покоем"
#ifndef IDLE_CURRENT_A
#define IDLE_CURRENT_A 0.15f
#endif
// Скорости, которые пробуются при поиске BMS. По документации — 9600.
#ifndef BMS_BAUD_RATES
#define BMS_BAUD_RATES {9600, 19200, 115200}
#endif

// ------------------------------------------------------------------ Выводы RS485
#ifdef BOARD_TCAN485
// LilyGO T-CAN485: трансивер MAX13487 с автонаправлением, всё разведено на плате
#define RS485_TX_PIN 22
#define RS485_RX_PIN 21
#define RS485_DE_PIN -1
#define TCAN_5V_EN_PIN 16     // повышающий преобразователь 5 В для трансиверов
#define TCAN_RS485_EN_PIN 17  // /RE трансивера
#define TCAN_RS485_SE_PIN 19  // /SHDN трансивера
#else
// ESP32 DevKit + модуль RS485
#ifndef RS485_TX_PIN
#define RS485_TX_PIN 17  // -> DI модуля (или TXD у модуля с автонаправлением)
#endif
#ifndef RS485_RX_PIN
#define RS485_RX_PIN 16  // <- RO модуля (или RXD)
#endif
#ifndef RS485_DE_PIN
#define RS485_DE_PIN 4  // -> DE+RE модуля MAX485; у модуля с автонаправлением не подключается
#endif
#endif
