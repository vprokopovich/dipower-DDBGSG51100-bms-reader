// Монитор LiFePO4-батареи Dipower DDBGSG51100 (BMS PACE) по RS485.
// ESP32 опрашивает BMS, отдаёт телеметрию по HTTP (страница + JSON) и в MQTT.
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <stdarg.h>

#include "config.h"
#include "pace_protocol.h"
#include "web_page.h"

using pace::BmsData;
using pace::Protocol;

// ============================================================== Журнал
// Кольцевой буфер последних сообщений: виден на http://<устройство>/log,
// чтобы отлаживать связь с BMS без подключения по USB.
static char logBuf[6000];
static size_t logHead = 0;
static bool logWrapped = false;

static void logf(const char *fmt, ...) {
  char line[220];
  int n = snprintf(line, sizeof(line), "[%7lu] ", (unsigned long)(millis() / 1000));
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line + n, sizeof(line) - n - 2, fmt, ap);
  va_end(ap);
  strcat(line, "\n");
  Serial.print(line);
  for (const char *c = line; *c; c++) {
    logBuf[logHead++] = *c;
    if (logHead == sizeof(logBuf)) {
      logHead = 0;
      logWrapped = true;
    }
  }
}

// ============================================================== Шина RS485
static HardwareSerial Bus(2);
static uint32_t busBaud = 0;
static uint8_t rx[1024];

static void busBegin(uint32_t baud) {
  if (baud == busBaud) return;
  if (busBaud == 0) {
    Bus.setRxBufferSize(1024);
    Bus.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  } else {
    Bus.updateBaudRate(baud);
  }
  busBaud = baud;
}

static void busSend(const uint8_t *data, size_t len) {
  while (Bus.available()) Bus.read();
  if (RS485_DE_PIN >= 0) digitalWrite(RS485_DE_PIN, HIGH);
  Bus.write(data, len);
  Bus.flush();  // ждём, пока уйдёт последний байт
  if (RS485_DE_PIN >= 0) digitalWrite(RS485_DE_PIN, LOW);
}

// Читает ответ: ждёт первый байт до firstMs, дальше — до паузы в 80 мс.
static size_t busRead(uint32_t firstMs) {
  size_t n = 0;
  const uint32_t start = millis();
  uint32_t last = start;
  for (;;) {
    while (Bus.available() && n < sizeof(rx)) {
      rx[n++] = Bus.read();
      last = millis();
    }
    const uint32_t now = millis();
    if (n == 0) {
      if (now - start >= firstMs) break;
    } else if (now - last >= 80 || now - start >= 2000 || n >= sizeof(rx)) {
      break;
    }
    delay(2);
  }
  return n;
}

// Некоторые модули RS485 возвращают эхо собственной передачи — убираем его.
static size_t stripEcho(size_t n, const uint8_t *req, size_t reqLen) {
  if (n >= reqLen && memcmp(rx, req, reqLen) == 0) {
    memmove(rx, rx + reqLen, n - reqLen);
    return n - reqLen;
  }
  return n;
}

static uint32_t lastGarbageLog = 0;
static void noteUnexpected(size_t n, const char *what) {
  if (n == 0 || millis() - lastGarbageLog < 5000) return;
  lastGarbageLog = millis();
  char hex[3 * 24 + 1] = "";
  for (size_t i = 0; i < n && i < 24; i++) sprintf(hex + i * 3, "%02X ", rx[i]);
  logf("%s: получено %u байт, не похоже на ответ: %s%s", what, (unsigned)n, hex,
       n > 24 ? "..." : "");
}

// ============================================================== Опрос BMS
struct Link {
  bool locked = false;
  Protocol protocol = Protocol::None;
  uint32_t baud = 0;
  uint8_t address = BMS_ADDRESS;
  bool asciiInfoAll = false;  // в поле INFO слать FF вместо номера батареи
};

static Link bmsLink;
static BmsData data;
static bool haveData = false;
static uint32_t dataAt = 0;
static uint32_t okCount = 0, errCount = 0;
static uint8_t failStreak = 0;
static uint32_t pollSeq = 0;

static bool modbusRead(uint8_t addr, uint16_t start, uint16_t count, BmsData &d,
                       uint32_t firstMs) {
  uint8_t req[8];
  pace::buildModbusRead(addr, start, count, req);
  busSend(req, sizeof(req));
  size_t n = stripEcho(busRead(firstMs), req, sizeof(req));
  const uint8_t *payload = pace::findModbusResponse(rx, n, addr, count);
  if (!payload) {
    noteUnexpected(n, "Modbus");
    return false;
  }
  pace::parseModbusRegisters(start, count, payload, d);
  delay(30);  // пауза между кадрами
  return true;
}

static pace::AsciiResult asciiQuery(uint8_t addr, uint8_t cid2, bool infoAll, const char **info,
                                    size_t *infoLen, uint32_t firstMs) {
  char infoHex[3];
  snprintf(infoHex, sizeof(infoHex), "%02X", infoAll ? 0xFF : addr);
  char req[32];
  size_t reqLen = pace::buildAsciiRequest(addr, cid2, infoHex, req, sizeof(req));
  busSend((const uint8_t *)req, reqLen);
  size_t n = stripEcho(busRead(firstMs), (const uint8_t *)req, reqLen);
  uint8_t rtn = 0;
  pace::AsciiResult res = pace::findAsciiFrame((const char *)rx, n, info, infoLen, &rtn);
  if (res == pace::AsciiResult::ErrorCode) {
    logf("ASCII: BMS ответила кодом ошибки 0x%02X на команду 0x%02X", rtn, cid2);
  } else if (res != pace::AsciiResult::Ok) {
    noteUnexpected(n, "ASCII");
  }
  delay(30);
  return res;
}

// Один полный опрос по заданным параметрам связи. probe = быстрая проба при поиске.
static bool pollBms(Link &l, BmsData &d, bool probe) {
  busBegin(l.baud);
  pace::reset(d);

  if (l.protocol == Protocol::Modbus) {
    if (!modbusRead(l.address, 0, 8, d, probe ? 250 : 500)) return false;
    if (!modbusRead(l.address, 15, 22, d, 500)) return false;
    modbusRead(l.address, 9, 4, d, 500);  // флаги; не критично, если не ответит
    return true;
  }

  const char *info = nullptr;
  size_t infoLen = 0;
  pace::AsciiResult res =
      asciiQuery(l.address, pace::CID2_ANALOG, l.asciiInfoAll, &info, &infoLen, probe ? 400 : 800);
  if (res == pace::AsciiResult::ErrorCode && probe && !l.asciiInfoAll) {
    l.asciiInfoAll = true;
    res = asciiQuery(l.address, pace::CID2_ANALOG, true, &info, &infoLen, 800);
    if (res != pace::AsciiResult::Ok) l.asciiInfoAll = false;
  }
  if (res != pace::AsciiResult::Ok || !pace::parseAsciiAnalog(info, infoLen, d)) return false;

  if (asciiQuery(l.address, pace::CID2_WARN, l.asciiInfoAll, &info, &infoLen, 800) ==
      pace::AsciiResult::Ok) {
    pace::parseAsciiWarn(info, infoLen, d);
  }
  return true;
}

// ---- Автопоиск: перебор скорость × адрес × протокол, по одной пробе за проход loop()
static const uint32_t BAUDS[] = BMS_BAUD_RATES;
static const size_t BAUD_COUNT = sizeof(BAUDS) / sizeof(BAUDS[0]);
static const size_t ADDR_SLOTS = 17;  // слот 0 = адрес из настроек, 1..16 = адреса 0..15
static size_t scanIndex = 0;
static uint32_t scanRounds = 0;

static void scanStep() {
  const size_t total = BAUD_COUNT * ADDR_SLOTS * 2;
  const size_t i = scanIndex;
  scanIndex = (scanIndex + 1) % total;
  if (scanIndex == 0) {
    scanRounds++;
    logf("BMS не найдена (полный перебор №%lu). Проверьте A/B, питание BMS и порт.",
         (unsigned long)scanRounds);
  }

  const size_t addrSlot = (i / 2) % ADDR_SLOTS;
  if (addrSlot > 0 && addrSlot - 1 == BMS_ADDRESS) return;  // уже пробовали в слоте 0

  Link candidate;
  candidate.baud = BAUDS[i / (2 * ADDR_SLOTS)];
  candidate.address = addrSlot == 0 ? BMS_ADDRESS : addrSlot - 1;
  candidate.protocol = (i % 2 == 0) ? Protocol::Modbus : Protocol::Ascii;

  BmsData d;
  if (!pollBms(candidate, d, true)) return;

  candidate.locked = true;
  bmsLink = candidate;
  data = d;
  haveData = true;
  dataAt = millis();
  failStreak = 0;
  okCount++;
  pollSeq++;
  logf("BMS найдена: протокол %s, адрес %u, %lu бод. SOC %.0f%%, %.2f В, %.2f А",
       pace::protocolName(bmsLink.protocol), bmsLink.address, (unsigned long)bmsLink.baud, d.soc,
       d.voltage, d.current);
}

static void bmsTick() {
  static uint32_t lastPoll = 0;
  if (!bmsLink.locked) {
    scanStep();
    return;
  }
  if (millis() - lastPoll < POLL_INTERVAL_MS) return;
  lastPoll = millis();

  BmsData d;
  if (pollBms(bmsLink, d, false)) {
    data = d;
    haveData = true;
    dataAt = millis();
    failStreak = 0;
    okCount++;
    pollSeq++;
    return;
  }
  errCount++;
  if (++failStreak >= 5) {
    logf("BMS перестала отвечать, начинаю поиск заново");
    bmsLink.locked = false;
    scanIndex = 0;
  }
}

static bool bmsOnline() {
  const uint32_t staleMs = max<uint32_t>(30000, POLL_INTERVAL_MS * 4);
  return haveData && millis() - dataAt < staleMs;
}

// ============================================================== JSON
struct Json {
  char buf[2600];
  size_t n = 0;
  bool first = true;

  void raw(const char *fmt, ...) {
    if (n >= sizeof(buf) - 1) return;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + n, sizeof(buf) - n, fmt, ap);
    va_end(ap);
    if (w > 0) n = min(n + (size_t)w, sizeof(buf) - 1);
  }
  void key(const char *k) {
    raw("%s\"%s\":", first ? "{" : ",", k);
    first = false;
  }
  void value(double v, int decimals) {
    if (isnan(v)) raw("null");
    else raw("%.*f", decimals, v);
  }
  void num(const char *k, double v, int decimals) { key(k); value(v, decimals); }
  void integer(const char *k, long v) { key(k); raw("%ld", v); }
  void boolean(const char *k, bool v) { key(k); raw(v ? "true" : "false"); }
  void tri(const char *k, int8_t v) { key(k); raw(v < 0 ? "null" : v ? "true" : "false"); }
  void str(const char *k, const char *s) { key(k); raw("\"%s\"", s); }  // строки без кавычек
  void end() { raw("}"); }
};

static Json json;

static const char *buildTelemetry() {
  Json &j = json;
  j.n = 0;
  j.first = true;
  const bool online = bmsOnline();

  j.boolean("online", online);
  j.boolean("searching", !bmsLink.locked);
  if (haveData) j.integer("age_ms", millis() - dataAt);
  else { j.key("age_ms"); j.raw("null"); }
  j.str("protocol", pace::protocolName(bmsLink.protocol));
  j.integer("address", bmsLink.address);
  j.integer("baud", bmsLink.baud);

  if (haveData) {
    const BmsData &d = data;
    const char *state = d.current >= IDLE_CURRENT_A    ? "charging"
                        : d.current <= -IDLE_CURRENT_A ? "discharging"
                                                       : "idle";
    j.num("soc", d.soc, 1);
    j.num("soh", d.soh, 0);
    j.num("voltage", d.voltage, 2);
    j.num("current", d.current, 2);
    j.num("power", d.voltage * d.current, 0);
    j.str("state", state);
    j.boolean("charging", d.current >= IDLE_CURRENT_A);
    j.boolean("discharging", d.current <= -IDLE_CURRENT_A);

    // Оценка времени до полного заряда / до разряда при текущем токе
    j.key("eta_min");
    if (d.current >= IDLE_CURRENT_A && d.fullAh > d.remainAh) {
      j.raw("%ld", lroundf((d.fullAh - d.remainAh) / d.current * 60));
    } else if (d.current <= -IDLE_CURRENT_A) {
      j.raw("%ld", lroundf(d.remainAh / -d.current * 60));
    } else {
      j.raw("null");
    }

    j.num("remaining_ah", d.remainAh, 2);
    j.num("full_ah", d.fullAh, 2);
    j.num("design_ah", d.designAh, 2);
    j.integer("cycles", d.cycles);

    j.key("cells_mv");
    j.raw("[");
    for (uint8_t i = 0; i < d.cellCount; i++) j.raw("%s%u", i ? "," : "", d.cellMv[i]);
    j.raw("]");
    uint16_t lo = 0, hi = 0;
    if (pace::cellStats(d, lo, hi)) {
      j.integer("cell_min_mv", lo);
      j.integer("cell_max_mv", hi);
      j.integer("cell_delta_mv", hi - lo);
    }

    j.key("temps_c");
    j.raw("[");
    for (uint8_t i = 0; i < d.tempCount; i++) {
      if (i) j.raw(",");
      j.value(d.tempC[i], 1);
    }
    j.raw("]");
    j.num("temp_mosfet_c", d.mosfetTempC, 1);
    j.num("temp_env_c", d.envTempC, 1);

    j.tri("charge_fet", d.chargeFet);
    j.tri("discharge_fet", d.dischargeFet);
    j.tri("current_limit", d.currentLimit);
    j.tri("heater", d.heater);
    j.tri("fully_charged", d.fullyCharged);
    j.integer("balancing_mask", d.balanceMask);
    j.str("warnings", d.warnings);
    j.str("protections", d.protections);
    j.str("faults", d.faults);
  }

  j.integer("ok_count", okCount);
  j.integer("err_count", errCount);
  j.integer("uptime_s", millis() / 1000);
  j.integer("rssi", WiFi.RSSI());
  j.str("ip", WiFi.localIP().toString().c_str());
  j.end();
  return j.buf;
}

// ============================================================== HTTP
static WebServer server(80);

static void handleLog() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain; charset=utf-8", "");
  if (logWrapped) server.sendContent(logBuf + logHead, sizeof(logBuf) - logHead);
  server.sendContent(logBuf, logHead);
  server.sendContent("");
}

static void setupHttp() {
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html; charset=utf-8", INDEX_HTML); });
  server.on("/api/telemetry", HTTP_GET, [] {
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "application/json", buildTelemetry());
  });
  server.on("/log", HTTP_GET, handleLog);
  server.on("/api/rescan", HTTP_POST, [] {
    bmsLink.locked = false;
    scanIndex = 0;
    logf("Поиск BMS запущен вручную");
    server.send(200, "application/json", "{\"ok\":true}");
  });
  server.onNotFound([] { server.send(404, "text/plain", "not found"); });
  server.begin();
}

// ============================================================== Wi-Fi
static bool netServicesStarted = false;
static bool apActive = false;

static void startNetServices() {
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setHostname(DEVICE_HOSTNAME);  // сам поднимает mDNS
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
  } else {
    MDNS.begin(DEVICE_HOSTNAME);
  }
  MDNS.addService("http", "tcp", 80);
  netServicesStarted = true;
}

static void wifiTick() {
  static uint32_t lastTry = 0;
  static uint32_t downSince = 0;
  static bool wasConnected = false;
  const bool connected = WiFi.status() == WL_CONNECTED;

  if (connected) {
    if (!wasConnected) {
      logf("Wi-Fi подключён: http://%s/ (http://%s.local/)", WiFi.localIP().toString().c_str(),
           DEVICE_HOSTNAME);
      if (!netServicesStarted) startNetServices();
      if (apActive) {
        WiFi.softAPdisconnect(true);
        apActive = false;
      }
    }
    wasConnected = true;
    return;
  }

  if (wasConnected || downSince == 0) {
    downSince = millis();
    wasConnected = false;
  }
  if (!apActive && millis() - downSince > 30000) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(DEVICE_HOSTNAME, FALLBACK_AP_PASSWORD);
    apActive = true;
    logf("Wi-Fi недоступен: поднята точка доступа \"%s\", страница на http://192.168.4.1/",
         DEVICE_HOSTNAME);
  }
  if (millis() - lastTry > 20000) {
    lastTry = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}

// ============================================================== MQTT
static WiFiClient mqttSocket;
static PubSubClient mqtt(mqttSocket);
static const bool mqttEnabled = strlen(MQTT_HOST) > 0;

static void mqttTick() {
  static uint32_t lastConnectTry = 0;
  static uint32_t lastPublish = 0;
  static uint32_t publishedSeq = 0;
  if (!mqttEnabled || WiFi.status() != WL_CONNECTED) return;

  if (!mqtt.connected()) {
    if (lastConnectTry != 0 && millis() - lastConnectTry < 10000) return;
    lastConnectTry = millis();
    char clientId[40];
    snprintf(clientId, sizeof(clientId), "%s-%06lx", DEVICE_HOSTNAME,
             (unsigned long)(ESP.getEfuseMac() & 0xFFFFFF));
    const char *user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
    const char *pass = strlen(MQTT_USER) ? MQTT_PASSWORD : nullptr;
    if (mqtt.connect(clientId, user, pass, MQTT_BASE_TOPIC "/status", 0, true, "offline")) {
      mqtt.publish(MQTT_BASE_TOPIC "/status", "online", true);
      logf("MQTT подключён к %s", MQTT_HOST);
    } else {
      logf("MQTT: не удалось подключиться к %s (код %d)", MQTT_HOST, mqtt.state());
      return;
    }
  }
  mqtt.loop();

  // Публикуем после каждого нового опроса; если BMS молчит — раз в 30 с (online=false)
  const bool fresh = pollSeq != publishedSeq;
  if (!fresh && millis() - lastPublish < 30000) return;
  publishedSeq = pollSeq;
  lastPublish = millis();
  mqtt.publish(MQTT_BASE_TOPIC "/telemetry", buildTelemetry(), true);
}

// ============================================================== setup / loop
void setup() {
  Serial.begin(115200);
  delay(200);
  logf("BMS-монитор запускается");

#ifdef BOARD_TCAN485
  pinMode(TCAN_5V_EN_PIN, OUTPUT);
  digitalWrite(TCAN_5V_EN_PIN, HIGH);
  pinMode(TCAN_RS485_EN_PIN, OUTPUT);
  digitalWrite(TCAN_RS485_EN_PIN, HIGH);
  pinMode(TCAN_RS485_SE_PIN, OUTPUT);
  digitalWrite(TCAN_RS485_SE_PIN, HIGH);
#endif
  if (RS485_DE_PIN >= 0) {
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // по умолчанию слушаем
  }

  pace::reset(data);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  setupHttp();

  if (mqttEnabled) {
    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(sizeof(json.buf) + 128);
    mqtt.setSocketTimeout(3);
  }
}

void loop() {
  wifiTick();
  server.handleClient();
  if (netServicesStarted && strlen(OTA_PASSWORD) > 0) ArduinoOTA.handle();
  mqttTick();
  bmsTick();
  delay(2);
}
