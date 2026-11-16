#include "pace_protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace pace {

const char *protocolName(Protocol p) {
  switch (p) {
    case Protocol::Modbus: return "modbus";
    case Protocol::Ascii: return "pace-ascii";
    default: return "none";
  }
}

void reset(BmsData &d) {
  memset(&d, 0, sizeof(d));
  d.soc = d.soh = NAN;
  d.mosfetTempC = d.envTempC = NAN;
  d.chargeFet = d.dischargeFet = d.currentLimit = d.heater = d.fullyCharged = -1;
}

bool cellStats(const BmsData &d, uint16_t &minMv, uint16_t &maxMv) {
  if (d.cellCount == 0) return false;
  minMv = 0xFFFF;
  maxMv = 0;
  for (uint8_t i = 0; i < d.cellCount; i++) {
    if (d.cellMv[i] < minMv) minMv = d.cellMv[i];
    if (d.cellMv[i] > maxMv) maxMv = d.cellMv[i];
  }
  return true;
}

static void appendItem(char *buf, size_t cap, const char *text) {
  size_t used = strlen(buf);
  if (used + strlen(text) + 3 >= cap) return;
  if (used) {
    strcat(buf, ", ");
  }
  strcat(buf, text);
}

static void appendBits(char *buf, size_t cap, uint16_t value, const char *const *names,
                       uint8_t count) {
  for (uint8_t bit = 0; bit < count; bit++) {
    if ((value & (1u << bit)) && names[bit]) appendItem(buf, cap, names[bit]);
  }
}

// ============================================================== Modbus RTU

uint16_t modbusCrc(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

size_t buildModbusRead(uint8_t addr, uint16_t start, uint16_t count, uint8_t *out) {
  out[0] = addr;
  out[1] = 0x03;
  out[2] = start >> 8;
  out[3] = start & 0xFF;
  out[4] = count >> 8;
  out[5] = count & 0xFF;
  uint16_t crc = modbusCrc(out, 6);
  out[6] = crc & 0xFF;  // CRC младшим байтом вперёд
  out[7] = crc >> 8;
  return 8;
}

const uint8_t *findModbusResponse(const uint8_t *buf, size_t len, uint8_t addr, uint16_t count) {
  const size_t dataLen = (size_t)count * 2;
  const size_t frameLen = 3 + dataLen + 2;
  if (len < frameLen) return nullptr;
  for (size_t i = 0; i + frameLen <= len; i++) {
    if (buf[i] != addr || buf[i + 1] != 0x03 || buf[i + 2] != dataLen) continue;
    uint16_t crc = modbusCrc(buf + i, 3 + dataLen);
    if (buf[i + 3 + dataLen] == (crc & 0xFF) && buf[i + 4 + dataLen] == (crc >> 8)) {
      return buf + i + 3;
    }
  }
  return nullptr;
}

static const char *const MB_WARNINGS[16] = {
    "Cell overvoltage",          "Cell undervoltage",          "Pack overvoltage",
    "Pack undervoltage",         "Charge overcurrent",         "Discharge overcurrent",
    nullptr,                     nullptr,                      "Charge overtemperature",
    "Discharge overtemperature", "Charge undertemperature",    "Discharge undertemperature",
    "Environment overtemperature", "Environment undertemperature", "MOSFET overtemperature",
    "Low SOC"};

static const char *const MB_PROTECTIONS[16] = {
    "Cell overvoltage",          "Cell undervoltage",          "Pack overvoltage",
    "Pack undervoltage",         "Charge overcurrent",         "Discharge overcurrent",
    "Short circuit",             "Charger overvoltage",        "Charge overtemperature",
    "Discharge overtemperature", "Charge undertemperature",    "Discharge undertemperature",
    "MOSFET overtemperature",    "Environment overtemperature", "Environment undertemperature",
    nullptr /* бит 15 = "батарея полная", это не авария */};

static const char *const FAULTS[8] = {"Charge MOSFET fault", "Discharge MOSFET fault",
                                      "Temperature sensor fault", nullptr,
                                      "Cell fault", "Sampling (AFE) fault",
                                      nullptr, nullptr};

static float validTemp(int16_t raw) {
  // 0.1 °C; значения вне разумного диапазона — датчика нет
  return (raw < -500 || raw > 1500) ? NAN : raw * 0.1f;
}

void parseModbusRegisters(uint16_t start, uint16_t count, const uint8_t *p, BmsData &d) {
  for (uint16_t i = 0; i < count; i++) {
    const uint16_t reg = start + i;
    const uint16_t v = (uint16_t)(p[i * 2] << 8) | p[i * 2 + 1];
    const int16_t s = (int16_t)v;

    if (reg >= 15 && reg <= 30) {
      const uint8_t idx = reg - 15;
      d.cellMv[idx] = v;
      if (v > 0 && v != 0xFFFF && idx + 1 > d.cellCount) d.cellCount = idx + 1;
      continue;
    }
    if (reg >= 31 && reg <= 34) {
      const uint8_t idx = reg - 31;
      d.tempC[idx] = validTemp(s);
      if (!isnan(d.tempC[idx]) && idx + 1 > d.tempCount) d.tempCount = idx + 1;
      continue;
    }
    switch (reg) {
      case 0: d.current = s * 0.01f; d.hasAnalog = true; break;
      case 1: d.voltage = v * 0.01f; break;
      case 2: d.soc = v; break;
      case 3: d.soh = v; break;
      case 4: d.remainAh = v * 0.01f; break;
      case 5: d.fullAh = v * 0.01f; break;
      case 6: d.designAh = v * 0.01f; break;
      case 7: d.cycles = v; break;
      case 9: appendBits(d.warnings, sizeof(d.warnings), v, MB_WARNINGS, 16); break;
      case 10:
        appendBits(d.protections, sizeof(d.protections), v, MB_PROTECTIONS, 16);
        d.fullyCharged = (v & 0x8000) ? 1 : 0;
        break;
      case 11:
        appendBits(d.faults, sizeof(d.faults), v & 0xFF, FAULTS, 8);
        if (v & 0x4000) appendItem(d.faults, sizeof(d.faults), "Charger reversed");
        d.chargeFet = (v & 0x0400) ? 1 : 0;
        d.dischargeFet = (v & 0x0800) ? 1 : 0;
        d.currentLimit = (v & 0x1000) ? 1 : 0;
        d.heater = (v & 0x8000) ? 1 : 0;
        d.hasStatus = true;
        break;
      case 12: d.balanceMask = v; break;
      case 35: d.mosfetTempC = validTemp(s); break;
      case 36: d.envTempC = validTemp(s); break;
      default: break;
    }
  }
}

// ============================================================== PACE ASCII

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static uint16_t asciiChecksum(const char *p, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += (uint8_t)p[i];
  return (uint16_t)(~sum + 1);
}

static uint16_t lengthField(size_t lenid) {
  uint8_t sum = ((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF);
  uint8_t lchk = (uint8_t)(~sum + 1) & 0xF;
  return (uint16_t)((lchk << 12) | (lenid & 0xFFF));
}

size_t buildAsciiRequest(uint8_t addr, uint8_t cid2, const char *infoHex, char *out, size_t cap) {
  const size_t infoLen = infoHex ? strlen(infoHex) : 0;
  const size_t total = 18 + infoLen;
  if (cap < total + 1 || infoLen > 0xFFF) return 0;
  int n = snprintf(out, cap, "~%02X%02X46%02X%04X%s", ASCII_VERSION, addr, cid2,
                   lengthField(infoLen), infoHex ? infoHex : "");
  snprintf(out + n, cap - n, "%04X\r", asciiChecksum(out + 1, n - 1));
  return total;
}

AsciiResult findAsciiFrame(const char *buf, size_t len, const char **info, size_t *infoLen,
                           uint8_t *rtn) {
  const char *soi = (const char *)memchr(buf, '~', len);
  if (!soi) return AsciiResult::NoFrame;
  const size_t rest = len - (soi - buf);
  const char *eoi = (const char *)memchr(soi, '\r', rest);
  if (!eoi) return AsciiResult::NoFrame;
  const size_t frameLen = eoi - soi + 1;
  if (frameLen < 18) return AsciiResult::BadFormat;
  for (const char *c = soi + 1; c < eoi; c++) {
    if (hexVal(*c) < 0) return AsciiResult::BadFormat;
  }
  auto hex8 = [](const char *c) { return (uint8_t)((hexVal(c[0]) << 4) | hexVal(c[1])); };
  auto hex16 = [&](const char *c) { return (uint16_t)((hex8(c) << 8) | hex8(c + 2)); };

  if (hex16(eoi - 4) != asciiChecksum(soi + 1, frameLen - 6)) return AsciiResult::BadChecksum;
  if (rtn) *rtn = hex8(soi + 7);
  const size_t actualInfo = frameLen - 18;
  if ((hex16(soi + 9) & 0xFFF) != actualInfo) return AsciiResult::BadLength;
  if (hex8(soi + 7) != 0) return AsciiResult::ErrorCode;
  if (info) *info = soi + 13;
  if (infoLen) *infoLen = actualInfo;
  return AsciiResult::Ok;
}

namespace {
struct HexReader {
  const char *p;
  size_t len;
  size_t pos = 0;
  bool ok = true;

  HexReader(const char *data, size_t n) : p(data), len(n) {}

  uint8_t u8() {
    if (pos + 2 > len) { ok = false; return 0; }
    int hi = hexVal(p[pos]), lo = hexVal(p[pos + 1]);
    pos += 2;
    if (hi < 0 || lo < 0) { ok = false; return 0; }
    return (uint8_t)((hi << 4) | lo);
  }
  uint16_t u16() {
    uint16_t hi = u8();
    return (uint16_t)((hi << 8) | u8());
  }
};
}  // namespace

bool parseAsciiAnalog(const char *info, size_t infoLen, BmsData &d) {
  HexReader r(info, infoLen);
  r.u8();  // INFOFLAG
  const uint8_t packs = r.u8();
  const uint8_t cells = r.u8();
  if (!r.ok || packs == 0 || cells == 0 || cells > 32) return false;

  d.cellCount = 0;
  for (uint8_t i = 0; i < cells; i++) {
    uint16_t mv = r.u16();
    if (i < MAX_CELLS) {
      d.cellMv[i] = mv;
      d.cellCount = i + 1;
    }
  }

  const uint8_t temps = r.u8();
  if (!r.ok || temps > 16) return false;
  float t[16];
  for (uint8_t i = 0; i < temps; i++) t[i] = ((int)r.u16() - 2730) * 0.1f;  // 0.1 K -> °C

  // У 16S плат PACE шесть датчиков: 4 на ячейках, затем MOSFET и окружение
  uint8_t cellTemps = temps;
  if (temps == 6) {
    cellTemps = 4;
    d.mosfetTempC = t[4];
    d.envTempC = t[5];
  }
  d.tempCount = cellTemps > MAX_TEMPS ? MAX_TEMPS : cellTemps;
  for (uint8_t i = 0; i < d.tempCount; i++) d.tempC[i] = t[i];

  d.current = (int16_t)r.u16() * 0.01f;
  d.voltage = r.u16() * 0.001f;
  d.remainAh = r.u16() * 0.01f;
  r.u8();  // число "пользовательских" полей
  d.fullAh = r.u16() * 0.01f;
  d.cycles = r.u16();
  d.designAh = r.u16() * 0.01f;
  if (!r.ok) return false;

  if (d.fullAh > 0) {
    d.soc = d.remainAh / d.fullAh * 100.0f;
    if (d.soc > 100) d.soc = 100;
  }
  if (d.designAh > 0) {
    d.soh = d.fullAh / d.designAh * 100.0f;
    if (d.soh > 100) d.soh = 100;
  }
  d.hasAnalog = true;
  return true;
}

static const char *const PROTECT1[8] = {"Cell overvoltage", "Cell undervoltage",
                                        "Pack overvoltage", "Pack undervoltage",
                                        "Charge overcurrent", "Discharge overcurrent",
                                        "Short circuit", nullptr};
static const char *const PROTECT2[8] = {"Charge overtemperature", "Discharge overtemperature",
                                        "Charge undertemperature", "Discharge undertemperature",
                                        "MOSFET overtemperature", "Environment overtemperature",
                                        "Environment undertemperature",
                                        nullptr /* бит 7 = "полная" */};

static void warnState(char *buf, size_t cap, const char *what, int index, uint8_t state) {
  if (state == 0) return;
  const char *kind = state == 1 ? "low" : state == 2 ? "high" : "fault";
  char item[40];
  if (index > 0) snprintf(item, sizeof(item), "%s %d %s", what, index, kind);
  else snprintf(item, sizeof(item), "%s %s", what, kind);
  appendItem(buf, cap, item);
}

bool parseAsciiWarn(const char *info, size_t infoLen, BmsData &d) {
  HexReader r(info, infoLen);
  r.u8();  // INFOFLAG
  const uint8_t packs = r.u8();
  const uint8_t cells = r.u8();
  if (!r.ok || packs == 0 || cells > 32) return false;

  char warn[sizeof(d.warnings)] = "";
  for (uint8_t i = 0; i < cells; i++) warnState(warn, sizeof(warn), "Cell", i + 1, r.u8());
  const uint8_t temps = r.u8();
  if (!r.ok || temps > 16) return false;
  for (uint8_t i = 0; i < temps; i++) warnState(warn, sizeof(warn), "Temp", i + 1, r.u8());
  warnState(warn, sizeof(warn), "Charge current", 0, r.u8());
  warnState(warn, sizeof(warn), "Pack voltage", 0, r.u8());
  warnState(warn, sizeof(warn), "Discharge current", 0, r.u8());

  const uint8_t protect1 = r.u8();
  const uint8_t protect2 = r.u8();
  const uint8_t instruction = r.u8();
  r.u8();  // control state
  const uint8_t fault = r.u8();
  if (!r.ok) return false;

  strcpy(d.warnings, warn);
  d.protections[0] = 0;
  d.faults[0] = 0;
  appendBits(d.protections, sizeof(d.protections), protect1, PROTECT1, 8);
  appendBits(d.protections, sizeof(d.protections), protect2, PROTECT2, 8);
  appendBits(d.faults, sizeof(d.faults), fault, FAULTS, 8);
  d.fullyCharged = (protect2 & 0x80) ? 1 : 0;
  d.currentLimit = (instruction & 0x01) ? 1 : 0;
  d.chargeFet = (instruction & 0x02) ? 1 : 0;
  d.dischargeFet = (instruction & 0x04) ? 1 : 0;
  d.heater = (instruction & 0x80) ? 1 : 0;

  // Дальше идут два байта балансировки (ячейки 1-8 и 9-16) — есть не во всех прошивках
  const uint8_t bal1 = r.u8();
  const uint8_t bal2 = r.u8();
  if (r.ok) d.balanceMask = (uint16_t)((bal2 << 8) | bal1);

  d.hasStatus = true;
  return true;
}

}  // namespace pace
