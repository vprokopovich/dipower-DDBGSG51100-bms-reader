// Тесты разбора протоколов на ПК (без ESP32):
//   c++ -std=c++17 -I lib/PaceBms test/host/test_protocol.cpp lib/PaceBms/pace_protocol.cpp -o t && ./t
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>
#include <vector>

#include "pace_protocol.h"

static int failures = 0;
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);               \
      failures++;                                                          \
    }                                                                      \
  } while (0)
#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 0.006)

static void putReg(std::vector<uint8_t> &v, uint16_t value) {
  v.push_back(value >> 8);
  v.push_back(value & 0xFF);
}

static std::vector<uint8_t> modbusFrame(uint8_t addr, const std::vector<uint8_t> &payload) {
  std::vector<uint8_t> f = {addr, 0x03, (uint8_t)payload.size()};
  f.insert(f.end(), payload.begin(), payload.end());
  uint16_t crc = pace::modbusCrc(f.data(), f.size());
  f.push_back(crc & 0xFF);
  f.push_back(crc >> 8);
  return f;
}

static void testModbus() {
  // Эталонный кадр из спецификации Modbus: 01 03 00 00 00 08 44 0C
  uint8_t req[8];
  CHECK(pace::buildModbusRead(1, 0, 8, req) == 8);
  const uint8_t expected[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0C};
  CHECK(memcmp(req, expected, 8) == 0);

  pace::BmsData d;
  pace::reset(d);

  // Регистры 0..7: ток -12.34 А, 52.80 В, SOC 87, SOH 99, 87/100/100 А·ч, 42 цикла
  std::vector<uint8_t> p;
  putReg(p, (uint16_t)-1234);
  putReg(p, 5280);
  putReg(p, 87);
  putReg(p, 99);
  putReg(p, 8700);
  putReg(p, 10000);
  putReg(p, 10000);
  putReg(p, 42);
  std::vector<uint8_t> f = modbusFrame(1, p);

  // Мусор перед кадром (например, эхо запроса) не должен мешать
  std::vector<uint8_t> noisy(req, req + 8);
  noisy.insert(noisy.end(), f.begin(), f.end());
  const uint8_t *payload = pace::findModbusResponse(noisy.data(), noisy.size(), 1, 8);
  CHECK(payload != nullptr);
  CHECK(pace::findModbusResponse(noisy.data(), noisy.size(), 2, 8) == nullptr);  // чужой адрес
  CHECK(pace::findModbusResponse(f.data(), f.size() - 1, 1, 8) == nullptr);      // обрезан
  f[5] ^= 0x01;
  CHECK(pace::findModbusResponse(f.data(), f.size(), 1, 8) == nullptr);          // битый CRC
  if (!payload) return;

  pace::parseModbusRegisters(0, 8, payload, d);
  CHECK(d.hasAnalog);
  CHECK(NEAR(d.current, -12.34));
  CHECK(NEAR(d.voltage, 52.80));
  CHECK(NEAR(d.soc, 87) && NEAR(d.soh, 99));
  CHECK(NEAR(d.remainAh, 87.0) && NEAR(d.fullAh, 100.0) && NEAR(d.designAh, 100.0));
  CHECK(d.cycles == 42);

  // Регистры 15..36: 16 ячеек, 4 температуры, MOSFET, окружение (нет датчика)
  p.clear();
  for (int i = 0; i < 16; i++) putReg(p, 3300 + i);
  putReg(p, 215);
  putReg(p, 220);
  putReg(p, (uint16_t)-35);
  putReg(p, 0x7FFF);  // невалидное значение
  putReg(p, 301);
  putReg(p, 0x8000);  // невалидное значение
  pace::parseModbusRegisters(15, 22, p.data(), d);
  CHECK(d.cellCount == 16);
  CHECK(d.cellMv[0] == 3300 && d.cellMv[15] == 3315);
  uint16_t lo, hi;
  CHECK(pace::cellStats(d, lo, hi) && lo == 3300 && hi == 3315);
  CHECK(d.tempCount == 3);
  CHECK(NEAR(d.tempC[0], 21.5) && NEAR(d.tempC[2], -3.5));
  CHECK(NEAR(d.mosfetTempC, 30.1));
  CHECK(isnan(d.envTempC));

  // Регистры 9..12: предупреждение "низкий SOC", защита "Cell undervoltage",
  // состояние: разряд, оба MOSFET открыты; балансировка ячеек 1 и 16
  p.clear();
  putReg(p, 0x8000);
  putReg(p, 0x0002);
  putReg(p, 0x0E04);
  putReg(p, 0x8001);
  pace::parseModbusRegisters(9, 4, p.data(), d);
  CHECK(d.hasStatus);
  CHECK(strcmp(d.warnings, "Low SOC") == 0);
  CHECK(strcmp(d.protections, "Cell undervoltage") == 0);
  CHECK(strcmp(d.faults, "Temperature sensor fault") == 0);
  CHECK(d.chargeFet == 1 && d.dischargeFet == 1 && d.currentLimit == 0 && d.heater == 0);
  CHECK(d.fullyCharged == 0);
  CHECK(d.balanceMask == 0x8001);
}

static void testAscii() {
  // Известный эталонный запрос PACE: аналоговые данные батареи 01
  char req[32];
  size_t n = pace::buildAsciiRequest(1, pace::CID2_ANALOG, "01", req, sizeof(req));
  CHECK(n == 20);
  CHECK(strcmp(req, "~25014642E00201FD30\r") == 0);

  // Эхо запроса — формально корректный кадр, но с "кодом ошибки" 0x42
  const char *info = nullptr;
  size_t infoLen = 0;
  uint8_t rtn = 0;
  CHECK(pace::findAsciiFrame(req, n, &info, &infoLen, &rtn) == pace::AsciiResult::ErrorCode);
  CHECK(rtn == 0x42);

  // INFO из реального ответа PACE BMS (проект Tertiush/bmspace), первая батарея:
  // 13 ячеек, 6 температур, -1.30 А, 53.199 В, 96.11 из 100 А·ч, 311 циклов
  const char *analogInfo =
      "00020D100A0FF40FEA100E10040FF50FFD10010FFD0FE50FF5100A1001"
      "060BD20BD20BD40BD40BF80C1AFF7ECFCF258B0227100137271060";
  pace::BmsData d;
  pace::reset(d);
  CHECK(pace::parseAsciiAnalog(analogInfo, strlen(analogInfo), d));
  CHECK(d.cellCount == 13);
  CHECK(d.cellMv[0] == 4106 && d.cellMv[12] == 4097);
  CHECK(d.tempCount == 4);
  CHECK(NEAR(d.tempC[0], 29.6) && NEAR(d.tempC[3], 29.8));
  CHECK(NEAR(d.mosfetTempC, 33.4) && NEAR(d.envTempC, 36.8));
  CHECK(NEAR(d.current, -1.30));
  CHECK(NEAR(d.voltage, 53.199));
  CHECK(NEAR(d.remainAh, 96.11) && NEAR(d.fullAh, 100.0) && NEAR(d.designAh, 100.0));
  CHECK(d.cycles == 311);
  CHECK(NEAR(d.soc, 96.11) && NEAR(d.soh, 100));

  // Обрезанные данные не должны разбираться
  pace::BmsData bad;
  pace::reset(bad);
  CHECK(!pace::parseAsciiAnalog(analogInfo, 60, bad));
  CHECK(!bad.hasAnalog);

  // Полный кадр-ответ с нашей контрольной суммой: проверка поиска и валидации
  std::string body = std::string("25014600") + "0000" + analogInfo;
  // поле длины: LENID = длина INFO, LCHKSUM = дополнение суммы ниблов
  size_t lenid = strlen(analogInfo);
  unsigned sum = ((lenid >> 8) & 0xF) + ((lenid >> 4) & 0xF) + (lenid & 0xF);
  char lengthHex[5];
  snprintf(lengthHex, sizeof(lengthHex), "%04X",
           (unsigned)((((~sum + 1) & 0xF) << 12) | lenid));
  body.replace(8, 4, lengthHex);
  unsigned chk = 0;
  for (char c : body) chk += (unsigned char)c;
  char chkHex[5];
  snprintf(chkHex, sizeof(chkHex), "%04X", (unsigned)((~chk + 1) & 0xFFFF));
  std::string frame = "\xFF\x00junk~" + body + chkHex + "\r";
  frame = std::string("xx") + "~" + body + chkHex + "\r";

  CHECK(pace::findAsciiFrame(frame.data(), frame.size(), &info, &infoLen, &rtn) ==
        pace::AsciiResult::Ok);
  CHECK(rtn == 0 && infoLen == lenid && memcmp(info, analogInfo, lenid) == 0);
  CHECK(pace::findAsciiFrame(frame.data(), frame.size() - 1, &info, &infoLen, &rtn) ==
        pace::AsciiResult::NoFrame);
  std::string corrupted = frame;
  corrupted[20] = corrupted[20] == '0' ? '1' : '0';
  CHECK(pace::findAsciiFrame(corrupted.data(), corrupted.size(), &info, &infoLen, &rtn) ==
        pace::AsciiResult::BadChecksum);

  // Состояния/аварии: реальный ответ (bmspace) — всё в норме, CFET и DFET открыты
  const char *warnInfo =
      "000210000000000000000000000000000000000600000000000000000000000E000000000000";
  CHECK(pace::parseAsciiWarn(warnInfo, strlen(warnInfo), d));
  CHECK(d.hasStatus);
  CHECK(d.warnings[0] == 0 && d.protections[0] == 0 && d.faults[0] == 0);
  CHECK(d.chargeFet == 1 && d.dischargeFet == 1 && d.currentLimit == 0);
  CHECK(d.fullyCharged == 0);

  // То же, но: ячейка 3 ниже порога, защита от переразряда ячейки, "полная", DFET закрыт
  std::string w = warnInfo;
  w.replace(6 + 2 * 2, 2, "01");   // ячейка 3
  size_t tail = 6 + 16 * 2 + 2 + 6 * 2 + 3 * 2;  // начало protect1
  w.replace(tail, 2, "02");        // protect1: cell undervoltage
  w.replace(tail + 2, 2, "80");    // protect2: fully
  w.replace(tail + 4, 2, "02");    // instruction: только CFET
  CHECK(pace::parseAsciiWarn(w.data(), w.size(), d));
  CHECK(strcmp(d.warnings, "Cell 3 low") == 0);
  CHECK(strcmp(d.protections, "Cell undervoltage") == 0);
  CHECK(d.fullyCharged == 1 && d.chargeFet == 1 && d.dischargeFet == 0);
}

int main() {
  testModbus();
  testAscii();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all protocol tests passed\n");
  return 0;
}
