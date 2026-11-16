// Протоколы PACE BMS (RS485): Modbus RTU и ASCII "версия 0x25".
// Чистый C++ без зависимостей от Arduino, чтобы код можно было тестировать на ПК.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace pace {

enum class Protocol : uint8_t { None = 0, Modbus = 1, Ascii = 2 };
const char *protocolName(Protocol p);

constexpr uint8_t MAX_CELLS = 16;
constexpr uint8_t MAX_TEMPS = 8;

struct BmsData {
  bool hasAnalog;
  bool hasStatus;

  float voltage;    // В
  float current;    // А, "+" заряд, "-" разряд
  float soc;        // %
  float soh;        // %
  float remainAh;
  float fullAh;
  float designAh;
  uint16_t cycles;

  uint8_t cellCount;
  uint16_t cellMv[MAX_CELLS];

  uint8_t tempCount;        // датчики на ячейках
  float tempC[MAX_TEMPS];
  float mosfetTempC;        // NAN если неизвестно
  float envTempC;           // NAN если неизвестно

  // -1 = неизвестно, 0 = нет, 1 = да
  int8_t chargeFet;
  int8_t dischargeFet;
  int8_t currentLimit;
  int8_t heater;
  int8_t fullyCharged;
  uint16_t balanceMask;     // бит N = балансируется ячейка N+1

  char warnings[200];
  char protections[200];
  char faults[120];
};

void reset(BmsData &d);
// Мин/макс напряжение ячеек, false если ячеек нет
bool cellStats(const BmsData &d, uint16_t &minMv, uint16_t &maxMv);

// ---------------------------------------------------------------- Modbus RTU
uint16_t modbusCrc(const uint8_t *data, size_t len);
// Запрос "Read Holding Registers" (0x03). out >= 8 байт. Возвращает длину (8).
size_t buildModbusRead(uint8_t addr, uint16_t start, uint16_t count, uint8_t *out);
// Ищет в буфере корректный ответ (адрес, 0x03, count*2 байт, CRC).
// Возвращает указатель на данные регистров или nullptr.
const uint8_t *findModbusResponse(const uint8_t *buf, size_t len, uint8_t addr, uint16_t count);
// Раскладывает блок регистров [start, start+count) в структуру.
void parseModbusRegisters(uint16_t start, uint16_t count, const uint8_t *payload, BmsData &d);

// ---------------------------------------------------------------- PACE ASCII
constexpr uint8_t ASCII_VERSION = 0x25;
constexpr uint8_t CID2_ANALOG = 0x42;
constexpr uint8_t CID2_WARN = 0x44;

enum class AsciiResult : uint8_t { Ok, NoFrame, BadFormat, BadChecksum, BadLength, ErrorCode };

// Собирает кадр "~VVAA46CCLLLL<info>SSSS\r". infoHex — уже в виде hex-текста.
// Возвращает длину кадра или 0, если не хватило места.
size_t buildAsciiRequest(uint8_t addr, uint8_t cid2, const char *infoHex, char *out, size_t cap);
// Ищет кадр в буфере и проверяет его. При Ok/ErrorCode заполняет rtn;
// при Ok — info/infoLen (hex-текст поля INFO внутри buf).
AsciiResult findAsciiFrame(const char *buf, size_t len, const char **info, size_t *infoLen,
                           uint8_t *rtn);
bool parseAsciiAnalog(const char *info, size_t infoLen, BmsData &d);
bool parseAsciiWarn(const char *info, size_t infoLen, BmsData &d);

}  // namespace pace
