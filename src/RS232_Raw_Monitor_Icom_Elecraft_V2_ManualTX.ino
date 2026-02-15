/*
  =============================================================================
  RS232_Raw_Monitor_Icom_Elecraft_V2_ManualTX  (Full Sketch)
  =============================================================================
  Zweck:
    - "Terminal/Monitor" zwischen USB-Serial (PC) und RS232 (MAX3232)
    - Unterstützt:
        * Icom CI-V (binär, HEX-Eingabe, AutoDecode für FREQ/MODE/ACK/NAK)
        * Elecraft KX2/KX3 (ASCII CAT, ';' terminiert, Decode für FA/FB/MD/RX/TX)
    - Keine Automatismen beim Senden:
        * Es wird NUR bei Enter gesendet
        * Du entscheidest TXFMT (HEX oder ASCII)
        * Bei ASCII wird NICHT automatisch ';' angehängt (außer bei Komfort-Befehlen)

  Bedienung (PC -> Controller über USB Serial Monitor, 115200 8N1):
    - HELP / ?            : komplette Hilfe
    - HELP ICOM           : nur Icom QuickRef
    - HELP ELE            : nur Elecraft QuickRef
    - INFO                : aktuelle Einstellungen
    - RESTART             : RS232 neu starten
    - PROTO ICOM|ELE      : Protokollwahl (Decode / Komfortbefehle)
    - TXFMT HEX|ASCII     : Sendeformat für RAW-Zeilen
    - BAUD <n>            : RS232 Baudrate (muss zum Gerät passen)
    - PINS <rx> <tx>      : UART Pins setzen
    - INVERT <rx> <tx>    : Invert RX/TX (0/1) (ESP32)
    - ASCII <0|1>         : ASCII-Spalte im Dump
    - TS <0|1>            : Timestamp [ms]
    - GAP <ms>            : RX Flush nach Lücke
    - CIV <0|1>           : Icom AutoDecode
    - ECHO <0|1>          : Icom Echo-Filter

  RAW Senden:
    - TXFMT HEX  : FE FE 94 E0 03 FD
    - TXFMT ASCII: FA;  MD2;  RX;  TX;  (du tippst ';' selbst)

  Hardware:
    - ESP32(-S3) HardwareSerial UART2 auf Pins RX=9 TX=10 (an MAX3232)
=============================================================================
*/

#include <Arduino.h>
#include <HardwareSerial.h>

// ---------------- Defaults / Config ----------------
namespace Defaults {
  static constexpr uint32_t USB_BAUD   = 115200;

  static constexpr int      UART_NUM   = 2;
  static constexpr uint32_t RS232_BAUD = 9600;
  static constexpr uint32_t RS232_CFG  = SERIAL_8N1;

  static constexpr int RS232_RX_PIN = 9;
  static constexpr int RS232_TX_PIN = 10;

  static constexpr bool RX_INVERT = false;
  static constexpr bool TX_INVERT = false;

  static constexpr bool SHOW_ASCII  = true;
  static constexpr bool SHOW_TS_MS  = true;

  // RX grouping:
  // - Icom: flush on gap (ms)
  // - Elecraft: flush on ';'
  static constexpr uint32_t RX_GAP_FLUSH_MS = 25;

  // CI-V features
  static constexpr bool CIV_AUTODECODE  = true;
  static constexpr bool CIV_ECHO_FILTER = true;

  static constexpr size_t RX_BUF_MAX   = 512;
  static constexpr size_t TX_MAX_BYTES = 256;
}

enum class Proto : uint8_t { ICOM=1, ELE=2 };

// IMPORTANT: Arduino has a macro HEX. Don't use "HEX" as enum member name.
enum class TxFormat : uint8_t { FMT_HEX=0, FMT_ASCII=1 };

struct Config {
  uint32_t usbBaud      = Defaults::USB_BAUD;

  int      uartNum      = Defaults::UART_NUM;
  uint32_t rsBaud       = Defaults::RS232_BAUD;
  uint32_t rsCfg        = Defaults::RS232_CFG;

  int      rxPin        = Defaults::RS232_RX_PIN;
  int      txPin        = Defaults::RS232_TX_PIN;

  bool     rxInvert     = Defaults::RX_INVERT;
  bool     txInvert     = Defaults::TX_INVERT;

  bool     showAscii    = Defaults::SHOW_ASCII;
  bool     showTsMs     = Defaults::SHOW_TS_MS;

  uint32_t rxGapFlushMs = Defaults::RX_GAP_FLUSH_MS;

  bool     civDecode    = Defaults::CIV_AUTODECODE;
  bool     civEchoFilter= Defaults::CIV_ECHO_FILTER;

  Proto    proto        = Proto::ICOM;
  TxFormat txFormat     = TxFormat::FMT_HEX;
};

static Config g;
static HardwareSerial* gRs = nullptr;

// ---------------- Utilities ----------------
static inline bool isHexChar(char c) {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}
static inline int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return -1;
}
static inline void printHex2(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX); // Arduino macro HEX is OK here
}
static void printHexArray(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    printHex2(data[i]);
    if (i + 1 < len) Serial.print(' ');
  }
}
static void printAsciiArray(const uint8_t* data, size_t len) {
  Serial.print("  |");
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (b >= 32 && b <= 126) Serial.write((char)b);
    else Serial.print('.');
  }
  Serial.print("|");
}

static bool parseHexString(const String& in, uint8_t* out, size_t outMax, size_t& outLen, String& err) {
  err = "";
  outLen = 0;
  String s = in; s.trim();
  if (s.length() == 0) { err = "Leere Eingabe."; return false; }

  String compact; compact.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    if (!isHexChar(c)) { err = "Ungueltiges Zeichen im Hex-String."; return false; }
    compact += c;
  }
  if ((compact.length() % 2) != 0) { err = "Ungerade Anzahl Hex-Zeichen (Byte fehlt)."; return false; }

  size_t bytes = compact.length() / 2;
  if (bytes > outMax) { err = String("Zu viele Bytes (max. ") + outMax + ")."; return false; }

  for (size_t i = 0; i < bytes; i++) {
    int hi = hexNibble(compact[i * 2]);
    int lo = hexNibble(compact[i * 2 + 1]);
    if (hi < 0 || lo < 0) { err = "Ungueltiges Hex-Byte."; return false; }
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  outLen = bytes;
  return true;
}

// ---------------- CI-V (Icom) helpers ----------------
static uint8_t gLastTx[Defaults::TX_MAX_BYTES];
static size_t  gLastTxLen = 0;
static bool    gLastTxIsCiv = false;

static bool isCivFrame(const uint8_t* data, size_t len) {
  return (len >= 6 && data[0] == 0xFE && data[1] == 0xFE && data[len - 1] == 0xFD);
}
static bool findCivFrame(const uint8_t* data, size_t len, size_t& start, size_t& end) {
  for (size_t i = 0; i + 1 < len; i++) {
    if (data[i] == 0xFE && data[i + 1] == 0xFE) {
      for (size_t j = i + 2; j < len; j++) {
        if (data[j] == 0xFD) { start = i; end = j; return true; }
      }
    }
  }
  return false;
}
static int bcdToInt2(uint8_t b) {
  int lo = b & 0x0F;
  int hi = (b >> 4) & 0x0F;
  if (lo > 9 || hi > 9) return -1;
  return hi * 10 + lo;
}
static bool decodeCivFreqHz(const uint8_t* b5, uint32_t& hzOut) {
  // 5 bytes BCD, LSB first, 10 digits total
  char digits[11];
  int pos = 0;
  for (int i = 4; i >= 0; i--) {
    int v = bcdToInt2(b5[i]);
    if (v < 0) return false;
    digits[pos++] = char('0' + (v / 10));
    digits[pos++] = char('0' + (v % 10));
  }
  digits[10] = '\0';

  uint64_t hz = 0;
  for (int i = 0; i < 10; i++) hz = hz * 10 + (uint64_t)(digits[i] - '0');
  if (hz > 0xFFFFFFFFULL) return false;
  hzOut = (uint32_t)hz;
  return true;
}
static const char* civModeName(uint8_t mode) {
  switch (mode) {
    case 0x00: return "LSB";
    case 0x01: return "USB";
    case 0x02: return "AM";
    case 0x03: return "CW";
    case 0x04: return "RTTY";
    case 0x05: return "FM";
    case 0x06: return "WFM";
    case 0x07: return "CW-R";
    case 0x08: return "RTTY-R";
    default:   return "UNKNOWN";
  }
}
static void printHzAsMHz(uint32_t hz) {
  uint32_t mhz = hz / 1000000UL;
  uint32_t rem = hz % 1000000UL;
  Serial.print(mhz);
  Serial.print('.');
  char buf[7];
  snprintf(buf, sizeof(buf), "%06lu", (unsigned long)rem);
  Serial.print(buf[0]); Serial.print(buf[1]); Serial.print(buf[2]);
  Serial.print('.');
  Serial.print(buf[3]); Serial.print(buf[4]); Serial.print(buf[5]);
  Serial.print(" MHz");
}
static bool isEchoOfLastTx(const uint8_t* civ, size_t civLen) {
  if (!gLastTxIsCiv || gLastTxLen != civLen) return false;
  for (size_t i = 0; i < civLen; i++) if (gLastTx[i] != civ[i]) return false;
  return true;
}
static void decodeAndPrintCivFrame(const uint8_t* civ, size_t civLen, const char* dirTag, bool isEcho=false) {
  if (civLen < 6) return;

  uint8_t to   = civ[2];
  uint8_t from = civ[3];
  uint8_t cmd  = civ[4];

  Serial.print("    ");
  Serial.print(dirTag);
  Serial.print(" CI-V: to="); printHex2(to);
  Serial.print(" from="); printHex2(from);
  Serial.print(" cmd="); printHex2(cmd);
  if (isEcho) Serial.print(" (ECHO)");

  if (cmd == 0xFA && civLen == 6) { Serial.print("  -> ACK (FA)"); Serial.println(); return; }
  if (cmd == 0xFB && civLen == 6) { Serial.print("  -> NAK (FB)"); Serial.println(); return; }

  // SET FREQ (00)
  if (cmd == 0x00 && civLen >= 11) {
    uint32_t hz;
    if (decodeCivFreqHz(&civ[5], hz)) {
      Serial.print("  -> SET FREQ: ");
      printHzAsMHz(hz);
    } else Serial.print("  -> SET FREQ: (decode error)");
    Serial.println();
    return;
  }
  // READ FREQ (03) response
  if (cmd == 0x03 && civLen >= 11) {
    uint32_t hz;
    if (decodeCivFreqHz(&civ[5], hz)) {
      Serial.print("  -> FREQ: ");
      printHzAsMHz(hz);
    } else Serial.print("  -> FREQ: (decode error)");
    Serial.println();
    return;
  }
  // READ MODE (04) response
  if (cmd == 0x04 && civLen >= 7) {
    uint8_t mode = civ[5];
    uint8_t filt = (civLen >= 8) ? civ[6] : 0x00;
    Serial.print("  -> MODE: ");
    Serial.print(civModeName(mode));
    Serial.print("  FILT="); printHex2(filt);
    Serial.println();
    return;
  }
  // SET MODE (01 <mode> <filter>) (typical 706)
  if (cmd == 0x01 && civLen >= 7) {
    uint8_t mode = civ[5];
    uint8_t filt = (civLen >= 8) ? civ[6] : 0x00;
    Serial.print("  -> SET MODE: ");
    Serial.print(civModeName(mode));
    Serial.print("  FILT="); printHex2(filt);
    Serial.println();
    return;
  }

  Serial.println();
}

// ---------------- Elecraft helpers ----------------
static const char* eleModeNameFromCode(int n) {
  switch (n) {
    case 1: return "LSB";
    case 2: return "USB";
    case 3: return "CW";
    case 4: return "FM";
    case 5: return "AM";
    case 6: return "DATA";
    case 7: return "CW-REV";
    case 9: return "DATA-REV";
    default: return "UNKNOWN";
  }
}
static void decodeAndPrintElecraft(const String& msg, const char* dirTag) {
  String m = msg;
  m.trim();
  if (!m.endsWith(";")) m += ';';

  String up = m; up.toUpperCase();

  Serial.print("    ");
  Serial.print(dirTag);
  Serial.print(" ELE: ");
  Serial.print(m);

  if (up.startsWith("FA") || up.startsWith("FB")) {
    bool isA = up.startsWith("FA");
    String digits = up.substring(2);
    digits.replace(";", "");
    bool allDigits = digits.length() > 0;
    for (size_t i = 0; i < (size_t)digits.length(); i++) {
      if (digits[i] < '0' || digits[i] > '9') { allDigits = false; break; }
    }
    if (allDigits && digits.length() >= 5) {
      uint64_t hz = 0;
      for (size_t i = 0; i < (size_t)digits.length(); i++) hz = hz * 10ULL + (uint64_t)(digits[i] - '0');
      Serial.print("  -> ");
      Serial.print(isA ? "VFO A FREQ: " : "VFO B FREQ: ");
      double mhz = (double)hz / 1000000.0;
      Serial.print(mhz, 6);
      Serial.print(" MHz");
    } else {
      Serial.print("  -> ");
      Serial.print(isA ? "VFO A FREQ (GET/SET)" : "VFO B FREQ (GET/SET)");
    }
    Serial.println();
    return;
  }

  if (up.startsWith("MD")) {
    String rest = up.substring(2);
    rest.replace(";", "");
    int n = rest.toInt();
    Serial.print("  -> MODE: ");
    Serial.print(eleModeNameFromCode(n));
    Serial.println();
    return;
  }

  if (up == "TX;") { Serial.println("  -> PTT: ON (TX)"); return; }
  if (up == "RX;") { Serial.println("  -> PTT: OFF (RX)"); return; }

  Serial.println();
}

// ---------------- HELP / QUICKREF ----------------
static const char* QUICKREF_ICOM =
"IC-706 / IC-7300 CI-V Quick Reference\n"
"------------------------------------\n"
"FRAME: FE FE <TO> <FROM> <CMD> [DATA...] FD\n"
"Typical: <FROM>=E0\n"
"\n"
"READ FREQ (CMD 03):\n"
"  7300 (94): FE FE 94 E0 03 FD\n"
"  706  (58): FE FE 58 E0 03 FD\n"
"\n"
"READ MODE (CMD 04):\n"
"  7300 (94): FE FE 94 E0 04 FD\n"
"  706  (58): FE FE 58 E0 04 FD\n"
"\n"
"SET MODE (CMD 01 <mode> <filt>):\n"
"  mode: 00 LSB, 01 USB, 02 AM, 03 CW, 04 RTTY, 05 FM\n"
"  Example USB (706): FE FE 58 E0 01 01 01 FD\n"
"\n"
"SET FREQ (CMD 05 or CMD 00 depends on rig; many accept CMD 05 for set VFO):\n"
"  Example 14.330.000 MHz -> BCD: 00 00 33 14 00\n"
"  FE FE 94 E0 05 00 00 33 14 00 FD\n"
"\n"
"ACK/NAK:\n"
"  FA = ACK, FB = NAK\n";

static const char* QUICKREF_ELE =
"Elecraft KX2/KX3 CAT (ASCII) Quick Reference\n"
"--------------------------------------------\n"
"Commands are ASCII and end with ';'\n"
"\n"
"FREQ:\n"
"  FA;              -> GET VFO A frequency\n"
"  FA00014060000;   -> SET VFO A to 14.060.000 MHz\n"
"  FB; / FB...;     -> VFO B\n"
"\n"
"MODE:\n"
"  MD;              -> GET mode\n"
"  MD2;             -> SET mode USB\n"
"  Mode codes: 1=LSB 2=USB 3=CW 4=FM 5=AM 6=DATA 7=CW-REV 9=DATA-REV\n"
"\n"
"PTT:\n"
"  TX;              -> PTT ON\n"
"  RX;              -> PTT OFF\n";

static void showHelpIcom() {
  Serial.println();
  Serial.println("=== HELP ICOM (CI-V Quick Reference) ===");
  Serial.println();
  Serial.print(QUICKREF_ICOM);
  Serial.println();
}
static void showHelpEle() {
  Serial.println();
  Serial.println("=== HELP ELE (Elecraft CAT Quick Reference) ===");
  Serial.println();
  Serial.print(QUICKREF_ELE);
  Serial.println();
}

static void showHelp() {
  Serial.println();
  Serial.println("=== RS232_Raw_Monitor_Icom_Elecraft_V2_ManualTX - HELP ===");
  Serial.println();
  Serial.println("Konfiguration:");
  Serial.println("  HELP / ?              -> diese Hilfe");
  Serial.println("  HELP ICOM             -> nur CI-V QuickRef anzeigen");
  Serial.println("  HELP ELE              -> nur Elecraft QuickRef anzeigen");
  Serial.println("  INFO                  -> aktuelle Einstellungen");
  Serial.println("  RESTART               -> RS232 neu starten");
  Serial.println("  PROTO ICOM|ELE        -> Protokollwahl");
  Serial.println("  TXFMT HEX|ASCII       -> RAW-Eingabeformat");
  Serial.println("  BAUD <n>              -> RS232 Baudrate");
  Serial.println("  PINS <rx> <tx>        -> UART Pins setzen");
  Serial.println("  INVERT <rx> <tx>      -> Invert RX/TX (0/1)");
  Serial.println("  ASCII <0|1>           -> ASCII-Spalte anzeigen");
  Serial.println("  TS <0|1>              -> Timestamp [ms] anzeigen");
  Serial.println("  GAP <ms>              -> RX Flush nach Luecke");
  Serial.println("  CIV <0|1>             -> Icom AutoDecode");
  Serial.println("  ECHO <0|1>            -> Icom Echo-Filter");
  Serial.println();
  Serial.println("RAW senden:");
  Serial.println("  (TXFMT=HEX)   FE FE 94 E0 03 FD");
  Serial.println("  (TXFMT=ASCII) FA;   MD2;   RX;   TX;");
  Serial.println();
  Serial.println("Komfort:");
  Serial.println("  FREQ?   / MODE?   / PTT 0|1   (Elecraft nutzt ASCII, Icom nur Lesen via Default-Addr)");
  Serial.println();
}

// ---------------- RS232 init ----------------
static void beginRs232() {
  if (gRs) { gRs->end(); delay(50); }

  static HardwareSerial rs1(1);
  static HardwareSerial rs2(2);
  gRs = (g.uartNum == 1) ? &rs1 : &rs2;

  gRs->begin(g.rsBaud, g.rsCfg, g.rxPin, g.txPin);
#if defined(ARDUINO_ARCH_ESP32)
  gRs->setRxInvert(g.rxInvert);
  gRs->setTxInvert(g.txInvert);
#endif
}

static void printInfo() {
  Serial.println();
  Serial.println("=== INFO ===");
  Serial.print("UART_NUM: "); Serial.println(g.uartNum);
  Serial.print("RS232 Baud: "); Serial.println(g.rsBaud);
  Serial.print("Pins RX/TX: "); Serial.print(g.rxPin); Serial.print(" / "); Serial.println(g.txPin);
  Serial.print("Invert RX/TX: "); Serial.print(g.rxInvert ? "1" : "0"); Serial.print(" / "); Serial.println(g.txInvert ? "1" : "0");
  Serial.print("SHOW_ASCII: "); Serial.println(g.showAscii ? "1" : "0");
  Serial.print("SHOW_TS_MS: "); Serial.println(g.showTsMs ? "1" : "0");
  Serial.print("RX_GAP_FLUSH_MS: "); Serial.println(g.rxGapFlushMs);
  Serial.print("PROTO: "); Serial.println(g.proto == Proto::ICOM ? "ICOM" : "ELE");
  Serial.print("TXFMT: "); Serial.println(g.txFormat == TxFormat::FMT_HEX ? "HEX" : "ASCII");
  Serial.print("CIV_AUTODECODE: "); Serial.println(g.civDecode ? "1" : "0");
  Serial.print("CIV_ECHO_FILTER: "); Serial.println(g.civEchoFilter ? "1" : "0");
  Serial.println("============");
  Serial.println();
}

// ---------------- RX grouping / printing ----------------
static uint8_t  gRxBuf[Defaults::RX_BUF_MAX];
static size_t   gRxLen = 0;
static uint32_t gLastRxMs = 0;

static void flushRxLine() {
  if (gRxLen == 0) return;

  // Echo filter (CI-V only) – optional: remove leading echo if buffer starts with last TX
  bool removedEchoAtStart = false;
  size_t startOffset = 0;

  if (g.civDecode && g.civEchoFilter && gLastTxIsCiv && gRxLen >= gLastTxLen && gLastTxLen > 0) {
    bool match = true;
    for (size_t i = 0; i < gLastTxLen; i++) {
      if (gRxBuf[i] != gLastTx[i]) { match = false; break; }
    }
    if (match && gRxLen > gLastTxLen) {
      startOffset = gLastTxLen;
      removedEchoAtStart = true;
    }
  }

  if (g.showTsMs) { Serial.print('['); Serial.print(millis()); Serial.print("] "); }
  Serial.print("RX: ");
  printHexArray(gRxBuf + startOffset, gRxLen - startOffset);
  if (g.showAscii) printAsciiArray(gRxBuf + startOffset, gRxLen - startOffset);
  Serial.println();

  if (removedEchoAtStart) Serial.println("    (Echo-Filter: TX-Echo am Anfang entfernt)");

  const uint8_t* p = gRxBuf + startOffset;
  size_t len = gRxLen - startOffset;

  bool doIcom = (g.proto == Proto::ICOM);
  bool doEle  = (g.proto == Proto::ELE);

  if (doIcom && g.civDecode) {
    size_t s, e;
    size_t cursor = 0;
    while (cursor < len) {
      if (!findCivFrame(p + cursor, len - cursor, s, e)) break;
      s += cursor; e += cursor;
      const uint8_t* civ = p + s;
      size_t civLen = (e - s + 1);

      bool echo = (g.civEchoFilter && isEchoOfLastTx(civ, civLen));
      decodeAndPrintCivFrame(civ, civLen, "RX", echo);
      cursor = e + 1;
    }
  }

  if (doEle) {
    String accum;
    accum.reserve(len + 4);
    for (size_t i = 0; i < len; i++) {
      char c = (char)p[i];
      if (c == '\r' || c == '\n') continue;
      accum += c;
      if (c == ';') {
        decodeAndPrintElecraft(accum, "RX");
        accum = "";
      }
    }
  }

  gRxLen = 0;
}

static void onRxByte(uint8_t b) {
  uint32_t now = millis();

  // Gap flush
  if (gRxLen > 0 && (now - gLastRxMs) > g.rxGapFlushMs) flushRxLine();
  gLastRxMs = now;

  if (gRxLen < sizeof(gRxBuf)) {
    gRxBuf[gRxLen++] = b;
  } else {
    flushRxLine();
    gRxBuf[gRxLen++] = b;
  }

  // Elecraft: flush immediately on ';'
  if (b == ';' && g.proto == Proto::ELE) {
    flushRxLine();
  }
}

// ---------------- TX / Command handling ----------------
static String gLine;

static void txPrintCommon(const uint8_t* bytes, size_t len, const String* asciiLineOrNull) {
  if (g.showTsMs) { Serial.print('['); Serial.print(millis()); Serial.print("] "); }
  Serial.print("TX: ");
  printHexArray(bytes, len);
  if (g.showAscii) printAsciiArray(bytes, len);
  Serial.println();

  if (asciiLineOrNull) {
    decodeAndPrintElecraft(*asciiLineOrNull, "TX");
  } else if (g.civDecode) {
    size_t s0, e0;
    size_t cursor = 0;
    while (cursor < len) {
      if (!findCivFrame(bytes + cursor, len - cursor, s0, e0)) break;
      s0 += cursor; e0 += cursor;
      decodeAndPrintCivFrame(bytes + s0, (e0 - s0 + 1), "TX", false);
      cursor = e0 + 1;
    }
  }
}

static void sendHexLine(const String& s) {
  uint8_t bytes[Defaults::TX_MAX_BYTES];
  size_t len = 0;
  String err;
  if (!parseHexString(s, bytes, sizeof(bytes), len, err)) {
    Serial.print("ERR: "); Serial.println(err);
    return;
  }

  gLastTxLen = min(len, sizeof(gLastTx));
  memcpy(gLastTx, bytes, gLastTxLen);
  gLastTxIsCiv = isCivFrame(bytes, len);

  if (gRs) {
    gRs->write(bytes, len);
    gRs->flush();
  }
  txPrintCommon(bytes, len, nullptr);
}

static void sendAsciiRaw(String s) {
  s.trim();
  if (gRs) {
    gRs->print(s);
    gRs->flush();
  }

  uint8_t bytes[Defaults::TX_MAX_BYTES];
  size_t len = min((size_t)s.length(), sizeof(bytes));
  for (size_t i = 0; i < len; i++) bytes[i] = (uint8_t)s[i];

  gLastTxLen = min(len, sizeof(gLastTx));
  memcpy(gLastTx, bytes, gLastTxLen);
  gLastTxIsCiv = isCivFrame(bytes, len);

  txPrintCommon(bytes, len, &s);
}

static bool startsWithCmd(const String& s, const char* cmd) {
  String u = s; u.trim(); u.toUpperCase();
  String c = String(cmd); c.toUpperCase();
  return u == c || u.startsWith(c + " ");
}

static bool parseProtoStrict(const String& arg, Proto& out) {
  String a = arg; a.trim(); a.toUpperCase();
  if (a == "ICOM") { out = Proto::ICOM; return true; }
  if (a == "ELE" || a == "ELECRAFT") { out = Proto::ELE; return true; }
  return false;
}

static void sendElecraftCat(String s) {
  s.trim();
  if (!s.endsWith(";")) s += ';';
  sendAsciiRaw(s);
}

static void handleCommand(String s) {
  s.trim();
  if (s.length() == 0) return;

  String up = s; up.toUpperCase();

  // HELP / INFO
  if (up == "HELP" || up == "?") { showHelp(); return; }
  if (up == "HELP ICOM") { showHelpIcom(); return; }
  if (up == "HELP ELE" || up == "HELP ELECRAFT") { showHelpEle(); return; }
  if (up == "INFO") { printInfo(); return; }
  if (up == "RESTART") { beginRs232(); Serial.println("OK: RS232 restarted."); return; }

  // PROTO (strict)
  if (startsWithCmd(up, "PROTO")) {
    String arg = s.substring(5);
    Proto p;
    if (!parseProtoStrict(arg, p)) {
      Serial.println("ERR: PROTO ICOM oder ELE");
      return;
    }
    g.proto = p;
    Serial.print("PROTO="); Serial.println(g.proto == Proto::ICOM ? "ICOM" : "ELE");
    return;
  }

  // TXFMT (strict)
  if (startsWithCmd(up, "TXFMT") || startsWithCmd(up, "FORMAT")) {
    String arg = s;
    int sp = arg.indexOf(' ');
    arg = (sp >= 0) ? arg.substring(sp + 1) : "";
    arg.trim(); arg.toUpperCase();
    if (arg == "HEX") g.txFormat = TxFormat::FMT_HEX;
    else if (arg == "ASCII") g.txFormat = TxFormat::FMT_ASCII;
    else { Serial.println("ERR: TXFMT HEX oder ASCII"); return; }
    Serial.print("TXFMT="); Serial.println(g.txFormat == TxFormat::FMT_HEX ? "HEX" : "ASCII");
    return;
  }

  // BAUD
  if (startsWithCmd(up, "BAUD")) {
    long b = s.substring(4).toInt();
    if (b < 300 || b > 1000000) { Serial.println("ERR: BAUD ungueltig."); return; }
    g.rsBaud = (uint32_t)b;
    beginRs232();
    Serial.print("OK: RS232 Baud="); Serial.println(g.rsBaud);
    return;
  }

  // PINS rx tx
  if (startsWithCmd(up, "PINS")) {
    String rest = s.substring(4); rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("ERR: PINS <rx> <tx>"); return; }
    int rx = rest.substring(0, sp).toInt();
    int tx = rest.substring(sp + 1).toInt();
    g.rxPin = rx; g.txPin = tx;
    beginRs232();
    Serial.print("OK: PINS RX/TX="); Serial.print(g.rxPin); Serial.print("/"); Serial.println(g.txPin);
    return;
  }

  // INVERT rx tx
  if (startsWithCmd(up, "INVERT")) {
    String rest = s.substring(6); rest.trim();
    int sp = rest.indexOf(' ');
    if (sp < 0) { Serial.println("ERR: INVERT <rx0|1> <tx0|1>"); return; }
    int rx = rest.substring(0, sp).toInt();
    int tx = rest.substring(sp + 1).toInt();
    g.rxInvert = (rx != 0);
    g.txInvert = (tx != 0);
    beginRs232();
    Serial.print("OK: INVERT RX/TX="); Serial.print(g.rxInvert ? "1" : "0"); Serial.print("/"); Serial.println(g.txInvert ? "1" : "0");
    return;
  }

  if (startsWithCmd(up, "ASCII")) {
    g.showAscii = (s.substring(5).toInt() != 0);
    Serial.print("SHOW_ASCII="); Serial.println(g.showAscii ? "1" : "0");
    return;
  }
  if (startsWithCmd(up, "TS")) {
    g.showTsMs = (s.substring(2).toInt() != 0);
    Serial.print("SHOW_TS_MS="); Serial.println(g.showTsMs ? "1" : "0");
    return;
  }
  if (startsWithCmd(up, "GAP")) {
    long v = s.substring(3).toInt();
    if (v < 1 || v > 10000) { Serial.println("ERR: GAP ungueltig."); return; }
    g.rxGapFlushMs = (uint32_t)v;
    Serial.print("RX_GAP_FLUSH_MS="); Serial.println(g.rxGapFlushMs);
    return;
  }
  if (startsWithCmd(up, "CIV")) {
    g.civDecode = (s.substring(3).toInt() != 0);
    Serial.print("CIV_AUTODECODE="); Serial.println(g.civDecode ? "1" : "0");
    return;
  }
  if (startsWithCmd(up, "ECHO")) {
    g.civEchoFilter = (s.substring(4).toInt() != 0);
    Serial.print("CIV_ECHO_FILTER="); Serial.println(g.civEchoFilter ? "1" : "0");
    return;
  }

  // Komfort-Kommandos (minimal)
  if (up == "FREQ?") {
    if (g.proto == Proto::ELE) { sendElecraftCat("FA;"); return; }
    if (g.proto == Proto::ICOM) { sendHexLine("FE FE 94 E0 03 FD"); return; } // default to 7300
  }
  if (up == "MODE?") {
    if (g.proto == Proto::ELE) { sendElecraftCat("MD;"); return; }
    if (g.proto == Proto::ICOM) { sendHexLine("FE FE 94 E0 04 FD"); return; } // default to 7300
  }
  if (startsWithCmd(up, "PTT")) {
    String arg = s.substring(3); arg.trim();
    int v = arg.toInt();
    if (g.proto == Proto::ELE) { sendElecraftCat(v ? "TX;" : "RX;"); return; }
    Serial.println("HINWEIS: ICOM PTT bitte per CI-V HEX (modell/menu abhaengig).");
    return;
  }

  // RAW senden (keine Automatismen)
  if (g.txFormat == TxFormat::FMT_HEX) sendHexLine(s);
  else sendAsciiRaw(s);
}

void setup() {
  Serial.begin(g.usbBaud);
  delay(200);
  Serial.println();
  Serial.println("RS232_Raw_Monitor_Icom_Elecraft_V2_ManualTX");
  Serial.println("Tippe HELP fuer Befehle.");
  beginRs232();
  showHelp();
}

void loop() {
  if (gRs) {
    while (gRs->available() > 0) onRxByte((uint8_t)gRs->read());
    if (gRxLen > 0 && (millis() - gLastRxMs) > g.rxGapFlushMs) flushRxLine();
  }

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = gLine;
      gLine = "";
      line.trim();
      if (line.length() > 0) handleCommand(line);
      continue;
    }
    gLine += c;
    if (gLine.length() > 512) {
      Serial.println("ERR: Eingabe zu lang.");
      gLine = "";
    }
  }
}
