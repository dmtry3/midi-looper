// ESP32 MIDI Looper Engine v60
// - индикация нот: диапазон 24..103 (без 2 нижних и 2 верхних октав)
// - индикация показывает и ноты, которые играет сам лупер
#include <HardwareSerial.h>
#include <MIDI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== USB MIDI (только ESP32-S2 / ESP32-S3) =====
// 1 = включить. В Arduino IDE: Tools -> USB Mode -> "USB-OTG (TinyUSB)".
#define USB_MIDI_ENABLED 1
#if USB_MIDI_ENABLED
  #if !(CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3)
    #error "USB MIDI requires ESP32-S2 or ESP32-S3 (native USB)"
  #endif
  #include "USB.h"
  #include "USBMIDI.h"
  USBMIDI usbMidi;
#endif

#define LOOPER_PIN 2
#define LED_PIN 1
#define MIDICHANNEL 12
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PRESET_COUNT 5

// Окно защиты от эха (мс): входящие ноты той же высоты сразу после того, как
// лупер сам их отправил, считаются эхом и игнорируются.
#define ECHO_GUARD_MS 40

// Ножная педаль/кнопка на LOOPER_PIN: короткое нажатие = REC/OVERDUB/PLAY.
// Поставьте 0, чтобы полностью отключить её действия.
#define PEDAL_ENABLED 1

// --- Энкодер ---
#define ENCODER_CLK 5
#define ENCODER_DT  4
#define ENCODER_SW  3

// --- OLED (I2C, SSD1315 совместим с SSD1306) ---
#define OLED_SDA 9
#define OLED_SCL 8
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define OLED_ENABLED 1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledReady = false;

// --- Индикация нот (низ экрана) ---
// Показываем диапазон нот VIZ_NOTE_LO..VIZ_NOTE_HI (без 2 нижних и 2 верхних октав),
// растянутый на всю ширину экрана. Две "дорожки":
//   vizIn  — входящие ноты (сплошные полоски),
//   vizOut — ноты, которые играет лупер (шахматные "серые" полоски).
#define VIZ_NOTE_LO 24                                   // C1 (ноты 0..23 не показываем)
#define VIZ_NOTE_HI 103                                  // G7 (ноты 104..127 не показываем)
#define VIZ_NOTE_COUNT (VIZ_NOTE_HI - VIZ_NOTE_LO + 1)   // 80
#define VIZ_BASE_Y 63            // нижняя строка: "линейка" (точки на каждой ноте До)
#define VIZ_MAX_H 6              // максимальная высота полоски (пиксели)
const unsigned long VIZ_DECAY_MS = 350;
const unsigned long VIZ_IN_HOLD_TIMEOUT_MS = 15000;
const unsigned long VIZ_OUT_HOLD_TIMEOUT_MS = 70000;

struct VizSlot {
  volatile uint8_t vel;
  volatile bool held;
  volatile unsigned long onT;
  volatile unsigned long offT;
};
VizSlot vizIn[128];
VizSlot vizOut[128];

void vizOn(VizSlot *a, byte p, byte vel) {
  if (p > 127) return;
  a[p].vel = vel;
  a[p].onT = millis();
  a[p].held = true;
}
void vizOff(VizSlot *a, byte p) {
  if (p > 127) return;
  if (a[p].held) {
    a[p].held = false;
    a[p].offT = millis();
  }
}
void vizClear(VizSlot *a) {
  unsigned long now = millis();
  for (int p = 0; p < 128; p++) {
    if (a[p].held) { a[p].held = false; a[p].offT = now; }
  }
}

// --- 4 кнопки возле экрана (сверху вниз: пины 13, 12, 11, 10) ---
// 0 (верх, пин 13): RESET
// 1 (пин 12): PRESETS — короткое = выбор слота для LOAD, удержание 2 с = выбор слота для SAVE
// 2 (пин 11): UNDO
// 3 (низ, пин 10): MENU
#define MENU_BTN_COUNT 4
const int menuButtonPins[MENU_BTN_COUNT] = {13, 12, 11, 10};
unsigned long menuBtnDebounce[MENU_BTN_COUNT] = {0, 0, 0, 0};
bool menuBtnState[MENU_BTN_COUNT] = {HIGH, HIGH, HIGH, HIGH};
const unsigned long PRESET_LONG_PRESS_MS = 2000;

HardwareSerial MIDI_Serial(2);
MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_Serial, MIDI);
const int pwmFreq = 5000;
const int pwmRes = 8;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
unsigned long lastBleUpdate = 0;
const unsigned long BLE_INTERVAL = 30;

// --- ПРЕСЕТЫ (save/load лупов через LittleFS) ---
bool needSendPresetStatus = false;
unsigned long connectStatusTime = 0;
bool visualDumpActive = false;
int visualDumpIndex = 0;
unsigned long lastVisualDumpTime = 0;
unsigned long visualDumpLastPosSend = 0;
int visualDumpFinishStage = 0;
unsigned long visualDumpFinishTime = 0;
const unsigned long VISUAL_DUMP_EVENT_INTERVAL = 5;
const unsigned long VISUAL_DUMP_POS_INTERVAL   = 80;
int pendingLoadSlot = -1;

// Кэш наличия пресетов (чтобы не читать флеш при каждой перерисовке экрана)
bool presetCache[PRESET_COUNT + 1] = {false};

// --- КВАНТИЗАЦИЯ ---
enum QuantTarget { QUANT_START = 0, QUANT_END = 1, QUANT_BOTH = 2 };
int quantGrid = 0;
int quantStrength = 100;
int quantTarget = QUANT_BOTH;
unsigned long loopBeatDuration = 545;
const int MIN_NOTE_LEN_MS = 20;

// --- ФИЛЬТР КАНАЛОВ ЗАПИСИ ---
// Бит 0 = канал 1 ... бит 15 = канал 16. 1 = канал записывается.
#define DEFAULT_CH_MASK 0xFFFF
uint16_t recChannelMask = DEFAULT_CH_MASK;

// --- ИСТОЧНИК MIDI-КЛОКА (DIN или USB): берём тот, что активен первым ---
enum ClockSrc : uint8_t { CLK_DIN = 0, CLK_USB = 1 };
uint8_t activeClockSrc = CLK_DIN;
unsigned long lastClockSrcTime = 0;
bool acceptClockSource(uint8_t src) {
  unsigned long now = millis();
  if (src == activeClockSrc || now - lastClockSrcTime > 500) {
    activeClockSrc = src;
    lastClockSrcTime = now;
    return true;
  }
  return false;
}

uint16_t activeNoteStartTs[17][128];

// Зажатые сейчас клавиши (velocity Note On, 0 = не зажата).
// Нужны, чтобы нота, нажатая чуть раньше старта записи, попала в луп с первой доли.
byte heldVel[17][128];

// ДВИЖОК ЛУПЕРА
enum LooperState { STATE_IDLE, STATE_RECORDING, STATE_PLAYING, STATE_OVERDUB };
LooperState looperState = STATE_IDLE;
volatile bool pendingStart = false;
volatile bool pendingStop = false;
volatile unsigned long lastBeatTime = 0;
volatile unsigned long lastClockTime = 0;
volatile int beatInBar = 0;
volatile int tickInBeat = 0;
unsigned long beatDuration = 545;
struct MidiEvent {
  uint16_t timestamp;
  byte type, d1, d2, ch, layer;
  bool played;
};
MidiEvent events[5000];
int eventCount = 0;
byte currentLayer = 0;
byte ccMaxLayer[17][128];
unsigned long loopLen = 0, recStart = 0, playStart = 0;
unsigned long lastSentNoteTime[16][128], lastSentCCTime[16][128], lastSentATTime[16], ccTakeover[17][128];

// --- UI ---
enum UiMode { UI_NORMAL = 0, UI_MENU_LIST, UI_MENU_EDIT, UI_MENU_CHANNELS, UI_SAVE_SLOT, UI_LOAD_SLOT };
UiMode uiMode = UI_NORMAL;
int pendingSlot = 1;
int selectedPresetSlot = 1;
unsigned long uiModeStart = 0;
const unsigned long UI_TIMEOUT = 12000;

// Пункты меню
enum MenuItem {
  MI_GRID = 0, MI_STRENGTH, MI_TARGET, MI_APPLY, MI_CHANNELS,
  MI_TRANSPOSE, MI_HALF, MI_DOUBLE, MI_DEL_PRESET,
  MI_COUNT
};
const char* const menuLabels[MI_COUNT] = {
  "Grid", "Strength", "Target", "Apply quantize", "Channels",
  "Transpose", "Half loop", "Double loop", "Delete preset"
};
int menuCursor = 0;
int editValue = 0;

// Подменю Channels: 0..15 = каналы 1..16, 16 = ALL ON, 17 = ALL OFF
#define CH_CURSOR_COUNT 18
int chCursor = 0;

// Кнопка пресетов (короткое/длинное нажатие)
bool presetBtnDown = false;
bool presetLongFired = false;
unsigned long presetBtnStart = 0;

// Короткие сообщения на экране (char-буфер: безопасно читать из задачи OLED)
char toastBuf[28] = "";
unsigned long toastUntil = 0;
void toast(const String &s) {
  strncpy(toastBuf, s.c_str(), sizeof(toastBuf) - 1);
  toastBuf[sizeof(toastBuf) - 1] = 0;
  toastUntil = millis() + 2000;
}
void uiTouch() { uiModeStart = millis(); }

void sendBleNotify(String text) {
  if (deviceConnected && pCharacteristic) {
    pCharacteristic->setValue(text.c_str());
    pCharacteristic->notify();
  }
}

// Отправка MIDI-сообщения на DIN и (если включено) на USB.
// Всё, что уходит отсюда — это ноты, которые играет лупер: показываем их на индикации.
void sendMidi(byte type, byte d1, byte d2, byte ch) {
  if (type == 0x90 && d2 > 0) vizOn(vizOut, d1, d2);
  else if (type == 0x80 || type == 0x90) vizOff(vizOut, d1);

  MIDI.send((midi::MidiType)type, d1, d2, ch);
#if USB_MIDI_ENABLED
  midiEventPacket_t pkt;
  pkt.header = (uint8_t)((type >> 4) & 0x0F);                 // cable 0 | CIN
  pkt.byte1  = (uint8_t)((type & 0xF0) | ((ch - 1) & 0x0F));
  pkt.byte2  = d1;
  pkt.byte3  = d2;
  usbMidi.writePacket(&pkt);
#endif
}

String stateName() {
  switch (looperState) {
    case STATE_RECORDING: return "RECORDING";
    case STATE_PLAYING:   return "PLAYING";
    case STATE_OVERDUB:   return "OVERDUB";
    default:              return "IDLE";
  }
}
void sendBeatInfo() {
  sendBleNotify("/beatinfo " + String(loopBeatDuration));
}

// ================== ФИЛЬТР КАНАЛОВ ==================
bool channelEnabled(byte ch) {
  return ch >= 1 && ch <= 16 && (recChannelMask & (1u << (ch - 1)));
}
int enabledChannelCount() {
  int n = 0;
  for (int i = 0; i < 16; i++) if (recChannelMask & (1u << i)) n++;
  return n;
}
void saveChannelMask() {
  File f = LittleFS.open("/chmask.bin", "w");
  if (!f) return;
  uint32_t magic = 0x43484D4BUL;
  uint16_t m = recChannelMask;
  f.write((uint8_t*)&magic, sizeof(magic));
  f.write((uint8_t*)&m, sizeof(m));
  f.close();
}
void loadChannelMask() {
  if (!LittleFS.exists("/chmask.bin")) return;
  File f = LittleFS.open("/chmask.bin", "r");
  if (!f) return;
  uint32_t magic = 0;
  uint16_t m = 0;
  f.read((uint8_t*)&magic, sizeof(magic));
  f.read((uint8_t*)&m, sizeof(m));
  f.close();
  if (magic == 0x43484D4BUL) recChannelMask = m;
}
void sendChannelStatus() {
  sendBleNotify("/chmask " + String(recChannelMask));
}

void resetActiveNoteTracking() {
  for (int i = 0; i < 17; i++) for (int j = 0; j < 128; j++) activeNoteStartTs[i][j] = 0xFFFF;
}
void resetLooper() {
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) {
      sendMidi(0x80, events[i].d1, 0, events[i].ch);
    }
  }
  vizClear(vizOut);

  looperState = STATE_IDLE;
  eventCount = 0;
  currentLayer = 0;
  loopLen = 0;
  recStart = 0;
  playStart = 0;
  pendingStart = false;
  pendingStop = false;
  pendingLoadSlot = -1;
  resetActiveNoteTracking();

  sendBleNotify("/reset 1");
}
void halfLoop() {
  if (loopLen < 500 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) sendMidi(0x80, events[i].d1, 0, events[i].ch);
  }
  loopLen /= 2;
  int newCount = 0;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].timestamp < loopLen) {
      events[newCount++] = events[i];
    }
  }
  eventCount = newCount;
}
void doubleLoop() {
  if (loopLen == 0 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  if (loopLen * 2 > 65000) return;
  int origCount = eventCount;
  unsigned long oldLen = loopLen;
  for (int i = 0; i < origCount; i++) {
    if (eventCount < 5000) {
      MidiEvent newEv = events[i];
      newEv.timestamp = (uint16_t)(events[i].timestamp + oldLen);
      newEv.played = false;
      events[eventCount++] = newEv;
    }
  }
  loopLen *= 2;
}

// --- ТРАНСПОНИРОВАНИЕ ЛУПА (полутоны) ---
void transposeLoop(int semitones) {
  if (loopLen == 0 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  if (semitones == 0) return;

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) sendMidi(0x80, events[i].d1, 0, events[i].ch);
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90 || events[i].type == 0x80) {
      int newPitch = (int)events[i].d1 + semitones;
      if (newPitch < 0) newPitch = 0;
      if (newPitch > 127) newPitch = 127;
      events[i].d1 = (byte)newPitch;
    }
  }

  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;
}

// --- UNDO: удаляет последний записанный слой овердаба ---
void undoLastLayer() {
  if (loopLen == 0 || currentLayer == 0) {
    sendBleNotify("/undo_err 1"); // нечего отменять
    return;
  }
  byte layerToRemove = currentLayer;

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90 && events[i].layer == layerToRemove) {
      sendMidi(0x80, events[i].d1, 0, events[i].ch);
    }
  }

  int newCount = 0;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].layer != layerToRemove) {
      events[newCount++] = events[i];
    }
  }
  eventCount = newCount;
  currentLayer--;

  for (int i = 0; i < 17; i++) for (int j = 0; j < 128; j++) ccMaxLayer[i][j] = 0;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0xB0 && events[i].layer > ccMaxLayer[events[i].ch][events[i].d1]) {
      ccMaxLayer[events[i].ch][events[i].d1] = events[i].layer;
    }
  }

  if (looperState == STATE_OVERDUB) {
    looperState = STATE_PLAYING;
  }

  sendBleNotify("/undo_ok 1");

  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;
}

// Длительность шага текущей сетки квантования в мс (0, если квант выключен)
float quantUnitMs() {
  if (quantGrid == 0 || loopBeatDuration == 0) return 0.0f;
  switch (quantGrid) {
    case 1: return (float)loopBeatDuration / 8.0f;  // Q32
    case 2: return (float)loopBeatDuration / 4.0f;  // Q16
    case 3: return (float)loopBeatDuration / 2.0f;  // Q8
    case 4: return (float)loopBeatDuration;         // Q4
    default: return 0.0f;
  }
}

uint16_t quantizeTimestamp(uint16_t rawTs, unsigned long loopLenLocal) {
  if (quantGrid == 0 || quantStrength <= 0 || loopBeatDuration == 0) return rawTs;
  float unit = quantUnitMs();
  if (unit < 1.0f) unit = 1.0f;

  float nearest = round((float)rawTs / unit) * unit;
  float blended = (float)rawTs + (nearest - (float)rawTs) * ((float)quantStrength / 100.0f);
  long result = (long)(blended + 0.5f);

  if (result < 0) result = 0;
  if (loopLenLocal > 0 && result >= (long)loopLenLocal) result = (long)loopLenLocal - 1;
  if (result > 65000) result = 65000;
  return (uint16_t)result;
}

uint16_t protectNoteEnd(uint16_t startTs, uint16_t endTs, unsigned long loopLenLocal, bool isDuringRecording) {
  float unit = quantUnitMs();
  long minLen = (unit >= 1.0f) ? (long)(unit + 0.5f) : MIN_NOTE_LEN_MS;
  if (minLen < MIN_NOTE_LEN_MS) minLen = MIN_NOTE_LEN_MS;

  long diff = (long)endTs - (long)startTs;
  long wrapThreshold = (long)(unit * 3.0f);
  if (wrapThreshold < 200) wrapThreshold = 200;
  bool looksLikeWrap = (diff < -wrapThreshold);
  if (looksLikeWrap || diff >= minLen) return endTs;

  long minEnd = (long)startTs + minLen;
  long bound = isDuringRecording ? 65000L : (loopLenLocal > 0 ? (long)loopLenLocal - 1 : 65000L);
  if (minEnd > bound) minEnd = bound;
  if (minEnd < 0) minEnd = 0;
  return (uint16_t)minEnd;
}

void applyQuantizeToLoop() {
  if (eventCount <= 0 || loopLen == 0) {
    sendBleNotify("/qapply_err 1");
    return;
  }
  if (quantGrid == 0 || quantStrength <= 0) {
    sendBleNotify("/qapply_err 2");
    return;
  }

  resetActiveNoteTracking();

  for (int i = 0; i < eventCount; i++) {
    bool isNoteStart = (events[i].type == 0x90 && events[i].d2 > 0);
    bool isNoteEnd   = (events[i].type == 0x80 || (events[i].type == 0x90 && events[i].d2 == 0));
    byte ch = events[i].ch;
    byte pitch = events[i].d1;
    uint16_t rawTs = events[i].timestamp;

    bool doQuant = false;
    if (events[i].type == 0xB0) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteStart) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteEnd) {
      doQuant = (quantTarget == QUANT_END || quantTarget == QUANT_BOTH);
    }

    uint16_t newTs = rawTs;
    if (doQuant) newTs = quantizeTimestamp(rawTs, loopLen);

    if (isNoteStart) {
      activeNoteStartTs[ch][pitch] = newTs;
    } else if (isNoteEnd) {
      uint16_t startTs = activeNoteStartTs[ch][pitch];
      if (startTs != 0xFFFF) {
        newTs = protectNoteEnd(startTs, newTs, loopLen, false);
        activeNoteStartTs[ch][pitch] = 0xFFFF;
      }
    }

    events[i].timestamp = newTs;
  }

  sendBleNotify("/qapply_ok 1");

  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;
}

String presetPath(int slot) {
  return "/preset" + String(slot) + ".bin";
}
void refreshPresetCache() {
  for (int i = 1; i <= PRESET_COUNT; i++) presetCache[i] = LittleFS.exists(presetPath(i));
}
bool presetExists(int slot) {
  return slot >= 1 && slot <= PRESET_COUNT && presetCache[slot];
}
void sendPresetStatus() {
  String s = "/presets ";
  for (int i = 1; i <= PRESET_COUNT; i++) s += presetExists(i) ? "1" : "0";
  sendBleNotify(s);
}
void sendQuantStatus() {
  sendBleNotify("/quant " + String(quantGrid) + " " + String(quantStrength) + " " + String(quantTarget));
}
bool savePreset(int slot) {
  if (eventCount <= 0 || loopLen == 0 || looperState == STATE_IDLE || looperState == STATE_RECORDING) {
    sendBleNotify("/save_err " + String(slot));
    return false;
  }
  File f = LittleFS.open(presetPath(slot), "w");
  if (!f) { sendBleNotify("/save_err " + String(slot)); return false; }
  uint32_t magic = 0x4C505345UL;
  uint32_t ll = (uint32_t)loopLen;
  int32_t ec = (int32_t)eventCount;
  f.write((uint8_t*)&magic, sizeof(magic));
  f.write((uint8_t*)&ll, sizeof(ll));
  f.write((uint8_t*)&ec, sizeof(ec));
  f.write((uint8_t*)events, sizeof(MidiEvent) * eventCount);
  f.close();
  presetCache[slot] = true;
  sendBleNotify("/save_ok " + String(slot));
  sendPresetStatus();
  return true;
}
bool loadPreset(int slot) {
  if (!presetExists(slot)) { sendBleNotify("/load_err " + String(slot)); return false; }
  File f = LittleFS.open(presetPath(slot), "r");
  if (!f) { sendBleNotify("/load_err " + String(slot)); return false; }

  uint32_t magic = 0, ll = 0;
  int32_t ec = 0;
  f.read((uint8_t*)&magic, sizeof(magic));
  f.read((uint8_t*)&ll, sizeof(ll));
  f.read((uint8_t*)&ec, sizeof(ec));

  if (magic != 0x4C505345UL || ec <= 0 || ec > 5000) {
    f.close();
    sendBleNotify("/load_err " + String(slot));
    return false;
  }

  unsigned long now = millis();
  unsigned long prevPos = 0;
  bool hadPrevLoop = (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB));
  if (hadPrevLoop) {
    prevPos = (now - playStart) % loopLen;
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) sendMidi(0x80, events[i].d1, 0, events[i].ch);
  }

  size_t toRead = sizeof(MidiEvent) * (size_t)ec;
  size_t got = f.read((uint8_t*)events, toRead);
  f.close();
  if (got != toRead) { sendBleNotify("/load_err " + String(slot)); return false; }

  loopLen = ll;
  eventCount = ec;
  currentLayer = 0;
  for (int i = 0; i < 17; i++) for (int j = 0; j < 128; j++) ccMaxLayer[i][j] = 0;
  for (int i = 0; i < eventCount; i++) {
    events[i].played = false;
    if (events[i].layer > currentLayer) currentLayer = events[i].layer;
    if (events[i].type == 0xB0 && events[i].layer > ccMaxLayer[events[i].ch][events[i].d1]) {
      ccMaxLayer[events[i].ch][events[i].d1] = events[i].layer;
    }
  }

  resetActiveNoteTracking();
  pendingStart = false;
  pendingStop = false;
  looperState = STATE_PLAYING;

  unsigned long newPos = (loopLen > 0) ? (prevPos % loopLen) : 0;
  playStart = now - newPos;

  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;

  sendBleNotify("/load_ok " + String(slot));
  return true;
}
// Загрузка всегда отложенная:
//  - если луп играет — в конце текущего круга (обрабатывается в playback());
//  - если лупа нет / идёт запись — на первой доле такта (см. triggerQuantizedAction()).
void requestLoadPreset(int slot) {
  pendingLoadSlot = slot;
  sendBleNotify("/load_pending " + String(slot));
}
void deletePreset(int slot) {
  String path = presetPath(slot);
  if (LittleFS.exists(path)) LittleFS.remove(path);
  if (slot >= 1 && slot <= PRESET_COUNT) presetCache[slot] = false;
  sendBleNotify("/del_ok " + String(slot));
  sendPresetStatus();
}

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      std::string rxValue = pChar->getValue().c_str();
      if (rxValue.length() > 0) {
        String cmd = String(rxValue.c_str());
        cmd.trim();
        if (cmd == "/half") {
          halfLoop();
        }
        else if (cmd == "/double") {
          doubleLoop();
        }
        else if (cmd == "/reset") {
          resetLooper();
        }
        else if (cmd == "/undo") {
          undoLastLayer();
        }
        else if (cmd == "/rec") {
          if (looperState == STATE_IDLE) pendingStart = true;
        }
        else if (cmd == "/stop") {
          if (looperState == STATE_RECORDING) pendingStop = true;
        }
        else if (cmd == "/overdub") {
          if (looperState == STATE_PLAYING) {
            currentLayer++;
            looperState = STATE_OVERDUB;
            sendBleNotify("/state OVERDUB");
          }
        }
        else if (cmd == "/play") {
          if (looperState == STATE_OVERDUB) {
            looperState = STATE_PLAYING;
            sendBleNotify("/state PLAYING");
          }
        }
        else if (cmd.startsWith("/transpose ")) {
          int v = cmd.substring(11).toInt();
          transposeLoop(v);
        }
        else if (cmd.startsWith("/save ")) {
          int slot = cmd.substring(6).toInt();
          if (slot >= 1 && slot <= PRESET_COUNT) savePreset(slot);
        }
        else if (cmd.startsWith("/load ")) {
          int slot = cmd.substring(6).toInt();
          if (slot >= 1 && slot <= PRESET_COUNT) requestLoadPreset(slot);
        }
        else if (cmd.startsWith("/delp ")) {
          int slot = cmd.substring(6).toInt();
          if (slot >= 1 && slot <= PRESET_COUNT) deletePreset(slot);
        }
        else if (cmd == "/getpresets") {
          sendPresetStatus();
        }
        else if (cmd.startsWith("/qgrid ")) {
          int v = cmd.substring(7).toInt();
          if (v >= 0 && v <= 4) { quantGrid = v; sendQuantStatus(); }
        }
        else if (cmd.startsWith("/qstrength ")) {
          int v = cmd.substring(11).toInt();
          if (v < 0) v = 0;
          if (v > 100) v = 100;
          quantStrength = v;
          sendQuantStatus();
        }
        else if (cmd.startsWith("/qtarget ")) {
          int v = cmd.substring(9).toInt();
          if (v >= 0 && v <= 2) { quantTarget = v; sendQuantStatus(); }
        }
        else if (cmd == "/getquant") {
          sendQuantStatus();
        }
        else if (cmd == "/qapply") {
          applyQuantizeToLoop();
        }
        else if (cmd.startsWith("/chmask ")) {
          long v = cmd.substring(8).toInt();
          if (v < 0) v = 0;
          if (v > 65535) v = 65535;
          recChannelMask = (uint16_t)v;
          saveChannelMask();
          sendChannelStatus();
        }
        else if (cmd == "/getch") {
          sendChannelStatus();
        }
      }
    }
};
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      needSendPresetStatus = true;
      connectStatusTime = millis();
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      visualDumpActive = false;
      visualDumpFinishStage = 0;
      BLEDevice::startAdvertising();
    }
};

// --- Нажатие педали: REC-OVERDUB-PLAY ---
void looperShortPress() {
  if (looperState == STATE_IDLE) pendingStart = true;
  else if (looperState == STATE_RECORDING) pendingStop = true;
  else if (looperState == STATE_PLAYING) {
    currentLayer++;
    looperState = STATE_OVERDUB;
    sendBleNotify("/state OVERDUB");
  }
  else if (looperState == STATE_OVERDUB) {
    looperState = STATE_PLAYING;
    sendBleNotify("/state PLAYING");
  }
}

// Ноты, зажатые к моменту старта записи, пишем с первой доли (timestamp 0),
// как будто они сыграны точно в начало такта.
void captureHeldNotesAtStart() {
  for (int ch = 1; ch <= 16; ch++) {
    if (!channelEnabled((byte)ch)) continue;
    for (int n = 0; n < 128; n++) {
      byte vel = heldVel[ch][n];
      if (vel == 0) continue;
      if (eventCount >= 5000) return;
      events[eventCount++] = {(uint16_t)0, (byte)0x90, (byte)n, vel, (byte)ch, currentLayer, false};
      activeNoteStartTs[ch][n] = 0;
      sendBleNotify("/rec_noteon " + String(n) + " 0");
    }
  }
}

void triggerQuantizedAction() {
  unsigned long now = millis();

  // Отложенная загрузка пресета, когда луп не играет (IDLE/RECORDING):
  // стартуем ровно с первой доли такта. Загрузка имеет приоритет над armed REC/STOP.
  if (pendingLoadSlot >= 1 && looperState != STATE_PLAYING && looperState != STATE_OVERDUB) {
    int slot = pendingLoadSlot;
    pendingLoadSlot = -1;
    pendingStart = false;
    pendingStop = false;
    loadPreset(slot);
    return;
  }

  if (pendingStart) {
    eventCount = 0; currentLayer = 0; recStart = now;
    looperState = STATE_RECORDING;
    for(int i=0; i<17; i++) for(int j=0; j<128; j++) ccMaxLayer[i][j] = 0;
    resetActiveNoteTracking();
    pendingStart = false;
    sendBleNotify("/state RECORDING");
    captureHeldNotesAtStart();
  }
  else if (pendingStop) {
    loopLen = now - recStart; playStart = now;
    looperState = STATE_PLAYING;
    pendingStop = false;
    sendBeatInfo();
    sendBleNotify("/state PLAYING");
  }
}
void handleStart() { tickInBeat = 0; beatInBar = 0; lastBeatTime = millis(); }
void handleClock() {
  unsigned long now = millis();
  lastClockTime = now;

  if (tickInBeat == 0) {
    beatDuration = now - lastBeatTime;
    lastBeatTime = now;
    loopBeatDuration = beatDuration;
    if (beatInBar == 0) triggerQuantizedAction();
  }
  tickInBeat++;
  if (tickInBeat >= 24) { tickInBeat = 0; beatInBar = (beatInBar + 1) % 4; }
}

void recordEvent(byte t, byte d1, byte d2, byte ch) {
  if (ch < 1 || ch > 16) return;

  bool isNoteStart = (t == 0x90 && d2 > 0);
  bool isNoteEnd   = (t == 0x80 || (t == 0x90 && d2 == 0));
  bool hasOpenNote = (activeNoteStartTs[ch][d1] != 0xFFFF);

  // Индикация входящих нот (показываем всё, что приходит, независимо от фильтров)
  if (isNoteStart) vizOn(vizIn, d1, d2);
  else if (isNoteEnd) vizOff(vizIn, d1);
  else if (t == 0xB0 && (d1 == 123 || d1 == 120)) vizClear(vizIn);

  // Учёт зажатых клавиш: отпускание и All Notes Off сбрасываем всегда
  if (isNoteEnd) heldVel[ch][d1] = 0;
  else if (t == 0xB0 && (d1 == 123 || d1 == 120)) {
    for (int p = 0; p < 128; p++) heldVel[ch][p] = 0;
  }

  // Фильтр каналов. Note Off для уже начатой ноты пропускаем всегда,
  // чтобы при отключении канала не остались "залипшие" ноты в лупе.
  if (!channelEnabled(ch)) {
    if (!(isNoteEnd && hasOpenNote)) return;
  }

  unsigned long now = millis();

  // Защита от эха: игнорируем ноты, пришедшие сразу после того, как лупер сам
  // отправил такую же. Note Off для реально открытой ноты пропускаем всегда.
  if ((isNoteStart || isNoteEnd) && !(isNoteEnd && hasOpenNote)) {
    if (now - lastSentNoteTime[ch-1][d1] < ECHO_GUARD_MS) return;
  }

  if (isNoteStart) heldVel[ch][d1] = d2;

  if (t == 0xB0) { if (now - lastSentCCTime[ch-1][d1] < 15) return; ccTakeover[ch][d1] = now; ccMaxLayer[ch][d1] = currentLayer; }
  if (t == 0xD0 || t == 0xA0) if (now - lastSentATTime[ch-1] < 15) return;

  bool isRecOrOver = (looperState == STATE_RECORDING || looperState == STATE_OVERDUB);

  // Что записываем в луп:
  //  - в REC/OVERDUB всё, кроме "сиротских" Note Off (без записанного Note On);
  //  - в PLAYING только Note Off для ноты, начатой в овердабе (чтобы не залипала).
  bool recordIt = false;
  if (isRecOrOver) recordIt = !(isNoteEnd && !hasOpenNote);
  else if (looperState == STATE_PLAYING && loopLen > 0 && isNoteEnd && hasOpenNote) recordIt = true;

  uint16_t finalTs = 0;
  if (isRecOrOver || recordIt) {
    unsigned long rawTs = (looperState == STATE_RECORDING) ? (now - recStart) : ((now - playStart) % loopLen);
    finalTs = (uint16_t)(rawTs > 65000 ? 65000 : rawTs);

    bool doQuant = false;
    if (t == 0xB0) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteStart) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteEnd) {
      doQuant = (quantTarget == QUANT_END || quantTarget == QUANT_BOTH);
    }
    if (doQuant) {
      unsigned long loopLenForClamp = (looperState == STATE_RECORDING) ? 0 : loopLen;
      finalTs = quantizeTimestamp(finalTs, loopLenForClamp);
    }

    if (isNoteStart) {
      activeNoteStartTs[ch][d1] = finalTs;
    } else if (isNoteEnd) {
      uint16_t startTs = activeNoteStartTs[ch][d1];
      if (startTs != 0xFFFF) {
        if (quantGrid != 0 && quantStrength > 0) {
          unsigned long loopLenLocal = (looperState == STATE_RECORDING) ? 0 : loopLen;
          finalTs = protectNoteEnd(startTs, finalTs, loopLenLocal, looperState == STATE_RECORDING);
        }
        activeNoteStartTs[ch][d1] = 0xFFFF;
      }
    }
  }

  if (deviceConnected) {
    if (looperState == STATE_RECORDING) {
      if (t == 0xB0) sendBleNotify("/rec_cc " + String(finalTs));
      else if (isNoteStart) sendBleNotify("/rec_noteon " + String(d1) + " " + String(finalTs));
      else if (isNoteEnd) sendBleNotify("/rec_noteoff " + String(d1) + " " + String(finalTs));
    }
    else {
      float posNorm = 0.0;
      if (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
        if (looperState == STATE_OVERDUB) {
          posNorm = (float)finalTs / (float)loopLen;
        } else {
          posNorm = (float)((now - playStart) % loopLen) / (float)loopLen;
        }
      }

      if (t == 0xB0) sendBleNotify("/cc " + String(posNorm, 4));
      else if (t == 0x90 && d2 > 0) sendBleNotify("/noteon " + String(d1) + " " + String(posNorm, 4));
      else if (t == 0x80 || (t == 0x90 && d2 == 0)) sendBleNotify("/noteoff " + String(d1) + " " + String(posNorm, 4));
    }
  }

  if (!recordIt) return;

  if (eventCount < 5000) {
    // Событие сыграно вживую прямо сейчас: в овердабе (и при дозаписи Note Off)
    // оно НЕ должно сразу проигрываться лупером ещё раз — только со следующего круга.
    bool alreadySounded = (looperState != STATE_RECORDING);
    events[eventCount++] = {finalTs, t, d1, d2, ch, currentLayer, alreadySounded};
  }
}

#if USB_MIDI_ENABLED
// Приём MIDI по USB: ноты/CC в рекордер, клок/старт — как с DIN
void handleUsbMidiInput() {
  midiEventPacket_t p;
  while (usbMidi.readPacket(&p)) {
    uint8_t status = p.byte1;

    if (status == 0xF8) { if (acceptClockSource(CLK_USB)) handleClock(); continue; }
    if (status == 0xFA || status == 0xFB) { if (acceptClockSource(CLK_USB)) handleStart(); continue; }

    uint8_t type = status & 0xF0;
    uint8_t ch = (status & 0x0F) + 1;
    if (type == 0x90 || type == 0x80 || type == 0xB0) {
      recordEvent(type, p.byte2, p.byte3, ch);
    }
  }
}
#endif

void playback() {
  if (looperState != STATE_PLAYING && looperState != STATE_OVERDUB) return;
  unsigned long now = millis();
  unsigned long pos = (now - playStart) % loopLen;
  static unsigned long lastPos = 0;

  if (pos < lastPos) {
    for (int i = 0; i < eventCount; i++) events[i].played = false;

    if (pendingLoadSlot >= 1) {
      int slotToLoad = pendingLoadSlot;
      pendingLoadSlot = -1;
      loadPreset(slotToLoad);
      return;
    }
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].played) continue;
    if (pos >= events[i].timestamp) {
      if (events[i].type == 0xB0) {
        if (now - ccTakeover[events[i].ch][events[i].d1] < 1200) { events[i].played = true; continue; }
        if (events[i].layer < ccMaxLayer[events[i].ch][events[i].d1]) { events[i].played = true; continue; }
        lastSentCCTime[events[i].ch-1][events[i].d1] = now;
      }
      if (events[i].type == 0x90 || events[i].type == 0x80) lastSentNoteTime[events[i].ch-1][events[i].d1] = now;

      sendMidi(events[i].type, events[i].d1, events[i].d2, events[i].ch);
      events[i].played = true;
    }
  }
  lastPos = pos;
}

// Резервный внутренний счёт долей, если внешнего MIDI-клока нет
void handleInternalClock() {
  unsigned long now = millis();
  if (now - lastClockTime >= 1200) {
    if (now - lastBeatTime >= 545) {
      lastBeatTime = now;
      if (beatInBar == 0) triggerQuantizedAction();
      beatInBar = (beatInBar + 1) % 4;
    }
  }
}

// Педаль LOOPER_PIN: действие при нажатии, дребезг без блокировки
void handlePedal() {
#if PEDAL_ENABLED
  static byte lastReading = HIGH, stable = HIGH;
  static unsigned long lastChange = 0;
  unsigned long now = millis();
  byte reading = digitalRead(LOOPER_PIN);
  if (reading != lastReading) { lastReading = reading; lastChange = now; }
  if (reading != stable && now - lastChange >= 20) {
    stable = reading;
    if (stable == LOW) looperShortPress();
  }
#endif
}

// ================== ИМЕНА / ТЕКСТЫ ==================
String gridName(int g) {
  switch (g) {
    case 1: return "Q32";
    case 2: return "Q16";
    case 3: return "Q8";
    case 4: return "Q4";
    default: return "OFF";
  }
}
// Что именно подтягивается к сетке: начала нот, концы нот или оба
String targetName(int t) {
  switch (t) {
    case QUANT_START: return "starts";
    case QUANT_END:   return "ends";
    default:          return "start+end";
  }
}
String signedStr(int v) {
  return (v > 0 ? "+" : "") + String(v);
}

// ================== UI: действия ==================
bool loopIsPlaying() {
  return loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB);
}

void uiDoSave(int slot) {
  selectedPresetSlot = slot;
  if (savePreset(slot)) toast("Saved " + String(slot));
  else toast("Save failed");
}
void uiDoLoad(int slot) {
  selectedPresetSlot = slot;
  if (!presetExists(slot)) { toast("Slot " + String(slot) + " empty"); return; }
  requestLoadPreset(slot);
  if (loopIsPlaying()) toast("Load " + String(slot) + " at loop end");
  else toast("Load " + String(slot) + " on beat 1");
}
void uiDoApplyQuant() {
  if (eventCount <= 0 || loopLen == 0) { toast("No loop"); return; }
  if (quantGrid == 0 || quantStrength <= 0) { toast("Grid is OFF"); return; }
  applyQuantizeToLoop();
  toast("Quantized");
}
void uiDoHalf() {
  if (!loopIsPlaying() || loopLen < 500) { toast("Need playing loop"); return; }
  halfLoop();
  toast("Loop /2");
}
void uiDoDouble() {
  if (!loopIsPlaying()) { toast("Need playing loop"); return; }
  if (loopLen * 2 > 65000) { toast("Too long"); return; }
  doubleLoop();
  toast("Loop x2");
}
void uiDoTranspose(int semitones) {
  if (semitones == 0) { toast("No change"); return; }
  if (!loopIsPlaying()) { toast("Need playing loop"); return; }
  transposeLoop(semitones);
  toast("Transpose " + signedStr(semitones));
}
void uiDoUndo() {
  if (loopLen == 0 || currentLayer == 0) { toast("Nothing to undo"); return; }
  undoLastLayer();
  toast("Undo");
}

// ================== UI: меню ==================
String menuValueText(int idx, bool editing) {
  switch (idx) {
    case MI_GRID: {
      String s = gridName(editing ? editValue : quantGrid);
      return editing ? ">" + s + "<" : s;
    }
    case MI_STRENGTH: {
      String s = String(editing ? editValue : quantStrength) + "%";
      return editing ? ">" + s + "<" : s;
    }
    case MI_TARGET: {
      String s = targetName(editing ? editValue : quantTarget);
      return editing ? ">" + s + "<" : s;
    }
    case MI_CHANNELS:
      return String(enabledChannelCount()) + "/16";
    case MI_TRANSPOSE:
      if (editing) return ">" + signedStr(editValue) + " st<";
      return "";
    case MI_DEL_PRESET:
      if (editing) return ">" + String(editValue) + (presetExists(editValue) ? "*" : "") + "<";
      return "";
    default:
      return "";
  }
}

// Клик энкодера на пункте списка: войти в редактирование/подменю или выполнить действие
void menuActivate() {
  switch (menuCursor) {
    case MI_GRID:       editValue = quantGrid;          uiMode = UI_MENU_EDIT; break;
    case MI_STRENGTH:   editValue = quantStrength;      uiMode = UI_MENU_EDIT; break;
    case MI_TARGET:     editValue = quantTarget;        uiMode = UI_MENU_EDIT; break;
    case MI_TRANSPOSE:  editValue = 0;                  uiMode = UI_MENU_EDIT; break;
    case MI_DEL_PRESET: editValue = selectedPresetSlot; uiMode = UI_MENU_EDIT; break;
    case MI_CHANNELS:   chCursor = 0;                   uiMode = UI_MENU_CHANNELS; break;
    case MI_APPLY:      uiDoApplyQuant();    uiMode = UI_NORMAL; break;
    case MI_HALF:       uiDoHalf();          uiMode = UI_NORMAL; break;
    case MI_DOUBLE:     uiDoDouble();        uiMode = UI_NORMAL; break;
  }
}
// Клик энкодера при редактировании значения: подтвердить
void menuCommit() {
  switch (menuCursor) {
    case MI_GRID:     quantGrid = editValue;     sendQuantStatus(); uiMode = UI_MENU_LIST; break;
    case MI_STRENGTH: quantStrength = editValue; sendQuantStatus(); uiMode = UI_MENU_LIST; break;
    case MI_TARGET:   quantTarget = editValue;   sendQuantStatus(); uiMode = UI_MENU_LIST; break;
    case MI_TRANSPOSE:
      uiDoTranspose(editValue);
      uiMode = UI_NORMAL;
      break;
    case MI_DEL_PRESET:
      if (presetExists(editValue)) { deletePreset(editValue); toast("Deleted " + String(editValue)); }
      else toast("Slot " + String(editValue) + " empty");
      uiMode = UI_NORMAL;
      break;
    default: uiMode = UI_MENU_LIST; break;
  }
}

// Клик в подменю Channels
void channelsClick() {
  if (chCursor < 16) {
    recChannelMask ^= (uint16_t)(1u << chCursor);
  } else if (chCursor == 16) {
    recChannelMask = 0xFFFF;   // ALL ON
  } else {
    recChannelMask = 0x0000;   // ALL OFF
  }
  saveChannelMask();
  sendChannelStatus();
}

void onEncoderClick() {
  uiTouch();
  switch (uiMode) {
    case UI_MENU_LIST:     menuActivate(); break;
    case UI_MENU_EDIT:     menuCommit(); break;
    case UI_MENU_CHANNELS: channelsClick(); break;
    case UI_SAVE_SLOT:     uiDoSave(pendingSlot); uiMode = UI_NORMAL; break;
    case UI_LOAD_SLOT:     uiDoLoad(pendingSlot); uiMode = UI_NORMAL; break;
    default: break;  // в обычном режиме энкодер ничего не делает
  }
}

void applyEncoderDelta(int delta) {
  if (delta == 0) return;
  uiTouch();
  switch (uiMode) {
    case UI_MENU_LIST:
      menuCursor = (menuCursor + delta + MI_COUNT) % MI_COUNT;
      break;
    case UI_MENU_EDIT:
      switch (menuCursor) {
        case MI_GRID:       editValue = constrain(editValue + delta, 0, 4); break;
        case MI_STRENGTH:   editValue = constrain(editValue + delta * 5, 0, 100); break;
        case MI_TARGET:     editValue = constrain(editValue + delta, 0, 2); break;
        case MI_TRANSPOSE:  editValue = constrain(editValue + delta, -12, 12); break;
        case MI_DEL_PRESET: editValue = constrain(editValue + delta, 1, PRESET_COUNT); break;
      }
      break;
    case UI_MENU_CHANNELS:
      chCursor = (chCursor + delta + CH_CURSOR_COUNT) % CH_CURSOR_COUNT;
      break;
    case UI_SAVE_SLOT:
    case UI_LOAD_SLOT:
      pendingSlot = constrain(pendingSlot + delta, 1, PRESET_COUNT);
      break;
    default:
      break;
  }
}

void handleUiTimeout() {
  if (uiMode != UI_NORMAL && millis() - uiModeStart > UI_TIMEOUT) uiMode = UI_NORMAL;
}

// ================== ЭНКОДЕР (табличный квадратурный декодер) ==================
volatile int8_t encoderDelta = 0;
volatile uint8_t encoderPrevAB = 0;

static const int8_t QEM[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(ENCODER_CLK);
  uint8_t b = digitalRead(ENCODER_DT);
  uint8_t currAB = (a << 1) | b;
  uint8_t idx = ((encoderPrevAB << 2) | currAB) & 0x0F;
  encoderDelta += QEM[idx];
  encoderPrevAB = currAB;
}

void handleEncoder() {
  noInterrupts();
  int delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  static int accum = 0;

  // В обычном режиме энкодер ничего не делает
  if (uiMode == UI_NORMAL) {
    accum = 0;
    return;
  }

  accum += delta;

  while (accum >= 4) {
    applyEncoderDelta(1);
    accum -= 4;
  }
  while (accum <= -4) {
    applyEncoderDelta(-1);
    accum += 4;
  }
}

void handleEncoderSwitch() {
  static unsigned long lastDebounce = 0;
  static bool lastState = HIGH;
  unsigned long now = millis();
  bool state = digitalRead(ENCODER_SW);
  if (state != lastState) {
    if (now - lastDebounce > 50) {
      lastState = state;
      lastDebounce = now;
      if (state == LOW) onEncoderClick();
    }
  }
}

// ================== 4 КНОПКИ У ЭКРАНА ==================
void onMenuButtonPress(int i) {
  switch (i) {
    case 0:  // RESET
      uiMode = UI_NORMAL;
      resetLooper();
      toast("Reset");
      break;
    case 1:  // PRESETS: фиксируем момент нажатия, решение — при отпускании / по таймеру
      presetBtnDown = true;
      presetLongFired = false;
      presetBtnStart = millis();
      break;
    case 2:  // UNDO
      uiDoUndo();
      break;
    case 3:  // MENU
      if (uiMode == UI_MENU_EDIT) uiMode = UI_MENU_LIST;          // отмена редактирования
      else if (uiMode == UI_MENU_CHANNELS) uiMode = UI_MENU_LIST; // выход из подменю
      else if (uiMode == UI_MENU_LIST) uiMode = UI_NORMAL;        // выход из меню
      else uiMode = UI_MENU_LIST;                                 // вход в меню
      break;
  }
}

void onMenuButtonRelease(int i) {
  if (i == 1) {
    if (presetBtnDown && !presetLongFired) {
      // короткое нажатие — выбор слота для LOAD
      if (uiMode == UI_LOAD_SLOT) uiMode = UI_NORMAL;
      else { pendingSlot = selectedPresetSlot; uiMode = UI_LOAD_SLOT; }
    }
    presetBtnDown = false;
  }
}

void handleMenuButtons() {
  unsigned long now = millis();
  for (int i = 0; i < MENU_BTN_COUNT; i++) {
    bool reading = digitalRead(menuButtonPins[i]);
    if (reading != menuBtnState[i] && now - menuBtnDebounce[i] > 50) {
      menuBtnState[i] = reading;
      menuBtnDebounce[i] = now;
      uiTouch();
      if (reading == LOW) onMenuButtonPress(i);
      else onMenuButtonRelease(i);
    }
  }

  // Удержание кнопки пресетов 2 с — выбор слота для SAVE
  if (presetBtnDown && !presetLongFired && now - presetBtnStart >= PRESET_LONG_PRESS_MS) {
    presetLongFired = true;
    if (uiMode == UI_SAVE_SLOT) uiMode = UI_NORMAL;
    else { pendingSlot = selectedPresetSlot; uiMode = UI_SAVE_SLOT; }
    uiTouch();
  }
}

// ================== OLED ==================
// Безопасная разность времён (задача OLED может прочитать "now" раньше, чем основной цикл обновит метку)
unsigned long safeSince(unsigned long now, unsigned long t) {
  long d = (long)(now - t);
  return d < 0 ? 0 : (unsigned long)d;
}

// BPM по длительности доли (MIDI-клок)
int currentBpm() {
  unsigned long bd = (loopBeatDuration > 0) ? loopBeatDuration : 545;
  return (int)((60000UL + bd / 2) / bd);
}

// Количество тактов: в записи — уже полностью записанные, в игре — длина лупа
String barsText() {
  unsigned long now = millis();
  unsigned long bd = (loopBeatDuration > 0) ? loopBeatDuration : 545;
  float barMs = 4.0f * (float)bd;
  float bars = 0.0f;

  if (looperState == STATE_RECORDING) {
    bars = floor((float)safeSince(now, recStart) / barMs);
  } else if (loopLen > 0) {
    bars = (float)loopLen / barMs;
  } else {
    return "- bars";
  }

  float r = round(bars);
  if (fabs(bars - r) < 0.1f) {
    int n = (int)r;
    return String(n) + (n == 1 ? " bar" : " bars");
  }
  return String(bars, 1) + " bars";
}

// Прогресс-бар по тактам: n сегментов на всю ширину, progress — позиция в сегментах (0..n)
void drawBarSegments(int n, float progress) {
  const int y = 18;
  const int h = 8;
  if (n < 1) n = 1;
  if (n > 40) n = 40;
  if (progress < 0.0f) progress = 0.0f;

  int gap = (n <= 16) ? 2 : 1;
  int segW = (SCREEN_WIDTH - gap * (n - 1)) / n;
  if (segW < 2) segW = 2;
  int total = segW * n + gap * (n - 1);
  int x0 = (SCREEN_WIDTH - total) / 2;
  if (x0 < 0) x0 = 0;

  for (int i = 0; i < n; i++) {
    int x = x0 + i * (segW + gap);
    float frac = progress - (float)i;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    display.drawRect(x, y, segW, h, SSD1306_WHITE);
    if (segW >= 6) {
      int innerW = segW - 4;
      int fillW = (int)(frac * innerW + 0.5f);
      if (fillW > 0) display.fillRect(x + 2, y + 2, fillW, h - 4, SSD1306_WHITE);
    } else {
      int fillW = (int)(frac * segW + 0.5f);
      if (fillW > 0) display.fillRect(x, y, fillW, h, SSD1306_WHITE);
    }
  }
}

// Текущая высота полоски ноты (0 = не рисовать): держится пока нота зажата, потом плавно гаснет
int vizHeight(const VizSlot &s, unsigned long now, unsigned long holdTimeout) {
  int hMax = 1 + ((int)s.vel * (VIZ_MAX_H - 1)) / 127;
  if (s.held) {
    if (safeSince(now, s.onT) < holdTimeout) return hMax;
    return 0;
  }
  if (s.offT != 0) {
    unsigned long dt = safeSince(now, s.offT);
    if (dt < VIZ_DECAY_MS) {
      return (int)((float)hMax * (1.0f - (float)dt / (float)VIZ_DECAY_MS) + 0.5f);
    }
  }
  return 0;
}

// Индикация нот в самом низу экрана:
//  - сплошные полоски — входящие ноты,
//  - шахматные ("серые") полоски — ноты, которые играет лупер.
void drawNoteViz() {
  unsigned long now = millis();
  for (int p = VIZ_NOTE_LO; p <= VIZ_NOTE_HI; p++) {
    int x = ((p - VIZ_NOTE_LO) * SCREEN_WIDTH) / VIZ_NOTE_COUNT;

    // "линейка": точка на каждой ноте До
    if (p % 12 == 0) display.drawPixel(x, VIZ_BASE_Y, SSD1306_WHITE);

    // ноты лупера (рисуем первыми, входящие поверх)
    int ho = vizHeight(vizOut[p], now, VIZ_OUT_HOLD_TIMEOUT_MS);
    for (int r = 0; r < ho; r++) {
      for (int c = 0; c < 2; c++) {
        if (((r + c) & 1) == 0) display.drawPixel(x + c, VIZ_BASE_Y - 1 - r, SSD1306_WHITE);
      }
    }

    // входящие ноты
    int hi = vizHeight(vizIn[p], now, VIZ_IN_HOLD_TIMEOUT_MS);
    if (hi > 0) display.fillRect(x, VIZ_BASE_Y - hi, 2, hi, SSD1306_WHITE);
  }
}

// Список меню (3 видимые строки)
void drawMenu() {
  const int visible = 3;
  int top = constrain(menuCursor - 1, 0, MI_COUNT - visible);
  for (int r = 0; r < visible; r++) {
    int idx = top + r;
    int y = 29 + r * 9;
    bool sel = (idx == menuCursor);
    String val = menuValueText(idx, sel && uiMode == UI_MENU_EDIT);

    if (sel) {
      display.fillRect(0, y - 1, SCREEN_WIDTH, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(2, y);
    display.print(menuLabels[idx]);
    if (val.length() > 0) {
      display.setCursor(SCREEN_WIDTH - 2 - (int)val.length() * 6, y);
      display.print(val);
    }
  }
  display.setTextColor(SSD1306_WHITE);
}

// Подменю Channels: сетка 2x8, залитая клетка = канал записывается, рамка = курсор
void drawChannelsMenu() {
  for (int i = 0; i < 16; i++) {
    int row = i / 8, col = i % 8;
    int cx = col * 16;
    int cy = (row == 0) ? 27 : 38;
    bool on = (recChannelMask & (1u << i)) != 0;
    bool sel = (chCursor == i);
    String t = String(i + 1);
    int tx = cx + (16 - (int)t.length() * 6) / 2;

    if (on) {
      display.fillRect(cx + 2, cy + 2, 12, 7, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(tx, cy + 2);
    display.print(t);
    display.setTextColor(SSD1306_WHITE);

    if (sel) display.drawRect(cx, cy, 16, 11, SSD1306_WHITE);
  }

  // Нижняя строка: ALL ON / ALL OFF
  bool selOn = (chCursor == 16);
  bool selOff = (chCursor == 17);

  if (selOn) {
    display.fillRect(0, 49, 44, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else display.setTextColor(SSD1306_WHITE);
  display.setCursor(2, 49);
  display.print("ALL ON");

  if (selOff) {
    display.fillRect(64, 49, 50, 8, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else display.setTextColor(SSD1306_WHITE);
  display.setCursor(66, 49);
  display.print("ALL OFF");

  display.setTextColor(SSD1306_WHITE);
}

// Выбор слота пресета (LOAD / SAVE)
void drawPresetPicker() {
  bool isSave = (uiMode == UI_SAVE_SLOT);
  display.setCursor(0, 27);
  if (isSave) display.print(presetExists(pendingSlot) ? "SAVE: overwrite!" : "SAVE preset");
  else        display.print(presetExists(pendingSlot) ? "LOAD preset" : "LOAD: empty slot");

  for (int i = 1; i <= PRESET_COUNT; i++) {
    int x = 4 + (i - 1) * 24;
    int y = 37;
    String t = String(i) + (presetExists(i) ? "*" : "");
    bool sel = (i == pendingSlot);
    if (sel) {
      display.fillRect(x, y, 22, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.drawRect(x, y, 22, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(x + (22 - (int)t.length() * 6) / 2, y + 2);
    display.print(t);
  }
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 50);
  display.print("Turn=pick Enc=OK");
}

void updateOLED() {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  unsigned long now = millis();
  unsigned long recElapsed = safeSince(now, recStart);

  // Шапка, строка 1: состояние + такты
  display.setCursor(0, 0);
  display.print(stateName());
  display.setCursor(80, 0);
  display.print(barsText());

  // Шапка, строка 2: BPM + длина в секундах (в записи — текущее время записи)
  unsigned long lenMs = (looperState == STATE_RECORDING) ? recElapsed : loopLen;
  display.setCursor(0, 9);
  display.print(String(currentBpm()) + " BPM");
  display.setCursor(80, 9);
  display.print(String(lenMs / 1000.0f, 2) + "s");

  // Прогресс-бар по тактам
  unsigned long bd = (loopBeatDuration > 0) ? loopBeatDuration : 545;
  float barMs = 4.0f * (float)bd;
  int segs = 1;
  float progress = 0.0f;

  if (looperState == STATE_RECORDING) {
    // растёт: записанные такты залиты, текущий заполняется
    float elapsedBars = (float)recElapsed / barMs;
    segs = (int)floorf(elapsedBars) + 1;
    progress = elapsedBars;
  } else if (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
    // столько сегментов, сколько тактов в лупе; заполнение по позиции воспроизведения
    segs = (int)roundf((float)loopLen / barMs);
    if (segs < 1) segs = 1;
    unsigned long pos = safeSince(now, playStart) % loopLen;
    progress = (float)pos / (float)loopLen * (float)segs;
  } else {
    // IDLE: один прямоугольник, заполняется по долям текущего такта
    unsigned long bdur = (beatDuration > 0) ? beatDuration : 545;
    unsigned long tib = safeSince(now, lastBeatTime);
    if (tib > bdur) tib = bdur;
    progress = ((float)beatInBar + (float)tib / (float)bdur) / 4.0f;
    segs = 1;
  }
  drawBarSegments(segs, progress);

  // Нижняя часть экрана
  UiMode mode = uiMode;
  if (mode == UI_MENU_LIST || mode == UI_MENU_EDIT) {
    drawMenu();
  } else if (mode == UI_MENU_CHANNELS) {
    drawChannelsMenu();
  } else if (mode == UI_SAVE_SLOT || mode == UI_LOAD_SLOT) {
    drawPresetPicker();
  } else {
    display.setCursor(0, 28);
    display.print("Grid:" + gridName(quantGrid) + " Str:" + String(quantStrength) + "%");

    display.setCursor(0, 37);
    display.print("Quantize:" + targetName(quantTarget));

    // Третья строка: сообщение / подсказка / ожидающая загрузка / текущий пресет
    display.setCursor(0, 46);
    if (now < toastUntil) {
      display.print(toastBuf);
    } else if (presetBtnDown && !presetLongFired && now - presetBtnStart > 300) {
      display.print("Hold 2s = SAVE...");
    } else if (pendingLoadSlot >= 1) {
      display.print("Load " + String(pendingLoadSlot) + " pending...");
    } else {
      display.print("Preset: " + String(selectedPresetSlot) + (presetExists(selectedPresetSlot) ? " [x]" : " [ ]"));
    }
  }

  // Индикация нот — в самом низу, во всех режимах
  drawNoteViz();

  display.display();
}

// Экран рисуется в отдельной задаче на втором ядре, чтобы I2C не тормозил MIDI
void oledTask(void *arg) {
  for (;;) {
    updateOLED();
    vTaskDelay(pdMS_TO_TICKS(60));
  }
}

// ================== LED ==================
// Плавный импульс: 1 -> 0 за dur мс (косинусное сглаживание)
float softPulse(unsigned long diff, unsigned long dur) {
  if (diff >= dur) return 0.0f;
  float x = (float)diff / (float)dur;
  return 0.5f * (1.0f + cosf(PI * x));
}

void updateLED() {
  unsigned long now = millis();
  unsigned long diff = now - lastBeatTime;
  bool accent = (beatInBar == 0);
  float level = 0.0f;

  if (pendingStart || pendingStop || pendingLoadSlot >= 1) {
    // ожидание старта/стопа/загрузки: плавное "дыхание" ~2 Гц
    float phase = (float)(now % 500) * (TWO_PI / 500.0f);
    level = 0.25f + 0.75f * (0.5f + 0.5f * sinf(phase));
  }
  else if (looperState == STATE_IDLE) {
    // очень мягкий пульс
    level = softPulse(diff, accent ? 500 : 320) * (accent ? 0.55f : 0.22f);
  }
  else if (looperState == STATE_PLAYING) {
    level = softPulse(diff, accent ? 420 : 260) * (accent ? 0.90f : 0.40f);
  }
  else {
    // REC / OVERDUB: светится, на доле плавно проседает
    float p = softPulse(diff, accent ? 320 : 200);
    level = 1.0f - p * (accent ? 0.90f : 0.55f);
  }

  if (level < 0.0f) level = 0.0f;
  if (level > 1.0f) level = 1.0f;
  int brightness = (int)(255.0f * level * level + 0.5f);   // гамма ~2
  ledcWrite(LED_PIN, brightness);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("BOOT: start");

  LittleFS.begin(true);
  Serial.println("BOOT: LittleFS ok");
  loadChannelMask();
  refreshPresetCache();
  Serial.println("BOOT: settings loaded");

  resetActiveNoteTracking();
  ledcAttach(LED_PIN, pwmFreq, pwmRes);
  pinMode(LOOPER_PIN, INPUT_PULLUP);
  Serial.println("BOOT: base pins ok");

  // Энкодер
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  encoderPrevAB = (digitalRead(ENCODER_CLK) << 1) | digitalRead(ENCODER_DT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);
  Serial.println("BOOT: encoder ok");

  // 4 кнопки у экрана
  for (int i = 0; i < MENU_BTN_COUNT; i++) pinMode(menuButtonPins[i], INPUT_PULLUP);
  Serial.println("BOOT: menu buttons ok");

  // OLED
#if OLED_ENABLED
  Serial.println("BOOT: init I2C/OLED...");
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setTimeOut(500);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Serial.println(oledReady ? "BOOT: OLED found" : "BOOT: OLED NOT FOUND");
  if (oledReady) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Looper v60 ready");
    display.display();
  }
#else
  Serial.println("BOOT: OLED disabled for diagnostics");
  oledReady = false;
#endif

  MIDI_Serial.begin(31250, SERIAL_8N1, 16, 17);
  MIDI.begin(MIDI_CHANNEL_OMNI); MIDI.turnThruOff();
  Serial.println("BOOT: MIDI ok");

#if USB_MIDI_ENABLED
  USB.productName("MIDI-Looper");
  usbMidi.begin();
  USB.begin();
  Serial.println("BOOT: USB MIDI ok");
#endif

  MIDI.setHandleClock([]() { if (acceptClockSource(CLK_DIN)) handleClock(); });
  MIDI.setHandleStart([]() { if (acceptClockSource(CLK_DIN)) handleStart(); });
  MIDI.setHandleContinue([]() { if (acceptClockSource(CLK_DIN)) handleStart(); });
  MIDI.setHandleNoteOn([](byte ch, byte n, byte v) { recordEvent(0x90, n, v, ch); });
  MIDI.setHandleNoteOff([](byte ch, byte n, byte v) { recordEvent(0x80, n, v, ch); });
  MIDI.setHandleControlChange([](byte ch, byte n, byte v) { recordEvent(0xB0, n, v, ch); });

  BLEDevice::init("MIDI-Looper");
  Serial.println("BOOT: BLE init ok");
  BLEDevice::setMTU(500);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);

  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BOOT: BLE advertising started");

  // Отрисовка экрана в отдельной задаче (ядро 0), основной цикл на ядре 1 не блокируется
  if (oledReady) xTaskCreatePinnedToCore(oledTask, "oled", 8192, NULL, 1, NULL, 0);

  Serial.println("BOOT: setup DONE");
}

void loop() {
  while (MIDI.read()) { }
#if USB_MIDI_ENABLED
  handleUsbMidiInput();
#endif
  handlePedal();
  handleInternalClock();
  handleEncoder();
  handleEncoderSwitch();
  handleMenuButtons();
  handleUiTimeout();
  playback();
  updateLED();

  if (needSendPresetStatus && deviceConnected && millis() - connectStatusTime > 400) {
    sendPresetStatus();
    sendQuantStatus();
    sendChannelStatus();
    sendBeatInfo();
    needSendPresetStatus = false;
  }

  if (visualDumpActive && deviceConnected) {
    unsigned long nowDump = millis();

    if (visualDumpIndex < eventCount) {
      if (visualDumpIndex > 0 && nowDump - visualDumpLastPosSend >= VISUAL_DUMP_POS_INTERVAL) {
        visualDumpLastPosSend = nowDump;
        sendBleNotify("/rec_pos " + String(events[visualDumpIndex - 1].timestamp + 1));
      }

      if (nowDump - lastVisualDumpTime >= VISUAL_DUMP_EVENT_INTERVAL) {
        lastVisualDumpTime = nowDump;
        MidiEvent &e = events[visualDumpIndex];
        if (e.type == 0x90 && e.d2 > 0) {
          sendBleNotify("/rec_noteon " + String(e.d1) + " " + String(e.timestamp));
        } else if (e.type == 0x80 || (e.type == 0x90 && e.d2 == 0)) {
          sendBleNotify("/rec_noteoff " + String(e.d1) + " " + String(e.timestamp));
        } else if (e.type == 0xB0) {
          sendBleNotify("/rec_cc " + String(e.timestamp));
        }
        visualDumpIndex++;
      }
    }
    else if (visualDumpFinishStage == 0) {
      sendBleNotify("/rec_pos " + String(loopLen));
      sendBleNotify("/state " + stateName());
      visualDumpFinishStage = 1;
      visualDumpFinishTime = nowDump;
    }
    else if (visualDumpFinishStage == 1 && nowDump - visualDumpFinishTime >= 20) {
      sendBleNotify("/rec_pos " + String(loopLen));
      sendBleNotify("/state " + stateName());
      visualDumpFinishStage = 0;
      visualDumpActive = false;
    }
  }

  if (deviceConnected && !visualDumpActive && (millis() - lastBleUpdate >= BLE_INTERVAL)) {
    lastBleUpdate = millis();
    if (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
      uint32_t currentPos = (millis() - playStart) % loopLen;
      float posNorm = (float)currentPos / (float)loopLen;
      sendBleNotify("/pos " + String(posNorm, 4));
    }
    else if (looperState == STATE_RECORDING) {
      sendBleNotify("/rec_pos " + String(millis() - recStart));
    }
    else if (looperState == STATE_IDLE) {
      unsigned long now = millis();
      unsigned long timeInBeat = now - lastBeatTime;
      if (timeInBeat > beatDuration) timeInBeat = beatDuration;

      float idlePosNorm = ((float)beatInBar + ((float)timeInBeat / (float)beatDuration)) / 4.0f;
      sendBleNotify("/pos " + String(idlePosNorm, 4));
    }
  }
}
