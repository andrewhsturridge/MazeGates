/*
 * Maze Gates – Unified Node v1 (ESP32-S3 Feather)
 * One binary for all nodes. Dynamic LED strips (+ buttons) via server config.
 * Transport: ESP-NOW ch.6; Jittered HELLO; OTA .bin over Wi-Fi.
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_system.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include "vl53l4cx_class.h"   // STM32duino VL53L4CX by ST

#include "MazeGates_Map.h"

// ---------- OTA defaults ----------
#ifndef OTA_SSID
#define OTA_SSID "GUD"
#define OTA_PASS "EscapE66"
#define OTA_BASE_URL "http://192.168.2.231:8000/"
#define OTA_NODE_PATH "MazeGates/MazeGates_Node/build/esp32.esp32.um_feathers3/MazeGates_Node.ino.bin"
#endif

// ---------- Feature switches ----------
#undef  ACCEPT_RS5
#define ACCEPT_RS5 1

// ---------- Fixed pins (I2C / lamps defaults) ----------
#define PIN_SDA 8
#define PIN_SCL 9
#define PIN_LAMP1 12
#define PIN_LAMP2 6
#define PIN_LAMP3 5

// ---------- Dynamic LED strips (up to 5) ----------
static const uint8_t MAX_STRIPS = 5;

struct StripCfg { uint8_t pin; uint16_t count; };
static StripCfg cfg[MAX_STRIPS];
static uint8_t  stripCount = 0;

static Adafruit_NeoPixel* strips[MAX_STRIPS] = {nullptr};

// NVS helpers (strips)
static void saveStripCfg(const StripCfg* c, uint8_t n){
  Preferences p; p.begin("maze", false);
  p.putUChar("sN", n);
  for (uint8_t i=0;i<MAX_STRIPS;i++){
    char kp[8], kc[8]; snprintf(kp,8,"s%up",i); snprintf(kc,8,"s%uc",i);
    p.putUChar(kp, (i<n)?c[i].pin:0);
    p.putUShort(kc, (i<n)?c[i].count:0);
  }
  p.end();
}
static void loadStripCfg(){
  Preferences p; p.begin("maze", true);
  uint8_t n = p.getUChar("sN", 0);
  if (n==0 || n>MAX_STRIPS){
    // Default (Node 4): 2 strips 10/11 @ 185
    stripCount = 2; cfg[0]={10,185}; cfg[1]={11,185};
  } else {
    stripCount = n;
    for (uint8_t i=0;i<stripCount;i++){
      char kp[8], kc[8]; snprintf(kp,8,"s%up",i); snprintf(kc,8,"s%uc",i);
      cfg[i].pin   = p.getUChar(kp, 0);
      cfg[i].count = p.getUShort(kc, 0);
    }
  }
  p.end();
}
static void applyStripCfg(){
  for (uint8_t i=0;i<MAX_STRIPS;i++){ if (strips[i]){ delete strips[i]; strips[i]=nullptr; } }
  for (uint8_t i=0;i<stripCount;i++){
    if (cfg[i].pin && cfg[i].count){
      strips[i] = new Adafruit_NeoPixel(cfg[i].count, cfg[i].pin, NEO_GRB + NEO_KHZ800);
      strips[i]->begin(); strips[i]->clear(); strips[i]->show();
    }
  }
}

// ---------- Dynamic button pins (up to 3) ----------
static const uint8_t MAX_BTNS = 3;
static uint8_t  btnPins[MAX_BTNS] = {17,18,14};
static uint8_t  btnCount = 3;
struct BtnState { bool last; uint32_t lastEdge; };
static BtnState btnState[MAX_BTNS] = {};

// NVS helpers (buttons)
static void saveBtnPins(const uint8_t *pins, uint8_t n){
  Preferences p; p.begin("maze", false);
  p.putUChar("bN", n);
  for (uint8_t i=0;i<MAX_BTNS;i++){
    char kp[8]; snprintf(kp,8,"b%up",i);
    p.putUChar(kp, (i<n)?pins[i]:0);
  }
  p.end();
}
static void loadBtnPins(){
  Preferences p; p.begin("maze", true);
  uint8_t n = p.getUChar("bN", 0);
  if (n==0 || n>MAX_BTNS){ btnCount = 3; btnPins[0]=17; btnPins[1]=18; btnPins[2]=14; }
  else {
    btnCount = n;
    for (uint8_t i=0;i<btnCount;i++){
      char kp[8]; snprintf(kp,8,"b%up",i);
      btnPins[i] = p.getUChar(kp, 0);
    }
  }
  p.end();
}
static void applyBtnPins(){
  for (uint8_t i=0;i<btnCount;i++){
    if (btnPins[i]) { pinMode(btnPins[i], INPUT_PULLUP); btnState[i] = {true, 0}; }
  }
}

// ---- Dynamic ToF channel -> gate map (8 entries) ----
static uint8_t tofGateByCh[8] = { 23,24,28,29,34,35,39,40 };

static void saveTofMap(const uint8_t g[8]){
  Preferences p; p.begin("maze", false);
  for (uint8_t i=0;i<8;i++){ char k[6]; snprintf(k,6,"tg%u",i); p.putUChar(k, g[i]); }
  p.end();
}
static void loadTofMap(){
  Preferences p; p.begin("maze", true);
  bool any = false;
  for (uint8_t i=0;i<8;i++){
    char k[6]; snprintf(k,6,"tg%u",i);
    uint8_t v = p.getUChar(k, 0xFF);
    if (v != 0xFF){ tofGateByCh[i] = v; any = true; }
  }
  p.end();
  // If you want non-Node4 nodes to start blank until set by server, uncomment:
  if (!any) memset(tofGateByCh, 0, sizeof(tofGateByCh));
}

// ---------- Protocol ----------
enum MsgType : uint8_t {
  HELLO=1, HELLO_REQ=2, CLAIM=3,
  GATE_EVENT=10, LED_RANGE=20, LAMP_CTRL=21, BUTTON_EVENT=30,
  OTA_START=50, OTA_ACK=51,
  NODE_STATUS=60,
  LED_MAP=62, LED_MAP_REQ=63, LED_MAP_RSP=64,
  BTN_PINS=70, BTN_PINS_REQ=71, BTN_PINS_RSP=72,
  TOF_MAP = 80, TOF_MAP_REQ = 81, TOF_MAP_RSP = 82,
  GAME_STATE  = 90, ROUND_CFG = 92
};

enum NodeGameState : uint8_t { NODE_IDLE=0, NODE_PLAYING=1, NODE_OVER=2 };
static volatile uint8_t nodeState = NODE_IDLE;

struct __attribute__((packed)) PktHeader { uint8_t type, version, nodeId, pad; uint16_t seq, len; };
static const uint8_t PROTO_VER = 1;

struct __attribute__((packed)) GameStateMsg {
  PktHeader h;
  uint8_t state;  // NodeGameState
  uint8_t r,g,b;  // overlay color to apply when not PLAYING
};

struct __attribute__((packed)) RoundCfgMsg {
  PktHeader h;            // h.pad carries epoch (optional)
  uint8_t   nTargets;     // 0..12
  uint8_t   targets[12];  // global Btn indices (1..12)
  uint8_t   walkBits[6];  // 44 gates -> 44 bits (LSB = gate1)
};

struct __attribute__((packed)) HelloMsg { PktHeader h; uint8_t role; uint8_t caps; };
struct __attribute__((packed)) ClaimMsg { PktHeader h; uint8_t newNodeId; };
struct __attribute__((packed)) GateEventMsg { PktHeader h; uint8_t gateId; uint8_t ev; uint16_t strengthMm; uint32_t tsMs; };
struct __attribute__((packed)) ButtonEventMsg { PktHeader h; uint8_t btnIdx; uint8_t ev; uint32_t tsMs; };
struct __attribute__((packed)) LedRangeMsg { PktHeader h; uint8_t strip; uint16_t start, count; uint8_t effect; uint8_t r,g,b; uint16_t durationMs; };
struct __attribute__((packed)) LampCtrlMsg { PktHeader h; uint8_t idx; uint8_t on; };
struct __attribute__((packed)) OtaStartMsg { PktHeader h; char url[200]; };
struct __attribute__((packed)) OtaAckMsg { PktHeader h; uint8_t status; };
struct __attribute__((packed)) TofMapMsg { PktHeader h; uint8_t g[8]; };
struct __attribute__((packed)) TofMapRsp { PktHeader h; uint8_t g[8]; };

struct __attribute__((packed)) NodeStatusMsg {
  PktHeader h; uint32_t uptimeMs; uint8_t initedMask; uint8_t errStreakMax; uint8_t reinitCount[8];
};

struct __attribute__((packed)) LedMapMsg {
  PktHeader h; uint8_t n; struct { uint8_t pin; uint16_t count; } e[5];
};
struct __attribute__((packed)) LedMapRsp {
  PktHeader h; uint8_t n; struct { uint8_t pin; uint16_t count; } e[5];
};
struct __attribute__((packed)) BtnPinsMsg {
  PktHeader h; uint8_t n; uint8_t pin[3];
};
struct __attribute__((packed)) BtnPinsRsp {
  PktHeader h; uint8_t n; uint8_t pin[3];
};

// ---------- Simple pending-work flags (no queues) ----------

// Apply GAME_STATE in loop (set by onNowRecv)
static volatile bool     statePending  = false;
static volatile uint8_t  pendingEpoch  = 0;
static volatile uint8_t  pendingState  = NODE_IDLE;
static volatile uint8_t  pendingR=0, pendingG=0, pendingB=0;

// Current epoch (optional; adopted from header.pad)
static volatile uint8_t  currentEpoch  = 0;

// Lamps: desired ON/OFF per idx, and which indices are dirty
// bit0 -> idx1, bit1 -> idx2, bit2 -> idx3
static volatile uint8_t  lampDesiredMask = 0;   // target state
static volatile uint8_t  lampDirtyMask   = 0;   // which to apply

// One pending LED_RANGE (last-wins)
static volatile bool     ledPending   = false;
static volatile uint8_t  ledP_strip   = 0;
static volatile uint16_t ledP_start   = 0;
static volatile uint16_t ledP_count   = 0;
static volatile uint8_t  ledP_r=0, ledP_g=0, ledP_b=0;
static volatile uint8_t  ledP_epoch   = 0;

// Round config
static volatile bool     rcPending    = false;
static volatile uint8_t  rcEpoch      = 0;
static volatile uint8_t  rcNT         = 0;
static volatile uint8_t  rcTargets[12];
static volatile uint8_t  rcWalkBits[6];

static void nodeEnterIdle(uint8_t epoch, uint8_t r, uint8_t g, uint8_t b){
  if (epoch) currentEpoch = epoch;
  nodeState = NODE_IDLE;
  ledPending = false;
  fillAllStrips(r,g,b);
  Serial.printf("[STATE] -> IDLE  rgb(%u,%u,%u) epoch=%u\n", r,g,b,currentEpoch);
  sendHello();  // ack to server
}

static void nodeEnterPlaying(uint8_t epoch){
  if (epoch) currentEpoch = epoch;
  nodeState = NODE_PLAYING;
  ledPending = false;
  fillAllStrips(0,0,0);
  Serial.printf("[STATE] -> PLAYING epoch=%u\n", currentEpoch);
  sendHello();  // ack to server
}

static void nodeEnterOver(uint8_t epoch, uint8_t r, uint8_t g, uint8_t b){
  if (epoch) currentEpoch = epoch;
  nodeState = NODE_OVER;
  ledPending = false;
  fillAllStrips(r,g,b);
  Serial.printf("[STATE] -> OVER  rgb(%u,%u,%u) epoch=%u\n", r,g,b,currentEpoch);
  sendHello();  // ack to server
}

static void applyGameStatePending(){
  if (!statePending) return;
  statePending = false;
  uint8_t e = pendingEpoch, s = pendingState;
  uint8_t r = pendingR, g = pendingG, b = pendingB;
  if (s == NODE_PLAYING)      nodeEnterPlaying(e);
  else if (s == NODE_OVER)    nodeEnterOver(e, r, g, b);
  else                        nodeEnterIdle(e, r, g, b);
}

// ---------- ESP-NOW ----------
static volatile bool gOtaPending = false;
static char gOtaUrlBuf[200] = {0};
static uint16_t gSeq=1;
Preferences prefs;
static uint8_t gNodeId=0;
static uint8_t kBroadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ----- Tiny non-blocking TX queue for ESP-NOW -----
#define TXQ_SIZE 16
struct TxFrame { uint8_t len; uint8_t buf[64]; };
static TxFrame txq[TXQ_SIZE];
static volatile uint8_t txHead = 0, txTail = 0;

static inline bool enqueueTx(const void* data, uint8_t len){
  uint8_t nhead = (uint8_t)((txHead + 1) % TXQ_SIZE);
  if (nhead == txTail) return false;        // full -> drop (or count drops)
  if (len > sizeof(txq[0].buf)) len = sizeof(txq[0].buf);
  memcpy(txq[txHead].buf, data, len);
  txq[txHead].len = len;
  txHead = nhead;
  return true;
}
static void pumpTx(){
  while (txTail != txHead){
    TxFrame &f = txq[txTail];
    esp_err_t r = esp_now_send(kBroadcast, f.buf, f.len);
    if (r == ESP_OK) txTail = (uint8_t)((txTail + 1) % TXQ_SIZE);
    else break; // try again next loop tick
  }
}
static void onNowSent(const wifi_tx_info_t* info, esp_now_send_status_t status) {
  pumpTx();  // kick the TX queue again
}

static void sendRaw(const uint8_t* mac, const uint8_t* data, size_t len){
  if (esp_now_send(mac, data, len) == ESP_OK) return;
  enqueueTx(data, (uint8_t)((len > 64) ? 64 : len));
}

static void sendHello(){ HelloMsg m{}; m.h={HELLO,PROTO_VER,gNodeId,0,gSeq++,sizeof(HelloMsg)}; m.role=1; m.caps=0; sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m)); }
static void sendGateEvent(uint8_t gateId, uint8_t ev, uint16_t strengthMm){
  GateEventMsg m{}; m.h={GATE_EVENT,PROTO_VER,gNodeId,0,gSeq++,sizeof(GateEventMsg)};
  m.gateId=gateId; m.ev=ev; m.strengthMm=strengthMm; m.tsMs=millis();
  sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m));
}
static void sendButtonEvent(uint8_t idx, uint8_t ev){
  ButtonEventMsg m{}; m.h={BUTTON_EVENT,PROTO_VER,gNodeId,0,gSeq++,sizeof(ButtonEventMsg)};
  m.btnIdx=idx; m.ev=ev; m.tsMs=millis();
  sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m));
}
static void sendOtaAck(uint8_t status){ OtaAckMsg m{}; m.h={OTA_ACK,PROTO_VER,gNodeId,0,gSeq++,sizeof(OtaAckMsg)}; m.status=status; sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m)); }

// ---------- LED MAP / BTN PINS replies ----------
static void sendLedMapRsp(){
  LedMapRsp r{}; r.h={ LED_MAP_RSP, PROTO_VER, gNodeId, 0, gSeq++, (uint16_t)sizeof(LedMapRsp) };
  r.n = stripCount;
  for (uint8_t i=0; i<stripCount && i<5; ++i){ r.e[i].pin=cfg[i].pin; r.e[i].count=cfg[i].count; }
  sendRaw(kBroadcast,(uint8_t*)&r,sizeof(r));
}
static void sendBtnPinsRsp(){
  BtnPinsRsp r{}; r.h={ BTN_PINS_RSP, PROTO_VER, gNodeId, 0, gSeq++, (uint16_t)sizeof(BtnPinsRsp) };
  r.n = btnCount; for (uint8_t i=0;i<btnCount;i++) r.pin[i]=btnPins[i];
  sendRaw(kBroadcast,(uint8_t*)&r,sizeof(r));
}

// ---------- HELLO jitter ----------
static volatile bool gHelloPending=false, gStatusPending=false;
static uint32_t helloDueAt=0, statusDueAt=0;

// ---------- Node status ----------
static bool     inited[8]    = {0};
static uint8_t  errStreak[8] = {0};
static void sendNodeStatus() {
  NodeStatusMsg m{};
  m.h = { NODE_STATUS, PROTO_VER, gNodeId, 0, gSeq++, (uint16_t)sizeof(NodeStatusMsg) };
  m.uptimeMs = millis();
  uint8_t mask=0, maxErr=0;
  for (uint8_t ch=0; ch<8; ++ch){
    if (inited[ch]) mask |= (1u<<ch);
    if (errStreak[ch] > maxErr) maxErr = errStreak[ch];
  }
  m.initedMask   = mask;
  m.errStreakMax = maxErr;
  sendRaw(kBroadcast, (uint8_t*)&m, sizeof(m));
}

// LED render throttle (event-driven)
static volatile bool ledDirty = false;
static uint32_t nextShowAt = 0;
static const uint32_t LED_MIN_INTERVAL_MS = 40;  // ~25 fps max, tweakable

// ---------- ESP-NOW RX ----------
static void onNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len){
  if (!info || len < (int)sizeof(PktHeader)) return; auto *h=(const PktHeader*)data;
  if (h->version!=PROTO_VER) return;

  if (h->type == GAME_STATE && len >= (int)sizeof(GameStateMsg)){
    const GameStateMsg* m = (const GameStateMsg*)data;

    // Stage for the loop to apply atomically
    pendingEpoch = h->pad;
    pendingState = (m->state == NODE_PLAYING) ? NODE_PLAYING
                : (m->state == NODE_OVER)    ? NODE_OVER
                                              : NODE_IDLE;
    pendingR = m->r; pendingG = m->g; pendingB = m->b;
    statePending = true;        // <— let the loop do the actual fill

    return;
  }

  if (h->type == ROUND_CFG && len >= (int)sizeof(RoundCfgMsg)){
    const RoundCfgMsg* r = (const RoundCfgMsg*)data;
    rcEpoch = h->pad;
    rcNT    = r->nTargets;
    if (rcNT > 12) rcNT = 12;
    for (uint8_t i=0;i<rcNT;i++) rcTargets[i] = r->targets[i];
    for (uint8_t i=0;i<6;i++)    rcWalkBits[i] = r->walkBits[i];
    rcPending = true;     // loop will apply atomically
    return;
  }

  if (h->type==HELLO_REQ){
    uint32_t jitter = (uint32_t)random(20, 181);   // 20–180 ms
    helloDueAt  = millis() + jitter;
    statusDueAt = helloDueAt + 5;
    gHelloPending  = true;
    gStatusPending = true;
    return;
  }

  if (h->type==CLAIM && len>= (int)sizeof(ClaimMsg)) {
    auto *m=(const ClaimMsg*)data; gNodeId = m->newNodeId;
    prefs.begin("maze",false); prefs.putUChar("nodeId",gNodeId); prefs.end();
    sendHello();
    return;
  }

  // LED_RANGE: ignore unless actively playing
  if (h->type == LED_RANGE && len >= (int)sizeof(LedRangeMsg)) {
    if (nodeState != NODE_PLAYING) return;
    const LedRangeMsg* m = (const LedRangeMsg*)data;
    if (m->strip < stripCount && strips[m->strip]) {
      uint16_t end = m->start + m->count;
      if (end > cfg[m->strip].count) end = cfg[m->strip].count;
      for (uint16_t i=m->start; i<end; ++i)
        strips[m->strip]->setPixelColor(i, strips[m->strip]->Color(m->r,m->g,m->b));
      ledDirty = true;
    }
    return;
  }

  // LAMP_CTRL: set desired bit and mark dirty (always honored)
  if (h->type==LAMP_CTRL && len >= (int)sizeof(LampCtrlMsg)) {
    const LampCtrlMsg* m=(const LampCtrlMsg*)data;
    if (m->idx>=1 && m->idx<=3){
      uint8_t bit = (1u << (m->idx-1));
      if (m->on) lampDesiredMask |=  bit; else lampDesiredMask &= ~bit;
      lampDirtyMask |= bit;   // schedule apply in loop
    }
    return;
  }

  // LED_MAP set (persist & reboot)
  if (h->type == LED_MAP && len >= (int)sizeof(PktHeader)+1) {
    const LedMapMsg* m = (const LedMapMsg*)data;
    uint8_t n = m->n; if (n > MAX_STRIPS) n = MAX_STRIPS;
    StripCfg tmp[MAX_STRIPS] = {};
    for (uint8_t i=0;i<n;i++){
      uint8_t pin=m->e[i].pin; uint16_t cnt=m->e[i].count;
      if (!pin || !cnt) { n=i; break; }
      if (cnt>600) cnt=600;
      tmp[i] = {pin,cnt};
    }
    saveStripCfg(tmp, n);
    Serial.printf("[LED_MAP] saved %u strips; rebooting\n", n);
    forceTofColdState();
    delay(100); ESP.restart();
    return;
  }

  // LED_MAP get
  if (h->type == LED_MAP_REQ && len >= (int)sizeof(PktHeader)) {
    sendLedMapRsp(); return;
  }

  // BTN_PINS set/get
  if (h->type == BTN_PINS && len >= (int)sizeof(BtnPinsMsg)){
    auto *m=(const BtnPinsMsg*)data; uint8_t n=m->n; if (n>MAX_BTNS) n=MAX_BTNS;
    uint8_t tmp[3]={0,0,0}; for (uint8_t i=0;i<n;i++) tmp[i]=m->pin[i];
    saveBtnPins(tmp, n);
    btnCount=n; for (uint8_t i=0;i<n;i++) btnPins[i]=tmp[i];
    applyBtnPins();
    sendBtnPinsRsp();
    return;
  }
  if (h->type == BTN_PINS_REQ && len >= (int)sizeof(PktHeader)){
    sendBtnPinsRsp(); return;
  }

  // OTA trigger
  if (h->type==OTA_START && len >= (int)sizeof(PktHeader)) {
    String url="";
    if (len > (int)sizeof(PktHeader)){
      const char* u = (const char*)(data + sizeof(PktHeader));
      if (u && *u) url = String(u);
    }
    sendOtaAck(0);
    if (url.length() > 0) { strncpy(gOtaUrlBuf, url.c_str(), sizeof(gOtaUrlBuf)-1); }
    else { snprintf(gOtaUrlBuf, sizeof(gOtaUrlBuf), "%s%s", OTA_BASE_URL, OTA_NODE_PATH); }
    gOtaPending = true;
    return;
  }

  // Set ToF map
  if (h->type == TOF_MAP && len >= (int)sizeof(TofMapMsg)){
    auto *m = (const TofMapMsg*)data;
    memcpy(tofGateByCh, m->g, 8);
    saveTofMap(tofGateByCh);
    Serial.printf("[TOF_MAP] saved: %u,%u,%u,%u,%u,%u,%u,%u\n",
      m->g[0],m->g[1],m->g[2],m->g[3],m->g[4],m->g[5],m->g[6],m->g[7]);
    // optional ack
    TofMapRsp r{}; r.h={ TOF_MAP_RSP, PROTO_VER, gNodeId,0,gSeq++,(uint16_t)sizeof(r) };
    memcpy(r.g, tofGateByCh, 8); sendRaw(kBroadcast,(uint8_t*)&r,sizeof(r));
    return;
  }

  // Get ToF map
  if (h->type == TOF_MAP_REQ && len >= (int)sizeof(PktHeader)){
    TofMapRsp r{}; r.h={ TOF_MAP_RSP, PROTO_VER, gNodeId,0,gSeq++,(uint16_t)sizeof(r) };
    memcpy(r.g, tofGateByCh, 8); sendRaw(kBroadcast,(uint8_t*)&r,sizeof(r));
    return;
  }
}

// ---------- OTA visuals (dynamic across all strips) ----------
static volatile bool gOtaMode = false;

static void drawOtaProgress(size_t done, size_t total){
  float f = total ? (float)done/(float)total : 0.f;
  uint32_t totalPix = 0; for (uint8_t i=0;i<stripCount;i++) totalPix += cfg[i].count;
  if (!totalPix) return;
  uint32_t lit = (uint32_t)(f * totalPix + 0.5f);

  // clear
  for (uint8_t i=0;i<stripCount;i++){
    if (!strips[i]) continue;
    for (uint16_t p=0;p<cfg[i].count;p++) strips[i]->setPixelColor(p, 0);
  }
  // fill across strips
  uint32_t rem = lit;
  for (uint8_t i=0;i<stripCount && rem>0;i++){
    if (!strips[i]) continue;
    uint32_t seg = (rem > cfg[i].count) ? cfg[i].count : rem;
    for (uint16_t p=0;p<seg;p++) strips[i]->setPixelColor(p, strips[i]->Color(0,150,0));
    rem -= seg;
  }
  for (uint8_t i=0;i<stripCount;i++) if (strips[i]) strips[i]->show();
}
static void showOtaError(){
  for (int k=0;k<3;k++){
    for (uint8_t i=0;i<stripCount;i++){
      if (!strips[i]) continue;
      for (uint16_t p=0;p<cfg[i].count;p++) strips[i]->setPixelColor(p, strips[i]->Color(150,0,0));
      strips[i]->show();
    }
    delay(150);
    for (uint8_t i=0;i<stripCount;i++){
      if (!strips[i]) continue;
      for (uint16_t p=0;p<cfg[i].count;p++) strips[i]->setPixelColor(p, 0);
      strips[i]->show();
    }
    delay(150);
  }
}
static void showOtaSuccess(){
  for (uint8_t i=0;i<stripCount;i++){
    if (!strips[i]) continue;
    for (uint16_t p=0;p<cfg[i].count;p++) strips[i]->setPixelColor(p, strips[i]->Color(0,150,0));
    strips[i]->show();
  }
}

static inline bool bitsetGet(const uint8_t* bits, uint8_t gate1to44){
  if (gate1to44 < 1 || gate1to44 > 44) return false;
  uint8_t i = (uint8_t)((gate1to44-1) >> 3);
  uint8_t b = (uint8_t)((gate1to44-1) & 7);
  return (bits[i] >> b) & 1u;
}

static void nodePaintWalkable(const uint8_t* walkBits){
  // Compute max extent for THIS node's strips
  uint16_t maxLen[8] = {0,0,0,0,0,0,0,0};
  for (uint8_t g=1; g<=44; ++g){
    const auto &e = GATE_MAP[g];
    if (e.nodeId != gNodeId) continue;
    uint16_t end = e.start + e.count;
    if (end > maxLen[e.strip]) maxLen[e.strip] = end;
  }

  // Base coat RED per used strip
  for (uint8_t s=0; s<8; ++s){
    if (s >= stripCount || !strips[s]) continue;
    uint16_t cnt = maxLen[s]; if (!cnt) continue;
    for (uint16_t p=0; p<cnt; ++p)
      strips[s]->setPixelColor(p, strips[s]->Color(150,0,0));
  }

  // Overlay GREEN for walkables (only our gates)
  for (uint8_t g=1; g<=44; ++g){
    const auto &e = GATE_MAP[g];
    if (e.nodeId != gNodeId) continue;
    bool ok = bitsetGet(walkBits, g);
    uint32_t color = strips[e.strip]->Color(ok?0:150, ok?150:0, 0);
    uint16_t end = e.start + e.count;
    if (e.strip >= stripCount || !strips[e.strip]) continue;
    uint16_t cap = cfg[e.strip].count; if (end > cap) end = cap;
    for (uint16_t i=e.start; i<end; ++i)
      strips[e.strip]->setPixelColor(i, color);
  }

  ledDirty = true; // render() will show()
}

static void applyTargetsOnThisNode(uint8_t nTargets, const uint8_t* targets){
  // Clear all lamp outputs first
  lampDesiredMask = 0;
  lampDirtyMask   = 0x07; // mark all 3 indices dirty -> will set below

  for (uint8_t i=0; i<nTargets && i<12; ++i){
    uint8_t b = targets[i];
    if (b < 1 || b > 12) continue;
    const BtnMapEntry &m = BTN_DEFAULT[b];
    if (m.nodeId == gNodeId && m.lampIdx>=1 && m.lampIdx<=3){
      lampDesiredMask |= (1u << (m.lampIdx-1));
    }
  }
}

// --- I2C bus recovery: 9 SCL pulses + STOP to release a stuck slave ---
static void i2cBusRecover(uint8_t sda=PIN_SDA, uint8_t scl=PIN_SCL){
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, OUTPUT);
  // If SDA is low, clock until it releases
  for (int i=0; i<9 && digitalRead(sda)==LOW; ++i){
    digitalWrite(scl, HIGH); delayMicroseconds(8);
    digitalWrite(scl, LOW);  delayMicroseconds(8);
  }
  // Generate STOP
  pinMode(sda, OUTPUT);
  digitalWrite(sda, LOW); delayMicroseconds(8);
  digitalWrite(scl, HIGH); delayMicroseconds(8);
  digitalWrite(sda, HIGH); delayMicroseconds(8);
  // Restore
  pinMode(sda, INPUT_PULLUP);
}

// ---------- ESP-NOW/OTA helpers ----------
static void stopEspNow(){ esp_now_deinit(); }
static esp_err_t startEspNow(){ if (esp_now_init()!=ESP_OK) return ESP_FAIL; esp_now_register_recv_cb(onNowRecv); esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6); p.channel=6; p.encrypt=false; p.ifidx=WIFI_IF_STA; esp_now_add_peer(&p); return ESP_OK; }
static bool connectWifiSta(const String& ssid, const String& pass, uint32_t timeoutMs=15000){ WiFi.disconnect(true); WiFi.mode(WIFI_STA); WiFi.begin(ssid.c_str(), pass.c_str()); uint32_t t0=millis(); while (WiFi.status()!=WL_CONNECTED && (millis()-t0)<timeoutMs) delay(100); return WiFi.status()==WL_CONNECTED; }
static void performHttpOta(String url, String ssid, String pass){
  gOtaMode = true;
  stopEspNow();
  if (ssid.length()==0) ssid = OTA_SSID;
  if (pass.length()==0) pass = OTA_PASS;
  if (url.length()==0)  url  = String(OTA_BASE_URL) + OTA_NODE_PATH;
  if (!connectWifiSta(ssid, pass)){
    Serial.println("[OTA] WiFi connect failed"); showOtaError();
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); startEspNow(); return;
  }
  Serial.printf("[OTA] Connected %s IP=%s\n", ssid.c_str(), WiFi.localIP().toString().c_str());
  Update.onProgress([](size_t d,size_t t){ drawOtaProgress(d,t); });
  WiFiClient client; HTTPUpdate httpUpdate; httpUpdate.rebootOnUpdate(false);
  Serial.printf("[OTA] GET %s\n", url.c_str());
  t_httpUpdate_return r = httpUpdate.update(client, url);
  if (r==HTTP_UPDATE_OK){ Serial.println("[OTA] OK, rebooting"); showOtaSuccess(); delay(400); forceTofColdState(); ESP.restart(); }
  else { Serial.printf("[OTA] Fail code=%d\n", (int)r); Serial.printf("[OTA] Error: %s\n", httpUpdate.getLastErrorString().c_str()); showOtaError(); esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); startEspNow(); gOtaMode = false; }
}

// ---------- TCA9548A ----------
#define TCA_ADDR 0x70
static inline void tcaDeselectAll(){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(2);
}
static inline bool tcaSelect(uint8_t ch){
  if (ch>7) return false;
  Wire.beginTransmission(TCA_ADDR); Wire.write(1<<ch);
  bool ok = (Wire.endTransmission()==0);
  if (ok) delay(5); // EXACT like test
  return ok;
}
static void tcaOff(){ Wire.beginTransmission(TCA_ADDR); Wire.write(0); Wire.endTransmission(); }

// ---------- ToF sensors: per-channel objects ----------
#define DEV_I2C Wire
#define NUM_SENSORS 8
VL53L4CX tof0(&DEV_I2C, 255), tof1(&DEV_I2C, 255), tof2(&DEV_I2C, 255), tof3(&DEV_I2C, 255),
         tof4(&DEV_I2C, 255), tof5(&DEV_I2C, 255), tof6(&DEV_I2C, 255), tof7(&DEV_I2C, 255);
VL53L4CX* TOF[NUM_SENSORS] = { &tof0,&tof1,&tof2,&tof3,&tof4,&tof5,&tof6,&tof7 };

// ---------- EXACT TEST DETECTION CONFIG ----------
static const uint16_t MIN_MM = 200, MAX_MM = 2400;
static const uint8_t  MAX_ERR_BEFORE_REINIT = 4;
static const unsigned long POLL_INTERVAL_MS = 60;

// ---------- Render ----------
static bool i2cBusy=false;

// Buffered solid fill across all configured strips (no show() here)
static void fillAllStrips(uint8_t r, uint8_t g, uint8_t b){
  for (uint8_t i=0; i<stripCount; i++){
    if (!strips[i]) continue;
    for (uint16_t p=0; p<cfg[i].count; p++){
      strips[i]->setPixelColor(p, strips[i]->Color(r,g,b));
    }
  }
  ledDirty = true;     // request a render
  nextShowAt = 0;      // force renderer on next loop tick
}

// Renderer flush (called from loop)
static void render(){
  if (!ledDirty) return;         // only show when pixels changed
  if (i2cBusy) return;           // never render during I2C (protect ToF)
  uint32_t now = millis();
  if (now < nextShowAt) return;  // fps cap

  for (uint8_t i=0; i<stripCount; i++){
    if (strips[i]) strips[i]->show();
  }
  ledDirty = false;
  nextShowAt = now + LED_MIN_INTERVAL_MS;
}

// --- Force every VL53L4CX into a known stopped state (no power cut needed) ---
static void forceTofColdState(){
  for (uint8_t ch=0; ch<8; ++ch){
    if (!tcaSelect(ch)) continue;
    TOF[ch]->VL53L4CX_StopMeasurement();
    tcaDeselectAll();
    delay(2);
  }
}

// ---------- Button polling (dynamic) ----------
static void pollButtons(){
  const uint32_t DEBOUNCE_MS = 30;
  uint32_t now = millis();
  for (uint8_t i=0;i<btnCount;i++){
    if (!btnPins[i]) continue;
    bool v = digitalRead(btnPins[i]);       // HIGH idle, LOW pressed
    if (v != btnState[i].last && (now - btnState[i].lastEdge) > DEBOUNCE_MS){
      btnState[i].last = v; btnState[i].lastEdge = now;
      uint8_t ev = v ? 2 : 1;               // 1=PRESS, 2=RELEASE
      sendButtonEvent(i+1, ev);
    }
  }
}

// ---------- ToF init / reinit / polling  ----------
static bool initOne(uint8_t i) {
  const uint8_t ch = i;

  for (int attempt = 0; attempt < 3; ++attempt) {
    if (!tcaSelect(ch)) {
      if (attempt == 2) { Serial.printf("[TCA] Channel select failed during init (ch %u)\n", ch); return false; }
      delay(5);
      continue;
    }

    delay(5);
    TOF[ch]->begin();
    VL53L4CX_Error st = TOF[ch]->InitSensor(0x52);
    if (st) {
      TOF[ch]->VL53L4CX_StopMeasurement();
      delay(5);
      st = TOF[ch]->InitSensor(0x52);
    }
    if (!st) st = TOF[ch]->VL53L4CX_StartMeasurement();

    tcaDeselectAll();

    if (!st) {
      inited[ch]    = true;
      errStreak[ch] = 0;
      Serial.printf("[VL53] READY ch %u\n", ch);
      return true;
    }

    Serial.printf("[VL53] Init/start failed ch %u (err=%d) attempt %d\n", ch, (int)st, attempt+1);
    tcaDeselectAll();
    delay(10);
  }
  return false;
}

static void reinitIfNeeded(uint8_t i, const char* why) {
  if (errStreak[i] < MAX_ERR_BEFORE_REINIT) return;
  uint8_t ch = i;
  Serial.printf("[RECOVER] Reinit ch %u (%s)\n", ch, why);
  inited[i] = false;
  if (initOne(i)) Serial.printf("[RECOVER] OK ch %u\n", ch);
  else            Serial.printf("[RECOVER] FAIL ch %u\n", ch);
  errStreak[i] = 0;
}

static uint32_t lastEnterSentMs[8] = {0};
#define ENTER_COOLDOWN_MS 150   // 0 = off; 150ms ≈ ~6–7 msgs/sec max per ch

static void pollOne(uint8_t i) {
  if (!inited[i]) return;

  static unsigned long lastPollMs[NUM_SENSORS] = {0};
  unsigned long now = millis();
  if (now - lastPollMs[i] < POLL_INTERVAL_MS) return;
  lastPollMs[i] = now;

  uint8_t ch = i;
  i2cBusy = true;
  if (!tcaSelect(ch)) { i2cBusy=false; errStreak[i]++; reinitIfNeeded(i, "TCA select"); return; }

  uint8_t ready = 0;
  int st = TOF[i]->VL53L4CX_GetMeasurementDataReady(&ready);
  if (st) {
    errStreak[i]++;
    tcaDeselectAll();
    i2cBusy=false;
    reinitIfNeeded(i, "GetReady");
    return;
  }
  if (!ready) {
    tcaDeselectAll();
    i2cBusy=false;
    return;
  }

  VL53L4CX_MultiRangingData_t data;
  st = TOF[i]->VL53L4CX_GetMultiRangingData(&data);
  if (st == 0) {
    errStreak[i] = 0;
    bool updated = false;
    uint16_t mm  = 8191;

    for (int j = 0; j < data.NumberOfObjectsFound; j++) {
      uint16_t m = data.RangeData[j].RangeMilliMeter;
      uint8_t  r = data.RangeData[j].RangeStatus;

#if ACCEPT_RS5
      bool ok = ((r == 0 || r == 5) && m >= MIN_MM && m <= MAX_MM);
#else
      bool ok = ((r == 0) && m >= MIN_MM && m <= MAX_MM);
#endif
      if (ok) { mm = m; updated = true; break; }
    }

    TOF[i]->VL53L4CX_ClearInterruptAndStartMeasurement();
    tcaDeselectAll();
    i2cBusy=false;

    if (updated) {
      uint8_t gate = tofGateByCh[ch];
      if (gate) {
        uint32_t nowMs = millis();
        if (ENTER_COOLDOWN_MS == 0 || (nowMs - lastEnterSentMs[ch]) >= ENTER_COOLDOWN_MS) {
          sendGateEvent(gate, /*ENTER*/1, mm);
          lastEnterSentMs[ch] = nowMs;
        }
      }
    }
  } else {
    tcaDeselectAll();
    i2cBusy=false;
    errStreak[i]++;
    reinitIfNeeded(i, "GetData");
  }
}

// ---------- Setup ----------
void setup(){
  Serial.begin(115200);
  delay(150);
  randomSeed(((uint32_t)ESP.getEfuseMac()) ^ micros());

  esp_reset_reason_t rr = esp_reset_reason();
  bool warmBoot = (rr != ESP_RST_POWERON && rr != ESP_RST_DEEPSLEEP);
  if (warmBoot) {
    Serial.printf("[BOOT] Warm reset (%d) – recovering I2C\n", (int)rr);
    i2cBusRecover();
  }

  // Lamps
  pinMode(PIN_LAMP1, OUTPUT); digitalWrite(PIN_LAMP1, LOW);
  pinMode(PIN_LAMP2, OUTPUT); digitalWrite(PIN_LAMP2, LOW);
  pinMode(PIN_LAMP3, OUTPUT); digitalWrite(PIN_LAMP3, LOW);

  loadStripCfg(); applyStripCfg();
  loadBtnPins();  applyBtnPins();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(200000);
  Wire.setTimeOut(20);
  delay(150);
  tcaDeselectAll();
  if (warmBoot) {
    forceTofColdState();
    tcaDeselectAll();
  }

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init()==ESP_OK){
    esp_now_register_send_cb(onNowSent);
    esp_now_register_recv_cb(onNowRecv);
    esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6);
    p.channel=6; p.encrypt=false; p.ifidx=WIFI_IF_STA; esp_now_add_peer(&p);
  }

  prefs.begin("maze", false); gNodeId = prefs.getUChar("nodeId", 4); prefs.end();
  loadTofMap();
  sendHello();

  // init per-channel sensors
  for (uint8_t ch=0; ch<8; ch++){
    if (!initOne(ch)) {
      inited[ch] = false;
      Serial.printf("[WARN] Sensor %u failed to init, will retry later.\n", ch);
    }
  }

  nodeState = NODE_IDLE;
}

// ---------- Loop ----------
void loop(){
  // 1) Apply pending GAME_STATE first (barrier)
  applyGameStatePending();

  // Apply round config immediately after PLAYING flips, and only for current epoch
  if (rcPending && nodeState == NODE_PLAYING){
    uint8_t epoch = rcEpoch;
    rcPending = false;       // take ownership

    // If you're using epochs, only apply when it matches
    if (currentEpoch == 0 || epoch == 0 || epoch == currentEpoch){
      uint8_t walkBits[6]; memcpy(walkBits, (const void*)rcWalkBits, 6);
      uint8_t targets[12];  uint8_t nT = rcNT; for (uint8_t i=0;i<nT;i++) targets[i]=rcTargets[i];

      // local paint + target lamps
      nodePaintWalkable(walkBits);
      applyTargetsOnThisNode(nT, targets);
    }
  }

  // 2) Sensing only while playing (buttons + ToF)
  const bool doSense = (nodeState == NODE_PLAYING) && !gOtaMode;
  if (doSense){
    for (uint8_t ch=0; ch<8; ch++) pollOne(ch);
    pollButtons();
  }

  // 3) Apply pending lamps (always honored; last-wins)
  if (lampDirtyMask){
    uint8_t mask = lampDirtyMask;      // take snapshot
    lampDirtyMask = 0;                 // clear for next time

    // idx 1
    if (mask & 0x01){ digitalWrite(PIN_LAMP1, (lampDesiredMask & 0x01) ? HIGH : LOW); }
    // idx 2
    if (mask & 0x02){ digitalWrite(PIN_LAMP2, (lampDesiredMask & 0x02) ? HIGH : LOW); }
    // idx 3
    if (mask & 0x04){ digitalWrite(PIN_LAMP3, (lampDesiredMask & 0x04) ? HIGH : LOW); }
  }

  // 4) Apply one pending LED paint (PLAYING only; last-wins)
  if (nodeState == NODE_PLAYING && ledPending){
    // snapshot and clear
    uint8_t  strip = ledP_strip;
    uint16_t start = ledP_start, count = ledP_count;
    uint8_t  r=ledP_r, g=ledP_g, b=ledP_b;
    uint8_t  epoch = ledP_epoch;
    ledPending = false;

    if (epoch==0 || epoch==currentEpoch){
      if (strip < stripCount && strips[strip]){
        uint16_t end = start + count;
        uint16_t cap = cfg[strip].count;
        if (end > cap) end = cap;
        for (uint16_t i=start; i<end; ++i)
          strips[strip]->setPixelColor(i, strips[strip]->Color(r,g,b));
        ledDirty = true;
      }
    }
  }

  // 5) Render LED buffers if anything changed
  render();

  // 6) OTA trigger
  if (gOtaPending){
    gOtaPending=false;
    String url = (gOtaUrlBuf[0] ? String(gOtaUrlBuf) : String());
    performHttpOta(url, String(), String());
  }

  // 7) HELLO/status timers
  uint32_t now = millis();
  if (gHelloPending && (int32_t)(now - helloDueAt) >= 0){ gHelloPending=false; sendHello(); }
  if (gStatusPending && (int32_t)(now - statusDueAt) >= 0){ gStatusPending=false; sendNodeStatus(); }

  // 8) Keep radio TX queue moving
  pumpTx();
}
