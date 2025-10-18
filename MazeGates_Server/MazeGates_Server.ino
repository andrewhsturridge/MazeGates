/*
 * Maze Gates – Server Harness v0 (clean compile)
 * Board: ESP32 (any), Transport: ESP-NOW ch.6
 * CLI: help, hello, roster, claim, setgate, fakegate, walkable, pushwalkable,
 * path set, lamp, ledmap (set/show/get), ota, ota all, status, tofvis, tofmap get/set,
 * btnmap, btnlamp, game start/end, poc start/end.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <map>
#include <vector>
#include <cstring>
#include <stdarg.h>

#include "MazeGates_Map.h"   // shared: GATE_MAP and BTN_DEFAULT

// ======= Protocol =======
enum MsgType : uint8_t {
  HELLO=1, HELLO_REQ=2, CLAIM=3,
  GATE_EVENT=10, LED_RANGE=20, LAMP_CTRL=21,
  BUTTON_EVENT=30,
  OTA_START=50, OTA_ACK=51,
  NODE_STATUS=60,
  LED_MAP=62, LED_MAP_REQ=63, LED_MAP_RSP=64,
  BTN_PINS=70, BTN_PINS_REQ=71, BTN_PINS_RSP=72,
  TOF_MAP=80, TOF_MAP_REQ=81, TOF_MAP_RSP=82,
  GAME_STATE=90,
  ROUND_CFG=92
};

enum GameStateWire : uint8_t { W_IDLE=0, W_PLAYING=1, W_OVER=2 };

struct __attribute__((packed)) PktHeader { uint8_t type, version, nodeId, pad; uint16_t seq, len; };
static const uint8_t PROTO_VER = 1;

struct __attribute__((packed)) GameStateMsg {
  PktHeader h;
  uint8_t state;    // GameStateWire
  uint8_t r,g,b;    // overlay color for non-PLAYING state
};

struct __attribute__((packed)) RoundCfgMsg {
  PktHeader h;            // h.pad carries epoch
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
struct __attribute__((packed)) BtnPinsMsg { PktHeader h; uint8_t n; uint8_t pin[3]; };
struct __attribute__((packed)) BtnPinsRsp { PktHeader h; uint8_t n; uint8_t pin[3]; };
struct __attribute__((packed)) TofMapMsg { PktHeader h; uint8_t g[8]; };
struct __attribute__((packed)) TofMapRsp { PktHeader h; uint8_t g[8]; };

struct __attribute__((packed)) NodeStatusMsg {
  PktHeader h;
  uint32_t uptimeMs;
  uint8_t  initedMask;
  uint8_t  errStreakMax;
  uint8_t  reinitCount[8];
};

struct __attribute__((packed)) LedMapMsg {
  PktHeader h;
  uint8_t n;
  struct { uint8_t pin; uint16_t count; } e[5];
};

struct __attribute__((packed)) LedMapRsp {
  PktHeader h;
  uint8_t n;
  struct { uint8_t pin; uint16_t count; } e[5];
};

// ======= Globals =======
static uint16_t gSeq=1;
static uint8_t  kBroadcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static uint8_t  gEpoch=0;   // <— declared early so helpers can use it

struct NodeInfo { uint8_t nodeId; uint8_t mac[6]; };
static std::map<uint8_t, NodeInfo> nodesById;

struct NodeStatus {
  uint32_t uptimeMs;
  uint8_t  initedMask;
  uint8_t  errStreakMax;
  uint8_t  reinitCount[8];
  uint32_t rxTsMs;
};
static std::map<uint8_t, NodeStatus> lastStatus;

// ======= Small helpers =======
static inline void walkSet(uint8_t* bits, uint8_t gate){
  if (gate<1 || gate>44) return;
  uint8_t i=(gate-1)>>3, b=(gate-1)&7; bits[i] |= (1u<<b);
}

// Default button map access (BTN_DEFAULT comes from shared header)
static inline bool getDefaultBtnLamp(uint8_t btn, uint8_t &nodeId, uint8_t &lampIdx){
  if (btn < 1 || btn > 12) return false;
  nodeId = BTN_DEFAULT[btn].nodeId;
  lampIdx = BTN_DEFAULT[btn].lampIdx;
  return nodeId != 0;
}

static inline int defaultGlobalBtnFrom(uint8_t nodeId, uint8_t localIdx){
  for (uint8_t b=1; b<=12; ++b){
    if (BTN_DEFAULT[b].nodeId == nodeId && BTN_DEFAULT[b].lampIdx == localIdx) return b;
  }
  return -1;
}

// ======= Logging =======
#define LOG_Q_SIZE 32
#define LOG_LINE_MAX 160
static char logQ[LOG_Q_SIZE][LOG_LINE_MAX];
static volatile uint8_t logHead = 0, logTail = 0;

static void qlogf(const char* fmt, ...) {
  char buf[LOG_LINE_MAX];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  uint8_t next = (uint8_t)((logTail + 1) % LOG_Q_SIZE);
  if (next == logHead) logHead = (uint8_t)((logHead + 1) % LOG_Q_SIZE);
  strncpy(logQ[logTail], buf, LOG_LINE_MAX-1);
  logQ[logTail][LOG_LINE_MAX-1] = '\0';
  logTail = next;
}
static void flushLogs() {
  while (logHead != logTail) {
    Serial.println(logQ[logHead]);
    logHead = (uint8_t)((logHead + 1) % LOG_Q_SIZE);
  }
}

// ======= Net utils =======
static String macToStr(const uint8_t m[6]){
  char b[18]; sprintf(b,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); return String(b);
}
static void addOrUpdateNode(uint8_t nodeId, const uint8_t mac[6]){
  NodeInfo n{nodeId,{0}}; memcpy(n.mac,mac,6); nodesById[nodeId]=n;
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,mac,6); p.channel=6; p.encrypt=false; p.ifidx = WIFI_IF_STA; esp_now_add_peer(&p);
}
static void sendRaw(const uint8_t mac[6], const uint8_t* data, size_t len){ esp_now_send(mac, data, len); }
static void bcastHelloReq(){ HelloMsg m{}; m.h={HELLO_REQ,PROTO_VER,0,0,gSeq++,sizeof(HelloMsg)}; m.role=0; m.caps=0; sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m)); }
static void sendClaim(const uint8_t mac[6], uint8_t nodeId){ ClaimMsg m{}; m.h={CLAIM,PROTO_VER,0,0,gSeq++,sizeof(ClaimMsg)}; m.newNodeId=nodeId; sendRaw(mac,(uint8_t*)&m,sizeof(m)); }

// ======= Drip TX queue =======
struct TxItem { uint8_t mac[6]; uint8_t len; uint8_t buf[64]; };
static TxItem txQ[64];
static volatile uint8_t qHead = 0, qTail = 0;
static uint32_t lastTxMs = 0;
static const uint16_t DRIP_MS = 5;

static void _enqueueTx(const uint8_t mac[6], const void* data, uint8_t len){
  uint8_t next = (uint8_t)((qTail + 1) & 63);
  if (next == qHead) qHead = (uint8_t)((qHead + 1) & 63); // drop oldest
  memcpy(txQ[qTail].mac, mac, 6);
  txQ[qTail].len = len;
  memcpy(txQ[qTail].buf, data, len);
  qTail = next;
}
static void dripPump(){
  if (qHead == qTail) return;
  if (millis() - lastTxMs < DRIP_MS) return;
  esp_now_send(txQ[qHead].mac, txQ[qHead].buf, txQ[qHead].len);
  lastTxMs = millis();
  qHead = (uint8_t)((qHead + 1) & 63);
}
static inline void txQClear(){ qHead = qTail; }
static void flushTxQueueQuickly(uint16_t ms=120){
  uint32_t t0 = millis();
  while (qHead != qTail && (millis() - t0) < ms){
    dripPump();
    delay(2);
  }
}

// ======= Send helpers (explicit header writes) =======
static void sendGameStateToAll(uint8_t state, uint8_t r, uint8_t g, uint8_t b, int repeats=3, int gap_ms=4){
  GameStateMsg m{};
  m.h.type = GAME_STATE;  m.h.version = PROTO_VER; m.h.nodeId = 0; m.h.pad = gEpoch;
  m.h.seq  = gSeq++;      m.h.len     = sizeof(GameStateMsg);
  m.state  = state;       m.r=r; m.g=g; m.b=b;

  for (int rep=0; rep<repeats; ++rep){
    // 1) broadcast
    sendRaw(kBroadcast, (uint8_t*)&m, sizeof(m));
    // 2) unicast to each known node
    for (auto &kv : nodesById) sendRaw(kv.second.mac, (uint8_t*)&m, sizeof(m));
    delay(gap_ms);
  }
}
static void sendGameStateBroadcast(uint8_t state, uint8_t r, uint8_t g, uint8_t b){
  GameStateMsg m{};
  m.h.type = GAME_STATE;  m.h.version = PROTO_VER; m.h.nodeId = 0; m.h.pad = gEpoch;
  m.h.seq  = gSeq++;      m.h.len     = sizeof(GameStateMsg);
  m.state  = state;       m.r=r; m.g=g; m.b=b;
  sendRaw(kBroadcast, (uint8_t*)&m, sizeof(m));
}
static void sendLedRange(uint8_t nodeId, uint8_t strip, uint16_t start, uint16_t count,
                         uint8_t r,uint8_t g,uint8_t b){
  auto it = nodesById.find(nodeId); if (it == nodesById.end()) return;
  LedRangeMsg m{};
  m.h.type=LED_RANGE; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=gEpoch;
  m.h.seq=gSeq++;     m.h.len=sizeof(LedRangeMsg);
  m.strip=strip; m.start=start; m.count=count; m.effect=0; m.r=r; m.g=g; m.b=b; m.durationMs=0;
  _enqueueTx(it->second.mac, &m, sizeof(m));
}
static void sendLampCtrl(uint8_t nodeId, uint8_t idx, bool on){
  auto it=nodesById.find(nodeId); if (it==nodesById.end()) return;
  LampCtrlMsg m{};
  m.h.type=LAMP_CTRL; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=gEpoch;
  m.h.seq=gSeq++;     m.h.len=sizeof(LampCtrlMsg);
  m.idx=idx; m.on=on?1:0;
  sendRaw(it->second.mac,(uint8_t*)&m,sizeof(m));
}
static void sendOtaStart(uint8_t nodeId, const String& url){
  auto it=nodesById.find(nodeId); if (it==nodesById.end()) return;
  OtaStartMsg m{};
  m.h.type=OTA_START; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=0;
  m.h.seq=gSeq++;     m.h.len=sizeof(OtaStartMsg);
  memset(m.url, 0, sizeof(m.url));
  if (url.length()>0) strncpy(m.url, url.c_str(), sizeof(m.url)-1);
  sendRaw(it->second.mac,(uint8_t*)&m,sizeof(m));
}

// ======= ROUND_CFG sender =======
static void sendRoundCfg(const uint8_t* targets, uint8_t nTargets, const uint8_t* walkBits){
  if (nTargets>12) nTargets=12;
  RoundCfgMsg m{};
  m.h.type=ROUND_CFG; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=gEpoch;
  m.h.seq=gSeq++;     m.h.len=sizeof(RoundCfgMsg);
  m.nTargets = nTargets;
  for (uint8_t i=0;i<nTargets;i++) m.targets[i]=targets[i];
  memset(m.walkBits, 0, sizeof(m.walkBits));
  if (walkBits) memcpy(m.walkBits, walkBits, 6);
  sendRaw(kBroadcast, (uint8_t*)&m, sizeof(m));
}

// ======= Routing helpers =======
static bool routeGate(uint8_t gateId, uint8_t &nodeId, uint8_t &strip, uint16_t &start, uint16_t &count) {
  if (gateId < 1 || gateId > 44) return false;
  const GateMapEntry &e = GATE_MAP[gateId];
  if (e.nodeId == 0) return false;
  nodeId = e.nodeId; strip = e.strip; start = e.start; count = e.count;
  return true;
}

// ======= Walkable fan-out (kept for CLI / debug) =======
#define GATE_MAX 44
static bool walkable[GATE_MAX+1]; // 1..44
static inline void clearWalkable(){ memset(walkable, 0, sizeof(walkable)); }
static inline bool isWalkable(uint8_t g){ return (g>=1 && g<=GATE_MAX) ? walkable[g] : false; }

static void pushWalkable(){
  // Base coat per (node,strip)
  uint16_t maxLen[8][8] = {}; bool seenStrip[8][8] = {}; bool seenNode[8] = {};
  for (uint8_t g=1; g<=44; ++g){
    const GateMapEntry &e = GATE_MAP[g]; if (!e.nodeId) continue;
    uint16_t end = e.start + e.count;
    if (end > maxLen[e.nodeId][e.strip]) maxLen[e.nodeId][e.strip] = end;
    seenStrip[e.nodeId][e.strip] = true; seenNode[e.nodeId] = true;
  }
  for (uint8_t n=0;n<8;++n){
    if (!seenNode[n]) continue;
    for (uint8_t s=0;s<8;++s){
      if (!seenStrip[n][s]) continue;
      uint16_t cnt = maxLen[n][s]; if (!cnt) continue;
      sendLedRange(n, s, 0, cnt, 150,0,0);
    }
  }
  // Greens
  for (uint8_t g=1; g<=44; ++g){
    const GateMapEntry &e = GATE_MAP[g]; if (!e.nodeId) continue;
    const bool ok = walkable[g];
    sendLedRange(e.nodeId, e.strip, e.start, e.count, ok?0:150, ok?150:0, 0);
  }
}

static void setWalkableFromBits(const uint8_t wb[6]){
  memset(walkable, 0, sizeof(walkable));
  for (uint8_t g=1; g<=44; ++g){
    uint8_t i = (uint8_t)((g-1) >> 3);
    uint8_t b = (uint8_t)((g-1) & 7);
    if ((wb[i] >> b) & 1u) walkable[g] = true;
  }
}

// ======= Gate events =======
static bool gTofVis=false; static uint8_t gTofR=0, gTofG=0, gTofB=255;

enum GameState { WAITING=0, PLAYING=1, GAME_OVER=2 };
struct Round { GameState st; uint8_t targetBtn; uint32_t t0; uint32_t deadlineMs; };
static Round G{WAITING, 0, 0, 0};

// Defer GAME_OVER out of onNowRecv
static volatile bool gEndPending = false;
static volatile bool gEndWin     = false;

static inline void scheduleGameEnd(bool win){
  if (G.st == PLAYING) G.st = GAME_OVER;  // freeze server logic immediately
  gEndWin     = win;
  gEndPending = true;
}

static void processGateEvent(uint8_t gateId, uint8_t ev){
  if (G.st != PLAYING){
    if (gTofVis && ev == 1){
      uint8_t nodeId, strip; uint16_t start, count;
      if (routeGate(gateId, nodeId, strip, start, count))
        sendLedRange(nodeId, strip, start, count, gTofR, gTofG, gTofB);
    }
    return;
  }
  if (ev == 1 && !isWalkable(gateId)){ scheduleGameEnd(false); return; }
  uint8_t nodeId, strip; uint16_t start, count;
  if (!routeGate(gateId, nodeId, strip, start, count)) return;
  const bool ok = isWalkable(gateId);
  sendLedRange(nodeId, strip, start, count, ok?0:150, ok?150:0, 0);
}

// ======= Lamps =======
struct BtnLamp { uint8_t nodeId; uint8_t lampIdx; bool valid; };
static BtnLamp BTNMAP[13] = {};

static bool gBtnEcho=false;
static uint32_t lampPulseUntil[13]={0};
static const uint16_t BTN_PULSE_MS=200;

static void setAllLamps(bool on){
  for (uint8_t btn=1; btn<=12; ++btn){
    uint8_t nid,lidx; if (getDefaultBtnLamp(btn, nid, lidx)) sendLampCtrl(nid, lidx, on);
  }
}
static void setTargetLamp(uint8_t btn, bool on){
  uint8_t nid,lidx; if (getDefaultBtnLamp(btn, nid, lidx)) sendLampCtrl(nid, lidx, on);
}

// ======= Start / End =======
static void gameStart(uint32_t seconds, uint8_t btn){
  gEpoch++;
  G.st=PLAYING; G.targetBtn=btn; G.t0=millis(); G.deadlineMs = seconds ? (seconds*1000UL) : 0;
  sendGameStateToAll(W_PLAYING, 0,0,0);
  delay(10);                                 // small settle so nodes apply PLAYING first
  setAllLamps(false);
  setTargetLamp(btn, true);
  // If using RoundCfg path, you'll send it in CLI (poc start). For classic mode:
  // pushWalkable();
  Serial.printf("[GAME] START %lus target=Btn%u epoch=%u\n", (unsigned)seconds, btn, gEpoch);
}
static void gameEnd(bool win){
  G.st = GAME_OVER;
  G.deadlineMs = 0;

  // Lamps off
  setAllLamps(false);

  // Tell nodes to freeze and paint local overlay
  uint8_t R = win ? 0   : 150;
  uint8_t Gc= win ? 150 : 0;
  sendGameStateToAll(W_OVER, R, Gc, 0);

  Serial.printf("[GAME] END (%s) epoch=%u\n", win ? "WIN" : "TIMEOUT/END", gEpoch);
}

// ======= RX =======
static void logNodeStatus(uint8_t nodeId, const NodeStatusMsg* m) {
  const uint8_t mask = m->initedMask;
  Serial.printf("STATUS node %u up=%lus ", nodeId, (unsigned)(m->uptimeMs/1000));
  Serial.print("up_ch=[");
  bool first=true; for (int ch=0; ch<8; ++ch) if (mask & (1u<<ch)){ if(!first)Serial.print(","); Serial.print(ch); first=false; }
  Serial.print("] down_ch=[");
  first=true; for (int ch=0; ch<8; ++ch) if (!(mask & (1u<<ch))){ if(!first)Serial.print(","); Serial.print(ch); first=false; }
  Serial.print("] ");
  Serial.printf("maxErr=%u\n", m->errStreakMax);
}

static void printNodeLedMap(uint8_t nodeId, const LedMapRsp* r){
  Serial.printf("LEDMAP node %u: ", nodeId);
  for (uint8_t i=0; i<r->n && i<5; ++i){
    if (i) Serial.print(", ");
    Serial.printf("%u:%u", r->e[i].pin, r->e[i].count);
  }
  if (r->n == 0) Serial.print("(none)");
  Serial.println();
}

static void sendLedMapReq(uint8_t nodeId){
  auto it = nodesById.find(nodeId); if (it==nodesById.end()) return;
  PktHeader h{ LED_MAP_REQ, PROTO_VER, 0, 0, gSeq++, (uint16_t)sizeof(PktHeader) };
  sendRaw(it->second.mac, (uint8_t*)&h, sizeof(h));
}
static void sendBtnPinsReq(uint8_t nodeId){
  auto it = nodesById.find(nodeId); if (it==nodesById.end()) { Serial.println("Unknown nodeId"); return; }
  PktHeader h{ BTN_PINS_REQ, PROTO_VER, 0,0, gSeq++, (uint16_t)sizeof(PktHeader) };
  sendRaw(it->second.mac,(uint8_t*)&h,sizeof(h));
}
static void sendBtnPinsSet(uint8_t nodeId, const uint8_t *pins, uint8_t n){
  auto it = nodesById.find(nodeId); if (it==nodesById.end()) { Serial.println("Unknown nodeId"); return; }
  BtnPinsMsg m{}; m.h.type=BTN_PINS; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=0; m.h.seq=gSeq++; m.h.len=sizeof(BtnPinsMsg);
  m.n=n; for (uint8_t i=0;i<n;i++) m.pin[i]=pins[i];
  _enqueueTx(it->second.mac,&m,sizeof(m));
}
static void printBtnPins(uint8_t nodeId, const BtnPinsRsp* r){
  Serial.printf("BTNPINS node %u: ", nodeId);
  if (r->n==0){ Serial.println("(none)"); return; }
  for (uint8_t i=0;i<r->n;i++){ if (i) Serial.print(", "); Serial.printf("%u", r->pin[i]); }
  Serial.println();
}
static void sendTofMapReq(uint8_t nodeId){
  auto it = nodesById.find(nodeId); if (it==nodesById.end()){ Serial.println("Unknown nodeId"); return; }
  PktHeader h{ TOF_MAP_REQ, PROTO_VER, 0,0, gSeq++, (uint16_t)sizeof(PktHeader) };
  sendRaw(it->second.mac, (uint8_t*)&h, sizeof(h));
}
static void sendTofMapSet(uint8_t nodeId, const uint8_t g[8]){
  auto it = nodesById.find(nodeId); if (it==nodesById.end()){ Serial.println("Unknown nodeId"); return; }
  TofMapMsg m{}; m.h.type=TOF_MAP; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=0; m.h.seq=gSeq++; m.h.len=sizeof(TofMapMsg);
  memcpy(m.g, g, 8);
  _enqueueTx(it->second.mac, &m, sizeof(m));
}
static void printTofMap(uint8_t nodeId, const TofMapRsp* r){
  Serial.printf("TOFMAP node %u: ", nodeId);
  for (int i=0;i<8;i++){ if (i) Serial.print(","); Serial.printf("%u", r->g[i]); }
  Serial.println();
}

static void onNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len){
  if (!info || len < (int)sizeof(PktHeader)) return;
  auto *h=(const PktHeader*)data; if (h->version!=PROTO_VER) return;
  const uint8_t* mac = info->src_addr;

  if (h->type==HELLO){
    uint8_t nid=h->nodeId; addOrUpdateNode(nid, mac);
    Serial.printf("HELLO from node %u (%s)\n", nid, macToStr(mac).c_str());
  }
  else if (h->type==GATE_EVENT && len>=(int)sizeof(GateEventMsg)){
    auto *m=(const GateEventMsg*)data;
    Serial.printf("GATE %u %s mm=%u from node %u\n",
                  m->gateId, (m->ev==1?"ENTER":(m->ev==2?"EXIT":"?")), m->strengthMm, h->nodeId);
    processGateEvent(m->gateId, m->ev);
  }
  else if (h->type==OTA_ACK && len>=(int)sizeof(OtaAckMsg)){
    auto *m=(const OtaAckMsg*)data;
    Serial.printf("OTA_ACK from node %u (status=%u)\n", h->nodeId, m->status);
  }
  else if (h->type==BUTTON_EVENT && len >= (int)sizeof(ButtonEventMsg)){
    const ButtonEventMsg* m = (const ButtonEventMsg*)data;
    Serial.printf("BUTTON%u %s from node %u\n",
                  m->btnIdx, (m->ev==1?"PRESS":"RELEASE"), h->nodeId);

    const int globalB = defaultGlobalBtnFrom(h->nodeId, m->btnIdx);
    if (globalB > 0) Serial.printf("  => B%d (default)\n", globalB);

    if (gBtnEcho && m->ev == 1){
      if (globalB >= 1 && globalB <= 12){
        uint8_t nid,lidx; if (getDefaultBtnLamp((uint8_t)globalB, nid, lidx)){
          sendLampCtrl(nid, lidx, true);
          lampPulseUntil[globalB] = millis() + BTN_PULSE_MS;
        }
      }
    }

    if (G.st == PLAYING && m->ev == 1){
      if (globalB > 0 && globalB == G.targetBtn) scheduleGameEnd(true);
      else { scheduleGameEnd(false); Serial.println("[GAME] Wrong button (penalty)"); }
    }
  }
  else if (h->type == NODE_STATUS && len >= (int)sizeof(NodeStatusMsg)) {
    const NodeStatusMsg* m = (const NodeStatusMsg*)data;
    lastStatus[h->nodeId] = { m->uptimeMs, m->initedMask, m->errStreakMax, {0}, millis() };
    memcpy(lastStatus[h->nodeId].reinitCount, m->reinitCount, 8);
    logNodeStatus(h->nodeId, m);
  }
  else if (h->type == LED_MAP_RSP && len >= (int)sizeof(LedMapRsp)) {
    const LedMapRsp* r = (const LedMapRsp*)data;
    if (r->n == 0) { qlogf("LEDMAP node %u: (none)", h->nodeId); }
    else {
      char line[LOG_LINE_MAX]; int pos = snprintf(line, sizeof(line), "LEDMAP node %u: ", h->nodeId);
      for (uint8_t i=0; i<r->n && i<5; ++i)
        pos += snprintf(line+pos, sizeof(line)-pos, "%s%u:%u", (i? ", " : ""), r->e[i].pin, r->e[i].count);
      qlogf("%s", line);
    }
  }
  else if (h->type == BTN_PINS_RSP && len >= (int)sizeof(BtnPinsRsp)) {
    const BtnPinsRsp* r = (const BtnPinsRsp*)data;
    if (r->n == 0) qlogf("BTNPINS node %u: (none)", h->nodeId);
    else if (r->n == 1) qlogf("BTNPINS node %u: %u", h->nodeId, r->pin[0]);
    else if (r->n == 2) qlogf("BTNPINS node %u: %u,%u", h->nodeId, r->pin[0], r->pin[1]);
    else                qlogf("BTNPINS node %u: %u,%u,%u", h->nodeId, r->pin[0], r->pin[1], r->pin[2]);
  }
  else if (h->type == TOF_MAP_RSP && len >= (int)sizeof(TofMapRsp)) {
    const TofMapRsp* r = (const TofMapRsp*)data;
    qlogf("TOFMAP node %u: %u,%u,%u,%u,%u,%u,%u,%u",
          h->nodeId, r->g[0],r->g[1],r->g[2],r->g[3],r->g[4],r->g[5],r->g[6],r->g[7]);
  }
}

// ======= Setup / CLI / Loop =======
static void printHelp(){
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  hello");
  Serial.println("  roster");
  Serial.println("  claim <nodeId> <mac>");
  Serial.println("  setgate <gateId> <r> <g> <b>");
  Serial.println("  fakegate <gateId> <enter|exit>");
  Serial.println("  walkable clear | add <ids> | show");
  Serial.println("  pushwalkable");
  Serial.println("  path set <ids>");
  Serial.println("  btnmap <btn> <nodeId> <lampIdx>   | btnmap show | btnmap clear <btn|all>");
  Serial.println("  btnlamp <btn> <on|off>            | btnlamptest [ms] | btnlamp echo on|off");
  Serial.println("  game start <seconds> <btn>");
  Serial.println("  game end");
  Serial.println("  lamp <nodeId> <idx> <on|off>");
  Serial.println("  ledmap show [nodeId]");
  Serial.println("  ledmap get <nodeId> | ledmap get all");
  Serial.println("  ledmap <nodeId> <pin:count>[,<pin:count>...]");
  Serial.println("  ota <nodeId> [url]");
  Serial.println("  ota all [url]");
  Serial.println("  status");
  Serial.println("  tofvis on|off  (optional: tofvis on <r> <g> <b>)");
  Serial.println("  tofmap get <nodeId> | tofmap get all");
  Serial.println("  tofmap set <nodeId> g0,g1,g2,g3,g4,g5,g6,g7");
  Serial.println("  poc start <seconds>   (B4 target; walkable: 39,28,17,6)");
  Serial.println("  poc end");
}

static void printLedMap(int filterNodeId){
  Serial.println("gate  node  strip  start  count");
  for (uint8_t g=1; g<=44; ++g){
    const auto &e = GATE_MAP[g];
    if (!e.nodeId) continue;
    if (filterNodeId>=0 && e.nodeId!=(uint8_t)filterNodeId) continue;
    Serial.printf("%3u   %3u    %2u    %4u   %4u\n", g, e.nodeId, e.strip, e.start, e.count);
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_recv_cb(onNowRecv);

  esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6);
  p.channel=6; p.encrypt=false; p.ifidx = WIFI_IF_STA; esp_now_add_peer(&p);

  delay(200);
  bcastHelloReq();
  printHelp();
}

static void handleCli(String s){
  s.trim();

  if (s=="help"){ printHelp(); return; }
  if (s=="hello"){ bcastHelloReq(); return; }

  if (s=="roster"){
    for (auto &kv : nodesById){
      auto &n=kv.second;
      Serial.printf("node %u @ %02X:%02X:%02X:%02X:%02X:%02X\n",
                    n.nodeId,n.mac[0],n.mac[1],n.mac[2],n.mac[3],n.mac[4],n.mac[5]);
    }
    return;
  }

  if (s.startsWith("claim ")){
    int nid; char macs[32];
    if (sscanf(s.c_str(),"claim %d %31s", &nid, macs)==2){
      uint8_t mac[6];
      if (sscanf(macs,"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                 &mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5])==6){
        sendClaim(mac,(uint8_t)nid); addOrUpdateNode((uint8_t)nid, mac);
        Serial.println("CLAIM sent");
      }
    }
    return;
  }

  if (s.startsWith("setgate ")){
    int gid,r,g,b;
    if (sscanf(s.c_str(),"setgate %d %d %d %d",&gid,&r,&g,&b)==4){
      uint8_t nodeId, strip; uint16_t start, count;
      if (routeGate((uint8_t)gid, nodeId, strip, start, count)){
        sendLedRange(nodeId, strip, start, count, (uint8_t)r,(uint8_t)g,(uint8_t)b);
        Serial.printf("LED_RANGE node=%u strip=%u start=%u count=%u rgb(%d,%d,%d)\n",
                      nodeId, strip, start, count, r,g,b);
      } else Serial.println("Unknown gateId or not mapped.");
    }
    return;
  }

  if (s.startsWith("fakegate ")){
    int gid; char kind[8]={0};
    if (sscanf(s.c_str(),"fakegate %d %7s",&gid,kind)==2){
      bool isEnter = (String(kind)=="enter" || String(kind)=="ENTER");
      processGateEvent((uint8_t)gid, isEnter?1:2);
      Serial.printf("FAKE %s gate %d\n", isEnter?"ENTER":"EXIT", gid);
    } else Serial.println("usage: fakegate <gateId> <enter|exit>");
    return;
  }

  if (s=="walkable clear"){ clearWalkable(); Serial.println("walkable cleared"); return; }

  if (s=="walkable show"){
    Serial.print("walkable: ");
    for (uint8_t g=1; g<=GATE_MAX; ++g) if (walkable[g]){ Serial.print(g); Serial.print(' '); }
    Serial.println();
    return;
  }

  if (s.startsWith("walkable add ")){
    char buf[256]; s.toCharArray(buf, sizeof(buf));
    char *p = strstr(buf, "add ");
    if (p){ p += 4; char *tok = strtok(p, " ,");
      while (tok){ int gid = atoi(tok); if (gid>=1 && gid<=GATE_MAX) walkable[gid]=true; tok = strtok(NULL, " ,"); }
      Serial.println("walkable updated");
    }
    return;
  }

  // path set <ids> -> clear -> add -> push
  if (s.startsWith("path set ")){
    char buf[256]; s.toCharArray(buf, sizeof(buf));
    char *p = strstr(buf, "set ");
    clearWalkable();
    if (p){ p += 4; char *tok = strtok(p, " ,");
      while (tok){ int gid = atoi(tok); if (gid>=1 && gid<=GATE_MAX) walkable[gid]=true; tok = strtok(NULL, " ,"); }
    }
    pushWalkable();
    Serial.println("path applied");
    return;
  }

  if (s=="pushwalkable"){ pushWalkable(); return; }

  if (s.startsWith("lamp ")){
    int nid, idx; char onoff[8]={0};
    if (sscanf(s.c_str(),"lamp %d %d %7s",&nid,&idx,onoff)==3){
      bool on = (String(onoff)=="on");
      sendLampCtrl((uint8_t)nid, (uint8_t)idx, on);
      Serial.printf("LAMP node=%d idx=%d %s\n", nid, idx, on?"ON":"OFF");
    }
    return;
  }

  // ledmap …
  if (s.startsWith("ledmap")){
    s.replace("\r",""); s.replace("\n",""); s.trim();

    if (s == "ledmap show"){ printLedMap(-1); return; }
    int filtId = -1;
    if (sscanf(s.c_str(),"ledmap show %d", &filtId)==1){ printLedMap(filtId); return; }

    if (s == "ledmap get all"){
      for (auto &kv : nodesById) sendLedMapReq(kv.first);
      Serial.println("LEDMAP GET sent to all nodes");
      return;
    }
    int getId = -1;
    if (sscanf(s.c_str(),"ledmap get %d", &getId)==1){
      sendLedMapReq((uint8_t)getId);
      Serial.printf("LEDMAP GET sent to node %d\n", getId);
      return;
    }

    int nid = -1; char list[256]={0};
    if (sscanf(s.c_str(),"ledmap %d %255s", &nid, list)==2 && nid>=0){
      auto it = nodesById.find((uint8_t)nid);
      if (it == nodesById.end()) { Serial.println("Unknown nodeId"); return; }

      LedMapMsg m{}; m.h.type=LED_MAP; m.h.version=PROTO_VER; m.h.nodeId=0; m.h.pad=0; m.h.seq=gSeq++; m.h.len=sizeof(LedMapMsg);
      m.n = 0;

      char buf[256]; strncpy(buf, list, sizeof(buf)-1); buf[sizeof(buf)-1]=0;
      char *tok = strtok(buf, ",");
      while (tok && m.n < 5){
        int pin=0, cnt=0;
        if (sscanf(tok, "%d:%d", &pin, &cnt)==2 && pin>0 && cnt>0){
          m.e[m.n].pin   = (uint8_t)pin;
          m.e[m.n].count = (uint16_t)cnt;
          m.n++;
        }
        tok = strtok(NULL, ",");
      }
      if (m.n==0){ Serial.println("usage: ledmap <nodeId> <pin:count>[,<pin:count>...]"); return; }

      _enqueueTx(it->second.mac, &m, sizeof(m));
      Serial.printf("LED_MAP sent to node=%d with %u strips\n", nid, m.n);
      return;
    }

    Serial.println("usage: ledmap show [nodeId] | ledmap get <nodeId>|all | ledmap <nodeId> <pin:count>[,<pin:count>...]");
    return;
  }

  // tofmap get
  if (s.startsWith("tofmap get")){
    int nid=-1;
    if (s=="tofmap get all"){ for (auto &kv : nodesById) sendTofMapReq(kv.first); Serial.println("TOFMAP GET sent to all"); return; }
    if (sscanf(s.c_str(),"tofmap get %d",&nid)==1){ sendTofMapReq((uint8_t)nid); Serial.printf("TOFMAP GET sent to node %d\n", nid); }
    else Serial.println("usage: tofmap get <nodeId>|all");
    return;
  }

  // tofmap set
  if (s.startsWith("tofmap set ")){
    int nid=-1; char list[128]={0};
    if (sscanf(s.c_str(),"tofmap set %d %127s",&nid,list)==2){
      uint8_t g[8]={0,0,0,0,0,0,0,0};
      char buf[128]; strncpy(buf,list,sizeof(buf)-1); buf[sizeof(buf)-1]=0;
      char *tok = strtok(buf, ","); int i=0;
      while (tok && i<8){ int v=atoi(tok); if (v<0) v=0; if (v>255) v=255; g[i++]=(uint8_t)v; tok = strtok(NULL,","); }
      if (i!=8){ Serial.println("usage: tofmap set <nodeId> g0,g1,g2,g3,g4,g5,g6,g7"); return; }
      sendTofMapSet((uint8_t)nid, g);
      Serial.printf("TOFMAP set node=%d [%u,%u,%u,%u,%u,%u,%u,%u]\n", nid,g[0],g[1],g[2],g[3],g[4],g[5],g[6],g[7]);
    } else Serial.println("usage: tofmap set <nodeId> g0,g1,g2,g3,g4,g5,g6,g7");
    return;
  }

  // ota
  if (s.startsWith("ota ")){
    if (s.startsWith("ota all")){
      char url[256]={0}; int n=sscanf(s.c_str(),"ota all %255s", url);
      for (auto &kv : nodesById){
        auto nid = kv.first;
        sendOtaStart(nid, (n==1)?String(url):String());
        Serial.printf("OTA_START node=%u %s\n", nid, (n==1)?url:"<default>");
      }
      return;
    }
    int nid; char url[256]={0}; int n=sscanf(s.c_str(),"ota %d %255s",&nid,url);
    sendOtaStart((uint8_t)nid, (n==2)?String(url):String());
    Serial.printf("OTA_START node=%d %s\n", nid, (n==2)?url:"<default>");
    return;
  }

  if (s=="status"){
    bcastHelloReq();
    Serial.println("node  uptime(s)  inited  maxErr  reinitCounts");
    for (auto &kv : lastStatus) {
      auto nid = kv.first; const auto &st = kv.second;
      Serial.printf("%3u  %9lu   0x%02X     %3u   [",
                    nid, (unsigned)(st.uptimeMs/1000), st.initedMask, st.errStreakMax);
      for (int i=0;i<8;i++) { Serial.printf("%u%s", st.reinitCount[i], i==7?"]\n":","); }
    }
    return;
  }

  // btnmap (dynamic override tools)
  if (s.startsWith("btnmap")){
    s.replace("\r",""); s.replace("\n",""); s.trim();

    if (s=="btnmap show"){
      Serial.println("BTN  node  lampIdx");
      for (int b=1;b<=12;b++){
        if (BTNMAP[b].valid) Serial.printf("%-3d %-5u %u\n", b, BTNMAP[b].nodeId, BTNMAP[b].lampIdx);
      }
      return;
    }
    if (s=="btnmap clear all"){
      for (int b=1;b<=12;b++) BTNMAP[b].valid=false;
      Serial.println("BTNMAP cleared");
      return;
    }
    int btn=-1;
    if (sscanf(s.c_str(),"btnmap clear %d",&btn)==1 && btn>=1 && btn<=12){
      BTNMAP[btn].valid=false;
      Serial.printf("BTNMAP: Btn%d cleared\n", btn);
      return;
    }
    int nid=-1,lidx=-1;
    if (sscanf(s.c_str(),"btnmap %d %d %d",&btn,&nid,&lidx)==3 && btn>=1 && btn<=12){
      BTNMAP[btn] = { (uint8_t)nid, (uint8_t)lidx, true };
      Serial.printf("BTNMAP: Btn%d -> node=%d lampIdx=%d\n", btn, nid, lidx);
      return;
    }
    Serial.println("usage: btnmap <btn 1..12> <nodeId> <lampIdx 1..3> | btnmap show | btnmap clear <btn|all>");
    return;
  }

  // btnlamp echo on|off
  if (s.startsWith("btnlamp echo ")){
    char onoff[8]={0};
    if (sscanf(s.c_str(),"btnlamp echo %7s", onoff)==1){
      gBtnEcho = (String(onoff)=="on");
      Serial.printf("btnlamp echo: %s\n", gBtnEcho?"ON":"OFF");
    } else Serial.println("usage: btnlamp echo on|off");
    return;
  }

  // btnlamp <btn> <on|off>
  if (s.startsWith("btnlamp ")){
    int btn=0; char onoff[8]={0};
    if (sscanf(s.c_str(),"btnlamp %d %7s",&btn,onoff)==2 && btn>=1 && btn<=12){
      uint8_t nid,lidx;
      if (getDefaultBtnLamp((uint8_t)btn, nid, lidx)){
        bool on = (String(onoff)=="on");
        sendLampCtrl(nid, lidx, on);
        Serial.printf("btnlamp Btn%d %s\n", btn, on?"ON":"OFF");
      } else Serial.println("btnlamp: no default mapping for that button");
    } else Serial.println("usage: btnlamp <btn 1..12> <on|off>");
    return;
  }

  // btnlamptest [ms]
  if (s.startsWith("btnlamptest")){
    int ms=200; sscanf(s.c_str(),"btnlamptest %d",&ms);
    if (ms<50) ms=50; if (ms>2000) ms=2000;
    for (int b=1;b<=12;b++){
      uint8_t nid,lidx; if (!getDefaultBtnLamp((uint8_t)b, nid, lidx)) continue;
      sendLampCtrl(nid, lidx, true);  delay(ms);
      sendLampCtrl(nid, lidx, false); delay(50);
    }
    Serial.println("btnlamptest done");
    return;
  }

  // game start <seconds> <btn>
  if (s.startsWith("game start ")){
    int secs=0, btn=0;
    if (sscanf(s.c_str(),"game start %d %d",&secs,&btn)==2 && btn>=1 && btn<=5){
      gameStart((uint32_t)secs, (uint8_t)btn);
    } else {
      Serial.println("usage: game start <seconds> <btn>");
    }
    return;
  }

  // btnpins set/get
  if (s.startsWith("btnpins set ")){
    int nid; char list[64]={0};
    if (sscanf(s.c_str(),"btnpins set %d %63s",&nid,list)==2){
      uint8_t pins[3]={0,0,0}; uint8_t n=0;
      char buf[64]; strncpy(buf,list,sizeof(buf)-1); buf[sizeof(buf)-1]=0;
      char *tok = strtok(buf, ",");
      while (tok && n<3){ int p=atoi(tok); if (p>0){ pins[n++]=(uint8_t)p; } tok=strtok(NULL,","); }
      if (n==0){ Serial.println("usage: btnpins set <nodeId> <pin[,pin[,pin]]>"); return; }
      sendBtnPinsSet((uint8_t)nid, pins, n);
      Serial.printf("BTNPINS set node=%d n=%u\n", nid, n);
    } else Serial.println("usage: btnpins set <nodeId> <pin[,pin[,pin]]>");
    return;
  }
  if (s.startsWith("btnpins get ")){
    int nid; if (sscanf(s.c_str(),"btnpins get %d",&nid)==1){ sendBtnPinsReq((uint8_t)nid); }
    else Serial.println("usage: btnpins get <nodeId>");
    return;
  }

  // tofvis
  if (s.startsWith("tofvis")){
    int r=0,g=0,b=255;
    if (s=="tofvis on"){ gTofVis=true; Serial.println("ToF visualization: ON (blue)"); return; }
    if (s=="tofvis off"){ gTofVis=false; Serial.println("ToF visualization: OFF"); return; }
    if (sscanf(s.c_str(),"tofvis on %d %d %d",&r,&g,&b)==3){
      if (r<0) r=0; if (r>255) r=255;
      if (g<0) g=0; if (g>255) g=255;
      if (b<0) b=0; if (b>255) b=255;
      gTofR=(uint8_t)r; gTofG=(uint8_t)g; gTofB=(uint8_t)b; gTofVis=true;
      Serial.printf("ToF visualization: ON rgb(%d,%d,%d)\n", r,g,b);
      return;
    }
    Serial.println("usage: tofvis on|off  OR  tofvis on <r> <g> <b>");
    return;
  }

  // game end
  if (s=="game end"){ gameEnd(false); return; }

  // ---- POC: start/end using RoundCfg (nodes paint locally) ----
  if (s.startsWith("poc start")){
    int secs = 20; sscanf(s.c_str(), "poc start %d", &secs);

    gEpoch++;                 // new epoch
    G.st = PLAYING; G.targetBtn = 4; G.t0 = millis();
    G.deadlineMs = secs > 0 ? (uint32_t)secs * 1000UL : 0;

    sendGameStateBroadcast(W_PLAYING, 0,0,0);
    delay(10);

    // Walkable bitset: 39, 28, 17, 6
    uint8_t walkBits[6] = {0,0,0,0,0,0};
    walkSet(walkBits, 39); walkSet(walkBits, 28); walkSet(walkBits, 17); walkSet(walkBits, 6);

    setWalkableFromBits(walkBits);

    const uint8_t targets[1] = { 4 };   // B4

    sendRoundCfg(targets, 1, walkBits);

    Serial.printf("[POC] START epoch=%u sec=%d target=B4 path=39,28,17,6\n", gEpoch, secs);
    return;
  }

  if (s == "poc end"){
    gameEnd(false);
    Serial.println("[POC] END");
    return;
  }

  if (s=="") return;
  Serial.println("Unknown command. Type: help");
}

void loop(){
  if (gEndPending){
    gEndPending = false;
    gameEnd(gEndWin);      // this runs in loop-context (like `poc end`)
  }

  dripPump();
  flushLogs();

  // Auto-off for echo pulses
  uint32_t nowMs = millis();
  for (uint8_t b=1; b<=12; ++b){
    if (lampPulseUntil[b] && nowMs >= lampPulseUntil[b]){
      uint8_t nid,lidx;
      if (getDefaultBtnLamp(b, nid, lidx)) sendLampCtrl(nid, lidx, false);
      lampPulseUntil[b] = 0;
    }
  }

  if (G.st == PLAYING && G.deadlineMs > 0 && (millis() - G.t0) > G.deadlineMs){
    gameEnd(false);
  }

  if (Serial.available()){
    String s=Serial.readStringUntil('\n');
    handleCli(s);
  }
}
