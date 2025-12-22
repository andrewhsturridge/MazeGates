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

// Temporary kill-switch: gate segment blink on FAIL
static bool gFailGateBlinkEnabled = true;   // ← OFF for now
// Test: sweep one maze per length within each level's range
static bool     gLenSweep     = false;   // toggle via CLI
static uint8_t  gLenCursor    = 0;       // index within current level's [min..max]

static bool gTestMode = false;

// Track OTA start and completion (server-side inference)
static std::map<uint8_t, uint32_t> lastOtaStartMs;       // nodeId -> millis() when we sent OTA_START
static std::map<uint8_t, uint32_t> lastUpdateCompleteMs; // nodeId -> millis() when we inferred success

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

// +++ NEW: add intermission as a wire state
enum GameStateWire : uint8_t { W_IDLE=0, W_PLAYING=1, W_OVER=2, W_INTERMISSION=3 };

// --- Level plan (1 maze per level for now)
struct LevelPlan { uint8_t nMazes; uint16_t mazeSecs; uint8_t minLen, maxLen; };
static const LevelPlan kLevels[3] = {
  {3, 40,  4,  6},   // Easy
  {3, 30,  7, 10},   // Medium
  {99, 20, 11, 15},   // Hard
};

// ======= Lives / retry same round =======
//  - 5 lives per session (no refill)
//  - Any fail (wrong gate, wrong button, maze timeout) costs ONE life
//  - On life-loss with lives remaining: show culprit feedback (white room, blink culprit red),
//    then enter intermission (all gates blink white) and restart SAME difficulty.
static const uint8_t  kStartLives      = 5;
static const uint32_t kLifeFeedbackMs  = 4000;   // 10 toggles @ 400ms ≈ 5 red flashes

// Per-maze success flash (green) before intermission blink
static const uint32_t kSuccessFeedbackMs = 1200;

static uint8_t gLivesRemaining = kStartLives;    // reset on gameStart()
static bool    gRetrySameRound  = false;         // consumed after intermission

// Life-loss feedback is deferred (WiFi task -> loop())
static volatile bool    gLifeFeedbackPending = false;
static uint32_t         gLifeFeedbackUntil   = 0;   // 0 when not in feedback
static volatile uint8_t gLifeCulpritGate     = 0;   // 1..44 or 0
static volatile uint8_t gLifeCulpritBtn      = 0;   // 1..12 or 0

// Per-maze success feedback is deferred (like life-loss feedback)
static volatile bool    gSuccessFeedbackPending = false;
static uint32_t         gSuccessFeedbackUntil   = 0;

// NOTE: Keep the *definition* of this below the protocol structs.
// Arduino's sketch preprocessor may auto-generate function prototypes near the top of the file;
// if a function is defined before the protocol structs, those auto-prototypes can end up before
// the type declarations and cause "does not name a type" errors.
static void handleFailWithLives(uint8_t culpritGate, uint8_t culpritBtn);

// --- Run context
static uint32_t gGameDeadlineMs = 0;        // session absolute end (win if survive X minutes)
static uint16_t gStage = 0;                 // increments on each successful maze (no advance on life loss)
static bool     gAdvanceStagePending = false;
static uint32_t gIntermissionUntil = 0;     // 0 when not in intermission


// Culprit tracking for fail flash
static uint8_t  gFailGate = 0;              // nonzero => wrong gate
static uint8_t  gFailBtn  = 0;              // nonzero => wrong button

// Simple blink scheduler for fail flash
static uint32_t gFailBlinkUntil = 0;
static uint32_t gFailBlinkNext  = 0;
static bool     gFailBlinkOn    = false;

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
static void sendGameStateToAll(uint8_t state, uint8_t r, uint8_t g, uint8_t b, int repeats=3, int gap_ms=3){
  GameStateMsg m{}; m.h={GAME_STATE,PROTO_VER,0,gEpoch,gSeq++,(uint16_t)sizeof(GameStateMsg)};
  m.state=state; m.r=r; m.g=g; m.b=b;
  for(int rep=0; rep<repeats; ++rep){
    sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m));
    for (auto &kv : nodesById) sendRaw(kv.second.mac,(uint8_t*)&m,sizeof(m));
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

  // NEW: remember when this OTA was triggered
  lastOtaStartMs[nodeId] = millis();
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

static inline void clearBits(uint8_t wb[6]){ memset(wb, 0, 6); }
static inline void setBit(uint8_t wb[6], uint8_t gate){ if (gate>=1 && gate<=44){ uint8_t i=(gate-1)>>3, b=(gate-1)&7; wb[i] |= (1u<<b); } }

// ---- Path generator helpers (place ABOVE startMaze) ----

// Is b 4-adjacent to a ?
static inline bool isAdjGate(uint8_t a, uint8_t b){
  const Adj &ad = GADJ[a];
  for (uint8_t i=0;i<ad.n;i++) if (ad.v[i] == b) return true;
  return false;
}

// Fisher–Yates shuffle for small neighbor arrays
static inline void shuffle(uint8_t* a, uint8_t n){
  for (uint8_t i=0;i<n; ++i){
    uint8_t j = i + (uint8_t)random(n - i);
    uint8_t t = a[i]; a[i] = a[j]; a[j] = t;
  }
}

// Depth-first search with "self-avoid" rule:
//  - no repeats
//  - next step cannot be 4-adjacent to ANY earlier place (except current)
static bool dfsPathNoTouch(uint8_t at,
                           const Adj *endSet,
                           uint8_t Lmin, uint8_t Lmax,
                           bool used[45],
                           uint8_t path[46], uint8_t len,
                           uint8_t outPath[46], uint8_t *outLen)
{
  // If within band and at an end gate (adjacent to chosen button), accept
  if (len >= Lmin){
    for (uint8_t i=0; i<endSet->n; ++i){
      if (at == endSet->v[i]){
        memcpy(outPath, path, len);
        *outLen = len;
        return true;
      }
    }
  }
  if (len == Lmax) return false;

  // Neighbor order (randomized each call)
  uint8_t nn = GADJ[at].n;
  uint8_t order[6];
  for (uint8_t i=0;i<nn;i++) order[i] = GADJ[at].v[i];
  shuffle(order, nn);

  for (uint8_t i=0;i<nn;i++){
    uint8_t nb = order[i];
    if (used[nb]) continue;

    // Self-avoid: nb must NOT touch any earlier place except the current (path[len-1]==at)
    bool touchesOld = false;
    for (uint8_t k=0; k + 1 < len; ++k){    // check path[0..len-2]
      if (isAdjGate(nb, path[k])){ touchesOld = true; break; }
    }
    if (touchesOld) continue;

    used[nb] = true;
    path[len] = nb;
    if (dfsPathNoTouch(nb, endSet, Lmin, Lmax, used, path, len+1, outPath, outLen)) return true;
    used[nb] = false;
  }
  return false;
}

// Public wrapper: produce a simple, self-avoiding path in [Lmin..Lmax]
static bool genPathSimple(uint8_t start, const Adj& endSet, uint8_t Lmin, uint8_t Lmax,
                          uint8_t outPath[46], uint8_t &outLen)
{
  bool    used[45] = {0};
  uint8_t path[46];
  path[0] = start;
  used[start] = true;
  outLen = 0;
  return dfsPathNoTouch(start, &endSet, Lmin, Lmax, used, path, /*len=*/1, outPath, &outLen);
}

// Branch-path generator for "tree" mazes (2 targets).
// Produces a self-avoiding branch (no-touch within the branch). Optionally also avoids
// being 4-adjacent to ANY trunk gate except the current gate (strict spacing).
static bool dfsBranchNoTouch(uint8_t at,
                             const Adj *endSet,
                             uint8_t Lmin, uint8_t Lmax,
                             const uint8_t *trunk, uint8_t trunkLen,
                             bool used[45],
                             uint8_t path[46], uint8_t len,
                             uint8_t outPath[46], uint8_t *outLen,
                             bool avoidTouchToTrunk)
{
  // Accept when within band and at an end gate (adjacent to chosen button)
  if (len >= Lmin){
    for (uint8_t i=0; i<endSet->n; ++i){
      if (at == endSet->v[i]){
        memcpy(outPath, path, len);
        *outLen = len;
        return true;
      }
    }
  }
  if (len == Lmax) return false;

  uint8_t nn = GADJ[at].n;
  uint8_t order[6];
  for (uint8_t i=0;i<nn;i++) order[i] = GADJ[at].v[i];
  shuffle(order, nn);

  for (uint8_t i=0;i<nn;i++){
    uint8_t nb = order[i];
    if (used[nb]) continue;

    // Self-avoid within the branch: nb must NOT touch any earlier branch place (except current)
    bool touchesOld = false;
    for (uint8_t k=0; k + 1 < len; ++k){
      if (isAdjGate(nb, path[k])){ touchesOld = true; break; }
    }
    if (touchesOld) continue;

    // Optional: keep at least one-gate spacing from the trunk, to make the maze visually "tree-like".
    if (avoidTouchToTrunk){
      for (uint8_t k=0; k<trunkLen; ++k){
        uint8_t tg = trunk[k];
        if (tg == at) continue;                 // allow adjacency to the current gate (needed at the branch point)
        if (isAdjGate(nb, tg)){ touchesOld = true; break; }
      }
      if (touchesOld) continue;
    }

    used[nb] = true;
    path[len] = nb;
    if (dfsBranchNoTouch(nb, endSet, Lmin, Lmax, trunk, trunkLen, used, path, len+1, outPath, outLen, avoidTouchToTrunk))
      return true;
    used[nb] = false;
  }
  return false;
}

static bool genBranchPath(uint8_t branchStart, const Adj& endSet,
                          uint8_t Lmin, uint8_t Lmax,
                          const uint8_t trunkPath[46], uint8_t trunkLen,
                          uint8_t outPath[46], uint8_t &outLen,
                          bool avoidTouchToTrunk)
{
  bool used[45] = {0};

  // Block overlap with the trunk (tree = trunk ∪ branch; they may share ONLY the branch point)
  for (uint8_t i=0; i<trunkLen; ++i){
    uint8_t g = trunkPath[i];
    if (g>=1 && g<=44) used[g] = true;
  }

  uint8_t path[46];
  path[0] = branchStart;
  outLen  = 0;

  return dfsBranchNoTouch(branchStart, &endSet, Lmin, Lmax,
                          trunkPath, trunkLen,
                          used, path, /*len=*/1,
                          outPath, &outLen,
                          avoidTouchToTrunk);
}

// Build walkBits from a path
static void buildWalkBitsFromPath(const uint8_t* path, uint8_t n, uint8_t wb[6]){
  clearBits(wb);
  for (uint8_t i=0;i<n;i++) setBit(wb, path[i]);
}

// ======= Gate events =======
static bool gTofVis=false; static uint8_t gTofR=0, gTofG=0, gTofB=255;

enum GameState { WAITING=0, PLAYING=1, GAME_OVER=2 };

struct Round {
  GameState st;

  uint8_t  nTargets;        // 0..12
  uint8_t  targets[12];     // global buttons (1..12)
  uint16_t targetsMask;     // bit i => button i required (1..12)
  uint16_t hitMask;         // bit i => button i already hit

  uint32_t t0;
  uint32_t deadlineMs;
};

static Round G = { WAITING, 0, {0}, 0, 0, 0, 0 };

static inline uint16_t btnBit(uint8_t b){
  return (b >= 1 && b <= 12) ? (uint16_t)(1u << b) : 0;
}

static uint8_t firstRemainingTargetBtn(){
  for (uint8_t i=0; i<G.nTargets && i<12; ++i){
    uint8_t b = G.targets[i];
    uint16_t bit = btnBit(b);
    if (bit && !(G.hitMask & bit)) return b;
  }
  return 0;
}


// Defer GAME_OVER out of onNowRecv
static volatile bool gEndPending = false;
static volatile bool gEndWin     = false;

static inline void scheduleGameEnd(bool win){
  if (G.st == PLAYING) G.st = GAME_OVER;  // freeze server logic immediately
  gEndWin     = win;
  gEndPending = true;
}

// Lives helper (definition intentionally placed after protocol structs)
static void handleFailWithLives(uint8_t culpritGate, uint8_t culpritBtn){
  if (culpritGate > 44) culpritGate = 0;
  if (culpritBtn  > 12) culpritBtn  = 0;

  if (gLivesRemaining > 1){
    gLivesRemaining--;
    gRetrySameRound = true;                 // do NOT advance level/length after intermission
    gLifeCulpritGate = culpritGate;
    gLifeCulpritBtn  = culpritBtn;
    gLifeFeedbackPending = true;            // loop() will start the feedback visuals
    Serial.printf("[LIFE] Lost one. Lives left=%u (retry same difficulty)\n", gLivesRemaining);
  } else {
    gLivesRemaining = 0;
    Serial.println("[LIFE] No lives left -> GAME OVER");
    scheduleGameEnd(false);
  }
}

static void enterIntermission(){
  G.st = WAITING;                         // stop processing gates
  gIntermissionUntil = millis() + 5000;   // 5 s
  setAllLamps(false);                     // keep buttons dark while players reset to safe positions
  sendGameStateToAll(W_INTERMISSION, 0,0,0);
  Serial.println("[MAZE] INTERMISSION 5s");
}

static void beginFailFlashGate(uint8_t gate){
  gFailGate = gate; gFailBtn = 0;
  gFailBlinkUntil = millis() + 5000;   // 5s
  gFailBlinkNext  = 0; gFailBlinkOn = false;
}
static void beginFailFlashButton(uint8_t btn){
  gFailGate = 0; gFailBtn = btn;
  gFailBlinkUntil = millis() + 5000;
  gFailBlinkNext  = 0; gFailBlinkOn = false;
}
static void tickFailFlash(){
  // Run culprit blink both for GAME_OVER and for life-loss feedback stage.
  if (G.st != GAME_OVER && gLifeFeedbackUntil == 0 && !gEndPending){
    gFailBlinkUntil = 0;
    gFailGate = 0;
    gFailBtn  = 0;
    return;
  }
  // If we ever have a gate culprit but blink is disabled, drop it
  if (!gFailGateBlinkEnabled && gFailGate){
    gFailGate = 0;
    gFailBlinkUntil = 0;
    // continue so button blink (if any) can run
  }

  if (!gFailBlinkUntil) return;
  uint32_t now = millis();
  if (now >= gFailBlinkUntil){ gFailBlinkUntil=0; gFailGate=0; gFailBtn=0; return; }
  if (now < gFailBlinkNext) return;
  gFailBlinkNext = now + 400;     // 2.5 Hz toggle
  gFailBlinkOn = !gFailBlinkOn;

  if (gFailGate){
    uint8_t nid, strip; uint16_t start, count;
    if (routeGate(gFailGate, nid, strip, start, count)){
      LedRangeMsg m{}; m.h={LED_RANGE,PROTO_VER,0,gEpoch,gSeq++,(uint16_t)sizeof(LedRangeMsg)};
      m.strip=strip; m.start=start; m.count=count; m.effect=1;        // overlay frame
      if (gFailBlinkOn){ m.r=150; m.g=0;   m.b=0;   } else { m.r=150; m.g=150; m.b=150; } // red <-> white
      auto it = nodesById.find(nid); if (it != nodesById.end()) _enqueueTx(it->second.mac,&m,sizeof(m));
      flushTxQueueQuickly(60);                                        // <— ensure it gets out NOW
    } else {
      Serial.printf("[FAIL] routeGate(G%u) failed; no blink\n", gFailGate);
    }
  } else if (gFailBtn){
    uint8_t nid,lidx; if (getDefaultBtnLamp(gFailBtn, nid, lidx)){
      sendLampCtrl(nid, lidx, gFailBlinkOn);
    }
  }
}

static void processGateEvent(uint8_t gateId, uint8_t ev){
  // When NOT playing, honor ToF visualization by sending an overlay frame (effect=1)
  // so nodes accept LED_RANGE in any state. We only paint on ENTER events.
  // (If you don't want tofvis during GAME_OVER, change the condition to:
  //   if (G.st != PLAYING && G.st != GAME_OVER) { ... } )
  if (G.st != PLAYING){
    if (gTofVis && ev == 1){
      uint8_t nodeId, strip; uint16_t start, count;
      if (routeGate(gateId, nodeId, strip, start, count)){
        LedRangeMsg m{};
        m.h.type    = LED_RANGE;
        m.h.version = PROTO_VER;
        m.h.nodeId  = 0;
        m.h.pad     = gEpoch;
        m.h.seq     = gSeq++;
        m.h.len     = (uint16_t)sizeof(LedRangeMsg);

        m.strip     = strip;
        m.start     = start;
        m.count     = count;
        m.effect    = 1;                 // << overlay so node accepts outside PLAYING
        m.r         = gTofR;
        m.g         = gTofG;
        m.b         = gTofB;
        m.durationMs= 0;

        auto it = nodesById.find(nodeId);
        if (it != nodesById.end()){
          _enqueueTx(it->second.mac, &m, sizeof(m));
        }
      }
    }
    return;
  }

  // PLAYING: only ENTER matters, strict path enforcement
  if (ev == 1){
    if (!isWalkable(gateId)){           // Wrong gate -> lose a life
      gFailGate = gateId;
      gFailBtn  = 0;
      G.st = WAITING;                   // freeze server logic immediately
      handleFailWithLives(gateId, 0);
      return;
    }
  }

  // Paint this gate segment in green if walkable, red otherwise
  uint8_t nodeId, strip; uint16_t start, count;
  if (!routeGate(gateId, nodeId, strip, start, count)) return;
  const bool ok = isWalkable(gateId);
  sendLedRange(nodeId, strip, start, count, ok ? 0 : 150, ok ? 150 : 0, 0);
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
// change signature you already adopted:
static void gameStart(uint32_t seconds){

  gLenCursor = 0;            // length-sweep cursor (only used when 'test lengths on')

  // Full reset
  gStage = 0;
  gAdvanceStagePending = false;
  gIntermissionUntil = 0;

// Lives reset (no refill during a session)
  gLivesRemaining = kStartLives;
  gRetrySameRound = false;
  gLifeFeedbackPending = false;
  gLifeFeedbackUntil   = 0;
  gLifeCulpritGate = 0;
  gLifeCulpritBtn  = 0;

  gSuccessFeedbackPending = false;
  gSuccessFeedbackUntil   = 0;

  gFailGate = 0; gFailBtn = 0;
  gFailBlinkUntil = 0; gFailBlinkNext = 0; gFailBlinkOn = false;

  gEndPending = false;          // <— ensure no deferred end from prior run
  gEndWin     = false;          // <— clear win/lose latch too

  txQClear();                       // drop any queued fail-flash overlay frames
  for (uint8_t b=1; b<=12; ++b)     // cancel any lamp echo pulses
    lampPulseUntil[b] = 0;

  if (seconds == 0) seconds = 300;
  gGameDeadlineMs = millis() + seconds * 1000UL;

  Serial.printf("[GAME] START %lus (stage progression)\n", (unsigned)seconds);
  startMaze();
}

// in gameEnd(bool win)
static void gameEnd(bool win){
  // Cancel any in-flight life-feedback sequence
  gLifeFeedbackPending = false;
  gLifeFeedbackUntil   = 0;
  gRetrySameRound      = false;
  gAdvanceStagePending = false;
  gIntermissionUntil   = 0;

  // Cancel any in-flight success feedback
  gSuccessFeedbackPending = false;
  gSuccessFeedbackUntil   = 0;

  G.st = GAME_OVER;
  G.deadlineMs = 0;
  gGameDeadlineMs = 0;                 // stop global timer
  setAllLamps(false);

  uint8_t R = win ? 0 : 150, Gc = win ? 150 : 0;
  sendGameStateToAll(W_OVER, R, Gc, 0);

  if (!win){
    // Gate blink disabled (room is already red). Keep button-blink if needed.
    if (gFailBtn) {
      beginFailFlashButton(gFailBtn);  // lamp blink still ok
    }
    // ensure we don't carry a gate blink forward
    gFailGate = 0;
    gFailBlinkUntil = 0; gFailBlinkNext = 0; gFailBlinkOn = false;
  }

  Serial.printf("[GAME] END (%s) epoch=%u\n", win ? "WIN" : "FAIL", gEpoch);
}

static bool startMaze(){
  // Reset any fail-blink state between mazes
  gFailGate = 0; gFailBtn  = 0;
  gFailBlinkUntil = 0; gFailBlinkNext = 0; gFailBlinkOn = false;

  // ---- Stage-based progression ----
  //  - Each success: stage++ (handled after intermission)
  //  - main path length increases with stage (clamped)
  //  - maze time decreases slightly with stage (clamped)
  //  - once length hits the "hard" band, introduce 2-target tree mazes (order doesn't matter)
  uint8_t  mainLen  = 0;     // desired trunk length (gates, includes start)
  uint16_t mazeSecs = 0;
  uint8_t  wantTargets = 1;
  uint8_t  branchLen = 0;    // desired branch length (gates, includes branch point)

  if (gLenSweep){
    // Test tool: sweep exact lengths 4..15 with the legacy time buckets
    mainLen = (uint8_t)(4 + gLenCursor);
    if (mainLen < 4)  mainLen = 4;
    if (mainLen > 15) mainLen = 15;
    mazeSecs = (mainLen <= 6) ? 40 : (mainLen <= 10) ? 30 : 20;
  } else {
    uint16_t rawLen = (uint16_t)4 + gStage;
    mainLen = (rawLen > 15) ? 15 : (uint8_t)rawLen;

    int t = 40 - (int)gStage;     // 1s less per stage (clamped)
    if (t < 20) t = 20;
    mazeSecs = (uint16_t)t;
  }

  wantTargets = (mainLen >= 11) ? 2 : 1;  // introduce multi-target once the trunk is long
  branchLen = (uint8_t)(mainLen / 2);
  if (branchLen < 4) branchLen = 4;
  if (branchLen > 7) branchLen = 7;

  // ---- Generate maze ----
  uint8_t start = 0;

  uint8_t targets[12] = {0};
  uint8_t nTargets = 0;

  uint8_t mainPath[46];   uint8_t mainPlen   = 0;
  uint8_t branchPath[46]; uint8_t branchPlen = 0;

  uint8_t wb[6]; clearBits(wb);

  bool ok = false;

  // 2-target tree: trunk reaches target A, branch reaches target B.
  if (wantTargets == 2){
    for (int tries=0; tries<650 && !ok; ++tries){
      start = (uint8_t)random(39, 45);

      uint8_t btnA = (uint8_t)random(1, 13);
      uint8_t btnB = (uint8_t)random(1, 13);
      if (btnB == btnA) continue;

      // Trunk: exact length
      if (!genPathSimple(start, BADJ[btnA], mainLen, mainLen, mainPath, mainPlen)) continue;
      if (mainPlen < 4) continue;  // need interior nodes to branch cleanly

      // Try multiple branch points along the trunk (exclude the trunk end gate)
      for (int bpTry=0; bpTry<140 && !ok; ++bpTry){
        uint8_t idx = (uint8_t)random(1, (int)mainPlen - 1);  // 1..mainPlen-2
        if (idx == 0 || idx >= mainPlen-1) continue;
        uint8_t bp = mainPath[idx];

        uint8_t bMin = branchLen;
        uint8_t bMax = (uint8_t)(branchLen + 1);
        if (bMax > 15) bMax = 15;

        // Branch generator: strict trunk spacing first, then relaxed as fallback
        bool okBr = genBranchPath(bp, BADJ[btnB], bMin, bMax, mainPath, mainPlen, branchPath, branchPlen, true);
        if (!okBr) okBr = genBranchPath(bp, BADJ[btnB], bMin, bMax, mainPath, mainPlen, branchPath, branchPlen, false);
        if (!okBr) continue;

        // Union bits trunk + branch
        clearBits(wb);
        for (uint8_t i=0;i<mainPlen;i++)   setBit(wb, mainPath[i]);
        for (uint8_t i=0;i<branchPlen;i++) setBit(wb, branchPath[i]);

        targets[0] = btnA;
        targets[1] = btnB;
        nTargets   = 2;
        ok         = true;
      }
    }
  }

  // 1-target fallback / default
  if (!ok){
    for (int tries=0; tries<500 && !ok; ++tries){
      start = (uint8_t)random(39, 45);
      uint8_t btn = (uint8_t)random(1, 13);
      if (genPathSimple(start, BADJ[btn], mainLen, mainLen, mainPath, mainPlen)){
        buildWalkBitsFromPath(mainPath, mainPlen, wb);
        targets[0] = btn;
        nTargets   = 1;
        ok         = true;
      }
    }
  }

  // Final fallback: relax length band ±1 if the exact-length generator struggled
  if (!ok){
    uint8_t Lmin = (mainLen > 4)   ? (uint8_t)(mainLen - 1) : mainLen;
    uint8_t Lmax = (mainLen < 15)  ? (uint8_t)(mainLen + 1) : mainLen;

    for (int tries=0; tries<600 && !ok; ++tries){
      start = (uint8_t)random(39, 45);
      uint8_t btn = (uint8_t)random(1, 13);
      if (genPathSimple(start, BADJ[btn], Lmin, Lmax, mainPath, mainPlen)){
        buildWalkBitsFromPath(mainPath, mainPlen, wb);
        targets[0] = btn;
        nTargets   = 1;
        ok         = true;
      }
    }
  }

  if (!ok){
    Serial.println("[MAZE] generator failed");
    return false;
  }

  setWalkableFromBits(wb);

  // Epoch bump so nodes accept fresh config
  gEpoch++;

  // Flip nodes to PLAYING and send round config
  sendGameStateToAll(W_PLAYING, 0,0,0);
  delay(10);
  sendRoundCfg(targets, nTargets, wb);

  // Arm the maze timer + targets
  G.st = PLAYING;
  G.nTargets = nTargets;
  memset(G.targets, 0, sizeof(G.targets));
  for (uint8_t i=0;i<nTargets && i<12;i++) G.targets[i] = targets[i];
  G.targetsMask = 0;
  for (uint8_t i=0;i<nTargets && i<12;i++) G.targetsMask |= btnBit(G.targets[i]);
  G.hitMask = 0;

  G.t0 = millis();
  G.deadlineMs = (uint32_t)mazeSecs * 1000UL;

  // Log
  Serial.printf("[MAZE] stage=%u len=%u time=%us%s start=G%u targets=",
                (unsigned)gStage, (unsigned)mainPlen, (unsigned)mazeSecs,
                gLenSweep ? " (sweep)" : "", start);

  for (uint8_t i=0;i<nTargets;i++){
    Serial.printf("%sB%u", (i ? "," : ""), targets[i]);
  }
  if (nTargets == 2){
    Serial.printf(" (tree: trunk=%u branch=%u)", (unsigned)mainPlen, (unsigned)branchPlen);
  }
  Serial.println();

  return true;
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

  // Auto-learn any node that talks to us (not just HELLO), so unicast LED_RANGE/LAMP_CTRL
  // always works even if a node missed HELLO_REQ at boot.
  if (h->nodeId != 0 && nodesById.find(h->nodeId) == nodesById.end()){
    addOrUpdateNode(h->nodeId, mac);
    Serial.printf("DISCOVER node %u (%s) via msg=%u\n", h->nodeId, macToStr(mac).c_str(), h->type);
  }

  if (h->type==HELLO){
    uint8_t nid=h->nodeId; addOrUpdateNode(nid, mac);
    Serial.printf("HELLO from node %u (%s)\n", nid, macToStr(mac).c_str());
  }
  else if (h->type==GATE_EVENT && len >= (int)sizeof(GateEventMsg)){
    if (h->pad != gEpoch && !gTestMode && !gTofVis) {
      Serial.printf("GATE (stale epoch %u != %u) ignored\n", h->pad, gEpoch);
      return;
    }
    auto *m=(const GateEventMsg*)data;
    Serial.printf("GATE %u %s mm=%u from node %u\n",
                  m->gateId,(m->ev==1?"ENTER":(m->ev==2?"EXIT":"?")),m->strengthMm,h->nodeId);
    processGateEvent(m->gateId, m->ev);
  }
  else if (h->type==OTA_ACK && len>=(int)sizeof(OtaAckMsg)){
    auto *m=(const OtaAckMsg*)data;
    Serial.printf("OTA_ACK from node %u (status=%u)\n", h->nodeId, m->status);
  }
  else if (h->type==BUTTON_EVENT && len >= (int)sizeof(ButtonEventMsg)){
    if (h->pad != gEpoch && !gTestMode && !gBtnEcho) {
      Serial.printf("BUTTON (stale epoch %u != %u) ignored\n", h->pad, gEpoch);
      return;
    }
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
      if (globalB > 0){
        uint16_t bit = btnBit((uint8_t)globalB);

        if (bit && (G.targetsMask & bit)){
          // Correct target (order doesn't matter). Turn it off once it's collected.
          if (!(G.hitMask & bit)){
            G.hitMask |= bit;

            uint8_t nid,lidx;
            if (getDefaultBtnLamp((uint8_t)globalB, nid, lidx)){
              sendLampCtrl(nid, lidx, false);
            }
          }

          // All targets hit -> success
          if (G.targetsMask && ((G.hitMask & G.targetsMask) == G.targetsMask)){
            gAdvanceStagePending = true;
            G.st = WAITING;                 // freeze server logic immediately
            gSuccessFeedbackPending = true; // loop() will show green success then start intermission
          }
        } else {
          // Wrong button -> lose a life
          gFailGate = 0;
          gFailBtn  = (uint8_t)globalB;
          G.st = WAITING;                 // freeze server logic immediately
          handleFailWithLives(0, gFailBtn);
        }
      } else {
        // Unmapped button -> lose a life (no specific culprit button)
        gFailGate = 0;
        gFailBtn  = 0;
        G.st = WAITING;
        handleFailWithLives(0, 0);
      }
    }
  }
  else if (h->type == NODE_STATUS && len >= (int)sizeof(NodeStatusMsg)) {
    const NodeStatusMsg* m = (const NodeStatusMsg*)data;
    lastStatus[h->nodeId] = { m->uptimeMs, m->initedMask, m->errStreakMax, {0}, millis() };
    memcpy(lastStatus[h->nodeId].reinitCount, m->reinitCount, 8);
    logNodeStatus(h->nodeId, m);

    // NEW: infer update completion if the node rebooted shortly after an OTA_START we sent
    uint32_t nowMs = millis();
    auto itStart = lastOtaStartMs.find(h->nodeId);
    if (itStart != lastOtaStartMs.end()){
      bool freshBoot = (m->uptimeMs <= 5000);              // 5s since boot = just restarted
      bool withinWnd = (nowMs - itStart->second) <= 120000; // 2 min window after OTA start
      if (freshBoot && withinWnd){
        lastUpdateCompleteMs[h->nodeId] = nowMs;           // mark completion time
        // optional: clear start marker so we don’t re-mark on next status
        lastOtaStartMs.erase(itStart);
        qlogf("UPDATE COMPLETE inferred for node %u", h->nodeId);
      }
    }
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
  Serial.println("  game start [seconds]");
  Serial.println("  game end");
  Serial.println("  lamp <nodeId> <idx> <on|off>");
  Serial.println("  ledmap show [nodeId]");
  Serial.println("  ledmap get <nodeId> | ledmap get all");
  Serial.println("  ledmap <nodeId> <pin:count>[,<pin:count>...]");
  Serial.println("  ota <nodeId> [url]");
  Serial.println("  ota all [url]");
  Serial.println("    sample url: http://172.20.10.3:8000/MazeGates/MazeGates_Node/build/esp32.esp32.um_feathers3/MazeGates_Node.ino.bin");
  Serial.println("  status");
  Serial.println("  tofvis on|off  (optional: tofvis on <r> <g> <b>)");
  Serial.println("  tofmap get <nodeId> | tofmap get all");
  Serial.println("  tofmap set <nodeId> g0,g1,g2,g3,g4,g5,g6,g7");
  Serial.println("  poc start <seconds>   (B4 target; walkable: 39,28,17,6)");
  Serial.println("  poc end");
  Serial.println("  test lengths on|off");
  Serial.println("  test on | test off");
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

  if (s=="test lengths on"){  gLenSweep = true;  Serial.println("test lengths: ON");  return; }
  if (s=="test lengths off"){ gLenSweep = false; Serial.println("test lengths: OFF"); return; }

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
    Serial.println("node  uptime(s)  inited  maxErr  reinitCounts                 lastUpdate(s ago)");
    for (auto &kv : lastStatus) {
      auto nid = kv.first; const auto &st = kv.second;
      uint32_t nowMs = millis();
      int32_t ageSec = -1;
      auto it = lastUpdateCompleteMs.find(nid);
      if (it != lastUpdateCompleteMs.end()){
        uint32_t ageMs = nowMs - it->second;
        ageSec = (int32_t)(ageMs / 1000UL);
      }

      Serial.printf("%3u  %9lu   0x%02X     %3u   [",
                    nid, (unsigned)(st.uptimeMs/1000), st.initedMask, st.errStreakMax);
      for (int i=0;i<8;i++) { Serial.printf("%u%s", st.reinitCount[i], i==7?"]   ":" ,"); }

      if (ageSec >= 0) Serial.printf("%6d", ageSec);
      else             Serial.printf("     -");
      Serial.println();
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
  if (s.startsWith("game start")){
    unsigned secs = 300;                    // default 5:00
    sscanf(s.c_str(), "game start %u", &secs);
    gameStart((uint32_t)secs);              // new signature below
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
    if (s=="tofvis on"){
      gTofVis=true;
      sendGameStateBroadcast(W_PLAYING, 0,0,0); // nodes sense during tofvis
      Serial.println("ToF visualization: ON (blue) [nodes set to PLAYING]");
      return;
    }
    if (s=="tofvis off"){ gTofVis=false; Serial.println("ToF visualization: OFF"); return; }
    if (sscanf(s.c_str(),"tofvis on %d %d %d",&r,&g,&b)==3){
      if (r<0) r=0; if (r>255) r=255;
      if (g<0) g=0; if (g>255) g=255;
      if (b<0) b=0; if (b>255) b=255;
      gTofR=(uint8_t)r; gTofG=(uint8_t)g; gTofB=(uint8_t)b; gTofVis=true;
      sendGameStateBroadcast(W_PLAYING, 0,0,0);
      Serial.printf("ToF visualization: ON rgb(%d,%d,%d) [nodes set to PLAYING]\n", r,g,b);
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
    G.st = PLAYING;
    G.nTargets = 1;
    memset(G.targets, 0, sizeof(G.targets));
    G.targets[0] = 4;
    G.targetsMask = btnBit(4);
    G.hitMask = 0;
    G.t0 = millis();
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

  if (s=="test on"){
    gTestMode = true;
    gBtnEcho  = true;   // convenience: lamp pulses on press
    gTofVis   = true;   // convenience: enable segment paint
    sendGameStateBroadcast(W_PLAYING, 0,0,0);  // nodes start sensing (node PLAYING)
    Serial.println("[TEST] ON: nodes PLAYING; epoch bypass for TOF (and buttons via echo)");
    return;
  }

  if (s=="test off"){
    gTestMode = false;
    gBtnEcho  = false;
    gTofVis   = false;
    sendGameStateBroadcast(W_IDLE, 0,0,0);     // nodes stop sensing
    Serial.println("[TEST] OFF");
    return;
  }

  if (s=="") return;
  Serial.println("Unknown command. Type: help");
}

void loop(){
  // Finish a deferred end (we already added this scheduler earlier)
  if (gEndPending){ gEndPending=false; gameEnd(gEndWin); }

  // Life-loss feedback (white room + blink culprit red) is started here (NOT in esp_now callback)
  if (gLifeFeedbackPending){
    gLifeFeedbackPending = false;

    // Base coat: all gates solid white (culprit will blink red over it)
    sendGameStateToAll(W_IDLE, 150,150,150);

    // Turn off all target lamps during feedback (culprit button, if any, will be blinked by tickFailFlash)
    setAllLamps(false);

    // Arm culprit blink (tickFailFlash will do the toggling)
    gFailGate = gLifeCulpritGate;
    gFailBtn  = gLifeCulpritBtn;

    gFailBlinkUntil = millis() + kLifeFeedbackMs;
    gLifeFeedbackUntil = gFailBlinkUntil;

    gFailBlinkNext = 0;
    gFailBlinkOn   = false;  // first toggle => ON (red / lamp on)

    Serial.printf("[LIFE] Feedback: culprit gate=%u btn=%u (%lums)\n",
                  gFailGate, gFailBtn, (unsigned long)kLifeFeedbackMs);
  }

  // Maze success feedback (solid green) is started here (NOT in esp_now callback)
  if (gSuccessFeedbackPending && !gLifeFeedbackUntil && !gIntermissionUntil && !gEndPending){
    gSuccessFeedbackPending = false;

    // Stop any blink state just in case
    gFailBlinkUntil = 0;
    gFailGate = 0;
    gFailBtn  = 0;
    gFailBlinkNext = 0;
    gFailBlinkOn   = false;

    // Solid green "success" flash
    sendGameStateToAll(W_IDLE, 0, 150, 0);
    setAllLamps(false);

    gSuccessFeedbackUntil = millis() + kSuccessFeedbackMs;
    Serial.printf("[MAZE] SUCCESS feedback (%lums)\n", (unsigned long)kSuccessFeedbackMs);
  }

  // Feedback finished -> intermission (all-gates white blink), then restart same difficulty
  if (gLifeFeedbackUntil && (int32_t)(millis() - gLifeFeedbackUntil) >= 0){
    gLifeFeedbackUntil = 0;

    // stop blinking
    gFailBlinkUntil = 0;
    gFailGate = 0;
    gFailBtn  = 0;
    gFailBlinkNext = 0;
    gFailBlinkOn   = false;

    enterIntermission();
  }

  // Success feedback finished -> intermission (all-gates white blink), then advance stage
  if (gSuccessFeedbackUntil && (int32_t)(millis() - gSuccessFeedbackUntil) >= 0){
    gSuccessFeedbackUntil = 0;
    if (!gIntermissionUntil) enterIntermission();
  }

  // Intermission auto-advance
  if (gIntermissionUntil && (int32_t)(millis() - gIntermissionUntil) >= 0){
    gIntermissionUntil = 0;

    if (gRetrySameRound){
      // Life was lost -> restart SAME difficulty (no progression)
      gRetrySameRound = false;
      gAdvanceStagePending = false;   // safety: don't accidentally advance
    } else {
      // Maze success -> stage++ (unless we're in length-sweep test mode)
      if (!gLenSweep && gAdvanceStagePending){
        gStage++;
      }
      gAdvanceStagePending = false;

      // Test tool: length sweep 4..15 (do not touch stage)
      if (gLenSweep){
        const uint8_t count = (uint8_t)(15 - 4 + 1);  // 12 lengths
        if (++gLenCursor >= count) gLenCursor = 0;
      }
    }

    startMaze();
  }

  // Game hard timer -> win
  if (gGameDeadlineMs && (int32_t)(millis() - gGameDeadlineMs) >= 0){
    gGameDeadlineMs = 0; scheduleGameEnd(true);
  }

  // Maze timer (strict -> fail)
  if (G.st == PLAYING && G.deadlineMs > 0 && (millis() - G.t0) > G.deadlineMs){
    // Timeout costs a life. Treat the target button as the "culprit" (you didn't hit it in time).
    gFailGate = 0;
    gFailBtn  = firstRemainingTargetBtn();
    G.st = WAITING;                 // freeze server logic immediately
    handleFailWithLives(0, gFailBtn);
  }

  // Drive fail-blink if active
  tickFailFlash();

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

  if (Serial.available()){
    String s=Serial.readStringUntil('\n');
    handleCli(s);
  }
}
