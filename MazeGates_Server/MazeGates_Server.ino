/*
 * Maze Gates – Server Harness v0 (clean compile)
 * Board: ESP32 (any), Transport: ESP‑NOW ch.6
 * Reuses message types from Node4; adds roster, lamp, ota, setgate, and fakegate <gateId> <enter|exit>.
 * NOTE: This harness routes Node 4 (L9/L10) with mixed 45/50‑px segments.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <map>
#include <vector>
#include <cstring>

// For Node 4 only (so we can show gates_down as well as ch_down)
static const uint8_t GATE_BY_CH_NODE4[8] = { 23, 24, 28, 29, 34, 35, 39, 40 };

// ======= Protocol (same as node) =======
enum MsgType : uint8_t { HELLO=1, HELLO_REQ=2, CLAIM=3, GATE_EVENT=10, LED_RANGE=20, LAMP_CTRL=21, BUTTON_EVENT=30, OTA_START=50, OTA_ACK=51, NODE_STATUS = 60, LED_MAP = 62 };
struct __attribute__((packed)) PktHeader { uint8_t type, version, nodeId, pad; uint16_t seq, len; };
static const uint8_t PROTO_VER = 1;
struct __attribute__((packed)) HelloMsg { PktHeader h; uint8_t role; uint8_t caps; };
struct __attribute__((packed)) ClaimMsg { PktHeader h; uint8_t newNodeId; };
struct __attribute__((packed)) GateEventMsg { PktHeader h; uint8_t gateId; uint8_t ev; uint16_t strengthMm; uint32_t tsMs; };
struct __attribute__((packed)) ButtonEventMsg { PktHeader h; uint8_t btnIdx; uint8_t ev; uint32_t tsMs; };
struct __attribute__((packed)) LedRangeMsg { PktHeader h; uint8_t strip; uint16_t start, count; uint8_t effect; uint8_t r,g,b; uint16_t durationMs; };
struct __attribute__((packed)) LampCtrlMsg { PktHeader h; uint8_t idx; uint8_t on; };
struct __attribute__((packed)) OtaStartMsg { PktHeader h; char url[200]; };
struct __attribute__((packed)) OtaAckMsg { PktHeader h; uint8_t status; /*0=starting*/ };
struct __attribute__((packed)) NodeStatusMsg { PktHeader h; uint32_t uptimeMs; uint8_t  initedMask; uint8_t  errStreakMax; uint8_t  reinitCount[8]; };
struct __attribute__((packed)) LedMapMsg {
  PktHeader h;
  uint8_t n;
  struct { uint8_t pin; uint16_t count; } e[5];
};

static uint16_t gSeq=1; static uint8_t kBroadcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

struct NodeInfo { uint8_t nodeId; uint8_t mac[6]; };
static std::map<uint8_t, NodeInfo> nodesById; // nodeId -> mac

struct NodeStatus {
  uint32_t uptimeMs;
  uint8_t  initedMask;
  uint8_t  errStreakMax;
  uint8_t  reinitCount[8];
  uint32_t rxTsMs;
};
static std::map<uint8_t, NodeStatus> lastStatus;

// Full-gate routing (server fan-out)
struct GateMapEntry {
  uint8_t  nodeId;   // LED owner node
  uint8_t  strip;    // local strip index on that node
  uint16_t start;    // pixel start on that strip
  uint16_t count;    // pixel count for the gate segment
};

// Index 0 unused for convenience; gates are 1..44
static const GateMapEntry GATE_MAP[45] = {
  /*0*/  {0,0,0,0},

  /*1*/  {4,0,135,50},  // L9: G34(0), G23(45), G12(90), G1(135)
  /*2*/  {4,1,135,50},  // L10: G35(0), G24(45), G13(90), G2(135)
  /*3*/  {5,4,135,50},  // L11
  /*4*/  {6,0,135,50},  // L12
  /*5*/  {6,1,135,50},  // L13
  /*6*/  {2,0, 90,50},  // L1: G8(0), G7(45), G6(90)
  /*7*/  {2,0, 45,45},  // L1
  /*8*/  {2,0,  0,45},  // L1

  /*9*/  {2,1,  0,45},  // L2: G9(0), G10(45), G11(90)
  /*10*/ {2,1, 45,45},  // L2
  /*11*/ {2,1, 90,50},  // L2

  /*12*/ {4,0, 90,45},  // L9
  /*13*/ {4,1, 90,45},  // L10
  /*14*/ {5,4, 90,45},  // L11
  /*15*/ {6,0, 90,45},  // L12
  /*16*/ {6,1, 90,45},  // L13

  /*17*/ {2,2, 90,50},  // L3: G19(0), G18(45), G17(90)
  /*18*/ {2,2, 45,45},  // L3
  /*19*/ {2,2,  0,45},  // L3

  /*20*/ {2,3,  0,45},  // L4: G20(0), G21(45), G22(90)
  /*21*/ {2,3, 45,45},  // L4
  /*22*/ {2,3, 90,50},  // L4

  /*23*/ {4,0, 45,45},  // L9
  /*24*/ {4,1, 45,45},  // L10
  /*25*/ {5,4, 45,45},  // L11
  /*26*/ {6,0, 45,45},  // L12
  /*27*/ {6,1, 45,45},  // L13

  /*28*/ {5,0, 90,50},  // L5: G30(0), G29(45), G28(90)
  /*29*/ {5,0, 45,45},  // L5
  /*30*/ {5,0,  0,45},  // L5

  /*31*/ {5,1,  0,45},  // L6: G31(0), G32(45), G33(90)
  /*32*/ {5,1, 45,45},  // L6
  /*33*/ {5,1, 90,50},  // L6

  /*34*/ {4,0,  0,45},  // L9
  /*35*/ {4,1,  0,45},  // L10
  /*36*/ {5,4,  0,45},  // L11

  /*37*/ {6,0,  0,45},  // L12
  /*38*/ {6,1,  0,45},  // L13

  /*39*/ {5,2, 90,50},  // L7: G41(0), G40(45), G39(90)
  /*40*/ {5,2, 45,45},  // L7
  /*41*/ {5,2,  0,45},  // L7

  /*42*/ {5,3,  0,45},  // L8: G42(0), G43(45), G44(90)
  /*43*/ {5,3, 45,45},  // L8
  /*44*/ {5,3, 90,50},  // L8
};

static String macToStr(const uint8_t m[6]){
  char b[18]; sprintf(b,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); return String(b);
}

static void addOrUpdateNode(uint8_t nodeId, const uint8_t mac[6]){
  NodeInfo n{nodeId,{0}}; memcpy(n.mac,mac,6); nodesById[nodeId]=n;
  // ensure peer exists
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,mac,6); p.channel=6; p.encrypt=false; p.ifidx = WIFI_IF_STA; esp_now_add_peer(&p);
}

static void sendRaw(const uint8_t mac[6], const uint8_t* data, size_t len){ esp_now_send(mac, data, len); }
static void bcastHelloReq(){ HelloMsg m{}; m.h={HELLO_REQ,PROTO_VER,0,0,gSeq++,sizeof(HelloMsg)}; m.role=0; m.caps=0; sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m)); }
static void sendClaim(const uint8_t mac[6], uint8_t nodeId){ ClaimMsg m{}; m.h={CLAIM,PROTO_VER,0,0,gSeq++,sizeof(ClaimMsg)}; m.newNodeId=nodeId; sendRaw(mac,(uint8_t*)&m,sizeof(m)); }

static void sendLedRange(uint8_t nodeId, uint8_t strip, uint16_t start, uint16_t count,
                         uint8_t r,uint8_t g,uint8_t b){
  auto it = nodesById.find(nodeId); if (it == nodesById.end()) return;

  LedRangeMsg m{}; 
  m.h = {LED_RANGE, PROTO_VER, 0, 0, gSeq++, (uint16_t)sizeof(LedRangeMsg)};
  m.strip = strip; m.start = start; m.count = count; m.effect = 0; m.r = r; m.g = g; m.b = b; m.durationMs = 0;

  _enqueueTx(it->second.mac, &m, sizeof(m));
}

static void sendLampCtrl(uint8_t nodeId, uint8_t idx, bool on){
  auto it=nodesById.find(nodeId); if (it==nodesById.end()) return;
  LampCtrlMsg m{}; m.h={LAMP_CTRL,PROTO_VER,0,0,gSeq++,sizeof(LampCtrlMsg)}; m.idx=idx; m.on=on?1:0;
  sendRaw(it->second.mac,(uint8_t*)&m,sizeof(m));
}

static void sendOtaStart(uint8_t nodeId, const String& url){
  auto it=nodesById.find(nodeId); if (it==nodesById.end()) return;
  OtaStartMsg m{}; m.h={OTA_START,PROTO_VER,0,0,gSeq++,sizeof(OtaStartMsg)};
  memset(m.url, 0, sizeof(m.url));
  if (url.length()>0) strncpy(m.url, url.c_str(), sizeof(m.url)-1);
  sendRaw(it->second.mac,(uint8_t*)&m,sizeof(m));
}

// --- Node 4 LED routing helper (only L9/L10 for bring‑up) ---
static bool routeGateToNode4(uint8_t gateId, /*out*/uint8_t &strip, uint16_t &start) {
  // L9 (strip 0): G34(45), G23(45), G12(45), G1(50)
  // L10 (strip 1): G35(45), G24(45), G13(45), G2(50)
  switch (gateId) {
    case 34: strip = 0; start =   0;  return true;
    case 23: strip = 0; start =  45;  return true;
    case 12: strip = 0; start =  90;  return true;
    case 1:  strip = 0; start = 135;  return true; // 50 px
    case 35: strip = 1; start =   0;  return true;
    case 24: strip = 1; start =  45;  return true;
    case 13: strip = 1; start =  90;  return true;
    case 2:  strip = 1; start = 135;  return true; // 50 px
    default: return false;
  }
}

static bool routeGate(uint8_t gateId,
                      /*out*/uint8_t &nodeId,
                      /*out*/uint8_t &strip,
                      /*out*/uint16_t &start,
                      /*out*/uint16_t &count) {
  if (gateId < 1 || gateId > 44) return false;
  const GateMapEntry &e = GATE_MAP[gateId];
  if (e.nodeId == 0) return false;
  nodeId = e.nodeId; strip = e.strip; start = e.start; count = e.count;
  return true;
}

// Fixed sizes for Node4 (matches your Node code)
static inline uint16_t gateCountNode4(uint8_t g) {
  return (g==1 || g==2) ? 50 : 45; // G1,G2 are 50; others 45
}

static void clearNode4Strips() {
  // L9 (strip 0) and L10 (strip 1) are 185px each
  sendLedRange(/*nodeId*/4, /*strip*/0, /*start*/0, /*count*/185, /*r*/0, /*g*/0, /*b*/0);
  sendLedRange(/*nodeId*/4, /*strip*/1, /*start*/0, /*count*/185, /*r*/0, /*g*/0, /*b*/0);
}

// --- Walkable set (server-side rule state) ---
#define GATE_MAX 44
static bool walkable[GATE_MAX+1]; // 1..44 used
static inline void clearWalkable(){ memset(walkable, 0, sizeof(walkable)); }
static inline void addWalkableGate(uint8_t g){ if (g>=1 && g<=GATE_MAX) walkable[g] = true; }
static inline bool isWalkable(uint8_t g){ return (g>=1 && g<=GATE_MAX) ? walkable[g] : false; }
// Full-field pushWalkable(): base-coat per (node,strip) then overlay greens
static void pushWalkable(){
  // 1) Scan the GATE_MAP to figure out max length per (node,strip)
  // Limits: nodeId in [0..7], strip in [0..7] (small fixed arrays keep it simple)
  uint16_t maxLen[8][8];    // max pixel (end) per (node,strip)
  bool     seenStrip[8][8]; // did we see any gate on (node,strip)?
  bool     seenNode[8];     // node has at least one strip

  // init
  for (int n=0; n<8; ++n){ seenNode[n]=false; for (int s=0; s<8; ++s){ maxLen[n][s]=0; seenStrip[n][s]=false; }}

  // pass 1: compute maxLen and seen sets
  for (uint8_t g=1; g<=44; ++g){
    const GateMapEntry &e = GATE_MAP[g];
    if (e.nodeId == 0) continue;                  // unmapped
    if (e.strip   > 7) continue;
    uint16_t end = e.start + e.count;
    if (end > maxLen[e.nodeId][e.strip]) maxLen[e.nodeId][e.strip] = end;
    seenStrip[e.nodeId][e.strip] = true;
    seenNode[e.nodeId] = true;
  }

  // 2) Base coat: for every seen (node,strip), paint full red
  for (uint8_t n=0; n<8; ++n){
    if (!seenNode[n]) continue;
    for (uint8_t s=0; s<8; ++s){
      if (!seenStrip[n][s]) continue;
      uint16_t count = maxLen[n][s];
      if (count == 0) continue;
      // red baseline
      sendLedRange(n, s, 0, count, 150, 0, 0);
    }
  }

  // 3) Overlays: for each gate, overlay green if walkable (else red is already there)
  for (uint8_t g=1; g<=44; ++g){
    const GateMapEntry &e = GATE_MAP[g];
    if (e.nodeId == 0) continue;
    const bool ok = walkable[g];
    const uint8_t r = ok ? 0 : 150;
    const uint8_t gr = ok ? 150 : 0;
    sendLedRange(e.nodeId, e.strip, e.start, e.count, r, gr, 0);
  }
}

// --- Process a (real or fake) GATE_EVENT ---
static void processGateEvent(uint8_t gateId, uint8_t ev){
  uint8_t nodeId, strip; uint16_t start, count;
  if (!routeGate(gateId, nodeId, strip, start, count)) return;
  const bool ok = isWalkable(gateId);
  const uint8_t r = ok ? 0 : 150;
  const uint8_t g = ok ? 150 : 0;
  sendLedRange(nodeId, strip, start, count, r, g, 0);
}

static void onNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len){
  if (!info || len < (int)sizeof(PktHeader)) return; auto *h=(const PktHeader*)data; if (h->version!=PROTO_VER) return;
  const uint8_t* mac = info->src_addr;
  if (h->type==HELLO){ uint8_t nid=h->nodeId; addOrUpdateNode(nid, mac); Serial.printf("HELLO from node %u (%s)", nid, macToStr(mac).c_str()); Serial.println(); }
  else if (h->type==GATE_EVENT && len>=(int)sizeof(GateEventMsg)){
    auto *m=(const GateEventMsg*)data; Serial.printf("GATE %u %s mm=%u from node %u", m->gateId, (m->ev==1?"ENTER":(m->ev==2?"EXIT":"?")), m->strengthMm, h->nodeId); Serial.println();
    processGateEvent(m->gateId, m->ev);
  }
  else if (h->type==OTA_ACK && len>=(int)sizeof(OtaAckMsg)){
    auto *m=(const OtaAckMsg*)data; Serial.printf("OTA_ACK from node %u (status=%u)", h->nodeId, m->status); Serial.println();
  }
  else if (h->type==BUTTON_EVENT && len>=(int)sizeof(ButtonEventMsg)){
    auto *m=(const ButtonEventMsg*)data; Serial.printf("BUTTON%u %s from node %u", m->btnIdx, (m->ev==1?"PRESS":"RELEASE"), h->nodeId); Serial.println();
  }
  else if (h->type == NODE_STATUS && len >= (int)sizeof(NodeStatusMsg)) {
    const NodeStatusMsg* m = (const NodeStatusMsg*)data;
    logNodeStatus(h->nodeId, m);
  }
}

static void logNodeStatus(uint8_t nodeId, const NodeStatusMsg* m) {
  const uint8_t mask = m->initedMask;

  Serial.printf("STATUS node %u up=%lus ", nodeId, (unsigned)(m->uptimeMs/1000));

  // up_ch=[...]
  Serial.print("up_ch=[");
  bool first = true;
  for (int ch = 0; ch < 8; ++ch) {
    if (mask & (1u << ch)) {
      if (!first) Serial.print(",");
      Serial.print(ch);
      first = false;
    }
  }
  Serial.print("] ");

  // down_ch=[...]
  Serial.print("down_ch=[");
  first = true;
  int downCount = 0;
  for (int ch = 0; ch < 8; ++ch) {
    if (!(mask & (1u << ch))) {
      if (!first) Serial.print(",");
      Serial.print(ch);
      first = false;
      downCount++;
    }
  }
  Serial.print("] ");

  Serial.printf("maxErr=%u", m->errStreakMax);

  // If this is node 4, also show gates_down=[...]
  if (nodeId == 4 && downCount > 0) {
    Serial.print(" gates_down=[");
    first = true;
    for (int ch = 0; ch < 8; ++ch) {
      if (!(mask & (1u << ch))) {
        uint8_t gate = GATE_BY_CH_NODE4[ch];
        if (!first) Serial.print(",");
        Serial.print(gate);
        first = false;
      }
    }
    Serial.print("]");
  }

  Serial.println();
}

// ---- Drip TX queue for ESP-NOW ----
struct TxItem {
  uint8_t mac[6];
  uint8_t len;
  uint8_t buf[64];    // plenty for our messages
};
static TxItem txQ[64];
static volatile uint8_t qHead = 0, qTail = 0;
static uint32_t lastTxMs = 0;
static const uint16_t DRIP_MS = 5;    // 3ms spacing (tune 2–5ms)

// Enqueue a raw frame (drop oldest if full)
static void _enqueueTx(const uint8_t mac[6], const void* data, uint8_t len){
  uint8_t next = (uint8_t)((qTail + 1) & 63);
  if (next == qHead) { qHead = (uint8_t)((qHead + 1) & 63); } // drop oldest
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

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE); esp_now_init(); esp_now_register_recv_cb(onNowRecv);
  // Add broadcast peer for HELLO_REQ
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6); p.channel=6; p.encrypt=false; p.ifidx = WIFI_IF_STA; esp_now_add_peer(&p);
  delay(200);
  bcastHelloReq();
  Serial.println("Commands:");
  Serial.println("  claim <nodeId> <mac>");
  Serial.println("  setgate <gateId> <r> <g> <b>");
  Serial.println("  fakegate <gateId> <enter|exit>");
  Serial.println("  walkable clear | add <ids> | show");
  Serial.println("  pushwalkable");
  Serial.println("  lamp <nodeId> <idx> <on|off>");
  Serial.println("  ota <nodeId> [url]");
  Serial.println("  hello");
  Serial.println("  roster");
}

static void handleCli(String s){
  s.trim();
  if (s=="hello"){ bcastHelloReq(); return; }
  if (s=="roster"){ for (auto &kv : nodesById){ auto &n=kv.second; Serial.printf("node %u @ %02X:%02X:%02X:%02X:%02X:%02X", n.nodeId,n.mac[0],n.mac[1],n.mac[2],n.mac[3],n.mac[4],n.mac[5]); Serial.println(); } return; }

  if (s.startsWith("claim ")){
    // claim 4 AA:BB:CC:DD:EE:FF
    int nid; char macs[32]; if (sscanf(s.c_str(),"claim %d %31s", &nid, macs)==2){
      uint8_t mac[6]; if (sscanf(macs,"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5])==6){
        sendClaim(mac,(uint8_t)nid); addOrUpdateNode((uint8_t)nid, mac); Serial.println("CLAIM sent");
      }
    }
    return;
  }

  if (s.startsWith("setgate ")) {
    int gid,r,g,b;
    if (sscanf(s.c_str(),"setgate %d %d %d %d", &gid,&r,&g,&b)==4) {
      uint8_t nodeId, strip; uint16_t start, count;
      if (routeGate((uint8_t)gid, nodeId, strip, start, count)) {
        sendLedRange(nodeId, strip, start, count, (uint8_t)r,(uint8_t)g,(uint8_t)b);
        Serial.printf("LED_RANGE node=%u strip=%u start=%u count=%u rgb(%d,%d,%d)\n",
                      nodeId, strip, start, count, r,g,b);
      } else {
        Serial.println("Unknown gateId or not mapped.");
      }
    }
    return;
  }

  if (s.startsWith("fakegate ")){
    int gid; char kind[8]={0};
    if (sscanf(s.c_str(),"fakegate %d %7s", &gid, kind)==2){
      bool isEnter = (String(kind)=="enter" || String(kind)=="ENTER");
      processGateEvent((uint8_t)gid, isEnter?1:2);
      Serial.printf("FAKE %s gate %d", isEnter?"ENTER":"EXIT", gid); Serial.println();
    } else {
      Serial.println("usage: fakegate <gateId> <enter|exit>");
    }
    return;
  }

  if (s=="walkable clear"){ clearWalkable(); Serial.println("walkable cleared"); return; }

  if (s=="walkable show"){ Serial.print("walkable: "); for (uint8_t g=1; g<=GATE_MAX; ++g){ if (walkable[g]){ Serial.print(g); Serial.print(' ');} } Serial.println(); return; }

  if (s.startsWith("walkable add ")){
    char buf[256]; s.toCharArray(buf, sizeof(buf));
    char *p = strstr(buf, "add ");
    if (p){ p += 4; char *tok = strtok(p, " ,"); while (tok){ int gid = atoi(tok); if (gid>=1 && gid<=GATE_MAX) walkable[gid]=true; tok = strtok(NULL, " ,"); } }
    Serial.println("walkable updated");
    return;
  }

  if (s=="pushwalkable"){ pushWalkable(); return; }

  if (s.startsWith("lamp ")){
    int nid, idx; char onoff[8]={0};
    if (sscanf(s.c_str(),"lamp %d %d %7s", &nid, &idx, onoff)==3){
      bool on = (String(onoff)=="on");
      sendLampCtrl((uint8_t)nid, (uint8_t)idx, on);
      Serial.printf("LAMP node=%d idx=%d %s", nid, idx, on?"ON":"OFF"); Serial.println();
    }
    return;
  }

  if (s=="status") {
    // ask nodes to send HELLO + NODE_STATUS
    bcastHelloReq();

    // print whatever we have cached now
    Serial.println("node  uptime(s)  inited  maxErr  reinitCounts");
    for (auto &kv : lastStatus) {
      auto nid = kv.first;
      const auto &st = kv.second;
      Serial.printf("%3u  %9lu   0x%02X     %3u   [",
                    nid, (unsigned)(st.uptimeMs/1000), st.initedMask, st.errStreakMax);
      for (int i=0;i<8;i++) { Serial.printf("%u%s", st.reinitCount[i], i==7?"]\n":","); }
    }
    return;
  }

  // ledmap <nodeId> <pin:count>[,<pin:count>...]
  if (s.startsWith("ledmap ")) {
    int nid; char list[256]={0};
    if (sscanf(s.c_str(),"ledmap %d %255s", &nid, list)==2){
      auto it = nodesById.find((uint8_t)nid);
      if (it == nodesById.end()) { Serial.println("Unknown nodeId"); return; }

      LedMapMsg m{}; m.h={ LED_MAP, PROTO_VER, 0,0, gSeq++, (uint16_t)sizeof(LedMapMsg) };
      m.n = 0;
      char *tok = strtok(list, ",");
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

      _enqueueTx(it->second.mac, &m, sizeof(m));  // or sendRaw
      Serial.printf("LED_MAP sent to node=%d with %u strips\n", nid, m.n);
    } else {
      Serial.println("usage: ledmap <nodeId> <pin:count>[,<pin:count>...]");
    }
    return;
  }

  if (s.startsWith("ota ")){
    int nid; char url[256]={0}; int n=sscanf(s.c_str(),"ota %d %255s", &nid, url);
    sendOtaStart((uint8_t)nid, (n==2)?String(url):String());
    Serial.printf("OTA_START node=%d %s", nid, (n==2)?url:"<default>"); Serial.println();
    return;
  }
}

void loop(){
  dripPump();
  if (Serial.available()){ String s=Serial.readStringUntil('\n'); handleCli(s); }
}