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

// ======= Protocol (same as node) =======
enum MsgType : uint8_t { HELLO=1, HELLO_REQ=2, CLAIM=3, GATE_EVENT=10, LED_RANGE=20, LAMP_CTRL=21, BUTTON_EVENT=30, OTA_START=50, OTA_ACK=51 };
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

static uint16_t gSeq=1; static uint8_t kBroadcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

struct NodeInfo { uint8_t nodeId; uint8_t mac[6]; };
static std::map<uint8_t, NodeInfo> nodesById; // nodeId -> mac

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

static void sendLedRange(uint8_t nodeId, uint8_t strip, uint16_t start, uint16_t count, uint8_t r,uint8_t g,uint8_t b){
  auto it=nodesById.find(nodeId); if (it==nodesById.end()) return; LedRangeMsg m{}; m.h={LED_RANGE,PROTO_VER,0,0,gSeq++,sizeof(LedRangeMsg)};
  m.strip=strip; m.start=start; m.count=count; m.effect=0; m.r=r; m.g=g; m.b=b; m.durationMs=0; sendRaw(it->second.mac,(uint8_t*)&m,sizeof(m));
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
static void pushWalkable(){
  // Node4-only in this harness: paint in known order to avoid surprises
  clearNode4Strips();

  const uint8_t gatesL9[] = {34,23,12,1};
  const uint8_t gatesL10[] = {35,24,13,2};

  for (uint8_t i=0;i<4;i++){
    uint8_t g = gatesL9[i];
    uint8_t strip; uint16_t start; if (!routeGateToNode4(g, strip, start)) continue;
    uint16_t count = gateCountNode4(g);
    bool ok = walkable[g];
    sendLedRange(/*nodeId*/4, strip, start, count, ok?0:150, ok?150:0, 0);
  }

  for (uint8_t i=0;i<4;i++){
    uint8_t g = gatesL10[i];
    uint8_t strip; uint16_t start; if (!routeGateToNode4(g, strip, start)) continue;
    uint16_t count = gateCountNode4(g);
    bool ok = walkable[g];
    sendLedRange(/*nodeId*/4, strip, start, count, ok?0:150, ok?150:0, 0);
  }
}

// --- Process a (real or fake) GATE_EVENT ---
static void processGateEvent(uint8_t gateId, uint8_t ev){
  uint8_t strip; uint16_t start; if (!routeGateToNode4(gateId, strip, start)) return;
  const uint16_t count = (gateId==1 || gateId==2) ? 50 : 45;
  bool ok = false;
  if (ev == 1) { // ENTER
    ok = isWalkable(gateId);
  } else if (ev == 2) { // EXIT: repaint baseline
    ok = isWalkable(gateId);
  }
  uint8_t r = ok ? 0 : 150; uint8_t gr = ok ? 150 : 0;
  sendLedRange(/*nodeId*/4, strip, start, count, r, gr, 0);
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
    int gid, r, g, b;
    if (sscanf(s.c_str(), "setgate %d %d %d %d", &gid, &r, &g, &b) == 4) {
      uint8_t strip; uint16_t start;
      if (routeGateToNode4((uint8_t)gid, strip, start)) {
        int gateCount = (gid == 1 || gid == 2) ? 50 : 45;  // Node4: L9/L10 last segments are 50
        sendLedRange(/*nodeId*/4, strip, start, (uint16_t)gateCount,
                     (uint8_t)r, (uint8_t)g, (uint8_t)b);
        Serial.printf("LED_RANGE node4 strip=%u start=%u count=%d rgb(%d,%d,%d)", strip, start, gateCount, r, g, b); Serial.println();
      } else {
        Serial.println("Gate not on Node4's strips in this harness.");
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

  if (s.startsWith("ota ")){
    int nid; char url[256]={0}; int n=sscanf(s.c_str(),"ota %d %255s", &nid, url);
    sendOtaStart((uint8_t)nid, (n==2)?String(url):String());
    Serial.printf("OTA_START node=%d %s", nid, (n==2)?url:"<default>"); Serial.println();
    return;
  }
}

void loop(){ if (Serial.available()){ String s=Serial.readStringUntil('\n'); handleCli(s); } }