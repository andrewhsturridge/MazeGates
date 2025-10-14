/*
 * Maze Gates – Server Harness v0 (clean compile)
 * Board: ESP32 (any), Transport: ESP-NOW ch.6
 * Full-field fan-out via GATE_MAP; CLI includes help, hello, roster, claim, setgate,
 * fakegate, walkable, pushwalkable, path set, lamp, ledmap (set/show), ota, ota all, status.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <map>
#include <vector>
#include <cstring>

// ======= Protocol (same as node) =======
enum MsgType : uint8_t {
  HELLO=1, HELLO_REQ=2, CLAIM=3,
  GATE_EVENT=10, LED_RANGE=20, LAMP_CTRL=21,
  BUTTON_EVENT=30,
  OTA_START=50, OTA_ACK=51,
  NODE_STATUS=60,
  LED_MAP=62,
  LED_MAP_REQ = 63,
  LED_MAP_RSP = 64
};

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

// response (matches node)
struct __attribute__((packed)) LedMapRsp {
  PktHeader h;
  uint8_t n;
  struct { uint8_t pin; uint16_t count; } e[5];
};

static uint16_t gSeq=1;
static uint8_t kBroadcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

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

// ======= Full-gate routing (server fan-out) =======
struct GateMapEntry { uint8_t nodeId; uint8_t strip; uint16_t start; uint16_t count; };

// Index 0 unused for convenience; gates are 1..44
static const GateMapEntry GATE_MAP[45] = {
  /*0*/  {0,0,0,0},

  /*1*/  {4,0,135,50},  /*2*/  {4,1,135,50},  /*3*/  {5,4,135,50},
  /*4*/  {6,0,135,50},  /*5*/  {6,1,135,50},

  /*6*/  {2,0, 90,50},  /*7*/  {2,0, 45,45},  /*8*/  {2,0,  0,45},
  /*9*/  {2,1,  0,45},  /*10*/ {2,1, 45,45},  /*11*/ {2,1, 90,50},

  /*12*/ {4,0, 90,45},  /*13*/ {4,1, 90,45},  /*14*/ {5,4, 90,45},
  /*15*/ {6,0, 90,45},  /*16*/ {6,1, 90,45},

  /*17*/ {2,2, 90,50},  /*18*/ {2,2, 45,45},  /*19*/ {2,2,  0,45},

  /*20*/ {2,3,  0,45},  /*21*/ {2,3, 45,45},  /*22*/ {2,3, 90,50},

  /*23*/ {4,0, 45,45},  /*24*/ {4,1, 45,45},  /*25*/ {5,4, 45,45},
  /*26*/ {6,0, 45,45},  /*27*/ {6,1, 45,45},

  /*28*/ {5,0, 90,50},  /*29*/ {5,0, 45,45},  /*30*/ {5,0,  0,45},

  /*31*/ {5,1,  0,45},  /*32*/ {5,1, 45,45},  /*33*/ {5,1, 90,50},

  /*34*/ {4,0,  0,45},  /*35*/ {4,1,  0,45},  /*36*/ {5,4,  0,45},

  /*37*/ {6,0,  0,45},  /*38*/ {6,1,  0,45},

  /*39*/ {5,2, 90,50},  /*40*/ {5,2, 45,45},  /*41*/ {5,2,  0,45},

  /*42*/ {5,3,  0,45},  /*43*/ {5,3, 45,45},  /*44*/ {5,3, 90,50},
};

// ======= Utilities =======
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

// ======= Drip TX queue for ESP-NOW =======
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

// ======= Send helpers (queued where appropriate) =======
static void sendLedRange(uint8_t nodeId, uint8_t strip, uint16_t start, uint16_t count,
                         uint8_t r,uint8_t g,uint8_t b){
  auto it = nodesById.find(nodeId); if (it == nodesById.end()) return;
  LedRangeMsg m{}; m.h = {LED_RANGE, PROTO_VER, 0, 0, gSeq++, (uint16_t)sizeof(LedRangeMsg)};
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

// ======= Routing helpers =======
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

// ======= Walkable / game fan-out =======
#define GATE_MAX 44
static bool walkable[GATE_MAX+1]; // 1..44 used
static inline void clearWalkable(){ memset(walkable, 0, sizeof(walkable)); }
static inline bool isWalkable(uint8_t g){ return (g>=1 && g<=GATE_MAX) ? walkable[g] : false; }

static void pushWalkable(){
  // 1) Compute base coat extents per (node,strip)
  uint16_t maxLen[8][8] = {}; bool seenStrip[8][8] = {}; bool seenNode[8] = {};
  for (uint8_t g=1; g<=44; ++g){
    const GateMapEntry &e = GATE_MAP[g];
    if (!e.nodeId) continue;
    uint16_t end = e.start + e.count;
    if (end > maxLen[e.nodeId][e.strip]) maxLen[e.nodeId][e.strip] = end;
    seenStrip[e.nodeId][e.strip] = true; seenNode[e.nodeId] = true;
  }
  // 2) Base coat red
  for (uint8_t n=0;n<8;++n){
    if (!seenNode[n]) continue;
    for (uint8_t s=0;s<8;++s){
      if (!seenStrip[n][s]) continue;
      uint16_t cnt = maxLen[n][s]; if (!cnt) continue;
      sendLedRange(n, s, 0, cnt, 150,0,0);
    }
  }
  // 3) Overlay greens where walkable
  for (uint8_t g=1; g<=44; ++g){
    const GateMapEntry &e = GATE_MAP[g]; if (!e.nodeId) continue;
    const bool ok = walkable[g];
    sendLedRange(e.nodeId, e.strip, e.start, e.count, ok?0:150, ok?150:0, 0);
  }
}

static void processGateEvent(uint8_t gateId, uint8_t ev){
  uint8_t nodeId, strip; uint16_t start, count;
  if (!routeGate(gateId, nodeId, strip, start, count)) return;
  const bool ok = isWalkable(gateId);
  sendLedRange(nodeId, strip, start, count, ok?0:150, ok?150:0, 0);
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
  // small message; send directly is fine (or _enqueueTx if you prefer)
  sendRaw(it->second.mac, (uint8_t*)&h, sizeof(h));
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
  else if (h->type==BUTTON_EVENT && len>=(int)sizeof(ButtonEventMsg)){
    auto *m=(const ButtonEventMsg*)data;
    Serial.printf("BUTTON%u %s from node %u\n",
                  m->btnIdx, (m->ev==1?"PRESS":"RELEASE"), h->nodeId);
  }
  else if (h->type == NODE_STATUS && len >= (int)sizeof(NodeStatusMsg)) {
    const NodeStatusMsg* m = (const NodeStatusMsg*)data;
    lastStatus[h->nodeId] = { m->uptimeMs, m->initedMask, m->errStreakMax, {0}, millis() };
    memcpy(lastStatus[h->nodeId].reinitCount, m->reinitCount, 8);
    logNodeStatus(h->nodeId, m);
  }
  else if (h->type == LED_MAP_RSP && len >= (int)sizeof(LedMapRsp)) {
    const LedMapRsp* r = (const LedMapRsp*)data;
    printNodeLedMap(h->nodeId, r);
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
  Serial.println("  lamp <nodeId> <idx> <on|off>");
  Serial.println("  ledmap show [nodeId]");
  Serial.println("  ledmap get <nodeId> | ledmap get all");
  Serial.println("  ledmap <nodeId> <pin:count>[,<pin:count>...]");
  Serial.println("  ota <nodeId> [url]");
  Serial.println("  ota all [url]");
  Serial.println("  status");
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

  // broadcast peer for HELLO_REQ
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

  // path set <ids>  -> clear -> add -> push
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

  // ledmap show [nodeId]    -> print server's GATE_MAP (routing view)
  // ledmap get <nodeId>|all -> query node(s) for stored pin:count (runtime view)
  // ledmap <nodeId> <pin:count>[,<pin:count>...] -> set and reboot node
  if (s.startsWith("ledmap")){
    // normalize whitespace
    s.replace("\r",""); s.replace("\n",""); s.trim();

    // SHOW
    if (s == "ledmap show"){ printLedMap(-1); return; }
    int filtId = -1;
    if (sscanf(s.c_str(),"ledmap show %d", &filtId)==1){ printLedMap(filtId); return; }

    // GET
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

    // SET
    int nid = -1; char list[256]={0};
    if (sscanf(s.c_str(),"ledmap %d %255s", &nid, list)==2 && nid>=0){
      auto it = nodesById.find((uint8_t)nid);
      if (it == nodesById.end()) { Serial.println("Unknown nodeId"); return; }

      LedMapMsg m{}; m.h={ LED_MAP, PROTO_VER, 0,0, gSeq++, (uint16_t)sizeof(LedMapMsg) };
      m.n = 0;

      // parse <pin:count>[,<pin:count>...]
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

      _enqueueTx(it->second.mac, &m, sizeof(m));   // use drip queue
      Serial.printf("LED_MAP sent to node=%d with %u strips\n", nid, m.n);
      return;
    }

    // Fallback usage
    Serial.println("usage: ledmap show [nodeId] | ledmap get <nodeId>|all | ledmap <nodeId> <pin:count>[,<pin:count>...]");
    return;
  }

  // ota <nodeId> [url]
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
    bcastHelloReq(); // on-demand status (nodes may piggyback)
    Serial.println("node  uptime(s)  inited  maxErr  reinitCounts");
    for (auto &kv : lastStatus) {
      auto nid = kv.first; const auto &st = kv.second;
      Serial.printf("%3u  %9lu   0x%02X     %3u   [",
                    nid, (unsigned)(st.uptimeMs/1000), st.initedMask, st.errStreakMax);
      for (int i=0;i<8;i++) { Serial.printf("%u%s", st.reinitCount[i], i==7?"]\n":","); }
    }
    return;
  }

  if (s=="") return;
  Serial.println("Unknown command. Type: help");
}

void loop(){
  dripPump();
  if (Serial.available()){
    String s=Serial.readStringUntil('\n');
    handleCli(s);
  }
}
