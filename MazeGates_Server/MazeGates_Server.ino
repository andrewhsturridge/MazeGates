/*
 * Maze Gates – Server Harness v0
 * Board: ESP32 (any), Transport: ESP‑NOW ch.6
 * Uses a minimal copy of the message types from Node4 sketch.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <map>
#include <vector>

// ======= Protocol (same as node) =======
enum MsgType : uint8_t { HELLO=1, HELLO_REQ=2, CLAIM=3, GATE_EVENT=10, LED_RANGE=20, BUTTON_EVENT=30 };
struct __attribute__((packed)) PktHeader { uint8_t type, version, nodeId, pad; uint16_t seq, len; };
static const uint8_t PROTO_VER = 1;
struct __attribute__((packed)) HelloMsg { PktHeader h; uint8_t role; uint8_t caps; };
struct __attribute__((packed)) ClaimMsg { PktHeader h; uint8_t newNodeId; };
struct __attribute__((packed)) GateEventMsg { PktHeader h; uint8_t gateId; uint8_t ev; uint16_t strengthMm; uint32_t tsMs; };
struct __attribute__((packed)) ButtonEventMsg { PktHeader h; uint8_t btnIdx; uint8_t ev; uint32_t tsMs; };
struct __attribute__((packed)) LedRangeMsg { PktHeader h; uint8_t strip; uint16_t start, count; uint8_t effect; uint8_t r,g,b; uint16_t durationMs; };

static uint16_t gSeq=1; static uint8_t kBroadcast[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

struct NodeInfo { uint8_t nodeId; uint8_t mac[6]; };
static std::map<uint8_t, NodeInfo> nodesById; // nodeId -> mac
static std::map<String, uint8_t> macStrToNodeId; // for CLI

static String macToStr(const uint8_t m[6]){
  char b[18]; sprintf(b,"%02X:%02X:%02X:%02X:%02X:%02X",m[0],m[1],m[2],m[3],m[4],m[5]); return String(b);
}

static void addOrUpdateNode(uint8_t nodeId, const uint8_t mac[6]){
  NodeInfo n{nodeId,{0}}; memcpy(n.mac,mac,6); nodesById[nodeId]=n; macStrToNodeId[macToStr(mac)] = nodeId;
  // ensure peer exists
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,mac,6); p.channel=6; p.encrypt=false; esp_now_add_peer(&p);
}

static void sendRaw(const uint8_t mac[6], const uint8_t* data, size_t len){ esp_now_send(mac, data, len); }

static void bcastHelloReq(){ HelloMsg m{}; m.h={HELLO_REQ,PROTO_VER,0,0,gSeq++,sizeof(HelloMsg)}; m.role=0; m.caps=0; sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m)); }

static void sendClaim(const uint8_t mac[6], uint8_t nodeId){ ClaimMsg m{}; m.h={CLAIM,PROTO_VER,0,0,gSeq++,sizeof(ClaimMsg)}; m.newNodeId=nodeId; sendRaw(mac,(uint8_t*)&m,sizeof(m)); }

static void sendLedRange(uint8_t nodeId, uint8_t strip, uint16_t start, uint16_t count, uint8_t r,uint8_t g,uint8_t b){
  auto it=nodesById.find(nodeId); if (it==nodesById.end()) return; LedRangeMsg m{}; m.h={LED_RANGE,PROTO_VER,0,0,gSeq++,sizeof(LedRangeMsg)};
  m.strip=strip; m.start=start; m.count=count; m.effect=0; m.r=r; m.g=g; m.b=b; m.durationMs=0; sendRaw(it->second.mac,(uint8_t*)&m,sizeof(m));
}

static void onNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len){
  if (!info || len < (int)sizeof(PktHeader)) return;
  const uint8_t* mac = info->src_addr;
  auto *h = (const PktHeader*)data;
  if (h->version != PROTO_VER) return;

  if (h->type == HELLO) {
    uint8_t nid = h->nodeId;
    addOrUpdateNode(nid, mac);
    Serial.printf("HELLO from node %u (%02X:%02X:%02X:%02X:%02X:%02X)\n",
                  nid, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  } else if (h->type == GATE_EVENT && len >= (int)sizeof(GateEventMsg)) {
    auto *m = (const GateEventMsg*)data;
    Serial.printf("GATE %u %s mm=%u from node %u\n",
      m->gateId, (m->ev==1?"ENTER":(m->ev==2?"EXIT":"?")),
      m->strengthMm, h->nodeId);
  } else if (h->type == BUTTON_EVENT && len >= (int)sizeof(ButtonEventMsg)) {
    auto *m = (const ButtonEventMsg*)data;
    Serial.printf("BUTTON%u %s from node %u\n",
      m->btnIdx, (m->ev==1?"PRESS":"RELEASE"), h->nodeId);
  }
}

// --- Node 4 LED routing helper (only L9/L10 for bring‑up) ---
static bool routeGateToNode4(uint8_t gateId, /*out*/uint8_t &strip, uint16_t &start){
  const uint16_t N=60; switch (gateId){
    case 34: strip=0; start=0*N; return true;  // L9 seg0
    case 23: strip=0; start=1*N; return true;  // L9 seg1
    case 12: strip=0; start=2*N; return true;  // L9 seg2
    case 1:  strip=0; start=3*N; return true;  // L9 seg3
    case 35: strip=1; start=0*N; return true;  // L10 seg0
    case 24: strip=1; start=1*N; return true;  // L10 seg1
    case 13: strip=1; start=2*N; return true;  // L10 seg2
    case 2:  strip=1; start=3*N; return true;  // L10 seg3
    default: return false;
  }
}

void setup(){
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  ESP_ERROR_CHECK( esp_now_init() );
  esp_now_register_recv_cb(onNowRecv);
  
  // Add broadcast peer for HELLO_REQ
  esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6); p.channel=6; p.encrypt=false; esp_now_add_peer(&p);
  delay(200);
  bcastHelloReq();
  Serial.println("Commands:\n  claim <nodeId> <mac>\n  setgate <gateId> <r> <g> <b>\n  hello");
}

static void handleCli(String s){
  s.trim(); if (s=="hello"){ bcastHelloReq(); return; }
  if (s.startsWith("claim ")){
    // claim 4 AA:BB:CC:DD:EE:FF
    int nid; char macs[32]; if (sscanf(s.c_str(),"claim %d %31s", &nid, macs)==2){
      uint8_t mac[6]; if (sscanf(macs,"%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5])==6){
        sendClaim(mac,(uint8_t)nid); addOrUpdateNode((uint8_t)nid, mac); Serial.println("CLAIM sent");
      }
    }
    return;
  }
  if (s.startsWith("setgate ")){
    int gid,r,g,b; if (sscanf(s.c_str(),"setgate %d %d %d %d", &gid,&r,&g,&b)==4){
      uint8_t strip; uint16_t start; if (routeGateToNode4((uint8_t)gid, strip, start)){
        sendLedRange(/*nodeId*/4, strip, start, 60, (uint8_t)r,(uint8_t)g,(uint8_t)b);
        Serial.printf("LED_RANGE node4 strip=%u start=%u count=60 rgb(%d,%d,%d)\n", strip,start,r,g,b);
      } else {
        Serial.println("Gate not on Node4's strips in this harness.");
      }
    }
    return;
  }
}

void loop(){ if (Serial.available()){ String s=Serial.readStringUntil('\n'); handleCli(s); } }
