/*
 * Maze Gates – Node 4 Foundation v0
 * Board: Unexpected Maker FeatherS3 (ESP32‑S3)
 * Transport: ESP‑NOW ch.6 (no Maintenance/Telnet), OTA .bin support later
 * Sensors: 8× VL53L4CX via TCA9548A @ 0x70
 * LEDs: L9@GPIO10 (G34,G23,G12,G1), L10@GPIO11 (G35,G24,G13,G2)
 * Buttons: B1=GPIO17, B2=GPIO18  (INPUT_PULLUP)
 * Lamps (MOSFET, 5V): LAMP1=GPIO12, LAMP2=GPIO6 (PWM‑capable)
 * I2C: SDA=8, SCL=9, 100 kHz
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include "vl53l4cx_class.h"   // STM32duino VL53L4CX by ST

// ===== Temporary: disable ToF scanning =====
#ifndef TOF_ENABLED
#define TOF_ENABLED 0
#endif

// ======= Pins (adjust later if needed) =======
#define PIN_SDA 8
#define PIN_SCL 9
#define PIN_LED_STRIP_A 10  // L9
#define PIN_LED_STRIP_B 11  // L10
#define PIN_BTN1 17
#define PIN_BTN2 18
#define PIN_LAMP1 12
#define PIN_LAMP2 6

// ======= LED setup =======
#define PIXELS_PER_GATE 60
#define STRIP_A_PIX (4 * PIXELS_PER_GATE) // G34,G23,G12,G1
#define STRIP_B_PIX (4 * PIXELS_PER_GATE) // G35,G24,G13,G2
Adafruit_NeoPixel stripA(STRIP_A_PIX, PIN_LED_STRIP_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripB(STRIP_B_PIX, PIN_LED_STRIP_B, NEO_GRB + NEO_KHZ800);

// Map gates to segments on Node4's strips
struct GateSeg { uint8_t strip; uint16_t start; uint16_t count; };
// strip: 0=A(L9), 1=B(L10), count: 60 (placeholder)
static bool getSegmentForGate(uint8_t gateId, GateSeg &seg) {
  switch (gateId) {
    // L9 (A): order from pixel0 → G34, G23, G12, G1
    case 34: seg = {0, 0 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 23: seg = {0, 1 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 12: seg = {0, 2 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 1:  seg = {0, 3 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    // L10 (B): order from pixel0 → G35, G24, G13, G2
    case 35: seg = {1, 0 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 24: seg = {1, 1 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 13: seg = {1, 2 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 2:  seg = {1, 3 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    default: return false; // Gates owned by other LED nodes
  }
}

// ======= Protocol (minimal) =======
enum MsgType : uint8_t { HELLO=1, HELLO_REQ=2, CLAIM=3, GATE_EVENT=10, LED_RANGE=20, BUTTON_EVENT=30 };
struct __attribute__((packed)) PktHeader { uint8_t type, version, nodeId, pad; uint16_t seq, len; };
static const uint8_t PROTO_VER = 1;

struct __attribute__((packed)) HelloMsg { PktHeader h; uint8_t role; uint8_t caps; };
struct __attribute__((packed)) ClaimMsg { PktHeader h; uint8_t newNodeId; };
struct __attribute__((packed)) GateEventMsg { PktHeader h; uint8_t gateId; uint8_t ev; uint16_t strengthMm; uint32_t tsMs; };
struct __attribute__((packed)) ButtonEventMsg { PktHeader h; uint8_t btnIdx; uint8_t ev; uint32_t tsMs; };
struct __attribute__((packed)) LedRangeMsg { PktHeader h; uint8_t strip; uint16_t start, count; uint8_t effect; uint8_t r,g,b; uint16_t durationMs; };

// ======= ESP‑NOW =======
static uint16_t gSeq=1; Preferences prefs; uint8_t gNodeId=0; // will be CLAIMed → persist
static uint8_t kBroadcast[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static void sendRaw(const uint8_t* mac, const uint8_t* data, size_t len) {
  esp_now_send(mac, data, len);
}

static void sendHello() {
  HelloMsg m{}; m.h={HELLO,PROTO_VER,gNodeId,0,gSeq++,sizeof(HelloMsg)}; m.role=1; m.caps=0; // role=1=node
  sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m));
}

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

static void onNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len){
  const uint8_t* mac = info ? info->src_addr : nullptr; // new ESP-NOW v3.x signature
  if (len < (int)sizeof(PktHeader)) return; auto *h=(const PktHeader*)data;
  if (h->version!=PROTO_VER) return;
  // Respond to HELLO_REQ so the server can re-roster us on demand
  if (h->type==HELLO_REQ) { sendHello(); return; }
  if (h->type==CLAIM && len>= (int)sizeof(ClaimMsg)) {
    auto *m=(const ClaimMsg*)data; gNodeId = m->newNodeId; prefs.begin("maze",false); prefs.putUChar("nodeId",gNodeId); prefs.end();
  }
  else if (h->type==LED_RANGE && len >= (int)sizeof(LedRangeMsg)) {
    auto *m=(const LedRangeMsg*)data; // Only implement solid fill for now
    if (m->strip==0) { // A
      for (uint16_t i=0; i<m->count; ++i) stripA.setPixelColor(m->start+i, stripA.Color(m->r,m->g,m->b));
    } else if (m->strip==1) { // B
      for (uint16_t i=0; i<m->count; ++i) stripB.setPixelColor(m->start+i, stripB.Color(m->r,m->g,m->b));
    }
  }
}

// ======= TCA9548A helpers =======
#define TCA_ADDR 0x70
static void tcaSelect(uint8_t ch){ Wire.beginTransmission(TCA_ADDR); Wire.write(1<<ch); Wire.endTransmission(); delayMicroseconds(200); }
static void tcaOff(){ Wire.beginTransmission(TCA_ADDR); Wire.write(0); Wire.endTransmission(); }

// ======= VL53L4CX =======
static VL53L4CX vl(&Wire, 0x52); // ST uses 0x52 (7‑bit 0x29 << 1)

// Node 4 TCA mapping (from CSV):
// ch0→port1: G23, ch1→port2: G24, ch2→port3: G28, ch3→port4: G29,
// ch4→port5: G34, ch5→port6: G35, ch6→port7: G39, ch7→port8: G40
static uint8_t kGateByTcaPort[8] = {
  /*ch0 (TCA port 1)*/ 23,
  /*ch1 (TCA port 2)*/ 24,
  /*ch2 (TCA port 3)*/ 28,
  /*ch3 (TCA port 4)*/ 29,
  /*ch4 (TCA port 5)*/ 34,
  /*ch5 (TCA port 6)*/ 35,
  /*ch6 (TCA port 7)*/ 39,
  /*ch7 (TCA port 8)*/ 40
};

// Detection params
static const uint16_t ABS_THRESH_MM = 1500; // fallback absolute
static const uint16_t DELTA_THRESH_MM = 600; // baseline‑delta
static const uint8_t  DEBOUNCE = 2; // consecutive samples
static const uint16_t REARM_MS = 500;

struct GateDet { bool present=false; uint8_t hits=0; uint32_t rearmUntil=0; uint16_t baseline=2000; };
static GateDet det[8]; // by TCA port

// ======= Render scheduling =======
static const uint32_t RENDER_MS = 33; // ~30fps
static uint32_t lastRender=0; static bool i2cBusy=false;
static void render(){
  if (millis()-lastRender < RENDER_MS) return; if (i2cBusy) return; lastRender=millis(); stripA.show(); stripB.show();
}

// ======= Buttons & Lamps =======
struct Btn { uint8_t pin; bool last; uint32_t lastEdge; };
static Btn btns[2] = { {PIN_BTN1, true, 0}, {PIN_BTN2, true, 0} };
static void pollButtons(){
  for (uint8_t i=0;i<2;i++){
    bool v = digitalRead(btns[i].pin); uint32_t now=millis();
    if (v!=btns[i].last && (now-btns[i].lastEdge)>30){ // debounce
      btns[i].last=v; btns[i].lastEdge=now;
      uint8_t ev = v?2:1; // 1=PRESS (LOW), 2=RELEASE (HIGH)
      sendButtonEvent(i+1, ev);
    }
  }
}

// ======= Setup =======
void setup(){
  Serial.begin(115200);
  pinMode(PIN_BTN1, INPUT_PULLUP); pinMode(PIN_BTN2, INPUT_PULLUP);
  pinMode(PIN_LAMP1, OUTPUT); pinMode(PIN_LAMP2, OUTPUT);
  digitalWrite(PIN_LAMP1, LOW); digitalWrite(PIN_LAMP2, LOW);

  stripA.begin(); stripB.begin(); stripA.clear(); stripB.clear(); stripA.show(); stripB.show();

  Wire.begin(PIN_SDA,PIN_SCL,100000);

  // ESP‑NOW init
  WiFi.mode(WIFI_STA); esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init()==ESP_OK){
    esp_now_register_recv_cb(onNowRecv);
    esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6); p.channel=6; p.encrypt=false; p.ifidx = WIFI_IF_STA; esp_now_add_peer(&p);
  }
  // Restore nodeId
  prefs.begin("maze", false); gNodeId = prefs.getUChar("nodeId", 4); prefs.end();
  sendHello();

  #if TOF_ENABLED
  // VL53 init on each active channel
  for (uint8_t ch=0; ch<8; ch++){
    if (kGateByTcaPort[ch]==0) continue; // unused channel
    tcaSelect(ch);
    if (vl.begin()!=0) { Serial.printf("[VL] ch%u begin failed
", ch); continue; }
    vl.InitSensor(0x52);
    vl.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
    vl.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000); // 33 ms
    vl.VL53L4CX_StartMeasurement();
  }
  tcaOff();
#else
  // ToF scanning is temporarily disabled (TOF_ENABLED=0)
#endif
}

// Sample one channel
static void sampleCh(uint8_t ch){
  if (kGateByTcaPort[ch]==0) return;
  i2cBusy=true; tcaSelect(ch);
  uint8_t ready=0; if (vl.VL53L4CX_GetMeasurementDataReady(&ready)!=0){ i2cBusy=false; return; }
  if (!ready){ i2cBusy=false; return; }
  VL53L4CX_MultiRangingData_t R{}; if (vl.VL53L4CX_GetMultiRangingData(&R)!=0){ i2cBusy=false; return; }
  uint16_t mm = (R.NumberOfObjectsFound > 0) ? R.RangeData[0].RangeMilliMeter : 8191; // 8191 ~ out-of-range // library layout
  vl.VL53L4CX_ClearInterruptAndStartMeasurement();
  tcaOff(); i2cBusy=false;

  // Baseline & detect
  auto &d = det[ch]; if (d.baseline<500 || d.baseline>4000) d.baseline = mm; // crude init
  uint16_t drop = (d.baseline>mm)? (d.baseline-mm):0;
  bool occ = (mm<ABS_THRESH_MM) || (drop>DELTA_THRESH_MM);
  if (millis()<d.rearmUntil) occ=false; // rearm window

  if (occ){ if (d.hits<255) d.hits++; } else if (d.hits>0) d.hits--; // simple debounce counter

  bool nowPresent = (d.hits>=DEBOUNCE);
  if (nowPresent && !d.present){
    d.present=true; d.rearmUntil = millis()+REARM_MS; // one ENTER per pass
    uint8_t gate = kGateByTcaPort[ch]; sendGateEvent(gate, /*ENTER*/1, mm);
  } else if (!nowPresent && d.present){
    d.present=false; // EXIT optional
    uint8_t gate = kGateByTcaPort[ch]; sendGateEvent(gate, /*EXIT*/2, mm);
  }
  // track baseline slowly when idle
  if (!nowPresent) d.baseline = (uint16_t)(0.9f*d.baseline + 0.1f*mm);
}

void loop(){
  // Poll sensors round‑robin
  #if TOF_ENABLED
  for (uint8_t ch=0; ch<8; ch++) sampleCh(ch);
#endif
  // Buttons
  pollButtons();
  // LED render frame
  render();
  // Local serial CLI: "sg <gateId> <r> <g> <b>" to light a segment for wiring check
  if (Serial.available()){
    String cmd = Serial.readStringUntil('\n'); cmd.trim();
    if (cmd.startsWith("sg ")){
      int gate=0,r=0,g=0,b=0; sscanf(cmd.c_str(),"sg %d %d %d %d", &gate,&r,&g,&b);
      GateSeg seg; if (getSegmentForGate((uint8_t)gate, seg)){
        uint32_t col = (uint32_t)((r&255)<<16)|((g&255)<<8)|((b&255));
        if (seg.strip==0){ for (uint16_t i=0;i<seg.count;i++) stripA.setPixelColor(seg.start+i, col); }
        else { for (uint16_t i=0;i<seg.count;i++) stripB.setPixelColor(seg.start+i, col); }
      }
    }
  }
}