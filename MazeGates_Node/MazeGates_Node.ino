/*
 * Maze Gates – Node 4 Foundation v0 (clean compile, no serial CLI)
 * Board: Unexpected Maker FeatherS3 (ESP32‑S3)
 * Transport: ESP‑NOW ch.6 (OTA .bin supported; no maintenance/long‑press)
 * Sensors: 8× VL53L4CX via TCA9548A @ 0x70 (temporarily disabled)
 * LEDs: L9@GPIO10 (G34,G23,G12,G1), L10@GPIO11 (G35,G24,G13,G2)
 * Buttons: B1=GPIO17, B2=GPIO18  (INPUT_PULLUP)
 * Lamps (MOSFET, 5V): LAMP1=GPIO12, LAMP2=GPIO6
 * I2C: SDA=8, SCL=9, 100 kHz
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include "vl53l4cx_class.h"   // STM32duino VL53L4CX by ST

// --- OTA defaults (hardcoded) ---
#ifndef OTA_SSID
#define OTA_SSID "GUD"
#define OTA_PASS "EscapE66"
#define OTA_BASE_URL "http://192.168.2.231:8000/"
#define OTA_NODE_PATH "MazeGates/MazeGates_Node/build/esp32.esp32.um_feathers3/MazeGates_Node.ino.bin"
#endif

// ===== Feature switches =====
#ifndef TOF_ENABLED
#define TOF_ENABLED 0   // temporarily disable ToF scanning
#endif

// ======= Pins =======
#define PIN_SDA 8
#define PIN_SCL 9
#define PIN_LED_STRIP_A 10  // L9
#define PIN_LED_STRIP_B 11  // L10
#define PIN_BTN1 17
#define PIN_BTN2 18
#define PIN_LAMP1 12
#define PIN_LAMP2 6

// ======= LED strips =======
#define PIXELS_PER_GATE 60
#define STRIP_A_PIX (4 * PIXELS_PER_GATE) // G34,G23,G12,G1
#define STRIP_B_PIX (4 * PIXELS_PER_GATE) // G35,G24,G13,G2
Adafruit_NeoPixel stripA(STRIP_A_PIX, PIN_LED_STRIP_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripB(STRIP_B_PIX, PIN_LED_STRIP_B, NEO_GRB + NEO_KHZ800);

struct GateSeg { uint8_t strip; uint16_t start; uint16_t count; };
static bool getSegmentForGate(uint8_t gateId, GateSeg &seg) {
  switch (gateId) {
    // L9 (A): pixel0 → G34, G23, G12, G1
    case 34: seg = {0, 0 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 23: seg = {0, 1 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 12: seg = {0, 2 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 1:  seg = {0, 3 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    // L10 (B): pixel0 → G35, G24, G13, G2
    case 35: seg = {1, 0 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 24: seg = {1, 1 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 13: seg = {1, 2 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    case 2:  seg = {1, 3 * PIXELS_PER_GATE, PIXELS_PER_GATE}; return true;
    default: return false;
  }
}

// ======= Protocol =======
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

// ======= ESP‑NOW =======
static volatile bool gOtaPending = false;
static char gOtaUrlBuf[200] = {0};
static uint16_t gSeq=1; Preferences prefs; uint8_t gNodeId=0; // CLAIMed → persist
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
static void sendOtaAck(uint8_t status){
  OtaAckMsg m{}; m.h={OTA_ACK,PROTO_VER,gNodeId,0,gSeq++,sizeof(OtaAckMsg)}; m.status=status;
  sendRaw(kBroadcast,(uint8_t*)&m,sizeof(m));
}

static void onNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len){
  if (!info || len < (int)sizeof(PktHeader)) return; auto *h=(const PktHeader*)data; if (h->version!=PROTO_VER) return;
  // Re‑HELLO on demand
  if (h->type==HELLO_REQ) { sendHello(); return; }
  // Persist CLAIM
  if (h->type==CLAIM && len>= (int)sizeof(ClaimMsg)) {
    auto *m=(const ClaimMsg*)data; gNodeId = m->newNodeId; prefs.begin("maze",false); prefs.putUChar("nodeId",gNodeId); prefs.end();
    Serial.printf("[CLAIM] nodeId set to %u", gNodeId); Serial.println();
    sendHello();
    return;
  }
  // LED paint (solid for now)
  if (h->type==LED_RANGE && len >= (int)sizeof(LedRangeMsg)) {
    auto *m=(const LedRangeMsg*)data;
    if (m->strip==0) { for (uint16_t i=0; i<m->count; ++i) stripA.setPixelColor(m->start+i, stripA.Color(m->r,m->g,m->b)); }
    else if (m->strip==1) { for (uint16_t i=0; i<m->count; ++i) stripB.setPixelColor(m->start+i, stripB.Color(m->r,m->g,m->b)); }
    return;
  }
  // Lamps
  if (h->type==LAMP_CTRL && len >= (int)sizeof(LampCtrlMsg)) {
    auto *m=(const LampCtrlMsg*)data; if (m->idx==1) digitalWrite(PIN_LAMP1, m->on?HIGH:LOW); else if (m->idx==2) digitalWrite(PIN_LAMP2, m->on?HIGH:LOW);
    return;
  }
  // OTA
  if (h->type==OTA_START && len >= (int)sizeof(PktHeader)) {
    String url="";
    if (len > (int)sizeof(PktHeader)){
      const char* u = (const char*)(data + sizeof(PktHeader));
      if (u && *u) url = String(u);
    }
    sendOtaAck(0);
    // Defer Wi‑Fi switch/HTTP update to the main loop (like Pizza) to avoid doing it in the recv callback
    if (url.length() > 0) { strncpy(gOtaUrlBuf, url.c_str(), sizeof(gOtaUrlBuf)-1); }
    else { gOtaUrlBuf[0] = '�'; }
    gOtaPending = true;
    return;
  }
}

// ======= OTA (.bin over HTTP) =======
static String loadWifiSsid(){ prefs.begin("maze", true); String s=prefs.getString("ota_ssid",""); prefs.end(); return s; }
static String loadWifiPass(){ prefs.begin("maze", true); String s=prefs.getString("ota_pass",""); prefs.end(); return s; }
static void saveWifiCreds(const String& ssid,const String& pass){ prefs.begin("maze", false); prefs.putString("ota_ssid",ssid); prefs.putString("ota_pass",pass); prefs.end(); }

static void drawOtaProgress(size_t done, size_t total){
  float f = total? (float)done/(float)total : 0.f;
  uint32_t totalPix = STRIP_A_PIX + STRIP_B_PIX;
  uint32_t lit = (uint32_t)(f * totalPix + 0.5f);
  for (uint16_t i=0;i<STRIP_A_PIX;i++) stripA.setPixelColor(i, 0);
  for (uint16_t i=0;i<STRIP_B_PIX;i++) stripB.setPixelColor(i, 0);
  uint32_t col = stripA.Color(0, 150, 0);
  // Fill A first
  uint32_t fillA = (lit <= STRIP_A_PIX) ? lit : STRIP_A_PIX;
  for (uint16_t i=0;i<fillA;i++) stripA.setPixelColor(i, col);
  // Remaining to B
  if (lit > STRIP_A_PIX){
    uint32_t left = lit-STRIP_A_PIX;
    uint32_t fillB = (left <= STRIP_B_PIX) ? left : STRIP_B_PIX;
    for (uint16_t i=0;i<fillB;i++) stripB.setPixelColor(i, col);
  }
  stripA.show(); stripB.show();
}
static void showOtaError(){
  for(int k=0;k<3;k++){
    for(uint16_t i=0;i<STRIP_A_PIX;i++) stripA.setPixelColor(i, stripA.Color(150,0,0));
    for(uint16_t i=0;i<STRIP_B_PIX;i++) stripB.setPixelColor(i, stripB.Color(150,0,0));
    stripA.show(); stripB.show(); delay(150);
    for(uint16_t i=0;i<STRIP_A_PIX;i++) stripA.setPixelColor(i, 0);
    for(uint16_t i=0;i<STRIP_B_PIX;i++) stripB.setPixelColor(i, 0);
    stripA.show(); stripB.show(); delay(150);
  }
}
static void showOtaSuccess(){
  for(uint16_t i=0;i<STRIP_A_PIX;i++) stripA.setPixelColor(i, stripA.Color(0,150,0));
  for(uint16_t i=0;i<STRIP_B_PIX;i++) stripB.setPixelColor(i, stripB.Color(0,150,0));
  stripA.show(); stripB.show();
}

static void stopEspNow(){ esp_now_deinit(); }
static esp_err_t startEspNow(){ if (esp_now_init()!=ESP_OK) return ESP_FAIL; esp_now_register_recv_cb(onNowRecv); esp_now_peer_info_t p{}; memcpy(p.peer_addr,kBroadcast,6); p.channel=6; p.encrypt=false; p.ifidx=WIFI_IF_STA; esp_now_add_peer(&p); return ESP_OK; }
static bool connectWifiSta(const String& ssid, const String& pass, uint32_t timeoutMs=15000){ WiFi.disconnect(true); WiFi.mode(WIFI_STA); WiFi.begin(ssid.c_str(), pass.c_str()); uint32_t t0=millis(); while (WiFi.status()!=WL_CONNECTED && (millis()-t0)<timeoutMs) delay(100); return WiFi.status()==WL_CONNECTED; }
static void performHttpOta(String url, String ssid, String pass){
  stopEspNow();
  if (ssid.length()==0) ssid = OTA_SSID;
  if (pass.length()==0) pass = OTA_PASS;
  if (url.length()==0)  url  = String(OTA_BASE_URL) + OTA_NODE_PATH;
  if (!connectWifiSta(ssid, pass)){
    Serial.println("[OTA] WiFi connect failed");
    showOtaError();
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    startEspNow();
    return;
  }
  Serial.printf("[OTA] Connected %s IP=%s", ssid.c_str(), WiFi.localIP().toString().c_str()); Serial.println();
  Update.onProgress([](size_t d,size_t t){ drawOtaProgress(d,t); });
  WiFiClient client; HTTPUpdate httpUpdate; httpUpdate.rebootOnUpdate(false);
  Serial.printf("[OTA] GET %s", url.c_str()); Serial.println();
  t_httpUpdate_return r = httpUpdate.update(client, url);
  if (r==HTTP_UPDATE_OK){
    Serial.println("[OTA] OK, rebooting");
    showOtaSuccess(); delay(400); ESP.restart();
  } else {
    Serial.printf("[OTA] Fail code=%d", (int)r); Serial.println();
    Serial.printf("[OTA] Error: %s", httpUpdate.getLastErrorString().c_str()); Serial.println();
    showOtaError();
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    startEspNow();
  }
}

// ======= TCA9548A helpers =======
#define TCA_ADDR 0x70
static void tcaSelect(uint8_t ch){ Wire.beginTransmission(TCA_ADDR); Wire.write(1<<ch); Wire.endTransmission(); delayMicroseconds(200); }
static void tcaOff(){ Wire.beginTransmission(TCA_ADDR); Wire.write(0); Wire.endTransmission(); }

// ======= VL53L4CX =======
static VL53L4CX vl(&Wire, 0x52); // ST uses 0x52 (7‑bit 0x29 << 1)

// Node 4 TCA mapping (from CSV)
static uint8_t kGateByTcaPort[8] = {
  /*ch0 (port1)*/ 23,
  /*ch1 (port2)*/ 24,
  /*ch2 (port3)*/ 28,
  /*ch3 (port4)*/ 29,
  /*ch4 (port5)*/ 34,
  /*ch5 (port6)*/ 35,
  /*ch6 (port7)*/ 39,
  /*ch7 (port8)*/ 40
};

// Detection params
static const uint16_t ABS_THRESH_MM = 1500; // fallback absolute
static const uint16_t DELTA_THRESH_MM = 600; // baseline‑delta
static const uint8_t  DEBOUNCE = 2; // consecutive samples
static const uint16_t REARM_MS = 500;

struct GateDet { bool present=false; uint8_t hits=0; uint32_t rearmUntil=0; uint16_t baseline=2000; };
static GateDet det[8]; // by TCA port

// ======= Render scheduling =======
static const uint32_t RENDER_MS = 33; // ~30 fps
static uint32_t lastRender=0; static bool i2cBusy=false;
static void render(){ if (millis()-lastRender < RENDER_MS) return; if (i2cBusy) return; lastRender=millis(); stripA.show(); stripB.show(); }

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
    if (vl.begin()!=0) { Serial.printf("[VL] ch%u begin failed", ch); Serial.println(); continue; }
    vl.InitSensor(0x52);
    vl.VL53L4CX_SetDistanceMode(VL53L4CX_DISTANCEMODE_LONG);
    vl.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000); // 33 ms
    vl.VL53L4CX_StartMeasurement();
  }
  tcaOff();
#endif
}

// Sample one channel
static void sampleCh(uint8_t ch){
  if (kGateByTcaPort[ch]==0) return;
  i2cBusy=true; tcaSelect(ch);
  uint8_t ready=0; if (vl.VL53L4CX_GetMeasurementDataReady(&ready)!=0){ i2cBusy=false; return; }
  if (!ready){ i2cBusy=false; return; }
  VL53L4CX_MultiRangingData_t R{}; if (vl.VL53L4CX_GetMultiRangingData(&R)!=0){ i2cBusy=false; return; }
  uint16_t mm = (R.NumberOfObjectsFound > 0) ? R.RangeData[0].RangeMilliMeter : 8191;
  vl.VL53L4CX_ClearInterruptAndStartMeasurement();
  tcaOff(); i2cBusy=false;

  // Baseline & detect
  auto &d = det[ch]; if (d.baseline<500 || d.baseline>4000) d.baseline = mm; // init
  uint16_t drop = (d.baseline>mm)? (d.baseline-mm):0;
  bool occ = (mm<ABS_THRESH_MM) || (drop>DELTA_THRESH_MM);
  if (millis()<d.rearmUntil) occ=false; // rearm window

  if (occ){ if (d.hits<255) d.hits++; } else if (d.hits>0) d.hits--; // debounce counter

  bool nowPresent = (d.hits>=DEBOUNCE);
  if (nowPresent && !d.present){
    d.present=true; d.rearmUntil = millis()+REARM_MS; // ENTER
    uint8_t gate = kGateByTcaPort[ch]; sendGateEvent(gate, /*ENTER*/1, mm);
  } else if (!nowPresent && d.present){
    d.present=false; // EXIT
    uint8_t gate = kGateByTcaPort[ch]; sendGateEvent(gate, /*EXIT*/2, mm);
  }
  if (!nowPresent) d.baseline = (uint16_t)(0.9f*d.baseline + 0.1f*mm);
}

void loop(){
#if TOF_ENABLED
  for (uint8_t ch=0; ch<8; ch++) sampleCh(ch);
#endif
  pollButtons();
  render();

  // Run deferred OTA here (not inside ESP‑NOW callback)
  if (gOtaPending) {
    gOtaPending = false;
    String url = (gOtaUrlBuf[0] ? String(gOtaUrlBuf) : String());
    performHttpOta(url, String(), String());
  }
}