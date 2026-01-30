#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <vector>
#include <cmath>
#include <time.h> // Pour l'heure NTP

// ==========================================
//  1. PARAMÈTRES DE MISE À JOUR (OTA) & WIFI
// ==========================================
const char* VERSION  = "2.5"; 
#ifndef WIFI_SSID
  #define WIFI_SSID "SSID_PAR_DEFAUT"
  #define WIFI_PASS "PASS_PAR_DEFAUT"
#endif

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;

const char* version_url = "https://raw.githubusercontent.com/louiscleon/starlings-box/main/version.txt";
const char* bin_url     = "https://raw.githubusercontent.com/louiscleon/starlings-box/build-bin/starlings-box.ino.bin";

// Paramètres Heure
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      // UTC+1 (Paris)
const int   daylightOffset_sec = 3600; // Heure d'été (+1h)

// ==========================================
//  2. PARAMÈTRES MURMURATION
// ==========================================
#define PANEL_RES_X 64
#define PANEL_RES_Y 32

static const int   NUM_BIRDS        = 60;
static const float MAX_SPEED        = 0.55f;
static const float MIN_SPEED        = 0.30f;
static const float NEIGHBOR_RAD     = 8.5f;
static const float SEP_DIST         = 2.6f;
static const float ALIGN_BASE       = 0.12f;
static const float COHESION_BASE    = 0.015f;
static const float SEPARATION_BASE  = 0.22f;
static const float BORDER_PUSH      = 0.18f;
static const float FLOW_STRENGTH    = 0.028f;
static const float FLOW_SCALE       = 0.085f;
static const float FLOW_SPEED       = 0.00028f;
static const uint32_t GUST_MIN_MS   = 3800;
static const uint32_t GUST_MAX_MS   = 10500;
static const uint32_t GUST_LEN_MS   = 520;
static const float    GUST_TURN     = 0.12f;
static const float    GUST_ALIGN_BOOST = 0.16f;
static const float DRIFT_STRENGTH   = 0.020f;
static const uint8_t DECAY_NUM      = 220;
static const uint8_t DEPOSIT_HEAD   = 230;
static const uint8_t DEPOSIT_TAIL   = 120;
static const int TRAIL_LEN          = 6;

struct Bird {
  float x, y;
  float vx, vy;
  float turnSignal;
  float tx[TRAIL_LEN];
  float ty[TRAIL_LEN];
  uint8_t tIdx;
};

MatrixPanel_I2S_DMA *dma_display = nullptr;
GFXcanvas1 *clock_mask = nullptr; // Masque invisible pour l'heure

static std::vector<Bird> birds;
static uint8_t  lum[PANEL_RES_X * PANEL_RES_Y];
static uint16_t palette[256];
static uint32_t lastFrame = 0, nextGustAt = 0, gustStart = 0, nextDriftAt = 0;
static float tFlow = 0.0f, driftX = 1.0f, driftY = 0.0f;
static bool gustOn = false;

// ==========================================
//  3. LOGIQUE DE MISE À JOUR & UTILS
// ==========================================

void check_for_updates() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, version_url);
  int httpCode = http.GET();
  if (httpCode == 200) {
    String newVersion = http.getString();
    newVersion.trim();
    if (newVersion != VERSION) {
      dma_display->fillScreen(0);
      dma_display->setCursor(2, 12);
      dma_display->setTextColor(dma_display->color565(255, 255, 255));
      dma_display->print("UPDATING...");
      dma_display->flipDMABuffer();
      httpUpdate.update(client, bin_url);
    }
  }
  http.end();
}

static inline float clampf(float v, float a, float b){ return (v<a)?a:(v>b)?b:v; }
static inline float fastInvSqrt(float x){ return 1.0f / sqrtf(x); }
static inline int idx(int x, int y){ return y * PANEL_RES_X + x; }
static inline bool inBounds(int x, int y){ return (x>=0 && x<PANEL_RES_X && y>=0 && y<PANEL_RES_Y); }

static inline uint32_t urand(uint32_t a, uint32_t b){
  return a + (uint32_t)(esp_random() % (b - a + 1));
}

static inline uint32_t hash2i(int x, int y) {
  uint32_t h = 2166136261u;
  h = (h ^ (uint32_t)x) * 16777619u;
  h = (h ^ (uint32_t)y) * 16777619u;
  h ^= (h >> 13); h *= 1274126177u; h ^= (h >> 16);
  return h;
}
static inline float rand01_from_hash(uint32_t h){ return (h & 0x00FFFFFF) / 16777215.0f; }
static inline float lerpf(float a, float b, float t){ return a + (b - a) * t; }
static inline float smooth(float t){ return t * t * (3.0f - 2.0f * t); }

static float noise2D(float x, float y){
  int x0 = (int)floorf(x), y0 = (int)floorf(y);
  float tx = smooth(x - x0), ty = smooth(y - y0);
  float v00 = rand01_from_hash(hash2i(x0, y0)), v10 = rand01_from_hash(hash2i(x0+1, y0));
  float v01 = rand01_from_hash(hash2i(x0, y0+1)), v11 = rand01_from_hash(hash2i(x0+1, y0+1));
  return lerpf(lerpf(v00, v10, tx), lerpf(v01, v11, tx), ty);
}

static inline void flowVector(float x, float y, float &fx, float &fy){
  float n = noise2D(x * FLOW_SCALE + tFlow, y * FLOW_SCALE - tFlow);
  float ang = (n * 6.2831853f);
  fx = cosf(ang); fy = sinf(ang);
}

static void buildPalette() {
  const int sr=255, sg=255, sb=255; // Fond pur blanc
  for (int i=0;i<256;i++){
    float k = i / 255.0f;
    // Dégradé du blanc (i=0) vers le noir (i=255)
    palette[i] = dma_display->color565((int)(sr*(1.0f-0.95f*k)), (int)(sg*(1.0f-0.95f*k)), (int)(sb*(1.0f-0.95f*k)));
  }
}

static void scheduleNextGust(uint32_t now){ nextGustAt = now + urand(GUST_MIN_MS, GUST_MAX_MS); }

static void updateGust(uint32_t now){
  if (!gustOn && now >= nextGustAt){ gustOn = true; gustStart = now; }
  if (gustOn && (now - gustStart) > GUST_LEN_MS){ gustOn = false; scheduleNextGust(now); }
}

static void updateDrift(uint32_t now){
  if (now >= nextDriftAt){
    nextDriftAt = now + urand(5000, 14000);
    float a = (esp_random()%628) / 100.0f;
    driftX = cosf(a); driftY = sinf(a);
  }
}

static void initBirds() {
  birds.clear();
  birds.reserve(NUM_BIRDS);
  for(int i=0;i<NUM_BIRDS;i++){
    Bird b;
    b.x = PANEL_RES_X*0.5f + (int)(esp_random()%28)-14;
    b.y = PANEL_RES_Y*0.5f + (int)(esp_random()%16)-8;
    float a = (esp_random()%628)/100.0f;
    float sp = 0.55f + (esp_random()%55)/100.0f;
    b.vx = cosf(a)*sp; b.vy = sinf(a)*sp;
    b.turnSignal = 0.0f; b.tIdx = 0;
    for(int t=0;t<TRAIL_LEN;t++){ b.tx[t]=b.x; b.ty[t]=b.y; }
    birds.push_back(b);
  }
}

static void applyBorders(Bird &b){
  const float m = 3.0f;
  if (b.x < m) b.vx += BORDER_PUSH;
  if (b.x > PANEL_RES_X - 1 - m) b.vx -= BORDER_PUSH;
  if (b.y < m) b.vy += BORDER_PUSH;
  if (b.y > PANEL_RES_Y - 1 - m) b.vy -= BORDER_PUSH;
}

static void limitSpeed(Bird &b){
  float s2 = b.vx*b.vx + b.vy*b.vy;
  if (s2 < 1e-6f) { b.vx = MIN_SPEED; b.vy = 0; return; }
  float inv = fastInvSqrt(s2);
  if ((1.0f/inv) > MAX_SPEED){ b.vx *= MAX_SPEED*inv; b.vy *= MAX_SPEED*inv; }
  else if ((1.0f/inv) < MIN_SPEED){ b.vx *= MIN_SPEED*inv; b.vy *= MIN_SPEED*inv; }
}

static void updateBird(int i){
  Bird &b = birds[i];
  float ax=0, ay=0, cx=0, cy=0, sx=0, sy=0, tx=0, ty=0; int n=0;
  for(int j=0;j<NUM_BIRDS;j++){
    if (j==i) continue;
    Bird &o = birds[j];
    float dx = o.x-b.x, dy = o.y-b.y, d2 = dx*dx+dy*dy;
    if (d2 < (NEIGHBOR_RAD*NEIGHBOR_RAD) && d2 > 1e-4f){
      float invd = fastInvSqrt(d2); float d = 1.0f/invd;
      ax += o.vx; ay += o.vy; cx += o.x; cy += o.y;
      if (d < SEP_DIST){ float w = pow((SEP_DIST-d)/SEP_DIST,2); sx -= dx*invd*w; sy -= dy*invd*w; }
      tx += o.turnSignal*(-dy)*invd; ty += o.turnSignal*(dx)*invd; n++;
    }
  }
  float fx, fy; flowVector(b.x, b.y, fx, fy);
  float alignF = ALIGN_BASE + (gustOn ? GUST_ALIGN_BOOST : 0.0f);
  if (n>0){
    float invn = 1.0f/n; b.vx += (ax*invn-b.vx)*alignF; b.vy += (ay*invn-b.vy)*alignF;
    b.vx += (cx*invn-b.x)*COHESION_BASE; b.vy += (cy*invn-b.y)*COHESION_BASE;
    b.vx += sx*SEPARATION_BASE; b.vy += sy*SEPARATION_BASE; b.vx += tx*0.010f; b.vy += ty*0.010f;
  }
  b.vx += fx*FLOW_STRENGTH+driftX*DRIFT_STRENGTH; b.vy += fy*FLOW_STRENGTH+driftY*DRIFT_STRENGTH;
  if (gustOn){ float rvx=-b.vy, rvy=b.vx; b.vx+=rvx*GUST_TURN; b.vy+=rvy*GUST_TURN; }
  applyBorders(b); limitSpeed(b);
  b.tx[b.tIdx] = b.x; b.ty[b.tIdx] = b.y; b.tIdx = (b.tIdx+1)%TRAIL_LEN;
  b.x += b.vx; b.y += b.vy;
  b.x = clampf(b.x, 0, PANEL_RES_X-1); b.y = clampf(b.y, 0, PANEL_RES_Y-1);
}

// ==========================================
//  4. GESTION DE L'HEURE (POCHOIR)
// ==========================================

void updateClockMask() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    return; // Pas encore synchronisé
  }
  char timeString[9];
  strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
  
  clock_mask->fillScreen(0); // On efface le masque
  clock_mask->setCursor(8, 2); // Centré dans le quart supérieur
  clock_mask->setTextColor(1); // On dessine en "allumé" (blanc dans le masque)
  clock_mask->print(timeString);
}

static void decayLuminance(){ 
  for(int i=0; i<PANEL_RES_X*PANEL_RES_Y; i++) lum[i] = (uint8_t)((lum[i]*DECAY_NUM)>>8); 
}

static void deposit(int x, int y, uint8_t amount){ 
  if(inBounds(x,y)){ 
    int k=idx(x,y); 
    int v=lum[k]+amount; 
    lum[k]=(v>255)?255:(uint8_t)v; 
  } 
}

static void renderToPanel(){ 
  for(int y=0; y<PANEL_RES_Y; y++){
    for(int x=0; x<PANEL_RES_X; x++){
      int k = idx(x,y);
      uint8_t luminance = lum[k];

      // LOGIQUE POCHOIR : 
      // Si le pixel fait partie de l'heure dans le masque, on force le BLANC (palette 0)
      if (clock_mask->getPixel(x, y)) {
        dma_display->drawPixel(x, y, palette[0]);
      } else {
        dma_display->drawPixel(x, y, palette[luminance]);
      }
    }
  }
}

// ==========================================
//  5. SETUP & LOOP
// ==========================================

void setup() {
  Serial.begin(115200);

  // Initialisation Matrix
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, 1);
  mxconfig.double_buff = true;
  mxconfig.gpio.r1 = 25; mxconfig.gpio.g1 = 26; mxconfig.gpio.b1 = 27;
  mxconfig.gpio.r2 = 14; mxconfig.gpio.g2 = 12; mxconfig.gpio.b2 = 13;
  mxconfig.gpio.a  = 23; mxconfig.gpio.b  = 19; mxconfig.gpio.c  = 5;  mxconfig.gpio.d = 17;
  mxconfig.gpio.lat = 4; mxconfig.gpio.oe = 15; mxconfig.gpio.clk = 16;
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(150);

  // Initialisation du masque pour l'heure
  clock_mask = new GFXcanvas1(PANEL_RES_X, PANEL_RES_Y);
  clock_mask->setTextWrap(false);

  // WiFi avec boucle d'attente
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Synchronisation de l'heure NTP
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    check_for_updates();
  }

  memset(lum, 0, sizeof(lum));
  initBirds();
  buildPalette();
  scheduleNextGust(millis());
  nextDriftAt = millis() + urand(2500, 8000);
  lastFrame = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t dt = now - lastFrame;
  lastFrame = now;
  tFlow += dt * FLOW_SPEED;

  // Mise à jour de l'heure (toutes les secondes)
  static uint32_t lastClockUpdate = 0;
  if (now - lastClockUpdate > 1000) {
    updateClockMask();
    lastClockUpdate = now;
  }

  updateGust(now);
  updateDrift(now);
  for(int i=0; i<NUM_BIRDS; i++) updateBird(i);
  
  decayLuminance();
  
  for(int i=0; i<NUM_BIRDS; i++){
    Bird &b = birds[i];
    deposit((int)lroundf(b.x), (int)lroundf(b.y), DEPOSIT_HEAD);
    for(int t=1; t<TRAIL_LEN; t++){
      int age = (b.tIdx-t+TRAIL_LEN)%TRAIL_LEN;
      deposit((int)lroundf(b.tx[age]), (int)lroundf(b.ty[age]), (uint8_t)(DEPOSIT_TAIL*(1.0f-(t/(float)TRAIL_LEN))));
    }
  }
  
  renderToPanel();
  dma_display->flipDMABuffer();

  // Mise à jour auto toutes les 30 min
  static uint32_t lastUpdateCheck = 0;
  if (now - lastUpdateCheck > 1800000) {
    check_for_updates();
    lastUpdateCheck = now;
  }
  delay(10);
}
