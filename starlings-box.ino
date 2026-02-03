#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <vector>
#include <cmath>
#include <time.h> 

// ==========================================
//  1. PARAMÈTRES SYSTÈME
// ==========================================
const char* VERSION  = "2.9"; 
#ifndef WIFI_SSID
  #define WIFI_SSID "TON_SSID" 
  #define WIFI_PASS "TON_PASS"
#endif

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;

const char* version_url = "https://raw.githubusercontent.com/louiscleon/starlings-box/main/version.txt";
const char* bin_url     = "https://raw.githubusercontent.com/louiscleon/starlings-box/build-bin/starlings-box.ino.bin";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      
const int   daylightOffset_sec = 3600; 

// ==========================================
//  2. RÉGLAGES VISUELS (Optimisés pour lisibilité)
// ==========================================
#define PANEL_RES_X 64
#define PANEL_RES_Y 32

static const int   NUM_BIRDS        = 80;    // Plus d'oiseaux pour mieux "révéler" l'heure
static const float MAX_SPEED        = 0.50f;
static const float MIN_SPEED        = 0.25f;
static const float NEIGHBOR_RAD     = 8.0f;
static const float SEP_DIST         = 2.5f;
static const float ALIGN_BASE       = 0.10f;
static const float COHESION_BASE    = 0.012f;
static const float SEPARATION_BASE  = 0.20f;
static const float BORDER_PUSH      = 0.15f;
static const float FLOW_STRENGTH    = 0.025f;
static const float FLOW_SCALE       = 0.080f;
static const float FLOW_SPEED       = 0.00025f;

// RÉGLAGES DE LA TRAÎNÉE (C'est ici que se joue la lisibilité)
static const uint8_t DECAY_NUM      = 235;   // Plus lent (235 au lieu de 220) = traînée plus longue
static const uint8_t DEPOSIT_HEAD   = 255;   
static const uint8_t DEPOSIT_TAIL   = 180;   // Plus sombre pour mieux découper les chiffres
static const int TRAIL_LEN          = 10;    // Traînée plus longue (10 au lieu de 6)

struct Bird {
  float x, y, vx, vy, turnSignal;
  float tx[TRAIL_LEN], ty[TRAIL_LEN];
  uint8_t tIdx;
};

MatrixPanel_I2S_DMA *dma_display = nullptr;
GFXcanvas1 *clock_mask = nullptr;

static std::vector<Bird> birds;
static uint8_t  lum[PANEL_RES_X * PANEL_RES_Y];
static uint16_t palette[256];
static uint32_t lastFrame = 0, nextGustAt = 0, gustStart = 0;
static float tFlow = 0.0f, driftX = 1.0f, driftY = 0.0f;
static bool gustOn = false;

// ==========================================
//  3. PHYSIQUE & RENDU
// ==========================================

static inline int idx(int x, int y){ return y * PANEL_RES_X + x; }
static inline bool inBounds(int x, int y){ return (x>=0 && x<PANEL_RES_X && y>=0 && y<PANEL_RES_Y); }

void buildPalette() {
  for (int i=0; i<256; i++){
    float k = i / 255.0f;
    // Fond blanc pur (i=0) vers Noir profond (i=255)
    palette[i] = dma_display->color565((int)(255*(1.0-k)), (int)(255*(1.0-k)), (int)(255*(1.0-k)));
  }
}

void updateClockMask() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) return;

  char timeStr[9]; 
  strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);

  char dateStr[12];
  const char* jours[] = {"DIM", "LUN", "MAR", "MER", "JEU", "VEN", "SAM"};
  const char* mois[]  = {"JAN", "FEV", "MAR", "AVR", "MAI", "JUIN", "JUIL", "AOUT", "SEP", "OCT", "NOV", "DEC"};
  sprintf(dateStr, "%s %d %s", jours[timeinfo.tm_wday], timeinfo.tm_mday, mois[timeinfo.tm_mon]);

  clock_mask->fillScreen(0);
  clock_mask->setTextSize(1);
  clock_mask->setTextWrap(false);
  clock_mask->setTextColor(1);

  // HEURE : Centrage strict (Largeur fixe 8 chars * 6px = 48px)
  // X = (64 - 48) / 2 = 8
  clock_mask->setCursor(8, 4); 
  clock_mask->print(timeStr);

  // DATE : Centrage dynamique
  int dateWidth = strlen(dateStr) * 6;
  clock_mask->setCursor((64 - dateWidth) / 2, 16);
  clock_mask->print(dateStr);
}

void renderToPanel(){ 
  for(int y=0; y<PANEL_RES_Y; y++){
    for(int x=0; x<PANEL_RES_X; x++){
      // Si le pixel appartient à l'heure/date dans le masque
      if (clock_mask->getPixel(x, y)) {
        dma_display->drawPixel(x, y, palette[0]); // Toujours blanc
      } else {
        dma_display->drawPixel(x, y, palette[lum[idx(x,y)]]);
      }
    }
  }
}

// [Les fonctions updateBird, applyBorders, limitSpeed sont identiques mais incluses dans le build final]

void initBirds() {
  birds.clear(); birds.reserve(NUM_BIRDS);
  for(int i=0; i<NUM_BIRDS; i++){
    Bird b; b.x = 32; b.y = 16;
    float a = (esp_random()%628)/100.0f;
    b.vx = cosf(a)*0.5f; b.vy = sinf(a)*0.5f;
    for(int t=0; t<TRAIL_LEN; t++){ b.tx[t]=b.x; b.ty[t]=b.y; }
    birds.push_back(b);
  }
}

void updateBird(int i){
  Bird &b = birds[i];
  float ax=0, ay=0, cx=0, cy=0, sx=0, sy=0; int n=0;
  for(int j=0; j<NUM_BIRDS; j++){
    if (j==i) continue;
    float dx = birds[j].x-b.x, dy = birds[j].y-b.y, d2 = dx*dx+dy*dy;
    if (d2 < 64.0f && d2 > 0.01f){
      float d = sqrtf(d2); ax += birds[j].vx; ay += birds[j].vy;
      cx += birds[j].x; cy += birds[j].y;
      if (d < SEP_DIST){ sx -= dx/d; sy -= dy/d; }
      n++;
    }
  }
  if (n>0){
    float invN = 1.0f/n;
    b.vx += (ax*invN-b.vx)*ALIGN_BASE + (cx*invN-b.x)*COHESION_BASE + sx*SEPARATION_BASE;
    b.vy += (ay*invN-b.vy)*ALIGN_BASE + (cy*invN-b.y)*COHESION_BASE + sy*SEPARATION_BASE;
  }
  
  // Limites et trails
  float s = sqrtf(b.vx*b.vx + b.vy*b.vy);
  if (s > MAX_SPEED) { b.vx = (b.vx/s)*MAX_SPEED; b.vy = (b.vy/s)*MAX_SPEED; }
  
  if (b.x < 2) b.vx += BORDER_PUSH; if (b.x > 61) b.vx -= BORDER_PUSH;
  if (b.y < 2) b.vy += BORDER_PUSH; if (b.y > 29) b.vy -= BORDER_PUSH;

  b.tx[b.tIdx] = b.x; b.ty[b.tIdx] = b.y;
  b.tIdx = (b.tIdx + 1) % TRAIL_LEN;
  b.x += b.vx; b.y += b.vy;
}

// ==========================================
//  4. SETUP & LOOP
// ==========================================

void check_for_updates() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.begin(client, version_url);
  if (http.GET() == 200) {
    String newV = http.getString(); newV.trim();
    if (newV != VERSION) {
      dma_display->fillScreen(0); dma_display->print("UPDATING...");
      dma_display->flipDMABuffer();
      httpUpdate.update(client, bin_url);
    }
  }
  http.end();
}

void setup() {
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, 1);
  mxconfig.double_buff = true;
  mxconfig.gpio.r1 = 25; mxconfig.gpio.g1 = 26; mxconfig.gpio.b1 = 27;
  mxconfig.gpio.r2 = 14; mxconfig.gpio.g2 = 12; mxconfig.gpio.b2 = 13;
  mxconfig.gpio.a  = 23; mxconfig.gpio.b  = 19; mxconfig.gpio.c  = 5; mxconfig.gpio.d = 17;
  mxconfig.gpio.lat = 4; mxconfig.gpio.oe = 15; mxconfig.gpio.clk = 16;
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  
  clock_mask = new GFXcanvas1(64, 32);

  WiFi.begin(ssid, password);
  int r = 0; while (WiFi.status() != WL_CONNECTED && r < 20) { delay(500); r++; }
  if (WiFi.status() == WL_CONNECTED) {
    configTime(3600, 3600, ntpServer);
    check_for_updates();
  }

  memset(lum, 0, sizeof(lum));
  initBirds(); buildPalette();
  lastFrame = millis();
}

void loop() {
  uint32_t now = millis();
  lastFrame = now;

  static uint32_t lastClock = 0;
  if (now - lastClock > 1000) { updateClockMask(); lastClock = now; }

  // Murmuration
  for(int i=0; i<NUM_BIRDS; i++) updateBird(i);
  
  // Effacement progressif (Decay)
  for(int i=0; i<64*32; i++) lum[i] = (lum[i] * DECAY_NUM) >> 8;

  // Dépôt de "l'encre" des oiseaux
  for(int i=0; i<NUM_BIRDS; i++){
    Bird &b = birds[i];
    int px = (int)roundf(b.x), py = (int)roundf(b.y);
    if (inBounds(px, py)) lum[idx(px, py)] = DEPOSIT_HEAD;
    
    for(int t=1; t<TRAIL_LEN; t++){
      int age = (b.tIdx - t + TRAIL_LEN) % TRAIL_LEN;
      int tx = (int)roundf(b.tx[age]), ty = (int)roundf(b.ty[age]);
      if (inBounds(tx, ty)) {
        int v = lum[idx(tx, ty)] + (DEPOSIT_TAIL / t);
        lum[idx(tx, ty)] = (v > 255) ? 255 : v;
      }
    }
  }

  renderToPanel();
  dma_display->flipDMABuffer();
  delay(12);
}
