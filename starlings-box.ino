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
//  1. PARAMÈTRES OTA & WIFI
// ==========================================
const char* VERSION  = "2.5.1"; 
#ifndef WIFI_SSID
  #define WIFI_SSID "SSID_PAR_DEFAUT"
  #define WIFI_PASS "PASS_PAR_DEFAUT"
#endif

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASS;

const char* version_url = "https://raw.githubusercontent.com/louiscleon/starlings-box/main/version.txt";
const char* bin_url     = "https://raw.githubusercontent.com/louiscleon/starlings-box/build-bin/starlings-box.ino.bin";

// Paramètres Heure (NTP)
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;      
const int   daylightOffset_sec = 3600; 

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
  float x, y, vx, vy, turnSignal;
  float tx[TRAIL_LEN], ty[TRAIL_LEN];
  uint8_t tIdx;
};

MatrixPanel_I2S_DMA *dma_display = nullptr;
GFXcanvas1 *clock_mask = nullptr; // Le masque binaire pour l'heure

static std::vector<Bird> birds;
static uint8_t  lum[PANEL_RES_X * PANEL_RES_Y];
static uint16_t palette[256];
static uint32_t lastFrame = 0, nextGustAt = 0, gustStart = 0, nextDriftAt = 0;
static float tFlow = 0.0f, driftX = 1.0f, driftY = 0.0f;
static bool gustOn = false;

// ==========================================
//  3. LOGIQUE TECHNIQUE
// ==========================================

void check_for_updates() {
  if (WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, version_url);
  if (http.GET() == 200) {
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

static void buildPalette() {
  const int sr=255, sg=255, sb=255; 
  for (int i=0;i<256;i++){
    float k = i / 255.0f;
    palette[i] = dma_display->color565((int)(sr*(1.0f-0.95f*k)), (int)(sg*(1.0f-0.95f*k)), (int)(sb*(1.0f-0.95f*k)));
  }
}

// ... (fonctions updateBird, initBirds, flowVector, etc. restent identiques)
// [Gardez ici vos fonctions de physique de la version 2.4]

// ==========================================
//  4. GESTION DE L'HEURE (POCHOIR)
// ==========================================

void updateClockMask() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) return;

  char timeString[9];
  strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
  
  clock_mask->fillScreen(0); // On efface TOUT le masque (fond à 0)
  clock_mask->setTextColor(1); // On écrit en "1" (pixels du pochoir)
  clock_mask->setTextSize(1);
  clock_mask->setTextWrap(false);

  // Centrage : "HH:MM:SS" fait 47 pixels de large (8 chars * 6px - 1)
  // X = (64 - 47) / 2 = 8.5 -> On met 8
  // Y = Position verticale dans le quart supérieur
  clock_mask->setCursor(9, 4); 
  clock_mask->print(timeString);
}

static void renderToPanel(){ 
  for(int y=0; y<PANEL_RES_Y; y++){
    for(int x=0; x<PANEL_RES_X; x++){
      // On lit la luminance calculée par les oiseaux
      uint8_t luminance = lum[idx(x,y)];

      // LOGIQUE POCHOIR : 
      // Si le pixel dans clock_mask est à 1, on FORCE le blanc (palette 0)
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
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, 1);
  mxconfig.double_buff = true;
  mxconfig.gpio.r1 = 25; mxconfig.gpio.g1 = 26; mxconfig.gpio.b1 = 27;
  mxconfig.gpio.r2 = 14; mxconfig.gpio.g2 = 12; mxconfig.gpio.b2 = 13;
  mxconfig.gpio.a  = 23; mxconfig.gpio.b  = 19; mxconfig.gpio.c  = 5; mxconfig.gpio.d = 17;
  mxconfig.gpio.lat = 4; mxconfig.gpio.oe = 15; mxconfig.gpio.clk = 16;
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(150);

  // Allocation du masque invisible (canvas)
  clock_mask = new GFXcanvas1(PANEL_RES_X, PANEL_RES_Y);

  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) { delay(500); retry++; }

  if (WiFi.status() == WL_CONNECTED) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    check_for_updates();
  }

  memset(lum, 0, sizeof(lum));
  initBirds(); buildPalette();
  lastFrame = millis();
}

void loop() {
  uint32_t now = millis();
  uint32_t dt = now - lastFrame;
  lastFrame = now;

  static uint32_t lastClockUpdate = 0;
  if (now - lastClockUpdate > 1000) {
    updateClockMask();
    lastClockUpdate = now;
  }

  // Mettre ici vos appels updateGust(), updateDrift() et le bloc updateBird
  // [...]

  // Rendu final
  renderToPanel();
  dma_display->flipDMABuffer();
  
  delay(10);
}
