#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <vector>
#include <cmath>

// =======================
//  CONFIGURATIONS GLOBALES
// =======================
const char* VERSION  = "1.0"; // <--- CHANGE MOI POUR DÉCLENCHER L'UPDATE
const char* ssid     = "Bbox-B0BFCA8A";
const char* password = "MR=gqxTd9tDSW+Mm";

// Remplace par tes vraies infos GitHub
const char* version_url = "https://raw.githubusercontent.com/TON_PSEUDO/TON_REPO/main/version.txt";
const char* bin_url     = "https://raw.githubusercontent.com/TON_PSEUDO/TON_REPO/build-bin/mon_projet.ino.bin";

#define PANEL_RES_X 64
#define PANEL_RES_Y 32
MatrixPanel_I2S_DMA *dma_display = nullptr;

// =======================
//  PARAMÈTRES MURMURATION (Ton code)
// =======================
static const int   NUM_BIRDS        = 85;
static const float MAX_SPEED        = 0.6f;
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

static std::vector<Bird> birds;
static uint8_t  lum[PANEL_RES_X * PANEL_RES_Y];
static uint16_t palette[256];
static uint32_t lastFrame = 0, nextGustAt = 0, gustStart = 0, nextDriftAt = 0;
static float tFlow = 0.0f, driftX = 1.0f, driftY = 0.0f;
static bool gustOn = false;

// =======================
//  FONCTIONS SYSTÈME (WiFi / Update)
// =======================

void check_for_updates() {
  if(WiFi.status() != WL_CONNECTED) return;
  
  HTTPClient http;
  http.begin(version_url);
  int httpCode = http.GET();

  if (httpCode == 200) {
    String newVersion = http.getString();
    newVersion.trim();
    if (newVersion != VERSION) {
      Serial.println("MAJ détectée !");
      httpUpdate.update(bin_url);
    }
  }
  http.end();
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }
}

// =======================
//  OUTILS MATHS & PHYSIQUE (Ton code)
// =======================
static inline float clampf(float v, float a, float b){ return (v<a)?a:(v>b)?b:v; }
static inline float fastInvSqrt(float x){ return 1.0f / sqrtf(x); }
static inline int idx(int x, int y){ return y * PANEL_RES_X + x; }
static inline bool inBounds(int x, int y){ return (x>=0 && x<PANEL_RES_X && y>=0 && y<PANEL_RES_Y); }
static inline uint32_t hash2i(int x, int y) {
  uint32_t h = 2166136261u;
  h = (h ^ (uint32_t)x) * 16777619u;
  h = (h ^ (uint32_t)y) * 16777619u;
  h ^= (h >> 13); h *= 1274126177u; h ^= (h >> 16);
  return h;
}
static inline float rand01_from_hash(uint32_t h){ return (h & 0x00FFFFFF) / 16777215.0f; }
static float noise2D(float x, float y){
  int x0 = (int)floorf(x), y0 = (int)floorf(y);
  float tx = (x - x0)*(x - x0)*(3.0f - 2.0f*(x - x0));
  float ty = (y - y0)*(y - y0)*(3.0f - 2.0f*(y - y0));
  float v00 = rand01_from_hash(hash2i(x0, y0)), v10 = rand01_from_hash(hash2i(x0+1, y0));
  float v01 = rand01_from_hash(hash2i(x0, y0+1)), v11 = rand01_from_hash(hash2i(x0+1, y0+1));
  return (v00+(v10-v00)*tx) + ((v01+(v11-v01)*tx) - (v00+(v10-v00)*tx)) * ty;
}

// ... [Insérer ici toutes tes fonctions de physique : buildPalette, updateBird, decayLuminance, etc.] ...
// (Note : Garde tes fonctions exactement comme dans ton message précédent)

void setup() {
  // 1. Matériel
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, 1);
  mxconfig.double_buff = true;
  mxconfig.gpio.r1 = 25; mxconfig.gpio.g1 = 26; mxconfig.gpio.b1 = 27;
  mxconfig.gpio.r2 = 14; mxconfig.gpio.g2 = 12; mxconfig.gpio.b2 = 13;
  mxconfig.gpio.a  = 23; mxconfig.gpio.b  = 19; mxconfig.gpio.c  = 5;  mxconfig.gpio.d = 17;
  mxconfig.gpio.lat = 4; mxconfig.gpio.oe = 15; mxconfig.gpio.clk = 16;
  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  
  // 2. WiFi & Update
  connectWiFi();
  check_for_updates(); 

  // 3. Init Murmuration
  memset(lum, 0, sizeof(lum));
  buildPalette(); // Appelle ta fonction buildPalette
  initBirds();    // Appelle ta fonction initBirds
  lastFrame = millis();
}

void loop() {
  uint32_t now = millis();
  // ... [Insérer ici toute la logique de ta boucle loop : updateBird, deposit, renderToPanel] ...

  // Vérification de mise à jour toutes les 30 minutes
  static uint32_t lastUpdateCheck = 0;
  if (now - lastUpdateCheck > 1800000) {
    check_for_updates();
    lastUpdateCheck = now;
  }
}
