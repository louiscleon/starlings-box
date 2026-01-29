#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <vector>
#include <cmath>
#include <WiFiClientSecure.h>

// ==========================================
//  PARAMÈTRES DE MISE À JOUR (OTA)
// ==========================================
const char* VERSION  = "1.3"; // <--- Incrémente ce chiffre (1.1, 1.2...) pour déclencher la mise à jour
const char* ssid     = "Bbox-B0BFCA8A";
const char* password = "MR=gqxTd9tDSW+Mm";

// Remplace par tes vraies infos GitHub
const char* version_url = "https://raw.githubusercontent.com/louiscleon/starlings-box/blob/main/version.txt";
const char* bin_url     = "https://raw.githubusercontent.com/louiscleon/starlings-box/blob/main/starlings-box.ino.bin";

// ==========================================
//  2. PARAMÈTRES MURMURATION
// ==========================================
#define PANEL_RES_X 64
#define PANEL_RES_Y 32

MatrixPanel_I2S_DMA *dma_display = nullptr;

// =======================
//  MURMURATION SETTINGS
// =======================
static const int   NUM_BIRDS        = 5;

static const float MAX_SPEED        = 0.95f;   // ralentit ≈ x0.55
static const float MIN_SPEED        = 0.30f;

static const float NEIGHBOR_RAD     = 8.5f;
static const float SEP_DIST         = 2.6f;

static const float ALIGN_BASE       = 0.12f;
static const float COHESION_BASE    = 0.015f;
static const float SEPARATION_BASE  = 0.22f;

static const float BORDER_PUSH      = 0.18f;

// Flow field (structure organique)
static const float FLOW_STRENGTH    = 0.028f;
static const float FLOW_SCALE       = 0.085f;
static const float FLOW_SPEED       = 0.00028f;

// Rafales (bascule courte)
static const uint32_t GUST_MIN_MS   = 3800;
static const uint32_t GUST_MAX_MS   = 10500;
static const uint32_t GUST_LEN_MS   = 520;
static const float    GUST_TURN     = 0.12f;   // réduit les ronds
static const float    GUST_ALIGN_BOOST = 0.16f;

// Drift global (cap doux) : empêche les orbites
static float driftX = 1.0f, driftY = 0.0f;
static uint32_t nextDriftAt = 0;
static const float DRIFT_STRENGTH = 0.020f;    // 0.012..0.03

// Buffer de "masse"
static const uint8_t DECAY_NUM = 220; // /256
static const uint8_t DEPOSIT_HEAD = 230;
static const uint8_t DEPOSIT_TAIL = 120;
static const int TRAIL_LEN = 6;

// =======================
//  DATA STRUCTURES
// =======================
struct Bird {
  float x, y;
  float vx, vy;
  float turnSignal;          // propage les bascules (onde)
  float tx[TRAIL_LEN];
  float ty[TRAIL_LEN];
  uint8_t tIdx;
};

static std::vector<Bird> birds;
static uint8_t  lum[PANEL_RES_X * PANEL_RES_Y];  // 0..255
static uint16_t palette[256];

// Timing
static uint32_t lastFrame = 0;
static float tFlow = 0.0f;

// Gust state
static uint32_t nextGustAt = 0;
static uint32_t gustStart  = 0;
static bool gustOn = false;

// =======================
//  UTILS
// =======================
static inline float clampf(float v, float a, float b){ return (v<a)?a:(v>b)?b:v; }
static inline float fastInvSqrt(float x){ return 1.0f / sqrtf(x); }
static inline int idx(int x, int y){ return y * PANEL_RES_X + x; }
static inline bool inBounds(int x, int y){ return (x>=0 && x<PANEL_RES_X && y>=0 && y<PANEL_RES_Y); }

static inline uint32_t urand(uint32_t a, uint32_t b){
  if (b<=a) return a;
  return a + (uint32_t)(esp_random() % (b - a + 1));
}

// --- cheap 2D value noise (stable) ---
static inline uint32_t hash2i(int x, int y) {
  uint32_t h = 2166136261u;
  h = (h ^ (uint32_t)x) * 16777619u;
  h = (h ^ (uint32_t)y) * 16777619u;
  h ^= (h >> 13); h *= 1274126177u; h ^= (h >> 16);
  return h;
}
static inline float rand01_from_hash(uint32_t h){
  return (h & 0x00FFFFFF) / 16777215.0f;
}
static inline float lerpf(float a, float b, float t){ return a + (b - a) * t; }
static inline float smooth(float t){ return t * t * (3.0f - 2.0f * t); }

static float noise2D(float x, float y){
  int x0 = (int)floorf(x), y0 = (int)floorf(y);
  int x1 = x0 + 1, y1 = y0 + 1;
  float tx = smooth(x - x0);
  float ty = smooth(y - y0);

  float v00 = rand01_from_hash(hash2i(x0, y0));
  float v10 = rand01_from_hash(hash2i(x1, y0));
  float v01 = rand01_from_hash(hash2i(x0, y1));
  float v11 = rand01_from_hash(hash2i(x1, y1));

  float a = lerpf(v00, v10, tx);
  float b = lerpf(v01, v11, tx);
  return lerpf(a, b, ty);
}

// Flow field -> unit vector
static inline void flowVector(float x, float y, float &fx, float &fy){
  float n = noise2D(x * FLOW_SCALE + tFlow, y * FLOW_SCALE - tFlow);
  float ang = (n * 6.2831853f);
  fx = cosf(ang);
  fy = sinf(ang);
}

// =======================
//  PALETTE (sky -> shadow)
// =======================
static void buildPalette() {
  const int sr=185, sg=205, sb=225;  // fond
  for (int i=0;i<256;i++){
    float k = i / 255.0f;
    int r = (int)(sr * (1.0f - 0.85f*k));
    int g = (int)(sg * (1.0f - 0.90f*k));
    int b = (int)(sb * (1.0f - 0.95f*k));
    palette[i] = dma_display->color565(r,g,b);
  }
}

// =======================
//  GUST / DRIFT
// =======================
static void scheduleNextGust(uint32_t now){
  nextGustAt = now + urand(GUST_MIN_MS, GUST_MAX_MS);
}
static void updateGust(uint32_t now){
  if (!gustOn && now >= nextGustAt){
    gustOn = true;
    gustStart = now;
  }
  if (gustOn && (now - gustStart) > GUST_LEN_MS){
    gustOn = false;
    scheduleNextGust(now);
  }
}

static void updateDrift(uint32_t now){
  if (now >= nextDriftAt){
    nextDriftAt = now + urand(5000, 14000); // 5-14s
    float a = (esp_random()%628) / 100.0f;
    driftX = cosf(a);
    driftY = sinf(a);
  }
}

// =======================
//  INIT BIRDS
// =======================
static void initBirds() {
  birds.reserve(NUM_BIRDS);
  float cx = PANEL_RES_X * 0.5f;
  float cy = PANEL_RES_Y * 0.5f;

  for(int i=0;i<NUM_BIRDS;i++){
    Bird b;
    b.x = cx + (int)(esp_random()%28) - 14;
    b.y = cy + (int)(esp_random()%16) - 8;

    float a  = (esp_random()%628) / 100.0f;
    float sp = 0.55f + (esp_random()%55)/100.0f; // 0.55..1.10
    b.vx = cosf(a) * sp;
    b.vy = sinf(a) * sp;

    b.turnSignal = 0.0f;
    b.tIdx = 0;
    for(int t=0;t<TRAIL_LEN;t++){ b.tx[t]=b.x; b.ty[t]=b.y; }
    birds.push_back(b);
  }
}

// =======================
//  PHYSICS HELPERS
// =======================
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
  float s = 1.0f / inv;

  if (s > MAX_SPEED){
    float k = MAX_SPEED * inv;
    b.vx *= k; b.vy *= k;
  } else if (s < MIN_SPEED){
    float k = MIN_SPEED * inv;
    b.vx *= k; b.vy *= k;
  }
}

// =======================
//  UPDATE ONE BIRD
// =======================
static void updateBird(int i){
  Bird &b = birds[i];

  float ax=0, ay=0;      // align
  float cx=0, cy=0;      // cohesion
  float sx=0, sy=0;      // separation
  float tx=0, ty=0;      // propagated turn wave
  int n=0;

  for(int j=0;j<NUM_BIRDS;j++){
    if (j==i) continue;
    Bird &o = birds[j];

    float dx = o.x - b.x;
    float dy = o.y - b.y;
    float d2 = dx*dx + dy*dy;

    if (d2 < (NEIGHBOR_RAD*NEIGHBOR_RAD) && d2 > 1e-4f){
      float invd = fastInvSqrt(d2);
      float d = 1.0f / invd;

      ax += o.vx; ay += o.vy;
      cx += o.x;  cy += o.y;

      // Separation non linéaire
      if (d < SEP_DIST){
        float w = (SEP_DIST - d) / SEP_DIST; // 0..1
        w = w*w;
        sx -= dx * invd * w;
        sy -= dy * invd * w;
      }

      // Propagation du signal de virage (onde)
      tx += o.turnSignal * (-dy) * invd;
      ty += o.turnSignal * ( dx) * invd;

      n++;
    }
  }

  // Flow vector
  float fx, fy;
  flowVector(b.x, b.y, fx, fy);

  // Gust parameters
  float alignF = ALIGN_BASE + (gustOn ? GUST_ALIGN_BOOST : 0.0f);
  float turnBias = gustOn ? GUST_TURN : 0.0f;

  if (n>0){
    float invn = 1.0f / n;
    ax *= invn; ay *= invn;
    cx *= invn; cy *= invn;

    // Align
    b.vx += (ax - b.vx) * alignF;
    b.vy += (ay - b.vy) * alignF;

    // Cohesion
    b.vx += (cx - b.x) * COHESION_BASE;
    b.vy += (cy - b.y) * COHESION_BASE;

    // Separation
    b.vx += sx * SEPARATION_BASE;
    b.vy += sy * SEPARATION_BASE;

    // Turn wave (réduit pour éviter les ronds)
    b.vx += tx * 0.010f;
    b.vy += ty * 0.010f;
  }

  // Flow field influence
  b.vx += fx * FLOW_STRENGTH;
  b.vy += fy * FLOW_STRENGTH;

  // Global drift: cap doux
  b.vx += driftX * DRIFT_STRENGTH;
  b.vy += driftY * DRIFT_STRENGTH;

  // Gust: rotation globale légère
  if (gustOn){
    float rvx = -b.vy;
    float rvy =  b.vx;
    b.vx += rvx * turnBias;
    b.vy += rvy * turnBias;
  }

  // Borders & speed
  applyBorders(b);
  limitSpeed(b);

  // Turn signal (onde)
  float steer = fabsf(fx * (-b.vy) + fy * (b.vx)) * 0.35f;
  float targetSignal = clampf(steer + (gustOn ? 0.18f : 0.0f), 0.0f, 1.0f);
  b.turnSignal = b.turnSignal * 0.88f + targetSignal * 0.12f;

  // Trail record
  b.tx[b.tIdx] = b.x;
  b.ty[b.tIdx] = b.y;
  b.tIdx = (b.tIdx + 1) % TRAIL_LEN;

  // Move
  b.x += b.vx;
  b.y += b.vy;

  // Clamp
  if (b.x < 0) b.x = 0;
  if (b.x > PANEL_RES_X-1) b.x = PANEL_RES_X-1;
  if (b.y < 0) b.y = 0;
  if (b.y > PANEL_RES_Y-1) b.y = PANEL_RES_Y-1;
}

// =======================
//  MASS BUFFER RENDER
// =======================
static void decayLuminance(){
  for(int i=0;i<PANEL_RES_X*PANEL_RES_Y;i++){
    lum[i] = (uint8_t)((lum[i] * DECAY_NUM) >> 8);
  }
}
static void deposit(int x, int y, uint8_t amount){
  if(!inBounds(x,y)) return;
  int k = idx(x,y);
  int v = lum[k] + amount;
  lum[k] = (v>255) ? 255 : (uint8_t)v;
}
static void renderToPanel(){
  for(int y=0;y<PANEL_RES_Y;y++){
    for(int x=0;x<PANEL_RES_X;x++){
      dma_display->drawPixel(x, y, palette[lum[idx(x,y)]]);
    }
  }
}

// =======================
//  SETUP / LOOP
// =======================
void setup() {
  HUB75_I2S_CFG mxconfig(PANEL_RES_X, PANEL_RES_Y, 1);
  mxconfig.double_buff = true;

  // GPIO mapping (ton câblage actuel)
  mxconfig.gpio.r1 = 25; mxconfig.gpio.g1 = 26; mxconfig.gpio.b1 = 27;
  mxconfig.gpio.r2 = 14; mxconfig.gpio.g2 = 12; mxconfig.gpio.b2 = 13;
  mxconfig.gpio.a  = 23; mxconfig.gpio.b  = 19; mxconfig.gpio.c  = 5;  mxconfig.gpio.d = 17;
  mxconfig.gpio.lat = 4; mxconfig.gpio.oe = 15; mxconfig.gpio.clk = 16;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(150);

  memset(lum, 0, sizeof(lum));
  initBirds();
  buildPalette();

  uint32_t now = millis();
  scheduleNextGust(now);
  nextDriftAt = now + urand(2500, 8000); // premier cap
  lastFrame = now;
}

void loop() {
  uint32_t now = millis();
  uint32_t dt = now - lastFrame;
  lastFrame = now;

  // Flow time
  tFlow += dt * FLOW_SPEED;

  // Updates
  updateGust(now);
  updateDrift(now);

  // Physics
  for(int i=0;i<NUM_BIRDS;i++){
    updateBird(i);
  }

  // Render: decay then deposit mass
  decayLuminance();

  for(int i=0;i<NUM_BIRDS;i++){
    Bird &b = birds[i];

    int hx = (int)lroundf(b.x);
    int hy = (int)lroundf(b.y);
    deposit(hx, hy, DEPOSIT_HEAD);

    for(int t=1;t<TRAIL_LEN;t++){
      int age = (b.tIdx - t + TRAIL_LEN) % TRAIL_LEN;
      int txi = (int)lroundf(b.tx[age]);
      int tyi = (int)lroundf(b.ty[age]);
      uint8_t a = (uint8_t)(DEPOSIT_TAIL * (1.0f - (t / (float)TRAIL_LEN)));
      deposit(txi, tyi, a);
    }
  }

  renderToPanel();
  dma_display->flipDMABuffer();

  delay(10);
}
