/*
  SpectraNode - ESP32 WiFi Channel Monitor & Interference Detector
  BET-S559 mini project

  WIRING
    OLED SSD1306 (I2C): VCC->3.3V  GND->GND  SDA->GPIO21  SCL->GPIO22
    Button 1 Refresh  : GPIO15 -> GND
    Button 2 Next     : GPIO16 -> GND
    Button 3 Back     : GPIO17 -> GND
    Power             : 18650 -> TP4056 -> ESP32 VIN / GND

  LIBRARIES (Arduino IDE -> Library Manager)
    - Adafruit SSD1306
    - Adafruit GFX Library
    - Adafruit BusIO (installed automatically with the two above)
    WiFi.h and Wire.h come with the ESP32 board package (nothing to install).

  BOARD
    Boards Manager -> "esp32" by Espressif Systems -> select "ESP32 Dev Module"
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ================= PINS (match the wiring table) =================
#define OLED_SDA     21
#define OLED_SCL     22
#define BTN_REFRESH  15
#define BTN_NEXT     16
#define BTN_BACK     17

// ================= DISPLAY =================
#define SCREEN_W   128
#define SCREEN_H   64
#define OLED_ADDR  0x3C          // if the screen stays blank, try 0x3D
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// Symbols from the OLED's built-in font (it has no star / warning sign)
const uint8_t SYM_DIAMOND = 0x04;   // used instead of the diamond mark
const uint8_t SYM_UP      = 0x18;   // arrow up
const uint8_t SYM_DOWN    = 0x19;   // arrow down
const uint8_t SYM_RIGHT   = 0x1A;   // arrow right

// ================= SETTINGS (tweak these while testing) =================
const uint32_t      SCAN_MS_PER_CHAN = 300;     // passive listen time per channel
const unsigned long AUTO_SCAN_MS     = 30000;   // auto re-scan every 30 s (0 = off)
const unsigned long DEBOUNCE_MS      = 40;
const int   NOISE_FLOOR     = -100;             // dBm; anything above this is a real signal
const float BLEED[5]        = {1.0, 0.7, 0.5, 0.3, 0.1}; // how much a channel leaks into one N channels away
const int   SCORE_FULL      = 200;              // score that fills a bar completely
const int   CONGESTED_SCORE = 60;               // score at/above this = congested channel
const int   TREND_DEADBAND  = 10;               // ignore score changes smaller than this

#define NUM_CH      13
#define NUM_SCREENS 5
#define HIST        5
#define MAX_OPEN    8

// ================= DATA =================
int netCount[NUM_CH + 1];            // networks per channel (index 1..13)
int strongest[NUM_CH + 1];           // strongest RSSI seen on each channel
int ownLoad[NUM_CH + 1];             // sum of signal strengths on the channel itself
int score[NUM_CH + 1];               // ownLoad + bleed from neighbours (lower = cleaner)
int order[NUM_CH];                   // channels sorted cleanest -> busiest
int history[HIST][NUM_CH + 1];       // last 5 scans, [0] = newest
int histCount = 0;

int totalNets = 0, congestedCh = 0, cleanCh = 0;
const char *noiseLabel = "LOW";
int bestNow = 1, bestLong = 1;

struct OpenNet { char ssid[33]; int ch; };
OpenNet openNets[MAX_OPEN];
int openStored = 0;                  // how many we kept the name of
int openTotal  = 0;                  // how many open networks exist

int currentScreen = 0;
unsigned long lastScanMs = 0;

// ================= BUTTONS =================
struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChange;
};
Button btnRefresh = {BTN_REFRESH, HIGH, HIGH, 0};
Button btnNext    = {BTN_NEXT,    HIGH, HIGH, 0};
Button btnBack    = {BTN_BACK,    HIGH, HIGH, 0};

// true once per press (button wired to GND, so LOW = pressed)
bool wasPressed(Button &b) {
  bool reading = digitalRead(b.pin);
  unsigned long now = millis();
  if (reading != b.lastReading) {
    b.lastReading = reading;
    b.lastChange = now;
  }
  if ((now - b.lastChange) > DEBOUNCE_MS && reading != b.stableState) {
    b.stableState = reading;
    if (reading == LOW) return true;
  }
  return false;
}

// ================= HELPERS =================
bool isNonOverlap(int ch) { return ch == 1 || ch == 6 || ch == 11; }

int clampInt(long v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int)v;
}

// bar length for a score; at least 1 pixel if the score is above zero
int barLen(int value, int maxLen) {
  if (value <= 0) return 0;
  return clampInt((long)value * maxLen / SCORE_FULL, 1, maxLen);
}

// ================= SCORING =================
void computeScores() {
  for (int ch = 1; ch <= NUM_CH; ch++) {
    float s = ownLoad[ch];
    for (int d = 1; d <= 4; d++) {
      if (ch - d >= 1)      s += BLEED[d] * ownLoad[ch - d];
      if (ch + d <= NUM_CH) s += BLEED[d] * ownLoad[ch + d];
    }
    score[ch] = (int)(s + 0.5f);
  }

  // insertion sort: cleanest first (ties stay in channel order)
  for (int i = 0; i < NUM_CH; i++) order[i] = i + 1;
  for (int i = 1; i < NUM_CH; i++) {
    int key = order[i];
    int j = i - 1;
    while (j >= 0 && score[order[j]] > score[key]) {
      order[j + 1] = order[j];
      j--;
    }
    order[j + 1] = key;
  }

  // Best Right Now = lowest score of all 13 (on a tie, prefer 1/6/11)
  bestNow = order[0];
  for (int i = 0; i < NUM_CH && score[order[i]] == score[order[0]]; i++) {
    if (isNonOverlap(order[i])) { bestNow = order[i]; break; }
  }

  // Best Long Term = lowest score among 1, 6, 11 only
  const int nonOv[3] = {1, 6, 11};
  bestLong = nonOv[0];
  for (int k = 1; k < 3; k++)
    if (score[nonOv[k]] < score[bestLong]) bestLong = nonOv[k];
  if (isNonOverlap(bestNow)) bestLong = bestNow;
}

void pushHistory() {
  for (int i = HIST - 1; i > 0; i--)
    memcpy(history[i], history[i - 1], sizeof(history[i]));
  memcpy(history[0], score, sizeof(history[0]));
  if (histCount < HIST) histCount++;
}

void computeStats() {
  congestedCh = 0;
  long totalScore = 0;
  for (int ch = 1; ch <= NUM_CH; ch++) {
    if (score[ch] >= CONGESTED_SCORE) congestedCh++;
    totalScore += score[ch];
  }
  cleanCh = NUM_CH - congestedCh;
  long avg = totalScore / NUM_CH;
  if (avg < 25)      noiseLabel = "LOW";
  else if (avg < 70) noiseLabel = "MED";
  else               noiseLabel = "HIGH";
}

// average score of the older scans (what "before" looked like)
int prevAverage(int ch) {
  if (histCount < 2) return history[0][ch];
  long sum = 0;
  for (int i = 1; i < histCount; i++) sum += history[i][ch];
  return (int)(sum / (histCount - 1));
}

// +1 worse, -1 better, 0 same
int trendOf(int ch) {
  int diff = history[0][ch] - prevAverage(ch);
  if (diff > TREND_DEADBAND)  return 1;
  if (diff < -TREND_DEADBAND) return -1;
  return 0;
}

// ================= SCAN =================
void processScan(int n) {
  for (int ch = 0; ch <= NUM_CH; ch++) {
    netCount[ch] = 0;
    strongest[ch] = -127;
    ownLoad[ch] = 0;
  }
  totalNets = 0;
  openTotal = 0;
  openStored = 0;

  for (int i = 0; i < n; i++) {
    int ch = WiFi.channel(i);
    if (ch < 1 || ch > NUM_CH) continue;
    int rssi = WiFi.RSSI(i);

    netCount[ch]++;
    totalNets++;
    if (rssi > strongest[ch]) strongest[ch] = rssi;

    // dBm is negative, so turn it into a positive "weight":
    // -50 dBm -> 50 (loud), -90 dBm -> 10 (faint)
    int w = rssi - NOISE_FLOOR;
    if (w < 0) w = 0;
    ownLoad[ch] += w;

    if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) {
      openTotal++;
      if (openStored < MAX_OPEN) {
        String s = WiFi.SSID(i);
        char *dst = openNets[openStored].ssid;
        if (s.length() == 0) {
          snprintf(dst, sizeof(openNets[0].ssid), "<hidden>");
        } else {
          snprintf(dst, sizeof(openNets[0].ssid), "%s", s.c_str());
          for (char *p = dst; *p; p++)
            if (*p < 32 || *p > 126) *p = '?';
        }
        openNets[openStored].ch = ch;
        openStored++;
      }
    }
  }

  computeScores();
  pushHistory();
  computeStats();
}

void printSerialReport() {
  Serial.println();
  Serial.println("--- SpectraNode scan ---");
  for (int ch = 1; ch <= NUM_CH; ch++) {
    Serial.printf("Ch%-2d nets:%d strongest:%d dBm score:%d%s\n",
                  ch, netCount[ch], strongest[ch], score[ch],
                  isNonOverlap(ch) ? "  (non-overlap)" : "");
  }
  Serial.printf("Total:%d  Open:%d  Best now: Ch%d  Best long-term: Ch%d\n",
                totalNets, openTotal, bestNow, bestLong);
}

// ================= DRAWING =================
void showMessage(const char *msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor((SCREEN_W - 6 * (int)strlen(msg)) / 2, 28);
  display.print(msg);
  display.display();
}

void drawHeader(const char *title, int page) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(title);
  char pg[8];
  snprintf(pg, sizeof(pg), "[%d/%d]", page, NUM_SCREENS);
  display.setCursor(SCREEN_W - 6 * (int)strlen(pg), 0);
  display.print(pg);
  display.drawFastHLine(0, 9, SCREEN_W, SSD1306_WHITE);
}

// ---- Screen 1: bar graph ----
void drawScreen1() {
  drawHeader("WiFi Spectrum", 1);
  const int slot = 9, x0 = 5, baseY = 45, maxH = 33;

  for (int ch = 1; ch <= NUM_CH; ch++) {
    int slotX = x0 + (ch - 1) * slot;

    int h = barLen(score[ch], maxH);
    if (h > 0) display.fillRect(slotX + 1, baseY - h, 7, h, SSD1306_WHITE);

    // channel number under odd channels only (two digits would collide)
    if (ch % 2 == 1) {
      char lb[4];
      snprintf(lb, sizeof(lb), "%d", ch);
      int w = (int)strlen(lb) * 6;
      display.setCursor(slotX + (slot - w) / 2, 47);
      display.print(lb);
    }

    // network count under every channel
    char cb[3];
    if (netCount[ch] > 9) snprintf(cb, sizeof(cb), "+");
    else                  snprintf(cb, sizeof(cb), "%d", netCount[ch]);
    display.setCursor(slotX + (slot - 6) / 2, 56);
    display.print(cb);
  }
  display.drawFastHLine(0, baseY, SCREEN_W, SSD1306_WHITE);
}

// ---- Screen 2: rankings + dual recommendation ----
void drawScreen2() {
  drawHeader("Ch Rankings", 2);

  for (int r = 0; r < 5; r++) {
    int ch = order[r];
    int y = 11 + r * 8;

    char rs[8];
    if (netCount[ch] > 0) snprintf(rs, sizeof(rs), "%4d", strongest[ch]);
    else                  snprintf(rs, sizeof(rs), "%4s", "---");

    char line[28];
    snprintf(line, sizeof(line), "Ch%2d %sdBm %dnet ", ch, rs, netCount[ch]);
    display.setCursor(0, y);
    display.print(line);
    if (ch == bestNow) display.write('*');       // * = best right now
    else               display.write(' ');
    if (isNonOverlap(ch)) display.write(SYM_DIAMOND);  // diamond = non-overlapping
  }

  display.drawFastHLine(0, 53, SCREEN_W, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.write('*');
  display.print("Now:");
  display.print(bestNow);
  if (!isNonOverlap(bestNow)) display.print("(ovl!)");
  display.print(' ');
  display.write(SYM_DIAMOND);
  display.print("Use:");
  display.print(bestLong);
}

// ---- Screen 3: congestion trend ----
void drawBar(int x, int y, int maxW, int value) {
  int w = barLen(value, maxW);
  if (w > 0) display.fillRect(x, y + 1, w, 6, SSD1306_WHITE);
}

void drawScreen3() {
  drawHeader("Trend Monitor", 3);

  int shown = 0;
  // busiest channels first (order[] is cleanest -> busiest)
  for (int i = NUM_CH - 1; i >= 0 && shown < 4; i--) {
    int ch = order[i];
    if (score[ch] == 0) break;               // everything left is empty
    int y = 13 + shown * 10;

    char lb[8];
    snprintf(lb, sizeof(lb), "Ch%2d", ch);
    display.setCursor(0, y);
    display.print(lb);

    drawBar(32, y, 26, prevAverage(ch));     // before
    display.setCursor(61, y);
    display.write(SYM_RIGHT);
    drawBar(72, y, 36, score[ch]);           // now

    int t = trendOf(ch);
    display.setCursor(114, y);
    display.write(t > 0 ? SYM_UP : (t < 0 ? SYM_DOWN : SYM_RIGHT));
    shown++;
  }
  if (shown == 0) {
    display.setCursor(0, 28);
    display.print("No activity seen");
  }

  display.setCursor(0, 56);
  display.write(SYM_UP);   display.print("worse ");
  display.write(SYM_DOWN); display.print("better ");
  display.write(SYM_RIGHT); display.print("same");
}

// ---- Screen 4: environment summary ----
void drawScreen4() {
  drawHeader("Environment", 4);
  char b[28];
  int y = 12;

  snprintf(b, sizeof(b), "%-15s:%4d", "Total Networks", totalNets);
  display.setCursor(0, y); display.print(b); y += 8;

  snprintf(b, sizeof(b), "%-15s:%4d", "Congested Ch", congestedCh);
  display.setCursor(0, y); display.print(b); y += 8;

  snprintf(b, sizeof(b), "%-15s:%4d", "Clean Ch", cleanCh);
  display.setCursor(0, y); display.print(b); y += 8;

  snprintf(b, sizeof(b), "%-15s:%4d", "Open Networks", openTotal);
  display.setCursor(0, y); display.print(b);
  if (openTotal > 0) display.write('!');
  y += 8;

  snprintf(b, sizeof(b), "%-15s: %s", "Noise Level", noiseLabel);
  display.setCursor(0, y); display.print(b); y += 8;

  snprintf(b, sizeof(b), "%-15s: Ch%d", "Best Channel", bestLong);
  display.setCursor(0, y); display.print(b);
}

// ---- Screen 5: open network alert ----
void drawScreen5() {
  drawHeader("! Security Alert", 5);

  if (openTotal == 0) {
    display.setCursor(0, 26);
    display.print("All networks secured");
    display.setCursor(0, 38);
    display.print("No open WiFi found");
    return;
  }

  display.setCursor(0, 12);
  display.print("Unsecured Networks:");

  const int maxRows = 4;
  int rows = (openTotal > maxRows) ? maxRows - 1 : openTotal;
  if (rows > openStored) rows = openStored;

  for (int i = 0; i < rows; i++) {
    char line[28];
    snprintf(line, sizeof(line), "> %-12.12s Ch%d", openNets[i].ssid, openNets[i].ch);
    display.setCursor(0, 22 + i * 8);
    display.print(line);
  }
  if (openTotal > rows) {
    display.setCursor(0, 22 + rows * 8);
    display.print("+");
    display.print(openTotal - rows);
    display.print(" more");
  }

  display.setCursor(0, 56);
  display.print("Avoid these networks!");
}

void render() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  switch (currentScreen) {
    case 0: drawScreen1(); break;
    case 1: drawScreen2(); break;
    case 2: drawScreen3(); break;
    case 3: drawScreen4(); break;
    case 4: drawScreen5(); break;
  }
  display.display();
}

// ================= SCAN + REFRESH =================
void runScan() {
  showMessage("Scanning...");
  // blocking, passive scan, include hidden networks
  int n = WiFi.scanNetworks(false, true, true, SCAN_MS_PER_CHAN);
  lastScanMs = millis();

  if (n < 0) {                    // scan failed: keep the old data
    Serial.println("Scan failed");
    showMessage("Scan failed");
    delay(1000);
    render();
    return;
  }
  processScan(n);
  WiFi.scanDelete();
  printSerialReport();
  render();
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);

  pinMode(BTN_REFRESH, INPUT_PULLUP);
  pinMode(BTN_NEXT,    INPUT_PULLUP);
  pinMode(BTN_BACK,    INPUT_PULLUP);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found - check SDA/SCL wiring and I2C address");
    while (true) delay(1000);
  }

  // splash
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(16, 8);
  display.print("Spectra");
  display.setCursor(40, 26);
  display.print("Node");
  display.setTextSize(1);
  display.setCursor(4, 50);
  display.print("WiFi Channel Monitor");
  display.display();
  delay(1500);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  runScan();
}

void loop() {
  if (wasPressed(btnRefresh)) runScan();

  if (wasPressed(btnNext)) {
    currentScreen = (currentScreen + 1) % NUM_SCREENS;
    render();
  }
  if (wasPressed(btnBack)) {
    currentScreen = (currentScreen + NUM_SCREENS - 1) % NUM_SCREENS;
    render();
  }

  if (AUTO_SCAN_MS > 0 && millis() - lastScanMs >= AUTO_SCAN_MS) runScan();
}