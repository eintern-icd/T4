/*
 * NORVI X-T4 Industrial Furnace Monitor - Channel 1
 * Industrial UI: 7-segment readout, LED bar gauge, live trend, alarm banner
 * Rendering is double-buffered (sprite) -> no flicker, no overlapping text.
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

#define SDA_PIN   8
#define SCL_PIN   9
#define MISO_PIN  13
#define MOSI_PIN  11
#define SCLK_PIN  12

#define ADS_ADDR 0x48
Adafruit_ADS1115 ads1;

// ---------- DISPLAY DRIVER (LovyanGFX) ----------
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI      _bus_instance;
public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host     = SPI2_HOST;
      cfg.spi_mode     = 0;
      cfg.freq_write   = 40000000;
      cfg.freq_read    = 16000000;
      cfg.spi_3wire    = false;
      cfg.use_lock     = true;
      cfg.dma_channel  = SPI_DMA_CH_AUTO;
      cfg.pin_sclk     = SCLK_PIN;
      cfg.pin_mosi     = MOSI_PIN;
      cfg.pin_miso     = MISO_PIN;
      cfg.pin_dc       = 46;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs            = 45;
      cfg.pin_rst           = 47;
      cfg.pin_busy          = -1;
      cfg.panel_width       = 240;
      cfg.panel_height      = 320;
      cfg.offset_x          = 0;
      cfg.offset_y          = 0;
      cfg.offset_rotation   = 0;
      cfg.dummy_read_pixel  = 8;
      cfg.dummy_read_bits   = 1;
      cfg.readable          = true;
      cfg.invert            = true;
      cfg.rgb_order         = false;
      cfg.dlen_16bit        = false;
      cfg.bus_shared        = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;
LGFX_Sprite canvas(&tft);   // full-screen off-screen buffer

// ---------- CONFIG & CONSTANTS ----------
const float SUPPLY_VOLTAGE      = 3.3;
const float SERIES_RESISTOR     = 10000.0;
const float THERMISTOR_NOMINAL  = 10000.0;
const float TEMPERATURE_NOMINAL = 25.0;
const float B_COEFFICIENT       = 3892.0;   // Littelfuse 103JG1K J-Curve

const float MAX_SAFE_TEMP       = 300.0;    // over-temperature alarm (C)
const float TEMP_HYSTERESIS     = 5.0;      // alarm clears below MAX - this
const float WARN_TEMP           = MAX_SAFE_TEMP * 0.85;  // amber warning zone
const float RATE_OF_RISE_LIMIT  = 15.0;     // C/min
const float GAUGE_MAX           = 400.0;    // bar gauge full scale

// Trend / rate-of-rise timing
const unsigned long TREND_INTERVAL = 3000;  // ms per trend sample
const int  TREND_POINTS = 100;              // 100 x 3 s = 5 min of history
const int  ROR_SAMPLES  = 20;               // 20 x 3 s = 60 s window
const unsigned long SENSE_INTERVAL   = 100; // ADC read period (ms)
const unsigned long DISPLAY_INTERVAL = 200; // screen refresh (ms)

float coldJunctionTemp = 25.0;

// ---------- COLOURS (RGB565) ----------
constexpr uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
const uint16_t C_BG       = RGB(8, 10, 14);
const uint16_t C_HEADER   = RGB(24, 34, 48);
const uint16_t C_PANEL    = RGB(16, 22, 30);
const uint16_t C_BORDER   = RGB(60, 74, 92);
const uint16_t C_TEXT     = RGB(232, 237, 242);
const uint16_t C_DIM      = RGB(130, 142, 158);
const uint16_t C_GREEN    = RGB(0, 225, 95);
const uint16_t C_GREEN_DK = RGB(0, 70, 34);
const uint16_t C_AMBER    = RGB(255, 176, 0);
const uint16_t C_AMBER_DK = RGB(80, 56, 0);
const uint16_t C_RED      = RGB(255, 50, 40);
const uint16_t C_RED_DK   = RGB(88, 14, 12);
const uint16_t C_CYAN     = RGB(0, 200, 255);
const uint16_t C_BANNER_OK= RGB(0, 110, 45);
const uint16_t C_BANNER_AL= RGB(190, 0, 0);

// ---------- STATE ----------
float tempF = NAN;             // filtered temperature
float rateOfRise = NAN;        // C/min (NAN = still warming up)
bool  overTempLatched = false;
int   badReadCount = 0;

float hist[TREND_POINTS];
int   histCount = 0;

unsigned long lastSense = 0, lastTrend = 0, lastDisplay = 0, lastSerial = 0;

// ---------- SENSOR ----------
float readThermistorC(int16_t adcRaw) {
  float voltage = ads1.computeVolts(adcRaw);
  if (voltage <= 0.001 || voltage >= SUPPLY_VOLTAGE - 0.001) return NAN;

  float ntcResistance = SERIES_RESISTOR * (SUPPLY_VOLTAGE / voltage - 1.0);
  float s = ntcResistance / THERMISTOR_NOMINAL;
  s = log(s);
  s /= B_COEFFICIENT;
  s += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  s = 1.0 / s;
  s -= 273.15;

  float cjcOffset = (coldJunctionTemp - 25.0) * 0.02;
  return s + cjcOffset;
}

void senseTemperature() {
  // Average 4 conversions to reduce noise
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += ads1.readADC_SingleEnded(0);
  float raw = readThermistorC((int16_t)(sum / 4));

  if (isnan(raw)) {
    if (++badReadCount >= 5) tempF = NAN;          // confirmed open/short
  } else {
    badReadCount = 0;
    if (isnan(tempF)) tempF = raw;
    else tempF += 0.15f * (raw - tempF);           // smooth display jitter
  }
}

void pushTrend() {
  if (histCount == TREND_POINTS) {
    memmove(hist, hist + 1, sizeof(float) * (TREND_POINTS - 1));
    histCount--;
  }
  hist[histCount++] = tempF;

  // Rate of rise over the last 60 s (first valid result after 60 s)
  if (histCount > ROR_SAMPLES) {
    float a = hist[histCount - 1];
    float b = hist[histCount - 1 - ROR_SAMPLES];
    if (isnan(a) || isnan(b)) rateOfRise = NAN;
    else rateOfRise = (a - b) * (60000.0f / (ROR_SAMPLES * TREND_INTERVAL));
  }
}

// ---------- UI HELPERS ----------
void hazardStripe(int y, int h, uint16_t a, uint16_t b) {
  canvas.fillRect(0, y, 240, h, b);
  canvas.setClipRect(0, y, 240, h);
  for (int x = -h; x < 240; x += 14) {
    canvas.fillTriangle(x, y + h, x + 7, y + h, x + 7 + h, y, a);
    canvas.fillTriangle(x, y + h, x + 7 + h, y, x + h, y, a);
  }
  canvas.clearClipRect();
}

void panel(int x, int y, int w, int h, uint16_t border) {
  canvas.fillRoundRect(x, y, w, h, 6, C_PANEL);
  canvas.drawRoundRect(x, y, w, h, 6, border);
}

void smallText(const char* s, int x, int y, uint16_t col, textdatum_t d) {
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(1);
  canvas.setTextDatum(d);
  canvas.setTextColor(col);
  canvas.drawString(s, x, y);
}

void drawGauge(int x, int y, int w, int h, float t) {
  int fillW = isnan(t) ? 0 : constrain((int)(w * t / GAUGE_MAX), 0, w);
  for (int i = 0; i < w; i++) {
    float v = GAUGE_MAX * i / w;
    uint16_t bright, dim;
    if (v < WARN_TEMP)           { bright = C_GREEN; dim = C_GREEN_DK; }
    else if (v < MAX_SAFE_TEMP)  { bright = C_AMBER; dim = C_AMBER_DK; }
    else                         { bright = C_RED;   dim = C_RED_DK;   }
    canvas.drawFastVLine(x + i, y, h, i < fillW ? bright : dim);
  }
  // LED-segment look
  for (int i = 7; i < w; i += 8) canvas.drawFastVLine(x + i, y, h, C_BG);

  // Scale ticks + labels
  char b[8];
  for (int v = 0; v <= (int)GAUGE_MAX; v += 100) {
    int px = x + (int)(w * v / GAUGE_MAX);
    if (px > x + w - 1) px = x + w - 1;
    canvas.drawFastVLine(px, y + h, 3, C_DIM);
    snprintf(b, sizeof(b), "%d", v);
    smallText(b, px, y + h + 9, C_DIM, textdatum_t::middle_center);
  }
  // Limit marker
  int lx = x + (int)(w * MAX_SAFE_TEMP / GAUGE_MAX);
  canvas.fillTriangle(lx - 4, y - 6, lx + 4, y - 6, lx, y - 1, C_RED);
}

void drawTrend(int x, int y, int w, int h) {
  panel(x, y, w, h, C_BORDER);
  smallText("TREND  5 MIN", x + 8, y + 9, C_DIM, textdatum_t::middle_left);

  float mn = 1e9, mx = -1e9;
  for (int i = 0; i < histCount; i++) {
    if (isnan(hist[i])) continue;
    if (hist[i] < mn) mn = hist[i];
    if (hist[i] > mx) mx = hist[i];
  }
  if (mn > mx) {
    smallText("NO DATA", x + w / 2, y + h / 2 + 6, C_DIM, textdatum_t::middle_center);
    return;
  }

  char b[24];
  snprintf(b, sizeof(b), "H %.1f  L %.1f", mx, mn);
  smallText(b, x + w - 8, y + 9, C_CYAN, textdatum_t::middle_right);

  float lo = mn, hi = mx;
  if (hi - lo < 10.0f) { float mid = (hi + lo) / 2; lo = mid - 5; hi = mid + 5; }
  else { float pad = (hi - lo) * 0.1f; lo -= pad; hi += pad; }

  int px = x + 8, py = y + 20, pw = w - 16, ph = h - 28;

  // dotted grid
  for (int g = 0; g <= 2; g++) {
    int gy = py + (ph - 1) * g / 2;
    for (int dx = 0; dx < pw; dx += 4) canvas.drawPixel(px + dx, gy, C_BORDER);
  }

  // trace (newest sample at right edge)
  for (int i = 1; i < histCount; i++) {
    if (isnan(hist[i]) || isnan(hist[i - 1])) continue;
    int p0 = TREND_POINTS - histCount + i - 1;
    int p1 = p0 + 1;
    int x0 = px + p0 * (pw - 1) / (TREND_POINTS - 1);
    int x1 = px + p1 * (pw - 1) / (TREND_POINTS - 1);
    int y0 = py + ph - 2 - (int)((hist[i - 1] - lo) / (hi - lo) * (ph - 3));
    int y1 = py + ph - 2 - (int)((hist[i]     - lo) / (hi - lo) * (ph - 3));
    canvas.drawLine(x0, y0, x1, y1, C_CYAN);
    canvas.drawLine(x0, y0 + 1, x1, y1 + 1, C_CYAN);
  }
}

// ---------- MAIN RENDER ----------
void render(unsigned long now) {
  bool blink = (now / 500) % 2;

  // ----- alarm logic -----
  bool sensorFault = isnan(tempF);
  if (!sensorFault) {
    if (tempF > MAX_SAFE_TEMP) overTempLatched = true;
    else if (tempF < MAX_SAFE_TEMP - TEMP_HYSTERESIS) overTempLatched = false;
  }
  bool rateAlarm = !isnan(rateOfRise) && rateOfRise > RATE_OF_RISE_LIMIT;
  bool alarm = sensorFault || overTempLatched || rateAlarm;
  bool warn  = !alarm && !sensorFault && tempF >= WARN_TEMP;

  uint16_t stateCol = alarm ? C_RED : (warn ? C_AMBER : C_GREEN);

  canvas.fillScreen(C_BG);
  char b[32];

  // ----- HEADER -----
  canvas.fillRect(0, 0, 240, 36, C_HEADER);
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextSize(1);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setTextColor(C_TEXT);
  canvas.drawString("FURNACE CONTROL", 10, 18);
  // status LED
  uint16_t led = alarm ? (blink ? C_RED : C_RED_DK) : (warn ? C_AMBER : C_GREEN);
  canvas.fillCircle(220, 18, 7, led);
  canvas.drawCircle(220, 18, 10, C_BORDER);
  hazardStripe(36, 5, C_AMBER, C_BG);

  // ----- CHANNEL ROW -----
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setTextColor(C_DIM);
  canvas.drawString("CH1 : OVEN ZONE", 10, 52);
  unsigned long s = now / 1000;
  snprintf(b, sizeof(b), "UP %02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  smallText(b, 230, 52, C_DIM, textdatum_t::middle_right);

  // ----- TEMPERATURE PANEL -----
  panel(8, 64, 224, 82, stateCol);
  if (alarm) canvas.drawRoundRect(9, 65, 222, 80, 5, stateCol);
  smallText("TEMPERATURE", 18, 75, C_DIM, textdatum_t::middle_left);
  snprintf(b, sizeof(b), "LIMIT %dC", (int)MAX_SAFE_TEMP);
  smallText(b, 222, 75, C_DIM, textdatum_t::middle_right);

  if (sensorFault) {
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextDatum(textdatum_t::middle_center);
    canvas.setTextColor(blink ? C_RED : C_RED_DK);
    canvas.drawString("SENSOR OPEN", 120, 108);
    smallText("CHECK THERMISTOR WIRING", 120, 132, C_DIM, textdatum_t::middle_center);
  } else {
    snprintf(b, sizeof(b), "%.1f", tempF);
    canvas.setFont(&fonts::Font7);          // 7-segment style digits
    canvas.setTextSize(1);
    canvas.setTextDatum(textdatum_t::middle_right);
    canvas.setTextColor(stateCol);
    canvas.drawString(b, 194, 114);
    // degree sign + unit
    canvas.drawCircle(203, 98, 3, stateCol);
    canvas.drawCircle(203, 98, 2, stateCol);
    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextDatum(textdatum_t::middle_left);
    canvas.drawString("C", 209, 110);
  }

  // ----- BAR GAUGE -----
  drawGauge(8, 156, 224, 10, tempF);

  // ----- TREND -----
  drawTrend(8, 182, 224, 70);

  // ----- RATE OF RISE -----
  uint16_t rc = C_TEXT;
  if (!isnan(rateOfRise)) {
    if (rateOfRise > RATE_OF_RISE_LIMIT) rc = C_RED;
    else if (rateOfRise > RATE_OF_RISE_LIMIT * 0.7f) rc = C_AMBER;
  }
  panel(8, 256, 224, 28, rc == C_TEXT ? C_BORDER : rc);
  canvas.setFont(&fonts::FreeSans9pt7b);
  canvas.setTextDatum(textdatum_t::middle_left);
  canvas.setTextColor(C_DIM);
  canvas.drawString("RATE OF RISE", 16, 270);
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextDatum(textdatum_t::middle_right);
  canvas.setTextColor(rc);
  if (isnan(rateOfRise)) snprintf(b, sizeof(b), "-- C/min");
  else                   snprintf(b, sizeof(b), "%+.1f C/min", rateOfRise);
  canvas.drawString(b, 224, 270);

  // ----- STATUS BANNER -----
  uint16_t bannerCol = alarm ? (blink ? C_BANNER_AL : C_RED_DK)
                             : (warn ? RGB(190, 120, 0) : C_BANNER_OK);
  canvas.fillRoundRect(8, 288, 224, 24, 6, bannerCol);
  canvas.setFont(&fonts::FreeSansBold9pt7b);
  canvas.setTextDatum(textdatum_t::middle_center);
  canvas.setTextColor(C_TEXT);
  const char* msg;
  if (sensorFault)         msg = "! SENSOR FAULT !";
  else if (overTempLatched) msg = "! OVER TEMP !";
  else if (rateAlarm)       msg = "! RISE TOO FAST !";
  else if (warn)            msg = "TEMP WARNING";
  else                      msg = "SYSTEM NORMAL";
  canvas.drawString(msg, 120, 300);

  // bottom edge: hazard stripe on alarm, thin line otherwise
  if (alarm) hazardStripe(314, 6, C_AMBER, C_BG);
  else       canvas.drawFastHLine(8, 316, 224, C_BORDER);

  canvas.pushSprite(0, 0);
}

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  if (!ads1.begin(ADS_ADDR)) Serial.println("ADS1115 not found!");
  ads1.setGain(GAIN_ONE);
  ads1.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  delay(100);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  canvas.setColorDepth(16);
  if (!canvas.createSprite(240, 320)) {       // fall back to 8-bit if RAM is tight
    canvas.setColorDepth(8);
    canvas.createSprite(240, 320);
    Serial.println("Sprite: using 8-bit fallback");
  }

  senseTemperature();
  render(millis());
}

void loop() {
  unsigned long now = millis();

  if (now - lastSense >= SENSE_INTERVAL) {
    lastSense = now;
    senseTemperature();
  }

  if (now - lastTrend >= TREND_INTERVAL) {
    lastTrend = now;
    pushTrend();
  }

  if (now - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = now;
    render(now);
  }

  if (now - lastSerial >= 1000) {
    lastSerial = now;
    Serial.printf("Furnace Z1: %.2f C | RoR: %.1f C/min\n", tempF, rateOfRise);
  }
}
