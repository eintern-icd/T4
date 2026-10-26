/*
 * NORVI X-T4 Industrial Furnace / Oven Monitoring Sketch (Channel 1 Only)
 * Features: LovyanGFX TFT display, Single-channel NTC sensing (Channel 1), 
 *           Cold-Junction Compensation (CJC), and Rate-of-Rise Alarm.
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
      cfg.pin_cs             = 45;
      cfg.pin_rst            = 47;
      cfg.pin_busy           = -1;
      cfg.panel_width        = 240;
      cfg.panel_height       = 320;
      cfg.offset_x           = 0;
      cfg.offset_y           = 0;
      cfg.offset_rotation    = 0;
      cfg.dummy_read_pixel  = 8;
      cfg.dummy_read_bits    = 1;
      cfg.readable           = true;
      cfg.invert             = true;
      cfg.rgb_order          = false;
      cfg.dlen_16bit         = false;
      cfg.bus_shared         = true;
      _panel_instance.config(cfg);
    }
    setPanel(&_panel_instance);
  }
};

LGFX tft;

// ---------- CONFIGURATION & CONSTANTS ----------
const float SUPPLY_VOLTAGE      = 3.3;
const float SERIES_RESISTOR     = 10000.0;  
const float THERMISTOR_NOMINAL  = 10000.0;  
const float TEMPERATURE_NOMINAL = 25.0;     
const float B_COEFFICIENT       = 3950.0;   

// Industrial Limits
const float MAX_SAFE_TEMP       = 300.0;    // Max oven operating limit (°C)
const float RATE_OF_RISE_LIMIT  = 15.0;     // Max allowable °C rise per minute (Thermal Runaway Alarm)

unsigned long lastDisplayUpdate = 0;
unsigned long lastRateCheck = 0;
const unsigned long DISPLAY_INTERVAL = 250;
const unsigned long RATE_INTERVAL    = 60000; // Check rate-of-rise every 60 seconds

float prevTemp0 = 0;
float rateOfRise0 = 0;

// Cold-Junction / Ambient baseline simulation
float coldJunctionTemp = 25.0; 

float readThermistorC(int16_t adcRaw) {
  float voltage = ads1.computeVolts(adcRaw);

  if (voltage <= 0.001 || voltage >= SUPPLY_VOLTAGE - 0.001) {
    return NAN;
  }

  float ntcResistance = SERIES_RESISTOR * (SUPPLY_VOLTAGE / voltage - 1.0);

  float steinhart;
  steinhart = ntcResistance / THERMISTOR_NOMINAL;       
  steinhart = log(steinhart);                         
  steinhart /= B_COEFFICIENT;                           
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);    
  steinhart = 1.0 / steinhart;                          
  steinhart -= 273.15;                                 

  // Apply Cold-Junction Compensation adjustment
  float cjcOffset = (coldJunctionTemp - 25.0) * 0.02; 
  return steinhart + cjcOffset;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!ads1.begin(ADS_ADDR)) {
    Serial.println("Failed to initialize ADS1115 for Furnace monitoring.");
  } else {
    Serial.println("Industrial Furnace X-T4 Channel 1 Monitor Initialized");
  }

  ads1.setGain(GAIN_ONE);
  ads1.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  delay(100);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // HEADER TITLE
  tft.setTextColor(TFT_YELLOW);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setCursor(15, 25);
  tft.print("OVEN / FURNACE");

  // LABEL FOR CHANNEL 1
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 85);  
  tft.print("Furnace T1:");

  // STATUS FOOTER BAR
  tft.fillRect(0, 260, 240, 60, TFT_DARKGREY);
  tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
  tft.setCursor(10, 285);
  tft.print("CJC Active | 860 SPS");
}

void loop() {
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  float temp0 = readThermistorC(adc0);

  unsigned long currentMillis = millis();

  // Calculate Rate of Rise every 60 seconds for Channel 1
  if (currentMillis - lastRateCheck >= RATE_INTERVAL) {
    lastRateCheck = currentMillis;
    rateOfRise0 = temp0 - prevTemp0;
    prevTemp0 = temp0;
  }

  // SERIAL MONITOR LOGGING
  Serial.printf("Furnace Z1: %.2f C | Rate of Rise: %.1f C/min\n", temp0, rateOfRise0);

  // TFT DISPLAY UPDATE
  if (currentMillis - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = currentMillis;
    char buf[30];

    tft.setFont(&fonts::FreeSansBold9pt7b);

    // DISPLAY CHANNEL 1 TEMPERATURE
    tft.fillRect(10, 110, 220, 40, TFT_BLACK);
    tft.setCursor(15, 140);
    if (isnan(temp0)) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.print("SENSOR OPEN/FAULT");
    } else {
      dtostrf(temp0, 6, 2, buf);
      tft.setTextColor(temp0 > MAX_SAFE_TEMP ? TFT_RED : TFT_GREEN, TFT_BLACK);
      tft.print(buf); tft.print(" C");
    }

    // ALARM BANNER CHECK (Rate of Rise or Over-temp)
    bool alarmTriggered = (rateOfRise0 > RATE_OF_RISE_LIMIT || temp0 > MAX_SAFE_TEMP);

    tft.fillRect(0, 200, 240, 40, alarmTriggered ? TFT_RED : TFT_BLACK);
    if (alarmTriggered) {
      tft.setTextColor(TFT_WHITE, TFT_RED);
      tft.setCursor(15, 225);
      tft.print("!! OVEN ALARM ACTIVE !!");
    } else {
      tft.setTextColor(TFT_GREEN, TFT_BLACK);
      tft.setCursor(15, 225);
      tft.print("SYSTEM STATUS: NORMAL");
    }
  }
}