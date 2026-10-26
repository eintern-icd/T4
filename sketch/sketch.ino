/*
 * NORVI X-T4 (LovyanGFX Version)
 */

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <Wire.h>
#include <WiFi.h>
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

// ---------- THERMISTOR / DIVIDER CONFIG ----------
const float SUPPLY_VOLTAGE = 3.3;
const float SERIES_RESISTOR      = 10000.0;  
const float THERMISTOR_NOMINAL   = 10000.0;  
const float TEMPERATURE_NOMINAL  = 25.0;     
const float B_COEFFICIENT        = 3950.0;   

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 100;

float lastDisplay0 = -999.0;
float lastDisplay1 = -999.0;
float lastDisplay2 = -999.0;
float lastDisplay3 = -999.0;

// Converts a raw ADS1115 single-ended reading into a temperature (deg C)
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

  return steinhart;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("NPD-NORVI X-T4 NTC TEMPERATURE TEST");
  
  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!ads1.begin(ADS_ADDR)) {
    Serial.println("Failed to initialize ADS 1.");
  } else {
    Serial.println("ADS1115 OK");
  }

  ads1.setGain(GAIN_ONE);
  ads1.setDataRate(RATE_ADS1115_860SPS);

  Serial.print("ADS1115 ADDRESS: 0x");
  if (ADS_ADDR < 0x10) Serial.print("0");
  Serial.println(ADS_ADDR, HEX);
  Serial.println("ADS1115 DATA RATE = 860 SPS");

  // SPI
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  delay(100);

  // TFT Init via LovyanGFX
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // TITLE
  tft.setTextColor(TFT_GREEN);
  tft.setFont(&fonts::FreeSansBold12pt7b);
  tft.setCursor(40, 30);
  tft.print("NORVI X-T4");

  // TEMPERATURE LABELS
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  
  tft.setCursor(10, 85);
  tft.print("TEMP 1 :");

  tft.setCursor(10, 115);
  tft.print("TEMP 2 :");

  tft.setCursor(10, 145);
  tft.print("TEMP 3 :");

  tft.setCursor(10, 175);
  tft.print("TEMP 4 :");

  // INITIAL TEMPERATURE VALUES
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  
  tft.setCursor(145, 85);
  tft.print("0.00 C");

  tft.setCursor(145, 115);
  tft.print("0.00 C");

  tft.setCursor(145, 145);
  tft.print("0.00 C");

  tft.setCursor(145, 175);
  tft.print("0.00 C");

  // ADS1115 ADDRESS AT BOTTOM
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setCursor(10, 215);
  tft.print("ADS ADDR : 0x");
  if (ADS_ADDR < 0x10) tft.print("0");
  tft.print(ADS_ADDR, HEX);

  Serial.println("TEST START");
}

void loop() {
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  int16_t adc1 = ads1.readADC_SingleEnded(1);
  int16_t adc2 = ads1.readADC_SingleEnded(2);
  int16_t adc3 = ads1.readADC_SingleEnded(3);

  float temp0 = readThermistorC(adc0);
  float temp1 = readThermistorC(adc1);
  float temp2 = readThermistorC(adc2);
  float temp3 = readThermistorC(adc3);

  // SERIAL MONITOR
  Serial.printf(
    "T1: %.2f C  T2: %.2f C  T3: %.2f C  T4: %.2f C\n",
    temp0, temp1, temp2, temp3
  );

  // TFT UPDATE
  if (millis() - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = millis();

    char buf[20];
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setFont(&fonts::FreeSansBold9pt7b);

    // TEMP 1
    if (temp0 != lastDisplay0) {
      tft.fillRect(140, 68, 90, 22, TFT_BLACK);
      tft.setCursor(145, 85);
      if (isnan(temp0)) {
        tft.print("--.-- C");
      } else {
        dtostrf(temp0, 5, 2, buf);
        tft.print(buf);
        tft.print(" C");
      }
      lastDisplay0 = temp0;
    }
    
    // TEMP 2
    if (temp1 != lastDisplay1) {
      tft.fillRect(140, 98, 90, 22, TFT_BLACK);
      tft.setCursor(145, 115);
      if (isnan(temp1)) {
        tft.print("--.-- C");
      } else {
        dtostrf(temp1, 5, 2, buf);
        tft.print(buf);
        tft.print(" C");
      }
      lastDisplay1 = temp1;
    }

    // TEMP 3
    if (temp2 != lastDisplay2) {
      tft.fillRect(140, 128, 90, 22, TFT_BLACK);
      tft.setCursor(145, 145);
      if (isnan(temp2)) {
        tft.print("--.-- C");
      } else {
        dtostrf(temp2, 5, 2, buf);
        tft.print(buf);
        tft.print(" C");
      }
      lastDisplay2 = temp2;
    }

    // TEMP 4
    if (temp3 != lastDisplay3) {
      tft.fillRect(140, 158, 90, 22, TFT_BLACK);
      tft.setCursor(145, 175);
      if (isnan(temp3)) {
        tft.print("--.-- C");
      } else {
        dtostrf(temp3, 5, 2, buf);
        tft.print(buf);
        tft.print(" C");
      }
      lastDisplay3 = temp3;
    }
  }
}