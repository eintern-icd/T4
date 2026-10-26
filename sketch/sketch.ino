/*
 * NORVI X-T4 Industrial Furnace Monitor - Channel 1 (Professional UI)
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

// ---------- CONFIG & CONSTANTS ----------
const float SUPPLY_VOLTAGE      = 3.3;
const float SERIES_RESISTOR     = 10000.0;  
const float THERMISTOR_NOMINAL  = 10000.0;  
const float TEMPERATURE_NOMINAL = 25.0;     
const float B_COEFFICIENT       = 3892.0;   // Matched to Littelfuse 103JG1K J-Curve

const float MAX_SAFE_TEMP       = 300.0;    
const float RATE_OF_RISE_LIMIT  = 15.0;     

unsigned long lastDisplayUpdate = 0;
unsigned long lastRateCheck = 0;
const unsigned long DISPLAY_INTERVAL = 250;
const unsigned long RATE_INTERVAL    = 60000; 

float prevTemp0 = 0;
float rateOfRise0 = 0;
float coldJunctionTemp = 25.0; 

float readThermistorC(int16_t adcRaw) {
  float voltage = ads1.computeVolts(adcRaw);
  if (voltage <= 0.001 || voltage >= SUPPLY_VOLTAGE - 0.001) {
    return NAN;
  }
  float ntcResistance = SERIES_RESISTOR * (SUPPLY_VOLTAGE / voltage - 1.0);
  float steinhart = ntcResistance / THERMISTOR_NOMINAL;       
  steinhart = log(steinhart);                         
  steinhart /= B_COEFFICIENT;                           
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);    
  steinhart = 1.0 / steinhart;                          
  steinhart -= 273.15;                                 

  float cjcOffset = (coldJunctionTemp - 25.0) * 0.02; 
  return steinhart + cjcOffset;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(SDA_PIN, SCL_PIN);
  ads1.begin(ADS_ADDR);
  ads1.setGain(GAIN_ONE);
  ads1.setDataRate(RATE_ADS1115_860SPS);

  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  delay(100);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // --- HEADER FRAME ---
  tft.fillRect(0, 0, 240, 40, 0x1883); // Dark Blue Header Background
  tft.setTextColor(TFT_WHITE);
  tft.setFont(&fonts::FreeSansBold9pt7b);
  tft.setCursor(42, 25);
  tft.print("FURNACE CONTROL");

  // --- STATIC LABELS ---
  tft.setTextColor(TFT_LIGHTGREY);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setCursor(15, 65);
  tft.print("CHANNEL 1 (OVEN)");

  tft.setCursor(15, 175);
  tft.print("RATE OF RISE:");

  // Divider Line
  tft.drawFastHLine(10, 150, 220, TFT_DARKGREY);

  // --- FOOTER STATUS BAR ---
  tft.fillRect(0, 280, 240, 40, 0x2104); // Dark Grey Footer
  tft.setTextColor(TFT_CYAN);
  tft.setFont(&fonts::FreeSans9pt7b);
  tft.setCursor(15, 305);
  tft.print("CJC Active | 860 SPS");
}

void loop() {
  int16_t adc0 = ads1.readADC_SingleEnded(0);
  float temp0 = readThermistorC(adc0);

  unsigned long currentMillis = millis();

  if (currentMillis - lastRateCheck >= RATE_INTERVAL) {
    lastRateCheck = currentMillis;
    rateOfRise0 = temp0 - prevTemp0;
    prevTemp0 = temp0;
  }

  Serial.printf("Furnace Z1: %.2f C | RoR: %.1f C/min\n", temp0, rateOfRise0);

  if (currentMillis - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = currentMillis;
    char buf[30];

    // --- TEMPERATURE DISPLAY BOX ---
    tft.fillRect(12, 85, 216, 55, 0x10A2); // Charcoal Box for Temp
    tft.setCursor(20, 123);
    tft.setFont(&fonts::FreeSansBold12pt7b);
    
    if (isnan(temp0)) {
      tft.setTextColor(TFT_RED);
      tft.print("OPEN SENSOR");
    } else {
      dtostrf(temp0, 6, 2, buf);
      tft.setTextColor(temp0 > MAX_SAFE_TEMP ? TFT_RED : TFT_GREEN);
      tft.print(buf); 
      tft.setFont(&fonts::FreeSansBold9pt7b);
      tft.print(" C");
    }

    // --- RATE OF RISE DISPLAY ---
    tft.fillRect(135, 158, 95, 28, TFT_BLACK);
    tft.setCursor(140, 178);
    dtostrf(rateOfRise0, 5, 1, buf);
    tft.setTextColor(rateOfRise0 > RATE_OF_RISE_LIMIT ? TFT_RED : TFT_WHITE);
    tft.print(buf); tft.print(" C/m");

    // --- STATUS / ALARM BANNER ---
    bool alarmTriggered = (rateOfRise0 > RATE_OF_RISE_LIMIT || temp0 > MAX_SAFE_TEMP);

    tft.fillRect(10, 210, 220, 50, alarmTriggered ? TFT_RED : 0x0320); // Red for alarm, Deep Green for normal
    tft.setCursor(22, 241);
    tft.setFont(&fonts::FreeSansBold9pt7b);
    
    if (alarmTriggered) {
      tft.setTextColor(TFT_WHITE);
      tft.print("!! ALARM ACTIVE !!");
    } else {
      tft.setTextColor(TFT_WHITE);
      tft.print("STATUS: NORMAL");
    }
  }
}