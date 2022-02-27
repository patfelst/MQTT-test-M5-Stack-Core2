#include <ArduinoOTA.h>
#include <ESP32-Chimera-Core.h>
#include <ESPmDNS.h>
#include <FastLED.h>
#include <OneButton.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "wifi_credentials.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;

// MQTT settings
IPAddress mqttServer(192, 168, 0, 16);  // MQTT Server IP address, i.e. ESPHome IP address
const int mqttPort = 1883;
const char* mqttUser = "esp32";
const char* mqttPassword = "core2";
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
const char* stateTopic = "iron_switch";
const char* commandTopic = "iron_cmd";

#define sw_version         "v0.20"
#define TFT_WIDTH          320  // The library WIDTH is the short side
#define TFT_HEIGHT         240  // The library HEIGHT is the long side
#define buz_duration       200  // When touch buttons are pressed, vibrate the motor for 200ms
#define timer_duration_sec 300

// Title bar data
#define title_txt_x_offs 44
#define title_txt_y_offs 8
#define title_bar_height 45
#define title_str        "Iron Timer"

// Battery icon data
#define batt_spr_wdth    85
#define batt_spr_ht      42
#define batt_rect_width  16
#define batt_rect_height 35
#define batt_button_wdth 6
#define batt_button_ht   4

// Time text mm:ss data
#define time_spr_wdth      135
#define time_spr_ht        42
#define time_spr_x         (TFT_WIDTH / 2) - (time_spr_wdth / 2)
#define time_spr_y         time_msg_y + 53
#define timer_txt_bg_color TFT_BLACK

// Text message above time text
#define time_msg_x               (TFT_WIDTH / 2)
#define time_msg_y               (TFT_HEIGHT / 2 - 50)
#define time_msg_bg_color        TFT_BLACK
#define secs_remain_shutdown_msg 5  // At 5 seconds remaining, let the user know
#define switch_off_msg           "Switch off in"
#define time_left_msg            "Time Left"

// Label the touch buttons at bottom of screen
#define BtnA_x        52
#define BtnB_x        (TFT_WIDTH / 2)
#define BtnC_x        (TFT_WIDTH - BtnA_x - 3)
#define Btn_tri_width 20
#define Btn_tri_ht    (TFT_HEIGHT - 20)

// Wake on touch GPIO
#define touch_pin_gpio          27
#define touch_pin_low_threshold 55  // When touched, the touch reading falls. If below this value, ESP32 will reboot

// Timer bar graph
#define tb_fill_color    TFT_GREEN
#define tb_border_color  TFT_DARKGREY
#define tb_left_margin   25
#define tb_right_margin  25
#define tb_bottom_margin 37
#define tb_width         (TFT_WIDTH - tb_right_margin - tb_left_margin)
#define tb_height        25

// RGB LED defines
#define LED_COUNT 10
#define LED_PIN   25

void draw_titlebar();
void label_touch_buttons();
uint8_t lipo_capacity_percent(float);
void disp_batt_symbol(uint16_t batt_x, uint16_t batt_y, bool disp_volts);
void draw_timer_msg(const char* msg);
void haptic_buzz(uint16_t duration_ms);
void display_pmu_vals();
uint8_t touch_x_to_percent(uint32_t touch_x);
void progress_bar(uint8_t percent);
void bargraph_scale(uint8_t major_ticks, bool scale_type);
void clear_centre_lcd();
void myOTA_onStart();
void myOTA_onProgress(unsigned int progress, unsigned int total);
void myOTA_onEnd();
void myOTA_onError(ota_error_t error);
void display_touch_read(uint8_t gpio_pin);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void button_1_click();
void button_1_longpress();
void button_2_click();
void button_2_longpress();

uint32_t iron_timer = timer_duration_sec;  // Initial time is 5 minutes = 300 seconds
uint16_t button_label_colour;
uint16_t title_bar_bg_colour = M5.Lcd.color24to16(0x99ddff);
uint16_t title_bar_txt_colour = M5.Lcd.color24to16(0x262626);
uint32_t last_iron_time = 0;
uint32_t last_display_update = 0;

TFT_eSprite BattSprite = TFT_eSprite(&M5.Lcd);
TFT_eSprite TimerTxtSprite = TFT_eSprite(&M5.Lcd);
TFT_eSprite TimerBarSprite = TFT_eSprite(&M5.Lcd);

// Input pin for the button / active low button / enable internal pull-up resistor
OneButton button_1 = OneButton(32, true, true);
OneButton button_2 = OneButton(33, true, true);

CRGB leds[LED_COUNT];  // WS2812 RGB LED object

/*
  touchCallback()

  Description:
  ------------
  * Callback function for GPIO touch interrupts

*/
void touchCallback() {
}

void button_1_click() {
  if (iron_timer >= 125)
    iron_timer -= 120;
  else
    iron_timer = 5;
}

void button_2_click() {
  iron_timer += 120;
}

void button_1_longpress() {
  if (iron_timer >= 5)
    iron_timer = 5;
}

void button_2_longpress() {
}

/*
-----------------
  setup()
-----------------
*/
void setup() {
  M5.begin();
  M5.Axp.SetLed(0);  // Turn off green LED

  draw_titlebar();

  // Setup RGB LED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.setBrightness(0);
  fill_solid(leds, LED_COUNT, CRGB::Black);
  FastLED.show();

  // Setup button one button callbacks
  button_1.attachClick(button_1_click);
  button_1.attachLongPressStart(button_1_longpress);
  button_2.attachClick(button_2_click);
  button_2.attachLongPressStart(button_2_longpress);

  // Setup for push button to wake from deep sleep
  // esp_sleep_enable_ext1_wakeup(0x300000000, ESP_EXT1_WAKEUP_ALL_LOW); // Wake up when GPIO32 and GPIO33 are pressed together
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);

  // Create sprite for battery symbol
  BattSprite.createSprite(batt_spr_wdth, batt_spr_ht);

  // Create sprite for time remaining mins:secs display
  TimerTxtSprite.createSprite(time_spr_wdth, time_spr_ht);
  TimerTxtSprite.setFont(&fonts::FreeSansBold24pt7b);
  TimerTxtSprite.setTextDatum(top_center);
  TimerTxtSprite.setTextPadding(TimerTxtSprite.textWidth("00:00"));

  // Create sprite for time remaining bargraph
  TimerBarSprite.createSprite(tb_width, tb_height);

  // Setup WiFi
  draw_timer_msg("Connecting WiFi");
  // Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Start MQTT client
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqtt_callback);

  // Setup callbacks for OTA updates
  ArduinoOTA.onStart(myOTA_onStart);
  ArduinoOTA.onProgress(myOTA_onProgress);
  ArduinoOTA.onEnd(myOTA_onEnd);
  ArduinoOTA.onError(myOTA_onError);

  draw_timer_msg("Done");
  delay(500);
  draw_timer_msg(time_left_msg);

  // Display a full progress bar to begin count down timer
  TimerBarSprite.drawRect(0, 0, tb_width, tb_height, tb_border_color);
  TimerBarSprite.pushSprite(tb_left_margin, TFT_HEIGHT - tb_height - tb_bottom_margin);
  progress_bar(100);  // Start timer with a full bar
  bargraph_scale(5, false);

  // Start the Over The Air (OTA) object
  ArduinoOTA.begin();
}

/*
-----------------
  loop()
-----------------
*/
void loop() {
  uint32_t tx = 0;
  uint32_t ty = 0;
  uint8_t percent = 0;
  uint16_t iron_seconds = 0;
  uint16_t iron_minutes = 0;

  // Check for WiFi OTA
  ArduinoOTA.handle();

  // Update MQTT client
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  button_1.tick();
  button_2.tick();

  if (M5.Lcd.getTouch(&tx, &ty)) {
    percent = touch_x_to_percent(tx);
    progress_bar(percent);
    iron_timer = (percent * timer_duration_sec) / 100;
    delay(20);
  }

  // Do the fast 250 ms updates
  if (millis() - last_display_update > 250) {
    last_display_update = millis();

    // Do the 1 second updates
    if (millis() - last_iron_time > 1000) {
      last_iron_time = millis();

      // Get Core2 battery charge capacity - only need to update this once per second
      disp_batt_symbol(TFT_WIDTH - batt_spr_wdth, 0, true);

      if (iron_timer > 0) {
        // Decrement the timer by 1 second
        iron_timer--;

      } else if (iron_timer == 0) {
        // MQTT code to turn iron OFF
        mqttClient.publish(stateTopic, "Off");
        M5.Lcd.sleep();
        // Enable interrupt on touch GPIO pin "touch_pin_gpio", when touch read falls below "touch_pin_low_threshold"
        // Core2 running on LiPo battery (not plugged into USB)
        //    Not touched:  75
        //    Touched:      53
        // Core2 plugged into mac via USB:
        //    Not touched:  75
        //    Touched:      20
        // touchAttachInterrupt(touch_pin_gpio, touchCallback, touch_pin_low_threshold);
        // esp_sleep_enable_touchpad_wakeup();

        delay(200);  // Give MQTT message time to be sent
        esp_deep_sleep_start();
      }
    }

    // For development, read touch level and display on LCD
    // display_touch_read(touch_pin_gpio);

    // Update the timer bar graph
    percent = (iron_timer * 100) / timer_duration_sec;
    progress_bar(percent);

    // Display the timer mm:ss text
    if (iron_timer > secs_remain_shutdown_msg)
      TimerTxtSprite.setTextColor(TFT_YELLOW, timer_txt_bg_color);
    else if (iron_timer <= secs_remain_shutdown_msg)
      TimerTxtSprite.setTextColor(TFT_RED, timer_txt_bg_color);
    iron_seconds = iron_timer % 60;
    iron_minutes = iron_timer / 60;
    char txt[50] = "";
    sprintf(txt, "%2d:%02d", iron_minutes, iron_seconds);
    // Serial.println(txt);
    TimerTxtSprite.drawString(txt, time_spr_wdth / 2, 0);
    // Display the sprite
    TimerTxtSprite.pushSprite(time_spr_x, time_spr_y);
  }
}

/*
  progress_bar()

  Description:
  ------------
  * Display in horizontal progress bar graph

  Inputs:
  -------
  * percent - 0% to 100%

  Return:
  -------
  * void
*/
void progress_bar(uint8_t percent) {
  static uint32_t last_x = 1;  // Start 1-pixel in to not overwrite border line
  uint32_t this_x = 0;
  int32_t width = 0;

  // Out of limit checks
  if (percent >= 100)
    this_x = tb_width - 1;  // Minus 1 so we don't overwrite the RHS of border rectangle
  else if (percent == 0)
    this_x = 1;  // Plus 1 so we don't overwrite the LHS of border rectangle
  else
    // Convert percent value into sprite width
    this_x = (percent * tb_width) / 100;
  // Serial.printf("this_x = %d, last_spr_x=%d\n", this_x, last_spr_x);

  if (this_x < last_x) {
    width = last_x - this_x;
    TimerBarSprite.fillRect(this_x, 1, width, tb_height - 2, TFT_BLACK);  // Erase the unneeded portion of this bar
  } else if (this_x > last_x) {
    width = this_x - last_x;
    TimerBarSprite.fillRect(last_x, 1, width, tb_height - 2, tb_fill_color);
  }
  TimerBarSprite.pushSprite(tb_left_margin, TFT_HEIGHT - tb_height - tb_bottom_margin);
  last_x = this_x;
}

/*
  bargraph_scale()

  Description:
  ------------
  * Draw ruler type scale under bargraph

  Inputs:
  -------
  * None
*/
void bargraph_scale(uint8_t major_ticks, bool scale_type) {
  uint16_t step = 0;
  uint16_t minor_ticks = major_ticks * 2;
  uint16_t pix_per_min_tick = (tb_width + 0) / minor_ticks;
  uint16_t pix_per_maj_tick = 2 * pix_per_min_tick;
  const uint16_t tick_y = TFT_HEIGHT - tb_bottom_margin + 3;
  const uint16_t txt_y = TFT_HEIGHT - 1;
  char txt[10] = "";

  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  M5.Lcd.setTextPadding(0);
  M5.Lcd.setTextDatum(bottom_center);

  // Clear the old scale
  M5.Lcd.fillRect(tb_left_margin - 20, TFT_HEIGHT - tb_bottom_margin + 1, tb_width + 30, tb_bottom_margin - 2, TFT_BLACK);

  for (step = 0; step <= tb_width; step++) {
    if (step % pix_per_maj_tick == 0) {
      M5.Lcd.drawFastVLine(tb_left_margin + step, tick_y, 12, TFT_LIGHTGRAY);

      if (scale_type) {
        sprintf(txt, "%2d", (step * 100) / tb_width);
        M5.Lcd.drawString(txt, tb_left_margin + step, txt_y);
      } else {
        sprintf(txt, "%d", (step * timer_duration_sec) / (tb_width * 60));
        M5.Lcd.drawString(txt, tb_left_margin + step, txt_y);
      }
    } else if (step % pix_per_min_tick == 0)
      M5.Lcd.drawFastVLine(tb_left_margin + step, tick_y, 5, TFT_LIGHTGRAY);
  }
}

/*
  touch_x_to_percent()

  Description:
  ------------
  * Takes touch screen x-coord and converts into pecentage for display on progress bar

  Inputs:
  -------
  * touch_x - 0 to TFT_WIDTH (320)

  Return:
  -------
  * 0% to 100%
*/
uint8_t touch_x_to_percent(uint32_t touch_x) {
  uint8_t percent = 0;
  // Convert touch x-coord into a sprite rectangle fill amount (this_spr_x), i.e. left margin is sprite zero
  if (touch_x >= (TFT_WIDTH - tb_right_margin))
    percent = 100;
  else if (touch_x <= tb_left_margin)
    percent = 0;
  else
    percent = (uint8_t)(((touch_x - tb_left_margin) * 100) / tb_width);
  // Serial.printf("this_spr_x = %d, last_spr_x=%d\n", this_spr_x, last_spr_x);
  return percent;
}

void draw_titlebar() {
  // Mix some new colours for the title bar

  // Draw title bar
  M5.Lcd.drawRect(0, 0, TFT_WIDTH, TFT_HEIGHT, title_bar_bg_colour);
  M5.Lcd.fillRect(0, 0, TFT_WIDTH, title_bar_height, title_bar_bg_colour);

  // Display Title String
  M5.Lcd.setFont(&fonts::FreeSansBold18pt7b);  // &fonts::FreeSerif9pt7b
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.setTextColor(title_bar_txt_colour);
  M5.Lcd.drawString(title_str, title_txt_x_offs, title_txt_y_offs);

  // Display software version
  M5.Lcd.setFont(&fonts::Font2);
  M5.Lcd.drawString(sw_version, 5, 15);
}

/*
-----------------
  Returns the estimated battery capacity remaining in a LiPo battery based on its measured voltage
  According to:
    Percentage = 123 - 123 / POWER(1 + POWER(Voltage /3.7, 80), 0.165)
-----------------
*/
uint8_t lipo_capacity_percent(float voltage) {
  float percent = 123.0 - (123.0 / pow(1 + pow(voltage / 3.7, 80), 0.165));

  if (percent > 100.0) percent = 100.0;
  if (percent < 0.0) percent = 0.0;
  return (uint8_t)(round(percent));
}

/*
-----------------
  Display Core2 battery symbol, % charge, and voltage
  batt_x - X coordinate of LEFT of battery symbol
  batt_y - Y coordinate of BOTTOM of battery symbol
  batt_volt - LiPo voltage. Range is 3.7V to 4.2V
  disp_volts - boolean, display voltage when true
-----------------
*/
void disp_batt_symbol(uint16_t batt_x, uint16_t batt_y, bool disp_volts) {
  float batt_volt = M5.Axp.GetBatVoltage();
  uint8_t batt_percent = lipo_capacity_percent(batt_volt);
  int16_t batt_fill_length = (batt_percent * batt_rect_height) / 100;
  uint16_t fill_colour = TFT_MAGENTA;
  uint16_t outline_colour = TFT_BLACK;
  uint16_t spr_x_offs = 9;  // X-axis offset of battery icon and voltage text in sprite
  const uint16_t erase_fill_colour = title_bar_bg_colour;

  // Clear the old values
  BattSprite.fillSprite(title_bar_bg_colour);  // Clear the battery icon sprite

  // Display battery % charge text
  BattSprite.setFont(&FreeSans9pt7b);
  BattSprite.setTextDatum(bottom_left);

  // Display battery percentage
  char txt[20] = "";
  uint16_t percent_txt_y = batt_spr_ht - 8;
  BattSprite.setTextColor(title_bar_txt_colour, title_bar_bg_colour);

  if (disp_volts) percent_txt_y -= 12;

  BattSprite.setTextPadding(BattSprite.textWidth("xxxx%"));
  sprintf(txt, "%3d%%", batt_percent);
  BattSprite.drawString(txt, spr_x_offs + batt_rect_width + 7, percent_txt_y);

  if (disp_volts) {
    sprintf(txt, "%.2fV", batt_volt);
    BattSprite.drawString(txt, spr_x_offs + batt_rect_width + 7, batt_spr_ht);
  }

  if (batt_percent < 20)
    fill_colour = TFT_RED;
  else if (batt_percent >= 20 && batt_percent < 50)
    fill_colour = TFT_ORANGE;
  else if (batt_percent >= 50)
    fill_colour = TFT_DARKGREEN;

  // Draw the battery symbol outline
  BattSprite.drawRect(spr_x_offs, batt_spr_ht - batt_rect_height, batt_rect_width, batt_rect_height, outline_colour);

  // Draw the button on top of the battery - intentional gap from the main battery rectangle
  BattSprite.fillRect(spr_x_offs + (batt_rect_width / 2) - (batt_button_wdth / 2), batt_spr_ht - batt_rect_height - batt_button_ht - 1, batt_button_wdth, batt_button_ht, outline_colour);

  // Erase the old battery level
  BattSprite.fillRect(spr_x_offs + 2, batt_spr_ht - batt_rect_height + 2, batt_rect_width - 4, batt_rect_height - 4, erase_fill_colour);

  // Draw the current battery level
  BattSprite.fillRect(spr_x_offs + 2, batt_spr_ht - batt_fill_length + 2, batt_rect_width - 4, batt_fill_length - 4, fill_colour);

  // Draw lighning bolt symbol
  if (M5.Axp.isCharging()) {
    uint16_t cntre_x = spr_x_offs + (batt_rect_width / 2);
    uint16_t cntre_y = batt_spr_ht - (batt_rect_height / 2) - 3;
    BattSprite.fillTriangle(cntre_x - 15, cntre_y - 2, cntre_x, cntre_y, cntre_x + 2, cntre_y + 6, TFT_ORANGE);
    BattSprite.fillTriangle(cntre_x + 15, cntre_y + 2, cntre_x, cntre_y, cntre_x - 2, cntre_y - 6, TFT_ORANGE);
  }

  // Display the sprite
  BattSprite.pushSprite(batt_x, batt_y);
}

/*
  draw_timer_msg()

  Description:
  ------------
  * Display message above the timer text, e.g. "Time Left"
    No need for a sprite as is relatively static text

  Inputs:
  -------
  * msg - pointer to string containing message
*/
void draw_timer_msg(const char* msg) {
  M5.Lcd.setFont(&fonts::FreeSansBold18pt7b);
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setTextColor(TFT_LIGHTGREY, time_msg_bg_color);
  M5.Lcd.setTextPadding(TFT_WIDTH - 20);
  M5.Lcd.drawString(msg, time_msg_x, time_msg_y);
}

/*
  label_touch_buttons()

  Description:
  ------------
  * Labels the three Core2 touch buttons below the main LCD with triangle pointers

  Inputs:
  -------
  * None
*/
void label_touch_buttons() {
  button_label_colour = title_bar_bg_colour;
  M5.Lcd.fillTriangle(BtnA_x - (Btn_tri_width / 2), Btn_tri_ht, BtnA_x + (Btn_tri_width / 2), Btn_tri_ht, BtnA_x, TFT_HEIGHT - 5, button_label_colour);
  M5.Lcd.fillTriangle(BtnC_x - (Btn_tri_width / 2), Btn_tri_ht, BtnC_x + (Btn_tri_width / 2), Btn_tri_ht, BtnC_x, TFT_HEIGHT - 5, button_label_colour);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Lcd.drawString("+60", BtnA_x - 25, Btn_tri_ht - 25);
  M5.Lcd.drawString("Off", BtnC_x - 16, Btn_tri_ht - 25);
}

/*
  haptic_buzz()

  Description:
  ------------
  * Pulse the Core2 haptic motor for a short period

  Inputs:
  -------
  * duration_ms - the amount of time to turn the buzzer on
*/
void haptic_buzz(uint16_t duration_ms) {
  M5.Axp.SetLDOEnable(3, true);  // Start the vibration
  delay(duration_ms);
  M5.Axp.SetLDOEnable(3, false);  // Stop the vibration
}

/*
  display_pmu_vals()

  Description:
  ------------
  * Display the Core2 Power Management Unit values

  Inputs:
  -------
  * None
*/
void display_pmu_vals() {
  char txt[40] = "";
  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLUE);
  M5.Lcd.setTextPadding(M5.Lcd.textWidth("VBusCurrent = 00.0mA"));
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.drawRect(0, 50, 200, TFT_HEIGHT - 55, TFT_YELLOW);

  uint16_t xpos = 6;
  uint16_t ypos = 55;
  const uint16_t font_ht = 22;

  // Note: isCharging() referred to wrong bit, should be 0x04, not 0x02
  // bool AXP192_M5Core2::isCharging()
  //   return ( Read8bit(0x00) & 0x04 ) ? true : false;
  sprintf(txt, "Bat Charging = %s\n", M5.Axp.isCharging() ? "true" : "false");
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;

  sprintf(txt, "BatVoltage = %.2fV\n", M5.Axp.GetBatVoltage());
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;

  sprintf(txt, "BatCurrent = %.2fmA\n", M5.Axp.GetBatCurrent());
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;

  sprintf(txt, "VinVoltage = %.2fV\n", M5.Axp.GetVinVoltage());
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;

  sprintf(txt, "VinCurrent = %.2fmA\n", M5.Axp.GetVinCurrent());
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;

  sprintf(txt, "VBusVoltage = %.2fV\n", M5.Axp.GetVBusVoltage());
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;

  sprintf(txt, "VBusCurrent = %.2fmA\n", M5.Axp.GetVBusCurrent());
  M5.Lcd.drawString(txt, xpos, ypos);
  ypos += font_ht;
}

void clear_centre_lcd() {
  // Clear the main central part of the LCD below title bar, and above bar graph
  M5.Lcd.fillRect(2, title_bar_height + 1, TFT_WIDTH - 4, TFT_HEIGHT - tb_height - tb_bottom_margin - title_bar_height - 5, TFT_BLACK);
}

/*
  myOTA_onStart()

  Description:
  ------------
  * Callback function for start of OTA update
*/
void myOTA_onStart() {
  String type;
  if (ArduinoOTA.getCommand() == U_FLASH)
    type = "sketch";
  else  // U_SPIFFS
    type = "filesystem";
  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  // Serial.println("Start updating " + type);

  // Display updating OTA message on LCD
  clear_centre_lcd();
  bargraph_scale(5, true);
  M5.Lcd.setFont(&fonts::FreeSansBold18pt7b);
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  M5.Lcd.drawString("Updating OTA", TFT_WIDTH / 2, title_bar_height + 15);

  // Display the ESP32's IP address
  char txt[40] = "";
  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setTextDatum(top_left);
  uint16_t ypos = title_bar_height + 15 + 50;
  sprintf(txt, "IP: %s", WiFi.localIP().toString().c_str());
  M5.Lcd.drawString(txt, 20, ypos);
}

/*
  myOTA_onProgress()

  Description:
  ------------
  * Callback for WiFI OTA upload progress
*/
void myOTA_onProgress(unsigned int progress, unsigned int total) {
  uint8_t percent = (uint8_t)((progress * 100) / total);
  char txt[40];

  // Display the ESP32's WiFi signal strength
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setTextPadding(120);
  uint16_t ypos = title_bar_height + 15 + 50;
  sprintf(txt, "RSSI: %2d dB", WiFi.RSSI());
  M5.Lcd.drawString(txt, 190, ypos);

  // Display percent done
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setFont(&fonts::FreeSansBold18pt7b);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.setTextPadding(80);
  sprintf(txt, "%2d%%", percent);
  M5.Lcd.drawString(txt, TFT_WIDTH / 2, TFT_HEIGHT - 100);

  // Display OTA progress bar
  progress_bar(percent);
}

/*
  myOTA_onEnd()

  Description:
  ------------
  * Callback for end of WiFI OTA upload
*/
void myOTA_onEnd() {
  // Serial.println("\nEnd");
  clear_centre_lcd();
  M5.Lcd.setFont(&fonts::FreeSansBold18pt7b);
  M5.Lcd.setTextColor(TFT_LIGHTGRAY, TFT_BLACK);
  M5.Lcd.drawString("Finished OTA!", TFT_WIDTH / 2, title_bar_height + 40);
  M5.Lcd.drawString("Rebooting...", TFT_WIDTH / 2, title_bar_height + 80);
}

/*
  myOTA_onError()

  Description:
  ------------
  * Callback for error during WiFI OTA upload
*/
void myOTA_onError(ota_error_t error) {
  char txt[40] = "";
  uint16_t ypos = title_bar_height + 15;
  uint16_t xpos = 50;

  // Prepare LCD for error display
  clear_centre_lcd();
  M5.Lcd.setTextPadding(0);
  M5.Lcd.setFont(&fonts::FreeSans12pt7b);
  M5.Lcd.setTextDatum(top_center);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.drawString("WiFi OTA Error", TFT_WIDTH / 2, ypos);

  ypos += 30;
  M5.Lcd.setTextDatum(top_left);

  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
  sprintf(txt, "Error[%u]:", error);
  M5.Lcd.drawString(txt, xpos, ypos);
  // Serial.printf("Error[%u]: ", error);
  xpos += 100;

  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  if (error == OTA_AUTH_ERROR) {
    // Serial.println("Auth Failed");
    M5.Lcd.drawString("Auth Failed", xpos, ypos);
  } else if (error == OTA_BEGIN_ERROR) {
    // Serial.println("Begin Failed");
    M5.Lcd.drawString("Begin Failed", xpos, ypos);
  } else if (error == OTA_CONNECT_ERROR) {
    // Serial.println("Connect Failed");
    M5.Lcd.drawString("Connect Failed", xpos, ypos);
  } else if (error == OTA_RECEIVE_ERROR) {
    // Serial.println("Receive Failed");
    M5.Lcd.drawString("Receive Failed", xpos, ypos);
  } else if (error == OTA_END_ERROR) {
    // Serial.println("End Failed");
    M5.Lcd.drawString("End Failed", xpos, ypos);
  }
}

/*
  display_touch_read()

  Description:
  ------------
  * For development only, read the touch pin value and display it on the LCD

  Inputs:
  -------
  * gpio_pin - the GPIO pin used for touch input
*/
void display_touch_read(uint8_t gpio_pin) {
  char txt2[50] = "";
  M5.Lcd.setFont(&fonts::FreeSans9pt7b);
  M5.Lcd.setTextDatum(top_left);
  M5.Lcd.setTextPadding(70);
  M5.Lcd.setTextColor(TFT_MAGENTA, TFT_BLACK);
  sprintf(txt2, "GPIO-%d touch=%d. Wake Thresh=%d", touch_pin_gpio, touchRead(gpio_pin), touch_pin_low_threshold);
  M5.Lcd.drawString(txt2, 5, 50);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    // digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    // digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

/*
  reconnect()

  Description:
  ------------
  * Reconnect connection with MQTT broker

*/
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random mqttClient ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Once connected, publish switch turn ON
      mqttClient.publish(stateTopic, "On");
      // ... and resubscribe
      mqttClient.subscribe(commandTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
