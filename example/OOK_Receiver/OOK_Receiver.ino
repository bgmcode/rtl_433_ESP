/*
 Basic rtl_433_ESP example for OOK/ASK Devices

*/

#include <ArduinoJson.h>
#include <ArduinoLog.h>
#include <rtl_433_ESP.h>
#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h" // legacy include: `#include "SSD1306.h"`
// or #include "SH1106Wire.h", legacy include: `#include "SH1106.h"`
// For a connection via I2C using brzo_i2c (must be installed) include
// #include <brzo_i2c.h> // Only needed for Arduino 1.6.5 and earlier
// #include "SSD1306Brzo.h"
// #include "SH1106Brzo.h"
// For a connection via SPI include
// #include <SPI.h> // Only needed for Arduino 1.6.5 and earlier
// #include "SSD1306Spi.h"
// #include "SH1106SPi.h"

// Include the UI lib
#include "OLEDDisplayUi.h"

// Install https://github.com/PaulStoffregen/Time
#include <TimeLib.h>
#define PINOUT_DOORBELL 12

const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};


SSD1306Wire display(0x3c, SDA, SCL);   // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SH1106Wire display(0x3c, SDA, SCL);

OLEDDisplayUi ui ( &display );

int screenW = 128;
int screenH = 64;
int clockCenterX = screenW / 2;
int clockCenterY = ((screenH/12)); // - 16) / 2) + 16; // top yellow part is 16 px height
int clockRadius = 23;

String timeLastRing = "";
String timeclockstring = "";
String lastRingString = "";
String ringStateString = "";

int numRings = 0;

#ifndef RF_MODULE_FREQUENCY
#  define RF_MODULE_FREQUENCY 433.92
#endif

#define JSON_MSG_BUFFER 512

char messageBuffer[JSON_MSG_BUFFER];
char messageBufferDisp[JSON_MSG_BUFFER];

rtl_433_ESP rf; // use -1 to disable transmitter

int count = 0;


double timer_sec = 0;
double startTime = 0;

bool doorbell_ringing = false;
bool doorbell_lockout = false;

int doorbell_counter_max = 2000;
int doorbell_counter = 0;
int doorbell_milli_start = 0;
int doorbell_lockout_counter_max = 5000;
bool test_code = false;

void resetTimer()
{
  startTime = millis();
  timer_sec = 0;
}

void updateTimer()
{
  timer_sec = (millis() - startTime)/1000.0f;
}

void ringDoorbell()
{
if(doorbell_lockout == false)
  {
  doorbell_ringing = true;
  doorbell_milli_start = millis();
  numRings++;
  timeLastRing = timeclockstring;
  lastRingString = String("Num Rings:") + String(numRings) + "\n Prev: " + timeLastRing;
  }
}


void rtl_433_Callback(char* message) {
  DynamicJsonBuffer jsonBuffer2(JSON_MSG_BUFFER);
  JsonObject& RFrtl_433_ESPdata = jsonBuffer2.parseObject(message);
  ringDoorbell();
  logJson(RFrtl_433_ESPdata);
  updateDisplay(RFrtl_433_ESPdata);
  resetTimer();

  count++;
}
void updateDisplay(JsonObject& jsondata) {


jsondata.printTo(messageBufferDisp,jsondata.measureLength() + 1);


}

void logJson(JsonObject& jsondata) {
#if defined(ESP8266) || defined(ESP32) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
  char JSONmessageBuffer[jsondata.measureLength() + 1];
#else
  char JSONmessageBuffer[JSON_MSG_BUFFER];
#endif
  jsondata.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
#if defined(setBitrate) || defined(setFreqDev) || defined(setRxBW)
  Log.setShowLevel(false);
  Log.notice(F("."));
  Log.setShowLevel(true);
#else
  Log.notice(F("Received message : %s" CR), JSONmessageBuffer);
#endif
}

// utility function for digital clock display: prints leading 0
String twoDigits(int digits) {
  if (digits < 10) {
    String i = '0' + String(digits);
    return i;
  }
  else {
    return String(digits);
  }
}

void jsonOverlay(OLEDDisplay *display, OLEDDisplayUiState* state)
{
}

void jsonFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  
 // char tmpjson[] = "'model':'Oregon-CM180i','id':128,'battery_ok':1,'power1_W':48,'power2_W':9289,'power3_W':8243,'sequence':9,'protocol':'Oregon Scientific Weather Sensor','rssi':-87,'duration':288001'";
  String jsonnow = "";

int lineLength = 20;

  for(int i = 0; i<sizeof(messageBufferDisp); i++)
  {
    if((i%lineLength) == 0)
    {

      jsonnow.concat("\n");
     
    }

      jsonnow.concat(messageBufferDisp[i]);
  
  }


  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  display->drawString(clockCenterX + x , clockCenterY + y, jsonnow);
}

void countOverlay(OLEDDisplay *display, OLEDDisplayUiState* state)
{
}

void countFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
 
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_10);
  display->drawString(clockCenterX + x , clockCenterY + y,ringStateString + "\n" + lastRingString + "\n" + timeclockstring);


}

void clockOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {

}

void updateClock()
{
  timeclockstring = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
}
void digitalClockFrame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String timenow = timeclockstring;
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24);
  display->drawString(clockCenterX + x , clockCenterY + y, timenow );
}


// This array keeps function pointers to all frames
// frames are the single views that slide in
//FrameCallback frames[] = { jsonFrame };
FrameCallback frames[] = { countFrame };

// how many frames are there?
int frameCount = 1;

// Overlays are statically drawn on top of a frame eg. a clock
//OverlayCallback overlays[] = { jsonOverlay };
OverlayCallback overlays[] = { countOverlay };

int overlaysCount = 1;


void setup() {
  pinMode(PINOUT_DOORBELL, OUTPUT);
  digitalWrite(PINOUT_DOORBELL,LOW);

  resetTimer();
  Serial.begin(921600);
  delay(1000);
#ifndef LOG_LEVEL
  LOG_LEVEL_SILENT
#endif
  Log.begin(LOG_LEVEL, &Serial);
  Log.notice(F(" " CR));
  Log.notice(F("****** setup ******" CR));
  rf.initReceiver(RF_MODULE_RECEIVER_GPIO, RF_MODULE_FREQUENCY);
  rf.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER);
  rf.enableReceiver();
  Log.notice(F("****** setup complete ******" CR));
  rf.getModuleStatus();
 
  ui.setTargetFPS(60);
  ui.setActiveSymbol(activeSymbol);
  //ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);
  ui.disableAutoTransition();

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();
  ui.disableAllIndicators();

  display.flipScreenVertically();
}

unsigned long uptime() {
  static unsigned long lastUptime = 0;
  static unsigned long uptimeAdd = 0;
  unsigned long uptime = millis() / 1000 + uptimeAdd;
  if (uptime < lastUptime) {
    uptime += 4294967;
    uptimeAdd += 4294967;
  }
  lastUptime = uptime;
  return uptime;
}

int next = uptime() + 30;

#if defined(setBitrate) || defined(setFreqDev) || defined(setRxBW)

#  ifdef setBitrate
#    define TEST    "setBitrate" // 17.24 was suggested
#    define STEP    2
#    define stepMin 1
#    define stepMax 300
// #    define STEP    1
// #    define stepMin 133
// #    define stepMax 138
#  elif defined(setFreqDev) // 40 kHz was suggested
#    define TEST    "setFrequencyDeviation"
#    define STEP    1
#    define stepMin 5
#    define stepMax 200
#  elif defined(setRxBW)
#    define TEST "setRxBandwidth"

#    ifdef defined(RF_SX1276) || defined(RF_SX1278)
#      define STEP    5
#      define stepMin 5
#      define stepMax 250
#    else
#      define STEP    5
#      define stepMin 58
#      define stepMax 812
// #      define STEP    0.01
// #      define stepMin 202.00
// #      define stepMax 205.00
#    endif
#  endif
float step = stepMin;
#endif

void loop() {

  updateClock();

  if(doorbell_ringing==true)
  {
    ringStateString = "Doorbell Ringing";

    digitalWrite(PINOUT_DOORBELL,HIGH);
    Serial.print("1 - doorbell ringing = true \n");
    doorbell_counter = millis() - doorbell_milli_start;
    if(doorbell_counter>doorbell_counter_max)
    {
      Serial.print("2 - doorbell done ringing \n");
        doorbell_ringing = false;
        doorbell_lockout = true;

        doorbell_milli_start = millis();
        digitalWrite(PINOUT_DOORBELL,LOW);
    }
  }
  else if(doorbell_lockout == true)
  {
        digitalWrite(PINOUT_DOORBELL,LOW);

    ringStateString = "Doorbell Lockout";

      Serial.print("3 - doorbell lockout = true \n");
      doorbell_counter = millis() - doorbell_milli_start;
      if(doorbell_counter > doorbell_lockout_counter_max)
      {
        Serial.print("4 - doorbell lockout = false \n");
        doorbell_lockout = false;
      }
  }
  else if(test_code == true)
  {
    Serial.print("5 - Test Code True \n");
    doorbell_lockout_counter_max = 5000;
    doorbell_counter_max = 5000;

    if(doorbell_lockout==false){
    ringDoorbell();}
  }
  else{
        ringStateString = "Doorbell Armed";
  }

  rf.loop();
  updateTimer();
  
  if(timer_sec > 30.0f)
  {
    char tmpBuffer[JSON_MSG_BUFFER] = "";

      memcpy(messageBufferDisp,tmpBuffer,JSON_MSG_BUFFER*sizeof(char));
  }

ui.update();

#if defined(setBitrate) || defined(setFreqDev) || defined(setRxBW)
  char stepPrint[8];
  if (uptime() > next) {
    next = uptime() + 120; // 60 seconds
    dtostrf(step, 7, 2, stepPrint);
    Log.notice(F(CR "Finished %s: %s, count: %d" CR), TEST, stepPrint, count);
    step += STEP;
    if (step > stepMax) {
      step = stepMin;
    }
    dtostrf(step, 7, 2, stepPrint);
    Log.notice(F("Starting %s with %s" CR), TEST, stepPrint);
    count = 0;

    int16_t state = 0;
#  ifdef setBitrate
    state = rf.setBitRate(step);
    RADIOLIB_STATE(state, TEST);
#  elif defined(setFreqDev)
    state = rf.setFrequencyDeviation(step);
    RADIOLIB_STATE(state, TEST);
#  elif defined(setRxBW)
    state = rf.setRxBandwidth(step);
    if ((state) != RADIOLIB_ERR_NONE) {
      Log.notice(F(CR "Setting  %s: to %s, failed" CR), TEST, stepPrint);
      next = uptime() - 1;
    }
#  endif

    rf.receiveDirect();
    // rf.getModuleStatus();
  }
#endif
}