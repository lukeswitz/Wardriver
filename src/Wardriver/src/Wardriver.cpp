#include "Arduino.h"
#include "Wardriver.h"
#include "Recon.h"
#include "Screen.h"
#include "graphics.h"
#include "driver/temp_sensor.h"

#include <TinyGPSPlus.h>
#include "../Vars.h"
#include "Filesys.h"

#if defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial SERIAL_VAR(GPS_RX, GPS_TX);  // RX, TX
#endif

TinyGPSPlus gps;

// SD Config
bool sdDedupe;
bool sdDynamicScan;
bool sdShowHidden;
int sdTimePerChan;

// CURRENT GPS & DATETIME STRING
float lat = 0;
float lng = 0;
int alt;
double hdop;
char strDateTime[30];
char currentGPS[20] = "...";

// RECON PARAMS
const int MAX_MACS = 150;
uint32_t totalNets = 0;
uint8_t clients = 0;
uint8_t openNets = 0;
uint8_t sats = 0;
uint8_t bat = 0;
uint8_t speed = 0;

// DYNAMIC SCAN VARS
const int popularChannels[] = { 1, 6, 11 };
const int standardChannels[] = { 2, 3, 4, 5, 7, 8, 9, 10 };
int timePerChannel[14] = { 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 50, 50, 50 };

// DASH ICONS
char satsC[4] = "...";
char totalC[4] = "...";
char openNetsC[4] = "...";
char tmpC[4] = "...";
char batC[4] = "...";
char speedC[4] = "...";

// CURRENT DATETIME
uint8_t hr;
uint8_t mn;
uint8_t sc;
uint16_t yr;
uint8_t mt;
uint8_t dy;
char currTime[10] = "...";

char* test[6] = { satsC, totalC, openNetsC, tmpC, batC, speedC };

Wardriver::Wardriver() {}

uint8_t getBattery() {
  return 0;
}

uint8_t getTemp() {
  float result = 0;
  temp_sensor_read_celsius(&result);
  return (int)result;
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (SERIAL_VAR.available())
      gps.encode(SERIAL_VAR.read());
  } while (millis() - start < ms);
}

bool isMACSeen(const char* mac, char seenMACs[][20], int& macCount) {
  for (int i = 0; i < macCount; i++) {
    if (strcmp(seenMACs[i], mac) == 0) {
      return true;
    }
  }
  if (macCount < MAX_MACS) {
    strcpy(seenMACs[macCount++], mac);
  }
  return false;
}

bool findInArray(int value, const int* array, int size) {
  for (int i = 0; i < size; i++) {
    if (array[i] == value) return true;
  }
  return false;
}

// Dynamic scan
void updateTimePerChannel(int channel, int networksFound) {
  const int FEW_NETWORKS_THRESHOLD = 1;
  const int MANY_NETWORKS_THRESHOLD = 5;
  const int POPULAR_TIME_INCREMENT = 50;    // Higher increment for popular channels
  const int STANDARD_TIME_INCREMENT = 100;  // Standard increment
  const int MAX_TIME = 400;
  const int MIN_TIME = 50;

  int timeIncrement;

  // Determine the time increment based on channel type
  if (findInArray(channel, popularChannels, sizeof(popularChannels) / sizeof(popularChannels[0]))) {
    timeIncrement = POPULAR_TIME_INCREMENT;
  } else {
    timeIncrement = STANDARD_TIME_INCREMENT;
  }

  // Adjust the time per channel based on the number of networks found
  if (networksFound >= MANY_NETWORKS_THRESHOLD) {
    timePerChannel[channel - 1] += timeIncrement;
    if (timePerChannel[channel - 1] > MAX_TIME) {
      timePerChannel[channel - 1] = MAX_TIME;
    }
  } else if (networksFound <= FEW_NETWORKS_THRESHOLD) {
    timePerChannel[channel - 1] -= timeIncrement;
    if (timePerChannel[channel - 1] < MIN_TIME) {
      timePerChannel[channel - 1] = MIN_TIME;
    }
  }
}

void updateGPS() {
  lat = gps.location.lat();
  lng = gps.location.lng();
  alt = (int)gps.altitude.meters();
  hdop = gps.hdop.hdop();
  sats = gps.satellites.value();
  speed = gps.speed.mph();

  yr = gps.date.year();
  mt = gps.date.month();
  dy = gps.date.day();

  hr = gps.time.hour();
  mn = gps.time.minute();
  sc = gps.time.second();

  sprintf(strDateTime, "%04d-%02d-%02d %02d:%02d:%02d", yr, mt, dy, hr, mn, sc);
  sprintf(currentGPS, "%1.3f,%1.3f", lat, lng);
  sprintf(currTime, "%02d:%02d", hr, mn);
  sprintf(satsC, "%u", sats);

  if (totalNets > 999) {
    sprintf(totalC, "%gK", ((totalNets) / 100) / 10.0);
  } else {
    sprintf(totalC, "%u", totalNets);
  }

  sprintf(openNetsC, "%u", openNets);
  uint8_t tmpTemp = getTemp();
  sprintf(tmpC, "%u", tmpTemp);
  sprintf(batC, "%u", getBattery());
  sprintf(speedC, "%u", speed);

  //Screen::setFooter("GPS: UPDATED");
  Screen::update();
}



void initGPS() {
#if defined(ESP32)
  SERIAL_VAR.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
#elif defined(ESP8266)
  SERIAL_VAR.begin(GPS_BAUD);
#endif

  Screen::setFooter("GPS Initializing...");
  Screen::update();

  unsigned long startGPSTime = millis();
  bool gpsFound = false;

  while (!gpsFound) {
    if (millis() - startGPSTime > 10000 && gps.charsProcessed() < 10) {
      Screen::setFooter("GPS: NOT FOUND");
      Screen::update();
      sats = gps.satellites.value();
      Serial.println(gps.location.isValid());
      yield();
      smartDelay(500);
    }

    if (gps.location.isValid()) {
      gpsFound = true;
      Screen::setFooter("GPS: LOCATION FOUND");
      updateGPS();
    } else {
      Screen::setFooter("GPS: Validating time...");
      yield();
      smartDelay(500);
      Serial.print("Year: ");
      Serial.println(gps.date.year());
    }
  }
}


void scanNets() {
  static char seenMACs[MAX_MACS][20];
  static int macCount = 0;

  char entry[250];
  Serial.println("[ ] Scanning WiFi networks...");
  Screen::setFooter("WiFi: Scanning...");
  Screen::update();

  Filesys::open();
  int numNets = 0;  // Reset at the start of each scan call to count unique networks this scan.

  auto processNetworks = [&](int networksFound) {
    for (int i = 0; i < networksFound; i++) {
      const char* mac = WiFi.BSSIDstr(i).c_str();
      if (!isMACSeen(mac, seenMACs, macCount)) {  // Process only if MAC not seen.
        const char* authType = getAuthType(WiFi.encryptionType(i));

        if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) openNets++;

        sprintf(entry, "%s,\"%s\",%s,%s,%u,%i,%f,%f,%i,%f,WIFI", mac, WiFi.SSID(i).c_str(), authType, strDateTime, WiFi.channel(i), WiFi.RSSI(i), lat, lng, alt, hdop);

        // Serial.println(entry);
        Filesys::write(entry);
        totalNets++;  // Increment only for unique SSIDs.
        numNets++;    // Counting unique networks found in this scan.
      }
    }
  };

  if (sdDynamicScan) {
    for (int channel = 1; channel <= 11; channel++) {
      int networksOnChannel = WiFi.scanNetworks(false, sdTimePerChan, false, timePerChannel[channel - 1], channel);
      processNetworks(networksOnChannel);
      updateTimePerChannel(channel, networksOnChannel);
    }
  } else {
    int networksFound = WiFi.scanNetworks(false, sdShowHidden, false, sdTimePerChan);
    processNetworks(networksFound);
  }

  char message[30];
  sprintf(message, "Logged %d New Networks", numNets);
  Screen::setFooter(message);
  Screen::update();

  Filesys::close();
  WiFi.scanDelete();
}

void Wardriver::init() {
  sdDedupe = Filesys::dedupe;
  sdDynamicScan = Filesys::dynamicScan;
  sdShowHidden = Filesys::showHidden;
  sdTimePerChan = Filesys::timePerChan;

  totalNets = 0;
  temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
  temp_sensor.dac_offset = TSENS_DAC_L2;  // TSENS_DAC_L2 is default; L4(-40°C ~ 20°C), L2(-10°C ~ 80°C), L1(20°C ~ 100°C), L0(50°C ~ 125°C)
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();

  Screen::init();
  Screen::drawSplash(2);
  Screen::setIcons(icons_bits, test, 6);
  Screen::setHeader(currentGPS, currTime);

  Filesys::init(updateScreen);
  delay(1000);

  initGPS();
  char fileDT[150];
  sprintf(fileDT, "%i-%02d-%02d", yr, mt, dy);
  delay(1000);
  Filesys::createLog(fileDT, updateScreen);
}

void Wardriver::updateScreen(char* message) {
  Screen::setFooter(message);
  Screen::update();
}

void Wardriver::scan() {
  Serial.println("In scan.");
  updateGPS();  // poll current GPS coordinates
  scanNets();   // scan WiFi nets
  smartDelay(150);
}
