#include "HardwareSerial.h"
#include "Arduino.h"
#include "Wardriver.h"
#include "Recon.h"
#include "Screen.h"

#include <TinyGPSPlus.h>
#include "../Vars.h"
#include "Filesys.h"

const int MAX_MACS = 30;
const int MAX_ENTRY_SIZE = 250;
char ssidBuffer[MAX_MACS][MAX_ENTRY_SIZE];
int ssidIndex = 0;

#if defined(ESP8266)
    #include <SoftwareSerial.h>
    SoftwareSerial SERIAL_VAR(GPS_RX, GPS_TX); // RX, TX
#endif

TinyGPSPlus gps;

// CURRENT GPS & DATTIME STRING
float lat = 0; float lng = 0;
int alt; double hdop;
char strDateTime[20];
char currentGPS[17];

// RECON PARAMS
uint32_t totalNets = 0;
uint8_t  clients = 0;
uint8_t  openNets = 0;
uint8_t  sats = 0;
uint8_t  bat = 0;
uint8_t  speed = 0;

// CURRENT DATETIME
uint8_t hr; uint8_t mn; uint8_t sc;
uint16_t yr; uint8_t mt; uint8_t dy;
char currTime[9];

Wardriver::Wardriver() {

}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (SERIAL_VAR.available())
      gps.encode(SERIAL_VAR.read());
  } while (millis() - start < ms);
}

void updateGPS() {
    lat = gps.location.lat(); 
    lng = gps.location.lng();
    alt = (int) gps.altitude.meters();
    hdop = gps.hdop.hdop();
    sats = gps.satellites.value();
    speed = gps.speed.mph();

    yr = gps.date.year();
    mt = gps.date.month();
    dy = gps.date.day();

    hr = gps.time.hour();
    mn = gps.time.minute();
    sc = gps.time.second();

    sprintf(strDateTime,"%04d-%02d-%02d %02d:%02d:%02d",yr,mt,dy,hr,mn,sc);
    sprintf(currentGPS,"%1.3f,%1.3f",lat,lng);
    sprintf(currTime,"%02d:%02d",hr,mn);

   // Screen::drawMockup(currentGPS,currTime,sats,totalNets,openNets,clients,bat,speed,"GPS: UPDATED");
}

void updateGPS(uint8_t override) {
    lat = 35.8715; 
    lng = -105.2730;
    alt = 220; hdop = 1.5;
    sats = 3; speed = 69;

    yr = 2023; mt = 8; dy = 11;
    hr = 10; mn = 36; sc = 56;

    sprintf(strDateTime,"%04d-%02d-%02d %02d:%02d:%02d",yr,mt,dy,hr,mn,sc);
    sprintf(currentGPS,"%1.3f,%1.3f",lat,lng);
    sprintf(currTime,"%02d:%02d",hr,mn);

  //  Screen::drawMockup(currentGPS,currTime,sats,totalNets,openNets,clients,bat,speed,"GPS: UPDATED");
}

// initialize GPS & get first coords
void initGPS() {

    #if defined(ESP32)
        SERIAL_VAR.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    #elif defined(ESP8266)
        SERIAL_VAR.begin(GPS_BAUD);
    #endif

    Screen::drawMockup("...","...",sats,totalNets,openNets,clients,bat,speed,"GPS: Initializing...");

    unsigned long startGPSTime = millis();
    while (! (gps.location.isValid())) {
        if (millis()-startGPSTime > 5000 && gps.charsProcessed() < 10) {
            Screen::drawMockup("...","...",sats,totalNets,openNets,clients,bat,speed,"GPS: NOT FOUND");
        }
        else if (gps.charsProcessed() > 10) {
            Screen::drawMockup("...","...",sats,totalNets,openNets,clients,bat,speed,"GPS: Waiting for fix...");
        }
        sats = gps.satellites.value();
        
        Serial.println(gps.location.isValid());
        delay(0); smartDelay(500);
    }
    while (!(gps.date.year() == 2023) && hdop > 20) {
        Screen::drawMockup("...","...",sats,totalNets,openNets,clients,bat,speed,"GPS: Calibrating...");
        delay(0); smartDelay(500);
    }
    Screen::drawMockup("...","...",sats,totalNets,openNets,clients,bat,speed,"GPS: LOCATION FOUND");

    updateGPS();
}

void initGPS(uint8_t override) {
    #if defined(ESP32)
        SERIAL_VAR.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    #endif
    Screen::drawMockup("...","...",0,0,0,0,0,0,"GPS: Waiting for fix");
    delay(500);

    updateGPS(0);
    Screen::drawMockup(currentGPS,currTime,sats,totalNets,openNets,clients,bat,speed,"GPS: LOCATION FOUND");
}
void addSSID(const char* ssid) {
    strncpy(ssidBuffer[ssidIndex % MAX_MACS], ssid, MAX_ENTRY_SIZE - 1);
    ssidBuffer[ssidIndex % MAX_MACS][MAX_ENTRY_SIZE - 1] = '\0';
    ssidIndex++;
}

bool isSSIDSeen(const char* ssid) {
    for (int j = 0; j < MAX_MACS; j++) {
        if (strncmp(ssidBuffer[j], ssid, MAX_ENTRY_SIZE) == 0) {
            return true;
        }
    }
    return false;
}

void scanNets() {
    char entry[250];
    char message[30];
    
    // Buffer and index for SSIDs
    static String ssidBuffer[MAX_MACS];
    static int ssidIndex = 0;
    int newNets = 0;
    
    Filesys::open();
    
    //Serial.println("[ ] Scanning WiFi networks...");
    Screen::drawMockup(currentGPS, currTime, sats, totalNets, openNets, clients, bat, speed, "WiFi: Scanning...");

    int n = WiFi.scanNetworks();
    //openNets = 0; //TOFIX: SHOW TOTAL NOT THE CURRENT OPEN FOR NOW

    for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        int entryLength = ssid.length() + MAX_ENTRY_SIZE;
        char entry[entryLength];

        if (!isSSIDSeen(ssid.c_str())) {
            addSSID(ssid.c_str());
            char* authType = getAuthType(WiFi.encryptionType(i));
            #if defined(ESP8266)
                if (WiFi.encryptionType(i) == ENC_TYPE_NONE) openNets++;
            #elif defined(ESP32)
                if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) openNets++;
            #endif
            snprintf(entry, sizeof(entry), "%s,%s,%s,%s,%u,%i,%f,%f,%i,%f,WIFI", WiFi.BSSIDstr(i).c_str(), ssid.c_str(), authType, strDateTime, WiFi.channel(i), WiFi.RSSI(i), lat, lng, alt, hdop);
            newNets++;
            Filesys::write(entry);
            Serial.println(entry); 
        }
    }
    
    // sprintf(message, "Logged %d networks.", newNets);
    // Screen::drawMockup(currentGPS, currTime, sats, totalNets, openNets, clients, bat, speed, message);

    totalNets += newNets;
    Filesys::close();
    WiFi.scanDelete();
}

void getBattery() {
    float analogVal = analogRead(A0);
    // bat = map(analogVal,0,100);
    bat = 0;
}

void Wardriver::init() {
    Screen::init();
    Screen::drawSplash(2);
    Filesys::init(updateScreen); delay(1000);
    
    getBattery();
    initGPS();
    
    char filename[23]; sprintf(filename,"%i-%02d-%02d",yr, mt, dy);
    Filesys::createLog(filename, updateScreen);
}

void Wardriver::updateScreen(char* message) {
    Screen::drawMockup(currentGPS,currTime,sats,totalNets,openNets,clients,bat,speed,message);
}

void Wardriver::scan() {
    updateGPS(); // poll current GPS coordinates
  //  getBattery();
    scanNets(); // scan WiFi nets
    smartDelay(500);
}

