
#if !( defined(ESP8266) ||  defined(ESP32) )
  #error This code is intended to run on the ESP8266 or ESP32 platform! Please check your Tools->Board setting.
#endif

#define ESP_WIFIMANAGER_VERSION_MIN_TARGET     "ESP_WiFiManager v1.7.2"

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _WIFIMGR_LOGLEVEL_    4

#include <FS.h>

//Ported to ESP32
#ifdef ESP32
  #include <esp_wifi.h>
  #include <WiFi.h>
  #include <WiFiClient.h>
  
  // From v1.1.0
  #include <WiFiMulti.h>
  WiFiMulti wifiMulti;

  // LittleFS has higher priority than SPIFFS
  #if ( ARDUINO_ESP32C3_DEV )
    // Currently, ESP32-C3 only supporting SPIFFS and EEPROM. Will fix to support LittleFS
    #define USE_LITTLEFS          false
    #define USE_SPIFFS            true
  #else
    #define USE_LITTLEFS    true
    #define USE_SPIFFS      false
  #endif

  #if USE_LITTLEFS
    // Use LittleFS
    #include "FS.h"

    // The library has been merged into esp32 core release 1.0.6
     #include <LITTLEFS.h>             // https://github.com/lorol/LITTLEFS
    
    FS* filesystem =      &LITTLEFS;
    #define FileFS        LITTLEFS
    #define FS_Name       "LittleFS"
  #elif USE_SPIFFS
    #include <SPIFFS.h>
    FS* filesystem =      &SPIFFS;
    #define FileFS        SPIFFS
    #define FS_Name       "SPIFFS"
  #else
    // Use FFat
    #include <FFat.h>
    FS* filesystem =      &FFat;
    #define FileFS        FFat
    #define FS_Name       "FFat"
  #endif
    //////
    
    #define ESP_getChipId()   ((uint32_t)ESP.getEfuseMac())

#define LED_BUILTIN       2
#define LED_ON            HIGH
#define LED_OFF           LOW

#else
  #include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
  //needed for library
  #include <DNSServer.h>
  #include <ESP8266WebServer.h>
  
  // From v1.1.0
  #include <ESP8266WiFiMulti.h>
  ESP8266WiFiMulti wifiMulti;
  
  #define USE_LITTLEFS      true
  
  #if USE_LITTLEFS
    #include <LittleFS.h>
    FS* filesystem =      &LittleFS;
    #define FileFS        LittleFS
    #define FS_Name       "LittleFS"
  #else
    FS* filesystem =      &SPIFFS;
    #define FileFS        SPIFFS
    #define FS_Name       "SPIFFS"
  #endif
  //////
  
  #define ESP_getChipId()   (ESP.getChipId())
  
  #define LED_ON      LOW
  #define LED_OFF     HIGH
#endif

// These defines must be put before #include <ESP_DoubleResetDetector.h>
// to select where to store DoubleResetDetector's variable.
// For ESP32, You must select one to be true (EEPROM or SPIFFS)
// For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
// Otherwise, library will use default EEPROM storage
#ifdef ESP32

  // These defines must be put before #include <ESP_DoubleResetDetector.h>
  // to select where to store DoubleResetDetector's variable.
  // For ESP32, You must select one to be true (EEPROM or SPIFFS)
  // Otherwise, library will use default EEPROM storage
  #if USE_LITTLEFS
    #define ESP_DRD_USE_LITTLEFS    true
    #define ESP_DRD_USE_SPIFFS      false
    #define ESP_DRD_USE_EEPROM      false
  #elif USE_SPIFFS
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      true
    #define ESP_DRD_USE_EEPROM      false
  #else
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      false
    #define ESP_DRD_USE_EEPROM      true
  #endif

#else //ESP8266

  // For DRD
  // These defines must be put before #include <ESP_DoubleResetDetector.h>
  // to select where to store DoubleResetDetector's variable.
  // For ESP8266, You must select one to be true (RTC, EEPROM, SPIFFS or LITTLEFS)
  // Otherwise, library will use default EEPROM storage
  #if USE_LITTLEFS
    #define ESP_DRD_USE_LITTLEFS    true
    #define ESP_DRD_USE_SPIFFS      false
  #else
    #define ESP_DRD_USE_LITTLEFS    false
    #define ESP_DRD_USE_SPIFFS      true
  #endif
  
  #define ESP_DRD_USE_EEPROM      false
  #define ESP8266_DRD_USE_RTC     false
#endif

#define DOUBLERESETDETECTOR_DEBUG       true  //false

#include <ESP_DoubleResetDetector.h>      //https://github.com/khoih-prog/ESP_DoubleResetDetector

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

//DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
DoubleResetDetector* drd;//////

// Onboard LED I/O pin on NodeMCU board
//const int PIN_LED = 2; // D4 on NodeMCU and WeMos. GPIO2/ADC12 of ESP32. Controls the onboard LED.

// SSID and PW for Config Portal
String ssid       = "ESP_" + String(ESP_getChipId(), HEX);
String password;

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

// From v1.1.0
// You only need to format the filesystem once
//#define FORMAT_FILESYSTEM       true
#define FORMAT_FILESYSTEM         false

#define MIN_AP_PASSWORD_SIZE    8

#define SSID_MAX_LEN            32
//From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN            64

typedef struct
{
  char wifi_ssid[SSID_MAX_LEN];
  char wifi_pw  [PASS_MAX_LEN];
}  WiFi_Credentials;

typedef struct
{
  String wifi_ssid;
  String wifi_pw;
}  WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS      2

// Assuming max 491 chars
#define TZNAME_MAX_LEN            50
#define TIMEZONE_MAX_LEN          50

typedef struct
{
  WiFi_Credentials  WiFi_Creds [NUM_WIFI_CREDENTIALS];
  char TZ_Name[TZNAME_MAX_LEN];     // "America/Toronto"
  char TZ[TIMEZONE_MAX_LEN];        // "EST5EDT,M3.2.0,M11.1.0"
  uint16_t checksum;
} WM_Config;

WM_Config         WM_config;

#define  CONFIG_FILENAME              F("/wifi_cred.dat")
//////

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES     true    //false

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false

// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP     true

// Just use enough to save memory. On ESP8266, can cause blank ConfigPortal screen
// if using too much memory
#define USING_AFRICA        false
#define USING_AMERICA       false
#define USING_ANTARCTICA    false
#define USING_ASIA          false
#define USING_ATLANTIC      false
#define USING_AUSTRALIA     false
#define USING_EUROPE        true
#define USING_INDIAN        false
#define USING_PACIFIC       false
#define USING_ETC_GMT       false

// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP          false

// New in v1.0.11
#define USING_CORS_FEATURE          true
//////

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
// Force DHCP to be true
#if defined(USE_DHCP_IP)
#undef USE_DHCP_IP
#endif
#define USE_DHCP_IP     true
#else
// You can select DHCP or Static IP here
#define USE_DHCP_IP     true
//#define USE_DHCP_IP     false
#endif

#if ( USE_DHCP_IP )
// Use DHCP
#warning Using DHCP IP
IPAddress stationIP   = IPAddress(0, 0, 0, 0);
IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
#else
// Use static IP
#warning Using static IP
#ifdef ESP32
IPAddress stationIP   = IPAddress(192, 168, 2, 232);
#else
IPAddress stationIP   = IPAddress(192, 168, 2, 186);
#endif

IPAddress gatewayIP   = IPAddress(192, 168, 2, 1);
IPAddress netMask     = IPAddress(255, 255, 255, 0);
#endif

#define USE_CONFIGURABLE_DNS      true

IPAddress dns1IP      = gatewayIP;
IPAddress dns2IP      = IPAddress(8, 8, 8, 8);

#define USE_CUSTOM_AP_IP          false

// New in v1.4.0
IPAddress APStaticIP  = IPAddress(192, 168, 232, 1);
IPAddress APStaticGW  = IPAddress(192, 168, 232, 1);
IPAddress APStaticSN  = IPAddress(255, 255, 255, 0);

#include <ESP_WiFiManager.h>              //https://github.com/khoih-prog/ESP_WiFiManager

// Function Prototypes
uint8_t connectMultiWiFi();



WiFi_AP_IPConfig  WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig)
{
  in_WM_AP_IPconfig._ap_static_ip   = APStaticIP;
  in_WM_AP_IPconfig._ap_static_gw   = APStaticGW;
  in_WM_AP_IPconfig._ap_static_sn   = APStaticSN;
}

void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig)
{
  in_WM_STA_IPconfig._sta_static_ip   = stationIP;
  in_WM_STA_IPconfig._sta_static_gw   = gatewayIP;
  in_WM_STA_IPconfig._sta_static_sn   = netMask;
#if USE_CONFIGURABLE_DNS  
  in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, ", gatewayIP =", in_WM_STA_IPconfig._sta_static_gw);
  LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
  LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, ", dns2IP =", in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
  #if USE_CONFIGURABLE_DNS  
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);  
  #else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
  #endif 
}

///////////////////////////////////////////

uint8_t connectMultiWiFi()
{
#if ESP32
  // For ESP32, this better be 0 to shorten the connect time.
  // For ESP32-S2/C3, must be > 500
  #if ( USING_ESP32_S2 || USING_ESP32_C3 )
    #define WIFI_MULTI_1ST_CONNECT_WAITING_MS           500L
  #else
    // For ESP32 core v1.0.6, must be >= 500
    #define WIFI_MULTI_1ST_CONNECT_WAITING_MS           800L
  #endif
#else
  // For ESP8266, this better be 2200 to enable connect the 1st time
  #define WIFI_MULTI_1ST_CONNECT_WAITING_MS             2200L
#endif

#define WIFI_MULTI_CONNECT_WAITING_MS                   500L

  uint8_t status;

  WiFi.mode(WIFI_STA);

  LOGERROR(F("ConnectMultiWiFi with :"));

  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass = "), Router_Pass );
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass );
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
    }
  }

  LOGERROR(F("Connecting MultiWifi..."));

  //WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP
  // New in v1.4.0
  configWiFi(WM_STA_IPconfig);
  //////
#endif

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) )
  {
    status = wifiMulti.run();

    if ( status == WL_CONNECTED )
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if ( status == WL_CONNECTED )
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP() );
  }
  else
  {
    LOGERROR(F("WiFi not connected"));

    // To avoid unnecessary DRD
    drd->loop();
  
#if ESP8266      
    ESP.reset();
#else
    ESP.restart();
#endif  
  }

  return status;
}

#if USE_ESP_WIFIMANAGER_NTP

void printLocalTime()
{
#if ESP8266
  static time_t now;
  
  now = time(nullptr);
  
  if ( now > 1451602800 )
  {
    //Serial.print("Local Date/Time: ");
    //Serial.print(ctime(&now));
  }
#else
  struct tm timeinfo;

  getLocalTime( &timeinfo );

  // Valid only if year > 2000. 
  // You can get from timeinfo : tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec
  if (timeinfo.tm_year > 100 )
  {
    //Serial.print("Local Date/Time: ");
    //Serial.print( asctime( &timeinfo ) );
  }
#endif
}

#endif

void heartBeatPrint()
{
#if USE_ESP_WIFIMANAGER_NTP
  printLocalTime();
#else
  static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    //Serial.print(F("H"));        // H means connected to WiFi
  else
    //Serial.print(F("F"));        // F means not connected to WiFi

  if (num == 80)
  {
    //Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    //Serial.print(F(" "));
  }
#endif  
}

void check_WiFi()
{
  if ( (WiFi.status() != WL_CONNECTED) )
  {
    //Serial.println(F("\nWiFi lost. Call connectMultiWiFi in loop"));
    connectMultiWiFi();
  }
}

void check_status()
{
  static ulong checkstatus_timeout  = 0;
  static ulong checkwifi_timeout    = 0;

  static ulong current_millis;

#define WIFICHECK_INTERVAL    1000L

#if USE_ESP_WIFIMANAGER_NTP
  #define HEARTBEAT_INTERVAL    60000L
#else
  #define HEARTBEAT_INTERVAL    10000L
#endif

  current_millis = millis();

  // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
  if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
  {
    check_WiFi();
    checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
  }

  // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
  if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
  {
    heartBeatPrint();
    checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
  }
}

int calcChecksum(uint8_t* address, uint16_t sizeToCalc)
{
  uint16_t checkSum = 0;
  
  for (uint16_t index = 0; index < sizeToCalc; index++)
  {
    checkSum += * ( ( (byte*) address ) + index);
  }

  return checkSum;
}

bool loadConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "r");
  LOGERROR(F("LoadWiFiCfgFile "));

  memset((void*) &WM_config,       0, sizeof(WM_config));

  // New in v1.4.0
  memset((void*) &WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));
  //////

  if (file)
  {
    file.readBytes((char *) &WM_config,   sizeof(WM_config));

    // New in v1.4.0
    file.readBytes((char *) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));

    if ( WM_config.checksum != calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) ) )
    {
      LOGERROR(F("WM_config checksum wrong"));
      
      return false;
    }
    
    // New in v1.4.0
    displayIPConfigStruct(WM_STA_IPconfig);
    //////

    return true;
  }
  else
  {
    LOGERROR(F("failed"));

    return false;
  }
}

void saveConfigData()
{
  File file = FileFS.open(CONFIG_FILENAME, "w");
  LOGERROR(F("SaveWiFiCfgFile "));

  if (file)
  {
    WM_config.checksum = calcChecksum( (uint8_t*) &WM_config, sizeof(WM_config) - sizeof(WM_config.checksum) );
    
    file.write((uint8_t*) &WM_config, sizeof(WM_config));

    displayIPConfigStruct(WM_STA_IPconfig);

    // New in v1.4.0
    file.write((uint8_t*) &WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
    //////

    file.close();
    LOGERROR(F("OK"));
  }
  else
  {
    LOGERROR(F("failed"));
  }
}
WiFiClient wifiClient;
//#include <DNSServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#define FIRMWARE_VERSION "1.0"

const int PIN_LED = 5;  // COM
const int PIN_LED2 = 0; // SRV
const int PIN_LED4 = 4; // NET

char *strings[100]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;

long lastSendVolt;
#include <SoftwareSerial.h>       // Leave the main serial line (USB) for debugging and flashing
#include <ModbusMaster.h>         // Modbus master library for ESP8266
#include "PubSubClient.h"

#define MAX485_DE       5         // D1, DE pin on the TTL to RS485 converter
#define MAX485_RE_NEG   4         // D2, RE pin on the TTL to RS485 converter
//#define MAX485_RX       4        // D5, RO pin on the TTL to RS485 converter
//#define MAX485_TX       3        // D6, DI pin on the TTL to RS485 converter
//SoftwareSerial modbus(MAX485_RX, MAX485_TX, false); //RX, TX
ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}




//=======================================================================
//                    MQTT DATA
//=======================================================================



const char* mqttServer = ""; // server
const int mqttPort = 1883; //port
const char* mqttUser = "";  //user
const char* mqttPassword = ""; //pass

WiFiClient espClient;
PubSubClient mqttClient(espClient);




//=======================================================================
//                    Power on setup
//=======================================================================

void setup()
{
  // put your setup code here, to run once:
  // initialize the LED digital pin as an output.
  pinMode(PIN_LED, OUTPUT);

  //modbus.begin(9600);
  Serial.begin(9600);
  ////Serial.println(F("\nDeye Solar Inverter to MQTT Gateway"));
  // Init outputs, RS485 in receive mode
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_LED2, OUTPUT);
  digitalWrite(PIN_LED2, LOW);
  pinMode(PIN_LED4, OUTPUT);
  digitalWrite(PIN_LED4, LOW);
  
  //digitalWrite(PIN_LED, HIGH);
  //digitalWrite(PIN_LED2, HIGH);
  //digitalWrite(PIN_LED4, HIGH);

  node.begin(1,Serial);
  node.preTransmission(preTransmission);
   node.postTransmission(postTransmission);


  //while (!Serial);

  delay(200);

  //Serial.print(F("\nStarting ConfigOnDoubleReset with DoubleResetDetect using ")); //Serial.print(FS_Name);
  //Serial.print(F(" on ")); //Serial.println(ARDUINO_BOARD);
  //Serial.println(ESP_WIFIMANAGER_VERSION);
  //Serial.println(ESP_DOUBLE_RESET_DETECTOR_VERSION);

  if ( String(ESP_WIFIMANAGER_VERSION) < ESP_WIFIMANAGER_VERSION_MIN_TARGET )
  {
    //Serial.print(F("Warning. Must use this example on Version equal or later than : "));
    //Serial.println(ESP_WIFIMANAGER_VERSION_MIN_TARGET);
  }

  Serial.setDebugOutput(false);

  if (FORMAT_FILESYSTEM)
    FileFS.format();

  // Format FileFS if not yet
#ifdef ESP32
  if (!FileFS.begin(true))
#else
  if (!FileFS.begin())
#endif
  {
#ifdef ESP8266
    FileFS.format();
#endif

    //Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));
  
    if (!FileFS.begin())
    {     
      // prevents debug info from the library to hide err message.
      delay(100);
      
      while (true)
      {
        delay(1);
      }
    }
  }

  drd = new DoubleResetDetector(DRD_TIMEOUT, DRD_ADDRESS);

  unsigned long startedAt = millis();

  // New in v1.4.0
  initAPIPConfigStruct(WM_AP_IPconfig);
  initSTAIPConfigStruct(WM_STA_IPconfig);
  //////

  //Local intialization. Once its business is done, there is no need to keep it around
  // Use this to default DHCP hostname to ESP8266-XXXXXX or ESP32-XXXXXX
  //ESP_WiFiManager ESP_wifiManager;
  // Use this to personalize DHCP hostname (RFC952 conformed)
  ESP_WiFiManager ESP_wifiManager("ConfigOnDoubleReset");

#if USE_CUSTOM_AP_IP
  //set custom ip for portal
  // New in v1.4.0
  ESP_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif

  ESP_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESP_wifiManager.setConfigPortalChannel(0);
  //////

#if !USE_DHCP_IP    
    // Set (static IP, Gateway, Subnetmask, DNS1 and DNS2) or (IP, Gateway, Subnetmask). New in v1.0.5
    // New in v1.4.0
    ESP_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);
    //////
#endif

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESP_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");  
#endif

  // We can't use WiFi.SSID() in ESP32 as it's only valid after connected.
  // SSID and Password stored in ESP32 wifi_ap_record_t and wifi_config_t are also cleared in reboot
  // Have to create a new function to store in EEPROM/SPIFFS for this purpose
  Router_SSID = ESP_wifiManager.WiFi_SSID();
  Router_Pass = ESP_wifiManager.WiFi_Pass();

  //Remove this line if you do not want to see WiFi password printed
  //Serial.println("ESP Self-Stored: SSID = " + Router_SSID + ", Pass = " + Router_Pass);

  // SSID/PWD to uppercase
  ssid.toUpperCase();
  //password = "Fotovoltaice365.ro";
 // password = "My" + ssid;
    password = "";

  bool configDataLoaded = false;

  // From v1.1.0, Don't permit NULL password
  if ( (Router_SSID != "") && (Router_Pass != "") )
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());

    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    //Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  }
  
  if (loadConfigData())
  {
    configDataLoaded = true;
    
    ESP_wifiManager.setConfigPortalTimeout(120); //If no access point name has been previously entered disable timeout.
    //Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal")); 

#if USE_ESP_WIFIMANAGER_NTP      
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

  #if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org"); 
  #else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  #endif   
    }
    else
    {
      //Serial.println(F("Current Timezone is not set. Enter Config Portal to set."));
    } 
#endif
  }
  else
  {
    // Enter CP only if no stored SSID on flash and file
    //Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
    initialConfig = true;
  }

  if (drd->detectDoubleReset())
  {
    // DRD, disable timeout.
    ESP_wifiManager.setConfigPortalTimeout(0);

    //Serial.println(F("Open Config Portal without Timeout: Double Reset Detected"));
    initialConfig = true;
  }

  if (initialConfig)
  {
    //Serial.print(F("Starting configuration portal @ "));
    
#if USE_CUSTOM_AP_IP    
    //Serial.print(APStaticIP);
#else
    //Serial.print(F("192.168.4.1"));
#endif

    //Serial.print(F(", SSID = "));
    //Serial.print(ssid);
    //Serial.print(F(", PWD = "));
    //Serial.println(password);

    digitalWrite(PIN_LED, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    //sets timeout in seconds until configuration portal gets turned off.
    //If not specified device will remain in configuration mode until
    //switched off via webserver or device is restarted.
    //ESP_wifiManager.setConfigPortalTimeout(600);

							    // Starts an access point
    if (!ESP_wifiManager.startConfigPortal((const char *) ssid.c_str(), (const char *) password.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
    {
      Serial.println(F("WiFi connected...yeey :)"));
    }												 
    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESP_wifiManager.getSSID(i);
      String tempPW   = ESP_wifiManager.getPW(i);

      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

#if USE_ESP_WIFIMANAGER_NTP      
    String tempTZ   = ESP_wifiManager.getTimezoneName();

    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);

    const char * TZ_Result = ESP_wifiManager.getTZ(WM_config.TZ_Name);
    
    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);
         
    if ( strlen(WM_config.TZ_Name) > 0 )
    {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

  #if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org"); 
  #else
      //configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
  #endif
    }
    else
    {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif

    // New in v1.4.0
    ESP_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////
    
    saveConfigData();
  }

  digitalWrite(PIN_LED, LED_OFF); // Turn led off as we are not in configuration mode.

  startedAt = millis();

  if (!initialConfig)
  {
    // Load stored data, the addAP ready for MultiWiFi reconnection
    if (!configDataLoaded)
      loadConfigData();

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ( (String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE) )
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw );
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

    if ( WiFi.status() != WL_CONNECTED ) 
    {
      //Serial.println(F("ConnectMultiWiFi in setup"));
     
      connectMultiWiFi();
    }
  }

  //Serial.print(F("After waiting "));
  //Serial.print((float) (millis() - startedAt) / 1000);
  //Serial.print(F(" secs more in setup(), connection result is "));

  if (WiFi.status() == WL_CONNECTED)
  {
    digitalWrite(PIN_LED4, LOW);
    //Serial.print(F("connected. Local IP: "));
    //Serial.println(WiFi.localIP());
  }
  else{
    //Serial.println(ESP_wifiManager.getStatus(WiFi.status()));

    }
        mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);
  while (!mqttClient.connected()) {
    //Serial.println("Connecting to MQTT...");
    if (mqttClient.connect(ssid.c_str(), mqttUser, mqttPassword )) {
       //Serial.print("Connected to MQTT with ID:");  
      //Serial.println(ssid.c_str()); 
} else {
       //Serial.print("failed with state ");
      //Serial.print(mqttClient.state());
      delay(2000);
     }
  }

}

void callback(char* topic, byte* payload, unsigned int length) {
 
  //Serial.print("Message arrived in topic: ");
  //Serial.println(topic);
 
  //Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    //Serial.print((char)payload[i]);
  }
 
  //Serial.println();
  //Serial.println("-----------------------");
 
}
void publish_metric (String topic, String value) {
  mqttClient.publish(topic.c_str(), value.c_str(), true);
}


//=======================================================================
//                    Loop
//=======================================================================


void loop()
{
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd->loop();

  // put your main code here, to run repeatedly
if (millis() - lastSendVolt > 15000) {

if (!mqttClient.connected()) {
    ////Serial.println("mqtt NOT connected");
    delay(1000);
    if (mqttClient.connect(ssid.c_str(), mqttUser, mqttPassword )){
 
    //  //Serial.println("Connected to MQTT"); 
    }

  } else {
    ////Serial.println("mqtt connected");
    mqttClient.loop();
  }
 
 uint8_t result;

result = node.readHoldingRegisters(150,45);
if (result==node.ku8MBSuccess)
{
    digitalWrite(PIN_LED2, LED_OFF);
    delay(500);
    digitalWrite(PIN_LED2, LED_ON);
    publish_metric((ssid) + ("/Deye/set1/150"), String(node.getResponseBuffer(00)*0.1)) ;   
    publish_metric((ssid) + ("/Deye/set1/151"), String(node.getResponseBuffer(1)*0.1));    
    publish_metric((ssid) + ("/Deye/set1/152"), String(node.getResponseBuffer(2)*0.1));   
    publish_metric((ssid) + ("/Deye/set1/153"), String(node.getResponseBuffer(3)*0.1));  
    publish_metric((ssid) + ("/Deye/set1/154"), String(node.getResponseBuffer(4)*0.1)); 
    publish_metric((ssid) + ("/Deye/set1/155"), String(node.getResponseBuffer(5)*0.1));
    publish_metric((ssid) + ("/Deye/set1/156"), String(node.getResponseBuffer(6)*0.1));
    publish_metric((ssid) + ("/Deye/set1/157"), String(node.getResponseBuffer(7)*0.1));
    publish_metric((ssid) + ("/Deye/set1/158"), String(node.getResponseBuffer(8)*0.1));
    publish_metric((ssid) + ("/Deye/set1/159"), String(node.getResponseBuffer(9)*0.1));
    publish_metric((ssid) + ("/Deye/set1/160"), String(node.getResponseBuffer(10)*0.01));
    publish_metric((ssid) + ("/Deye/set1/161"), String(node.getResponseBuffer(11)*0.01));
    publish_metric((ssid) + ("/Deye/set1/162"), String(node.getResponseBuffer(12)*0.01));
    publish_metric((ssid) + ("/Deye/set1/163"), String(node.getResponseBuffer(13)*0.01));   
    publish_metric((ssid) + ("/Deye/set1/164"), String(node.getResponseBuffer(14)*0.01));  
    publish_metric((ssid) + ("/Deye/set1/165"), String(node.getResponseBuffer(15)*0.01));  
    publish_metric((ssid) + ("/Deye/set1/166"), String(node.getResponseBuffer(16)));
    publish_metric((ssid) + ("/Deye/set1/167"), String(node.getResponseBuffer(17)));
    publish_metric((ssid) + ("/Deye/set1/168"), String(node.getResponseBuffer(18)));
    publish_metric((ssid) + ("/Deye/set1/169"), String(node.getResponseBuffer(19)));
    publish_metric((ssid) + ("/Deye/set1/170"), String(node.getResponseBuffer(20)));
    publish_metric((ssid) + ("/Deye/set1/171"), String(node.getResponseBuffer(21)));
    publish_metric((ssid) + ("/Deye/set1/172"), String(node.getResponseBuffer(22)));
    publish_metric((ssid) + ("/Deye/set1/173"), String(node.getResponseBuffer(23)));
    publish_metric((ssid) + ("/Deye/set1/174"), String(node.getResponseBuffer(24)));
    publish_metric((ssid) + ("/Deye/set1/175"), String(node.getResponseBuffer(25)));
    publish_metric((ssid) + ("/Deye/set1/176"), String(node.getResponseBuffer(26)));
    publish_metric((ssid) + ("/Deye/set1/177"), String(node.getResponseBuffer(27)));
    publish_metric((ssid) + ("/Deye/set1/178"), String(node.getResponseBuffer(28)));
    publish_metric((ssid) + ("/Deye/set1/179"), String(node.getResponseBuffer(29)*0.01));
    publish_metric((ssid) + ("/Deye/set1/180"), String(node.getResponseBuffer(30)*0.01));
    publish_metric((ssid) + ("/Deye/set1/181"), String(node.getResponseBuffer(31)));
    publish_metric((ssid) + ("/Deye/set1/182"), String(node.getResponseBuffer(32)));
    publish_metric((ssid) + ("/Deye/set1/183"), String(node.getResponseBuffer(33)*0.01));
    publish_metric((ssid) + ("/Deye/set1/184"), String(node.getResponseBuffer(34)));
    publish_metric((ssid) + ("/Deye/set1/185"), String(node.getResponseBuffer(35)));
    publish_metric((ssid) + ("/Deye/set1/186"), String(node.getResponseBuffer(36)));
    publish_metric((ssid) + ("/Deye/set1/187"), String(node.getResponseBuffer(37)));
    publish_metric((ssid) + ("/Deye/set1/188"), String(node.getResponseBuffer(38)));
    publish_metric((ssid) + ("/Deye/set1/189"), String(node.getResponseBuffer(39)));
    publish_metric((ssid) + ("/Deye/set1/190"), String(node.getResponseBuffer(40)));
    publish_metric((ssid) + ("/Deye/set1/191"), String(node.getResponseBuffer(41)*0.01));
    publish_metric((ssid) + ("/Deye/set1/192"), String(node.getResponseBuffer(42)*0.01));
    publish_metric((ssid) + ("/Deye/set1/193"), String(node.getResponseBuffer(43)*0.01));
    publish_metric((ssid) + ("/Deye/set1/194"), String(node.getResponseBuffer(44)));
    digitalWrite(PIN_LED2, LED_OFF);
    delay(500);
    digitalWrite(PIN_LED2, LED_ON);
}
delay(500);
  
result = node.readHoldingRegisters(195,45);
if (result==node.ku8MBSuccess)
{
    publish_metric((ssid) + ("/Deye/set2/195"), String(node.getResponseBuffer(00))) ;   
    publish_metric((ssid) + ("/Deye/set2/196"), String(node.getResponseBuffer(1)));    
    publish_metric((ssid) + ("/Deye/set2/197"), String(node.getResponseBuffer(2)));   
    publish_metric((ssid) + ("/Deye/set2/198"), String(node.getResponseBuffer(3)));  
    publish_metric((ssid) + ("/Deye/set2/199"), String(node.getResponseBuffer(4)*0.1)); 
    publish_metric((ssid) + ("/Deye/set2/200"), String(node.getResponseBuffer(5)));
    publish_metric((ssid) + ("/Deye/set2/201"), String(node.getResponseBuffer(6)*0.1));
    publish_metric((ssid) + ("/Deye/set2/202"), String(node.getResponseBuffer(7)));
    publish_metric((ssid) + ("/Deye/set2/203"), String(node.getResponseBuffer(8)*0.1));
    publish_metric((ssid) + ("/Deye/set2/204"), String(node.getResponseBuffer(9)));
    publish_metric((ssid) + ("/Deye/set2/205"), String(node.getResponseBuffer(10)*0.1));
    publish_metric((ssid) + ("/Deye/set2/206"), String(node.getResponseBuffer(11)));
    publish_metric((ssid) + ("/Deye/set2/207"), String(node.getResponseBuffer(12)*0.1));
    publish_metric((ssid) + ("/Deye/set2/208"), String(node.getResponseBuffer(13)));   
    publish_metric((ssid) + ("/Deye/set2/209"), String(node.getResponseBuffer(14)*0.1));  
    publish_metric((ssid) + ("/Deye/set2/210"), String(node.getResponseBuffer(15)));  
    publish_metric((ssid) + ("/Deye/set2/211"), String(node.getResponseBuffer(16)*0.1));
    publish_metric((ssid) + ("/Deye/set2/212"), String(node.getResponseBuffer(17)));
    publish_metric((ssid) + ("/Deye/set2/213"), String(node.getResponseBuffer(18)*0.1));
    publish_metric((ssid) + ("/Deye/set2/214"), String(node.getResponseBuffer(19)));
    publish_metric((ssid) + ("/Deye/set2/215"), String(node.getResponseBuffer(20)*0.1));
    publish_metric((ssid) + ("/Deye/set2/216"), String(node.getResponseBuffer(21)));
    publish_metric((ssid) + ("/Deye/set2/217"), String(node.getResponseBuffer(22)*0.1));
    publish_metric((ssid) + ("/Deye/set2/218"), String(node.getResponseBuffer(23)*0.1));
    publish_metric((ssid) + ("/Deye/set2/219"), String(node.getResponseBuffer(24)*0.1));
    publish_metric((ssid) + ("/Deye/set2/220"), String(node.getResponseBuffer(25)*0.1));
    publish_metric((ssid) + ("/Deye/set2/221"), String(node.getResponseBuffer(26)*0.1));
    publish_metric((ssid) + ("/Deye/set2/222"), String(node.getResponseBuffer(27)*0.1));
    publish_metric((ssid) + ("/Deye/set2/223"), String(node.getResponseBuffer(28)*0.1));
    publish_metric((ssid) + ("/Deye/set2/224"), String(node.getResponseBuffer(29)*0.1));
    publish_metric((ssid) + ("/Deye/set2/225"), String(node.getResponseBuffer(30)*0.1));
    publish_metric((ssid) + ("/Deye/set2/226"), String(node.getResponseBuffer(31)*0.1));
    publish_metric((ssid) + ("/Deye/set2/227"), String(node.getResponseBuffer(32)*0.1));
    publish_metric((ssid) + ("/Deye/set2/228"), String(node.getResponseBuffer(33)*0.1));
    publish_metric((ssid) + ("/Deye/set2/229"), String(node.getResponseBuffer(34)*0.1));
    publish_metric((ssid) + ("/Deye/set2/230"), String(node.getResponseBuffer(35)));
    publish_metric((ssid) + ("/Deye/set2/231"), String(node.getResponseBuffer(36)));
    publish_metric((ssid) + ("/Deye/set2/232"), String(node.getResponseBuffer(37)));
    digitalWrite(PIN_LED2, LED_OFF);
    delay(500);
    digitalWrite(PIN_LED2, LED_ON);
 
}
delay(500);
result = node.readHoldingRegisters(233,45);
if (result==node.ku8MBSuccess)
{
    publish_metric((ssid) + ("/Deye/set3/233"), String(node.getResponseBuffer(00))) ;   
    publish_metric((ssid) + ("/Deye/set3/234"), String(node.getResponseBuffer(1)));    
    publish_metric((ssid) + ("/Deye/set3/235"), String(node.getResponseBuffer(2)));   
    publish_metric((ssid) + ("/Deye/set3/236"), String(node.getResponseBuffer(3)*0.01));  
    publish_metric((ssid) + ("/Deye/set3/237"), String(node.getResponseBuffer(4)*0.1)); 
    publish_metric((ssid) + ("/Deye/set3/238"), String(node.getResponseBuffer(5)*0.1));
    publish_metric((ssid) + ("/Deye/set3/239"), String(node.getResponseBuffer(6)*0.1));
    publish_metric((ssid) + ("/Deye/set3/240"), String(node.getResponseBuffer(7)*0.1));
    publish_metric((ssid) + ("/Deye/set3/241"), String(node.getResponseBuffer(8)*0.1));
    publish_metric((ssid) + ("/Deye/set3/242"), String(node.getResponseBuffer(9)*0.1));
    publish_metric((ssid) + ("/Deye/set3/243"), String(node.getResponseBuffer(10)*0.1));
    publish_metric((ssid) + ("/Deye/set3/244"), String(node.getResponseBuffer(11)*0.1));
    publish_metric((ssid) + ("/Deye/set3/245"), String(node.getResponseBuffer(12)*0.1));
    publish_metric((ssid) + ("/Deye/set3/246"), String(node.getResponseBuffer(13)*0.1));   
    publish_metric((ssid) + ("/Deye/set3/247"), String(node.getResponseBuffer(14)*0.1));  
    publish_metric((ssid) + ("/Deye/set3/248"), String(node.getResponseBuffer(15)*0.1));  
    publish_metric((ssid) + ("/Deye/set3/249"), String(node.getResponseBuffer(16)*0.1));
    publish_metric((ssid) + ("/Deye/set3/250"), String(node.getResponseBuffer(17)*0.1));
    publish_metric((ssid) + ("/Deye/set3/251"), String(node.getResponseBuffer(18)*0.1));
    publish_metric((ssid) + ("/Deye/set3/252"), String(node.getResponseBuffer(19)*0.1));
    publish_metric((ssid) + ("/Deye/set3/253"), String(node.getResponseBuffer(20)));
    publish_metric((ssid) + ("/Deye/set3/254"), String(node.getResponseBuffer(21)));
    publish_metric((ssid) + ("/Deye/set3/255"), String(node.getResponseBuffer(22)));
    publish_metric((ssid) + ("/Deye/set3/256"), String(node.getResponseBuffer(23)*0.01));
    publish_metric((ssid) + ("/Deye/set3/257"), String(node.getResponseBuffer(24)*0.1));
    publish_metric((ssid) + ("/Deye/set3/258"), String(node.getResponseBuffer(25)*0.1));
    publish_metric((ssid) + ("/Deye/set3/259"), String(node.getResponseBuffer(26)*0.1));
    publish_metric((ssid) + ("/Deye/set3/260"), String(node.getResponseBuffer(27)*0.1));
    publish_metric((ssid) + ("/Deye/set3/261"), String(node.getResponseBuffer(28)*0.1));
    publish_metric((ssid) + ("/Deye/set3/262"), String(node.getResponseBuffer(29)*0.1));
    publish_metric((ssid) + ("/Deye/set3/263"), String(node.getResponseBuffer(30)*0.1));
    publish_metric((ssid) + ("/Deye/set3/264"), String(node.getResponseBuffer(31)*0.1));
    publish_metric((ssid) + ("/Deye/set3/265"), String(node.getResponseBuffer(32)*0.1));
    publish_metric((ssid) + ("/Deye/set3/266"), String(node.getResponseBuffer(33)*0.1));
    publish_metric((ssid) + ("/Deye/set3/267"), String(node.getResponseBuffer(34)*0.1));
    publish_metric((ssid) + ("/Deye/set3/268"), String(node.getResponseBuffer(35)*0.1));
    publish_metric((ssid) + ("/Deye/set3/269"), String(node.getResponseBuffer(36)*0.1));
    publish_metric((ssid) + ("/Deye/set3/270"), String(node.getResponseBuffer(37)*0.1));
    publish_metric((ssid) + ("/Deye/set3/271"), String(node.getResponseBuffer(38)*0.1));
    publish_metric((ssid) + ("/Deye/set3/272"), String(node.getResponseBuffer(39)*0.1));
    publish_metric((ssid) + ("/Deye/set3/273"), String(node.getResponseBuffer(40)));
    publish_metric((ssid) + ("/Deye/set3/274"), String(node.getResponseBuffer(41)));
    publish_metric((ssid) + ("/Deye/set3/275"), String(node.getResponseBuffer(42)));
    publish_metric((ssid) + ("/Deye/set3/276"), String(node.getResponseBuffer(43)));
    publish_metric((ssid) + ("/Deye/set3/277"), String(node.getResponseBuffer(44)));
    
    publish_metric((String("online/") + String(ssid) ).c_str(), "2");  // Send online status
    digitalWrite(PIN_LED2, LED_OFF);
    delay(500);
    digitalWrite(PIN_LED2, LED_ON);
} else {
    publish_metric((String("online/") + String(ssid) ).c_str(), "1");
    publish_metric((String("user/") + String(ssid) ).c_str(), "Deye");
  digitalWrite(PIN_LED2, LED_ON);
    delay(500);
  digitalWrite(PIN_LED2, LED_OFF);
    delay(500);
  digitalWrite(PIN_LED2, LED_ON);
    delay(500);
  digitalWrite(PIN_LED2, LED_OFF);

delay(1000);
  }
  publish_metric((ssid) + ("/Deye/Firmware Dongle Vers"), String(FIRMWARE_VERSION));
  ESPhttpUpdate.setLedPin(PIN_LED, LOW);
//t_httpUpdate_return ret = ESPhttpUpdate.update(espClient, "http://.bin");
        lastSendVolt = millis();
      } //end if milis
  
  //check_status();
} //end loop
