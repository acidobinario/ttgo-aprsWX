// MQTT VARS
#define mqtt_server ""  // Home Assistant IP address
#define mqtt_user ""           // Home assistant user name
#define mqtt_password "" // Home Assistant password
#define DEVICE_ID "ESP32"        // give a unique name to your device

// WiFi VARS
const char *ssid = "";  //  your wifi SSID
const char *pass = ""; //  your wifi password

// Telnet Configuration from APRS server
const String USER = "";                // write your aprs callsign
const String PAS = "";                     // write your aprs password
const String LAT = "";                  // write your latitude
const String LON = "";                 // write your longitute
const String COMMENT = "APRS weather on ESP32"; // write some comment
const String SERVER = "brazil.aprs2.net";       // write the address of the aprs server
const int APRSPORT = 14579;                     // write the aprs server APRSPORT
