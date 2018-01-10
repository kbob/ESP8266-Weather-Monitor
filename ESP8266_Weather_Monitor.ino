#include <FS.h>     // must be first, says someone on the Internet

#include <Adafruit_MQTT_Client.h>
#include <ArduinoJson.h>
#include <SparkFun_Si7021_Breakout_Library.h>
#include <WiFiManager.h>

const uint32_t POLL_INTERVAL_sec = 5 * 60;

const int SERIAL_BAUD = 115200;
const uint32_t SERIAL_SETTLING_msec = 100;

// Jumper shorts pins 12 and 13.
const int JP1 = 12;
const int JP2 = 16;

const char CONFIG_FILE[] = "/config.json";

const char     NTP_SERVER_NAME[] = "time.nist.gov";
const uint16_t NTP_PORT = 123;
const uint16_t NTP_LOCAL_PORT = 2390;
const size_t   NTP_PACKET_SIZE = 48;
const uint32_t NTP_TIMEOUT_msec = 200;

// Configuration strings
char device_ident[10];  // "inside", "outside", "basement", "garage"
char aio_user[32];      // Adafruit IO user name
char aio_user_key[40];  // Adafruit IO user key
char aio_temp_key[32];  // Adafruit IO temperature stream key
char aio_humid_key[32]; // Adafruit IO humidity stream key

bool config_needs_saving;

uint32_t ntp_time_sec;  // seconds since 1900-Jan-01-00:00:00Z
uint32_t ntp_offset_msec; // millis when ntp_time_sec set

float temp_degC;        // last temperature reading, degrees Celsius
float humid_pct;        // last humidity reading, relative percent

//#define Z (Serial.print(__func__), \
//           Serial.print(" : "),    \
//           Serial.println(__LINE__))

void init_serial()
{
  Serial.begin(SERIAL_BAUD);
  uint32_t t0 = millis();      
  while (millis() < t0 + SERIAL_SETTLING_msec)
    continue;
  Serial.print("\n\n");
  info("starting");
}

void fatal(const char *msg)
{
  Serial.print("fatal: ");
  Serial.println(msg);
  delay(10);            // wait for UART to finish
  ESP.deepSleep(POLL_INTERVAL_sec * 1000000);
}

void warn(const char *msg)
{
  Serial.print("warning: ");
  Serial.println(msg);
}

void notice(const char *msg)
{
  Serial.print("notice: ");
  Serial.println(msg);
}

void info(const char *msg)
{
  Serial.print("info: ");
  Serial.println(msg);
}

void init_SPIFFS()
{
  if (!SPIFFS.begin()) {
    fatal("SPIFFS mount failed");
  }
  info("SPIFFS mounted ok");
}

void strlcpy0(char *dest, const char *src, size_t size)
{
  if (src) {
    strlcpy(dest, src, size);
  } else if (size) {
    dest[0] = '\0';
  }
}

bool read_config_data()
{
  File f = SPIFFS.open(CONFIG_FILE, "r");
  if (!f) {
    notice("failed to open config for reading");
    return false;
  }
  info("SPIFFS opened config.");

  size_t size = f.size();
  char buf[size];
  f.readBytes(buf, size);
  f.close();
  
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.parseObject(buf);
  if (!json.success()) {
    warn("JSON parse failed");
    return false;
  }

  #define GET(name) (strlcpy0(name, json[#name], sizeof name))
    GET(device_ident);
    GET(aio_user);
    GET(aio_user_key);
    GET(aio_temp_key);
    GET(aio_humid_key);
  #undef GET

  return device_ident[0] &&
         aio_user[0] &&
         aio_temp_key[0] &&
         aio_humid_key[0];
}

void write_config_data()
{
  info("writing config data");
  File f = SPIFFS.open(CONFIG_FILE, "w");
  if (!f) {
    warn("failed to open config for writing");
    return;
  }
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["device_ident"] = device_ident;
  json["aio_user"] = aio_user;
  json["aio_user_key"] = aio_user_key;
  json["aio_temp_key"] = aio_temp_key;
  json["aio_humid_key"] = aio_humid_key;

  Serial.print("info: writing ");
  json.printTo(Serial);
  Serial.println();

  json.printTo(f);
  f.close();
}

// true iff JP1 and JP2 are jumpered together
bool jumper_is_present()
{
  pinMode(JP1, INPUT_PULLUP);
  pinMode(JP2, INPUT_PULLUP);
  if (digitalRead(JP1) != HIGH) {
    return false;
  }
  if (digitalRead(JP2) != HIGH) {
    return false;
  }
  pinMode(JP1, OUTPUT);
  digitalWrite(JP1, LOW);
  if (digitalRead(JP2) != LOW) {
    return false;
  }
  pinMode(JP1, INPUT_PULLUP);
  pinMode(JP2, OUTPUT);
  digitalWrite(JP2, LOW);
  if (digitalRead(JP1) != LOW) {
    return false;
  }
  info("jumper is present");
  return true;
}

void save_config_callback()
{
  info("save_config called back");
  config_needs_saving = true;
}

void init_WiFi()
{
  bool ok;

  char ssid[32];
  if (*device_ident) {
    snprintf(ssid, sizeof ssid,
             "Weather Logger (%s)", device_ident);
  } else {
    snprintf(ssid, sizeof ssid,
             "Weather Logger (%d)", ESP.getChipId());
  }
  info(ssid);

  #define DCL_PARAM(param, var, desc) \
    WiFiManagerParameter param(#var, desc, var, sizeof var)

    DCL_PARAM(did_param, device_ident, "Device Identity");
    DCL_PARAM(au_param, aio_user, "Adafruit IO User Name");
    DCL_PARAM(auk_param, aio_user_key, "Adafruit IO User Key");
    DCL_PARAM(atk_param,
              aio_temp_key,
              "Adafruit IO Temperature Key");
    DCL_PARAM(ahk_param, aio_humid_key, "Adafruit IO Humidity Key");
  #undef DCL_PARAM

  WiFiManager mgr;

  mgr.addParameter(&did_param);
  mgr.addParameter(&au_param);
  mgr.addParameter(&auk_param);
  mgr.addParameter(&atk_param);
  mgr.addParameter(&ahk_param);

  mgr.setSaveConfigCallback(save_config_callback);

  if (jumper_is_present()) {
    ok = mgr.startConfigPortal(ssid);
  } else {
    ok = mgr.autoConnect(ssid);
  }
  if (!ok) {
    fatal("failed to configure Wi-Fi");
  }

  if (config_needs_saving) {
    #define RD_PARAM(param, var) \
      (strlcpy(var, param.getValue(), sizeof var))

      RD_PARAM(did_param, device_ident);
      RD_PARAM(au_param, aio_user);
      RD_PARAM(auk_param, aio_user_key);
      RD_PARAM(atk_param, aio_temp_key);
      RD_PARAM(ahk_param, aio_humid_key);

    #undef RD_PARAM

    write_config_data();
  }
}

void send_NTP_packet(WiFiUDP& udp, IPAddress& address)
{
  char buf[NTP_PACKET_SIZE];
  memset(buf, 0, sizeof buf);
  buf[0] = 0b11100011;          // LI, Version, Mode
  buf[1] = 0;                   // Stratum
  buf[2] = 6;                   // Polling Interval
  buf[3] = 0xEC;                // Peer Clock Precision
  // bytes 4-11 are zero
  buf[12] = 49;
  buf[13] = 0x4E;
  buf[14] = 49;
  buf[15] = 52;

  udp.beginPacket(address, NTP_PORT);
  udp.write(buf, NTP_PACKET_SIZE);
  udp.endPacket();
}

uint32_t receive_NTP_packet(WiFiUDP& udp)
{
  char buf[NTP_PACKET_SIZE];
  udp.read(buf, sizeof buf);
  unsigned char *p = (unsigned char *)(buf + 40);
  return p[0] << 24 | p[1] << 16 | p[2] << 8 | p[3] << 0;
}

void get_NTP_time()
{
  WiFiUDP udp;
  udp.begin(NTP_LOCAL_PORT);
  IPAddress ntp_server_ip;
  WiFi.hostByName(NTP_SERVER_NAME, ntp_server_ip);
  send_NTP_packet(udp, ntp_server_ip);
  uint32_t t0 = millis();
  int cb;
  while (1) {
    cb = udp.parsePacket();
    if (cb || millis() >= t0 + NTP_TIMEOUT_msec) {
      break;
    }
    delay(1);
  }
  uint32_t t1 = millis();
  ntp_time_sec = receive_NTP_packet(udp);
  ntp_offset_msec = t1;

  Serial.printf("info: NTP latency ");
  Serial.print(t1 - t0);
  Serial.println("msec");

  Serial.print("info: ntp_time = ");
  Serial.print(ntp_time_sec);
  Serial.println(" sec");

  Serial.print("info: ntp_offset = ");
  Serial.print(ntp_offset_msec);
  Serial.println(" msec");
}

void read_sensors()
{
  Weather sensor;
  sensor.begin();
  humid_pct = sensor.getRH();
  temp_degC = sensor.getTemp();
  {
    //vaprintf not found; do it the hard way.
    Serial.print("info: humidity = ");
    Serial.print(humid_pct);
    Serial.print("%, temp = ");
    Serial.print(temp_degC);
    Serial.println(" degC");
  }
}

const char AIO_SERVER[] = "io.adafruit.com";
const uint16_t AIO_SERVER_PORT = 8883;
const char AIO_FINGERPRINT[] = "AD 4B 64 B3 67 40 B5 FC 0E 51 "
                               "9B BD 25 E9 7F 88 B6 2A A3 5B";

void log_data_Adafruit_IO()
{
  char temp_topic[64], humid_topic[64];
  snprintf(temp_topic, sizeof temp_topic,
           "%s/feeds/%s", aio_user, aio_temp_key);
  snprintf(humid_topic,sizeof humid_topic,
           "%s/feeds/%s", aio_user, aio_humid_key);
  info(temp_topic);
  info(humid_topic);

  WiFiClientSecure client;
  Adafruit_MQTT_Client mqtt(&client,
                            AIO_SERVER,
                            AIO_SERVER_PORT, 
                            aio_user, 
                            aio_user_key);
  Adafruit_MQTT_Publish temp(&mqtt, temp_topic, MQTT_QOS_1);
  Adafruit_MQTT_Publish humid(&mqtt, humid_topic, MQTT_QOS_1);

  client.connect(AIO_SERVER, AIO_SERVER_PORT);
  if (client.verify(AIO_FINGERPRINT, AIO_SERVER)) {
    info("AIOfingerprint verified");
  } else {
    fatal("AIO fingerprint verification failed");
  }

  if (mqtt.connect() != 0) {
    warn("AIO MQTT connection failed");
    return;
  }
  info("AIO MQTT connected");

  if (!temp.publish(temp_degC)) {
    warn("failed to publish temperature to AIO");
  }
  if (!humid.publish(humid_pct)) {
    warn("failed to publish humidity to AIO");
  }

  mqtt.disconnect();
}

void go_to_sleep()
{
  Serial.print("info: sleeping at msec ");
  Serial.println(millis());

  // Calculate how long to sleep so we wake up "on the hour".
  const uint32_t poll_interval_usec = POLL_INTERVAL_sec * 1000000;
  uint32_t sleep_time_usec = poll_interval_usec;
  sleep_time_usec += ntp_offset_msec * 1000;
  sleep_time_usec -= sleep_time_usec % poll_interval_usec;
  sleep_time_usec -= millis() * 1000;

  Serial.print("info: sleeping "); 
  Serial.print(sleep_time_usec);
  Serial.println(" usec");
  delay(10);  // Let UART finish

  ESP.deepSleep(sleep_time_usec);  
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  init_serial();
  init_SPIFFS();
  read_config_data();
  init_WiFi();
  get_NTP_time();
  read_sensors();

  log_data_Adafruit_IO();

  go_to_sleep();
}

void loop()
{
  // never called
}
