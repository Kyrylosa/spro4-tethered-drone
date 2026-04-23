#include <WiFi.h>
#include <esp_now.h>
#include <OneWire.h>  //to use the temperature sensors
#include <DallasTemperature.h> //for easier work with the temp sensors

#define ONE_WIRE_BUS 16 //gpio where the sensor data line is connected

OneWire oneWire(ONE_WIRE_BUS);  //initiates onewire
DallasTemperature sensors(&oneWire); // initiates stuff for dallastemp


// for 1 temp sensor
uint8_t sensor_address[8]; //stores rom code of the sensor
bool sensors_found = false; //becomes true when a sensor is found
float ambient_temp = 0.0f; //stores the temp value
bool temp_valid = false; //turns true after first reading so it doesnt send nonsense

unsigned long last_request_time = 0;
const unsigned long conversion_time = 750; // DS18B20 max conversion time in ms


//mac address of esp that this red esp talks to (brown esp)
static const uint8_t brown_peer_mac[6] = {0xCC, 0x8D, 0xA2, 0x2B, 0xA8, 0x7C};


//for sending stuff with esp now
struct message {
  char     source; //B = brown, R = red
  char     type; //T = temp, K = keyboard typing
  float    temperature; //valid when type = T
  char     command; //valid when type = K
  uint32_t counter; //grows every time we send a message
};


bool initialized = false;
bool esp_now_ready = false; //turns true when peer is added successfully

uint32_t send_counter = 0; //counts how many messages have been sent
unsigned long last_send_time = 0;
const unsigned long send_interval = 4000; //send temp every 4 sec


void setup() {
    Serial.begin(115200); //setup has nothing else because my laptop doesnt print anything inside of setup no matter how i adjust it
}


//callback for esp now every time esp now tries to send something
void sending(const uint8_t *mac_address, esp_now_send_status_t status) {
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}


//callback for esp now when it receives data
void receiving(const uint8_t *mac_address, const uint8_t *data, int len) {
    //len is the number of bytes in the incoming esp message

    if (len != sizeof(message)) { //check if len matches message struct size
        Serial.print("unexpected message length: ");
        Serial.println(len);
        return;
    }

    message msg;
    memcpy(&msg, data, sizeof(msg));

    Serial.print("RX from ");
    Serial.print(msg.source);
    Serial.print(" \n type=");
    Serial.print(msg.type);

    if (msg.type == 'T') {
        Serial.print(" \n temp = ");
        Serial.print(msg.temperature, 2);
        Serial.print(" °C");
    } else if (msg.type == 'K') {
        Serial.print(" \n command = '");
        Serial.print(msg.command);
        Serial.print("'");
    }

    Serial.print(" \n counter = ");
    Serial.println(msg.counter);
}



bool init_esp_now() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("esp now init failed");
        return false;
    }

    esp_now_register_send_cb(sending);
    esp_now_register_recv_cb(receiving);

    esp_now_peer_info_t peerInfo{};
    memcpy(peerInfo.peer_addr, brown_peer_mac, 6);
    peerInfo.channel = 0; //same wifi channel
    peerInfo.encrypt = false;  //no encryption

    if (!esp_now_is_peer_exist(brown_peer_mac)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("failed to add esp now as a peer");
            return false;
        }
    }

    Serial.println("esp now initialized");
    return true;
}



void init_sensors() {
    sensors.begin();

    if (!sensors.getAddress(sensor_address, 0)) {
        Serial.println("no DS18B20 sensor found");
        sensors_found = false;
        return;
    }

    sensors_found = true;
    temp_valid = false;

    sensors.setWaitForConversion(false); 
    sensors.requestTemperatures(); 
    last_request_time = millis();

    Serial.print("DS18B20 address: ");
    for (uint8_t i = 0; i < 8; i++) {
        if (sensor_address[i] < 16) Serial.print("0");
        Serial.print(sensor_address[i], HEX);
    }
    Serial.println();
}



void update_temperature() {
    if (!sensors_found) return;

    unsigned long now = millis();
    if (now - last_request_time >= conversion_time) {

        float t = sensors.getTempC(sensor_address);

        
        if (t > -100.0f && t < 125.0f) {
            ambient_temp = t;
            temp_valid = true;
        }

        sensors.requestTemperatures(); //start next conversion
        last_request_time = now;
    }
}




void send_temperature_if_needed() {
    if (!esp_now_ready || !sensors_found || !temp_valid) return;

    unsigned long now = millis();
    if (now - last_send_time < send_interval) {
        return; //not time yet
    }

    message msg{};
    msg.source = 'R';  //this is the red esp
    msg.type = 'T';
    msg.temperature = ambient_temp;
    msg.command = 0;
    msg.counter = ++send_counter;

    esp_err_t result = esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));

    Serial.print("TX to BROWN (temp) | temp=");
    Serial.print(msg.temperature, 2);
    Serial.print(" °C \n counter=");
    Serial.print(msg.counter);
    Serial.print(" \n status=");
    Serial.println(result == ESP_OK ? "OK" : "ERROR");

    last_send_time = now;
}




void for_keyboard_commands() {
    if (!esp_now_ready) return;

    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') continue; //ignore enter

        message msg{};
        msg.source = 'R';
        msg.type = 'K';
        msg.temperature = ambient_temp;
        msg.command = c;
        msg.counter = ++send_counter;

        esp_err_t result = esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));

        Serial.print("TX to BROWN (command) \n '");
        Serial.print(c);
        Serial.print("' counter=");
        Serial.print(msg.counter);
        Serial.print(" \n status=");
        Serial.println(result == ESP_OK ? "OK" : "ERROR");
    }
}


void loop() {

    if (!initialized) {

        delay(1000); //delay for serial

        Serial.print("my mac: ");
        Serial.println(WiFi.macAddress());

        WiFi.mode(WIFI_STA);
        esp_now_ready = init_esp_now();
        init_sensors();

        initialized = true;
    }

    update_temperature(); //checking temp  
    send_temperature_if_needed(); //send temp messages
    for_keyboard_commands(); //send keyboard messages
}


//
//temperature sensor + uart + esp now
//

#include <WiFi.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 16

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//uart
static const int UART_RX_PIN = 18; //red ESP RX to 3rd ESP TX
static const int UART_TX_PIN = 17; //red ESP TX to 3rd ESP RX
static const uint32_t UART_BAUD = 115200;

HardwareSerial UartFromThird(1);


static char uartLine[64];
static size_t uartPos = 0;

// temp sensor
uint8_t sensor_address[8];
bool sensors_found = false;
float ambient_temp = 0.0f;
bool temp_valid = false;

unsigned long last_request_time = 0;
const unsigned long conversion_time = 750;

// brown peer mac
static const uint8_t brown_peer_mac[6] = {0xCC, 0x8D, 0xA2, 0x2B, 0xA8, 0x7C};


struct message {
  char source; // B=brown, R=red, 3=third
  char  type; // T=temp, K=keyboard
  float temperature; // valid if type isT
  char command; // valid if type is K
  uint32_t counter;
};

bool initialized = false;
bool esp_now_ready = false;

uint32_t send_counter = 0;
unsigned long last_send_time = 0;
const unsigned long send_interval = 4000;

void setup() {
  Serial.begin(115200); // wont do anything that is in setup for some reason
}

// esp now callbacks
void sending(const uint8_t *mac_address, esp_now_send_status_t status) {
  Serial.print("Send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void receiving(const uint8_t *mac_address, const uint8_t *data, int len) {
  if (len != sizeof(message)) {
    Serial.print("unexpected message length: ");
    Serial.println(len);
    return;
  }

  message msg;
  memcpy(&msg, data, sizeof(msg));

  Serial.print("RX from ");
  Serial.print(msg.source);
  Serial.print(" \n type=");
  Serial.print(msg.type);

  if (msg.type == 'T') {
    Serial.print(" \n temp = ");
    Serial.print(msg.temperature, 2);
    Serial.print(" °C");
  } else if (msg.type == 'K') {
    Serial.print(" \n command = '");
    Serial.print(msg.command);
    Serial.print("'");
  }

  Serial.print(" \n counter = ");
  Serial.println(msg.counter);

  if (msg.type == 'T') {
    char line[32];
    snprintf(line, sizeof(line), "T,%.2f\n", msg.temperature);
    UartFromThird.print(line);
  } else if (msg.type == 'K') {
    UartFromThird.print("K,");
    UartFromThird.write(msg.command);
    UartFromThird.print("\n");
  }
}


bool init_esp_now() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("esp now init failed");
    return false;
  }

  esp_now_register_send_cb(sending);
  esp_now_register_recv_cb(receiving);

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, brown_peer_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(brown_peer_mac)) {
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("failed to add esp now as a peer");
      return false;
    }
  }

  Serial.println("esp now initialized");
  return true;
}

// ds18b20
void init_sensors() {
  sensors.begin();

  if (!sensors.getAddress(sensor_address, 0)) {
    Serial.println("no DS18B20 sensor found");
    sensors_found = false;
    return;
  }

  sensors_found = true;
  temp_valid = false;

  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  last_request_time = millis();
}

void update_temperature() {
  if (!sensors_found) return;

  unsigned long now = millis();
  if (now - last_request_time >= conversion_time) {
    float t = sensors.getTempC(sensor_address);

    if (t > -100.0f && t < 125.0f) {
      ambient_temp = t;
      temp_valid = true;
    }

    sensors.requestTemperatures();
    last_request_time = now;
  }
}

// send red's temperature to brown esp
void send_temperature_if_needed() {
  if (!esp_now_ready || !sensors_found || !temp_valid) return;

  unsigned long now = millis();
  if (now - last_send_time < send_interval) return;

  message msg{};
  msg.source = 'R';
  msg.type = 'T';
  msg.temperature = ambient_temp;
  msg.command = 0;
  msg.counter = ++send_counter;

  esp_err_t result = esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));

  Serial.print("tx to brown temp=");
  Serial.print(msg.temperature, 2);
  Serial.print(" °C \n counter=");
  Serial.print(msg.counter);
  Serial.print(" \n status=");
  Serial.println(result == ESP_OK ? "OK" : "ERROR");

  last_send_time = now;
}

// keyboard
void for_keyboard_commands() {
  if (!esp_now_ready) return;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') continue;

    message msg{};
    msg.source = 'R';
    msg.type = 'K';
    msg.temperature = ambient_temp;
    msg.command = c;
    msg.counter = ++send_counter;

    esp_err_t result = esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));

    Serial.print("TX to BROWN (command) \n '");
    Serial.print(c);
    Serial.print("' counter=");
    Serial.print(msg.counter);
    Serial.print(" \n status=");
    Serial.println(result == ESP_OK ? "OK" : "ERROR");
  }
}

// uart receive, forward to esp with brown capacitors

void handle_uart_from_third() {
  if (!esp_now_ready) {
    //so buffer doesn't fill
  }

  while (UartFromThird.available()) {
    char ch = (char)UartFromThird.read();


    if (ch == '\n') {
      uartLine[uartPos] = '\0'; 
      uartPos = 0;

      if (uartLine[0] == 'T' && uartLine[1] == ',') {
        float t = atof(&uartLine[2]);

        message msg{};
        msg.source = '3';// coming from 3rd ESP
        msg.type = 'T';
        msg.temperature = t;
        msg.command = 0;
        msg.counter = ++send_counter;

        if (esp_now_ready) {
          esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));
        }

        Serial.print("uart -- esp now forwarded temp from 3rd: ");
        Serial.println(t, 2);
      }
      else if (uartLine[0] == 'K' && uartLine[1] == ',') {
        char c = uartLine[2];

        message msg{};
        msg.source = '3';
        msg.type = 'K';
        msg.temperature = ambient_temp;
        msg.command = c;
        msg.counter = ++send_counter;

        if (esp_now_ready) {
          esp_now_send(brown_peer_mac, (uint8_t *)&msg, sizeof(msg));
        }

        Serial.print("uart -- esp now forwarded key from 3rd: '");
        Serial.print(c);
        Serial.println("'");
      }

      continue;
    }

    // ignore cr
    if (ch == '\r') continue;

    // store char if buffer space exists
    if (uartPos < sizeof(uartLine) - 1) {
      uartLine[uartPos++] = ch;
    } else {
      // overflow reset (drop line)
      uartPos = 0;
    }
  }
}

void loop() {
  if (!initialized) {
    delay(1000);

    Serial.print("my mac: ");
    Serial.println(WiFi.macAddress());

    WiFi.mode(WIFI_STA);
    esp_now_ready = init_esp_now();
    init_sensors();

    // uart init to 3rd esp (not in setup)
    UartFromThird.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    initialized = true;
  }

  update_temperature();
  send_temperature_if_needed();
  for_keyboard_commands();

  handle_uart_from_third();
}

//
//
//



//
// attempt at putting it all together
//
#ifdef RED_ESP_BUILD

//red esp
//heating + esp now + uart to purple in one place

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h> //isnan



struct message {
  char source; //'b' brown or 'r' red
  char type;  //'t' temp, 'k' command, 's' status
  float temperature; //optional float temp for easy printing
  char command;  //command letter when type is 'k' or 's'
  uint32_t counter; //counts to spot dropped packets
  int16_t ambient_centi //ambient temp * 100
  int16_t backup_centi; //free field for later
  uint16_t humidity_tenths; //free field
};

static const bool k_debug = true; 

static void debugln(const String &s) {
  if (k_debug) Serial.println(s);
}




static const uint8_t broadcast_mac[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static uint8_t learned_peer_mac[6] = {0, 0, 0, 0, 0, 0};
static bool peer_known = false;

static bool esp_now_ready = false;
static uint32_t send_counter = 0;

static bool add_peer_if_needed(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return true;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  return (esp_now_add_peer(&peer) == ESP_OK);
}

static const uint8_t *get_send_mac() {
  return peer_known ? learned_peer_mac : broadcast_mac;
}

static void send_msg(const message &msg) {
  if (!esp_now_ready) return;
  const uint8_t *dest = get_send_mac();
  esp_now_send(dest, (const uint8_t *)&msg, sizeof(msg));
}


//uart to purple esp

static const int uart_tx_pin = 17;
static const int uart_rx_pin = 18;

static void forward_motor_command(char cmd) {
  //we just forward single letters
  Serial1.write(cmd);
  Serial1.flush(false);
}


//heating ??


static const int one_wire_pin = 5;

static const int max_sensors  = 14;
static const int max_channels = 8;

static const int pwm_freq_hz = 500;
static const int pwm_resolution_bits = 10;
static const int pwm_top = (1 << pwm_resolution_bits) - 1; //1023

static const float temp_limit_c = 45.0f;

static const uint32_t ds_conversion_ms = 750;
static const uint32_t control_interval_ms = 1000;
static const uint32_t report_interval_ms = 4000;


//ds18b20
OneWire oneWire(one_wire_pin);
DallasTemperature ds18(&oneWire);

struct temp_sensor {
  uint8_t addr[8];
  const char *name;
  float last_c;
  bool ok;
};

//14 sensor addresses

static temp_sensor sensor_list[max_sensors] = {
  {{0x28, 0x72, 0xE6, 0xF8, 0x0F, 0x00, 0x00, 0x9C}, "front_head", 0.0f, false},
  {{0x28, 0x6A, 0x18, 0xF9, 0x0F, 0x00, 0x00, 0xED}, "right_torso", 0.0f, false},
  {{0x28, 0x49, 0x71, 0x0B, 0x10, 0x00, 0x00, 0x83}, "right_shoulder", 0.0f, false},
  {{0x28, 0x99, 0xD5, 0xF8, 0x0F, 0x00, 0x00, 0x87}, "left_shoulder", 0.0f, false},
  {{0x28, 0xD9, 0x33, 0xF9, 0x0F, 0x00, 0x00, 0x3B}, "right_head", 0.0f, false},
  {{0x28, 0x35, 0x45, 0xF8, 0x0F, 0x00, 0x00, 0x11}, "left_head", 0.0f, false},
  {{0x28, 0xDD, 0x6D, 0xF8, 0x0F, 0x00, 0x00, 0x5F}, "front_legs_a", 0.0f, false},
  {{0x28, 0x43, 0x5F, 0xF9, 0x0F, 0x00, 0x00, 0x75}, "front_legs_b", 0.0f, false},
  {{0x28, 0xB3, 0x1A, 0x0B, 0x10, 0x00, 0x00, 0x5F}, "left_torso_a", 0.0f, false},
  {{0x28, 0xF3, 0x0B, 0xF8, 0x0F, 0x00, 0x00, 0x77}, "left_torso_b", 0.0f, false},
  {{0x28, 0x1B, 0x09, 0xF8, 0x0F, 0x00, 0x00, 0x7C}, "front_torso_a", 0.0f, false},
  {{0x28, 0x87, 0x2D, 0x0C, 0x10, 0x00, 0x00, 0x3D}, "front_torso_b", 0.0f, false},
  {{0x28, 0x8F, 0x0F, 0x0C, 0x10, 0x00, 0x00, 0xE7}, "side_legs_right", 0.0f, false},
  {{0x28, 0xCF, 0xFF, 0x0A, 0x10, 0x00, 0x00, 0x3D}, "side_legs_left", 0.0f, false},
};

struct pid_state {
  float kp;
  float ki;
  float kd;

  float integral;
  float last_error;
  uint32_t last_ms;

  bool enabled; //false = warmup, true = pid
};

struct heater_channel {
  uint8_t pwm_ch;
  uint8_t pwm_pin;

  //warmup pwm is the fixed pwm
  uint16_t warmup_pwm;

  //max pwm cap for safety (and so pid cannot overdrive)
  uint16_t pwm_max;

  //1 or 2 sensors control the channel
  temp_sensor *s1;
  temp_sensor *s2;

  const char *area;

  //target offset from ambient
  float offset_c;


  float target_c;
  float temp_c;
  uint16_t duty;

  pid_state pid;
};


static heater_channel channel_list[max_channels] = {
  {0, 11, 327, 327, &sensor_list[0],  &sensor_list[0],  "head front", 14.0f, 0, 0, 0, {0}},
  {1, 10, 164, 164, &sensor_list[1],  &sensor_list[1],  "torso right", 7.0f, 0, 0, 0, {0}},
  {2, 13, 327, 327, &sensor_list[2],  &sensor_list[3],  "shoulders", 7.0f, 0, 0, 0, {0}},
  {3, 12, 460, 460, &sensor_list[4],  &sensor_list[5],  "head sides", 14.0f, 0, 0, 0, {0}},
  {4, 21, 225, 225, &sensor_list[6],  &sensor_list[7],  "legs front", 7.0f, 0, 0, 0, {0}},
  {5, 14, 256, 256, &sensor_list[8],  &sensor_list[9],  "torso left", 7.0f, 0, 0, 0, {0}},
  {6, 48, 225, 225, &sensor_list[10], &sensor_list[11], "torso front", 7.0f, 0, 0, 0, {0}},
  {7, 47, 102, 102, &sensor_list[12], &sensor_list[13], "legs sides", 7.0f, 0, 0, 0, {0}},
};

//ambient comes from brown esp
static float ambient_temp_c = 21.0f;
static bool ambient_valid = false;

//enable flags
static bool heating_enabled = false;
static bool motors_enabled = false;

//timing vars
static uint32_t last_ds_request_ms = 0;
static uint32_t last_control_ms = 0;
static uint32_t last_report_ms = 0;


//heating helpers

static bool looks_like_real_temp(float t) {
  //ds18 has two classic bad readings
  //85 shows right after boot sometimes
  //-127 shows when the sensor is not reachable
  if (t == 85.0f) return false;
  if (t == -127.0f) return false;
  if (t < -55.0f) return false;
  if (t > 125.0f) return false;
  return true;
}

static float get_channel_temp(const heater_channel &ch) {
  //average if both sensors work
  //else use the one that works
  if (ch.s1->ok && ch.s2->ok) return (ch.s1->last_c + ch.s2->last_c) * 0.5f;
  if (ch.s1->ok) return ch.s1->last_c;
  if (ch.s2->ok) return ch.s2->last_c;
  return NAN;
}

static void heater_write(uint8_t pwm_ch, uint16_t duty) {
  duty = constrain(duty, 0, pwm_top);
  ledcWrite(pwm_ch, duty);
}

static void turn_off_all_heaters() {
  for (int i = 0; i < max_channels; i++) {
    channel_list[i].duty = 0;
    channel_list[i].pid.enabled = false;
    channel_list[i].pid.integral = 0;
    channel_list[i].pid.last_error = 0;
    heater_write(channel_list[i].pwm_ch, 0);
  }
}

static void init_pwm_and_pid() {
  for (int i = 0; i < max_channels; i++) {
    //pwm hardware
    ledcSetup(channel_list[i].pwm_ch, pwm_freq_hz, pwm_resolution_bits);
    ledcAttachPin(channel_list[i].pwm_pin, channel_list[i].pwm_ch);
    heater_write(channel_list[i].pwm_ch, 0);

    //pid numbers
    channel_list[i].pid.kp = 2.0f;
    channel_list[i].pid.ki = 0.05f;
    channel_list[i].pid.kd = 0.0f;

    channel_list[i].pid.integral = 0;
    channel_list[i].pid.last_error = 0;
    channel_list[i].pid.enabled = false;
    channel_list[i].pid.last_ms = millis();
  }
}

static uint16_t pid_step(heater_channel &ch, float dt_s) {
  //dt protection
  if (dt_s < 0.1f) dt_s = 0.1f;

  float error = ch.target_c - ch.temp_c;

  ch.pid.integral += error * dt_s;
  ch.pid.integral = constrain(ch.pid.integral, -200.0f, 200.0f);

  float derivative = (error - ch.pid.last_error) / dt_s;

  float output = (ch.pid.kp * error) + (ch.pid.ki * ch.pid.integral) + (ch.pid.kd * derivative);
  ch.pid.last_error = error;

  output = constrain(output, 0.0f, (float)pwm_top);
  return (uint16_t)output;
}

static void update_ds18_nonblocking() {
  //request, wait, read
  //this way we do not block the loop with delays

  uint32_t now = millis();
  if (now - last_ds_request_ms < ds_conversion_ms) return;

  //read all sensors
  for (int i = 0; i < max_sensors; i++) {
    float t = ds18.getTempC(sensor_list[i].addr);
    sensor_list[i].last_c = t;
    sensor_list[i].ok = looks_like_real_temp(t);
  }

  //start next conversion
  ds18.requestTemperatures();
  last_ds_request_ms = now;
}

static void run_heaters() {
  //stage 1 warmup to stage 2 pid

  uint32_t now = millis();
  if (now - last_control_ms < control_interval_ms) return;
  last_control_ms = now;

  for (int i = 0; i < max_channels; i++) {
    heater_channel &ch = channel_list[i];

    //target is always ambient + offset
    ch.target_c = ambient_temp_c + ch.offset_c;

    //read temp
    ch.temp_c = get_channel_temp(ch);

    //if sensors are dead, shut off
    if (isnan(ch.temp_c)) {
      ch.duty = 0;
      ch.pid.enabled = false;
      heater_write(ch.pwm_ch, 0);
      continue;
    }

    //safety cutoff
    if (ch.temp_c > temp_limit_c) {
      ch.duty = 0;
      ch.pid.enabled = false;
      ch.pid.integral = 0;
      ch.pid.last_error = 0;
      heater_write(ch.pwm_ch, 0);
      if (k_debug) debugln(String("hard shutoff ch ") + i);
      continue;
    }

    //stage 1 warmup
    if (!ch.pid.enabled) {
      ch.duty = ch.warmup_pwm;
      if (ch.duty > ch.pwm_max) ch.duty = ch.pwm_max;
      heater_write(ch.pwm_ch, ch.duty);

      //switch to pid once the target is reached
      if (ch.temp_c >= ch.target_c) {
        ch.pid.enabled = true;
        ch.pid.integral = 0;
        ch.pid.last_error = 0;
        ch.pid.last_ms = now;
        if (k_debug) debugln(String("ch ") + i + " - pid");
      }

      continue;
    }

    //stage 2 pid
    float dt_s = (now - ch.pid.last_ms) / 1000.0f;
    ch.pid.last_ms = now;

    ch.duty = pid_step(ch, dt_s);
    if (ch.duty > ch.pwm_max) ch.duty = ch.pwm_max;
    heater_write(ch.pwm_ch, ch.duty);
  }
}

static void report_to_brown() {
  //send the hottest channel temp to brown every few seconds
  //kept it small so the packet is easy

  uint32_t now = millis();
  if (now - last_report_ms < report_interval_ms) return;
  last_report_ms = now;

  float max_t = -999.0f;
  int max_i = -1;

  for (int i = 0; i < max_channels; i++) {
    float t = channel_list[i].temp_c;
    if (!isnan(t) && t > max_t) {
      max_t = t;
      max_i = i;
    }
  }

  message msg{};
  msg.source = 'R';
  msg.type = 'T';
  msg.temperature = max_t;
  msg.command = (max_i >= 0 && max_i <= 9) ? (char)('0' + max_i) : 'n';
  msg.counter = ++send_counter;
  msg.ambient_centi = (int16_t)(ambient_temp_c * 100.0f);

  send_msg(msg);
}


//esp now callbacks

static void on_send(const uint8_t *, esp_now_send_status_t status) {
  if (!k_debug) return;
  if (status != ESP_NOW_SEND_SUCCESS) debugln("send fail");
}

static void on_receive(const uint8_t *mac, const uint8_t *data, int len) {
  //learn the peer mac once
  if (!peer_known && mac) {
    memcpy(learned_peer_mac, mac, 6);
    peer_known = true;
    add_peer_if_needed(learned_peer_mac);
    if (k_debug) debugln("learned peer");
  }

  if (len != (int)sizeof(message)) return;

  message msg{};
  memcpy(&msg, data, sizeof(msg));

  //ambient temp from brown
  if ((msg.type == 'T' || msg.type == 't') && (msg.source == 'B' || msg.source == 'b')) {
    if (msg.ambient_centi != 0) {
      ambient_temp_c = msg.ambient_centi / 100.0f;
      ambient_valid = true;
    } else {
      ambient_temp_c = msg.temperature;
      ambient_valid = true;
    }
  }

  //commands from brown
  if (msg.type == 'K' || msg.type == 'k') {
    char c = msg.command;

    //start everything
    if (c == 'S' || c == 's') {
      heating_enabled = true;
      motors_enabled = true;
      forward_motor_command('S');
    }

    //just heat
    if (c == 'H' || c == 'h') {
      heating_enabled = true;
    }

    //stop all
    if (c == 'X' || c == 'x') {
      heating_enabled = false;
      motors_enabled = false;
      turn_off_all_heaters();
      forward_motor_command('X');
    }

    //quick ack back to brown to see it got the command
    message ack{};
    ack.source = 'R';
    ack.type = 'S';
    ack.command = c;
    ack.counter = ++send_counter;
    ack.ambient_centi = (int16_t)(ambient_temp_c * 100.0f);
    send_msg(ack);
  }
}

static bool init_esp_now() {
  if (esp_now_init() != ESP_OK) {
    debugln("esp now init failed");
    return false;
  }

  esp_now_register_send_cb(on_send);
  esp_now_register_recv_cb(on_receive);

  //add broadcast peer so we can talk before we know the other mac
  if (!add_peer_if_needed(broadcast_mac)) {
    debugln("failed to add broadcast peer");
    return false;
  }

  return true;
}

void setup() {
  if (k_debug) Serial.begin(115200);

  //wifi sta mode is needed for esp now
  WiFi.mode(WIFI_STA);
  esp_now_ready = init_esp_now();

  //uart to purple
  Serial1.begin(115200, SERIAL_8N1, uart_rx_pin, uart_tx_pin);

  //ds18 setup
  ds18.begin();
  ds18.setWaitForConversion(false);
  ds18.requestTemperatures();
  last_ds_request_ms = millis();

  //pwm and pid setup
  init_pwm_and_pid();
  turn_off_all_heaters();

  if (k_debug) {
    debugln(String("my mac is ") + WiFi.macAddress());
    debugln("waiting for brown packet");
  }
}

void loop() {
  //keep temps updating in the background
  update_ds18_nonblocking();

  //only do the full shutoff once when we go on to off
  static bool last_heating_enabled = false;
  if (!heating_enabled) {
    if (last_heating_enabled) {
      turn_off_all_heaters();
    }
    last_heating_enabled = false;
    return;
  }
  last_heating_enabled = true;

  //if we never got ambient from brown yet, we still run using the default 21c
  //once brown starts sending, targets shift automatically
  run_heaters();

  //send a small report back
  report_to_brown();
}

#endif //red_esp_build

