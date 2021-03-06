/*
Copyright (C) RainingPhotons 2018.
Distributed under the MIT License (license terms are at http://opensource.org/licenses/MIT).
*/

#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <FastLED.h>
#include <SPI.h>
#include <UIPEthernet.h>
#include <Wire.h>

#define DEBUG_OUTPUT

#ifdef DEBUG_OUTPUT
#define DEBUG_OUT(a) (Serial.println(a))
#else
#define DEBUG_OUT(a)
#endif

#define LED_PIN_1   PB3
#define LED_PIN_2   PA15
#define NUM_LEDS    120
#define BRIGHTNESS  64
#define LED_TYPE    SK6812
#define COLOR_ORDER GRB
CRGB leds_1[NUM_LEDS];
CRGB leds_2[NUM_LEDS];

char udp_read_buffer[64];
EthernetUDP udp;
uint8_t address = 0;
bool accelerometer_enabled = false;
uint32_t sample_rate = 1000;
bool auto_refresh = true;

#define SELF_IPADDRESS 192,168,1
#define SELF_PORT 5000
#define SERVER_IPADDRESS 192,168,1,28
#define SERVER_PORT 5002

static const uint8_t kBaseAddress = 200;
static const uint8_t kIPMask[4] = {255,255,255,0};
static const uint8_t kDNS[4] = {192,168,1,1};
static const uint8_t kGateway[4] = {192,168,1,1};

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

uint8_t read_address() {
  pinMode(PB11, INPUT_PULLUP);
  pinMode(PB10, INPUT_PULLUP);
  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB0, INPUT_PULLUP);
  pinMode(PA3, INPUT_PULLUP);
  pinMode(PA2, INPUT_PULLUP);
  pinMode(PA1, INPUT_PULLUP);
  pinMode(PA0, INPUT_PULLUP);

  uint8_t address = 0;
  address |= (!digitalRead(PB11) & 0x1) << 0;
  address |= (!digitalRead(PB10) & 0x1) << 1;
  address |= (!digitalRead(PB1) & 0x1) << 2;
  address |= (!digitalRead(PB0) & 0x1) << 3;
  address |= (!digitalRead(PA3) & 0x1) << 4;
  address |= (!digitalRead(PA2) & 0x1) << 5;
  address |= (!digitalRead(PA1) & 0x1) << 6;
  address |= (!digitalRead(PA0) & 0x1) << 7;

  return address;
}

void set_all_leds(uint8_t command, int32_t value) {
  CRGB set_value;
  switch(command) {
    case 'r' : set_value = CRGB::Red; break;
    case 'g' : set_value = CRGB::Green; break;
    case 'b' : set_value = CRGB::Blue; break;
    default : set_value = CRGB::White; break;
  }

  int brightness = value;
  if (brightness > 0 && brightness < 256)
    FastLED.setBrightness(brightness);

  for (int i = 0; i < NUM_LEDS; ++i) {
    leds_1[i] = set_value;
    leds_2[i] = set_value;
 }

  if (auto_refresh) {
    FastLED[0].showLeds(brightness);
    FastLED[1].showLeds(brightness);
  }
}

void show_value(uint8_t strand, uint8_t value) {
  int i;

  CRGB *leds = leds_1;
  if (strand == 1)
    leds = leds_2;

  for (int i = 0; i < value; ++i)
    leds[i] = CRGB::Red;

  FastLED.show();
}

void setup() {
  // Not sure this is necessary
  delay(3000); // power-up safety delay

  // Builtin LED
  pinMode(PC13, OUTPUT);

  // Necessary to use PA15 and PB3 for LED output.
  // Defaults to being used for JTAG
  disableDebugPorts();

  FastLED.addLeds<LED_TYPE, LED_PIN_1, COLOR_ORDER>(leds_1, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.addLeds<LED_TYPE, LED_PIN_2, COLOR_ORDER>(leds_2, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setDither(0);
  FastLED.setBrightness(16);
  FastLED.show();

  address = read_address();
  show_value(0, address);
  address += kBaseAddress;

  uint8_t mac[6] = {0xde,0xad,0xbe,0xef,0x04,address};

  uint8_t ip_address[4] = {SELF_IPADDRESS, address};
  Ethernet.begin(mac, ip_address, kDNS, kGateway, kIPMask);

#ifdef DEBUG_OUTPUT
  while (!Serial);
  Serial.begin(115200);
#endif
  DEBUG_OUT("Starting up");

  DEBUG_OUT(Ethernet.localIP());
  DEBUG_OUT(Ethernet.subnetMask());
  DEBUG_OUT(Ethernet.gatewayIP());
  DEBUG_OUT(Ethernet.dnsServerIP());

  if (!lis.begin(0x18)) {
    DEBUG_OUT("Couldnt start");
    show_value(1, 1);
    accelerometer_enabled = false;
  } else {
    DEBUG_OUT("LIS3DH found!");
    accelerometer_enabled = false;
    lis.setRange(LIS3DH_RANGE_4_G);
  }
}

char message[1024];

void loop() {
  //check for new udp-packet:
  udp.begin(SELF_PORT);
  int size = udp.parsePacket();
  if (size > 0) {
    do {
        if (size == 720 || size == 722) {
          udp.read(message, size + 1);
          memcpy(leds_1, message + 0, sizeof(leds_1));
          memcpy(leds_2, message + sizeof(leds_1), sizeof(leds_2));
          int brightness_1 = 255;
          int brightness_2 = 255;
          if (size == 722) {
            brightness_1 = message[720];
            brightness_2 = message[721];
          }
          if (auto_refresh) {
            FastLED[0].showLeds(brightness_1);
            FastLED[1].showLeds(brightness_2);
          }
        } else if (size == 360 || size == 361) {
          udp.read(message, size + 1);
          memcpy(leds_1, message, sizeof(leds_1));
          memcpy(leds_2, message, sizeof(leds_2));
          int brightness = 255;
          if (size == 361)
            brightness = message[360];
          if (auto_refresh) {
            FastLED[0].showLeds(brightness);
            FastLED[1].showLeds(brightness);
          }
        } else if (size < sizeof(udp_read_buffer)) {
          int len = udp.read(udp_read_buffer, size + 1);
          udp_read_buffer[len]=0;
          DEBUG_OUT(udp_read_buffer);
          uint8_t command = udp_read_buffer[0];
          int32_t value = atoi(udp_read_buffer + 1);

          switch(command) {
            case 's' : sample_rate = value; break;
            case 'a' : accelerometer_enabled = value; break;
            case 'c' : auto_refresh = value; break;
            case 'd' :
              FastLED[0].showLeds(value);
              FastLED[1].showLeds(value);
              break;
            default : set_all_leds(command, value);
          }
        } else {
          // TODO(frk) : Handle this error
          udp.flush();
          DEBUG_OUT("This is bad, recieved a packet too large to process, dropping on the floor");
        }
    } while ((size = udp.available())>0);
    //finish reading this packet:
    udp.flush();
    udp.stop();
  }

  blinkLED(sample_rate);
  if (accelerometer_enabled)
    readLIS3DH(sample_rate);
}

void blinkLED(long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(PC13, !digitalRead(PC13));
  }
}

void readLIS3DH(long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    lis.read();
    bool success;
    if (udp.beginPacket(IPAddress(SERVER_IPADDRESS),SERVER_PORT)) {
      uint16_t buffer[4];
      buffer[0] = address;
      buffer[1] = lis.x;
      buffer[2] = lis.y;
      buffer[3] = lis.z;

      udp.write(buffer, sizeof(buffer));
      udp.endPacket();
      udp.stop();
    } else {
      DEBUG_OUT("Unable to beginPacket");
    }
  }
}
