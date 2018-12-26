// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
//#include <SPI.h>
//#include <SSD1306.h>
#include <Wifi.h>
#include "driver/rtc_io.h"

const char* ssid = "phone";
const char* password = "";

#define LEDPIN 2
#define BUTTONPIN 0
#define SWITCHPIN 13

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

unsigned int counter = 0;
char TTN_response[30];

RTC_DATA_ATTR int _triggerCount = 0;
RTC_DATA_ATTR bool _isJoined = false;
RTC_DATA_ATTR lmic_t _lmic = { 0 };

esp_sleep_wakeup_cause_t  _wakeup_reason;

#if 0
SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);
#endif

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5*60     /* Time ESP32 will go to sleep (in seconds) */

// I2C code

#define SCL_PIN          12               /*!< gpio number for I2C master clock */
#define SDA_PIN          13               /*!< gpio number for I2C master data  */

//#define SCL_PIN          15               /*!< gpio number for I2C master clock */
//#define SDA_PIN          4               /*!< gpio number for I2C master data  */

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes.
// This should also be in little endian format, see above.
//For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.

static const u1_t PROGMEM APPEUI[8]={ 0x1A, 0x5A, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };   // Chose LSB mode on the console and then copy it here.
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static u1_t PROGMEM DEVEUI[8]={ 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };   // LSB mode
void os_getDevEui (u1_t* buf) { 
    memcpy_P(buf,(const void *)DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x67, 0xDE, 0x14, 0x6C, 0xE3, 0x49, 0x90, 0xBE, 0xF7, 0xC9, 0x43, 0xA7, 0x71, 0xB6, 0xB6, 0x7D }; // MSB mode

void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

char _buff[80];

void do_send(osjob_t* j){
    // Payload to send (uplink)
    // PID,ITEMNO,STATE,TRIGGER,COUNT
    static uint8_t message[] = "ttgo,1,1,1,0";

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

    //    int buttonValue = digitalRead(BUTTONPIN);
    //    bool state_on = buttonValue == 0;
        int switchValue = digitalRead(SWITCHPIN);
        bool state_on = switchValue == 1;

        // PID,ITEMNO,STATE,TRIGGER,COUNT
        uint64_t chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).

        sprintf(_buff, "ttgo,%llx,%d,%d,%d", chipid,state_on,_wakeup_reason,_triggerCount);
              
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (unsigned char *)&_buff, strlen(_buff), 0);
        Serial.println(F("Sending uplink packet..."));
        digitalWrite(LEDPIN, HIGH);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
           case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
              Serial.println(F("Received ack"));
            }

            if (LMIC.dataLen) {
              int i = 0;
              // data received in rx slot after tx
              Serial.print(F("Data Received: "));
              Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              Serial.println();
              Serial.println(LMIC.rssi);

                // For now any downlink message flashes the LED and clears the counters
                _triggerCount = 0;

                // Flash LED...
                for(int i = 0; i < 5; i++)
                {
                    digitalWrite(LEDPIN, HIGH);
                    delay(250);
                    digitalWrite(LEDPIN, LOW);
                    delay(250);
                }
            }

            // Copy across our structure to NVRAM backed storage
            memcpy( (void *)&_lmic, (const void *)&LMIC, sizeof(LMIC));

            digitalWrite(LEDPIN, LOW);

            Serial.println(F("Sleep"));
            pinMode(LEDPIN,INPUT);

            // Schedule next transmission
//            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

#if 0
            // Wake up on button press
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_0,0);
            rtc_gpio_pulldown_dis( GPIO_NUM_0);
            rtc_gpio_pullup_en( GPIO_NUM_0);
            esp_deep_sleep_start();
#endif

            // Wake up on sleep timer
            esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

            // Wake up on switch change
            {
                int switchValue = digitalRead(SWITCHPIN);
                bool state = switchValue == 1;
                
                if(state)
                {
                    // Wake on change to OFF
                    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,0);
                    rtc_gpio_pulldown_dis( GPIO_NUM_13);
                    rtc_gpio_pullup_en( GPIO_NUM_13);
                }
                else
                {
                    // Wake on change to ON
                    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13,1);
                    rtc_gpio_pullup_dis( GPIO_NUM_13);
                    rtc_gpio_pulldown_en( GPIO_NUM_13);
                }
                esp_deep_sleep_start();
            }
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING: -> Joining..."));
            break;
        case EV_JOINED: {
              Serial.println(F("EV_JOINED"));

              _isJoined = true;
              
              // Copy across our structure to NVRAM backed storage
              memcpy( (void *)&_lmic, (const void *)&LMIC, sizeof(LMIC));

              // Disable link check validation (automatically enabled
              // during join, but not supported by TTN at this time).
              LMIC_setLinkCheckMode(0);
            }
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }

}

void setup() {
    esp_err_t ret;
    uint8_t sensor_data_h, sensor_data_l;

    // We were using this for RTC sleep wakeup... (maybe)
    // Button
    rtc_gpio_deinit(GPIO_NUM_0); 
    // Switch
    rtc_gpio_deinit(GPIO_NUM_13); 

    Serial.begin(115200);
    delay(2500);                      // Give time to the serial monitor to pick up
    Serial.println(F("Starting..."));

    // Use the Blue pin to signal transmission.
    pinMode(LEDPIN,OUTPUT);

    pinMode(BUTTONPIN, INPUT);
    pinMode(SWITCHPIN, INPUT);

    uint64_t chipID=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    Serial.printf("ChipID: %llx\r\n", chipID);

    // Update DevEUI
    memcpy_P(DEVEUI,(const void *)&chipID, sizeof(uint64_t));

    // Get wakeup reason (might only be able to do this once after start?)
    _wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(_wakeup_reason)
    {
        case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
        case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
        case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
        case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
        default : Serial.printf("Wakeup was not caused by deep sleep: %d\r\n",_wakeup_reason); break;
    }

    // Incremement if the switch state change woke us
    if(_wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
      _triggerCount++;

    Serial.print("TriggerCount: ");
    Serial.println(_triggerCount);

    int buttonValue = digitalRead(BUTTONPIN);
    Serial.print("ButtonValue: ");
    Serial.println(buttonValue);

    int switchValue = digitalRead(SWITCHPIN);
    Serial.print("SwitchValue: ");
    Serial.println(switchValue);

    // Wifi - was intending to pull acc. data from a phone app but we won't be doing this now.
#if 0
    int count = 20;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED && count--) {
        delay(500);
        display.clear();
        display.drawString(0,0,"Connecting to WiFi..");
        display.display();
    } 
    display.clear();
    if(WiFi.status() == WL_CONNECTED) {
        display.drawString(0,0,"Connected !!!");
    }
    else{
        display.drawString(0,0,"Error Connecting to WiFi !!!");
    }
    display.display();
    delay(3000);

    int stat;
    int acc[3];
#endif

    // LMIC init
    os_init();

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.

    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    /*
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
*/

    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
//    LMIC.dn2Dr = DR_SF9;
    LMIC.dn2Dr = DR_SF7;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    //LMIC_setDrTxpow(DR_SF11,14);
    //  LMIC_setDrTxpow(DR_SF9,14);
    LMIC_setDrTxpow(DR_SF7,14);

    if(_isJoined)
    {
        LMIC_setSession (_lmic.netid, _lmic.devaddr, _lmic.nwkKey, _lmic.artKey);
        LMIC.seqnoDn = _lmic.seqnoDn;
        LMIC.seqnoUp= _lmic.seqnoUp;
    }

    // Start job
    do_send(&sendjob);     // Will fire up also the join
    //LMIC_startJoining();
}

void loop() {
    os_runloop_once();
}
