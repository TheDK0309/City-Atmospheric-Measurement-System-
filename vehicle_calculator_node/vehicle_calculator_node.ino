#include <SoftwareSerial.h> 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#include <TinyGPS++.h>

// LoRaWAN NwkSKey, network session key
static const PROGMEM u1_t NWKSKEY[16] = { 0xAE, 0xBD, 0x9B, 0xBC, 0x43, 0x41, 0xFB, 0x47, 0x3D, 0xA5, 0x8D, 0x0D, 0x48, 0x98, 0xEF, 0xCF };

// LoRaWAN AppSKey, application session key
static const u1_t PROGMEM APPSKEY[16] = { 0xE1, 0x51, 0x0F, 0xB1, 0x59, 0x4C, 0xC8, 0xD7, 0xAA, 0x43, 0xAC, 0xE7, 0xE2, 0xBC, 0x5F, 0x1B };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = { 0x260314F7 };

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

unsigned char mydata[29];
static osjob_t sendjob;

int count=0; //vehicle number variable

#define trig 6
#define echo 2 //pins of ultrasonic
bool ultra_active=false; //variable false defaultly, true when detecting vehicles.

// variables that to track the duration of the echo
volatile long echo_start = 0;
volatile long echo_end = 0;
int distance = 0;
#define max_distance 100 //height to mount the utrasonic

#define IR_receiver 3
#define IR_trans 4
int IR_value=0;
bool IR_active=false; //variable false defaultly, true when detecting vehicles.

// Schedule data trasmission in every this many seconds (might become longer due to duty
// cycle limitations).
// we set 10 seconds interval
const unsigned TX_INTERVAL = 10;

// Pin mapping for LoRa Module
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = {3, PIN_CONNECT_TO_IO1, PIN_CONNECT_TO_IO2}
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.println(ev);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println("EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println("EV_JOINED");
      break;
    case EV_RFU1:
      Serial.println("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      Serial.println("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println("EV_LINK_ALIVE");
      break;
    default:
      Serial.println("Unknown event");
      break;
  }
}

void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    String temporary = "";
    temporary = String(count) + String(' ');
    temporary.toCharArray((char *)mydata, temporary.length());
    
    LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  erial.begin(115200);
  // Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting");

  GPS_SoftSerial.begin(9600);  // connect gps sensor 

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT_PULLUP);

  pinMode(IR_receiver,INPUT);
  pinMode(IR_transmit,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(echo), echo_interrupt, CHANGE);  //detect object by interrupt, when change of pulse from echo pin

  
  
  #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
  #endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  //LMIC_setClockError(MAX_CLOCK_ERROR * 1/100);
  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  #if defined(CFG_eu868)
  /*
  Set up the channels used by the Things Network, which corresponds
  to the defaults of most gateways. Without this, only three base
  channels from the LoRaWAN specification are used, which certainly
  works, so it is good for debugging, but can overload those
  frequencies, so be sure to configure the full frequency range of
  your network here (unless your network autoconfigures them).
  Setting up channels should happen after LMIC_setSession, as that
  configures the minimal channel set.
  NA-US channels 0-71 are configured automatically
*/
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);  // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);   // g2-band
/*
  TTN defines an additional channel at 869.525Mhz using SF9 for class B
  devices' ping slots. LMIC does not have an easy way to define set this
  frequency and support for class B is spotty and untested, so this
  frequency is not configured here.
*/
  #elif defined(CFG_us915)
  /*
  NA-US channels 0-71 are configured automatically
  but only one group of 8 should (a subband) should be active
  TTN recommends the second sub band, 1 in a zero based count.
  https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
*/
    LMIC_selectSubBand(1);
  #endif
  
  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7, 14);

  printabpinformation();
  // Start job
  do_send(&sendjob);

}

void loop() {
  os_runloop_once();

  smartDelay(1000);
  unsigned long start;

  digitalWrite(trig, LOW); 
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  
  delayMicroseconds(10); 
  digitalWrite(trig, LOW); 
  
  delay(1000);

  IR();

  if(ultra_active==true && IR_active==true){
    count++;
  }
  
  lat_val = gps.location.lat(); 
  loc_valid = gps.location.isValid(); 
  lng_val = gps.location.lng();
  alt_m_val = gps.altitude.meters(); 
  alt_valid = gps.altitude.isValid(); 

  if (!loc_valid)
  {
    Serial.print("Latitude : ");
    Serial.println("*****");
    Serial.print("Longitude : ");
    Serial.println("*****");
    delay(4000);
  }
  else
  {
    Serial.println("GPS READING: ");
    DegMinSec(lat_val);
    Serial.print("Latitude in Decimal Degrees : ");
    Serial.println(lat_val, 6);

    DegMinSec(lng_val); 
    Serial.print("Longitude in Decimal Degrees : ");
    Serial.println(lng_val, 6);
    delay(4000);
  }
}
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (GPS_SoftSerial.available()) 
    gps.encode(GPS_SoftSerial.read());
  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)
{
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}

void echo_interrupt() {
  switch (digitalRead(echo))
  {
  case HIGH:
    echo_end = 0;
    echo_start = micros();
    break;
  case LOW:
    if (echo_end == 0) {
      echo_end = micros();
      long duration = echo_end - echo_start;
      
      distance = durationOneWay * 0.03435 / 2; // distance in cm
    }
    break;
  }
  if(distance<distance_max){
    ultra_active=true;
  }
  else{
    ultra_active=false;
  }
}
void IR(void){
  digitalWrite(IR_transmit,HIGH);
  IR_value=analogRead(IR_receiver);
  Serial.print(IR_value);
  if(IR_value < 100){
    IR_active=true;
  }
  else{
    IR_active=false;
  }
}
