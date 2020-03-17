/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
//Sensors librairies
#include <SI7021.h>

#define debugSerial Serial
#define SHOW_DEBUGINFO
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }


//Define sensor PIN
#define PDPIN 3  // PIN with PIR Sensor Digital output
#define SOUNDPIN A3     // what pin we're connected to
#define VCCSOUNDPIN 4   // VCC sound pin
#define LAPIN A0 // PIN with Light sensor analog output 
#define LPPIN 5 // PIN with Light power input

#define USE_SI7021 // Define is the SI7021 sensor is used
#define USE_SOUND // Define is the sound sensor is used


// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x260110A0;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0xC3, 0xA8, 0x8F, 0x00, 0x01, 0x0B, 0x38, 0xAD, 0xE8, 0x53, 0x57, 0xB5, 0xDC, 0xF8, 0x21, 0x1A };


// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0xE7, 0xED, 0x37, 0x35, 0x25, 0x2D, 0xEB, 0xB1, 0xA2, 0xAA, 0x42, 0x57, 0x13, 0xE9, 0x03, 0x66 };

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;



// global enviromental parameters
static float batvalue;
static boolean presence;
static byte pres [128];
static unsigned int pres_it = 0; 
static float pres_avg = 0;
static int waiting_presence = 0; // This int is incremented if no presence is detected during a sensing slot
static int sound [64];
static unsigned int snd_it = 0;
static float sound_avg = 0;
static boolean sound_bool = 0;
static int sound_treshold = 400;
static float temp = 0.0;
static float humidity = 0.0;
static float light = 0.0;
static boolean presence_detected = 0;

#ifdef USE_SI7021
SI7021 sensor;
#endif


// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 8,
    .dio = {2, 7, 9},
};



// ---------------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------------

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
unsigned int TX_INTERVAL = 300;
unsigned int LONG_SLEEP = 1800;



extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

// Function do_sleep_ware : sleep for sleepyTime unleast an event happen
void do_sleep_aware(unsigned int sleepyTime) {
unsigned int eights = sleepyTime / 8;

  attachInterrupt(digitalPinToInterrupt(3), wakeUp, FALLING); // Interrupt is added on PIN to detect a movement

  // if no presence detected for two slots, wait for 30mn or an event for next uplink


  #ifdef SHOW_DEBUGINFO
  debugPrint(F("Sleeping for "));
  debugPrint(sleepyTime);
  debugPrint(F(" seconds"));
  debugPrint(F(" or wake up if an activity is detected"));
  delay(30); //Wait for serial to complete
  #endif

    for ( int x = 0; x < eights; x++) { // Sleep for 30mn if no movement detected
    // put the processor to sleep for 8 seconds
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
      if ( digitalRead(PDPIN)) { // if a movement is detected, send an uplink and move back to normal mode
     waiting_presence = 0 ;
     pres [1] = 2;
     pres_it = 2;
     // Disable external pin interrupt on wake up pin.
    detachInterrupt(1); 
     return;
        }
      if ((x % 8) == 7 ){ // each 1 minute, test if the light has strongly changed, can be used to detect if the light has been swith on or off
       float light_actual = readLight();
       float diff= abs (light - light_actual);
        if (diff > light || diff > light_actual){ // For a change hiher than 100%, wake-up
        detachInterrupt(1);
        return;
          }  
        
        }
      
    }
  
}


void do_sleep(unsigned int sleepyTime) {
  

  unsigned int eights = sleepyTime / 8;
  unsigned int fours = (sleepyTime % 8) / 4;
  unsigned int twos = ((sleepyTime % 8) % 4) / 2;
  unsigned int ones = ((sleepyTime % 8) % 4) % 2;
 

  #ifdef SHOW_DEBUGINFO
  debugPrint(F("Sleeping for "));
  debugPrint(sleepyTime);
  debugPrint(F(" seconds = "));
  debugPrint(eights);
  debugPrint(F(" x 8 + "));
  debugPrint(fours);
  debugPrint(F(" x 4 + "));
  debugPrint(twos);
  debugPrint(F(" x 2 + "));
  debugPrintLn(ones);
  delay(50); //Wait for serial to complete
#endif
  
    for ( int x = 0; x < eights; x++) {
    
    pres [pres_it] = digitalRead(PDPIN);
                    
    if (pres_it < 127) {
      pres_it++;
          }
    else { // The array is full, start again
    pres_it = 0;
    pres [0] = pres [127];    
    }
    
    #ifdef USE_SOUND
    if (readSound()) { // if a sound is detected      
      sound_bool = 1 ;
    }
    #endif

    // put the processor to sleep for 8 seconds
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    
  }
  
  for ( int x = 0; x < fours; x++) {
    // put the processor to sleep for 4 seconds
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < twos; x++) {
    // put the processor to sleep for 2 seconds
    LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  }
  for ( int x = 0; x < ones; x++) {
    // put the processor to sleep for 1 seconds
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  addMillis(sleepyTime * 1000); 
    
}


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

float readLight() {
  float result;
  // Light sensor Voltage
digitalWrite(LPPIN, HIGH); // Power the sensor
LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);// wait for 15ms for the sensor to set-up

int sensorValue = analogRead(LAPIN);
 
  result = (float)sensorValue * (batvalue / 1023.0); // Batvalue is in tens of mV, so the result has to be divided by 100

    result = (float)sensorValue * (110 / 1023.0); // Batvalue is in tens of mV, so the result has to be divided by 100
    result = result*20.0; // multiply by 200 to have Lx
    digitalWrite(LPPIN, LOW); // switch off the sensor
    return result;
}

bool readSound() {
  digitalWrite(VCCSOUNDPIN, HIGH); // Switch on Sound sensor
  int detection = 0;
  bool sound_detect = 0;
    // Spleep for 30ms for the sound detector to wake up
    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
    delay(1);// don't know why, but we need a delay
    int sensorValue = (1023-analogRead(SOUNDPIN)); // ADC in 10 bit resolution
    sensorValue = 1000.0 * (float)sensorValue / batvalue;
    int sum=sensorValue;
     
    for (int i=1;i<10;i++){
    if (sensorValue > sound_treshold){
         detection++;   
    }
    sensorValue = 1023-analogRead(SOUNDPIN); // ADC in 10 bit resolution
    sensorValue = 1000.0 * (float)sensorValue / batvalue;
      sum = sum + sensorValue;
      }
    
    digitalWrite(VCCSOUNDPIN, LOW);
    sound[snd_it] = sum;
    if (snd_it < 63) {    
    snd_it++;
    }
    else { // The array is full, start again
    snd_it = 0;
    sound [0] = sound [63];
    }
    if (detection >= 2){
      sound_detect = 1;    
    }
    return sound_detect;
        
}

void updateEnvParameters()
{
    pres [pres_it] = digitalRead(PDPIN); // It is important to read the PIR Value before switching any output PIN due to noise sensitivity of the PIR
    pres_it++;
    
    
    #ifdef USE_SI7021
    // get humidity and temperature in one shot, saves power because sensor takes temperature when doing humidity anyway
    si7021_env data = sensor.getHumidityAndTemperature();
  
    temp = data.celsiusHundredths;
    humidity = data.humidityBasisPoints;
    #endif

    batvalue = (int)(readVcc()/10);  // readVCC returns in tens of mVolt for Cayenne Payload
    

    #ifdef USE_SOUND
    if (readSound()) { // if a sound is detected      
      sound_bool = 1 ;
    }
    #endif  

  // Compute average from sound sensor
    long somme = sound[0];
    for (int i = 1 ; i < snd_it ; i++)
    {
        somme += sound[i] ; //somme des valeurs (db) du tableau
    }   

    sound_avg = (float)somme / ((float)snd_it) ; //valeur moyenne
    snd_it = 0; // reset sound counter
  
    // Compute average from PIR SR-501
    somme = pres[pres_it-1];
    for (int i = 1 ; i < pres_it ; i++)
    {
        somme += (int)pres[i] ; //somme des valeurs (db) du tableau
    }
    
    pres_avg = (float)somme / ((float)pres_it) ; //valeur moyenne
    pres_it = 0; // reset presence counter

    if (pres_avg > 0 || sound_bool){ // Sense presence based on a sound detected or a PIR sense
      presence_detected = 1;
    }

    else {
      presence_detected = 0;
    }
  
 light = readLight(); // Read Light value
 
  #ifdef SHOW_DEBUGINFO
  // print out the value you read:
  Serial.print("Humidity : ");
  Serial.println(humidity);
  Serial.print("T°c : ");
  Serial.println(temp);
  Serial.print("Light : ");
  Serial.println(light);
  Serial.print("Vbatt : ");
  Serial.println(batvalue);
  Serial.print("Average Presence : ");
  Serial.println(pres_avg);
  Serial.print("Average Sound : ");
  Serial.println(sound_avg);
  delay(5);
  #endif
  
}




void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
              Serial.print(F("0"));
              }
              Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
            }
            // Schedule next transmission
            do_sleep(TX_INTERVAL);
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            os_setCallback(&sendjob, do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
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

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    debugPrintLn(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    // Here the sensor information should be retrieved
    
    updateEnvParameters();
    
      if ( pres_avg == 0 && sound_bool == 0) {
      
      waiting_presence++; // if no presence is detected, activate the waiting presence mode
        }
      else {
      waiting_presence = 0; // if a presence is detected during the slot, start again the waiting presence counter
        }


    if(waiting_presence == 2){ // if no movement is detected on the second slot, do not send uplink and move to sleep aware mode
      do_sleep_aware(LONG_SLEEP-TX_INTERVAL); // go to sleep aware mode 
      updateEnvParameters();} 

    

      


    int t = (int)((temp) /10);
    int h = (int)(humidity /50);
    int bat = batvalue; // Cayenne analog output is 0.01 Signed
    unsigned int l = (unsigned int)light; // light sensor in 0.1 signed Lx
    boolean p = presence; // Presence indicator
    int p_avg = (int) 10000 * pres_avg; // Cayenne analog output is 0.01 Signed and PIR sensor will be in %
    int s_avg = (int) 1 * sound_avg; // Cayenne analog output is 0.01 Signed 
    boolean p_d= presence_detected; // Sound indicator
    sound_bool = 0;

    unsigned char mydata[26];
    mydata[0] = 0x1;
    mydata[1] = 0x2;
    mydata[2] = bat >> 8;
    mydata[3] = bat & 0xFF;
    mydata[4] = 0x2;
    mydata[5] = 0x2;
    mydata[6] = p_avg>> 8;
    mydata[7] = p_avg & 0xFF;
    mydata[8] = 0x3;
    mydata[9] = 0x2;
    mydata[10] = s_avg>> 8;
    mydata[11] = s_avg & 0xFF;
    mydata[12] = 0x4;
    mydata[13] = 0x67;
    mydata[14] = t >> 8;
    mydata[15] = t & 0xFF;
    mydata[16] = 0x5;
    mydata[17] = 0x68;    
    mydata[18] = h & 0xFF;
    mydata[19] = 0x6;
    mydata[20] = 0x65;
    mydata[21] = l >> 8;
    mydata[22] = l & 0xFF;
    mydata[23] = 0x7;
    mydata[24] = 0x0;
    mydata[25] = p_d;
    
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    debugPrintLn(F("PQ")); //Packet queued


    
      
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void wakeUp()
{
    // Just a handler for the pin interrupt.
}

void setup() {
    Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // Blink  
  delay(500); //Wait 1s in order to avoid UART programmer issues when a battery is used
  digitalWrite(13, LOW); // Blink 
  delay(250); //Wait 1s in order to avoid UART programmer issues when a battery is used
  digitalWrite(13, HIGH); // Blink 
  delay(100); //Wait 1s in order to avoid UART programmer issues when a battery is used
  digitalWrite(13, LOW); // Blink 
  delay(100); //Wait 1s in order to avoid UART programmer issues when a battery is used
  digitalWrite(13, HIGH); // Blink 
  delay(100); //Wait 1s in order to avoid UART programmer issues when a battery is used
  digitalWrite(13, LOW); // Blink 
  

    #ifdef SHOW_DEBUGINFO
  debugPrintLn(F("Starting"));
  delay(100);
  #endif
  
  Wire.begin();
  pinMode(PDPIN, INPUT);
  pinMode(VCCSOUNDPIN, OUTPUT);
  pinMode(LPPIN, OUTPUT);

  #ifdef USE_SI7021
  sensor.begin();
  #endif

  updateEnvParameters(); // To have value for the first Tx
  

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

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

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
