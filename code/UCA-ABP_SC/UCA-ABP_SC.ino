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

/*******************************************************************************/
 // Region definition (will change de frequency bands
 // Define only 1 country
 //
#define CFG_EU 1
//#define CFG_VN 1

#define SOUNDPIN A0     // what pin we're connected to
#define VCCSOUNDPIN A1   // VCC sound pin
#define PDPIN 3  // PIN with PIR Sensor Digital output

//#define USE_SOUND // Define is the sound sensor is used

//#define SHOW_DEBUGINFO

//#define DEBUG_SLEEP

/*******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "LowPower.h"
//Sensors librairies
#include <Wire.h>
#include <LTR303.h>
#include "SHTC3.h"

// Create an LTR303 object, here called "light":

LTR303 lightsensor;
SHTC3 s(Wire);

// Global variables:

// LoRaWAN end-device address (DevAddr)

static const u4_t DEVADDR = 0x260B9105;

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = {  0x20, 0xEE, 0x1A, 0x1B, 0xE4, 0xFF, 0x70, 0x14, 0x44, 0x10, 0xFF, 0x42, 0xE3, 0xD7, 0xDD, 0x88 };


// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = {  0x46, 0x2F, 0x81, 0x21, 0x5F, 0x36, 0x6A, 0x11, 0x07, 0x1E, 0xE8, 0x9F, 0x48, 0xD0, 0x17, 0x86 };


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 300;
unsigned int LONG_SLEEP = 1800;

// global enviromental parameters
 float batvalue;
 boolean presence;
 byte pres [128];
 unsigned int pres_it = 0; 
 float pres_avg = 0;
 int waiting_presence = 0; // This int is incremented if no presence is detected during a sensing slot
 int sound_treshold = 400;
 boolean presence_detected = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 8,
    .dio = {6, 6, 6},
};

// ---------------------------------------------------------------------------------
// Functions
// ---------------------------------------------------------------------------------

extern volatile unsigned long timer0_overflow_count;
extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

// sleep unleast an event happen
void do_sleep_aware(unsigned int sleepyTime) {
  unsigned int eights = sleepyTime / 8;
  
 attachInterrupt(digitalPinToInterrupt(3), wakeUp, FALLING); // Interrupt is added on PIN to detect a movement
 // if no presence detected for two slots, wait for 30mn or an event for next uplink

  #ifdef SHOW_DEBUGINFO
        Serial.print("Sleep during ");
        Serial.print(sleepyTime);
        Serial.println("sec");
        delay(50);
        Serial.end();
   #endif
        
          for ( int x = 0; x < eights; x++) {
            // put the processor to sleep for 8 seconds
            LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
              if (  waiting_presence == 0) { // if a movement is detected, send an uplink and move back to normal mode
                 return;
                  }
            
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
              sei();
          }         

 #ifdef SHOW_DEBUGINFO
          Serial.begin(9600);
          #endif
}

// sleep unleast an event happen
void do_sleep(unsigned int sleepyTime) {
  unsigned int eights = sleepyTime / 8;
  unsigned int fours = (sleepyTime % 8) / 4;
  unsigned int twos = ((sleepyTime % 8) % 4) / 2;
  unsigned int ones = ((sleepyTime % 8) % 4) % 2;
 
  #ifdef SHOW_DEBUGINFO
        Serial.print("Sleep during ");
        Serial.print(sleepyTime);
        Serial.println("sec");
        delay(50);
        Serial.end();
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
             
            
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
              sei();
          }
          for ( int x = 0; x < fours; x++) {
            // put the processor to sleep for 4 seconds
            LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 4 * 64 * clockCyclesPerMicrosecond();
              sei();
          }
          for ( int x = 0; x < twos; x++) {
            // put the processor to sleep for 2 seconds
            LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+= 2 * 64 * clockCyclesPerMicrosecond();
              sei();
          }
          for ( int x = 0; x < ones; x++) {
            // put the processor to sleep for 1 seconds
            LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
            // LMIC uses micros() to keep track of the duty cycle, so
              // hack timer0_overflow for a rude adjustment:
              cli();
              timer0_overflow_count+=  64 * clockCyclesPerMicrosecond();
              sei();
          }
          addMillis(sleepyTime * 1000);

          Serial.begin(9600);
}


// ReadVcc function to read MCU Voltage
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

// Read Light function for 
int readLight() { 
        double result;
        unsigned int data0, data1;
        lightsensor.getData(data0,data1);
    
    // Perform lux calculation:
    lightsensor.getLux(0,1,data0,data1,result);  
            return result;
}

int readSound() {
 
 digitalWrite(VCCSOUNDPIN, HIGH); // Switch on Sound sensor
    LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
    int s_avg = 1032-analogRead(SOUNDPIN);
    for (int i=1;i<10;i++){
    s_avg =s_avg + 1032-analogRead(SOUNDPIN);//(int) 1 * sound_avg; // Cayenne analog output is 0.01 Signed
    delay(2);
    }
    digitalWrite(VCCSOUNDPIN, LOW); // Switch on Sound sensor
    s_avg=s_avg/10;   
    
    return s_avg;
        
}

void onEvent (ev_t ev) {
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
             #ifdef SHOW_DEBUGINFO        
            Serial.println(F("EV_TXCOMPLETE"));
            #endif
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
              Serial.print(F("0"));
              }
              Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
              Serial.println("");
            }
            // Schedule next transmission
            
            os_setTimedCallback(&sendjob,os_getTime()+sec2osticks(1), do_send);

            #ifdef DEBUG_SLEEP
            delay(5000);
            digitalWrite(13, HIGH); // Blink  
            delay(2000); //Wait 1s in order to avoid UART programmer issues when a battery is used
            digitalWrite(13, LOW); // Blink 
            #else         
            do_sleep(TX_INTERVAL);
            #endif
                        
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


void do_send(osjob_t* j){

  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        //Serial.println(F("OP_TXRXPEND"));
    } 
    
    else {  
   
    
      if ( pres_avg == 0 ) {
      
      waiting_presence++; // if no presence is detected, activate the waiting presence mode
        }
      else {
      waiting_presence = 0; // if a presence is detected during the slot, start again the waiting presence counter
        }


    if(waiting_presence == 2){ // if no movement is detected on the second slot, do not send uplink and move to sleep aware mode

       #ifdef DEBUG_SLEEP
        delay(5000);
            
       #else
      do_sleep_aware(LONG_SLEEP-TX_INTERVAL); // go to sleep aware mode 

      #endif
          
      }
      
    pres [pres_it] = digitalRead(PDPIN); // It is important to read the PIR Value before switching any output PIN due to noise sensitivity of the PIR
    pres_it++;
     // Compute average from PIR SR-501
   long somme = pres[pres_it-1];
    for (int i = 1 ; i < pres_it ; i++)
    {
        somme += (int)pres[i] ; //somme des valeurs (db) du tableau
    }
    
    pres_avg = (float)somme / ((float)(pres_it)) ; //valeur moyenne
    pres_it = 0; // reset presence counter

    if (pres_avg > 0 ){ // Sense presence based on a sound detected or a PIR sense
      presence_detected = 1;
    }

    else {
      presence_detected = 0;
    }

    s.begin(true);
    int t = (int)(s.readTempC()*10);
    int h = (int)(s.readHumidity()*4);
    int bat = (int)(readVcc()/10); // Cayenne analog output is 0.01 Signed
    unsigned int l = (unsigned int)readLight(); // light sensor in 0.1 signed Lx
    boolean p = presence; // Presence indicator
    int p_avg = (int) 10000 * pres_avg; // Cayenne analog output is 0.01 Signed and PIR sensor will be in %

    int s_avg = readSound();
    
    boolean p_d= presence_detected; // Sound indicator

    #ifdef SHOW_DEBUGINFO
  // print out the value you read:
  Serial.print("Humidity : ");
  Serial.println(h/2);
  Serial.print("T°c : ");
  Serial.println(t/10);
  Serial.print("Light : ");
  Serial.println(l);
  Serial.print("Vbatt : ");
  Serial.println(bat);
  Serial.print("Average Presence : ");
  Serial.println(p_avg);
  Serial.print("Average Sound : ");
  Serial.println(s_avg);
  delay(100);
  #endif 

 // Build packet              
            unsigned char mydata[26];
            mydata[0] = 0x1; // CH1
            mydata[1] = 0x67; // Temp
            mydata[2] = t >> 8;
            mydata[3] = t & 0xFF;
            mydata[4] = 0x2; // CH2
            mydata[5] = 0x68; // Humidity
            mydata[6] = h & 0xFF;
            mydata[7] = 0x3; // CH3
            mydata[8] = 0x2; // Analog output
            mydata[9] = bat >> 8;
            mydata[10] = bat & 0xFF;
            mydata[11] = 0x4; // CH4
            mydata[12] = 0x65; // Luminosity
            mydata[13] = l >> 8;
            mydata[14] = l & 0xFF;
            mydata[15] = 0x5;
            mydata[16] = 0x2;
            mydata[17] = p_avg>> 8;
            mydata[18] = p_avg & 0xFF;
            mydata[19] = 0x6;
            mydata[20] = 0x2;
            mydata[21] = s_avg>> 8;
            mydata[22] = s_avg & 0xFF;
            mydata[23] = 0x7;
            mydata[24] = 0x0;
            mydata[25] = p_d;
            
            LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
      
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH); // Blink  
    delay(1000); //Wait 1s in order to avoid UART programmer issues when a battery is used
    digitalWrite(13, LOW); // Blink
    delay(1000); //Wait 1s in order to avoid UART programmer issues when a battery is used 
   
    #ifdef SHOW_DEBUGINFO
    Serial.begin(115200);
    Serial.println("Starting");
    delay(100);
    #endif
      
    Wire.begin();
    pinMode(PDPIN, INPUT);
    pinMode(VCCSOUNDPIN, OUTPUT);
    pinMode(SOUNDPIN, INPUT);
   
    s.begin(true);
    
    // Set-up sensors
    lightsensor.begin();    
    lightsensor.setPowerUp();

    //updateEnvParameters(); // To have value for the first Tx

       
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    /* This function is intended to compensate for clock inaccuracy (up to ±10% in this example), 
    but that also works to compensate for inaccuracies due to software delays. 
    The downside of this compensation is a longer receive window, which means a higher battery drain. 
    So if this helps, you might want to try to lower the percentage (i.e. lower the 10 in the above call), 
    often 1% works well already. */
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 2 / 100);

    
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

    #if defined(CFG_EU)
    // Set up the 8 channels used    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    //LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    
    #elif defined(CFG_VN)
    // Set up the 8 channels used    
    LMIC_setupChannel(0, 921400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 921600000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 921800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 922000000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 922200000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 922400000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 922600000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 922800000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 922700000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band     
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

void wakeUp()
{
    // Just a handler for the pin interrupt.
      waiting_presence = 0 ;
               pres [0] = 2;
               pres_it = 1;
               // Disable external pin interrupt on wake up pin.
              detachInterrupt(1); 
}
