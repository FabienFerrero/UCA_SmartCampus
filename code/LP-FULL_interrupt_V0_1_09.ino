
/*******************************************************************************
   Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   
   This uses OTAA (Over-the-air activation), where where a DevEUI and
   application key is configured, which are used in an over-the-air
   activation procedure where a DevAddr and session keys are
   assigned/generated for use with all further communication.

   Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
   g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
   violated by this sketch when left running for longer)!

   To use this sketch, first register your application and device with
   the things network, to set or generate an AppEUI, DevEUI and AppKey.
   Multiple devices can use the same AppEUI, but each device has its own
   DevEUI and AppKey.

   Do not forget to define the radio type correctly in config.h.

/*******************************************************************************
 This exemples has been modified by Fabien Ferrero to work on UCA board 
 and to send various sensors payload
 ****************************************************************************************
 */



#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "LowPower.h"

//Sensors librairies
#include <SI7021.h>

//Define sensor PIN


#define PDPIN 3  // PIN with PIR Sensor Digital output
#define SOUNDPIN A3     // what pin we're connected to
#define VCCSOUNDPIN 4   // VCC sound pin
#define LAPIN A0 // PIN with Light sensor analog output 
#define LPPIN 5 // PIN with Light power input


#define debugSerial Serial
#define SHOW_DEBUGINFO
#define debugPrintLn(...) { if (debugSerial) debugSerial.println(__VA_ARGS__); }
#define debugPrint(...) { if (debugSerial) debugSerial.print(__VA_ARGS__); }

#define USE_SI7021 // Define is the SI7021 sensor is used
#define USE_SOUND // Define is the sound sensor is used



//Commented out keys have been zeroed for github

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0xBA, 0xB1, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x09, 0x00, 0x01, 0x00, 0x00, 0x1A, 0xFF, 0x50 };
void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x36, 0x5C, 0x03, 0x50, 0xBB, 0xEE, 0x35, 0x55, 0x23, 0x06, 0x77, 0x6E, 0x8A, 0xCF, 0xE3, 0x3C };
void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}

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

void setDataRate() {
  switch (LMIC.datarate) {
    case DR_SF12:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF12"));
    #endif      
      TX_INTERVAL = 1800;
      LONG_SLEEP = 1800;
      break;
    case DR_SF11: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF11"));
    #endif
      TX_INTERVAL = 1800;
      LONG_SLEEP = 1800;
      break;
    case DR_SF10: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF10"));
    #endif
      TX_INTERVAL = 1200;
      LONG_SLEEP = 1800;
      break;
    case DR_SF9: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF9"));
    #endif
      TX_INTERVAL = 600;
      LONG_SLEEP = 1800;
      break;
    case DR_SF8: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF8"));
    #endif
      TX_INTERVAL = 300;
      LONG_SLEEP = 1800;
      break;
    case DR_SF7: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF7"));
    #endif
      TX_INTERVAL = 300;
      LONG_SLEEP = 1800;
      break;
    case DR_SF7B: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: SF7B"));
    #endif
      TX_INTERVAL = 180;
      break;
    case DR_FSK: 
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("Datarate: FSK"));
    #endif
      TX_INTERVAL = 180;
      break;
    default: debugPrint(F("Datarate Unknown Value: "));
      debugPrintLn(LMIC.datarate); TX_INTERVAL = 600;
      break;
  }
}

extern volatile unsigned long timer0_millis;
void addMillis(unsigned long extra_millis) {
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis += extra_millis;
  SREG = oldSREG;
  sei();
}

// Function do_sleep_ware : sleep for sleepyTime unleast an even happen
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
  #ifdef SHOW_DEBUGINFO
  Serial.print(os_getTime());
  Serial.print(": ");
  #endif
  switch (ev) {
    case EV_SCAN_TIMEOUT:
    #ifdef SHOW_DEBUGINFO
  debugPrintLn(F("EV_SCAN_TIMEOUT"));
  #endif
     
      break;
    case EV_BEACON_FOUND:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_BEACON_FOUND"));
    #endif      
      break;
    case EV_BEACON_MISSED:
      //debugPrintLn(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //debugPrintLn(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOINING"));
    #endif      
      break;
    case EV_JOINED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOINED"));
    #endif      
      setDataRate();      
      // Ok send our first data in 10 ms
      os_setTimedCallback(&sendjob, os_getTime() + ms2osticks(10), do_send);
      break;
    case EV_RFU1:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_RFU1"));
    #endif
      
      break;
    case EV_JOIN_FAILED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_JOIN_FAILED"));
    #endif
      
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_REJOIN_FAILED:
    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_REJOIN_FAILED"));
    #endif
      
      lmicStartup(); //Reset LMIC and retry
      break;
    case EV_TXCOMPLETE:

    #ifdef SHOW_DEBUGINFO
    debugPrintLn(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    #endif
      
      if (LMIC.txrxFlags & TXRX_ACK)
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("Received ack"));
      #endif
        
      if (LMIC.dataLen) {
        #ifdef SHOW_DEBUGINFO
        debugPrintLn(F("Received "));
        debugPrintLn(LMIC.dataLen);
        debugPrintLn(F(" bytes of payload"));
        #endif 
       
      }
            // Schedule next transmission
      setDataRate();
        
      if (waiting_presence > 1){
        do_sleep_aware(LONG_SLEEP);
        }
      else {
      do_sleep(TX_INTERVAL);}
      os_setCallback(&sendjob, do_send);
      break;
    case EV_LOST_TSYNC:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_LOST_TSYNC"));
      #endif      
      break;
    case EV_RESET:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_RESET"));
      #endif        
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_RXCOMPLETE"));
      #endif      
      break;
    case EV_LINK_DEAD:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_LINK_DEAD"));
      #endif       
      break;
    case EV_LINK_ALIVE:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("EV_LINK_ALIVE"));
      #endif       
      break;
    default:
      #ifdef SHOW_DEBUGINFO
      debugPrintLn(F("Unknown event"));
      #endif      
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

void lmicStartup() {
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setLinkCheckMode(1);
  LMIC_setAdrMode(1);
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // Increase window time for clock accuracy problem
  
  

  // Start job (sending automatically starts OTAA too)
  // Join the network, sending will be
  // started after the event "Joined"
  LMIC_startJoining();
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
  
  Serial.begin(115200);
  
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
    
  // LMIC init
  os_init();
  lmicStartup();

}

void loop() {
  os_runloop_once();
}

