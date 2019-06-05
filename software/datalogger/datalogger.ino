/*       Data Logger
 *       developed by Grzegorz Sapijaszko
 *       
 *       
 */

#include <Wire.h>                   // I2C lib needs 128 bytes of ram for Serial buffer
//#include <avr/power.h>            // I am using this for the ADC peripheral shutdown
//#include <avr/sleep.h>            // used in sleepNwait4D3Interrupt & sleepNwait4RTC
#include <PString.h>                // from  http://arduiniana.org/
#include <SPI.h>
#include <SdFat.h>                  // needs 512 byte ram buffer! see https://github.com/greiman/SdFat
SdFat sd;                           // Create the objects to talk to the SD card
SdFile file;
const uint8_t chipSelect = 10;      // sd card chip select

#include <LowPower.h>               // https://github.com/rocketscream/Low-Power and https://github.com/rocketscream/Low-Power 
// To set Duration of Rocket Screams low power sleep modes: SLEEP_15Ms,SLEEP_30MS,SLEEP_60MS,SLEEP_120MS,SLEEP_250MS,SLEEP_500MS,SLEEP_1S,SLEEP_2S,SLEEP_4S,SLEEP_8S,
// eg: LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); use SLEEP_FOREVER with hardware interrupts

#include <DS3232RTC.h>              // https://github.com/JChristensen/DS3232RTC
//DS3231 RTC variables
//====================
#define RTC_VCC_PIN 5               // Power ON/OFF pin for DS3231 RTC 
                                    // When the uControler is awake, it powers the rtc from this pin (70uA), when uController sleeps pin set low & rtc runs on battery at <3uA
// BH1750 lightmeter
#include <BH1750.h>                 // https://github.com/claws/BH1750
BH1750 lightMeter;

#include <BME280I2C.h>              // https://github.com/finitespace/BME280

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_Off,
   BME280::SpiEnable_False,
   0x76 // I2C address. I2C specific.
);

BME280I2C bme(settings);

// I2C EEPROM, 24C64 - 64 Kbit, 8 Kbyte, page size: 32 byte
// One 24C64 EEPROMs on the bus
const uint32_t totalKBytes = 8;        //for read and write test functions
#include "extEEPROM.h"
extEEPROM logEEPROM(kbits_64, 1, 32);     //device size, number of devices, page size
#define EEpromPageSize 32                 // 32 bytes is page size for the AT24C32 & AT24C256 - can't move more than 30 bytes with wire.h anyway
unsigned long CurrentEEpromMemAddress = 0;     // set to zero at the start of each cycle - must increment by steps of 32
uint16_t RolloverEEpMemAddress = 0;       // only used on midnight rollover
char EEPROMBuffer[30];                    //size=(PageSize - 2) This char array recieves a string of ascii from the pstring function 
boolean printStatusLogHeaders = false;    // flag that triggers the status logg header printing at startup
boolean midnightRollover = true;          // flag that triggers the saving ONCE per day data

uint16_t SamplesPerCycle = 64;             // AUTOADJUSTS later based on the eeprom being used as a buffer & number of pages buffered

#define MY_DEBUG                    // Enable debug prints to serial monitor

// Define for LED(s)
#define RED_PIN 6
#define GREEN_PIN 7
#define BLUE_PIN 8

// Alarm input form DS3231
#define INT_SQW 2

// Battery voltage measurement pins
#define ENABLE A1
#define BATTERY_SENSE_PIN A0

#define SampleIntervalMinutes 0 //15              // Allowed values: 1,2,3,5,10,15,20 or 30, -MUST be a number that divides equally into 60! Time aligns to hour rollover no matter when unit is started
#define SampleIntervalSeconds 10              // 1Hz is maximum possible rate of recording discrete time stamped records with this logger!: 

uint8_t Alarmday = 1;
uint8_t Alarmhour = 1;
uint8_t Alarmminute = 1;
uint8_t Alarmsecond = 1; 

time_t teraz = 0;

//TimeStamps created with sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d",t_year,t_month,t_day,t_hour,t_minute);
char CycleTimeStamp[] = "0000-00-00,00:00"; //16 characters without seconds!


uint8_t PagesBuffered2Eeprom = 2;          //default value for 1st buffer dump on startup only! If you put the wrong number here it AUTOADJUSTS to match the ACTUAL pages buffered later
uint8_t chunkSize = 64;    //this can be changed, but must be a multiple of 4 since we're writing 32-bit integers


void setup() {
  #ifdef RTC_VCC_PIN
    pinMode(RTC_VCC_PIN, OUTPUT);// IF you are pin-powering the chip:
    digitalWrite(RTC_VCC_PIN, HIGH); // driving this high supplies power to the RTC Vcc pin while arduino is awake
  #endif
  // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, false);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE); 

  pinMode(INT_SQW, INPUT_PULLUP);
  attachInterrupt(INT0, alarmIsr, FALLING);

  #ifdef MY_DEBUG
    Serial.begin(9600);
  #endif
  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  #ifdef MY_DEBUG
    Serial.print(F("RTC sync "));
    if (timeStatus() == timeSet)
        Serial.println(F("OK"));
    else
        Serial.println(F("FAIL!"));
  #endif

  RTC.setAlarm(ALM1_MATCH_SECONDS, 10, 0, 0, 1);  // daydate parameter should be between 1 and 7
  RTC.alarm(ALARM_1);                   // ensure RTC interrupt flag is cleared
  RTC.alarmInterrupt(ALARM_1, true); 

  //turn on internal pullups for three SPI lines to help some SD cards go to sleep faster
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH); //pullup the CS pin on the SD card (but only if you dont already have a hardware pullup on your module)
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH); //pullup the MOSI pin on the SD card
  pinMode(12, INPUT);
  digitalWrite(12, HIGH); //pullup the MISO pin on the SD card
  // NOTE: In Mode (0), the SPI interface holds the CLK line low when the bus is inactive, so DO NOT put a pullup on it.
  // NOTE: when the SPI interface is active, digitalWrite() cannot effect MISO,MOSI,CS or CLK

  pinMode(RED_PIN, OUTPUT);      // used for error state, SD card writes & low voltage warning
  pinMode(GREEN_PIN, OUTPUT);     // indicates sensor reading
  pinMode(BLUE_PIN, OUTPUT);      // eeprom writes & early low voltage warning

  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  //pinMode(BATTERY_SENSE_PIN, INPUT);

  // light meter start
  lightMeter.begin();
  
  // BME280
  while(!bme.begin())
  {
    #ifdef MY_DEBUG
      Serial.println("Could not find BME280I2C sensor!");
    #endif
    blinkDiode(RED_PIN, 3);
    delay(1000);
  }


 byte i2cStat = logEEPROM.begin(logEEPROM.twiClock100kHz);
  if ( i2cStat != 0 ) {
    #ifdef MY_DEBUG
      Serial.println(F("I2C Problem"));
    #endif
    blinkDiode(RED_PIN, 4);
    delay(1000);
  }



/////
PagesBuffered2Eeprom = 0; //do not zero this again before SamplesPerCycle is calculated!

  digitalWrite(BLUE_PIN, HIGH);
  eeErase(chunkSize, 0, 8 * 1024 - 1);
  digitalWrite(BLUE_PIN, LOW);
  delay(100);
  
  // just a couple of blinks for the end of setup
  blinkDiode(RED_PIN, 2);
  blinkDiode(GREEN_PIN, 2);
  blinkDiode(BLUE_PIN, 2);  

}

volatile boolean alarmIsrWasCalled = false;

void alarmIsr()
{
    alarmIsrWasCalled = true;
} 

void loop()
{
  if (alarmIsrWasCalled) {
    if ( CurrentEEpromMemAddress < totalKBytes * 1024  - 3*EEpromPageSize ) {
      
    }
    else {
        Serial.println(F("Zrzut:"));
        for (unsigned long i = 0; i < totalKBytes * 1024 - 1; i+=32) {
          logEEPROM.read(i, EEPROMBuffer, sizeof(EEPROMBuffer));
          Serial.print(i); Serial.print(F(" :  "));Serial.println(EEPROMBuffer);
        }

      eeErase(chunkSize, 0, 8 * 1024 - 1);
      CurrentEEpromMemAddress = 0;
    }
        #ifdef MY_DEBUG
          Serial.println(F("\n"));
          // Serial.println(F("Alarm! Alarm!"));
        #endif
        
        digitalWrite(RTC_VCC_PIN, HIGH);
        LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_ON);  //note the ADC is left for this cap stabilization period
        delay(50);
        time_t t = RTC.get();
        RTC.alarm(ALARM_1);
        sprintf(CycleTimeStamp, "%04d-%02d-%02d %02d:%02d", year(t), month(t), day(t), hour(t), minute(t));
          #ifdef MY_DEBUG
            Serial.print(CycleTimeStamp); Serial.print("\t");
          #endif
    
        Alarmhour = hour(t); 
        Alarmminute = minute(t) + SampleIntervalMinutes;
        // Make sure your sensor readings don't take longer than your sample interval or you pass your next alarm time & clock fails!
        Alarmday = day(t); 
        Alarmsecond = second(t) + SampleIntervalSeconds; //only used for special testing & debugging runs - ignored unless SampleIntervalMinutes=0
    
        // Check for RTC TIME ROLLOVERS: THEN SET the next RTC alarm and go back to sleep
        //============================================================================
        if (SampleIntervalMinutes > 0) //then our alarm is in (SampleInterval) minutes
            {
              if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
                Alarmminute = 0;
                Alarmhour = Alarmhour + 1;
                if (Alarmhour > 23) {
                  Alarmhour = 0;
                  midnightRollover = true;
                }
              }  //terminator for if (Alarmminute > 59) rollover catching
        
            }  //terminator for if (SampleIntervalMinutes > 0)
        
        else  //to get sub-minute alarms use the full setA1time function
            
            {  // for testing & debug I sometimes want the alarms more frequent than 1 per minute.
              if (Alarmsecond >59){
                Alarmsecond =0;
                Alarmminute = Alarmminute+1;  
                if (Alarmminute > 59) 
                {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
                  Alarmminute =0; 
                  Alarmhour = Alarmhour+1; 
                  if (Alarmhour > 23) { //uhoh a day rollover, but we dont know the month..so we dont know the next day number?
                    Alarmhour =0; 
                    midnightRollover = true;      
                     // sleep for a total of 64 seconds (12 x 8s) to force day "rollover" while we are in this loop
                     // this causes a small gap in the timing once per day, but I only use sub minute sampling for debugging anyway.
                     for (int j = 0; j <12; j++){
                        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
                     }
                     Alarmday = day(t);
                     Alarmhour = hour(t);
                     Alarmminute = minute(t);
                     Alarmsecond = second(t) + SampleIntervalSeconds;
                  }
                }
              }
              
              //The sample interval must ALWAYS be greater than the time to acquire samples and flush eeprom buffer data to the SD PLUS ~1 second for SD write latency
              // ====>>> RTC_DS3231_setA1Time(Alarmday, Alarmhour, Alarmminute, Alarmsecond, 0b00001000, false, false, false);  
              //The variables ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b111 respectively.
              //RTC_DS3231_setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM)
        } // terminator for second else case of if (SampleIntervalMinutes > 0) 
        
        #ifdef MY_DEBUG
          Serial.print("Next Alarm: dzień: ");Serial.print(Alarmday);Serial.print(", godzina: ");Serial.print(Alarmhour);Serial.print(", minuta: ");Serial.print(Alarmminute);Serial.print(", sek: ");Serial.println(Alarmsecond);
        #endif
        RTC.setAlarm(ALM1_MATCH_HOURS, Alarmsecond, Alarmminute, Alarmhour, 0);
        RTC.alarm(ALARM_1);                   // ensure RTC interrupt flag is cleared
        RTC.alarmInterrupt(ALARM_1, true); 
        delay(50);
        digitalWrite(RTC_VCC_PIN, LOW);
      
        int batVolt = getBatteryVoltage();
        #ifdef MY_DEBUG
          Serial.print("Battery voltage: ");
          Serial.print(batVolt);
          Serial.print("   ");
          Serial.print(batVolt/1000);
          Serial.print(".");
          Serial.print(batVolt%1000);
          Serial.print(" V\t");
        #endif
        int Vcc = readVcc();
        #ifdef MY_DEBUG
          Serial.print("Vcc = ");
          Serial.print(Vcc/1000);
          Serial.print(".");
          Serial.print(Vcc%1000);
          Serial.println(" V");
        #endif
    
          char buffer[6];
          PString stra(buffer, sizeof(buffer));
          uint16_t luxa = getLightLevel();
    
          if (luxa < 10000) {
            stra.print (F("0"));
          } 
          if (luxa < 1000) {
            stra.print (F("0"));
          } 
          if (luxa < 100) {
            stra.print (F("0"));
          } 
          if( luxa < 10 ) {
            stra.print (F("0"));
          } 
          stra.print(luxa);
    
          Serial.print("Light: "); Serial.print(luxa); Serial.print(F("  ")); Serial.print(stra); Serial.print("\t");
        
        // BME280
        settings.mode = BME280::Mode_Forced;
        bme.setSettings(settings);
        delay(50);
        #ifdef MY_DEBUG  
          printBME280Data(&Serial);
        #endif
        settings.mode = BME280::Mode_Sleep;
        bme.setSettings(settings);
    
      
        PagesBuffered2Eeprom = 0; //do not zero this again before SamplesPerCycle is calculated!
    
        PString str(EEPROMBuffer, sizeof(EEPROMBuffer));
        str = "";                                       //30 character max payload:  wires buffer is 32 bytes, but 2 get used for bus&reg addresses
        str.print(CycleTimeStamp); str.print(F(","));   //17 / 16 characters without seconds plus comma
        str.print(batVolt); str.print(F(","));          //5 char
        str.print(Vcc); str.print(F(","));              //5 char
    //    str.print(TEMP_Raw);  //"raw" 12-bit int. for most tempsensors, 4095 is largest value =4 charaters +1 for comma  //RTC is only 3 characters +1
        //str.print(rtc_TEMP_Raw);str.print(F(","));  //without a second temp sensor rtc_TEMP is TEMP_Raw
        str.print(F("      "));  //NO trailing comma on last data in string - this fills the end of the buffer with blank spaces - Pstring ignores any excess so will not generate an overflow error
    
    //    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentEEpromMemAddress, EEPROMBuffer); // whole page is written at once here
    
        Serial.println(str);
        Serial.print("Adres: "); Serial.println(CurrentEEpromMemAddress);
        delay(100);
        
        digitalWrite(BLUE_PIN, HIGH);
        delay(50);
    
        byte i2cStat = 0;
        i2cStat = logEEPROM.write(CurrentEEpromMemAddress, EEPROMBuffer, sizeof(EEPROMBuffer));
        if ( i2cStat != 0 ) {
          //there was a problem
          Serial.print(F("I2C Problem: "));
            if ( i2cStat == EEPROM_ADDR_ERR) {
            Serial.println(F("Wrong address"));
            } else {
            Serial.print(F("I2C error: "));
            Serial.print(i2cStat);
            Serial.println(F(""));
          }
        }
/*
        PString(EEPROMBuffer, sizeof(EEPROMBuffer), "");
        Serial.print("A teraz z pamieci, adress = "); Serial.println(CurrentEEpromMemAddress);
        logEEPROM.read(CurrentEEpromMemAddress, EEPROMBuffer, sizeof(EEPROMBuffer));
        Serial.println(EEPROMBuffer);
        Serial.println("Zrzut:");
        for (byte i = 0; i <= CurrentEEpromMemAddress ; i+=32) {
          logEEPROM.read(i, EEPROMBuffer, sizeof(EEPROMBuffer));
          Serial.println(EEPROMBuffer);
        }
*/        
        CurrentEEpromMemAddress += EEpromPageSize; 
        PagesBuffered2Eeprom++;
        Serial.print("Nowy adres: "); Serial.println(CurrentEEpromMemAddress);
    
        digitalWrite(BLUE_PIN, LOW);


    
        blinkDiode(GREEN_PIN, 2);
        delay(1200);
    
  //      dump(0, CurrentEEpromMemAddress);            //the first 32 bytes
     
        alarmIsrWasCalled = false; 
      
  }
  else {
   LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
   // tuStandard();
 
  } // end of else
}

void tuStandard() {
   digitalWrite(RTC_VCC_PIN, HIGH);
    delay(50);
    time_t t = RTC.get();
    if (t - teraz >= 17) {
      RTC.alarm(ALARM_1);
      char buf[25];
          sprintf(buf, "%.4d-%.2d-%.2d %.2d:%.2d:%.2d",
              year(t), month(t), day(t), hour(t), minute(t), second(t));
          #ifdef MY_DEBUG
            Serial.println(buf);
          #endif
    digitalWrite(RTC_VCC_PIN, LOW);
    delay(50);
  
  
  
    // BME280
    settings.mode = BME280::Mode_Forced;
    bme.setSettings(settings);
    delay(50);
    #ifdef MY_DEBUG  
      printBME280Data(&Serial);
    #endif
    settings.mode = BME280::Mode_Sleep;
    bme.setSettings(settings);
    teraz = t;
    } // end if t-teraz
}

uint16_t getLightLevel() {
  lightMeter.configure(BH1750_POWER_ON);
  lightMeter.configure(BH1750_ONE_TIME_LOW_RES_MODE);
  uint16_t lux = lightMeter.readLightLevel();
  lightMeter.configure(BH1750_POWER_DOWN);
  return lux;
}

void printBME280Data (Stream* client)
{
   float temp(NAN), hum(NAN), pres(NAN);

   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_Pa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   client->print(temp);
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   client->print(hum);
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println(" hPa");

//   delay(1000);
}


long getBatteryVoltage() {
  digitalWrite(ENABLE, HIGH);
//  LowPower.powerDown(SLEEP_30MS, ADC_ON, BOD_ON);  //note the ADC is left for this cap stabilization period
  delay(50);
  int batterySenseValue = analogRead(BATTERY_SENSE_PIN);
  digitalWrite(ENABLE, LOW);
  // (330e3+100e3)/100e3 - resistors divider
  float batteryVoltage = batterySenseValue * (330e3+100e3)/100e3 / 1023;
  batteryVoltage = batteryVoltage * readVcc();
  return batteryVoltage; // returns batteryVoltage in mV
}



long readVcc() {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
    #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
      ADMUX = _BV(MUX5) | _BV(MUX0);
    #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
      ADMUX = _BV(MUX3) | _BV(MUX2);
    #else
      ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    #endif  
   
    delay(5); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA,ADSC)); // measuring
   
    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
    uint8_t high = ADCH; // unlocks both
   
    long result = (high<<8) | low;
   
    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
  }

  void blinkDiode(int pin, int times) {
    for (int i = 1; i<=times; i++) {
      digitalWrite(pin, HIGH);     
      delay(200);
      digitalWrite(pin, LOW);
      delay(300);
    }
  }


  //dump eeprom contents, 16 bytes at a time.
//always dumps a multiple of 16 bytes.
void dump(uint32_t startAddr, uint32_t nBytes)
{
  Serial.print(F("EEPROM DUMP 0x"));
  Serial.print(startAddr, HEX);
  Serial.print(F(" 0x"));
  Serial.print(nBytes, HEX);
  Serial.print(F(" "));
  Serial.print(startAddr);
  Serial.print(F(" "));
  Serial.println(nBytes);
  uint32_t nRows = (nBytes + 15) >> 4;

  uint8_t d[16];
  for (uint32_t r = 0; r < nRows; r++) {
    uint32_t a = startAddr + 16 * r;
    logEEPROM.read(a, d, 16);
    Serial.print(F("0x"));
    if ( a < 16 * 16 * 16 ) Serial.print(F("0"));
    if ( a < 16 * 16 ) Serial.print(F("0"));
    if ( a < 16 ) Serial.print(F("0"));
    Serial.print(a, HEX); Serial.print(F(" "));
    for ( int c = 0; c < 16; c++ ) {
 //     if ( d[c] < 16 ) {
//        Serial.print(F("0"));
        Serial.print(d[c], HEX);
        Serial.print( c == 7 ? "  " : " ");
//      }
    }
    Serial.println(F(""));
  }
}


//write 0xFF to eeprom, "chunk" bytes at a time
void eeErase(uint8_t chunk, uint32_t startAddr, uint32_t endAddr)
{
  chunk &= 0xFC;                //force chunk to be a multiple of 4
  uint8_t data[chunk];
  Serial.println(F("Erasing..."));
  for (int i = 0; i < chunk; i++) data[i] = 0xFF;
  uint32_t msStart = millis();

  for (uint32_t a = startAddr; a <= endAddr; a += chunk) {
    if ( (a & 0xFFF) == 0 ) Serial.println(a);
    logEEPROM.write(a, data, chunk);
  }
  uint32_t msLapse = millis() - msStart;
  Serial.print(F("Erase lapse: "));
  Serial.print(msLapse);
  Serial.print(F(" ms"));
}
