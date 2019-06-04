/*       Data Logger
 *       developed by Grzegorz Sapijaszko
 *       
 *       
 */

#include <Wire.h>       // I2C lib needs 128 bytes of ram for Serial buffer
#include <avr/power.h>  // I am using this for the ADC peripheral shutdown
#include <avr/sleep.h>  // used in sleepNwait4D3Interrupt & sleepNwait4RTC
#include <PString.h>    // from  http://arduiniana.org/
#include <SPI.h>
#include <SdFat.h>      // needs 512 byte ram buffer! see https://github.com/greiman/SdFat
SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const uint8_t chipSelect = 10; //sd card chip select

#include <LowPower.h>   // https://github.com/rocketscream/Low-Power and https://github.com/rocketscream/Low-Power 
// To set Duration of Rocket Screams low power sleep modes: SLEEP_15Ms,SLEEP_30MS,SLEEP_60MS,SLEEP_120MS,SLEEP_250MS,SLEEP_500MS,SLEEP_1S,SLEEP_2S,SLEEP_4S,SLEEP_8S,
// eg: LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); use SLEEP_FOREVER with hardware interrupts

#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC
//DS3231 RTC variables
//====================
#define RTC_VCC_PIN 5               // Assumes you are powering your DS3231 RTC from this pin. Even if you are not, it is harmless to leave this in unless you have something else connected to digital pin 7. 
////When the arduino is awake, power the rtc from this pin (70uA), when arduino sleeps pin set low & rtc runs on battery at <3uA
// #define DS3231_ADDRESS     0x68      //=104 dec
// #define DS3231_STATUS_REG  0x0F
// #define DS3231_CONTROL_REG 0x0E
// #define DS3231_TMP_UP_REG  0x11



// BH1750 lightmeter
#include <BH1750.h>
BH1750 lightMeter;

#include <BME280I2C.h>      // https://github.com/finitespace/BME280

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

// Enable debug prints to serial monitor
#define MY_DEBUG

// Define for LED(s)
#define RED_PIN 6
#define GREEN_PIN 7
#define BLUE_PIN 8

// Alarm input form DS3231
#define INT_SQW 2

// Battery voltage measurement pins
#define ENABLE A1
#define BATTERY_SENSE_PIN A0

#define SampleIntervalMinutes 1 //15              // Allowed values: 1,2,3,5,10,15,20 or 30, -MUST be a number that divides equally into 60! Time aligns to hour rollover no matter when unit is started
#define SampleIntervalSeconds 0              // 1Hz is maximum possible rate of recording discrete time stamped records with this logger!: 

uint8_t Alarmday = 1;
uint8_t Alarmhour = 1;
uint8_t Alarmminute = 1;
uint8_t Alarmsecond = 1; 

boolean midnightRollover = true;     // flag that triggers the saving ONCE per day data

time_t teraz = 0;

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

  // just a couple of blinks for the end of setup
  blinkDiode(RED_PIN, 2);
  blinkDiode(GREEN_PIN, 2);
  blinkDiode(BLUE_PIN, 2);  

    RTC.setAlarm(ALM1_MATCH_SECONDS, 20, 0, 0, 1);  // daydate parameter should be between 1 and 7
    RTC.alarm(ALARM_1);                   // ensure RTC interrupt flag is cleared
    RTC.alarmInterrupt(ALARM_1, true); 

}

volatile boolean alarmIsrWasCalled = false;

void alarmIsr()
{
    alarmIsrWasCalled = true;
} 

void loop()
{
  if (alarmIsrWasCalled) { 
    Serial.println(F("\n"));
    // Serial.println(F("Alarm! Alarm!"));
    
    digitalWrite(RTC_VCC_PIN, HIGH);
    LowPower.powerDown(SLEEP_15MS, ADC_ON, BOD_ON);  //note the ADC is left for this cap stabilization period
    delay(50);
    time_t t = RTC.get();
    RTC.alarm(ALARM_1);
    char buf[25];
        sprintf(buf, "%.4d-%.2d-%.2d %.2d:%.2d:%.2d",
            year(t), month(t), day(t), hour(t), minute(t), second(t));
        #ifdef MY_DEBUG
          Serial.println(buf);
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


    // Serial.print("Next Alarm: dzień: ");Serial.print(Alarmday);Serial.print(", godzina: ");Serial.print(Alarmhour);Serial.print(", minuta: ");Serial.print(Alarmminute);Serial.print(", sek: ");Serial.println(Alarmsecond);
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
      Serial.print(" V\t\t");
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
      PString str(buffer, sizeof(buffer));
      uint16_t luxa = getLightLevel();

      if (luxa < 10000) {
        str.print (F("0"));
      } 
      if (luxa < 1000) {
        str.print (F("0"));
      } 
      if (luxa < 100) {
        str.print (F("0"));
      } 
      if( luxa < 10 ) {
        str.print (F("0"));
      } 
      str.print(luxa);

      Serial.print("Light: "); Serial.print(luxa); Serial.print(F("  ")); Serial.println(str);
    
    // delay(200);

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
   BME280::PresUnit presUnit(BME280::PresUnit_hPa);

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
