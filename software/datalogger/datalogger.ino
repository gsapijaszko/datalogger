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
#define DS3231_ADDRESS     0x68      //=104 dec
#define DS3231_STATUS_REG  0x0F
#define DS3231_CONTROL_REG 0x0E
#define DS3231_TMP_UP_REG  0x11

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

void setup() {
  #ifdef RTC_VCC_PIN
    pinMode(RTC_VCC_PIN, OUTPUT);// IF you are pin-powering the chip:
    digitalWrite(RTC_VCC_PIN, HIGH); // driving this high supplies power to the RTC Vcc pin while arduino is awake
  #endif

  #ifdef MY_DEBUG
    Serial.begin(9600);
  #endif

    setSyncProvider(RTC.get);   // the function to get the time from the RTC
    Serial.print(F("RTC sync "));
    if (timeStatus() == timeSet)
        Serial.println(F("OK"));
    else
        Serial.println(F("FAIL!"));

  //turn on internal pullups for three SPI lines to help some SD cards go to sleep faster
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH); //pullup the CS pin on the SD card (but only if you dont already have a hardware pullup on your module)
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH); //pullup the MOSI pin on the SD card
  pinMode(12, INPUT);
  digitalWrite(12, HIGH); //pullup the MISO pin on the SD card
  // NOTE: In Mode (0), the SPI interface holds the CLK line low when the bus is inactive, so DO NOT put a pullup on it.
  // NOTE: when the SPI interface is active, digitalWrite() cannot effect MISO,MOSI,CS or CLK

  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, HIGH);     // used for error state, SD card writes & low voltage warning
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, LOW);  // indicates sensor reading
  pinMode(BLUE_PIN, OUTPUT);
  digitalWrite(BLUE_PIN, LOW);    // eeprom writes & early low voltage warning


  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  //pinMode(BATTERY_SENSE_PIN, INPUT);
}

void loop()
{
    digitalWrite(RTC_VCC_PIN, HIGH);
    delay(50);
    time_t t = RTC.get();
    digitalWrite(RTC_VCC_PIN, LOW);
    
    char buf[25];
        sprintf(buf, "%.4d-%.2d-%.2d %.2d:%.2d:%.2d",
            year(t), month(t), day(t), hour(t), minute(t), second(t));
        Serial.println(buf);

    int batVolt = getBatteryVoltage();
    Serial.print("Battery voltage: ");
    Serial.print(batVolt);
    Serial.print("   ");
    Serial.print(batVolt/1000);
    Serial.print(".");
    Serial.print(batVolt%1000);
    Serial.println(" V");
    
    int Vcc = readVcc();
    Serial.print("Vcc = ");
    Serial.print(Vcc/1000);
    Serial.print(".");
    Serial.print(Vcc%1000);
    Serial.println(" V");

    delay(5000);
    
}




long getBatteryVoltage() {
  digitalWrite(ENABLE, HIGH);
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
