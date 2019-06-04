/*       Data Logger
 *       developed by Grzegorz Sapijaszko
 *       
 *       
 */

#include <Wire.h>       // I2C lib needs 128 bytes of ram for Serial buffer
#include <DS3232RTC.h>      // https://github.com/JChristensen/DS3232RTC


//DS3231 RTC variables
//====================
#define RTC_VCC_PIN 5               // Assumes you are powering your DS3231 RTC from this pin. Even if you are not, it is harmless to leave this in unless you have something else connected to digital pin 7. 
////When the arduino is awake, power the rtc from this pin (70uA), when arduino sleeps pin set low & rtc runs on battery at <3uA

// Enable debug prints to serial monitor
#define MY_DEBUG

void setup() {
  #ifdef RTC_VCC_PIN
    digitalWrite(RTC_VCC_PIN, OUTPUT);// IF you are pin-powering the chip:
    pinMode(RTC_VCC_PIN, HIGH); // driving this high supplies power to the RTC Vcc pin while arduino is awake
  #endif

  #ifdef MY_DEBUG
    Serial.begin(9600);

    Serial.println(F("\n" __FILE__ " " __DATE__ " " __TIME__));
    setTime(11, 55, 00, 4, 6, 2019); 
    RTC.set(now());
    setSyncProvider(RTC.get);   // the function to get the time from the RTC
    Serial.print(F("RTC sync "));
    if (timeStatus() == timeSet)
        Serial.println(F("OK"));
    else
        Serial.println(F("FAIL!"));
  #endif

}

void loop()
{
    digitalClockDisplay();
    delay(1000);
}

void digitalClockDisplay()
{
    // digital clock display of the time
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(' ');
    Serial.print(day());
    Serial.print(' ');
    Serial.print(month());
    Serial.print(' ');
    Serial.print(year());
    Serial.println();
}

void printDigits(int digits)
{
    // utility function for digital clock display: prints preceding colon and leading 0
    Serial.print(':');
    if(digits < 10)
        Serial.print('0');
    Serial.print(digits);
}
