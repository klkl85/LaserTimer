#include <OneWire.h>
#include <Adafruit_RGBLCDShield.h>
#include <math.h>
#include "FastLED.h"

// A0 == LASER FIRE SENSOR
// A4 == SDA (Green)
// A5 == SCL (Yellow)
// D4 == Status LED Strip

#define PinSDA A4
#define PinSCL A5
#define PinLaserFire A0
#define PinLED 4

//byte arrowNE[8] = {B0, B1111, B11, B101, B1001, B10000, B0, B0};
//byte arrowSE[8] = {B0, B10000, B1001, B101, B11, B1111, B0, B0};
//byte arrowSW[8] = {B0, B1, B10010, B10100, B11000, B11110, B0, B0};
//byte arrowNW[8] = {B0, B11110, B11000, B10100, B10010, B1, B0, B0};

byte poundSign[8] = {B111, B1000, B11100, B1000, B1000, B10000, B11111, B0};
byte laser1[8] = {B11111, B1010, B1110, B0, B0, B0, B0, B0}; // Module Blank
byte laser2[8] = {B11111, B1010, B1110, B100, B100, B0, B0, B0}; // Module Fire 1
byte laser3[8] = {B11111, B1010, B1110, B0, B100, B100, B0, B0}; // Module Fire 2
byte laser4[8] = {B11111, B1010, B1110, B0, B0, B100, B100, B0}; // Module Fire 3
byte laser5[8] = {B11111, B1010, B1110, B0, B0, B1010, B100, B0}; // Module Explode
byte laser6[8] = {B11111, B1010, B1110, B0, B0, B0, B0, B0}; // Module Blank

unsigned long lastUpdate = 0;                                 // keeps track of when to update the screen
unsigned long animationTimer = 0;                             // keeps track of when to update the laser firing animation
int animationCounter = 0;                                     // used to determine which animation glyph to display
bool BOOL_onJob = false;                                      // display prep flag
bool BOOL_firstRun = true;                                    // which screen to display
float cost = 0.0;                                             // keeps track of cumulative cost of laser use
float runningTime = 0.0;                                      // keeps track of laser firing time
float costRatePerMinute = 15.0;                              // COST PER MINUTE IN PENCE
int PollingIntervalMS = 50;                                  // How often to check for laser firing
float pollsPerMinute = (60.0 * 1000.0) / PollingIntervalMS;   // Used to calculate incremental cost
float costPerPoll = (costRatePerMinute / pollsPerMinute);     // Incremental cost
CRGB leds[3];

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();          // LCD Sheild Prep

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT_PULLUP);                                  // PIN for testing laser fire
  lcd.begin(16, 2);
  lcd.createChar(7, poundSign);                               // LCD Character Glyphs
  lcd.createChar(0, laser1);
  lcd.createChar(1, laser2);
  lcd.createChar(2, laser3);
  lcd.createChar(3, laser4);
  lcd.createChar(4, laser5);
  lcd.createChar(5, laser6);
  FastLED.addLeds<NEOPIXEL, PinLED>(leds, 3);                 // Status LED Stuffs
  FastLED.setBrightness(40);
  leds[2] = CRGB(0, 0, 255);
  FastLED.show();
}

void loop() {
  // Update LEDS
  if (cost > 0 || runningTime > 0) {
    leds[0] = CRGB(0, 255, 0);
  }
  if (isLaserFiring()) {
    leds[1] = CRGB(255, 0, 0);
  } else {
    leds[1] = CRGB(0, 0, 0);
  }

  // Do the thing
  if (BOOL_firstRun) {
    screenStandby();
    delay(100);
    BOOL_firstRun = false;
  } else {
    isLaserFiring();
    if (isLaserFiring()) {
      cost = cost + costPerPoll;
      runningTime = runningTime + (PollingIntervalMS / 1000.0);
      if (BOOL_onJob) {
        if (millis() - animationTimer > 250) {
          fireAnimation();
          animationTimer = millis();
        }
        if (millis() - lastUpdate > 1000) {
          screenUpdateCost();
          lastUpdate = millis();
        }
      } else {
        BOOL_onJob = true;
        screenOnJob();
      }
    } else {
      // WHEN LASER NOT FIRING THEN BLANK OUT THE ANIMATION
      lcd.setCursor(15, 0);
      lcd.print(" ");
    }
    // WAIT THEN RE-POLL
    FastLED.show();
    delay(PollingIntervalMS);
  }
}

// DETERMINE IF THE LASER DIODE IS IN USE (ACTUALLY FIRING)
bool isLaserFiring() {
  int laserSensorValue = analogRead(A0);
  float voltage = laserSensorValue * (5.0 / 1023.0);
  if (voltage < 3) {
    return true;
  } else {
    return false;
  }
}

// DEFAULT WELCOME SCREEN -- DISPLAYS CONFIGURED COST PER MINUTE
void screenStandby() {
  lcd.clear();
  lcd.setCursor(2, 0); // X,Y
  lcd.print("LASER  TIMER");
  lcd.setCursor(0, 1);
  lcd.print("COST ");
  lcd.setCursor(5, 1);
  lcd.write(byte(7));
  lcd.setCursor(6, 1);

  int pounds = floor(costRatePerMinute / 100);
  int pence = costRatePerMinute - (pounds * 100);
  char rateCostC[5];
  sprintf(rateCostC, "%01d.%02d", pounds, pence);
  lcd.print(rateCostC);

  lcd.setCursor(11, 1);
  lcd.print("/ min");
}

// PREP SCREEN FOR COST / TIME DISPLAY
void screenOnJob() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("COST =");
  lcd.setCursor(7, 0);
  lcd.write(byte(7));
  lcd.setCursor(0, 1);
  lcd.print("TIME =");
}

// UPDATE COST / TIME SCREEN ONLY WITH RELEVANT PARTS
void screenUpdateCost() {
// STRINGS IN C ARE NULL TERMINATED (MAKE ARRAY 1 LONGER THAN WANT TO PRINT! THANKS GREG!!!)
//  char poundsC[4]; 
//  char penceC[3];
  char costC[7];
  int jobCost = floor(cost);
  int pounds = floor(jobCost / 100);
  int pence = jobCost - (pounds * 100);
  //  Serial.println(pounds);
  //  Serial.println(pence);
//  sprintf(poundsC, "%02d.", pounds);
//  sprintf(penceC, "%02d", pence);
  sprintf(costC, "%d.%02d", pounds, pence);
  lcd.setCursor(8, 0);
  lcd.print(costC);

//  char hoursC[4];
//  char minutesC[4];
//  char secondsC[3];

  char timeC[11];

  int runTime = floor(runningTime);
  int hours = floor(runTime / (60 * 60));
  int minutes = floor(runTime / 60);
  int seconds = (runTime - (hours * 60 * 60) - (minutes * 60));
//  sprintf(hoursC, "%02d:", hours);
//  sprintf(minutesC, "%02d:", minutes);
//  sprintf(secondsC, "%02d", seconds);

  sprintf(timeC, "%02d:%02d:%02d", hours, minutes, seconds);


  lcd.setCursor(7, 1);
  lcd.print(timeC);
//  lcd.setCursor(10, 1);
//  lcd.print(minutesC);
//  lcd.setCursor(13, 1);
//  lcd.print(secondsC);
}

// LASER FIRING ANIMATION
void fireAnimation() {
  lcd.setCursor(15, 0);
  lcd.write(byte(animationCounter));
  animationCounter = (animationCounter + 1) % 6;
}
