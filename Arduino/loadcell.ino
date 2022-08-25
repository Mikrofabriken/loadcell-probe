/*

  A load cell probe for Plasma CNC at Mikrofabriken by Ulrik Holmén
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

*/

#include "HX711.h"
#include "LiquidCrystal.h"
// #include "Wire.h"

// Thresholds for detection
int zero_thrshld = 0;
int touch_thrshld = 0;
int limit_thrshld = 0;
int touchdir = 1; // Only report an up tilting touch

/* Sensitivity levels 
 *  Just a guessing game so far, will have to be adjusted
 *  to reality
 */
int senslevel = 0; // 3 separate sensitivity levels, starting at 0
char *sens_strings[3] = {"Low", "Medium", "High"};
int sens_zero[3] = { 5, 10, 15 }; 
int sens_touch[3] = { 25, 35, 50 };
int sens_limit[3] = { 100, 115, 130};

// Setup load cell 1 and 2 via HX711 
const int SCALE1_LOADCELL_DOUT_PIN = 3;
const int SCALE1_LOADCELL_SCK_PIN = 2;
const int SCALE2_LOADCELL_DOUT_PIN = 5;
const int SCALE2_LOADCELL_SCK_PIN = 4;
HX711 scale1;
HX711 scale2;
int unit_scale = 10000;

// lcd
LiquidCrystal lcd(A5, A4, A3, A2, A1, A0);
const int indicator_time = 1000;

// led
const int RED_PIN = 6;
const int GREEN_PIN = 7;
const int BLUE_PIN = 8;

// buttons
const int btn_resetPin = 10;
const int btn_sensPin = 9;
const int probe_pin = 13;

// relay
const int relay_touch_pin = 11;
const int relay_limit_pin = 12;

float progress_factor = 0.0;
float max_factor = 16 * 5;

// load cell reading holders
long reading1 = 0;
long reading2 = 0;
long reading = 0;
long prev_reading = 0;

// States
bool scale_over_zero = false;
bool scale_over_touch = false;
bool scale_over_limit = false;
bool limit = false;
bool touch = false;
bool tilt = false;
bool probing = false;
int probe_reading = LOW;

// 0 = no dir, 1 = up, 2 = down, 3 = left, 4 = right
int tiltdir = 0;

// Display
char line1[16] = "";
char line2[16] = "";
char buf[80] = "";

// Buttons
int btn_resetState = 0;   

// Setup special characters
// arrows
byte UP_ARROW[8] = {
0b00000,
0b00100,
0b01010,
0b10001,
0b00000,
0b00000,
0b00000,
0b00000
};

byte DOWN_ARROW[8] = {
0b00000,
0b00000,
0b00000,
0b00000,
0b10001,
0b01010,
0b00100,
0b00000
};

// smooth bars
byte BAR0[8] = {
0b11011,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000,
0b11011
};

byte BAR1[8] = {
0b11111,
0b10000,
0b10000,
0b10000,
0b10000,
0b10000,
0b10000,
0b11111
};

byte BAR2[8] = {
0b11111,
0b11000,
0b11000,
0b11000,
0b11000,
0b11000,
0b11000,
0b11111
};

byte BAR3[8] = {
0b11111,
0b11100,
0b11100,
0b11100,
0b11100,
0b11100,
0b11100,
0b11111
};

byte BAR4[8] = {
0b11111,
0b11110,
0b11110,
0b11110,
0b11110,
0b11110,
0b11110,
0b11111
};

byte BAR5[8] = {
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111,
0b11111
};

void set_sens(int level) {

  zero_thrshld = sens_zero[level];
  touch_thrshld = sens_touch[level];
  limit_thrshld = sens_limit[level];

  lcd.setCursor(0,0);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,0);
  sprintf(line1, "Level: %s", sens_strings[level]); 
  lcd.print(line1);

  digitalWrite(GREEN_PIN, HIGH);
  lcd.setCursor(0,1);
  sprintf(line2, "Zero at: %d", zero_thrshld); 
  lcd.print(line2);
  delay(indicator_time);
  digitalWrite(GREEN_PIN, LOW);
  
  digitalWrite(BLUE_PIN, HIGH);
  lcd.setCursor(0,1);
  sprintf(line2, "Touch at: %d", touch_thrshld); 
  lcd.print(line2);
  delay(indicator_time);
  digitalWrite(BLUE_PIN, LOW);

  digitalWrite(RED_PIN, HIGH);
  lcd.setCursor(0,1);
  sprintf(line2, "Limit at: %d", limit_thrshld); 
  lcd.print(line2);
  delay(indicator_time);
  digitalWrite(RED_PIN, LOW);  

  progress_factor = ( max_factor / limit_thrshld );

  sprintf(line2, "                ");
  lcd.print(line2);
}

void setup() {
  /* bootstrap the system */
  Serial.begin(57600);
  /* Wire.begin(4);  
   *   Not using Wire communication as we decided to keep complexity
   *   down
   */

  // Set pin modes, old fashion in and out
  /* In */
  pinMode(btn_resetPin, INPUT);
  pinMode(btn_sensPin, INPUT);
  pinMode(probe_pin, INPUT);

  /* Out */
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(relay_touch_pin, OUTPUT);
  pinMode(relay_limit_pin, OUTPUT);

  lcd.begin(16,2);
  lcd.clear();

  lcd.createChar(0, BAR0);
  lcd.createChar(1, BAR1);
  lcd.createChar(2, BAR2);
  lcd.createChar(3, BAR3);
  lcd.createChar(4, BAR4);
  lcd.createChar(5, BAR5);
  lcd.createChar(6, UP_ARROW);
  lcd.createChar(7, DOWN_ARROW);

  lcd.setCursor(0,0);
  lcd.print("Load Cell Probe ");
  lcd.setCursor(0,1);
  lcd.print("Initializing");

  scale1.begin(SCALE1_LOADCELL_DOUT_PIN, SCALE1_LOADCELL_SCK_PIN);
  scale1.tare();
  scale1.set_scale(unit_scale);
  scale2.begin(SCALE2_LOADCELL_DOUT_PIN, SCALE2_LOADCELL_SCK_PIN);
  scale2.tare();
  scale2.set_scale(unit_scale);
  delay(indicator_time);

  lcd.setCursor(0,1);
  lcd.print("Sensors zeroed  ");
  delay(indicator_time);
  lcd.setCursor(0,1);
  lcd.print("                ");

  // Set sensitivity values between 0-2
  set_sens(senslevel);

  digitalWrite(relay_touch_pin, LOW);
  digitalWrite(relay_limit_pin, LOW);

  Serial.println("Limit and factor:");
  Serial.print(limit_thrshld);
  progress_factor = ( max_factor / limit_thrshld );
  Serial.print(", ");
  Serial.print(progress_factor);
  Serial.println("");
  
  Serial.println("Load Cell probe started");
  lcd.setCursor(0,1);
  lcd.print("Initialized     ");
  delay(indicator_time);
  lcd.clear();
  lcd.setCursor(0,0);
  for (int i = 0; i <= 15; i++) {
    lcd.setCursor(i,0);
    delay(100);
    lcd.write(byte(0));
  }
  // Good to go, green lights on
  digitalWrite(GREEN_PIN, HIGH);
}

void loop() {

  // check pushbuttons
   
  if (digitalRead(btn_resetPin) == HIGH) {
    lcd.setCursor(0,0);
    lcd.print("Sensors zeroed  ");
    scale1.tare();
    scale2.tare();
    delay(500);
  }
  
  if (digitalRead(btn_sensPin) == HIGH) {
    senslevel++;
    if (senslevel > 2)
      senslevel = 0;
    set_sens(senslevel);
  }

  // Check if we are probing
  probe_reading = digitalRead(probe_pin);
  if (probe_reading == HIGH) {
    probing = true;
  } else {
    probing = false;
  }

  /*  TODO: Add button for sensitivity setting
   *  where you can adjust thresholds without 
   *  having to build a new image
   */

  if (scale1.is_ready() || scale2.is_ready()) {
    reading1 = round(scale1.get_units());
    reading2 = round(scale2.get_units());

    /* debug readouts 
    Serial.print("scale 1: ");
    Serial.print(reading1);
    Serial.print(", scale2: ");
    Serial.print(reading2);
    Serial.println("");
      end debug */

    /* 
     *  sanity check on reading(s), occasionally crazy values are read
     *  and we need to ensure they don't accidently flip the limit relay
    */
    if ( !scale_over_zero && (abs(reading1) > limit_thrshld * 2 )) {
      Serial.print("Crazy value from scale 1: ");
      Serial.print(reading1);
      Serial.println();
      reading1 = ( round(scale1.get_units()) + prev_reading ) / 2;
      reading = round(((abs(reading1) + abs(reading2)) / 2));
    } else if ( !scale_over_zero && (abs(reading2) > limit_thrshld * 2 )) {
      Serial.print("Crazy value from scale 2: ");
      Serial.print(reading2);
      Serial.println();
      reading2 = ( round(scale2.get_units()) + prev_reading ) / 2;
      reading = round(((abs(reading1) + abs(reading2)) / 2));
    } else {
      reading = round(((abs(reading1) + abs(reading2)) / 2));
      prev_reading = reading;
    }
  } else {
    Serial.println("Scales not ready.");
  }

  /* debug 
  Serial.print("avg reading: ");
  Serial.print(reading);
  Serial.println("");

  */

  scale_over_zero = scale_over_touch = scale_over_limit = false;
  
  // Check value to thresholds
  if (abs(reading) >= limit_thrshld ) {
    scale_over_limit = true;
/*    Wire.beginTransmission(8);
    Wire.write(3);              // sends one byte  
    Wire.endTransmission();    // stop transmitting */
  } else if (abs(reading) >= touch_thrshld ) {
    scale_over_touch = true;
/*    Wire.beginTransmission(8);
    Wire.write(2);              // sends one byte  
    Wire.endTransmission();    // stop transmitting */
  } else if (abs(reading) >= zero_thrshld) {
    scale_over_zero = true;
/*    Wire.beginTransmission(8);
    Wire.write(1);              // sends one byte  
    Wire.endTransmission();    // stop transmitting */
  } else {
/*    Wire.beginTransmission(8);
    Wire.write(0);              // sends one byte  
    Wire.endTransmission();    // stop transmitting */
  }

  // Check tilt, may have to adjust logic if not working in real world 
  if ((reading1 < 0) && (reading2 < 0)) {
    // up
    tiltdir = 1;
  } else if ((reading1 > 0) && (reading2 > 0)) {
    // down
    tiltdir = 2;
  } else if ((reading1 > 0) && (reading2 < 0)) {
    // left
    tiltdir = 3;
  } else if ((reading1 < 0) && (reading2 > 0)) {
    // right
    tiltdir = 4;
  }

  if (probing) {
    sprintf(line2, "%-7s", "Probing"); 
  } else {
    sprintf(line2, "%-7s", "     ");
  }

  // Now check if zero, touch or limit is reached
  if ( scale_over_limit ) {
    Serial.println("Limit exceeded");
    Serial.println("scale 1:");
    Serial.print(reading1);
    Serial.print(", scale2: ");
    Serial.print(reading2);
    Serial.println(", avg: ");
    Serial.print(reading);
    Serial.println("");
    sprintf(line2, "%-7s", "Limit");
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(relay_limit_pin, HIGH);
    if (probing) {
      digitalWrite(relay_touch_pin, HIGH);
    }
    limit = true;
  }  else if ( scale_over_touch ) {
    Serial.println("Touch exceeded");
    Serial.println("scale 1:");
    Serial.print(reading1);
    Serial.print(", scale2: ");
    Serial.print(reading2);
    Serial.println(", avg: ");
    Serial.print(reading);
    Serial.println("");
    digitalWrite(relay_limit_pin, LOW);
    // Recognise a touch only if direction is touchdir 
    if (probing && (tiltdir == touchdir) ) {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(relay_touch_pin, HIGH);
      if (probing) {
        sprintf(line2, "%-7s", "Touch"); 
      } 
    } else {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
    }
    touch = true;
  } else if ( scale_over_zero ) {
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(relay_limit_pin, LOW);
    digitalWrite(relay_touch_pin, LOW);
    tilt = true;
  } else {
    tilt = touch = limit = false;
    tiltdir = 0;
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(BLUE_PIN, LOW);
    digitalWrite(RED_PIN, LOW);
    digitalWrite(relay_limit_pin, LOW);
    digitalWrite(relay_touch_pin, LOW);
  }

  // Progress bar, hacker stylish
  int progress = abs(reading) * progress_factor;
  int whole = floor( progress / 5);
  int remainder = progress % 5;
  lcd.setCursor(0,0);
  int i;
  for (i = 0; i < whole; i++) {
    lcd.setCursor(i,0);
    lcd.write(byte(5));
  }
  lcd.setCursor(i,0);
  lcd.write(byte(remainder));

  i++;
  for (i = i; i <= 15; i++) {
    lcd.setCursor(i,0);
    lcd.write(byte(0));
  }

  // Tilt and messages
  lcd.setCursor(0,1);
  lcd.print("        ");
  lcd.setCursor(0,1);
  if ( scale_over_zero || scale_over_touch || scale_over_limit) {
    lcd.print(reading);
    lcd.setCursor(5,1);
    switch (tiltdir) {
      case 1:
        lcd.write(byte(6));
        break;
      case 2:
        lcd.write(byte(7));
        break;
      case 3:
        lcd.print("<");
        break;
      case 4:
        lcd.print(">");
        break;
      default:
        lcd.print("*");
        break;
    }
  } else {
    lcd.print(reading);
  }
  lcd.setCursor(8,1);
  lcd.print(line2); 
  delay(250);
}
