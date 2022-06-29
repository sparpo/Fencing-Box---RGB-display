//===========================================================================//
//                                                                           //
//  Desc:    Arduino Code to implement a fencing scoring apparatus           //
//  Dev:     Wnew                                                            //
//  Date:    Nov  2012                                                       //
//  Updated: Sept 2015                                                       //
//  Notes:   1. Basis of algorithm from digitalwestie on github. Thanks Mate //
//           2. Used uint8_t instead of int where possible to optimise       //
//           3. Set ADC prescaler to 16 faster ADC reads                     //
//                                                                           //
//  To do:   1. Could use shift reg on lights and mode LEDs to save pins     //
//           2. Implement short circuit LEDs (already provision for it)      //
//           3. Set up debug levels correctly                                //
//                                                                           //
//===========================================================================//

//============
// #defines
//============
//TODO: set up debug levels correctly
//#define DEBUG 0
//#define TEST_LIGHTS       // turns on lights for a second on start up
//#define TEST_ADC_SPEED    // used to test sample rate of ADCs
//#define REPORT_TIMING     // prints timings over serial interface
#define BUZZERTIME  1000  // length of time the buzzer is kept on after a hit (ms)
#define LIGHTTIME   3000  // length of time the lights are kept on after a hit (ms)
#define BAUDRATE   9600  // baudrate of the serial debug interface
#define R 255 //max red value. Red LEDs resistor too large. Blue and green need to be same brightness
#define G 128 //max green value
#define B 138 //max blue value

//============
// Pin Setup
//============

const uint8_t rB = 3;
const uint8_t gB = 5;
const uint8_t bB = 6;
const uint8_t rA = 9;
const uint8_t gA = 10;
const uint8_t bA = 11;

const uint8_t lamePinA   = A0;    // Lame   A pin - Analog (Epee return path)
const uint8_t weaponPinA = A1;    // Weapon A pin - Analog
const uint8_t groundPinA = A2;    // Ground A pin - Analog

const uint8_t lamePinB   = A3;    // Lame   B pin - Analog (Epee return path)
const uint8_t weaponPinB = A4;    // Weapon B pin - Analog
const uint8_t groundPinB = A5;    // Ground B pin - Analog

const uint8_t buzzerPin  =  7;        // buzzer pin
const uint8_t soundPin   =  12;       // Pin turns sound on or off

const uint8_t modePin    =  2;        // Mode change button interrupt pin 0 (digital pin 2)
const uint8_t modeLeds[] = {4, 5, 6}; // LED pins to indicate weapon mode selected {f e s}

const uint8_t bleedPin = 13;     // Current bleeder resistor pin. Only needed if using phone power bank as power source.

//=========================
// values of analog reads
//=========================
int weaponA = 0;
int weaponB = 0;
int lameA   = 0;
int lameB   = 0;
int groundA = 0;
int groundB = 0;

int soundEn = 1; //sound enable

//=======================
// depress and timeouts
//=======================
long depressAtime = 0;
long depressBtime = 0;
bool lockedOut    = false;

//==========================
// Lockout & Depress Times
//==========================
// the lockout time between hits for foil is 300ms +/-25ms
// the minimum amount of time the tip needs to be depressed for foil 14ms +/-1ms
// the lockout time between hits for epee is 45ms +/-5ms (40ms -> 50ms)
// the minimum amount of time the tip needs to be depressed for epee 2ms
// the lockout time between hits for sabre is 170ms +/-10ms
// the minimum amount of time the tip needs to be depressed (in contact) for sabre 0.1ms -> 1ms
// These values are stored as micro seconds for more accuracy
//                         foil   epee   sabre
const long lockout [] = {300000,  45000, 170000};  // the lockout time between hits
const long depress [] = { 14000,   2000,   1000};  // the minimum amount of time the tip needs to be depressed



//=================
// mode constants
//=================
const uint8_t FOIL_MODE  = 0;
const uint8_t EPEE_MODE  = 1;
const uint8_t SABRE_MODE = 2;

long bleedTime = 0;

uint8_t currentMode = 1;

bool modeJustChangedFlag = false;

//=========
// states
//=========
boolean depressedA  = false;
boolean depressedB  = false;
boolean hitOnTargA  = false;
boolean hitOffTargA = false;
boolean hitOnTargB  = false;
boolean hitOffTargB = false;

#ifdef TEST_ADC_SPEED
long now;
long loopCount = 0;
bool done = false;
#endif


//================
// Configuration
//================
void setup() {
  // set the internal pullup resistor on modePin
  pinMode(modePin, INPUT_PULLUP);
  pinMode(soundPin, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  // add the interrupt to the mode pin (interrupt is pin 0)
  attachInterrupt(digitalPinToInterrupt(modePin), changeMode, FALLING);
  pinMode(modeLeds[0], OUTPUT);
  pinMode(modeLeds[1], OUTPUT);
  pinMode(modeLeds[2], OUTPUT);

  // set the light pins to outputs

  pinMode(bleedPin, OUTPUT);
  pinMode(rA, OUTPUT);
  pinMode(gB, OUTPUT);
  pinMode(bA,  OUTPUT);
  pinMode(rB,  OUTPUT);
  pinMode(gA,  OUTPUT);
  pinMode(bB,  OUTPUT);
  pinMode(buzzerPin,  OUTPUT);

  digitalWrite(modeLeds[currentMode], HIGH);
  digitalWrite(bleedPin, LOW);
#ifdef TEST_LIGHTS
  testLights();
#endif
  
  // this optimises the ADC to make the sampling rate quicker
  adcOpt();

  Serial.begin(BAUDRATE);
  Serial.println("3 Weapon Scoring Box");
  Serial.println("====================");
  Serial.print  ("Mode : ");
  Serial.println(currentMode);

  startUpAnimation();

  resetValues();
}


//=============
// ADC config
//=============
void adcOpt() {

  // the ADC only needs a couple of bits, the atmega is an 8 bit micro
  // so sampling only 8 bits makes the values easy/quicker to process
  // unfortunately this method only works on the Due.
  //analogReadResolution(8);

  // Data Input Disable Register
  // disconnects the digital inputs from which ever ADC channels you are using
  // an analog input will be float and cause the digital input to constantly
  // toggle high and low, this creates noise near the ADC, and uses extra
  // power Secondly, the digital input and associated DIDR switch have a
  // capacitance associated with them which will slow down your input signal
  // if you’re sampling a highly resistive load
  DIDR0 = 0x7F;

  // set the prescaler for the ADCs to 16 this allowes the fastest sampling
  bitClear(ADCSRA, ADPS0);
  bitClear(ADCSRA, ADPS1);
  bitSet  (ADCSRA, ADPS2);
}


//============
// Main Loop
//============
void loop() {
  // use a while as a main loop as the loop() has too much overhead for fast analogReads
  // we get a 3-4% speed up on the loop this way
  while (1) {

    checkIfModeChanged();
    
    // read analog pins
    weaponA = analogRead(weaponPinA);
    weaponB = analogRead(weaponPinB);
    lameA   = analogRead(lamePinA);
    lameB   = analogRead(lamePinB);
    groundA   = analogRead(groundPinA);
    groundB   = analogRead(groundPinB);
    soundEn = digitalRead(soundPin);
#ifdef DEBUG
    String outputs = String("Data: ") + weaponA  + "," + lameA + "," + groundA + "," + weaponB + "," + lameB + "," + groundB;
    Serial.println(outputs);
    
#endif
    
    signalHits();
    if      (currentMode == FOIL_MODE)
      foil();
    else if (currentMode == EPEE_MODE)
      epee();
    else if (currentMode == SABRE_MODE)
      sabre();

#ifdef TEST_ADC_SPEED
    if (loopCount == 0) {
      now = micros();
    }
    loopCount++;
    if ((micros() - now >= 1000000) && done == false) {
      Serial.print(loopCount);
      Serial.println(" readings in 1 sec");
      done = true;
    }
#endif

    bleedResistor();
  }
}


//=====================
// Bleed Pin
//=====================
void bleedResistor() {  //Needed for power banks. Stops them from shutting down.
  long now = millis();
  if (now > (bleedTime + 5000) && now < (bleedTime + 6000)) { // For 1 second out of every 6, bleedPin goes high. This stops power bank from turning off.
    digitalWrite(bleedPin, HIGH);
  } else if (now > (bleedTime + 7000)) {
    digitalWrite(bleedPin, LOW);
    bleedTime = millis();
  }
}

//=====================
// Mode pin interrupt
//=====================
void changeMode() {
  // set a flag to keep the time in the ISR to a min
  modeJustChangedFlag = true;
}


//============================
// Sets the correct mode led
//============================
void setModeLeds() {
  if (currentMode == FOIL_MODE) {
    //digitalWrite(onTargetA, HIGH);
    analogWrite(bA, B);
    analogWrite(bB, B);

  } else {
    if (currentMode == EPEE_MODE) {
      //digitalWrite(onTargetB, HIGH);
      analogWrite(rA, R);
      analogWrite(rB, R);
    } else {
      if (currentMode == SABRE_MODE) {
        //digitalWrite(onTargetA, HIGH);
        //digitalWrite(onTargetB, HIGH);
        analogWrite(gA, G);
        analogWrite(gB, G);

      }
    }
  }
  delay(250);
  resetValues();
}


//========================
// Run when mode changed
//========================
void checkIfModeChanged() {
  if (modeJustChangedFlag) {
    //if (!(digitalRead(modePin))) {
      //Serial.print("yes");
      if (currentMode == 2)
        currentMode = 0;
      else
        currentMode++;
    //}
    setModeLeds();
#ifdef DEBUG
    Serial.print("Mode changed to: ");
    Serial.println(currentMode);
#endif
    modeJustChangedFlag = false;
  }
}


//===================
// Main foil method
//===================
void foil() {

  long now = micros();
  if (((hitOnTargA || hitOffTargA) && (depressAtime + lockout[0] < now)) ||
      ((hitOnTargB || hitOffTargB) && (depressBtime + lockout[0] < now))) {
    lockedOut = true;
  }
  // weapon A
  if (hitOnTargA == false && hitOffTargA == false) { // ignore if A has already hit
    // off target

    if (550 < weaponA && lameB < 50) {
      //Serial.println("yes");
      if (!depressedA) {
        depressAtime = micros();
        depressedA   = true;
      } else {
        if (depressAtime + depress[0] <= micros()) {
          hitOffTargA = true;
        }
      }
    } else {
      // on target
      if (450 < weaponA && weaponA < 550 && 450 < lameB && lameB < 550) {
        if (!depressedA) {
          depressAtime = micros();
          depressedA   = true;
        } else {
          if (depressAtime + depress[0] <= micros()) {
            hitOnTargA = true;
          }
        }
      } else {
        // reset these values if the depress time is short.
        depressAtime = 0;
        depressedA   = 0;
      }
    }
  }

  // weapon B
  if (hitOnTargB == false && hitOffTargB == false) { // ignore if B has already hit
    // off target
    if (550 < weaponB && lameA < 50) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB   = true;
      } else {
        if (depressBtime + depress[0] <= micros()) {
          hitOffTargB = true;
        }
      }
    } else {
      // on target
      if (450 < weaponB && weaponB < 550 && 450 < lameA && lameA < 550) {
        if (!depressedB) {
          depressBtime = micros();
          depressedB   = true;
        } else {
          if (depressBtime + depress[0] <= micros()) {
            hitOnTargB = true;
          }
        }
      } else {
        // reset these values if the depress time is short.
        depressBtime = 0;
        depressedB   = 0;
      }
    }
  }
}


//===================
// Main epee method
//===================
void epee() {
  long now = micros();
  if ((hitOnTargA && (depressAtime + lockout[1] < now)) || (hitOnTargB && (depressBtime + lockout[1] < now))) {
    lockedOut = true;
  }
  
  // weapon A
  //  no hit for A yet    && weapon depress    && opponent lame touched
  if (hitOnTargA == false) {
    if (450 < weaponA && weaponA < 550 && 450 < lameA && lameA < 550) {
      
      if (!depressedA) {
        depressAtime = micros();
        depressedA   = true;
      } else {
        if (depressAtime + depress[1] <= micros()) {
          hitOnTargA = true;
        }
      }
      
      #ifdef REPORT_TIMING  //check if timing is correct
        Serial.println(micros() - depressAtime);
      #endif
      
    } else {
      // reset these values if the depress time is short.
      if (depressedA == true) {
        depressAtime = 0;
        depressedA   = 0;
      }
    }
  }

  // weapon B
  //  no hit for B yet    && weapon depress    && opponent lame touched
  if (hitOnTargB == false) {
    if (450 < weaponB && weaponB < 550 && 450 < lameB && lameB < 550) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB   = true;
      } else {
        if (depressBtime + depress[1] <= micros()) {
          hitOnTargB = true;
        }
      }
      
      #ifdef REPORT_TIMING //check if timing is correct
        Serial.println(micros() - depressBtime);
      #endif
      
    } else {
      // reset these values if the depress time is short.
      if (depressedB == true) {
        depressBtime = 0;
        depressedB   = 0;
      }
    }
  }
}


//===================
// Main sabre method
//===================
void sabre() {

  long now = micros();
  if (((hitOnTargA || hitOffTargA) && (depressAtime + lockout[2] < now)) ||
      ((hitOnTargB || hitOffTargB) && (depressBtime + lockout[2] < now))) {
    lockedOut = true;
  }

  // weapon A
  if (hitOnTargA == false && hitOffTargA == false) { // ignore if A has already hit
    // on target
    if (weaponA < 450 &&  50 < lameB) {
      if (!depressedA) {
        depressAtime = micros();
        depressedA   = true;
      } else {
        if (depressAtime + depress[2] <= micros()) {
          hitOnTargA = true;
        }
      }
            
      #ifdef REPORT_TIMING  //check if timing is correct
        Serial.println(micros() - depressAtime);
      #endif
      
    } else {
      // reset these values if the depress time is short.
      depressAtime = 0;
      depressedA   = 0;
    }
  }

  // weapon B
  if (hitOnTargB == false && hitOffTargB == false) { // ignore if B has already hit
    // on target
    if ( weaponB < 450 && 50 < lameA) {
      if (!depressedB) {
        depressBtime = micros();
        depressedB   = true;
              
      #ifdef REPORT_TIMING  //check if timing is correct
        Serial.println(micros() - depressBtime);
      #endif
      
      } else {
        if (depressBtime + depress[2] <= micros()) {
          hitOnTargB = true;
        }
      }
    } else {
      // reset these values if the depress time is short.
      depressBtime = 0;
      depressedB   = 0;
    }
  }
}


//==============
// Signal Hits
//==============
void signalHits() {
  // non time critical, this is run after a hit has been detected
  if (lockedOut) {

    if(soundEn) { //if sound enabled
      digitalWrite(buzzerPin,  HIGH);
    }
    
    if (hitOnTargA) { //green
      analogWrite(rA, R);
      analogWrite(gA, 0);
      analogWrite(bA, 0);
    }
    if (hitOffTargA) { //white
      analogWrite(rA, R);
      analogWrite(gA, G);
      analogWrite(bA, B);
    }
    if (hitOffTargB) { //white
      analogWrite(rB, R);
      analogWrite(gB, G);
      analogWrite(bB, B);
    }
    if (hitOnTargB) { //red
      analogWrite(rB, 0);
      analogWrite(gB, G);
      analogWrite(bB, 0);
    }

    
#ifdef DEBUG
    String serData = String("hitOnTargA  : ") + hitOnTargA  + "\n"
                     + "hitOffTargA : "  + hitOffTargA + "\n"
                     + "hitOffTargB : "  + hitOffTargB + "\n"
                     + "hitOnTargB  : "  + hitOnTargB  + "\n"
                     + "Locked Out  : "  + lockedOut   + "\n";
    Serial.println(serData);
#endif
    resetValues();
  }
}


//======================
// Reset all variables
//======================
void resetValues() {
  delay(BUZZERTIME);             // wait before turning off the buzzer
  digitalWrite(buzzerPin,  LOW);
  delay(LIGHTTIME - BUZZERTIME); // wait before turning off the lights
  /*digitalWrite(onTargetA,  LOW);
    digitalWrite(offTargetA, LOW);
    digitalWrite(offTargetB, LOW);
    digitalWrite(onTargetB,  LOW);
    digitalWrite(shortLEDA,  LOW);
    digitalWrite(shortLEDB,  LOW);*/
  analogWrite(rA, 0);
  analogWrite(gA, 0);
  analogWrite(bA, 0);
  analogWrite(rB, 0);
  analogWrite(gB, 0);
  analogWrite(bB, 0);
  lockedOut    = false;
  depressAtime = 0;
  depressedA   = false;
  depressBtime = 0;
  depressedB   = false;

  hitOnTargA  = false;
  hitOffTargA = false;
  hitOnTargB  = false;
  hitOffTargB = false;

  delay(100);
}


//==============
// Test lights
//==============
void testLights() {
  /*digitalWrite(offTargetA, HIGH);
    digitalWrite(onTargetA,  HIGH);
    digitalWrite(offTargetB, HIGH);
    digitalWrite(onTargetB,  HIGH);
    digitalWrite(shortLEDA,  HIGH);
    digitalWrite(shortLEDB,  HIGH);*/
  analogWrite(rA, R);
  analogWrite(gA, G);
  analogWrite(bA, B);
  analogWrite(rB, R);
  analogWrite(gB, G);
  analogWrite(bB, B);
  delay(1000);
  resetValues();
}

//====================
// Start up animation
//====================
void startUpAnimation() {
  int d = 2;
  for (int i = 0; i < 255; i++) {
    analogWrite(rA, i);
    analogWrite(gA, i / 2);
    analogWrite(bA, 128 - i / 2);

    analogWrite(rB, i);
    analogWrite(gB, 0);
    analogWrite(bB, 128);
    delay(d);
  }

  for (int i = 0; i < 255; i++) {
    analogWrite(rA, 255 - i);
    analogWrite(gA, 128);
    analogWrite(bA, i / 2);

    analogWrite(rB, 255);
    analogWrite(gB, i / 2);
    analogWrite(bB, 128 - i / 2);
    delay(d);
  }

  for (int i = 0; i < 255; i++) {
    analogWrite(rA, i);
    analogWrite(gA, 128 - i / 2);
    analogWrite(bA, 128 - i / 2);

    analogWrite(rB, 255 - i);
    analogWrite(gB, 128);
    analogWrite(bB, 0);
    delay(d);
  }
}
