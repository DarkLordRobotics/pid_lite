int speedLeft;       // PWM value for speed, range 0 to 255
int speedRight;      // PWM value for speed, range 0 to 255

// Bile (devastator chassis):
//int speedLeftDefault  = 191;  // debug only
//int speedRightDefault = 187;  // debug only 185?
int speedLeftDefault  = 255;  // PWM value for speed, range 0 to 255
int speedRightDefault = 248;  // PWM value for speed, range 0 to 255


#define encoderPinA 3
#define encoderPinB 2
volatile int encoderPosA = 0;
volatile int encoderPosB = 0;
double encoderCountA = 0.0;
double encoderCountB = 0.0;
double encoderAccumCountA = 0.0;
double encoderAccumCountB = 0.0;

// IC interval count error:
int encoderCountAmB = 0;  // A minus B for proportional factor (P)
int encoderCountAmBlast = 0;  // previous A minus B for differential factor (D)
// TC total count error:
double encoderAccumCountAmB = 0;  // cumulative A minus B for integral factor (I)

#define encoderReadInterval 100  // time in milliseconds between reads (counting)

// needed?
unsigned long timeNow;
unsigned long timeLast;
unsigned long timeElapsed;

/*
PID control loop:
read encoders @ intercal (100 ms?), find count difference
reduce A or B throttle (T) until counts are equal (delta throttle DT)
total count error (TC) = since start of move (integral)
interval count error (IC) = since last read (proportional)
delta throttle calculation:
// DT = Kp*IC + Ki*TC + Kd*(ICn - ICn-1);
// speedRight = speedRight + DT * speedLeft/255;
*/
// Kx factors:
// speed 254: **100.20.10: new, or *80.20.10: new
// speed 192: *80.20.10: new
float DT;
//float Kp = 1.0;  // 255 speed
float Kp = 0.8;  // 191 speed
float Ki = 0.22;  // was 0.2
float Kd = 0.1;  // was 0.1


////////////////////////////////////////////////////////////////////////////////
void setup() {
////////////////////////////////////////////////////////////////////////////////

  if (debug >= 1) {
    Serial.begin(115200);
    Serial.println();
    Serial.println("floorbotBile_0");
    Serial.println(version);
    Serial.println();
  }

  // interrupts for encoders
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  // (RISING, FALLING and CHANGE all work with this library)

  // Arduino Motor Shield (R3) pin mode
  pinMode(DIR_A_PIN,OUTPUT);
  pinMode(DIR_B_PIN,OUTPUT);
  pinMode(PWM_A_PIN,OUTPUT);
  pinMode(PWM_B_PIN,OUTPUT);

  moveStop();

  // initalize speed settings
  speedLeft  = speedLeftDefault;
  speedRight = speedRightDefault;


////////////////////////////////////////////////////////////////////////////////
void loop() {
////////////////////////////////////////////////////////////////////////////////

  if (debug == 2) {
    //Serial.println(" ");
    Serial.println(" ");
    Serial.print("Time loop start: ");
    Serial.println(millis());
    Serial.println(" ");
    //Serial.println(" ");
  }

  timeLast = millis();

  navUpdatePID();

}


// -------------------------------------
void navUpdatePID() {
  // read encoders & calculate PID loop changes

  // RH add something to compensate for left side motor speed, a scaling factor:
  // DT = (Kp*IC + Ki*TC + Kd*(ICn - ICn-1)) * speedLeft/speedLeftDefault;
  // DT = Kp*IC + Ki*TC + Kd*(ICn - ICn-1)
  // IC interval count error:
  //int encoderCountAmB = 0;  // A minus B for proportional factor (P)
  //int encoderCountAmBlast = 0;  // previous A minus B for differential factor (D)
  // TC total count error:
  //double encoderAccumCountAmB = 0;  // cumulative A minus B for integral factor (I)
  
  timeNow = millis();
  timeElapsed = timeNow - timeLast;
  //float DTnew;

  // update once every encoderReadInterval:
  //if (timeElapsed >= 1) {
  if (timeElapsed >= encoderReadInterval) {

    if (debug >= 1) {
      Serial.println("navUpdatePID...");
      Serial.print("timeNow: ");
      Serial.print(timeNow);
      Serial.print(" timeLast: ");
      Serial.print(timeLast);
      Serial.print(" timeElapsed: ");
      Serial.println(timeElapsed);
    }
    encoderCountAmBlast = encoderCountAmB;
    readEncoders();  // updates encoderCountAmB, encoderAccumCountAmB
    timeLast = timeNow;

    // DT = Kp*IC + Ki*TC + Kd*(ICn - ICn-1)   // ?
    //speedRight = speedRight + Kp*encoderCountAmB + Ki*encoderAccumCountAmB + Kd*(encoderCountAmB - encoderCountAmBlast);
    //DT = (Kp*encoderCountAmB + Kd*encoderAccumCountAmB + Ki*(encoderCountAmB - encoderCountAmBlast)) * (speedLeft/255) * float(100.0/timeElapsed);

    DT = (Kp*encoderCountAmB + Kd*encoderAccumCountAmB + Ki*(encoderCountAmB - encoderCountAmBlast));
    speedRight = speedRight + DT;

    //DTnew = DT * speedLeft/timeElapsed * 100/255;
    //speedRight = speedRight + DTnew;
    //speedRight = speedRight + DT * speedLeft/timeElapsed * 100/255;
    //speedRight = speedRight + DT * (speedLeft/255) * float(100.0/timeElapsed);

    if (speedRight >= 255) {
      speedRight = 255;
    }
    else if (speedRight <= 0 || speedLeft == 0) {
      speedRight = 0;
    }
    analogWrite(PWM_B_PIN, speedRight);
  }
  if (debug >= 1) {
    Serial.print("encoderCountAmBlast: ");
    Serial.println(encoderCountAmBlast);
    Serial.print("speedLeft (A): ");
    Serial.print(speedLeft);
    Serial.print(" speedRight (B): ");
    Serial.println(speedRight);
    Serial.print("DT: ");
    Serial.print(DT);
    //Serial.print(" DTnew: ");
    //Serial.println(DTnew);
  }
}


// -------------------------------------
// encoder INT service routines
void doEncoderA() {
  encoderPosA++;
}

void doEncoderB() {
  encoderPosB++;
}

// -------------------------------------
// read encoder counts & update position
void readEncoders() {

  encoderCountA = encoderPosA;
  encoderCountB = encoderPosB;
  if (debug >= 1) {
    Serial.println();
    Serial.print("encoderCountB: ");
    Serial.println(encoderCountB);
  }
  //encoderCountB = encoderCountB - 3;  // calibration +0.4% based on ~500 ticks per read
  encoderCountB = encoderCountB * 994/1000;  // calibration +0.6%, 0.4%
  if (debug >= 1) {
    Serial.println();
    Serial.print("encoderCountB cal: ");
    Serial.println(encoderCountB);
  }

  encoderPosA = 0;
  encoderPosB = 0;
  encoderCountAmB = encoderCountA - encoderCountB;  // A minus B for course corrections, turns, etc.

  encoderAccumCountA = encoderAccumCountA + encoderCountA;
  encoderAccumCountB = encoderAccumCountB + encoderCountB;
  encoderAccumCountAmB = encoderAccumCountA - encoderAccumCountB;  // A minus B for course corrections, turns, etc.

  if (debug >= 1) {
    Serial.println();
    Serial.print("encoderCountAmB: ");
    Serial.println(encoderCountAmB);
    Serial.print("encoderCountA: ");
    Serial.println(encoderCountA);
    Serial.print("encoderCountB: ");
    Serial.println(encoderCountB);
    Serial.print("encoderAccumCountAmB: ");
    Serial.println(encoderAccumCountAmB);
    Serial.print("encoderAccumCountA: ");
    Serial.println(encoderAccumCountA);
    Serial.print("encoderAccumCountB: ");
    Serial.println(encoderAccumCountB);
  }
}

// -------------------------------------
// read encoder counts & update position
void clearEncoders() {

  encoderCountA = 0;
  encoderCountB = 0;
  encoderCountAmB = 0;
  encoderPosA = 0;
  encoderPosB = 0;
  encoderAccumCountA = 0;
  encoderAccumCountB = 0;
  encoderAccumCountAmB = 0;
}

//-----------------------------------------------
// move code for Arduino Motor Shield R3

void moveStop() {
  digitalWrite(DIR_A_PIN,LOW);
  digitalWrite(DIR_B_PIN,LOW);
  analogWrite(PWM_A_PIN,0);
  analogWrite(PWM_B_PIN,0);
}

//void moveForward(int speedLeft, int speedRight) {
void moveForward() {
  digitalWrite(DIR_A_PIN, HIGH);
  digitalWrite(DIR_B_PIN, HIGH);
  analogWrite(PWM_A_PIN, speedLeft);
  analogWrite(PWM_B_PIN, speedRight);
}


