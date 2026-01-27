/*
 =========================================================
   STM32 Blue Pill - FUZZY LOGIC Control Robot Arm
   Author  : ---
   Board   : STM32F103C8 (Blue Pill)
   Baudrate: 115200
   Create  : 22 Jan 2026

   Format Serial:
   gripper,armSperhead,armBoxBottom,armBoxTop\n

   fuzzy logic
 ========================================================= 
*/

// ===================== PARAMETER FUZZY ===================
#define DEADBAND 3
#define MAX_PWM 255
#define MIN_PWM 20

// Membership function boundaries
#define ERROR_NB -60  // Negative Big
#define ERROR_NM -30  // Negative Medium
#define ERROR_NS -10  // Negative Small
#define ERROR_ZE 0    // Zero
#define ERROR_PS 10   // Positive Small
#define ERROR_PM 30   // Positive Medium
#define ERROR_PB 60   // Positive Big

#define DERROR_NB -15
#define DERROR_NM -8
#define DERROR_NS -3
#define DERROR_ZE 0
#define DERROR_PS 3
#define DERROR_PM 8
#define DERROR_PB 15

// ===================== PIN MOTOR =========================
#define GRIPPER_CLOSE PB6
#define GRIPPER_OPEN PB7

#define ARM_SPH_UP PB9
#define ARM_SPH_DOWN PB8

#define ARM_BOT_UP PB0
#define ARM_BOT_DOWN PB1

#define ARM_TOP_UP PA6
#define ARM_TOP_DOWN PA7

// ===================== 74HC4067 ==========================
#define MUX_SIG PA4
#define MUX_S0 PB12
#define MUX_S1 PB13
#define MUX_S2 PB14
#define MUX_S3 PB15

// ===================== TARGET ============================
int tgtGrip = 90;
int tgtSph = 90;
int tgtBot = 30;
int tgtTop = 90;

// ===================== FUZZY STATE =======================
struct FuzzyState {
  int lastError;
  int lastPWM;
};

FuzzyState fuzzySph = { 0, 0 };
FuzzyState fuzzyBot = { 0, 0 };
FuzzyState fuzzyTop = { 0, 0 };

// =========================================================
// MEMBERSHIP FUNCTIONS - Triangular
// =========================================================
float trimf(float x, float a, float b, float c) {
  if (x <= a || x >= c) return 0.0;
  if (x == b) return 1.0;
  if (x > a && x < b) return (x - a) / (b - a);
  return (c - x) / (c - b);
}

// Trapezoidal membership function
float trapmf(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0.0;
  if (x >= b && x <= c) return 1.0;
  if (x > a && x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// =========================================================
// FUZZIFICATION - Error
// =========================================================
void fuzzifyError(int error, float* mu) {
  // NB, NM, NS, ZE, PS, PM, PB
  mu[0] = trapmf(error, -180, -180, ERROR_NB, ERROR_NM);  // NB
  mu[1] = trimf(error, ERROR_NB, ERROR_NM, ERROR_NS);     // NM
  mu[2] = trimf(error, ERROR_NM, ERROR_NS, ERROR_ZE);     // NS
  mu[3] = trimf(error, ERROR_NS, ERROR_ZE, ERROR_PS);     // ZE
  mu[4] = trimf(error, ERROR_ZE, ERROR_PS, ERROR_PM);     // PS
  mu[5] = trimf(error, ERROR_PS, ERROR_PM, ERROR_PB);     // PM
  mu[6] = trapmf(error, ERROR_PM, ERROR_PB, 180, 180);    // PB
}

// =========================================================
// FUZZIFICATION - Delta Error
// =========================================================
void fuzzifyDError(int dError, float* mu) {
  // NB, NM, NS, ZE, PS, PM, PB
  mu[0] = trapmf(dError, -50, -50, DERROR_NB, DERROR_NM);
  mu[1] = trimf(dError, DERROR_NB, DERROR_NM, DERROR_NS);
  mu[2] = trimf(dError, DERROR_NM, DERROR_NS, DERROR_ZE);
  mu[3] = trimf(dError, DERROR_NS, DERROR_ZE, DERROR_PS);
  mu[4] = trimf(dError, DERROR_ZE, DERROR_PS, DERROR_PM);
  mu[5] = trimf(dError, DERROR_PS, DERROR_PM, DERROR_PB);
  mu[6] = trapmf(dError, DERROR_PM, DERROR_PB, 50, 50);
}

// =========================================================
// FUZZY RULE BASE (49 rules: 7x7)
// =========================================================
// Output: NB=-255, NM=-180, NS=-100, ZE=0, PS=100, PM=180, PB=255
int fuzzyRules(float* muE, float* muDE) {
  // Rule outputs (crisp values)
  int outputs[] = { -255, -180, -100, 0, 100, 180, 255 };

  // Fuzzy rule table [Error][DError] -> Output index
  // Rows: Error (NB, NM, NS, ZE, PS, PM, PB)
  // Cols: DError (NB, NM, NS, ZE, PS, PM, PB)
  int ruleTable[7][7] = {
    { 0, 0, 0, 0, 1, 2, 3 },  // Error = NB
    { 0, 0, 0, 1, 2, 3, 4 },  // Error = NM
    { 0, 0, 1, 2, 3, 4, 5 },  // Error = NS
    { 0, 1, 2, 3, 4, 5, 6 },  // Error = ZE
    { 1, 2, 3, 4, 5, 6, 6 },  // Error = PS
    { 2, 3, 4, 5, 6, 6, 6 },  // Error = PM
    { 3, 4, 5, 6, 6, 6, 6 }   // Error = PB
  };

  // Inference using MIN-MAX-COG method
  float numerator = 0.0;
  float denominator = 0.0;

  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      float strength = min(muE[i], muDE[j]);  // MIN (AND operation)
      if (strength > 0) {
        int outputIdx = ruleTable[i][j];
        numerator += strength * outputs[outputIdx];
        denominator += strength;
      }
    }
  }

  // Defuzzification: Center of Gravity
  if (denominator == 0) return 0;
  return (int)(numerator / denominator);
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);

  pinMode(GRIPPER_CLOSE, OUTPUT);
  pinMode(GRIPPER_OPEN, OUTPUT);

  pinMode(ARM_SPH_UP, OUTPUT);
  pinMode(ARM_SPH_DOWN, OUTPUT);

  pinMode(ARM_BOT_UP, OUTPUT);
  pinMode(ARM_BOT_DOWN, OUTPUT);

  pinMode(ARM_TOP_UP, OUTPUT);
  pinMode(ARM_TOP_DOWN, OUTPUT);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  pinMode(MUX_SIG, INPUT_ANALOG);
  analogWriteResolution(10);
}

// =========================================================
// BACA MUX 74HC4067
// =========================================================
int readMux(uint8_t ch) {
  digitalWrite(MUX_S0, ch & 0x01);
  digitalWrite(MUX_S1, ch & 0x02);
  digitalWrite(MUX_S2, ch & 0x04);
  digitalWrite(MUX_S3, ch & 0x08);
  delayMicroseconds(5);
  return analogRead(MUX_SIG);
}

// =========================================================
// ADC KE DERAJAT
// =========================================================
int adcToDeg(int adc) {
  return map(adc, 0, 1023, 0, 180);
}

// =========================================================
// FUZZY LOGIC CONTROLLER
// =========================================================
void motorFuzzy(int target, int current,
                uint8_t pinUp, uint8_t pinDown,
                FuzzyState& fuzzy) {

  int error = target - current;

  // Deadband check
  if (abs(error) <= DEADBAND) {
    analogWrite(pinUp, 0);
    analogWrite(pinDown, 0);
    fuzzy.lastPWM = 0;
    fuzzy.lastError = error;
    return;
  }

  int dError = error - fuzzy.lastError;

  // Fuzzification
  float muError[7] = { 0 };
  float muDError[7] = { 0 };
  fuzzifyError(error, muError);
  fuzzifyDError(dError, muDError);

  // Fuzzy Inference
  int fuzzyOutput = fuzzyRules(muError, muDError);

  // Convert to PWM (with smoothing)
  int pwm = abs(fuzzyOutput);
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);

  // Smooth transition
  fuzzy.lastPWM = (fuzzy.lastPWM * 0.7) + (pwm * 0.3);

  // Apply to motor
  if (fuzzyOutput > 0) {
    analogWrite(pinUp, fuzzy.lastPWM);
    analogWrite(pinDown, 0);
  } else if (fuzzyOutput < 0) {
    analogWrite(pinDown, fuzzy.lastPWM);
    analogWrite(pinUp, 0);
  } else {
    analogWrite(pinUp, 0);
    analogWrite(pinDown, 0);
  }

  fuzzy.lastError = error;
}

// =========================================================
// GRIPPER MODE SERVO 360
// =========================================================
void gripper360(int input) {
  int diff = input - 90;

  if (abs(diff) < 3) {
    analogWrite(GRIPPER_OPEN, 0);
    analogWrite(GRIPPER_CLOSE, 0);
    return;
  }

  int pwm = constrain(abs(diff) * 3, 60, 250);

  if (diff > 0) {
    analogWrite(GRIPPER_OPEN, pwm);
    analogWrite(GRIPPER_CLOSE, 0);
  } else {
    analogWrite(GRIPPER_CLOSE, pwm);
    analogWrite(GRIPPER_OPEN, 0);
  }
}

// =========================================================
// BACA SERIAL (CSV)
// =========================================================
void readSerial() {
  if (!Serial.available()) return;

  static char buf[40];
  int len = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[len] = '\0';

  sscanf(buf, "%d,%d,%d,%d",
         &tgtGrip,
         &tgtSph,
         &tgtBot,
         &tgtTop);

  tgtGrip = constrain(tgtGrip, 0, 180);
  tgtSph = constrain(tgtSph, 0, 180);
  tgtBot = constrain(tgtBot, 0, 180);
  tgtTop = constrain(tgtTop, 0, 180);
}

// =========================================================
// LOOP UTAMA
// =========================================================
void loop() {
  readSerial();

  int sphDeg = adcToDeg(readMux(0));
  int botDeg = adcToDeg(readMux(1));
  int topDeg = adcToDeg(readMux(2));

  // Serial.printf("sph:%d(%d) bot:%d(%d) top:%d(%d)\n",
  //               sphDeg, fuzzySph.lastPWM,
  //               botDeg, fuzzyBot.lastPWM,
  //               topDeg, fuzzyTop.lastPWM);


  Serial.printf("bot:%d(%d)\n",
                botDeg, fuzzyBot.lastPWM);

  gripper360(tgtGrip);

  motorFuzzy(tgtSph, sphDeg, ARM_SPH_UP, ARM_SPH_DOWN, fuzzySph);
  motorFuzzy(tgtBot, botDeg, ARM_BOT_UP, ARM_BOT_DOWN, fuzzyBot);
  motorFuzzy(tgtTop, topDeg, ARM_TOP_UP, ARM_TOP_DOWN, fuzzyTop);
}