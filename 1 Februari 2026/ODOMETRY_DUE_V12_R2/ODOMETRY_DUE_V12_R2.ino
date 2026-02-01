/*
  Program Kontrol Robot Omni-directional dengan Odometry dan Sensor
  Update kalibrasi ulang dan penyusunan rumus odometry

  26 januari 2026
  update coba komunikasi ke slave aktuator atas...
  dengan komunikasi dalam bentuk data biner byte...
*/

/*************************************************
 * KONFIGURASI LIBRARY DAN PERIFERAL
 *************************************************/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/*************************************************
 * KONSTANTA DAN DEFINISI
 *************************************************/

/* MUX dan MPU */
#define MUX_ADDR 0x70
#define CH7 (1 << 7)

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

/* Serial */
#define slaveSerial Serial1
#define cmpSerial Serial2
#define nanoSerial Serial3

/* COMMAND */
#define AT_RESET "RESET"
#define AT_RPS "RPS"

/* PIN ENCODER */
#define encBelakang_CHA 29
#define encBelakang_CHB 27
#define encKanan_CHA 25
#define encKanan_CHB 23
#define encKiri_CHA 24
#define encKiri_CHB 22


/* PARAMETER ODOMETRY */
#define MATH_PI M_PI
#define WHEEL_RADIUS 2.95
#define WHEEL_BASE 9.40
#define ENC_RESOLUTION 1600.0
#define POS_CONV_FACTOR ((2 * MATH_PI * WHEEL_RADIUS) / ENC_RESOLUTION)
#define GYRO_Z_SCALE 1.0047

/* TOLERANSI GERAK */
float MIN_POS_RES = 3.0;
float MIN_DEG_RES = 3.0;

/* PARAMETER FUZZY - Omega */
#define AVLOW_SPEED 10
#define ALOW_SPEED 20
#define AMEDIUM_SPEED 30
#define AFAST_SPEED 50
#define AVFAST_SPEED 75

/* PARAMETER FUZZY - Linear */
#define LVLOW_SPEED 20
#define LLOW_SPEED 40
#define LMEDIUM_SPEED 50
#define LFAST_SPEED 60
#define LVFAST_SPEED 190

/* PARAMETER PID */
// PID Linear
float KP_LINEAR = 0.8;
float KI_LINEAR = 0.0;
float KD_LINEAR = 0.05;
float MAX_V_OUTPUT = 250.0;

// PID Angular
float KP_ANGULAR = 1.0;
float KI_ANGULAR = 0.0;
float KD_ANGULAR = 0.09;
float MAX_W_OUTPUT = 75.0;

#define MAX_DW 90.0  // derajat

/* PIN SENSOR DAN SOLENOID */
const uint8_t sensorPin[5] = { A5, A6, A7, A8, A9 };

#define mBoxBawah_0 30
#define mBoxBawah_1 32
#define mDepan_0 34
#define mDepan_1 44
#define mBelakang_0 26
#define mBelakang_1 28
#define mBoxAtas_0 46
#define mBoxAtas_1 48


const uint8_t solPin[6] = { mDepan_0, mDepan_1, mBelakang_0, mBelakang_1 };

/* PATTERN SENSOR DAN SOLENOID */
// const uint8_t pattern0[] = {
//   0b00000,
//   0b11101,
//   0b11001,
//   0b10001,
//   0b00001
// };
const uint8_t pattern0[] = {
  0b11111,
  0b00010,
  0b00110,
  0b01110,
  0b11110
};
const uint8_t solPattern0[] = {
  0b0101,
  0b1010,
  0b1001,
  0b1001,
  0b0101
};

// const uint8_t solPattern0[] = {
//   0b010101,
//   0b011010,
//   0b011001,
//   0b011001,
//   0b010101
// };
const uint8_t pattern1[] = {
  0b11000,
  0b10000,
  0b00000,
  0b00000,
  0b00000
};
const uint8_t solPattern1[] = {
  0b0101,
  0b0110,
  0b0110,
  0b1010,
  0b1010
};

/* JARAK ENCODER KE PUSAT ROBOT (cm) */
#define L_L 14.44766114        // kiri -> pusat
#define L_R -14.410804862      // kanan -> pusat
#define B_OFFSET -8.464044339  // belakang -> pusat

/*************************************************
 * STRUKTUR DATA
 *************************************************/

/* POSISI */
struct POS {
  float X, Y, T;
} currentPOS, lastPOS;

/* ENCODER */
struct ENC {
  volatile long int pulseCnt;
  long int lastpulse;
} EncBelakang, EncKanan, EncKiri;

/* KOMPAS */
struct cmps {
  float raw;
  float heading;
  float lastHeading;
  float offset;
  bool dmpReady = false;
  float ypr[3];
  uint8_t fifoBuffer[64];
  uint16_t packetSize;
} cmps;


/* VARIABEL GLOBAL */
float sin_t, cos_t;
int RPM_FR, RPM_RR, RPM_FL, RPM_RL;
int count = 0;
bool Move = false;

/* VARIABEL FSM UNTUK PNEUMATIC*/
bool kondisiLocked = false;
bool kondisi = false;  // false = kondisi 0
uint8_t mode = 0;
uint8_t lastMode = 255;

/* TIMER */
uint32_t tReset = 0;
uint32_t tSol = 0;
uint32_t tSerial = 0;
bool waitReset = false;
bool solActive = false;

/* STATE PID */
float error_prev_linear = 0;
float integral_linear = 0;
unsigned long lastTime_linear = 0;

float error_prev_angular = 0;
float integral_angular = 0;
unsigned long lastTime_angular = 0;

/*data aktuator*/
uint8_t _grip_sph = 90,
        _arm_sph = 15,
        _arm_box_bot = 25,
        _arm_box_top = 135,
        _conveyor = 90,
        _tgt6 = 90;

uint8_t stateGripperSph = 0;
unsigned long lTimeGripper = 0;


bool kontrolManual = true;
bool dataNew = false;

#define led1 A0
#define led2 A1

/*************************************************
 * DEKLARASI FUNGSI
 *************************************************/

/* FUNGSI UTAMA SETUP */
void setup();

/* FUNGSI KOMUNIKASI */
void initSerial();
void initI2C();

/* FUNGSI MUX DAN I2C */
void selectMux(uint8_t ch);
void resetI2C();

/* FUNGSI ENCODER */
void initEncoder();
void encBelakang_INTT_A();
void encBelakang_INTT_B();
void encKanan_INTT_A();
void encKanan_INTT_B();
void encKiri_INTT_A();
void encKiri_INTT_B();

/* FUNGSI KOMPAS */
void initCMPS();
void updateCMPS();
void resetCMPS();

/* FUNGSI ODOMETRI */
void initOdometry();
void fullResetENC();
void updateOdometry();

/* FUNGSI KONTROL GERAK */
void setRPM(int rlRPM, int rrRPM, int frRPM, int flRPM);
void sendData();
bool goXYT(int x, int y, int t);
float angleError(float target, float current);

/* FUNGSI FUZZY */
float FuzzyOmega(float wSpeed);
float FuzzyLinear(float vSpeed);

/* FUNGSI PID */
float PIDLinear(float error);
float PIDAngular(float error);
void resetPID();

/* FUNGSI SENSOR DAN SOLENOID */
void initSensor();
void initSolenoid();
void sensor_solenoid();
void setSolenoid(uint8_t mask);

/* FUNGSI KONTROL UTAMA */
void controlCompute();

/* FUNGSI TEST */
void trialEncMaster();

/*************************************************
 * IMPLEMENTASI FUNGSI
 *************************************************/

void setup() {
  /* Inisialisasi*/
  initSerial();

  /* Inisialisasi I2C */
  initI2C();

  /* Setup Encoder */
  initEncoder();

  /* Setup Sensor dan Solenoid */
  initSensor();
  initSolenoid();

  /* Reset dan Inisialisasi */
  initCMPS();
  initOdometry();

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  SerialUSB.println("Main loop started");
  controlCompute();  // Masuk ke fungsi kontrol utama
}

void loop() {
  // Program berjalan di controlCompute()
}


/*************************************************
 * FUNGSI SETUP SERIAL
 *************************************************/

void initSerial() {
  Serial.begin(115200);
  SerialUSB.begin(115200);
  cmpSerial.begin(115200);
  slaveSerial.begin(115200);
  nanoSerial.begin(115200);
}


/*************************************************
 * FUNGSI SETUP I2C
 *************************************************/

void initI2C() {
  resetI2C();
  Wire.begin();
  Wire.setClock(400000);
}


/*************************************************
 * FUNGSI MUX DAN I2C
 *************************************************/

void selectMux(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch);
  Wire.endTransmission();
}

void resetI2C() {
  pinMode(SDA, OUTPUT);
  pinMode(SCL, OUTPUT);

  digitalWrite(SDA, HIGH);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
}

/*************************************************
 * FUNGSI ENCODER
 *************************************************/

void initEncoder() {
  pinMode(encBelakang_CHA, INPUT_PULLUP);
  pinMode(encBelakang_CHB, INPUT_PULLUP);
  pinMode(encKanan_CHA, INPUT_PULLUP);
  pinMode(encKanan_CHB, INPUT_PULLUP);
  pinMode(encKiri_CHA, INPUT_PULLUP);
  pinMode(encKiri_CHB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encBelakang_CHA), encBelakang_INTT_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encBelakang_CHB), encBelakang_INTT_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKanan_CHA), encKanan_INTT_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKanan_CHB), encKanan_INTT_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKiri_CHA), encKiri_INTT_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encKiri_CHB), encKiri_INTT_B, CHANGE);

  fullResetENC();
}

void encBelakang_INTT_A() {
  if (digitalRead(encBelakang_CHA) == HIGH) {
    digitalRead(encBelakang_CHB) == LOW ? EncBelakang.pulseCnt-- : EncBelakang.pulseCnt++;
  } else {
    digitalRead(encBelakang_CHB) == HIGH ? EncBelakang.pulseCnt-- : EncBelakang.pulseCnt++;
  }
}

void encBelakang_INTT_B() {
  if (digitalRead(encBelakang_CHA) == HIGH) {
    digitalRead(encBelakang_CHB) == HIGH ? EncBelakang.pulseCnt-- : EncBelakang.pulseCnt++;
  } else {
    digitalRead(encBelakang_CHB) == LOW ? EncBelakang.pulseCnt-- : EncBelakang.pulseCnt++;
  }
}

void encKanan_INTT_A() {
  if (digitalRead(encKanan_CHA) == HIGH) {
    digitalRead(encKanan_CHB) == LOW ? EncKanan.pulseCnt-- : EncKanan.pulseCnt++;
  } else {
    digitalRead(encKanan_CHB) == HIGH ? EncKanan.pulseCnt-- : EncKanan.pulseCnt++;
  }
}

void encKanan_INTT_B() {
  if (digitalRead(encKanan_CHA) == HIGH) {
    digitalRead(encKanan_CHB) == HIGH ? EncKanan.pulseCnt-- : EncKanan.pulseCnt++;
  } else {
    digitalRead(encKanan_CHB) == LOW ? EncKanan.pulseCnt-- : EncKanan.pulseCnt++;
  }
}

void encKiri_INTT_A() {
  if (digitalRead(encKiri_CHA) == HIGH) {
    digitalRead(encKiri_CHB) == LOW ? EncKiri.pulseCnt-- : EncKiri.pulseCnt++;
  } else {
    digitalRead(encKiri_CHB) == HIGH ? EncKiri.pulseCnt-- : EncKiri.pulseCnt++;
  }
}

void encKiri_INTT_B() {
  if (digitalRead(encKiri_CHA) == HIGH) {
    digitalRead(encKiri_CHB) == HIGH ? EncKiri.pulseCnt-- : EncKiri.pulseCnt++;
  } else {
    digitalRead(encKiri_CHB) == LOW ? EncKiri.pulseCnt-- : EncKiri.pulseCnt++;
  }
}

/*************************************************
 * FUNGSI KOMPAS
 *************************************************/

void initCMPS() {
  selectMux(CH7);
  delay(10);
  mpu.initialize();

  if (!mpu.testConnection()) {
    SerialUSB.println("MPU6050 gagal!");
    while (1)
      ;
  }
  if (mpu.dmpInitialize() == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    cmps.dmpReady = true;
    cmps.packetSize = mpu.dmpGetFIFOPacketSize();
    SerialUSB.println("Siap! Kirim 'r' untuk reset cmps.heading");
  } else {
    SerialUSB.println("DMP gagal!");
    while (1)
      ;
  }
}

void updateCMPS() {
  if (!cmps.dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(cmps.fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, cmps.fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(cmps.ypr, &q, &gravity);

    // === Heading sekarang (deg) ===
    float heading = cmps.ypr[0] * 180.0f / M_PI - cmps.offset;

    // Normalisasi heading ke -180..180
    while (heading > 180) heading -= 360;
    while (heading < -180) heading += 360;

    // === Delta yaw (RAW ala GY-25) ===
    float delta = heading - cmps.lastHeading;

    // Handle wrap-around
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;

    cmps.raw = delta;        // ini "raw yaw change"
    cmps.heading = heading;  // heading stabil
    cmps.lastHeading = heading;
  }
}

void resetCMPS() {
  cmps.heading = cmps.ypr[0] * 180 / M_PI;
}

/*************************************************
 * FUNGSI ODOMETRI
 *************************************************/

void initOdometry() {
  currentPOS.X = lastPOS.X = currentPOS.Y = lastPOS.Y = currentPOS.T = 0;
  EncKanan.pulseCnt = EncBelakang.pulseCnt = EncKiri.pulseCnt = 0;
  EncKanan.lastpulse = EncBelakang.lastpulse = EncKiri.lastpulse = 0;
}

void fullResetENC() {
  initOdometry();
  slaveSerial.println(AT_RESET);
}

void updateOdometry() {
  static unsigned long l_print = 0;
  static uint8_t count1 = 0;

  if (millis() - l_print > 5) {
    l_print = millis();
    updateCMPS();

    // Delta pulsa
    long dL = EncKiri.pulseCnt - EncKiri.lastpulse;
    long dR = EncKanan.pulseCnt - EncKanan.lastpulse;
    long dB = EncBelakang.pulseCnt - EncBelakang.lastpulse;

    // Konversi ke jarak (cm)
    float sL = dL * POS_CONV_FACTOR;
    float sR = dR * POS_CONV_FACTOR;
    float sB = dB * POS_CONV_FACTOR;

    float _t = cmps.heading;
    float headingRad = radians(_t);
    sin_t = sin(headingRad);
    cos_t = cos(headingRad);

    // Perhitungan odometry FTC style
    float dx_body = ((sR * L_L) - (sL * L_R)) / (L_L - L_R);
    float dy_body = sB - (B_OFFSET * radians(cmps.raw));

    // Transformasi body -> global
    currentPOS.X = lastPOS.X + dx_body * cos_t + dy_body * sin_t;
    currentPOS.Y = lastPOS.Y - dx_body * sin_t + dy_body * cos_t;
    currentPOS.T = -_t;

    // Update referensi encoder
    EncKiri.lastpulse = EncKiri.pulseCnt;
    EncKanan.lastpulse = EncKanan.pulseCnt;
    EncBelakang.lastpulse = EncBelakang.pulseCnt;

    // Update posisi terakhir
    lastPOS.X = currentPOS.X;
    lastPOS.Y = currentPOS.Y;

    // Debug print
    count1++;
    if (count1 >= 84 || dataNew) {
      dataNew = false;
      count1 = 0;
      // nanoSerial.print("_ENC KANAN: ");
      // nanoSerial.print(EncKanan.pulseCnt);
      // nanoSerial.print("\t_ENC KIRI: ");
      // nanoSerial.print(EncKiri.pulseCnt);
      // nanoSerial.print("\t_ENC BELAKANG: ");
      // nanoSerial.print(EncBelakang.pulseCnt);

      // SerialUSB.print("\t_RL: ");
      // SerialUSB.print(RPM_RL);
      // SerialUSB.print("\t_RR: ");
      // SerialUSB.print(RPM_RR);
      // SerialUSB.print("\t_FR: ");
      // SerialUSB.print(RPM_FR);
      // SerialUSB.print("\t_FL: ");
      // SerialUSB.print(RPM_FL);
      // SerialUSB.print("\n");
      Serial.print("\tX: ");
      Serial.print(-currentPOS.Y);
      Serial.print("\tY: ");
      Serial.print(currentPOS.X);
      Serial.print("\tT radian: ");
      Serial.print(radians(currentPOS.T));
      Serial.print("\tT: ");
      Serial.println(currentPOS.T);
    }
  }
}

/*************************************************
 * FUNGSI KONTROL GERAK
 *************************************************/

float angleError(float target, float current) {
  float e = target - current;
  while (e > 180) e -= 360;
  while (e < -180) e += 360;
  return e;
}

int Vx_manual = 0;
int Vy_manual = 0;
int w_manual = 0;
bool goXYT(int x, int y, int t) {
  // Variabel lokal
  float s, v, w, Vx, Vy;
  float alpha, TB, dx, dy;
  float w1, w2, w3, w4, v1, v2, v3, v4;
  float FR, FL, RR, RL, dw;
  float Vx_local, Vy_local;

  // Inisialisasi
  v1 = v2 = v3 = v4 = 0;
  w1 = w2 = w3 = w4 = 0;
  v = w = 0;

  // Delta posisi
  dx = x - (-currentPOS.Y);
  dy = y - currentPOS.X;
  s = sqrt(dx * dx + dy * dy);

  // Error sudut
  dw = angleError(t, (-currentPOS.T));

  // Logika kondisi berhenti
  if (s <= MIN_POS_RES) {
    v = 0;
    if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES && Move == true) {
      count++;
      if (count == 2) setRPM(0, 0, 0, 0);
    }
    if (s <= MIN_POS_RES && abs(dw) <= MIN_DEG_RES && Move == false) {
      count = 0;
      setRPM(0, 0, 0, 0);
    }
  }

  // Kontrol angular
  if (abs(dw) > MIN_DEG_RES) {
    w = 2.5 * FuzzyOmega(abs(dw));
    if (dw < 0) w *= -1;
  }

  // Kontrol linear dengan PID
  if (s > MIN_POS_RES) {
    v = PIDLinear(s);
  } else {
    v = 0;
    integral_linear = 0;
  }

  // Faktor rotasi untuk mencegah overshoot
  float rot_factor = constrain(1.0 - (abs(dw) / MAX_DW), 0.3, 1.0);
  v *= rot_factor;

  // Hitung vektor kecepatan
  if (s > 0.0) {
    Vx = v * dx / s;
    Vy = v * dy / s;
  } else {
    Vx = Vy = 0;
  }

  /* TRANSFORMASI GLOBAL TO LOCAL v2 */
  if (kontrolManual) {
    Vx_local = Vx_manual;
    Vy_local = Vy_manual;
    w = w_manual;
  } else {
    Vx_local = (cos_t * Vx) + (sin_t * Vy);
    Vy_local = -(sin_t * Vx) + (cos_t * Vy);
  }

  float k = 0.485;  // Jarak x dan y antara titik pusat roda dengan titik pusat robot

  // Perhitungan kecepatan roda
  w1 = (-Vx_local + Vy_local + w * k);
  w2 = (-Vx_local - Vy_local + w * k);
  w3 = (Vx_local - Vy_local + w * k);
  w4 = (Vx_local + Vy_local + w * k);

  // Konversi ke kecepatan linear [m/s]
  v1 = float(0.01 * w1);
  v2 = float(0.01 * w2);
  v3 = float(0.01 * w3);
  v4 = float(0.01 * w4);

  // Konversi ke RPM
  RL = (60 * v1) / (MATH_PI * 0.15);
  RR = (60 * v2) / (MATH_PI * 0.15);
  FR = (60 * v3) / (MATH_PI * 0.15);  // 0.15 adalah diameter roda
  FL = (60 * v4) / (MATH_PI * 0.15);

  // Set RPM motor
  setRPM(RL, RR, FR, FL);

  // Simpan nilai RPM untuk debugging
  RPM_FR = FR;
  RPM_FL = FL;
  RPM_RR = RR;
  RPM_RL = RL;

  return false;
}

void sendData() {
  static bool waitingACK = false;
  static unsigned long lastSend = 0;
  static uint8_t packet[9];
  static uint8_t lastGrip = 255, lastSph = 255, lastBot = 255, lastTop = 255, lastConveyor = 255, last6 = 255;
  const unsigned long RETRY_INTERVAL = 50;

  // LED blink
  static unsigned long led1Time = 0, led2Time = 0;

  // Cek ACK
  if (Serial3.available()) {
    if (Serial3.read() == 0xAA) {
      waitingACK = false;
      // Trigger LED
      digitalWrite(led1, HIGH);
      led1Time = millis();
    }
  }

  // LED1 mati setelah 100ms, nyalakan LED2
  if (led1Time > 0 && millis() - led1Time >= 100) {
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
    led2Time = millis();
    led1Time = 0;
  }

  // LED2 mati setelah 100ms
  if (led2Time > 0 && millis() - led2Time >= 100) {
    digitalWrite(led2, LOW);
    led2Time = 0;
  }

  // Matikan LED saat tunggu ACK
  if (waitingACK) {
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    led1Time = 0;
    led2Time = 0;
  }

  // Cek data berubah
  bool dataChanged = (_grip_sph != lastGrip) || (_arm_sph != lastSph) || (_arm_box_bot != lastBot) || (_arm_box_top != lastTop) || (_conveyor != lastConveyor) ||  // ← TAMBAH
                     (_tgt6 != last6);

  // Kirim jika data berubah atau retry
  if ((dataChanged && !waitingACK) || (waitingACK && millis() - lastSend >= RETRY_INTERVAL)) {

    if (dataChanged && !waitingACK) {
      // Paket baru
      packet[0] = 0xAA;
      packet[1] = _grip_sph;
      packet[2] = _arm_sph;
      packet[3] = _arm_box_bot;
      packet[4] = _arm_box_top;
      packet[5] = _conveyor;
      packet[6] = _tgt6;

      uint8_t checksum = 0;
      for (int i = 1; i <= 6; i++) checksum ^= packet[i];
      packet[7] = checksum;
      packet[8] = 0x55;

      lastGrip = _grip_sph;
      lastSph = _arm_sph;
      lastBot = _arm_box_bot;
      lastTop = _arm_box_top;
      lastConveyor = _conveyor;  // ← TAMBAH
      last6 = _tgt6;             // ← TAMBAH
    }

    Serial3.write(packet, 9);
    waitingACK = true;
    lastSend = millis();
  }
}

void setRPM(int rlRPM, int rrRPM, int frRPM, int flRPM) {
  uint8_t packet[15];

  packet[0] = 0xAA;  // HeadernanoSerial

  // Slave 1 data
  packet[1] = 0x00;
  packet[2] = (rlRPM >> 8) & 0xFF;
  packet[3] = rlRPM & 0xFF;

  // Slave 2 data
  packet[4] = 0x00;
  packet[5] = (rrRPM >> 8) & 0xFF;
  packet[6] = rrRPM & 0xFF;

  // Slave 3 data
  packet[7] = 0x00;
  packet[8] = (frRPM >> 8) & 0xFF;
  packet[9] = frRPM & 0xFF;

  // Slave 4 data
  packet[10] = 0x00;
  packet[11] = (flRPM >> 8) & 0xFF;
  packet[12] = flRPM & 0xFF;

  // Checksum
  uint8_t checksum = 0;
  for (int i = 1; i <= 12; i++) checksum ^= packet[i];
  packet[13] = checksum;

  packet[14] = 0x55;
  slaveSerial.write(packet, 15);
}

/*************************************************
 * FUNGSI FUZZY
 *************************************************/

float FuzzyOmega(float wSpeed) {
  float Omega[6] = { 0, 10, 90, 120, 150, 180 };
  float Speed[5] = { 0 };
  float output = 0;

  /* Very Near */
  if (wSpeed > Omega[0] && wSpeed <= Omega[1]) {
    Speed[0] = 1;
  } else if (wSpeed > Omega[1] && wSpeed <= Omega[2]) {
    Speed[0] = (Omega[2] - wSpeed) / (Omega[2] - Omega[1]);
  }

  /* Near */
  if (wSpeed > Omega[1] && wSpeed <= Omega[2]) {
    Speed[1] = (wSpeed - Omega[1]) / (Omega[2] - Omega[1]);
  } else if (wSpeed > Omega[2] && wSpeed <= Omega[3]) {
    Speed[1] = (Omega[3] - wSpeed) / (Omega[3] - Omega[2]);
  }

  /* Middle */
  if (wSpeed > Omega[2] && wSpeed <= Omega[3]) {
    Speed[2] = (wSpeed - Omega[2]) / (Omega[3] - Omega[2]);
  } else if (wSpeed > Omega[3] && wSpeed <= Omega[4]) {
    Speed[2] = (Omega[4] - wSpeed) / (Omega[4] - Omega[3]);
  }

  /* Far */
  if (wSpeed > Omega[3] && wSpeed <= Omega[4]) {
    Speed[3] = (wSpeed - Omega[3]) / (Omega[4] - Omega[3]);
  } else if (wSpeed > Omega[4] && wSpeed <= Omega[5]) {
    Speed[3] = (Omega[5] - wSpeed) / (Omega[5] - Omega[4]);
  }

  /* Very Far */
  if (wSpeed <= Omega[4]) {
    Speed[4] = 0;
  } else if (wSpeed > Omega[4] && wSpeed <= Omega[5]) {
    Speed[4] = (wSpeed - Omega[4]) / (Omega[5] - Omega[4]);
  } else {
    Speed[4] = 1;
  }

  // Defuzzifikasi
  float sum = Speed[0] + Speed[1] + Speed[2] + Speed[3] + Speed[4];
  if (sum > 0) {
    output = ((Speed[0] * AVLOW_SPEED) + (Speed[1] * ALOW_SPEED) + (Speed[2] * AMEDIUM_SPEED) + (Speed[3] * AFAST_SPEED) + (Speed[4] * AVFAST_SPEED)) / sum;
  }

  return output;
}

float FuzzyLinear(float vSpeed) {
  float fSpeed[6] = { 0, 35, 80, 130, 180, 230 };
  float Speed[5] = { 0 };
  float output = 0;

  /* Very Near */
  if (vSpeed > fSpeed[0] && vSpeed <= fSpeed[1]) {
    Speed[0] = 1;
  } else if (vSpeed > fSpeed[1] && vSpeed <= fSpeed[2]) {
    Speed[0] = (fSpeed[2] - vSpeed) / (fSpeed[2] - fSpeed[1]);
  }

  /* Near */
  if (vSpeed > fSpeed[1] && vSpeed <= fSpeed[2]) {
    Speed[1] = (vSpeed - fSpeed[1]) / (fSpeed[2] - fSpeed[1]);
  } else if (vSpeed > fSpeed[2] && vSpeed <= fSpeed[3]) {
    Speed[1] = (fSpeed[3] - vSpeed) / (fSpeed[3] - fSpeed[2]);
  }

  /* Middle */
  if (vSpeed > fSpeed[2] && vSpeed <= fSpeed[3]) {
    Speed[2] = (vSpeed - fSpeed[2]) / (fSpeed[3] - fSpeed[2]);
  } else if (vSpeed > fSpeed[3] && vSpeed <= fSpeed[4]) {
    Speed[2] = (fSpeed[4] - vSpeed) / (fSpeed[4] - fSpeed[3]);
  }

  /* Far */
  if (vSpeed > fSpeed[3] && vSpeed <= fSpeed[4]) {
    Speed[3] = (vSpeed - fSpeed[3]) / (fSpeed[4] - fSpeed[3]);
  } else if (vSpeed > fSpeed[4] && vSpeed <= fSpeed[5]) {
    Speed[3] = (fSpeed[5] - vSpeed) / (fSpeed[5] - fSpeed[4]);
  }

  /* Very Far */
  if (vSpeed <= fSpeed[4]) {
    Speed[4] = 0;
  } else if (vSpeed > fSpeed[4] && vSpeed <= fSpeed[5]) {
    Speed[4] = (vSpeed - fSpeed[4]) / (fSpeed[5] - fSpeed[4]);
  } else {
    Speed[4] = 1;
  }

  // Defuzzifikasi
  float sum = Speed[0] + Speed[1] + Speed[2] + Speed[3] + Speed[4];
  if (sum > 0) {
    output = ((Speed[0] * LVLOW_SPEED) + (Speed[1] * LLOW_SPEED) + (Speed[2] * LMEDIUM_SPEED) + (Speed[3] * LFAST_SPEED) + (Speed[4] * LVFAST_SPEED)) / sum;
  }

  return output;
}

/*************************************************
 * FUNGSI PID
 *************************************************/

float PIDLinear(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime_linear) * 0.001;

  if (dt <= 0 || dt > 0.1) dt = 0.02;
  lastTime_linear = now;

  float P = KP_LINEAR * error;

  integral_linear += error * dt;
  integral_linear = constrain(integral_linear, -100, 100);
  float I = KI_LINEAR * integral_linear;

  float D = KD_LINEAR * (error - error_prev_linear) / dt;
  error_prev_linear = error;

  float output = P + I + D;
  return constrain(output, -MAX_V_OUTPUT, MAX_V_OUTPUT);
}

float PIDAngular(float error) {
  unsigned long now = millis();
  float dt = (now - lastTime_angular) * 0.001;

  if (dt <= 0 || dt > 0.1) dt = 0.02;
  lastTime_angular = now;

  float P = KP_ANGULAR * error;

  integral_angular += error * dt;
  integral_angular = constrain(integral_angular, -100, 100);
  float I = KI_ANGULAR * integral_angular;

  float D = KD_ANGULAR * (error - error_prev_angular) / dt;
  error_prev_angular = error;

  float output = P + I + D;
  return constrain(output, -MAX_V_OUTPUT, MAX_V_OUTPUT);
}

void resetPID() {
  error_prev_linear = integral_linear = 0;
  lastTime_linear = 0;
  error_prev_angular = integral_angular = 0;
  lastTime_angular = 0;
}

/*************************************************
 * FUNGSI SENSOR DAN SOLENOID
 *************************************************/

void initSensor() {
  for (uint8_t i = 0; i < 5; i++) pinMode(sensorPin[i], INPUT);
}

void initSolenoid() {
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(solPin[i], OUTPUT);
    digitalWrite(solPin[i], LOW);
  }
}

void setSolenoid(uint8_t mask) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(solPin[i], (mask >> i) & 1);
  }
}

void sensor_solenoid() {
  uint32_t now = millis();

  /* Baca sensor -> bitmask */
  uint8_t sensorMask = 0;
  for (uint8_t i = 0; i < 5; i++) {
    sensorMask |= (digitalRead(sensorPin[i]) << i);
  }

  /* KONDISI AWAL (AUTO DETECT) */
  if (!kondisiLocked && mode == 0) {
    if (sensorMask == 0b11111) {
      kondisi = false;  // pattern 0
      kondisiLocked = true;
    } else if (sensorMask == 0b11000) {
      kondisi = true;  // pattern 1
      kondisiLocked = true;
    }
  }

  /* RESET MODE */
  if (waitReset) {
    if (now - tReset >= 1000) {
      mode = 0;
      waitReset = false;
      kondisiLocked = false;
    }
    return;
  }

  /* PILIH PATTERN */
  const uint8_t* sensorPattern = kondisi ? pattern1 : pattern0;
  const uint8_t* solPattern = kondisi ? solPattern1 : solPattern0;
  const uint8_t patternLen = kondisi ? sizeof(pattern1) : sizeof(pattern0);

  /* FSM SENSOR */
  if (mode < patternLen && sensorMask == sensorPattern[mode]) {
    mode++;
    if (mode == patternLen) {
      tReset = now;
      waitReset = true;
    }
  }

  // Serial2.println(mode);

  /* SOLENOID EDGE TRIGGER */
  if (mode != lastMode && mode < patternLen) {
    setSolenoid(solPattern[mode]);
    solActive = true;
    tSol = now;
    lastMode = mode;
  }

  /* SOLENOID AUTO OFF */
  if (solActive && now - tSol >= 1000) {
    setSolenoid(0);
    solActive = false;
  }
}

/*************************************************
 * FUNGSI KONTROL UTAMA
 *************************************************/

void controlCompute() {
  struct data {
    int16_t p1, p2, p3;
  } targetXYT;

  bool MoveGoal = false;
  bool writeODM = false;
  uint16_t speed_manual = 100;

  // Waypoints untuk MoveGoal
  int ZoneArr[][3] = {
    { 0, 250, 0 },
    { -20, 250, -90 },
    { -20, 300, -90 },
    { 0, 100, 0 },
    { 0, 250, 0 },
    { 0, 50, -90 },
    { -20, 50, -90 },
    { 0, 100, 0 },
    { 0, 0, 0 },
  };
  int zoneCount = sizeof(ZoneArr) / sizeof(ZoneArr[0]);

  while (1) {
    sensor_solenoid();
    updateOdometry();

    /* Pemrosesan perintah ODM */
    if (writeODM) {
      if (goXYT(targetXYT.p1, targetXYT.p2, targetXYT.p3)) {
        writeODM = false;
      }
    }

    /*kontrol manual*/
    if (kontrolManual) {
      // SerialUSB.println("coba");
      if (cmpSerial.available()) {
        char IR64_manual = cmpSerial.read();
        SerialUSB.println(IR64_manual);
        dataNew = true;
        // SerialUSB.println(IR64_manual);
        if (IR64_manual == 'W') {  // START
          Vx_manual = 0;
          Vy_manual = speed_manual;
          w_manual = 0;
        }
        if (IR64_manual == 'S') {  // START
          Vx_manual = 0;
          Vy_manual = -speed_manual;
          w_manual = 0;
        }
        if (IR64_manual == 'A') {  // START
          Vx_manual = -speed_manual;
          Vy_manual = 0;
          w_manual = 0;
        }
        if (IR64_manual == 'D') {  // START
          Vx_manual = speed_manual;
          Vy_manual = 0;
          w_manual = 0;
        }
        if (IR64_manual == 'R') {  // START
          Vx_manual = 0;
          Vy_manual = 0;
          w_manual = speed_manual;
        }
        if (IR64_manual == 'L') {  // START
          Vx_manual = 0;
          Vy_manual = 0;
          w_manual = -speed_manual;

          // targetXYT = { 0, -254, 0 };
        }
        if (IR64_manual == 'X') {  // START
          Vx_manual = 0;
          Vy_manual = 0;
          w_manual = 0;
        }
        if (IR64_manual == 'O') {  // START
          speed_manual += 50;
          if (speed_manual > 400) speed_manual = 400;
        }
        if (IR64_manual == 'I') {  // START
          speed_manual -= 50;
          if (speed_manual < 50) speed_manual = 50;
        }
        if (IR64_manual == 'B') {
          if (_arm_box_bot == 90) {
            _arm_box_bot = 25;
          } else {
            _arm_box_bot = 90;
          }
        }
        if (IR64_manual == 'C') {
          if (stateGripperSph == 0) {
            stateGripperSph = 1;
          } else if (stateGripperSph == 1) {
            stateGripperSph = 2;
            lTimeGripper = millis();
          } else if (stateGripperSph == 2) {
            lTimeGripper = millis();
          //   stateGripperSph = 3;
          // } else if (stateGripperSph == 3) {
          //   lTimeGripper = millis();
            stateGripperSph = 1;
          }
        }
        if (IR64_manual == 'E') {
          if (_arm_box_top == 90) {
            _arm_box_top = 135;
          } else {
            _arm_box_top = 90;
          }
        }
        if (IR64_manual == 'F') {
          if (_conveyor == 120) {
            _conveyor = 60;  // keluar
          } else if (_conveyor == 60) {
            _conveyor = 90;
          } else {
            _conveyor = 120;  // masuk
          }
        }
      }

      if (stateGripperSph == 0) {
        _arm_sph = 15;
        lTimeGripper = millis();
      } else if (stateGripperSph == 1) {
        if (millis() - lTimeGripper <= 1000) {
          _grip_sph = 160;  //buka
        } else {
          _grip_sph = 90;
        }
        _arm_sph = 83;
      } else if (stateGripperSph == 2) {
        if (millis() - lTimeGripper <= 1000) {
          _grip_sph = 10;  //tutup
        } else {
          _grip_sph = 70;
          _arm_sph = 145;
        }
      // } else if (stateGripperSph == 3) {
      }
      goXYT(targetXYT.p1, targetXYT.p2, targetXYT.p3);
    }

    /* Pemrosesan MoveGoal */
    if (MoveGoal) {
      if (count < zoneCount) {
        goXYT(ZoneArr[count][0], ZoneArr[count][1], ZoneArr[count][2]);

        // Adjust toleransi untuk waypoint tertentu
        if (count == 1 || count == 2) {
          MIN_POS_RES = 5.0;
          MIN_DEG_RES = 5.0;
        } else {
          MIN_POS_RES = 3.0;
          MIN_DEG_RES = 3.0;
        }
      } else {
        setRPM(0, 0, 0, 0);
        count = zoneCount - 1;  // Last Count
      }
    }

    /* Pemrosesan Serial Commands */
    static String cmd = "";
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n') {
        if (cmd.length() > 0) {
          // Command satu karakter
          if (cmd.length() == 1) {
            char IR64 = cmd[0];
            SerialUSB.println(IR64);

            switch (IR64) {
              case 'O':  // START
                writeODM = false;
                MoveGoal = true;
                Move = true;
                count = 0;
                break;

              case 'W':  // L3 - Maju
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 0, 254, 0 };
                resetPID();
                break;

              case 'S':  // R3 - Mundur
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 0, -254, 0 };
                resetPID();
                break;

              case 'D':  // Triangle - Kanan
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 100, 0, 0 };
                resetPID();
                break;

              case 'A':  // Cross - Kiri
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { -100, 0, 0 };
                resetPID();
                break;

              case 'X':  // Rotasi 180
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 0, 0, 180 };
                resetPID();
                break;

              case 'C':  // Reset orientasi
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 0, 0, 0 };
                resetPID();
                break;

              case 'Z':  // Stop
                writeODM = false;
                MoveGoal = false;
                Move = false;
                setRPM(0, 0, 0, 0);
                resetPID();
                break;

              case 'U':  // Kiri atas
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { -100, 100, 0 };
                resetPID();
                break;

              case 'I':  // Kanan atas
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 100, 100, 0 };
                resetPID();
                break;

              case 'J':  // Kiri bawah
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { -100, -100, 0 };
                resetPID();
                break;

              case 'K':  // Kanan bawah
                writeODM = true;
                MoveGoal = false;
                Move = false;
                targetXYT = { 100, -100, 0 };
                resetPID();
                break;

              case 'R':  // Reset encoder
                writeODM = false;
                MoveGoal = false;
                Move = false;
                setRPM(0, 0, 0, 0);
                fullResetENC();
                resetPID();
                break;

              case 'T':  // Reset kompas
                writeODM = false;
                MoveGoal = false;
                Move = false;
                setRPM(0, 0, 0, 0);
                resetCMPS();
                resetPID();
                break;

              case 'M':  // Waypoint khusus
                kontrolManual = true;
                break;

              case 'N':  // Waypoint khusus
                kontrolManual = false;
                break;

              case 'L':  // Set heading 90
                cmps.heading = 90.0;
                break;
            }
          }

          // Command dengan parameter (Q,x,y,t)
          if (cmd[0] == 'Q') {
            int i1 = cmd.indexOf(',');
            int i2 = cmd.indexOf(',', i1 + 1);
            int i3 = cmd.indexOf(',', i2 + 1);

            if (i1 > 0 && i2 > 0 && i3 > 0) {
              writeODM = true;
              MoveGoal = false;
              Move = false;

              targetXYT.p1 = cmd.substring(i1 + 1, i2).toInt();
              targetXYT.p2 = cmd.substring(i2 + 1, i3).toInt();
              targetXYT.p3 = cmd.substring(i3 + 1).toInt();

              resetPID();
            }
          }
        }
        cmd = "";
      } else {
        cmd += c;
      }
    }

    /* Debugging output */
    static unsigned long l_print = 0;
    if (millis() - l_print > 50) {
      l_print = millis();
      // Tambahkan output debugging sesuai kebutuhan
    }

    // Tambahkan pemrosesan sensor_solenoid jika diperlukan
    // sensor_solenoid();
    sendData();
  }
}

/*************************************************
 * FUNGSI TEST
 *************************************************/

void trialEncMaster() {
  while (1) {
    updateOdometry();

    SerialUSB.print("BE: ");
    SerialUSB.print(EncBelakang.pulseCnt);
    SerialUSB.print("\tKA: ");
    SerialUSB.print(EncKanan.pulseCnt);
    SerialUSB.print("\tKI: ");
    SerialUSB.print(EncKiri.pulseCnt);
    SerialUSB.print("\tX: ");
    SerialUSB.print(currentPOS.X);
    SerialUSB.print("\tY: ");
    SerialUSB.print(currentPOS.Y);
    SerialUSB.print("\tT: ");
    SerialUSB.println(currentPOS.T);

    Serial1.print("BE: ");
    Serial1.print(EncBelakang.pulseCnt);
    Serial1.print("\tKA: ");
    Serial1.print(EncKanan.pulseCnt);
    Serial1.print("\tKI: ");
    Serial1.print(EncKiri.pulseCnt);
    Serial1.print("\tX: ");
    Serial1.print(currentPOS.X);
    Serial1.print("\tY: ");
    Serial1.print(currentPOS.Y);
    Serial1.print("\tT: ");
    Serial1.println(currentPOS.T);

    delay(20);
  }
}