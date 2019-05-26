#include <Wire.h>
#include <TimedAction.h>  // for multi-thread

//#define TEST_CODE
#define DEBUG
#define UNO
#ifndef UNO
  #define MEGA2560
#endif

// I2C
#define ADDR_SLAVE 0x04
#define LEN_DATA_I2C_IN 2
#define ID_IN_MOTOR_LEFT 0
#define ID_IN_MOTOR_RIGHT 1
uint8_t data_i2c_in[LEN_DATA_I2C_IN];
int id_in = 0;
#define LEN_DATA_I2C_OUT 16
#define ID_OUT_ODOM_LEFT 0
#define ID_OUT_ODOM_RIGHT 4
#define ID_OUT_VEL_LEFT 8
#define ID_OUT_VEL_RIGHT 12
uint8_t data_i2c_out[LEN_DATA_I2C_OUT];

// Motor
const int MAX_MOTOR_VAL = 220;
const float SCALE_MOTOR = (float)MAX_MOTOR_VAL / 128.0;
const float SHIFT_MOTOR = -128.0;
// left/right motor values
float pwm_ml = 0;
float pwm_mr = 0;

// left
// IN1: LOW IN2: +    -> forward
// IN1: +   IN2: LOW  -> backward
#ifdef UNO
  #define IN1 5
  #define IN2 6
#elif MEGA2560
  #define IN1 11
  #define IN2 12
#endif
// right
// IN3: +   IN4: LOW  -> forward
// IN3: LOW IN4: +    -> backward
#define IN3 10
#define IN4 9

#define ENC_A_LEFT 2
#define ENC_B_LEFT 4
#define ENC_A_RIGHT 3
#ifdef UNO
  #define ENC_B_RIGHT 7
#elif MEGA2560
  #define ENC_B_RIGHT 5
#endif

const float EPSILON = 0.0001;
// PPR (Pulses Per Revolution)
const float ENCODEROUTPUT = 11.0;
// circumference of the wheel. unit: meter
const float CIRCUMFERENCE = M_PI * 0.035;
const float VAL_ENC2RPM = 60.0 / (2.0 * ENCODEROUTPUT);
const float VAL_RPM2VEL = CIRCUMFERENCE / 60.0;
// NOTE: forward is positive for pos & vel
// TODO: add mutex
float rpm_l = 0, rpm_r = 0, vel_l = 0, vel_r = 0, odom_l = 0, odom_r = 0;

union float2Bytes {
  char buf[4];
  float val;
} conv_f2b;

volatile long enc_pul_l = 0;
volatile long enc_pul_r = 0;
// unit: ms
volatile int ts_prev_enc = 0, ts_now_enc = 0, dt = 0, ts_interval = 1000;
const float TS_MS2S = 0.001;

// unit: ms
const float interval2Compute = 250;
void computeEncoder();
TimedAction computeEncoderThread = TimedAction(interval2Compute, computeEncoder);

// Test code
const int len_foo = 8;
int foo[len_foo] = {0, 60, 100, 60, 0, -60, -100, -60};
int id_foo = 0;
void testMotorControl();
#ifdef TEST_CODE
TimedAction testThread = TimedAction(3000, testMotorControl);
#endif

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("motor's i2c control");
  Serial.println("-------------------");
#endif

  // Init I2C as slave
  Wire.begin(ADDR_SLAVE);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_A_LEFT, INPUT);
  pinMode(ENC_B_LEFT, INPUT);
  pinMode(ENC_A_RIGHT, INPUT);
  pinMode(ENC_B_RIGHT, INPUT);

  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A_LEFT), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A_RIGHT), rightEncoderEvent, CHANGE);

#ifdef TEST_CODE
  testThread.check();
#endif
}

// encoder event for the interrupt call
void leftEncoderEvent() {
  if (digitalRead(ENC_A_LEFT) == HIGH) {
    if (digitalRead(ENC_B_LEFT) == LOW) {
      enc_pul_l++;
    } else {
      enc_pul_l--;
    }
  } else {
    if (digitalRead(ENC_B_LEFT) == LOW) {
      enc_pul_l--;
    } else {
      enc_pul_l++;
    }
  }
}

// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(ENC_A_RIGHT) == HIGH) {
    if (digitalRead(ENC_B_RIGHT) == LOW) {
      enc_pul_r++;
    } else {
      enc_pul_r--;
    }
  } else {
    if (digitalRead(ENC_B_RIGHT) == LOW) {
      enc_pul_r--;
    } else {
      enc_pul_r++;
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
#ifdef TEST_CODE
  testThread.check();
#endif

  computeEncoderThread.check();
}

// callback for received data
void receiveData(int byteCount) {
  id_in = 0;
  while (Wire.available()) {
    if (id_in >= LEN_DATA_I2C_IN) {
      Serial.println("[ERR] data out of buffer.");
      break;
    }

    data_i2c_in[id_in++] = Wire.read();
  }
#ifdef DEBUG
  Serial.println("receiveData");
  for (int i = 0; i < LEN_DATA_I2C_IN; ++i) {
    Serial.print(data_i2c_in[i]);
    Serial.print(" ");
  }
  Serial.println();
#endif

  motorControl();
}

// callback for sending data
void sendData() {
  // disassemble pos, vel to 8 bit for transfer
  conv_f2b.val = odom_l;
  for (int i = 0; i < 4; ++i) {
    data_i2c_out[ID_OUT_ODOM_LEFT + i] = conv_f2b.buf[i];
  }
  conv_f2b.val = odom_r;
  for (int i = 0; i < 4; ++i) {
    data_i2c_out[ID_OUT_ODOM_RIGHT + i] = conv_f2b.buf[i];
  }
  conv_f2b.val = vel_l;
  for (int i = 0; i < 4; ++i) {
    data_i2c_out[ID_OUT_VEL_LEFT + i] = conv_f2b.buf[i];
  }
  conv_f2b.val = vel_r;
  for (int i = 0; i < 4; ++i) {
    data_i2c_out[ID_OUT_VEL_RIGHT + i] = conv_f2b.buf[i];
  }

  Wire.write(data_i2c_out, LEN_DATA_I2C_OUT);

#ifdef DEBUG
  Serial.println("sendData");
  for (int i = 0; i < LEN_DATA_I2C_OUT; ++i) {
    Serial.print(data_i2c_out[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("equal to");
  Serial.print(odom_l, 6);
  Serial.print(" ");
  Serial.print(odom_r, 6);
  Serial.print(" ");
  Serial.print(vel_l, 6);
  Serial.print(" ");
  Serial.print(vel_r, 6);
  Serial.println();
#endif
}

void testMotorControl() {
  Serial.println("M<ooootor> PWM set to: ");
  Serial.println(foo[id_foo]);
  if (foo[id_foo] >= 0) {
    pwm_ml = foo[id_foo];
    pwm_mr = foo[id_foo];
    pwm_ml = (pwm_ml < MAX_MOTOR_VAL ? pwm_ml : MAX_MOTOR_VAL) > -MAX_MOTOR_VAL ? pwm_ml : -MAX_MOTOR_VAL;
    pwm_mr = (pwm_mr < MAX_MOTOR_VAL ? pwm_mr : MAX_MOTOR_VAL) > -MAX_MOTOR_VAL ? pwm_mr : -MAX_MOTOR_VAL;
    
    analogWrite(IN1, LOW);
    analogWrite(IN2, (uint8_t)pwm_ml);
    analogWrite(IN3, (uint8_t)pwm_mr);
    analogWrite(IN4, LOW);
  } else {
    pwm_ml = -foo[id_foo];
    pwm_mr = -foo[id_foo];
    pwm_ml = (pwm_ml < MAX_MOTOR_VAL ? pwm_ml : MAX_MOTOR_VAL) > -MAX_MOTOR_VAL ? pwm_ml : -MAX_MOTOR_VAL;
    pwm_mr = (pwm_mr < MAX_MOTOR_VAL ? pwm_mr : MAX_MOTOR_VAL) > -MAX_MOTOR_VAL ? pwm_mr : -MAX_MOTOR_VAL;

    analogWrite(IN1, (uint8_t)pwm_ml);
    analogWrite(IN2, LOW);
    analogWrite(IN3, LOW);
    analogWrite(IN4, (uint8_t)pwm_mr);
  }
  id_foo = id_foo + 1 == len_foo ? 0 : id_foo + 1;
  Serial.println("M<ooootor> - END");
}

void computeEncoder() {
  ts_now_enc = millis();
  dt = ts_now_enc - ts_prev_enc;
  if (dt > ts_interval) {
    ts_prev_enc = ts_now_enc;

    float val = VAL_ENC2RPM / (float)dt;
    rpm_l = (float)enc_pul_l * val;
    rpm_r = (float)enc_pul_r * val;

    vel_l = rpm_l * VAL_RPM2VEL;
    vel_r = -rpm_r * VAL_RPM2VEL;   // convert direction

    odom_l += vel_l * dt * TS_MS2S;
    odom_r += vel_r * dt * TS_MS2S;

#ifdef DEBUG
    if (abs(pwm_ml - 0.0) > EPSILON && abs(pwm_mr - 0.0) > EPSILON) {
      Serial.print("[encoder] l: ");
      Serial.print(enc_pul_l);
      Serial.print(" r: ");
      Serial.println(enc_pul_r);
      Serial.print("est. rpm_l: ");
      Serial.print(rpm_l, 6);
      Serial.print(" rpm_r: ");
      Serial.println(rpm_r, 6);
      Serial.print("est. vel_l: ");
      Serial.print(vel_l, 6);
      Serial.print(" vel_r: ");
      Serial.println(vel_r, 6);
      Serial.print("est. pos_l: ");
      Serial.print(odom_l, 6);
      Serial.print(" pos_r: ");
      Serial.print(odom_r, 6);
      Serial.println();
    }
#endif

    enc_pul_l = enc_pul_r = 0;
  }
}

void motorControl() {
  pwm_ml = ((float)data_i2c_in[ID_IN_MOTOR_LEFT] + SHIFT_MOTOR) * SCALE_MOTOR;
  pwm_mr = ((float)data_i2c_in[ID_IN_MOTOR_RIGHT] + SHIFT_MOTOR) * SCALE_MOTOR;
  pwm_ml = (pwm_ml < MAX_MOTOR_VAL ? pwm_ml : MAX_MOTOR_VAL) > -MAX_MOTOR_VAL ? pwm_ml : -MAX_MOTOR_VAL;
  pwm_mr = (pwm_mr < MAX_MOTOR_VAL ? pwm_mr : MAX_MOTOR_VAL) > -MAX_MOTOR_VAL ? pwm_mr : -MAX_MOTOR_VAL;
#ifdef DEBUG
  Serial.print("output to motor: ");
  Serial.print(pwm_ml);
  Serial.print(" ");
  Serial.print(pwm_mr);
  Serial.println();
#endif

  // output motor
  if (pwm_ml >= 0.0) {
    analogWrite(IN1, LOW);
    analogWrite(IN2, (uint8_t)pwm_ml);
  } else {
    analogWrite(IN1, (uint8_t)-pwm_ml);
    analogWrite(IN2, LOW);
  }

  if (pwm_mr >= 0.0) {
    analogWrite(IN3, (uint8_t)pwm_mr);
    analogWrite(IN4, LOW);
  } else {
    analogWrite(IN3, LOW);
    analogWrite(IN4, (uint8_t)-pwm_mr);
  }
}
