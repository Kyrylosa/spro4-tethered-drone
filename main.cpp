#include <Arduino.h>
#include <deque>
// GET THE PIN VALUES FROM THE SCHEMATICS ON DISCORD
// Motor Left motor A
#define PWM_LEFT_DIRECTION 1       // switch sign if the direction is wrong
#define PWM_LEFT_PIN_3 GPIO_NUM_37 // nmos 3 pwm output has channel 1 from left
#define PWM_LEFT_PIN_4 GPIO_NUM_36 // nmos 4 pwm output has channel 2 from left
#define PWM_CHANNEL_LEFT_1 0
#define PWM_CHANNEL_LEFT_2 1
#define MOTOR_LEFT_PIN_1 GPIO_NUM_38 // pmos 1 enable pin high/low
#define MOTOR_LEFT_PIN_2 GPIO_NUM_48 // pmos 2 enable pin high/low
#define ENC_LEFT_DIRECTION -1
#define ENCODER_LEFT_PIN_1 GPIO_NUM_13 // input pin from encoder
#define ENCODER_LEFT_PIN_2 GPIO_NUM_12 // input pin from encoder
volatile int LEFT = 0;
int LEFT_RPM = 0;
volatile unsigned long LEFT_last_pulse_time = 0;
volatile unsigned long LEFT_time_between_pulses_change = 0;
volatile unsigned long LEFT_time_between_pulses = -1; // in microseconds
//  Motor Right motor B
#define PWM_RIGHT_DIRECTION -1      // switch sign if the direction is wrong
#define PWM_RIGHT_PIN_3 GPIO_NUM_35 // nmos 3 pwm output, has channel 1 from right
#define PWM_RIGHT_PIN_4 GPIO_NUM_14 // nmos 4 pwm output, has channel 2 from right
#define PWM_CHANNEL_RIGHT_1 2
#define PWM_CHANNEL_RIGHT_2 3
#define MOTOR_RIGHT_PIN_1 GPIO_NUM_47 // pmos 1 enable pin high/low
#define MOTOR_RIGHT_PIN_2 GPIO_NUM_21 // pmos 2 enable pin high/low
#define ENC_RIGHT_DIRECTION 1
#define ENCODER_RIGHT_PIN_1 GPIO_NUM_11 // input pin from encoder
#define ENCODER_RIGHT_PIN_2 GPIO_NUM_10 // input pin from encoder
volatile int RIGHT = 0;
int RIGHT_RPM = 0;
volatile unsigned long RIGHT_last_pulse_time = 0;
volatile unsigned long RIGHT_time_between_pulses_change = 0;
volatile unsigned long RIGHT_time_between_pulses = -1; // in microseconds
//  pwm spec
#define PWM_FREQ 1000 // hz
#define PWM_RESOLUTION 10
#define PWM_MAX_DUTY ((1 << PWM_RESOLUTION) - 1)                         // 2^resolution -1
#define MAX_ALLOWED_TORQUE 0.7                                           // Nm
#define CPR_ENCODER 897.96f                                              // 3591.8f                                           // pulses per revolution
#define WHEEL_RADIUS 0.05f                                               // in meters
#define WHEEL_WIDTH 0.245f                                               // in meters
#define PULSE_DISTANCE (2.0f * 3.14159265f * WHEEL_RADIUS) / CPR_ENCODER // meters per pulse
#define ROBOT_ANGLE_PER_PULSE atan2f(PULSE_DISTANCE, WHEEL_WIDTH)        // radians per pulse, positive is CCW
enum MotorState
{              // for H bridge, 1 and 2 are for VCC pins and turn on with LOW, 3 and 4 are for GND pins and turn on with HIGH, we do PWM on 3 and 4
  FORWARD,     // LHLH`
  MIDFORWARD,  // HHLH
  STOPPED,     // HHHH
  MIDBACKWARD, // HHHL
  BACKWARD     // HLHL
};
MotorState LeftMotorState = STOPPED;
MotorState RightMotorState = STOPPED;
enum Motor
{
  LEFT_MOTOR,
  RIGHT_MOTOR
};
// Navigation setup
typedef struct
{
  float X;
  float Y;
  float distanceTillFullStop;
  int direction; // 1 for forward, -1 for backward
} navPoint;
std::deque<navPoint> navPoints;
navPoint currentTargetPoint; // points are removed when they are reached
bool switchToNextPoint = false;
// ON/OFF switch for the controller
bool movementEnabled = false;
float distanceToTarget = 0.0F; // sqrtf((currentTargetPoint.X - currentX) * (currentTargetPoint.X - currentX) + (currentTargetPoint.Y - currentY) * (currentTargetPoint.Y - currentY));
// Where in the map is the sensor stand
#define standX 0.0F
#define standY 0.0F

// Communication setup
#define INDICATOR_LED_PIN GPIO_NUM_8 // high/low
#define UART_RX_PIN GPIO_NUM_44      // rx U0
#define UART_TX_PIN GPIO_NUM_43      // tx U0
#define SERIAL_BAUD 115200           // bps
enum commandType
{
  STOP,
  SET_HOME,
  RETURN_HOME,
  GO_TO_XYmode,
  DO_MOTION_TAN,
  DO_MOTION_NORM,
  CHARGING
};
// Movement controls
enum movementStage
{
  IDLE,
  ROTATING,
  ACCELERATING,
  CRUISING,
  DECELERATING
};
movementStage currentStageMovement = IDLE;
enum movementType
{
  MOTION_TANGENTIAL,
  MOTION_RADIAL
};

// Project requirements
#define TARGET_ACCELERATION 0.4             // m/s^2
#define ACCELERATION_ALLOWED_DEVIATION 0.05 // +-m/s^2
#define TARGET_VELOCITY 0.5                 // m/s
#define VELOCITY_ALLOWED_DEVIATION 0.05     // +-m/s
#define TARGET_DECELERATION -0.75           // m/s^2
#define DECELERATION_ALLOWED_DEVIATION 0.05 // +-m/s^2
navPoint startAccelerationPoint;
unsigned long startAcceleration = 0;
navPoint stopAccelerationPoint;
unsigned long stopAcceleration = 0;
float minVelocity = infinityf();
float maxVelocity = -infinityf();
navPoint startDecelerationPoint;
unsigned long startDeceleration = 0; 
navPoint stopDecelerationPoint;
unsigned long stopDeceleration = 0;

#define LOOP_FREQ 200
unsigned long my_time = 0;
unsigned long print_time = 0;
// Acceleration PID constants
#define ACC_PID_P 150.0F  // 1000 //LOOP_FREQ
#define ACC_PID_I 5600.0F // 40000
#define ACC_PID_D 0
float ACCerrorI = 0;
float lastACCerror = 0;
// Rotational PID constants
// 25000.0F oscillation period 0,0625 seconds
#define ROT_PID_P 0.0F
#define ROT_PID_I 0.0F
#define ROT_PID_D 0.0F
float ROTerrorI = 0;
float lastROTerror = 0;

//
enum TestType
{
  TEST_SETDUYTRAW,
  TEST_SETMOTORDUTY,
  TEST_PID,
  TEST_PIDCONTROLLOOP,
  TEST_STEERING180DEG,
  TEST_STEERING90DEG,
  TEST_2METER_DISTANCE,
  TEST_2METER_STRAIGHT,
  TEST_2METER_45DEG,
  TEST_2METER_90DEG,
  TEST_2METER_180DEG,
  TEST_NAV_SQUARE
};
// Functions declarations
void pwmInit();
void encoderInit();
void setDutyRaw(int channel, uint32_t value);
void setMotorDuty(Motor motor, MotorState *state, int duty); // meant for +- as input
void processSerial();
void encoderLeft();
void encoderRight();
void updateVelocityAcc();
void PIDtoPWM(int tanPID, int rotPID);
void PIDcontrolLoop(float distanceToTarget, float targetAngle);
int PID_ACC_control(float targetAcceleration);
int PID_ROT_control(float targetAngle);
int DUTY_limiter(int DUTY, Motor motor);
void executePath(void);
void planPath(movementType type, float targetX, float targetY);
void updateDistanceToTarget(navPoint *first, navPoint *second);
void test(TestType type);
bool evaluateRequirements(void);
// XY coordinates system
float currentX = 0;     // units in meters, at 10 meters the precision is 0.92um, we are getting arount 87,465um per encoder
float currentY = 0;     // units in meters, at 10 meters the precision is 0.92um, we are getting arount 87,465um per encoder
float currentAngle = 0; // units in radians, positive is CCW, angle is counted from the X-axis

float velocity = 0;
float acceleration = 0;
std::deque<float> velHistory;
float velHistorySum = 0;
#define VEL_HISTORY_SIZE 10
std::deque<float> accHistory;
float accHistorySum = 0;
#define ACC_HISTORY_SIZE 5
float lastX = 0;
float lastY = 0;
float lastVelocity = 0;

// debugging variables
int testVar = 0;
int L = 0;
int R = 0;
bool doOnce = true;
bool finished = false;

void setup()
{
  Serial.begin(SERIAL_BAUD);
  pwmInit();
  encoderInit();
  delay(5000);
  Serial.println("Starting");
  // if (currentStageMovement == IDLE)
  //{
  //   currentStageMovement = ACCELERATING;
  // }
}

void loop()
{
  if (millis() > my_time)
  {
    // my_time = millis() + 1000 / LOOP_FREQ; // to run x tims per second
    my_time += 1000 / LOOP_FREQ;
    updateVelocityAcc();
    //  executePath();
    //  processSerial();
    test(TEST_2METER_DISTANCE);
  }

  // other functions....
}
void pwmInit()
{
  ledcSetup(PWM_CHANNEL_LEFT_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT_2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_LEFT_PIN_3, PWM_CHANNEL_LEFT_1);
  ledcAttachPin(PWM_LEFT_PIN_4, PWM_CHANNEL_LEFT_2);
  ledcAttachPin(PWM_RIGHT_PIN_3, PWM_CHANNEL_RIGHT_1);
  ledcAttachPin(PWM_RIGHT_PIN_4, PWM_CHANNEL_RIGHT_2);

  // start with high duty
  setDutyRaw(PWM_CHANNEL_LEFT_1, PWM_MAX_DUTY);
  setDutyRaw(PWM_CHANNEL_LEFT_2, PWM_MAX_DUTY);
  setDutyRaw(PWM_CHANNEL_RIGHT_1, PWM_MAX_DUTY);
  setDutyRaw(PWM_CHANNEL_RIGHT_2, PWM_MAX_DUTY);
  pinMode(INDICATOR_LED_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_1, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_2, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_1, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_2, OUTPUT);
  digitalWrite(MOTOR_LEFT_PIN_1, HIGH);  // disable motor driver
  digitalWrite(MOTOR_LEFT_PIN_2, HIGH);  // disable motor driver
  digitalWrite(MOTOR_RIGHT_PIN_1, HIGH); // disable motor driver
  digitalWrite(MOTOR_RIGHT_PIN_2, HIGH); // disable motor driver
}
void encoderInit()
{
  pinMode(ENCODER_LEFT_PIN_1, INPUT);
  pinMode(ENCODER_LEFT_PIN_2, INPUT);
  pinMode(ENCODER_RIGHT_PIN_1, INPUT);
  pinMode(ENCODER_RIGHT_PIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN_1), encoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN_1), encoderRight, RISING);
}
void IRAM_ATTR encoderLeft()
{
  unsigned long current_time = micros();
  if (current_time - LEFT_last_pulse_time < LEFT_time_between_pulses * 3 / 5 && current_time - LEFT_last_pulse_time > LEFT_time_between_pulses * 2 / 5)
  {
    return; // debounce
  }
  //LEFT_time_between_pulses_change = current_time - LEFT_last_pulse_time - LEFT_time_between_pulses;
  LEFT_time_between_pulses = current_time - LEFT_last_pulse_time;
  LEFT_last_pulse_time = current_time;
  // ENCODER PINS FOR ENCODER HAVE TO BE BETWEEN 0 AND 32
  // switch (digitalRead(ENCODER_LEFT_PIN_2)) // to make the code about 1us faster switch ((GPIO.in >> ENCODER_LEFT_PIN_2) & 1)
  switch ((GPIO.in >> ENCODER_LEFT_PIN_2) & 1)
  {
  case HIGH:
    // LEFT += ENC_LEFT_DIRECTION;
    // currentX += 0.5 * PULSE_DISTANCE * cosf(currentAngle) * ENC_LEFT_DIRECTION;
    // currentY += 0.5 * PULSE_DISTANCE * sinf(currentAngle) * ENC_LEFT_DIRECTION;
    // currentAngle -= ROBOT_ANGLE_PER_PULSE * ENC_LEFT_DIRECTION;
    LEFT += ENC_LEFT_DIRECTION; // CHECK THAT THEESE SIGNS MAKE SENSe
    break;
  case LOW:
    // LEFT -= ENC_LEFT_DIRECTION;
    // currentX -= 0.5 * PULSE_DISTANCE * cosf(currentAngle) * ENC_LEFT_DIRECTION;
    // currentY -= 0.5 * PULSE_DISTANCE * sinf(currentAngle) * ENC_LEFT_DIRECTION;
    // currentAngle += ROBOT_ANGLE_PER_PULSE * ENC_LEFT_DIRECTION;
    LEFT -= ENC_LEFT_DIRECTION;
    break;
  default:
    break;
  }
}
void IRAM_ATTR encoderRight()
{
  unsigned long current_time = micros();
  if (current_time - RIGHT_last_pulse_time < RIGHT_time_between_pulses * 3 / 5 && current_time - RIGHT_last_pulse_time > RIGHT_time_between_pulses * 2 / 5)
  {
    return; // debounce
  }
  //RIGHT_time_between_pulses_change = current_time - RIGHT_last_pulse_time - RIGHT_time_between_pulses;
  RIGHT_time_between_pulses = current_time - RIGHT_last_pulse_time;
  RIGHT_last_pulse_time = current_time;
  // switch (digitalRead(ENCODER_RIGHT_PIN_2))
  switch ((GPIO.in >> ENCODER_RIGHT_PIN_2) & 1)
  {
  case HIGH:
    // RIGHT += ENC_LEFT_DIRECTION;
    // currentX += 0.5 * PULSE_DISTANCE * cosf(currentAngle) * ENC_RIGHT_DIRECTION;
    // currentY += 0.5 * PULSE_DISTANCE * sinf(currentAngle) * ENC_RIGHT_DIRECTION;
    // currentAngle += ROBOT_ANGLE_PER_PULSE * ENC_RIGHT_DIRECTION;
    RIGHT += ENC_RIGHT_DIRECTION; // CHECK THEESE SIGNS MAKE SENSE
    break;
  case LOW:
    // RIGHT -= ENC_LEFT_DIRECTION;
    // currentX -= 0.5 * PULSE_DISTANCE * cosf(currentAngle) * ENC_RIGHT_DIRECTION;
    // currentY -= 0.5 * PULSE_DISTANCE * sinf(currentAngle) * ENC_RIGHT_DIRECTION;
    // currentAngle -= ROBOT_ANGLE_PER_PULSE * ENC_RIGHT_DIRECTION;
    RIGHT -= ENC_RIGHT_DIRECTION;
    break;
    // default:
    // break;
  }
}
void updateVelocityAcc()
{
  // Read encoders
  L = LEFT; // No of pulses,
  LEFT -= L;
  // LEFT_RPM = (2 * 3.14159265F / CPR_ENCODER)*L * LOOP_FREQ * 60 / (2 * 3.14159265F);
  LEFT_RPM = 60 * L * LOOP_FREQ / CPR_ENCODER;
  R = RIGHT;
  RIGHT -= R;
  RIGHT_RPM = 60 * R * LOOP_FREQ / CPR_ENCODER;
  float vel_RIGHT = 0;
  //float acc_RIGHT = 0;
  // float RPM_RIGHT = 60000000.0F / (CPR_ENCODER * RIGHT_time_between_pulses);
  if (RIGHT_RPM > 10.0F)
  {
    // RPM_RIGHT = 0.0F;
    vel_RIGHT = constrain(1000000.0F * PULSE_DISTANCE / RIGHT_time_between_pulses, -1.5F, 1.5F); // m/s
    //acc_RIGHT = ((1000000.0F * PULSE_DISTANCE / RIGHT_time_between_pulses) - (1000000.0F * PULSE_DISTANCE / (RIGHT_time_between_pulses - RIGHT_time_between_pulses_change)))*(1000000.0F/RIGHT_time_between_pulses) ;
  }
  else if (RIGHT_RPM < -10.0F)
  {
    // RPM_RIGHT = 0.0F;
    vel_RIGHT = constrain(-1000000.0F * PULSE_DISTANCE / RIGHT_time_between_pulses, -1.5F, 1.5F); // m/s
    //acc_RIGHT = ((-1000000.0F * PULSE_DISTANCE / RIGHT_time_between_pulses) - (-1000000.0F * PULSE_DISTANCE / (RIGHT_time_between_pulses - RIGHT_time_between_pulses_change)))*(1000000.0F/RIGHT_time_between_pulses) ;

  }
  // float RPM_LEFT = 60000000.0F / (CPR_ENCODER * LEFT_time_between_pulses);
  float vel_LEFT = 0;
  //float acc_LEFT = 0;
  if (LEFT_RPM > 10.0F)
  {
    // RPM_LEFT = 0.0F;
    vel_LEFT = constrain(1000000.0F * PULSE_DISTANCE / LEFT_time_between_pulses, -1.5F, 1.5F); // m/s
    //acc_LEFT = ((1000000.0F * PULSE_DISTANCE / LEFT_time_between_pulses) - (1000000.0F * PULSE_DISTANCE / (LEFT_time_between_pulses - LEFT_time_between_pulses_change)))*(1000000.0F/LEFT_time_between_pulses) ;
  }
  else if (LEFT_RPM < -10.0F)
  {
    // RPM_LEFT = 0.0F;
    vel_LEFT = constrain(-1000000.0F * PULSE_DISTANCE / LEFT_time_between_pulses, -1.5F, 1.5F); // m/s
    //acc_LEFT = ((-1000000.0F * PULSE_DISTANCE / LEFT_time_between_pulses) - (-1000000.0F * PULSE_DISTANCE / (LEFT_time_between_pulses - LEFT_time_between_pulses_change)))*(1000000.0F/LEFT_time_between_pulses) ;
  }
  // Update position
  float deltaDistance = ((L + R) / 2.0f) * PULSE_DISTANCE; // meters
  currentX += deltaDistance * cosf(currentAngle);
  currentY += deltaDistance * sinf(currentAngle);
  currentAngle += ((R - L) * ROBOT_ANGLE_PER_PULSE); // radians
  if (currentAngle > 3.14159265F * 1.1F)
  {
    currentAngle -= 2.0f * 3.14159265F;
  }
  else if (currentAngle < -3.14159265F * 1.1F)
  {
    currentAngle += 2.0f * 3.14159265F;
  }
  // Update velocity and acceleration
  float deltaX = currentX - lastX; // meters
  float deltaY = currentY - lastY; // meters
  float distance = sqrtf(deltaX * deltaX + deltaY * deltaY);
  // float newVelocity = distance * LOOP_FREQ; // m/s
  float newVelocity = (vel_LEFT + vel_RIGHT) / 2.0f; // m/s
  /*
  velHistory.push_back(distance * LOOP_FREQ);
  velHistorySum += distance * LOOP_FREQ;
  if (velHistory.size() > VEL_HISTORY_SIZE)
  {
    velHistorySum -= velHistory.front();
    velHistory.pop_front();
  }
  velocity = velHistorySum / velHistory.size();
  */
  velocity = newVelocity;

  float newAcceleration = constrain((newVelocity - lastVelocity) * LOOP_FREQ, -5.0F, 5.0F); // m/s^2
  //Serial.printf("L: %.3f R: %.3f NA %.3f, OA: %.3f ", acc_LEFT, acc_RIGHT, (acc_LEFT + acc_RIGHT)/2.0F, newAcceleration);

  accHistory.push_back(newAcceleration);
  accHistorySum += newAcceleration;
  if (accHistory.size() > ACC_HISTORY_SIZE)
  {
    accHistorySum -= accHistory.front();
    accHistory.pop_front();
  }
  acceleration = accHistorySum / accHistory.size();

  lastX = currentX;
  lastY = currentY;
  lastVelocity = newVelocity;
}
void setDutyRaw(int channel, uint32_t value)
{
  if (channel < 0 || channel > 3)
    return;
  // currentDuty[channel] = value;
  ledcWrite(channel, value);
}
void processSerial()
{
  // from meta
  commandType command = STOP; // idk a command i receiveor sth - can be just a single byte
  switch (command)
  {
  case STOP:
    break;
  case SET_HOME:
    break;
  case RETURN_HOME:
    // some code
    break;
  case GO_TO_XYmode:
    // some code
    break;
  case DO_MOTION_TAN:
    // some code
    break;
  case DO_MOTION_NORM:
    // some code
    break;
  case CHARGING:
    // some code
    // either enabled or disabled - to disable H bridge and stop motors
    break;
  }
}
void setMotorDuty(Motor motor, MotorState *state, int duty)
{
  duty = DUTY_limiter(duty, motor);
  duty = duty * (motor == LEFT_MOTOR ? PWM_LEFT_DIRECTION : PWM_RIGHT_DIRECTION);
  // Serial.printf("duty: %d\n", duty);
  switch (*state)
  {
  case FORWARD: // LHLH
    if (duty > 0)
    {
      digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_1 : MOTOR_RIGHT_PIN_1, LOW);    // enable pin 1
      digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_2 : MOTOR_RIGHT_PIN_2, HIGH);   // disable pin 2
      setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_1 : PWM_CHANNEL_RIGHT_1, 0);    // disable pin 3
      setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_2 : PWM_CHANNEL_RIGHT_2, duty); // pwm on pin 4
    }
    else
    { // duty == 0 or less
      *state = MIDFORWARD;
    }
    break;
  case MIDFORWARD: // HHLH

    digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_1 : MOTOR_RIGHT_PIN_1, HIGH);           // disable pin 1
    digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_2 : MOTOR_RIGHT_PIN_2, HIGH);           // disable pin 2
    setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_1 : PWM_CHANNEL_RIGHT_1, 0);            // disable pin 3
    setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_2 : PWM_CHANNEL_RIGHT_2, PWM_MAX_DUTY); // enable pin 4
    if (duty > 0)
    {
      *state = FORWARD;
    }
    else
    {
      *state = STOPPED;
    }
    break;
  case STOPPED:                                                                               // HHHH
    digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_1 : MOTOR_RIGHT_PIN_1, HIGH);           // disable pin 1
    digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_2 : MOTOR_RIGHT_PIN_2, HIGH);           // disable pin 2
    setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_1 : PWM_CHANNEL_RIGHT_1, PWM_MAX_DUTY); // enable pin 3
    setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_2 : PWM_CHANNEL_RIGHT_2, PWM_MAX_DUTY); // enable pin 4
    if (duty > 0)
    {
      *state = MIDFORWARD;
    }
    else if (duty < 0)
    {
      *state = MIDBACKWARD;
    }
    break;
  case MIDBACKWARD: // HHHL

    digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_1 : MOTOR_RIGHT_PIN_1, HIGH);           // disable pin 1
    digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_2 : MOTOR_RIGHT_PIN_2, HIGH);           // disable pin 2
    setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_1 : PWM_CHANNEL_RIGHT_1, PWM_MAX_DUTY); // enable pin 3
    setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_2 : PWM_CHANNEL_RIGHT_2, 0);            // disable pin 4
    if (duty < 0)
    {
      *state = BACKWARD;
    }
    else
    {
      *state = STOPPED;
    }
    break;
  case BACKWARD: // HLHL
    if (duty < 0)
    {
      digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_1 : MOTOR_RIGHT_PIN_1, HIGH);    // disable pin 1
      digitalWrite(motor == LEFT_MOTOR ? MOTOR_LEFT_PIN_2 : MOTOR_RIGHT_PIN_2, LOW);     // enable pin 2
      setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_1 : PWM_CHANNEL_RIGHT_1, -duty); // pwm on pin 3
      setDutyRaw(motor == LEFT_MOTOR ? PWM_CHANNEL_LEFT_2 : PWM_CHANNEL_RIGHT_2, 0);     // disable pin 4
    }
    else
    {
      *state = MIDBACKWARD;
    }
    break;
  }
}
void PIDtoPWM(int tanPID, int rotPID)
{
  int leftPWMdutyraw = constrain(tanPID - rotPID, -PWM_MAX_DUTY, PWM_MAX_DUTY);
  int rightPWMdutyraw = constrain(tanPID + rotPID, -PWM_MAX_DUTY, PWM_MAX_DUTY);
  setMotorDuty(LEFT_MOTOR, &LeftMotorState, leftPWMdutyraw);
  setMotorDuty(RIGHT_MOTOR, &RightMotorState, rightPWMdutyraw);
  Serial.printf("TanI %0.2f, TanPID: %d, RotPID: %d, LeftDuty: %d, RightDuty: %d ",ACCerrorI, tanPID, rotPID, leftPWMdutyraw, rightPWMdutyraw);
}
void PIDcontrolLoop(float distanceToTarget, float targetAngle)
{
  if (distanceToTarget <= 0.01F)
  {
    distanceToTarget = 0.01F; // to avoid division by zero
  }
  switch (currentStageMovement)
  {
  case IDLE:
  {
    PIDtoPWM(0, 0);
    ACCerrorI = 0;
    lastACCerror = 0;
    ROTerrorI = 0;
    lastROTerror = 0;
    // if (distanceToTarget > 0.1f)
    //{
    //   currentStageMovement = ACCELERATING;
    // }
  }
  break;
  case ROTATING:
  {
    float errorRot = 0.0F;
    float errorRot0 = targetAngle - currentAngle;
    float errorRot1 = targetAngle - currentAngle - 2 * 3.14159265F;
    float errorRot2 = targetAngle - currentAngle + 2 * 3.14159265F;
    if (abs(errorRot0) < abs(errorRot1))
    {
      if (abs(errorRot0) < abs(errorRot2))
      {
        errorRot = errorRot0;
      }
      else
      {
        errorRot = errorRot2;
      }
    }
    else
    {
      if (abs(errorRot1) < abs(errorRot2))
      {
        errorRot = errorRot1;
      }
      else
      {
        errorRot = errorRot2;
      }
    }
    if (abs(errorRot) > 0.4F)
    {
      PIDtoPWM(0, PID_ROT_control(targetAngle));
    }
    else
    {
      currentStageMovement = ACCELERATING;
      ACCerrorI = 0;
    }
  }
  break;
  case ACCELERATING:
  {
    if (velocity > 0.1F && startAcceleration == 0){
      startAcceleration = millis();
      startAccelerationPoint.X = currentX;
      startAccelerationPoint.Y = currentY;
    }
    PIDtoPWM(PID_ACC_control(TARGET_ACCELERATION), PID_ROT_control(targetAngle));
    if (velocity >= TARGET_VELOCITY)
    {
      currentStageMovement = CRUISING;
      stopAcceleration = millis();
      stopAccelerationPoint.X = currentX;
      stopAccelerationPoint.Y = currentY;
    }
    else if (-velocity * velocity / (2 * distanceToTarget) <= TARGET_DECELERATION + DECELERATION_ALLOWED_DEVIATION)
    { // if we are that far away rom target that if we start deceleration, it will be at TARGET_DECELERATION and we reach the target position perfectly
      currentStageMovement = DECELERATING;
      stopAcceleration = millis();
      stopAccelerationPoint.X = currentX;
      stopAccelerationPoint.Y = currentY;
      maxVelocity = velocity;
      minVelocity = -1.0F;
      startDeceleration = millis();
      startDecelerationPoint.X = currentX;
      startDecelerationPoint.Y = currentY;
    }
  }
  break;
  case CRUISING:
  {
    if (minVelocity > velocity)
    {
      minVelocity = velocity;
    }
    if (maxVelocity < velocity)
    {
      maxVelocity = velocity;
    }
    PIDtoPWM(PID_ACC_control(0), PID_ROT_control(targetAngle));
    if (-velocity * velocity / (2 * distanceToTarget) <= TARGET_DECELERATION + DECELERATION_ALLOWED_DEVIATION)
    { // if we are that far away rom target that if we start deceleration, it will be at TARGET_DECELERATION and we reach the target position perfectly
      currentStageMovement = DECELERATING;
      startDeceleration = millis();
      startDecelerationPoint.X = currentX;
      startDecelerationPoint.Y = currentY;
    }
  }
  break;
  case DECELERATING:
  {
    PIDtoPWM(PID_ACC_control(constrain(-velocity * velocity / (2 * distanceToTarget), TARGET_DECELERATION - DECELERATION_ALLOWED_DEVIATION, TARGET_DECELERATION + DECELERATION_ALLOWED_DEVIATION)), PID_ROT_control(targetAngle));
    // PIDtoPWM(PID_ACC_control(TARGET_DECELERATION), PID_ROT_control(targetAngle));
    // PIDtoPWM(PID_ACC_control(-velocity * velocity / (2 * distanceToTarget)), PID_ROT_control(targetAngle));
    if (velocity <= 0.1f)
    { // near zero velocity
      currentStageMovement = IDLE;
      switchToNextPoint = true;
      stopDeceleration = millis();
      stopDecelerationPoint.X = currentX;
      stopDecelerationPoint.Y = currentY;
    }
  }
  break;
  }
}
int PID_ACC_control(float targetAcceleration)
{
  float errorAcc = targetAcceleration - acceleration;
  float ACCerrorD = (errorAcc - lastACCerror) * LOOP_FREQ;
  lastACCerror = errorAcc;
  ACCerrorI += errorAcc / LOOP_FREQ;
  ACCerrorI = constrain(ACCerrorI, -PWM_MAX_DUTY / (ACC_PID_I * 2), PWM_MAX_DUTY / (ACC_PID_I * 2)); // constrain integral
  return (int)(errorAcc * ACC_PID_P + ACCerrorI * ACC_PID_I + ACCerrorD * ACC_PID_D);
}
int PID_ROT_control(float targetAngle)
{
  float errorRot = 0.0F;
  float errorRot0 = targetAngle - currentAngle;
  float errorRot1 = targetAngle - currentAngle - 2 * 3.14159265F;
  float errorRot2 = targetAngle - currentAngle + 2 * 3.14159265F;
  if (abs(errorRot0) < abs(errorRot1))
  {
    if (abs(errorRot0) < abs(errorRot2))
    {
      errorRot = errorRot0;
    }
    else
    {
      errorRot = errorRot2;
    }
  }
  else
  {
    if (abs(errorRot1) < abs(errorRot2))
    {
      errorRot = errorRot1;
    }
    else
    {
      errorRot = errorRot2;
    }
  }
  float ROTerrorD = (errorRot - lastROTerror) * LOOP_FREQ;
  lastROTerror = errorRot;
  ROTerrorI = constrain(ROTerrorI + errorRot / LOOP_FREQ, -PWM_MAX_DUTY / (ROT_PID_I), PWM_MAX_DUTY / (ROT_PID_I)); // constrain integral
  return (int)(errorRot * ROT_PID_P + ROTerrorI * ROT_PID_I + ROTerrorD * ROT_PID_D);
}
int DUTY_limiter(int DUTY, Motor motor)
{
  // At 100% duty cycle theese equations hold RPM=130-63,222τ
  // I_max=0.16+2,2434*τ
  //(130-RPM*sign(PWM))/63,222*PWM/100=τ -- motor model that also should hold at negative rpm
  // positive RPM, positve pwm     torque = ((130-RPM)/0.62)*(PWM/100);
  // negative RPM, positive pwm    torque = ((130-RPM)/0.62)*(PWM/100);
  // negative RPM, negative pwm    torque = ((130+RPM)/0.62)*(PWM/100);
  // positive RPM, negative pwm    torque = ((130+RPM)/0.62)*(PWM/100);
  // max pwm is when torque at given rpm and pwm would be max torque 0.8Nm thus constraining PWM such that the would be generated torque would be never over that value
  // ABS( (130-RPM*sign(PWM))/63,222*PWM/100 ) has to be less than τ
  // ABS( (130-RPM*sign(PWM))/63,222*PWM/100 ) < τ
  //-τ < (130-RPM*sign(PWM))/63,222*PWM/100 < τ
  // constrain  -τ =  (130-RPM*sign(PWM))/63,222*PWM/100   and    (130-RPM*sign(PWM))/63,222*PWM/100 = τ
  // constrain  -τ =  (130-RPM*sign(PWM))/63,222*DUTY/1023   and    (130-RPM*sign(PWM))/63,222*DUTY/1023 = τ
  // constrain  -τ =  (130-RPM*sign(PWM))/63,222*DUTY/1023   and    (130-RPM*sign(PWM))/63,222*DUTY/1023 = τ
  // constrain  -τ *1023 =  (130-RPM*sign(PWM))/63,222*DUTY  and    (130-RPM*sign(PWM))/63,222*DUTY = τ * 1023
  // constrain  -τ *1023 / ( (130-RPM*sign(PWM))/63,222 ) =  DUTY  and    DUTY = τ * 1023 / ( (130-RPM*sign(PWM))/63,222 )
  int sign = 0;
  if (DUTY > 0)
  {
    sign = 1;
  }
  else if (DUTY < 0)
  {
    sign = -1;
  }
  int RPM = 0;
  switch (motor)
  {
  case LEFT_MOTOR:
    RPM = LEFT_RPM;
    break;
  case RIGHT_MOTOR:
    RPM = RIGHT_RPM;
    break;
  }
  RPM = constrain(RPM, -120, 120); // to avoid division by zero or negative division by zero
  return constrain(DUTY, -MAX_ALLOWED_TORQUE * PWM_MAX_DUTY / ((130 - RPM * sign) / 63.222F), MAX_ALLOWED_TORQUE * PWM_MAX_DUTY / ((130 - RPM * sign) / 63.222F));
}
void executePath(void) //
{
  if (movementEnabled && !navPoints.empty())
  {
    if (currentStageMovement == IDLE)
    {
      currentStageMovement = ROTATING;
      currentTargetPoint = navPoints.front();
    }
    distanceToTarget = sqrtf((currentTargetPoint.X - currentX) * (currentTargetPoint.X - currentX) + (currentTargetPoint.Y - currentY) * (currentTargetPoint.Y - currentY));
    while (distanceToTarget < 0.05F)
    {
      navPoints.pop_front();
      if (!navPoints.empty())
      {
        currentTargetPoint = navPoints.front();
        distanceToTarget = sqrtf((currentTargetPoint.X - currentX) * (currentTargetPoint.X - currentX) + (currentTargetPoint.Y - currentY) * (currentTargetPoint.Y - currentY));
      }
      else
      {
        currentStageMovement = IDLE;
        currentTargetPoint.X = currentX;
        currentTargetPoint.Y = currentY;
        currentTargetPoint.distanceTillFullStop = 0.0F;
      }
    }
  }
  else
  {
    currentStageMovement = IDLE;
    currentTargetPoint.X = currentX;
    currentTargetPoint.Y = currentY;
    currentTargetPoint.distanceTillFullStop = 0.0F;
  }
  float targetAngle = atan2f(currentTargetPoint.Y - currentY, currentTargetPoint.X - currentX);
  PIDcontrolLoop(distanceToTarget + currentTargetPoint.distanceTillFullStop, targetAngle);
  // PIDcontrolLoop(distanceToTarget + currentTargetPoint.distanceTillFullStop, targetAngle, currentTargetPoint.direction);  //implement directions so robot can go backwards as well, to be able to do tests
}
void planPath(movementType type, float targetX, float targetY)
{
  navPoint centerPoint;
  centerPoint.X = targetX;
  centerPoint.Y = targetY;
  // all test will by default be with the right shoulder pointing at the detector
  // all test will by default be with the robot going towards the detector
  // get vector from centerPoint to Stand
  float vecX = standX - centerPoint.X;
  float vecY = standY - centerPoint.Y;
  float vecLength = sqrtf(vecX * vecX + vecY * vecY);
  vecX /= vecLength;
  vecY /= vecLength;
  switch (type)
  {
  case MOTION_TANGENTIAL:
    break;
  case MOTION_RADIAL:
    navPoint genPath[3];
    // the robot should stop 1.5m before the test stand (worst case in case the test stand does not respond), so it does not ram into the stand
    genPath[2].X = -vecX * 1.5F;
    genPath[2].Y = -vecY * 1.5F;
    genPath[2].distanceTillFullStop = 0.0F; // the robot should wait for 1 second on full stops
    genPath[1] = centerPoint;
    updateDistanceToTarget(&genPath[1], &genPath[2]);
    genPath[0].X = genPath[1].X - vecX * 0.5F; // 0.5 meter before point 1
    genPath[0].Y = genPath[1].Y - vecY * 0.5F; // 0.5 meter before point 1
    updateDistanceToTarget(&genPath[0], &genPath[1]);
    // the robot is expected to have the path interrupted by the STOP command before it reaches point 2
    break;
  }
}
void updateDistanceToTarget(navPoint *first, navPoint *second)
{
  first->distanceTillFullStop = sqrtf((second->X - first->X) * (second->X - first->X) + (second->Y - first->Y) * (second->Y - first->Y)) + second->distanceTillFullStop;
}
bool evaluateRequirements(void)
{
  float distAcc = sqrtf((startAccelerationPoint.X - stopAccelerationPoint.X) * (startAccelerationPoint.X - stopAccelerationPoint.X) + (startAccelerationPoint.Y - stopAccelerationPoint.Y) * (startAccelerationPoint.Y - stopAccelerationPoint.Y));
  float timeAcc = (stopAcceleration - startAcceleration) / 1000.0F;
  float avgAcc =  2 * distAcc / (timeAcc * timeAcc); // s=0.5*a*t^2 -> a=2s/t^2
  float distDec = sqrtf((startDecelerationPoint.X - stopDecelerationPoint.X) * (startDecelerationPoint.X - stopDecelerationPoint.X) + (startDecelerationPoint.Y - stopDecelerationPoint.Y) * (startDecelerationPoint.Y - stopDecelerationPoint.Y));
  float timeDec = (stopDeceleration - startDeceleration) / 1000.0F;
  float avgDec = -2 * distDec / (timeDec * timeDec); // s=0.5*a*t^2 -> a=2s/t^2
  Serial.printf("AvgAcc: %0.2f, AvgDec: %0.2f, MinV: %0.2f, MaxV: %0.2f\n", avgAcc, avgDec, minVelocity, maxVelocity);
  if ((avgAcc > TARGET_ACCELERATION - ACCELERATION_ALLOWED_DEVIATION) && (avgAcc < TARGET_ACCELERATION + ACCELERATION_ALLOWED_DEVIATION) && (avgDec > TARGET_DECELERATION - DECELERATION_ALLOWED_DEVIATION) && (avgDec < TARGET_DECELERATION + DECELERATION_ALLOWED_DEVIATION) && (((minVelocity > TARGET_VELOCITY - VELOCITY_ALLOWED_DEVIATION) && (maxVelocity < TARGET_VELOCITY + VELOCITY_ALLOWED_DEVIATION))||minVelocity==-1.0F))
  {
    return true;
  }
  else
  {
    return false;
  }
} 
void test(TestType type)
{
  switch (type)
  {
  case TEST_SETDUYTRAW: // DO NOT RUN WITH THE H BRIDGE WIRED UP
  {
    setDutyRaw(PWM_CHANNEL_LEFT_1, PWM_MAX_DUTY / 5);
    setDutyRaw(PWM_CHANNEL_LEFT_2, PWM_MAX_DUTY * 2 / 5);
    setDutyRaw(PWM_CHANNEL_RIGHT_1, PWM_MAX_DUTY * 3 / 5);
    setDutyRaw(PWM_CHANNEL_RIGHT_2, PWM_MAX_DUTY * 4 / 5);
  }
  break;
  case TEST_SETMOTORDUTY:
  {
    if (millis() % 10000 < 5000)
    {
      // setMotorDuty(LEFT_MOTOR, &LeftMotorState, PWM_MAX_DUTY / 5);
      setMotorDuty(RIGHT_MOTOR, &RightMotorState, PWM_MAX_DUTY / 3);
      setMotorDuty(LEFT_MOTOR, &LeftMotorState, PWM_MAX_DUTY / 3);
    }
    else
    {
      // setMotorDuty(LEFT_MOTOR, &LeftMotorState, -PWM_MAX_DUTY / 5);
      setMotorDuty(RIGHT_MOTOR, &RightMotorState, PWM_MAX_DUTY);
      setMotorDuty(LEFT_MOTOR, &LeftMotorState, PWM_MAX_DUTY);
    }
    Serial.printf("RightRPM: %d, LeftRPM %d, RightPT: %u , LeftPT: %u, Velo: %02f, Accel: %02f \n", RIGHT_RPM, LEFT_RPM, RIGHT_time_between_pulses, LEFT_time_between_pulses, velocity, acceleration);
  }
  break;
  case TEST_PID:
  {
    PIDtoPWM(PID_ACC_control(0.4F), PID_ROT_control(0.0F));
  }
  break;
  case TEST_PIDCONTROLLOOP:
  {
    if (doOnce)
    {
      doOnce = false;
      currentStageMovement = ACCELERATING;
    }
    navPoint testPoint;
    testPoint.X = 2.0F;
    testPoint.Y = 0.0F;
    PIDcontrolLoop(abs(testPoint.X - currentX), 0.0F);
    Serial.printf("V: %0.2f, A: %0.2f \n", velocity, acceleration);
    if (currentStageMovement == IDLE)
    {
      Serial.printf("X: %0.3f, Y: %0.3f reached\n", currentX, currentY);
    }
  }
  break;
  case TEST_STEERING180DEG:
  {
    navPoint anglePoint;
    anglePoint.X = -10.0F;
    anglePoint.Y = 0.0F;
    float targetAngle = atan2f(anglePoint.Y - currentY, anglePoint.X - currentX);
    float targetDistance = sqrtf((anglePoint.X - currentX) * (anglePoint.X - currentX) + (anglePoint.Y - currentY) * (anglePoint.Y - currentY));
    if (abs(targetAngle) > 0.01F)
    {
      PIDtoPWM(0, PID_ROT_control(targetAngle));
    }
    else
    {
      PIDcontrolLoop(0, 0);
    }

    Serial.printf("tAngle: %0.2f, Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", targetAngle, currentAngle, currentX, currentY, velocity, acceleration);
  }
  break;
  case TEST_STEERING90DEG:
  {
    navPoint anglePoint;
    anglePoint.X = 0.0F;
    anglePoint.Y = -10.0F;
    float targetAngle = atan2f(anglePoint.Y - currentY, anglePoint.X - currentX);
    float targetDistance = sqrtf((anglePoint.X - currentX) * (anglePoint.X - currentX) + (anglePoint.Y - currentY) * (anglePoint.Y - currentY));
    if (abs(targetAngle) > 0.01F)
    {
      PIDtoPWM(0, PID_ROT_control(targetAngle));
    }
    else
    {
      PIDcontrolLoop(0, 0);
    }

    Serial.printf("tAngle: %0.2f, Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", targetAngle, currentAngle, currentX, currentY, velocity, acceleration);
  }
  break;
  case TEST_2METER_DISTANCE:
  {
    if (doOnce)
    {
      doOnce = false;
      currentStageMovement = ACCELERATING;
      ACCerrorI = 1.0F;
    }
    float sDistance = 2.0F - currentX;
    if (currentStageMovement == IDLE && (!finished))
    {
      if(evaluateRequirements())
      {
        Serial.printf("REQ PASSED\n");
      }
      else
      {
        Serial.printf("REQ FAILED\n");
      }
      finished = true;
    }
    if (!finished)
    {
      PIDcontrolLoop(sDistance, 0.0F);
      Serial.printf("Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", currentAngle, currentX, currentY, velocity, acceleration);
    }
  }
  break;
  case TEST_2METER_STRAIGHT:
  {
    if (doOnce)
    {
      doOnce = false;
      currentStageMovement = ROTATING;
    }
    navPoint straightPoint;
    straightPoint.X = 2.0F;
    straightPoint.Y = 0.0F;
    float sAngle = atan2f(straightPoint.Y - currentY, straightPoint.X - currentX);
    float sDistance = sqrtf((straightPoint.X - currentX) * (straightPoint.X - currentX) + (straightPoint.Y - currentY) * (straightPoint.Y - currentY));
    PIDcontrolLoop(sDistance, sAngle);
    Serial.printf("tAngle: %0.2f, Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", sAngle, currentAngle, currentX, currentY, velocity, acceleration);
  }
  break;
  case TEST_2METER_45DEG:
  {
    if (doOnce)
    {
      doOnce = false;
      currentStageMovement = ROTATING;
    }
    navPoint straightPoint;
    straightPoint.X = 2.0F;
    straightPoint.Y = 2.0F;
    float sAngle = atan2f(straightPoint.Y - currentY, straightPoint.X - currentX);
    float sDistance = sqrtf((straightPoint.X - currentX) * (straightPoint.X - currentX) + (straightPoint.Y - currentY) * (straightPoint.Y - currentY));
    PIDcontrolLoop(sDistance, sAngle);
    Serial.printf("tAngle: %0.2f, Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", sAngle, currentAngle, currentX, currentY, velocity, acceleration);
  }
  break;
  case TEST_2METER_90DEG:
  {
    if (doOnce)
    {
      doOnce = false;
      currentStageMovement = ROTATING;
    }
    navPoint straightPoint;
    straightPoint.X = 0.0F;
    straightPoint.Y = 2.0F;
    float sAngle = atan2f(straightPoint.Y - currentY, straightPoint.X - currentX);
    float sDistance = sqrtf((straightPoint.X - currentX) * (straightPoint.X - currentX) + (straightPoint.Y - currentY) * (straightPoint.Y - currentY));
    PIDcontrolLoop(sDistance, sAngle);
    Serial.printf("tAngle: %0.2f, Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", sAngle, currentAngle, currentX, currentY, velocity, acceleration);
  }
  break;
  case TEST_2METER_180DEG:
  {
    if (doOnce)
    {
      doOnce = false;
      currentStageMovement = ROTATING;
    }
    navPoint straightPoint;
    straightPoint.X = -2.0F;
    straightPoint.Y = 0.0F;
    float sAngle = atan2f(straightPoint.Y - currentY, straightPoint.X - currentX);
    float sDistance = sqrtf((straightPoint.X - currentX) * (straightPoint.X - currentX) + (straightPoint.Y - currentY) * (straightPoint.Y - currentY));
    PIDcontrolLoop(sDistance, sAngle);
    Serial.printf("tAngle: %0.2f, Angle: %02f, X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", sAngle, currentAngle, currentX, currentY, velocity, acceleration);
  }
  break;
  case TEST_NAV_SQUARE:
  {
    if (doOnce)
    {
      doOnce = false;
      navPoint p1;
      p1.X = 1.0F;
      p1.Y = 1.0F;
      p1.distanceTillFullStop = 0.0F;
      navPoint p2;
      p2.X = 1.0F;
      p2.Y = -1.0F;
      p2.distanceTillFullStop = 0.0F;
      navPoint p3;
      p3.X = -1.0F;
      p3.Y = -1.0F;
      p3.distanceTillFullStop = 0.0F;
      navPoint p4;
      p4.X = -1.0F;
      p4.Y = 1.0F;
      p4.distanceTillFullStop = 0.0F;
      navPoint p5;
      p5.X = 1.0F;
      p5.Y = 1.0F;
      p5.distanceTillFullStop = 0.0F;
      navPoints.push_back(p1);
      navPoints.push_back(p2);
      navPoints.push_back(p3);
      navPoints.push_back(p4);
      navPoints.push_back(p5);
      movementEnabled = true;
    }
    executePath();
    Serial.printf("St: %d X:%0.2f, Y: %0.2f, V: %0.2f, A: %0.2f \n", currentStageMovement, currentX, currentY, velocity, acceleration);
  }
  break;
  }
}