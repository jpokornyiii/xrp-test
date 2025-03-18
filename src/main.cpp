#include <Arduino.h>
#include "pins.h"
#include <Servo.h>
#include <vector>

// Servo pulses
#define XRP_SERVO_MIN_PULSE_US 500
#define XRP_SERVO_MAX_PULSE_US 2500

uint32_t _lastUpdateTime = 0;
boolean direction = false;
double speed = 0.8;

#define NUM_OF_SERVOS 4

Servo Servos[NUM_OF_SERVOS];

int ServoPinMap[NUM_OF_SERVOS] = {
  SERVO_1,
  SERVO_2,
  SERVO_3,
  SERVO_4
};

bool _initServos() {
  bool success = true;
  for(int i=0;i < NUM_OF_SERVOS;i++) {
    if(ServoPinMap[i] != __XRP_PIN_UNDEF && (Servos[i].attach(ServoPinMap[i], XRP_SERVO_MIN_PULSE_US, XRP_SERVO_MAX_PULSE_US) == -1)) {
      Serial.printf("[ERR] Failed to attach servo %d", i+1);
      Serial.println("\n");
      success = false;
    }
  }

  return success;
}

void _setServoPwmValueInternal(int servoIdx, double value) {
  int val = ((value + 1.0) / 2.0) * 180;

  if(Servos[servoIdx].attached()) {
    Servos[servoIdx].write(val);
  }
}

void setup() {
  delay(5000);
  Serial.begin(115200);

  // Left
  pinMode(MOTOR_L_IN_1, OUTPUT);
  pinMode(MOTOR_L_IN_2, OUTPUT);

  // RIGHT
  pinMode(MOTOR_R_IN_1, OUTPUT);
  pinMode(MOTOR_R_IN_2, OUTPUT);

  // Motor 3
  pinMode(MOTOR_3_IN_1, OUTPUT);
  pinMode(MOTOR_3_IN_2, OUTPUT);

  // Motor 4
  pinMode(MOTOR_4_IN_1, OUTPUT);
  pinMode(MOTOR_4_IN_2, OUTPUT);

  _initServos();

  pinMode(BOARD_USER_BUTTON, INPUT_PULLUP);

  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println("Initialization Complete\n");
}


#ifdef PICO_RP2350

void _setMotorPwmValueInternal(int in2, int in1, double value) {
  boolean is_forward = (value >= 0.0);
  int speed = (abs(value) * 255);
  
  // Direction determines which pin should be the brake
  if(is_forward) {
    digitalWrite(in2,LOW);
    analogWrite(in1, speed);
  } else {
    digitalWrite(in1,LOW);
    analogWrite(in2, speed);    
  }
}

#else

void _setMotorPwmValueInternal(int en, int ph, double value) {
  
  PinStatus phValue = (value < 0.0) ? LOW : HIGH;
  int enValue = (abs(value) * 255);

  digitalWrite(ph, phValue);
  analogWrite(en, enValue);
}

#endif


void loop() {

  uint32_t currTime = millis();
  // Limit to 200 ms to see LED changes
  if (currTime - _lastUpdateTime < 500) {
    return;
  }

  if( direction ) {
    digitalWrite(LED_BUILTIN, HIGH);
    _setMotorPwmValueInternal(MOTOR_L_IN_2, MOTOR_L_IN_1, 0.8);
    _setMotorPwmValueInternal(MOTOR_R_IN_2, MOTOR_R_IN_1, 0.8);

    for(int i = 0; i < NUM_OF_SERVOS; i++) {
      // Set different servo positions per servo port
      if( ServoPinMap[i] != __XRP_PIN_UNDEF ) {
        _setServoPwmValueInternal(i, double(i+1)/NUM_OF_SERVOS);
      }
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    _setMotorPwmValueInternal(MOTOR_L_IN_2, MOTOR_L_IN_1, -0.8);
    _setMotorPwmValueInternal(MOTOR_R_IN_2, MOTOR_R_IN_1, -0.8);

    // Reset Motors to zero and servo to 90 degs
    for(int i = 0;i < NUM_OF_SERVOS; i++) {
      if (ServoPinMap[i] != __XRP_PIN_UNDEF ) { 
        _setServoPwmValueInternal(i, 0.0);
      }
    }
  }

  direction = !direction;
  _lastUpdateTime = currTime;
}
