#ifndef __WINDING_MECHANISM_H__
#define __WINDING_MECHANISM_H__

#include "Mechanism.h"

// state machine, 0 is winding
// 1 is hold
// 2 is playback
#define WIND_BACK 0
#define WIND_HOLD 1
#define WIND_FRONT 2

// min and max values
#define MIN_FRONT_SPEED 40
#define MAX_FRONT_SPEED 450
#define MIN_REAR_SPEED -40
#define MAX_REAR_SPEED -450

int motor_speed = 0;
int target_motor_speed = 0;
int next_motor_speed = 0;
int motor_time = 0;
int next_motor_time = 0;

class WindingMechanism {
    public:
      WindingMechanism(uint8_t ap, uint8_t dp, uint8_t dd, double f, uint8_t ot);
      void update();
      void queueStrike(double vel, uint32_t len);
      void strike();
      double getFreq() {return freq;};
      void setPrint(bool p){PRINT_MECHANISM_DEBUG = p;};

    private:
      uint8_t dampener_delay = 10;// how long to let the dampener be released for before the striking solenoid acts:
      double velocity = 1.0;      // this is really the length of time
      uint32_t note_length = 30;  // how long to leave damper disenguaged after act strikes
      uint8_t state = BELL_INACTIVE; // is it
      uint8_t on_time = 30;
      double freq; // what is the frequency of this mechanism

      uint8_t act_pin;

      uint8_t damp_pin;

      elapsedMillis last_action;
      bool PRINT_MECHANISM_DEBUG = false;
};

BellMechanism::BellMechanism(uint8_t ap, uint8_t dp, uint8_t dd, double f, uint8_t ot) {
  act_pin = ap;
  damp_pin = dp;
  dampener_delay = dd;
  freq = f;
  on_time = ot;
}

void BellMechanism::queueStrike(double vel, uint32_t len) {
    velocity = vel;
    note_length = len;
    dprint(PRINT_MECHANISM_DEBUG, "new strike queued for bell mechanism - vel:");
    dprint(PRINT_MECHANISM_DEBUG, vel);
    dprint(PRINT_MECHANISM_DEBUG, " len: ");
    dprintln(PRINT_MECHANISM_DEBUG, note_length);
}

void BellMechanism::strike() {
    state = BELL_REMOVE_DAMPER;
}

void BellMechanism::update() {
  switch (state) {
    case BELL_INACTIVE:
        // if it has been long enough since the last bell strike and the bell it not currently active
        break;
    case BELL_REMOVE_DAMPER:
        dprintln(PRINT_MECHANISM_DEBUG, "removing bell dampener");
        digitalWrite(damp_pin, HIGH); // turn on the dampener
        last_action = 0; // reset the last bell activity timer
        state = BELL_DAMPENER_UP; // change bell state to reflect status of dampener
        break;
    case BELL_DAMPENER_UP:
      // if the dampener has been up for long enough then strike the solenoid
      if (last_action > dampener_delay) {
        dprintln(PRINT_MECHANISM_DEBUG, "striking the bell");
        if (dampener_delay != 0) {
            digitalWrite(act_pin, HIGH);
        }
        last_action = 0;       // reset the last bell activity timer
        state = BELL_STRIKING; // change bell state to reflect status of dampener
        break;
      }
      break;
    case BELL_STRIKING:
      // if the bell has been struck for long enough deactivate the striking solenoid
      if (last_action > velocity * on_time) {
        dprintln(PRINT_MECHANISM_DEBUG, "stopping bell strike");
        digitalWrite(act_pin, LOW);
        last_action = 0;     // reset the last bell activity timer
        state = BELL_STRUCK; // change bell state to reflect status of dampener
        break;
      }
      break;
    case BELL_STRUCK:
      // if the bell has rung for long enough then allow the dampener to re-engage
      if (last_action > note_length) {
        dprintln(PRINT_MECHANISM_DEBUG, "reapplying the dampener");
        digitalWrite(damp_pin, LOW);
        last_action = 0;       // reset the last bell activity timer
        state = BELL_INACTIVE; // change bell state to inactive
        break;
      }
      break;
  }
}


void shake(int on_speed, int on_time, int rev_speed, int rev_time)
{
  if (rev_speed > 0)
  {
    rev_speed *= -1;
  }
  target_motor_speed = on_speed;
  next_motor_speed = rev_speed;
  motor_time = on_time;
  next_motor_time = rev_time;

  Serial.print("shaking (onspeed, ontime, revspeed, revtime): ");
  Serial.print(on_speed);
  Serial.print("\t");
  Serial.print(on_time);
  Serial.print("\t");
  Serial.print(rev_speed);
  Serial.print("\t");
  Serial.println(rev_time);

  neos[0].colorWipe(0, 40, 125, 1.0);

  // rev the motor up
  rampSpinnerMotor(0, on_speed, on_time * 0.05);

  // Serial.print("starting / ending pos: ");
  // enc_pos = enc.read();
  // Serial.print(enc_pos);

  // let motor spin, with new color
  neos[0].colorWipe(200, 200, 255, 1.0);
  delay(on_time * 0.95);

  // ramp up motor to it's reverse speed
  neos[0].colorWipe(125, 40, 0, 1.0);
  rampSpinnerMotor(on_speed, rev_speed, rev_time * 0.1);

  neos[0].colorWipe(50, 40, 0, 1.0);
  // let things rotate for a bit
  delay(rev_time * 0.75);
  // rev down to off
  neos[0].colorWipe(25, 20, 0, 1.0);
  rampSpinnerMotor(rev_speed, 0, rev_time * 0.15);

  // enc_pos = enc.read();
  // Serial.print(enc_pos);
  neos[0].colorWipe(0, 0, 0, 1.0);
}

void rampSpinnerMotor(int16_t start, int16_t target, int ramp_total_time)
{
  Serial.print("Ramping Motor (start, target, time) - ");
  Serial.print(start);
  Serial.print("\t");
  Serial.print(target);
  Serial.print("\t");
  Serial.println(ramp_total_time);

  int difference = target - start;
  float step_delay = abs(difference / ramp_total_time) * 1.0;

  Serial.print(" dif: ");
  Serial.print(difference);
  Serial.print(" stepd: ");
  Serial.print(step_delay);
  motors.enableDrivers(0);

  if (difference > 0)
  {
    for (int16_t i = start; i <= target; i++)
    {
      motors.setM1Speed(i);
      // Serial.println(i);
      delayMicroseconds(step_delay);
    }
  }
  else
  {
    for (int16_t i = start; i > target; i--)
    {
      motors.setM1Speed(i);
      delayMicroseconds(step_delay);
      // Serial.println(i);
    }
  }

  if (target == 0)
  {
    motors.setM1Speed(0);
    motors.disableDrivers(0);
  }
  Serial.println("Disabled Drivers");
}

void sustainedShake(int on_speed, int ramp_time, int on_time, int deviation) {
  target_motor_speed = on_speed;
  motor_time = on_time;

  Serial.print("sustained shake (on_speed, ramp_time, on_time): ");
  Serial.print(on_speed);
  Serial.print("\t");
  Serial.print(ramp_time);
  Serial.print("\t");
  Serial.println(on_time);

  // TODO - replace with an update to the visual feedback system
  neos[0].colorWipe(0, 40, 125, 1.0);

  // rev the motor up
  rampSpinnerMotor(0, on_speed, ramp_time * 0.05);

  // Serial.print("starting / ending pos: ");
  // enc_pos = enc.read();
  // Serial.print(enc_pos);

  // let motor spin, with new color
  neos[0].colorWipe(200, 200, 255, 1.0);
  elapsedMillis t;
  while (t < on_time){
    target_motor_speed += map(random(0, deviation), 0, deviation, (-0.5 * deviation), (0.5*deviation));
    motors.setM1Speed(target_motor_speed);
    Serial.print("set motor speed to: ");
    Serial.println(target_motor_speed);
    delay(20 + random(100));
  }

  // ramp up motor to it's reverse speed
  neos[0].colorWipe(125, 40, 0, 1.0);
  rampSpinnerMotor(target_motor_speed, 0, ramp_time * 0.1);
  neos[0].colorWipe(0, 0, 0, 1.0);
  Serial.println("------------ finished with sustained spin ------------------");
}

#endif