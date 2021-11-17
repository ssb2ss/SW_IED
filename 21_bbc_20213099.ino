#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN 1200 // servo full clockwise position (high position)
#define _DUTY_NEU 1600 // servo neutral position
#define _DUTY_MAX 2100 // servo full counterclockwise position (low position)

#define _SERVO_SPEED 500 // servo speed limit (unit: degree/second)
#define _INTERVAL_DIST 20 // IR interval (unit: ms)
#define _INTERVAL_SERVO 20 // servo interval (unit: ms)
#define _INTERVAL_SERIAL 100 // serial interval (unit: ms)

#define _DIST_ALPHA 0.2

int a, b; // unit: mm
int duty_chg_per_interval; // maximum duty difference per interval
float dist_raw, dist_cali, dist_ema;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; // unit: ms
bool event_dist, event_servo, event_serial;

int duty_target, duty_curr;

Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
// initialize serial port
  Serial.begin(57600);

  a = 85;
  b = 364;

  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * _INTERVAL_SERVO / 1000;

  dist_raw = dist_cali = dist_ema = 250.0;

  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  duty_target = duty_curr = _DUTY_NEU;

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NEU);
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }

  if(event_dist) {
    event_dist = false;
    
    dist_raw = ir_distance();
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
    dist_ema = _DIST_ALPHA * dist_cali + (1 - _DIST_ALPHA) * dist_ema;

    if(dist_ema > 270) {
      duty_target = _DUTY_MIN;
    }
    else {
      duty_target = _DUTY_MAX;
    }
  }

  if(event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else if(duty_target < duty_curr){
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if(event_serial) {
    event_serial = false;
    
    Serial.print("min:0,max:500,dist:");
    Serial.print(dist_raw);
    Serial.print(",dist_cali:");
    Serial.print(dist_cali);
    Serial.print(",dist_ema:");
    Serial.println(dist_ema);
  }
}
