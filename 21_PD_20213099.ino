#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9            // [20200000] LED핀 설정
#define PIN_SERVO 10         //[20191979] Servo 핀 설정
#define PIN_IR A0            // [20191979] 적외선 핀(아날로그핀)

// Framework setting
#define _DIST_TARGET 225    // [20213085] 정지하려는 위치 목표값
#define _DIST_MIN 100         //[20191979] 최소거리
#define _DIST_MAX 340         //[20191979] 최대거리

// Distance sensor
#define _DIST_ALPHA 0.25 //[20203118] DIST_ALPHA 값 설정

// Servo range
#define _DUTY_MIN 1100    // [20213083]서보 각도의 최솟값 설정
#define _DUTY_NEU 1600    // [20213090]서보 수평 각도 펄스 값
#define _DUTY_MAX 2100        // [20213081] 서보 각도의 최댓값

// Servo speed control
#define _SERVO_ANGLE 20 //[20203118] 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 1000 //[20203118] 서보 속도 설정

// Event periods
#define _INTERVAL_DIST 20   // [20213099] 거리 센서 주기 설정
#define _INTERVAL_SERVO 20  // [20213099] 서보 주기 설정
#define _INTERVAL_SERIAL 100  // [20213099] 시리얼 표시 주기 설정

// PID parameters
#define _KP 2.5        //[20191979] 비례제어 값
#define _KD 25        // 미분제어 값

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;  // [20213090] 서보 인스턴스 선언

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; //[20203118] 거리와 노이즈 필터 적용 후 거리를 저장하기 위한 변수

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; //[20203118] 마지막으로 측정한 거리, 마지막으로 측정한 서보 각도(각 이벤트별로 업데이트를 위해 측정하는 시간)
bool event_dist, event_servo, event_serial; //[20203118] 이벤트 별로 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; //[20213086]interval 당 최대 duty 변화량
int duty_target, duty_curr; //[20213086] 목표 duty와 현재 duty 값

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
//[20213086] error_curr: 현재 주기 오차값 / error_prev : 이전 주기 오차 값 / control : PID 제어량 / pterm, dterm, iterm : 비례,적분,미분 이득값

float x[7] = {85, 145, 195, 235, 285, 330, 390};
float y[7] = {100, 150, 200, 250, 300, 350, 400};
float nd = sizeof(x)/sizeof(float);


void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);

  // move servo to neutral position
  myservo.attach(PIN_SERVO);
  duty_target = duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);

  // initialize global variables
  duty_curr = _DUTY_MIN; //[20203118] duty_curr 값 초기화
  dist_raw = dist_ema = ir_distance_filtered(); // 초기화
  error_curr = error_prev = dist_target - dist_ema;  // 초기화
  dist_target = _DIST_TARGET; //[20203118] dist_target 값 초기화

  last_sampling_time_dist = last_sampling_time_servo = last_sampling_time_serial = 0;
  event_dist = event_servo = event_serial = false;

  // initialize serial port
  Serial.begin(57600);

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_DIST / 1000.0); //[20203118] duty_chg_per_interval 값 설정
}



void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  // [20213090]
  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }


  ////////////////////
  // Event handlers //
  ////////////////////

  if (event_dist) {
    event_dist = false;
    // get a distance reading from the distance sensor
    dist_raw = ir_distance_filtered();
    dist_ema = (1 - _DIST_ALPHA) * dist_ema + _DIST_ALPHA * dist_raw; // [20213083]

    // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr; //[20213083]
    dterm = _KD * (error_curr - error_prev);  //[20191979]
    control = pterm + dterm; //[20203118]

    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control; //[20213086]

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    if(duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    //[20213086]

    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false; //[20203118]

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    } //[20213086]

    // update servo position
    myservo.writeMicroseconds(duty_curr); //[20213086]
  }

  if (event_serial) {
    event_serial = false; //[20203118]
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",dist_ema:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",dterm:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  val = 100 + 300.0 / (_DIST_MAX - _DIST_MIN) *(val - _DIST_MIN);
  // val = 100 + 300.0 / (b - a) * (val - a);
  return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  float dist = ir_distance();
  dist = Lagrange_interpolation(x, y, dist);
  return dist;
}

float Lagrange_interpolation(float x[], float y[], float a) {
  float b = 0;
  for(int i=0;i<nd;i++) {
      float p=1;
      for(int j=0;j<nd;j++) {
          if(i!=j) {
            p = p*(a-x[j])/(x[i]-x[j]);
          }
      }
      b += p * y[i];
  }
  return b;  
}
