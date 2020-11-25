#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9  //[1692] LED 9번핀에 연결
#define PIN_SERVO 10 // [3228] 서보10핀에 연결
#define PIN_IR A0  // [3133] 적외선 센서 signal -> A0핀

// Framework setting
#define _DIST_TARGET 255  //[0028] 목표 위치가 25.5cm임을 선언
#define _DIST_MIN 60
#define _DIST_MAX 450

// Distance sensor
#define _DIST_ALPHA 0.5   // [1628] ema 필터의 측정 보정치

// Servo range
#define _DUTY_MIN 1200 
#define _DUTY_NEU 1455 
#define _DUTY_MAX 2200

// Servo speed control
#define _SERVO_ANGLE 40        // [3131] servo 각도 설정
#define _SERVO_SPEED 90        // [3141] servo 속도 설정

// Event periods
#define _INTERVAL_DIST 20         // [3123] distance 측정 이벤트 주기
#define _INTERVAL_SERVO 20     // [3123] servo 조정 이벤트 주기
#define _INTERVAL_SERIAL 100   // [3123] serial 출력 이벤트 주기

// PID parameters
#define _KP 0.7

//////////////////////
// global variables //
//////////////////////
float dist_min, dist_max, filtered_dist, alpha; // [3228]
// Servo instance
Servo myservo; 

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval; 
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
// initialize GPIO pins for LED and attach servo 
 pinMode(PIN_LED,OUTPUT);
 digitalWrite(PIN_LED, 1);
 myservo.attach(PIN_SERVO); 

// initialize global variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  dist_raw = 0.0;
  dist_ema = 0;
  alpha = _DIST_ALPHA;
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
  

// move servo to neutral position
 myservo.writeMicroseconds(2399); 

// initialize serial port
Serial.begin(57600); //[3128] 시리얼 포트 초기화

// convert angle speed into duty change per interval. ****How to?
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * ((float)_SERVO_SPEED / 180) * ((float)_INTERVAL_SERVO / 1000); //[3128]
}

void loop() {
/////////////////////
// Event generator // [3133] 이벤트 실행 간격 구현 
/////////////////////
  if (millis() >= last_sampling_time_dist + _INTERVAL_DIST) event_dist = true;
  if (millis() >= last_sampling_time_servo + _INTERVAL_SERVO) event_servo = true;
  if (millis() >= last_sampling_time_serial + _INTERVAL_SERIAL) event_serial = true;

////////////////////
// Event handlers //
////////////////////
  if(event_dist) {
      event_dist = false; // [3133]
  // get a distance reading from the distance sensor
// ****      filtered_dist = ir_distance_filtered(); //[0028] 적외선 센서 필터링 값 저장

  // PID control logic
    error_curr = _DIST_TARGET - ir_distance(); // 현재 오차 저장, 일단은 필터링 미구현
    pterm = error_curr * _KP; //[0028] kp * 오차
    control = pterm;  //[0028] 제어량 계산

  // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

  // [3133] keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN; 
  
    last_sampling_time_dist = millis(); // [3133] 마지막 dist event 처리 시각 기록
  }
  
  if(event_servo) {
    event_servo = false; // [3133]
    // update servo position
    myservo.writeMicroseconds(duty_target);
    // adjust duty_curr toward duty_target by duty_chg_per_interval ...How to?


    
    last_sampling_time_servo = millis(); // [3133] 마지막 servo event 처리 시각 기록

  }
  dist_raw = ir_distance();
  duty_curr = myservo.read();
  if(event_serial) {
    event_serial = false; // [3133]
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    last_sampling_time_serial = millis(); // [3133] 마지막 serial event 처리 시각 기록

  }
}

float ir_distance(void){ // return value unit: mm 
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;

}
