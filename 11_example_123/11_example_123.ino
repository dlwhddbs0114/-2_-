
#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 180.0   // minimum distance to be measured (unit: mm) -> 18cm
#define _DIST_MAX 360.0   // maximum distance to be measured (unit: mm) -> 36cm

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

// EMA 필터의 가중치 (alpha 값). 노이즈와 반응 속도 간의 균형을 맞추기 위한 최적 값.
#define _EMA_ALPHA 0.5 

// 서보 제어를 위한 상수
#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_MAX 2000 // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = _DIST_MAX;  // unit: mm
unsigned long last_sampling_time;       // unit: ms

Servo myservo;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_MIN); // 초기화: 0도

  // initialize USS related variables
  dist_prev = _DIST_MIN; // raw distance output from USS (unit: mm)

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  // wait until next sampling time. 
  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // 범위 필터 적용: 18cm~36cm 범위에서만 유효
  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;           // 범위 밖이면 이전 거리값 사용
    digitalWrite(PIN_LED, LOW);     // 범위 밖이면 LED OFF
  } else {
    digitalWrite(PIN_LED, HIGH);    // 범위 안에 들어오면 LED ON
    dist_prev = dist_raw;           // 범위 내면 현재 거리값 저장
  }

  // EMA 필터 적용
  dist_ema = _EMA_ALPHA * dist_raw + (1 - _EMA_ALPHA) * dist_prev;
  dist_prev = dist_ema; // EMA 값을 이전 거리로 갱신

  // 서보 제어: 거리에 비례하여 0도 ~ 180도 사이로 서보 각도 조정
  int servo_pos;
  if (dist_ema <= _DIST_MIN) {
    servo_pos = _DUTY_MIN;  // 18cm 이하일 경우 0도
  } else if (dist_ema >= _DIST_MAX) {
    servo_pos = _DUTY_MAX;  // 36cm 이상일 경우 180도
  } else {
    // 18cm~36cm 사이일 경우 0도~180도를 비례적으로 변화
    servo_pos = map(dist_ema, _DIST_MIN, _DIST_MAX, _DUTY_MIN, _DUTY_MAX);
  }

  // 서보 모터 제어
  myservo.writeMicroseconds(servo_pos);

  // 거리 및 서보 각도 출력
  Serial.print("Raw:");    Serial.print(dist_raw);
  Serial.print(", EMA:");  Serial.print(dist_ema);
  Serial.print(", Servo:"); Serial.print(servo_pos);  
  Serial.println("");

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// 초음파 센서로부터 거리를 측정하는 함수 (mm 단위로 반환)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
