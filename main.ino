#include <ESP32_Servo.h>
#include <PID_v1.h>
Servo servo1;
Servo servo2;
const int SERVO_str = 16;     // Servo for steering - connected to pin 16
const int SERVO_sonic = 18;  // Servo for ultrasonic - connected to pin 18
const int IR_R1 = 36;         // IR sensor pins 34, 35, 36, 39
const int IR_R2 = 39;
const int IR_L2 = 34;
const int IR_L1 = 35;
const int M1_IN1 = 2;         // DC motor 1 IN1 pin
const int M1_IN2 = 4;         // DC motor 1 IN2 pin
const int M2_IN1 = 13;        // DC motor 2 IN1 pin
const int M2_IN2 = 27;        // DC motor 2 IN2 pin
const int S_trig = 32;        // Ultrasonic sensor trigger pin
const int S_echo = 23;        // Ultrasonic sensor echo pin
const int TRIG_PIN = S_trig;  // Alias for compatibility
const int ECHO_PIN = S_echo;  // Alias for compatibility
float speedL_fr, speedR_fr, speedL_bk, speedR_bk;  // Left and right speed control
int IR_detect = 500;  // Threshold for detecting white line
float cur_steering;  // Current steering angle variable
int cur_IR_R1, cur_IR_R2, cur_IR_L2, cur_IR_L1;  // Current IR sensor values
int IR_R1_detect_w = 500; // Threshold for detecting white line
int IR_R2_detect_w = 500;
int IR_L2_detect_w = 500;
int IR_L1_detect_w = 500;
const int MAX_DISTANCE = 1800;
//######################### for test #########################//
int state = 0; //그냥 할 때는 0, 테스트 할 때는 원하는 지점부터
//######################### for test #########################//
// Ultrasonic sensor PID control variables and constants
//시작용
double START_WALL_TARGET_DISTANCE = 9;
double START_WALL_DISTANCE_MIN = 5;
double START_WALL_DISTANCE_MAX = 20;
double input_sonic, output_sonic;
double Kp_sonic = 5.0;  // P constant
double Ki_sonic = 0;    // I constant
double Kd_sonic = 0;    // D constant
PID pid_sonic(&input_sonic, &output_sonic, &START_WALL_TARGET_DISTANCE, Kp_sonic, Ki_sonic, Kd_sonic, DIRECT);
int pid_switch = 0;
// vertical_parking hyperparameters
int Rear_i=0;
// Obstacle detection hyperparameters
int obstacle_num = 0;
float OBSTACLE_DETECTION_DISTANCE = 25;
int OBSTCL1_AVDC_T = 2500;
int OBSTCL1_PRLL_T = 2100;
int OBSTCL2_AVDC_T = 2700;
int OBSTCL2_PRLL_T = 2100;
// Ultrasonic sensor PID control for parallel parking variables and constants
// 평행주차용 hyperparameters
int parallel_switch = 0;
double PARALLER_WALL_TARGET_DISTANCE = 9;
double PARALLER_WALL_DISTANCE_MIN = 5;
double PARALLER_WALL_DISTANCE_MAX = 24;
double input_sonic_for_p_p, output_sonic_for_p_p;
double Kp_sonic_for_p_p = 5.0;  // P constant
double Ki_sonic_for_p_p = 0;    // I constant
double Kd_sonic_for_p_p = 0;    // D constant
PID pid_sonic_for_p_p(&input_sonic_for_p_p, &output_sonic_for_p_p, &PARALLER_WALL_TARGET_DISTANCE, Kp_sonic_for_p_p, Ki_sonic_for_p_p, Kd_sonic_for_p_p, DIRECT);
//-------------------Basic Function-------------------//
// Speed control function //
bool obstacle_detected() {
  if (GetDistance() < OBSTACLE_DETECTION_DISTANCE) {
    return true;
  } else {
    return false;
  }
}
void Setspeed(float speed) {
  speedL_fr = speed;
  speedR_fr = speed;
  speedL_bk = abs(speed);
  speedR_bk = abs(speed);
  if (speed > 0) {
    analogWrite(M1_IN1, speedL_fr);
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, speedR_fr);
    analogWrite(M2_IN2, 0);
  } else if (speed < 0) {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, speedL_bk); // 후진 상태일 때는 speedL_fr 대신 speedL_bk 사용
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, speedR_bk); // 후진 상태일 때는 speedR_fr 대신 speedR_bk 사용
  } else {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, 0);
  }
}
// pause function //
void pause(int pause_time) {
  // Stop and normalize steering
  Setspeed(0);
  Setsteering(0);
  delay(pause_time);
}
// steering control function //
void Setsteering(float steering) {
  cur_steering = constrain(steering, -45, 45); // Adjust the range as needed
  servo1.write(100 + cur_steering);
}
// sonic servo function //
void Setsonicservo(float angle){ //180 왼, 90 중간, 0 오른
  servo2.write(angle);
}
// detecting stop line //
bool detect_stop_line(){
  if (cur_IR_R2 < IR_R2_detect_w && cur_IR_L2 < IR_L2_detect_w) {
    return true;
  } else {
    return false;
  }
}
// Infrared line following //
void straight_ir() {
  cur_IR_R1 = analogRead(IR_R1);
  cur_IR_R2 = analogRead(IR_R2);
  cur_IR_L2 = analogRead(IR_L2);
  cur_IR_L1 = analogRead(IR_L1);
  // Lane detection code implementation
  if (cur_IR_R1 > IR_R1_detect_w && cur_IR_R2 > IR_R2_detect_w && cur_IR_L2 > IR_L2_detect_w && cur_IR_L1 > IR_L1_detect_w) {
    // 선 검출 x (차도 위) -> 직진
    Setspeed(255);
    Setsteering(0);
  } else {
    // If the lane is not detected, adjust the steering to follow the lane
    // The specific logic adjusts the steering angle to make the vehicle align with the center of the lane
    // There are multiple methods, so experiment to find a suitable logic
    // The following is an example code that adjusts the steering angle depending on whether the lane is on the left or right.
    if (cur_IR_L1 < IR_L1_detect_w) {
      // 왼쪽 선 검출 -> 우회전(+)
      Setsteering(30);
      delay(100);
      Setsteering(0);
    } else if (cur_IR_R1 < IR_R1_detect_w) {
      // 오른쪽 선 검출 -> 좌회전(-)
      Setsteering(-30);
      delay(100);
      Setsteering(0);
    } else {
      // If both sides are detected, go straight
      Setsteering(0);
    }
  }
}
// 주차를 위한 적외선 주행 코드
void straight_p() {
  cur_IR_R1 = analogRead(IR_R1);
  cur_IR_R2 = analogRead(IR_R2);
  cur_IR_L2 = analogRead(IR_L2);
  cur_IR_L1 = analogRead(IR_L1);
  // Lane detection code implementation
  if (cur_IR_R1 > IR_R1_detect_w && cur_IR_R2 > IR_R2_detect_w && cur_IR_L2 > IR_L2_detect_w && cur_IR_L1 > IR_L1_detect_w) {
    // 선 검출 x (차도 위) -> 직진
    Setspeed(80);
    Setsteering(0);
  } else {
    if (cur_IR_L1 < IR_L1_detect_w) {
      // 왼쪽 선 검출 -> 우회전(+)
      Setsteering(30);
      delay(100);
      Setsteering(0);
    } else if (cur_IR_R1 < IR_R1_detect_w) {
      // 오른쪽 선 검출 -> 좌회전(-)
      Setsteering(-30);
      delay(100);
      Setsteering(0);
    } else {
      Setsteering(0);
    }
  }
}
// Ultrasonic distance measurement //
float GetDistance() {
  // Assuming TRIG_PIN and ECHO_PIN are defined elsewhere in your code
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 5000);
  float distance = (duration == 0) ? MAX_DISTANCE : duration * 0.034 / 2;
  // Print distance to Serial Monitor
  Serial.print("obstacle_num: ");
  Serial.print(obstacle_num);
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}
//----------------mission_start-------------// 후방주차 전 기본 주행 함수
void mission_start() {
  // 초음파 서보를 180도(왼)로 회전
  Setsonicservo(180);
  Setsteering(0);
  if (detect_stop_line()) { //첫 정지선 검출시 정지 후 출발
    pause(1500);
    pid_sonic.SetMode(AUTOMATIC);
    Setspeed(150);
    delay(1000);
  }
  float distance = GetDistance();
  while (distance >= START_WALL_DISTANCE_MIN && distance <= START_WALL_DISTANCE_MAX) { //초음파 거리가 적정 거리 이내에서 작동 
    pid_switch = 1;
    Setspeed(200);
    pid_sonic.SetOutputLimits(-45, 45);
    // 초음파 거리 측정
    float distance_for_start = GetDistance();
    // PID 입력에 현재 거리 설정
    input_sonic = distance_for_start;
    // PID 계산
    pid_sonic.Compute();
    // PID 출력을 사용하여 SERVO_str을 조절
    Setsteering(output_sonic);
    // 일정 간격으로 반복
    delay(100);
    Serial.println("초음파 거리가 적정치입니다.");
    if (distance_for_start > START_WALL_DISTANCE_MAX && distance_for_start != MAX_DISTANCE){
      break;
    }
  }
  if (pid_switch == 1) {
    pause(500);
    pid_switch = 2;
  }
  straight_ir(); //별 일 없으면 기본주행
  if (pid_switch == 2) { //pid 구간 끝나면 제어기 종료 및 정지
    pause(1000);
    pid_sonic.SetMode(MANUAL);
    state = 1;
  }
}




//-------------------vertical_parking-------------------// 후진주차에서 따로 작성한 함수
//  !!!!!!!!!!!!!필독!!!!!!!!!!!
// 여기서는 중앙값을 98로 줘서 전부 -2 함 !!!!!
// pause 함수 많이 썼는데, 조향 0 만들고, 속도 0 만드는 함수임
// 후방주차시 조향 움직이는 동시에 차가 움직이면 부정확할 수 있어서, 정확한 조향 및 움직임 위해 조향 움직이고 딜레이 조금씩 넣었음.
void rear_parking() {
  
  // b1. 건널목 넘어가는 부분
  // 스위치들 정확성을 위해 부등호 -> 등호 !!!!!
  if (Rear_i == 0) {
    // b1-1. 교차지점에서 출발 및 카운트 올림 (나경누나꺼 1.)
    Setsteering(-2);
    servo2.write(180); // 시작 코드와 중복되는 내용들 몇개 지움 !!!!!
    pause(300); // 서보 움직이는 시간 생각해서 딜레이 !!!!!
    Setspeed(100);
    delay(600); // 1-> 2로 가자마자 실행 시, 앞쪽 벽을 계산할까봐 딜레이 넣음 !!!!!
    // b1-2. 교차지점 넘어서 건너편 벽 인식 (나경누나꺼 2.)
    while (true) {
      float temp_distance_for_v_p_1 = GetDistance();
      if(temp_distance_for_v_p_1 < 14){
        pause(500); //잠시 중지는 b2보다 여기서 하는게 나음 !!!!!
        Rear_i = 1;
        break;
      }
    }
  }
  // b2.  건널목에서 앞으로 갔다 ㄱ자로 뒤로 빠지기 (b2에서는 while(true) 써서 코드 리딩을 가두는 구간이 좀 많습니다.)
  else if (Rear_i == 1) {
    // b2-1. 앞으로 가는 부분 (나경누나꺼 3.)
    Setspeed(100);
    delay(800);
    pause(200);
    // b2-2. ㄱ자 좌회전 및 후진 (나경누나꺼 3~4.)
    Setsteering(-32);
    delay(200);
    Setspeed(-100);
    delay(2000);
    pause(100);
    Setsteering(-2);
    delay(200);
    Setspeed(-80);
    // b2-3. 모서리(50) 인식 이후 정지하고, 28(30)으로 둘 중 하나 디텍팅까지 전진 우회전. (나경누나꺼 5.)
    while (true) {
      float temp_distance_for_v_p_2 = GetDistance();
      if(temp_distance_for_v_p_2 < 50) { // 50 아래면 멈춤
        pause(500);
        Setsteering(-10);
        delay(200);
        Setspeed(120);
        while (true) { // !!!!! 정지선 검출 시 까지 void 루프가 아닌, 코드가 이 구간에 멈추도록 함. -> set speed는 가장 최신인 150일거. (이짓거리 한 이유는 다시 보이드 루프 타면, 예기치 못한 오류 발생 가능성 유)
          cur_IR_R1 = analogRead(IR_R1);
          cur_IR_R2 = analogRead(IR_R2);
          cur_IR_L2 = analogRead(IR_L2);
          cur_IR_L1 = analogRead(IR_L1);
          if (cur_IR_L2 < IR_L2_detect_w || cur_IR_R2 < IR_R2_detect_w) { // 둘 중 하나 디텍팅 시 pause !!!!!
            pause(500);
            break; // 2번째 중첩된 무한루프 while (true) 탈출
          }
        }
        Rear_i = 2; // 이제 앞으로 b3로 갈거 (보이드 루프 도는거임)
        break; // 첫번쨰 while (true) 탈출
      }
    }
  }

  // b3 (나경누나꺼 6.) 미세조정 파트라 이 부분은 안 건들이겠습니다...
  else if( Rear_i == 2 ){
    // 이 아래 두줄은 왜 있는 지 모르겠어서 주석처리 했습니다... !!!!!

    Setsteering(-37);
    delay(200);
    Setspeed(-150);
    delay(800);
    pause(300);

    Setsteering(18);
    delay(200);
    Setspeed(-150);
    delay(500);
    pause(300);
    
    // b3-2. 정지선까지 후진~~ (나경누나꺼 7.)
    Setsteering(-2);
    Setspeed(-150);
    while (true) { //무한루프 가둬서 정지 검출detect_stop_line로, 멈추는거 pause로 바꿈  !!!!!
      cur_IR_R1 = analogRead(IR_R1);
      cur_IR_R2 = analogRead(IR_R2);
      cur_IR_L2 = analogRead(IR_L2);
      cur_IR_L1 = analogRead(IR_L1);
      if(detect_stop_line()){
        pause(3000);
        Setspeed(150);
        delay(1000);
        Rear_i = 3; // 그냥 쓰레기 값 넣음 !!!!!
        break;
      }
    }
  }

  // b4. ????? 회피주행 이후까지 직진하는 코드... 정지선 검출 이후 앞으로 살짝 가고 적외선 갱신
  else if( Rear_i == 3 ) {
    straight_ir();
    if (detect_stop_line()){
      pause(3000);
      Setspeed(80);
      delay(800);
      straight_ir();
      state = 2; // 2번 state로 보내줌 !!!!! (다음 루프에는 obstacle로 갈 것)
    }
    else {
      straight_ir();
    }
  }
}




//-------------------Obstacle-------------------// 장애물 회피 주행에서 따로 작성한 함수
void obstacle_avoidance() {
  //c1. If there is an obstacle
  Setsonicservo(80);
  if (obstacle_detected()) {
    // c1-1. Obstacle 1
    if (obstacle_num == 0) {
      pause(50);
      Setsteering(30); // Turn right
      delay(100);
      Setspeed(80);
      delay(OBSTCL1_AVDC_T); // Minimum avoidance time
      while (obstacle_detected()) // Keep turning right if obstacle is detected
      {
        delay(100);
      }
      // If no obstacle, align parallel
      pause(50);
      Setsteering(-30); // Turn left
      delay(100);
      Setspeed(80);
      delay(OBSTCL1_PRLL_T); // Horizontal alignment time
      obstacle_num++;
    }
    // c1-2. Obstacle 2
    else if (obstacle_num == 1) {
      pause(50);
      Setsteering(-30); // Turn left
      delay(100);
      Setspeed(80);
      delay(OBSTCL2_AVDC_T); // Minimum avoidance time
      while (obstacle_detected()) // Keep turning left if obstacle is detected
      {
        delay(100);
      }
      // If no obstacle, align parallel
      pause(50);
      Setsteering(30); // Turn right
      delay(100);
      Setspeed(80);
      delay(OBSTCL2_PRLL_T); // Horizontal alignment time
      obstacle_num++;
    }
  }
  // c2. If there is no obstacle
  else {
    //c2-1. 급커브 만났을 때 후진 후진 좌회전
    if (obstacle_num == 2){
      if (detect_stop_line()){
        Serial.println("detect_stop_line1");
        pause(1000);
        Setsteering(17);
        Setspeed(-150);
        delay(750);
        pause(500);
        Setsteering(-17);
        Setspeed(-150);
        delay(750);
        pause(500);
        Setsteering(-25);
        Setspeed(150);
        delay(1800);
        obstacle_num = 3;
      }
      else{
        straight_ir(); // Call the lane detection function if no obstacle is detected
      }
    }
    //c2-2. 급커브 이후에 정지선 검출 시 스테이트 변경 및 (살짝 전진은 커브에서) 
    else if (obstacle_num == 3){
      straight_ir();
      if (detect_stop_line()){
        pause(1000);
        obstacle_num = 4;
        state = 3;
      }
      else { // 정지선 안 만나면, 그냥 움직
        straight_ir();
        Serial.print(obstacle_num);
      }
    }
    //c2-3. 급커브 이전에는 그냥 기본 주행
    else{
      straight_ir(); // Call the lane detection function if no obstacle is detected
    }
  }
}
//--------------------Curve--------------------// 장애물 회피 주행에서 따로 작성한 함수
void curve() {
  // d1. 커브에서 살짝 전진
  Setsteering(10);
  Setspeed(80);
  delay(1000);
  // d2. 정지선 만날때까지 커브 회전 + 살짝 전진 + 초음파 왼쪽보게
  while(true){
    straight_ir();
    if (detect_stop_line()){
      Setsonicservo(90);
      pause(2000);
      
      Setsteering(10);
      Setsonicservo(180);
      Setspeed(80);
      delay(1000);
      state = 4;
      break;
    }
  }
}




//----------------Parallel&End----------------// 평행주차에서 따로 작성한 함수
// e2-1. 제어기 시작 전 속도 할당
void pidControl() {  
  Setsonicservo(180); //테스트용
  Setspeed(150);
  delay(200);
  // e2-2. 제어기 시작 (코드 가둠)
  while (true) {
    float temp_distance_for_p_p_1 = GetDistance();
    pid_sonic_for_p_p.SetOutputLimits(-45, 45);
    input_sonic_for_p_p = temp_distance_for_p_p_1;
    pid_sonic_for_p_p.Compute();
    Setsteering(output_sonic_for_p_p);
    delay(100);  
    // e2-3. 거리가 일정 값 이상이면 PID 제어 종료 및 정지
    if (temp_distance_for_p_p_1 > PARALLER_WALL_DISTANCE_MAX && temp_distance_for_p_p_1 != MAX_DISTANCE) {
      pause(2000);
      Setsonicservo(90);
      delay(200);
      pid_sonic_for_p_p.SetMode(MANUAL);
      parallel_switch = 2;
      Serial.println("주차시작");
      break;
    }
  }
}
// e3-1. 전진 1차진입 (스위치2)
void parking() {
  Serial.println("pstart");
  Serial.println(parallel_switch);
  if (parallel_switch == 2) { 
    Setsteering(-3);
    delay(200);
    Setspeed(150);
    delay(2920);
    pause(200);
    parallel_switch = 3;
    // cur_IR_R1 = analogRead(IR_R1);
    // cur_IR_R2 = analogRead(IR_R2);
    // cur_IR_L2 = analogRead(IR_L2);
    // cur_IR_L1 = analogRead(IR_L1);
    // Setsteering(-2);
    // Setspeed(80); // 좌회전 속도
    // Serial.println("ㄴㄴ");
    // if (cur_IR_L1 <= IR_L1_detect_w && cur_IR_L2 <= IR_L2_detect_w) { // 왼쪽 적외선 만나면 다음 스위치3
    //   Serial.println("ㅇㅇ");
    //   pause(1000);
    //   parallel_switch = 3;
      
    // }
  }
  // e3-2. 좌->우회전 후진 주차 주차 2차진입 (스위치3)
  if (parallel_switch == 3) { //검은색일때 좌회전
    Setsteering(-35);
    delay(200);
    Setspeed(-100); // 좌회전 후진 속도
    delay(1800);
    pause(500);

    Setsteering(35);
    delay(200);
    Setspeed(-100); // 우회전 후진 속도
    delay(1800);
    pause(500);
    parallel_switch = 4; // 하드 코딩 이후 다음 스위치 4
  }
  // e3-3. 탈출
  if (parallel_switch == 4) {
    // e3-3-1. 후진
    Setspeed(-100);
    delay(750);
    pause(500);
    // e3-3-2. 우회전 전진
    Setsteering(25);
    delay(100);
    Setspeed(100); // 우회전 속도
    delay(1750);
    pause(200);
    // e3-3-3. 좌회전 전진
    Setsteering(-25);
    delay(100);
    Setspeed(100); // 좌회전 속도
    delay(1700);
    pause(200);
    Setspeed(255);
    parallel_switch = 5;
  }
  // e4. 주차지점까지 진입 및 주차
  if (parallel_switch == 5){
    straight_ir();
    if (detect_stop_line()){
      Serial.println("end");
      pause(10000);
    }
  }
}
// ## e ## //
void parallel_parking(){
  Serial.println("pp start");
  Serial.println(parallel_switch);
  Setsonicservo(180);
  // e1-1. 벽 만나기 전까지 적외선 주행 (살짝 전진 + 초음파 왼쪽보게 한 이후에 시작임)
  if (parallel_switch == 0){
    Serial.println("pp sw 0");
    straight_ir();
    float distance = GetDistance();
    // e1-2. 벽 만남 -> pid 세팅 시작
    if (distance >= PARALLER_WALL_DISTANCE_MIN && distance <= PARALLER_WALL_DISTANCE_MAX){
      pause(500);
      double target_distance = PARALLER_WALL_TARGET_DISTANCE;
      double distance_min = PARALLER_WALL_DISTANCE_MIN;
      double distance_max = PARALLER_WALL_DISTANCE_MAX;
      double Kp = Kp_sonic_for_p_p;
      double Ki = Ki_sonic_for_p_p;
      double Kd = Kd_sonic_for_p_p;
      pid_sonic_for_p_p.SetMode(AUTOMATIC);
      parallel_switch = 1;
      delay(800);
    }
  }
  // e2. pid 진입 -> 구멍 앞 정지 
  if (parallel_switch == 1) {
    //PID 제어로 주차공간까지 오면
    pidControl();
  }

  // e3. 주차 및 탈출 -> e4. 종료
  else {
    Serial.println("dhflkajsfljsdfklahsdffh");
    parking();
  }
}




//-------------------Main-------------------// 코드 여기에 집어넣으면댐
void driving(){
  if(state==0){                       //스타트 함수
    Serial.print("State:");
    Serial.println(state);
    mission_start();
  }
  else if(state==1) {                 //후방주차
    Serial.print("State:");
    Serial.println(state);
    rear_parking();
  }
  else if(state==2) {                  //장애물 회피주행
    Serial.print("State:");
    Serial.println(state);
    obstacle_avoidance();
  }
  else if(state==3) {                   //커브 주행
    Serial.print("State:");
    Serial.println(state);
    curve();
  }
  else if(state==4) {                   //평행주차 및 정지
    // Serial.print("State:");
    Serial.print("State:");
    Serial.println(state);
    Serial.println(parallel_switch);
    
    parallel_parking();
  }
}




void setup(){
  Serial.begin(115200); // Set the baud rate to 115200 for Serial Monitor
  servo1.attach(SERVO_str);
  servo2.attach(SERVO_sonic);
  pinMode(IR_R1, INPUT);
  pinMode(IR_R2, INPUT);
  pinMode(IR_L2, INPUT);
  pinMode(IR_L1, INPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop(){
  driving();
}
//알바조타 setup 값이랑 main 함수별 직렬모니터 출력 추가함
