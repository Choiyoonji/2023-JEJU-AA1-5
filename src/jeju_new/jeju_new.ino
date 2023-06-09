#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

#include "math.h"

//////////////////////////////
const int RUN_PWM = 4; // move or stop
const int RUN_DIR = 5; // foward or backward
const int RUN_BRK = 6; // let motor move
const int STEER_PWM = 8; // motor speed
const int STEER_DIR = 9; // left or right
const int STEER_BRK = 10; // let motor move

#define CLK 2   // 3번핀을 CLK로 지정 otb
#define DT 3 // 2번핀을 DT로 지정 ota
//////////////////////////////

//////////////////////////////
int counter = 0;           // 회전 카운터 측정용 변수
int currentStateCLK;       // CLK의 현재 신호상태 저장용 변수
int lastStateCLK;          // 직전 CLK의 신호상태 저장용 변수 
String currentDir ="";      // 현재 회전 방향 출력용 문자열 저장 변수
unsigned long lastButtonPress = 0;     // 버튼 눌림 상태 확인용 변수
bool E_STOP = 1;
bool MANUAL = 1;
bool AUTO = 0;
////////////////////////////////////
const int velocity = 81;
////////////////////////////////
void goForward(int intVelocity);
void goBackward(int intVelocity);
void turnLeft(int intSteer);
void turnRight(int intSteer);
void brake(bool k); 
void encoder();
///////////////////////////////

ros::NodeHandle  nh;

std_msgs::Int16 read_speed;
std_msgs::Int32 read_steer;

const int MAX_SPEED = 81;
const int MAX_STEER = 22;

int currentSpeed = 0;
int currentSteer = 0;
int currentGear = 0;

void setMode(const geometry_msgs::Twist& msg){
  float speed = msg.linear.x;
  float steer = msg.angular.z;
  int en = counter*4.4;

  digitalWrite(RUN_BRK, LOW);

  if(steer > MAX_STEER) steer = MAX_STEER;
  else if(steer < -MAX_STEER) steer = -MAX_STEER;

  if(en < steer) turnRight(steer);
  else turnLeft(steer);

  if(speed > MAX_SPEED) speed = MAX_SPEED;
  else if(speed < -MAX_SPEED) speed = -MAX_SPEED;

  if(speed == 0.0) brake(1);
  else if(speed > 0) goForward(speed);
  else goBackward(30);
}

void setCommand(const geometry_msgs::Twist& msg){
  float a = msg.angular.z;
  int angle = abs(8*msg.angular.z);
  float v = msg.linear.x;
  int vel = 20*msg.linear.x;

  static int velocity_F = 0;
  static int velocity_B = 0;
  encoder();
  int currentAngle = counter*4.4;

  digitalWrite(RUN_BRK, LOW);

  if (v == 0.5){
    velocity_B = 0;
    velocity_F += vel;
    if (velocity_F > MAX_SPEED) velocity_F = MAX_SPEED;
    goForward(velocity_F);
  }
  else if (v == 0){
    if (velocity_F + velocity_B) brake(1);
    else brake(0);
    velocity_F = 0;
    velocity_B = 0;
  }
  else if (v == -0.5){
    velocity_F = 0;
    velocity_B = 60;
    goBackward(velocity_B);
  }

  if (a == -1){
    int angular = currentAngle + angle;
    if (angular > MAX_STEER) angular = MAX_STEER;
    turnRight(angular);
  }
  else if (a == 1){
    int angular = currentAngle - angle;
    if (angular < -MAX_STEER) angular = -MAX_STEER;
    turnLeft(angular);
  }
  else {
    if ((v == 0)&&(velocity_F + velocity_B)){
      if (currentAngle < 0) turnRight(0);
      else turnLeft(0);  
    }
    else if ((v == 0)&&(velocity_F + velocity_B == 0)){
      brake(0);
    }
    else{
      if (currentAngle < 0) turnRight(0);
      else turnLeft(0);
    }
  }
}

ros::Publisher speed_read("speed_read",&read_speed);
ros::Publisher steer_read("steer_read",&read_steer);
ros::Subscriber<geometry_msgs::Twist> getCMD("cmd_write", setCommand);
ros::Subscriber<geometry_msgs::Twist> erp_write("erp_write", setMode);

void goForward(int intVelocity = velocity)
{
  currentSpeed = intVelocity;
  currentGear = 1;
  digitalWrite(RUN_BRK, LOW);  
  analogWrite(RUN_PWM, intVelocity);
  digitalWrite(RUN_DIR, LOW);   
  delay(5);
  // delay(3000);
  // analogWrite(RUN_PWM, intVelocity);
  // delay(400);
}

void goBackward(int intVelocity = velocity)  
{
  currentSpeed = intVelocity;
  currentGear = 2;
  digitalWrite(RUN_BRK, LOW);
  analogWrite(RUN_PWM, intVelocity);
  digitalWrite(RUN_DIR, HIGH);   
  delay(5);
}

void turnLeft(int intSteer)
{
  digitalWrite(STEER_BRK, LOW); 
  digitalWrite(STEER_DIR, LOW);
  analogWrite(STEER_PWM, 60);

  int min_en = intSteer/4.4 - 1;

  if(min_en < -7) min_en = -7;

  int max_en = min_en + 1;

  while(1){
    encoder();
    int en = counter; 

    if(en >= min_en && en <= max_en) break;
  }

  digitalWrite(STEER_BRK, HIGH);
  analogWrite(STEER_PWM, 0);

  delay(5); 
}

void turnRight(int intSteer)
{
  digitalWrite(STEER_BRK, LOW);
  digitalWrite(STEER_DIR, HIGH); 
  analogWrite(STEER_PWM, 60);

  int max_en = intSteer/4.4+1;

  if(max_en > 6) max_en = 6;

  int min_en = max_en - 1;

  while(1){
    encoder();
    int en = counter; 
    if(en >= min_en && en <= max_en) break;
  }

  digitalWrite(STEER_BRK, HIGH);
  analogWrite(STEER_PWM, 0);

  delay(5); 
}

void brake(bool k = 1) // k가 1이면 brake 핀으로 정지, 0이면 pwm 핀으로 정지
{
  currentSpeed = 0;
  if (k) digitalWrite(RUN_BRK, HIGH); 
  analogWrite(RUN_PWM, 0);
  delay(30);
}

void encoder()
{
	// CLK핀의 상태를 확인
	currentStateCLK = digitalRead(CLK);

	// CLK핀의 신호가 바뀌었고(즉, 로터리엔코더의 회전이 발생했고), 그 상태가 HIGH이면(최소 회전단위의 회전이 발생했다면) 
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

		// DT핀의 신호를 확인해서 엔코더의 회전 방향을 확인함.

		if (digitalRead(DT) != currentStateCLK) {  // 신호가 다르다면 시계방향 회전
			counter ++;                           // 카운팅 용 숫자 1 증가
			currentDir ="우회전";
		} else {                                 // 신호가 같다면 반시계방향 회전
			counter --;                         // 카운팅 용 숫자 1 감소
			currentDir ="좌회전";
		}
	}

  if(counter > 22) counter = 22;
  else if(counter < -22) counter = -22;

	// 현재의 CLK상태를 저장
	lastStateCLK = currentStateCLK;
	
  delay(1);

}

void setup() {
  nh.initNode();

  nh.subscribe(getCMD);
  nh.subscribe(erp_write);

  nh.advertise(speed_read);
  nh.advertise(steer_read);

  nh.negotiateTopics();

  pinMode(RUN_DIR, OUTPUT);
  pinMode(RUN_PWM, OUTPUT);
  pinMode(RUN_BRK, OUTPUT);

  pinMode(STEER_DIR, OUTPUT);
  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_BRK, OUTPUT);

  pinMode(CLK,INPUT);
	pinMode(DT,INPUT);

	// CLK핀의 현재 상태 확인
	lastStateCLK = digitalRead(CLK);	

  attachInterrupt(0, encoder, CHANGE);
	attachInterrupt(1, encoder, CHANGE);
}

int intSpeed = 0;
int intSteer = 0;

void loop() {
  intSteer = counter*4.4;

  // erpRead.read_E_stop = 0;
  // erpRead.read_gear = currentGear;
  // erpRead.read_steer = intSteer;
  // erpRead.read_speed = currentSpeed;

  read_speed.data = currentSpeed;
  read_steer.data = intSteer;

  speed_read.publish(&read_speed);
  steer_read.publish(&read_steer);
  
  nh.spinOnce();

  delay(10);
}