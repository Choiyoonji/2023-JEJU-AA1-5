#include <Arduino.h>
//#include <ros.h>
//#include <std_msgs/String.h>
// #include <std_msgs/Int16.h>
// #include <std_msgs/Int32.h>
// #include <geometry_msgs/Twist.h>
// #include "math.h"
////////////////////////////////////
const int RUN_DIR = 4; // foward or backward
const int RUN_PWM = 3; // move or stop
const int RUN_BRK = 5; // let motor move
const int STEER_DIR = 9; // left or right
const int STEER_PWM = 8; // motor speed
const int STEER_BRK = 10; // let motor move
const int DEFAULT_DEG = 18; // steer 각도 세팅

#define CLK 3   // 2번핀을 CLK로 지정 otb
#define DT 2 // 3번핀을 DT로 지정 ota
// #define SW 44    // 4번핀을 스위치핀으로 지정 ots

int counter = 0;           // 회전 카운터 측정용 변수
int currentStateCLK;       // CLK의 현재 신호상태 저장용 변수
int lastStateCLK;          // 직전 CLK의 신호상태 저장용 변수 
String currentDir ="";      // 현재 회전 방향 출력용 문자열 저장 변수
unsigned long lastButtonPress = 0;     // 버튼 눌림 상태 확인용 변수
////////////////////////////////////
const int velocity = 255 / 3;
////////////////////////////////
void goForward(int intVelocity);
void goBackward();
void turnLeft(int intSteer);
void turnRight(int intSteer);
void brake();
///////////////////////////////
// ros::NodeHandle  nh;
// std_msgs::Int16 speed_read;
// std_msgs::Int32 steer_read;
///////////////////////////////
void goForward(int intVelocity = velocity)
{
  digitalWrite(RUN_BRK, LOW);  
  analogWrite(RUN_PWM, intVelocity);
  delay(400);
  digitalWrite(RUN_DIR, LOW);   
  delay(3000);
  analogWrite(RUN_PWM, intVelocity);
  delay(400);
}

void goBackward(int intVelocity = velocity)
{
  digitalWrite(RUN_BRK, LOW);  
  analogWrite(RUN_PWM, intVelocity);
  delay(400);
  digitalWrite(RUN_DIR, HIGH);   
  delay(3000);
  analogWrite(RUN_PWM, intVelocity);
  delay(400);
}

void turnLeft(int intSteer)
{
  digitalWrite(STEER_BRK, LOW); 
  digitalWrite(STEER_DIR, LOW);
  analogWrite(STEER_PWM, 35);
  // delay(100);
  while(1)
    if(encoder() >= -7 && encoder() <= -6){
      Serial.println("done");
      break;
    }
      
    // if(encoder()*DEFAULT_DEG >= intSteer-5 || encoder()*DEFAULT_DEG <= intSteer+5)
    //   break;
  analogWrite(STEER_PWM, 0);
  delay(1000); 
}

void turnRight(int intSteer)
{
  digitalWrite(STEER_BRK, LOW);
  digitalWrite(STEER_DIR, HIGH); 
  analogWrite(STEER_PWM, 35);
  while(1){
    int en = encoder(); 
    if(en >= 5 && en <= 6){
      Serial.println(en);
      break;
    }
  }
    // if(encoder()*DEFAULT_DEG >= intSteer-5 || encoder()*DEFAULT_DEG <= intSteer+5)
    //   break;
  analogWrite(STEER_PWM, 0);
  delay(1000);
}

void straight()
{
  digitalWrite(STEER_BRK, LOW);
  digitalWrite(STEER_DIR, HIGH); 
  analogWrite(STEER_PWM, 35);
  while(1){
    int en = encoder(); 
    if(en >= 1 && en <= 0){
      Serial.println(en);
      break;
    }
  }
    // if(encoder()*DEFAULT_DEG >= intSteer-5 || encoder()*DEFAULT_DEG <= intSteer+5)
    //   break;
  analogWrite(STEER_PWM, 0);
  delay(1000);
}

void brake() 
{
  digitalWrite(RUN_BRK, HIGH);  
  delay(1000);
}

int encoder()
{
	// CLK핀의 상태를 확인
	currentStateCLK = digitalRead(CLK);

	// CLK핀의 신호가 바뀌었고(즉, 로터리엔코더의 회전이 발생했했고), 그 상태가 HIGH이면(최소 회전단위의 회전이 발생했다면) 
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

		// DT핀의 신호를 확인해서 엔코더의 회전 방향을 확인함.

		if (digitalRead(DT) != currentStateCLK) {  // 신호가 다르다면 시계방향 회전
			counter ++;                           // 카운팅 용 숫자 1 증가
			currentDir ="우회전";
		} else {                                 // 신호가 같다면 반시계방향 회전
			counter --;                         // 카운팅 용 숫자 1 감소
			currentDir ="좌회전";
		}
      
		Serial.print("회전방향: ");             
		Serial.print(currentDir);           //회전방향 출력
		Serial.print(" | Counter: ");
		Serial.println(counter);           // 회전 카운팅 출력
	}

	// 현재의 CLK상태를 저장
	lastStateCLK = currentStateCLK;

	// 버튼(스위치)이 눌렸는지 확인
	// int btnState = digitalRead(SW);

	// // 버튼(스위치)가 눌리면
	// if (btnState == LOW ) {
	// 	//버튼이 눌린지 50ms가 지났는지 확인, 즉 버튼이 한번 눌린 후 최소 50 ms는 지나야 버튼이 다시 눌린것으로 감지
	// 	if (millis() - lastButtonPress > 50) {  // 50ms 이상 지났다면 
	// 		Serial.println("버튼 눌림!");  //버튼 눌림 메시지 출력
	// 	}

	// 	// 마자막 버튼이 눌린 시간 저장
	// 	lastButtonPress = millis();
  // }
	delay(1);
  return counter;
}

void setup() 
{
  pinMode(RUN_DIR, OUTPUT);
  pinMode(RUN_PWM, OUTPUT);
  pinMode(RUN_BRK, OUTPUT);
  pinMode(STEER_DIR, OUTPUT);
  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_BRK, OUTPUT);
  pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
  // 스위치핀은 내부 풀업저항 사용
  Serial.begin(9600);

	// CLK핀의 현재 상태 확인
	lastStateCLK = digitalRead(CLK);	
}


void loop() {
  // put your main code here, to run repeatedly:
  // turnLeft(50);
  // turnLeft(50);
  turnRight(50);
  // turnLeft(50);
  // delay(1000);
  // brake();
  // delay(2000);
  // goBackward(255/3);
  // delay(1000);
  // brake();
  // turnRight(20);
  // encoder();
  // turnLeft(20);
  delay(2000000000);
  // encoder();
  // delay(500000000000);
  // delay(50);  
  // turnLeft(20);
  // delay(50);  
  // turnRight(20);
  // delay(50);
}
