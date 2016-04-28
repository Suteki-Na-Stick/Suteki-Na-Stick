
//
//  inverted_penulum_system_2.hpp
//  Created by 早坂彪流 on 2016/03/06.
//  Copyright © 2016年 takeru haysaka. All rights reserved.
//



//motermodeDefinitions
typedef enum {
  tw_phase8 = 0,
	tw_phaseF,
	one_tw_phase2p,
	one_tw_phaseF,
	sleep_2,
	}MoterMode;


	//data
#define MAX_SPEED 10000
#define MIN_SPEED 0.1
#define Index_gain_flag 5
#define MSTAP 120
#define Moter1 1
#define Moter2 2
#define Moter3 3
//pinDefinitions
#define Moter_1_clock 22
#define Moter_1_M1 23
#define Moter_1_M2 24
#define Moter_1_M3 25
#define Moter_1_syn 26
#define Moter_1_Refsleep 27
#define Moter_1_Sequential 28

#define Moter_2_clock 29
#define Moter_2_M1 30
#define Moter_2_M2 31
#define Moter_2_M3 32
#define Moter_2_syn 33
#define Moter_2_Refsleep 34
#define Moter_2_Sequential 35


#define Moter_3_clock 36
#define Moter_3_M1 37
#define Moter_3_M2 38
#define Moter_3_M3 39
#define Moter_3_syn 40
#define Moter_3_Refsleep 41
#define Moter_3_Sequential 42


//protypefunction
inline void Initmoterset(int, MoterMode);//モーター番号、モータmode
inline void dmpDataReady() ;


//#include <AccelStepper.h>
// 
//AccelStepper stepper1(1,5,6);
//int stepperspeed = 700;
//int movetopos;
//char cmd;
//float cyclelen = 400;//2*20 one cycle length
//float steps = 192; //360 度の手順 (3.75 度 = 360/3.75 * 2 (halfstep))
//#define switchpin 12;
// 
//void setup(){
//  pinMode(12,INPUT);
//  digitalWrite(12, HIGH);
// 
//  delay(1000);
//  pinMode(7, OUTPUT);
//  digitalWrite(7,HIGH); //for microstepping high output
//  pinMode(8, OUTPUT);
//  digitalWrite(8,LOW);  //for microstepping low output
// 
//  stepper1.setMaxSpeed(stepperspeed*100);
//  stepper1.setAcceleration(1000.0);
//  stepper1.move(1);
//  stepper1.setSpeed(stepperspeed);
// 
//  Serial.begin(9600);
//  Serial.println("Ready [s###]/[m###]/[d##(cm)]");
//}
 
//void loop(){
//  if (Serial.available()){
//    cmd = Serial.read();
//    movetopos = Serial.parseInt();
//    if (cmd == 's'){//速度
//      stepperspeed = movetopos;
//      Serial.print("speed ");
//      Serial.println(movetopos);
//    }else if (cmd == 'm'){//ステップ
//      Serial.print("move ");
//      Serial.println(movetopos);
//      stepper1.move(movetopos);
//      stepper1.setSpeed(stepperspeed);
//    }else if (cmd == 'd'){//cm
//      Serial.print("move ");
//      Serial.print(movetopos);
//      Serial.print("cm : ");
//      int movesteps = (int)((((float)movetopos*100)/cyclelen)*steps);
//      Serial.println(movesteps);
//      stepper1.move(movesteps);
//      stepper1.setSpeed(stepperspeed);
//    }
//  }
//  if (digitalRead(12) == LOW){
//    Serial.println("100cm feed");
//    int movesteps = (int)((((float)10000)/cyclelen)*steps);
//    stepper1.move(movesteps);
//    stepper1.setSpeed(stepperspeed);
//    while (stepper1.distanceToGo() != 0)
//      stepper1.runSpeedToPosition();
//    stepper1.move(0);
//  }
// 
//  stepper1.runSpeedToPosition();
//}
