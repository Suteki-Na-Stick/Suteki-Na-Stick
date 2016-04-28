
//
//  inverted_penulum_system_2.ino
//  Created by 早坂彪流 on 2016/03/06.
//  Copyright © 2016年 takeru haysaka. All rights reserved.
//
/*
  historys
  0314:AccelStepperにモーター駆動を置き換え
*/
#include <AccelStepper.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "inverted_penulum_system_2.hpp"


// MPU control/status vars
//MPU6050 mpu;
bool dmpReady = false;  // DMP の初期化が成功した場合は true に設定します。
uint8_t mpuIntStatus;   // MPU から実際の割り込みステータス バイトを保持します。
uint8_t devStatus;      // デバイス操作のたびに状態を返す(0 = success, !0 = error)
uint16_t packetSize;    // DMP パケット サイズ (デフォルトは 42 バイト)
uint16_t fifoCount;     // すべての FIFOのbyte の現在の値
uint8_t fifoBuffer[64]; // FIFO buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion
VectorInt16 aa;         // [x, y, z]            加速度センサー 計測値
VectorInt16 aaReal;     // [x, y, z]            重力加速度 計測値
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            重力ベクトル
float euler[3];         // [psi, theta, phi]    オイラー角
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t ax, ay, az;     //加速度センサー
int16_t gx, gy, gz;     //ジャイロセンサー

//motercontroll
AccelStepper stepper1(1, Moter_1_clock, Moter_1_Sequential);//引数最初の1が制御ドライバ使用、2,3引数がステップ/方向ピン番号
AccelStepper stepper2(1, Moter_2_clock, Moter_2_Sequential);
AccelStepper stepper3(1, Moter_3_clock, Moter_3_Sequential);
unsigned int Moter_time_1 = 2, Moter_time_2 = 2, Moter_time_3 = 2; //モーターの時間（立ち上がり）
// unsigned int MotorSpeedX = 0, MotorSpeedY = 0, YawSpeed = 0; //モーターの出力値

// long distance_x = 0, vx = 0, thofx = 0,   acx = 0, thx = 0, thvx = 0, xofs = 0;//距離,速度,初期状態保存,    加速度,角度,角速度,ofset
// long distance_y = 0, vy = 0, thofy = 0,   acy = 0, thy = 0, thvy = 0, yofs = 0;
// long Ks[Index_gain_flag] = {100, 200, 50, 10}; //gains

// int lastcont = 0; //初期保存の時のフラグ
// // ================================================================
// // ===               割り込み検出ルーチン                         ===
// // ================================================================

// volatile bool mpuInterrupt = false;     // MPU割り込み端子がハイ・レベルになったかどうかを示します
// inline void dmpDataReady() {
//   mpuInterrupt = true;
// }

// ================================================================
// ===               モーターに対するスレッド                      ===
// ================================================================

void moterset(int motersel, MoterMode mode) {
  if (motersel == Moter1) {
    switch (mode) {
      case tw_phase8:

        digitalWrite(Moter_1_M1, LOW);
        digitalWrite(Moter_1_M2, LOW);
        digitalWrite(Moter_1_M3, LOW);
        break;
      case tw_phaseF:

        digitalWrite(Moter_1_M1, HIGH);

        digitalWrite(Moter_1_M2, LOW);

        digitalWrite(Moter_1_M3, LOW);
        break;
      case one_tw_phase2p:

        digitalWrite(Moter_1_M1, LOW);

        digitalWrite(Moter_1_M2, HIGH);

        digitalWrite(Moter_1_M3, LOW);
        break;
      case one_tw_phaseF:
        digitalWrite(Moter_1_M1, HIGH);

        digitalWrite(Moter_1_M2, HIGH);

        digitalWrite(Moter_1_M3, LOW);
        break;
      case sleep_2:
        digitalWrite(Moter_1_M1, HIGH);
        digitalWrite(Moter_1_M2, HIGH);
        digitalWrite(Moter_1_M3, HIGH);
      default:
        // pinMode(Moter_1_Refsleep, OUTPUT);
        //digitalWrite(Moter_1_Refsleep, !digitalRead(Moter_1_Refsleep));
        break;
    }
    //最大値
    stepper1.setMaxSpeed(MAX_SPEED);
  } else if (motersel == Moter2) {
    switch (mode) {
      case tw_phase8:
        digitalWrite(Moter_2_M1, LOW);
        digitalWrite(Moter_2_M2, LOW);
        digitalWrite(Moter_2_M3, LOW);
        break;
      case tw_phaseF:
        digitalWrite(Moter_2_M1, HIGH);
        digitalWrite(Moter_2_M2, LOW);
        digitalWrite(Moter_2_M3, LOW);
        break;
      case one_tw_phase2p:
        digitalWrite(Moter_2_M1, LOW);
        digitalWrite(Moter_2_M2, HIGH);
        digitalWrite(Moter_2_M3, LOW);
        break;
      case one_tw_phaseF:
        digitalWrite(Moter_2_M1, HIGH);
        digitalWrite(Moter_2_M2, HIGH);
        digitalWrite(Moter_2_M3, LOW);
        break;
      case sleep_2:
        digitalWrite(Moter_2_M1, HIGH);
        digitalWrite(Moter_2_M2, HIGH);
        digitalWrite(Moter_2_M3, LOW);
        break;
      default:
        //  pinMode(Moter_1_Refsleep, OUTPUT);
        //digitalWrite(Moter_1_Refsleep, !digitalRead(Moter_1_Refsleep));
        break;
    }
    //最大値
    stepper2.setMaxSpeed(MAX_SPEED);
  } else if (motersel == Moter3) {
    switch (mode) {
      case tw_phase8:
        digitalWrite(Moter_3_M1, LOW);
        digitalWrite(Moter_3_M2, LOW);
        digitalWrite(Moter_3_M3, LOW);
        break;
      case tw_phaseF:
        digitalWrite(Moter_3_M1, HIGH);
        digitalWrite(Moter_3_M2, LOW);
        digitalWrite(Moter_3_M3, LOW);
        break;
      case one_tw_phase2p:
        digitalWrite(Moter_3_M1, LOW);
        digitalWrite(Moter_3_M2, HIGH);
        digitalWrite(Moter_3_M3, LOW);
        break;
      case one_tw_phaseF:
        digitalWrite(Moter_3_M1, HIGH);
        digitalWrite(Moter_3_M2, HIGH);
        digitalWrite(Moter_3_M3, LOW);
        break;
      case sleep_2:
        digitalWrite(Moter_3_M1, HIGH);
        digitalWrite(Moter_3_M2, HIGH);
        digitalWrite(Moter_3_M3, LOW);
        break;
      default:
        // pinMode(Moter_1_Refsleep, OUTPUT);
        //   digitalWrite(Moter_1_Refsleep, !digitalRead(Moter_1_Refsleep));
        break;
    }
    //最大値
    stepper3.setMaxSpeed(MAX_SPEED);
  }

  /*pinMode(Moter_1_Sequential, OUTPUT);
    digitalWrite(Moter_1_Sequential, digitalRead(Moter_1_Sequential));
    pinMode(Moter_1_syn, OUTPUT);
    digitalWrite(Moter_1_syn, !digitalRead(Moter_1_syn));
    pinMode(Moter_1_Refsleep, OUTPUT);
    digitalWrite(Moter_1_Refsleep, !digitalRead(Moter_1_Refsleep));*/
}

void setup() {

  pinMode(Moter_1_M1, OUTPUT);
  pinMode(Moter_1_M2, OUTPUT);
  pinMode(Moter_1_M3, OUTPUT);
  pinMode(Moter_2_M1, OUTPUT);
  pinMode(Moter_2_M2, OUTPUT);
  pinMode(Moter_2_M3, OUTPUT);
  pinMode(Moter_3_M1, OUTPUT);
  pinMode(Moter_3_M2, OUTPUT);
  pinMode(Moter_3_M3, OUTPUT);

  digitalWrite(Moter_1_M1, LOW);
  digitalWrite(Moter_1_M2, LOW);
  digitalWrite(Moter_1_M3, LOW);
  digitalWrite(Moter_2_M1, LOW);
  digitalWrite(Moter_2_M2, LOW);
  digitalWrite(Moter_2_M3, LOW);
  digitalWrite(Moter_3_M1, LOW);
  digitalWrite(Moter_3_M2, LOW);
  digitalWrite(Moter_3_M3, LOW);
stepper1.setMaxSpeed(MAX_SPEED);
stepper2.setMaxSpeed(MAX_SPEED);
  //moterset
 // moterset(Moter1, tw_phase8);
 // moterset(Moter2, tw_phase8);
  //    moterset(Moter3, tw_phase8);
}

void loop() {


  //モーターへ出力
  //    //stepper1.move(Moter_time_1);
  //    stepper1.setSpeed(Moter_time_1);
  //    //stepper2.move(movetopos);
  //    stepper2.setSpeed(Moter_time_2);
  //    //stepper3.move(movetopos);
  //    stepper3.setSpeed(Moter_time_3);
  //モーターへ出力
  //stepper1.move(Moter_time_1);
  stepper1.setSpeed(50);
  //stepper2.move(movetopos);
  stepper2.setSpeed(50);
  //stepper3.move(movetopos);
  stepper3.setSpeed(50);
  stepper1.runSpeed();
  stepper2.runSpeed();
  // stepper3.runSpeed();

}


