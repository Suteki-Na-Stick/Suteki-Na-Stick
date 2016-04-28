

/*TakeruHayasaka,20160302*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <pt.h>

//pinの番号
#define Moter_1 22
#define Moter_2 11
#define Moter_3 12
#define Index_gain_flag 5

// MPU control/status vars
MPU6050 mpu;
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
static struct pt pt1, pt2, pt3; //スレッドの変数
unsigned int Moter_time_1 = 2, Moter_time_2 = 2, Moter_time_3 = 2; //モーターの時間（立ち上がり）
unsigned long U_Moter_time_1 = 2, U_Moter_time_2 = 2, U_Moter_time_3 = 2; //・・（たち下がり）
unsigned int MotorSpeedX = 0, MotorSpeedY = 0, YawSpeed = 0; //モーターの出力値

long x = 0, vx = 0, thofx = 0, acx = 0, thx = 0, thvx = 0, xofs = 0,        y = 0, vy = 0, thofy = 0, acy = 0, thy = 0, thvy = 0, yofs = 0; //?,?,初期状態保存,決定したデータ,現在,?,ofset
int lastcont = 0; //初期保存の時のフラグ
long Ks[Index_gain_flag] = {100, 200, 50, 10}; //gains
// ================================================================
// ===               割り込み検出ルーチン                         ===
// ================================================================

volatile bool mpuInterrupt = false;     // MPU割り込み端子がハイ・レベルになったかどうかを示します
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               モーターに対するスレッド                      ===
// ================================================================
static int protothread1(struct pt *pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    //up
    PT_WAIT_UNTIL(pt, millis() - timestamp > Moter_time_1);
    timestamp = millis();
    digitalWrite(Moter_1, !digitalRead(Moter_1));
    //down
    PT_WAIT_UNTIL(pt, millis() - timestamp > U_Moter_time_1);
    timestamp = millis();
    digitalWrite(Moter_1, !digitalRead(Moter_1));

  }
  PT_END(pt);
}

static int protothread2(struct pt *pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    //up
    PT_WAIT_UNTIL(pt, millis() - timestamp > Moter_time_2);
    timestamp = millis();
    digitalWrite(Moter_2, !digitalRead(Moter_2));
    //down
    PT_WAIT_UNTIL(pt, millis() - timestamp > U_Moter_time_2);
    timestamp = millis();
    digitalWrite(Moter_2, !digitalRead(Moter_2));
  }
  PT_END(pt);
}
static int protothread3(struct pt *pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    //up
    PT_WAIT_UNTIL(pt, millis() - timestamp > Moter_time_3);
    timestamp = millis();
    digitalWrite(Moter_3, !digitalRead(Moter_3));
    //down
    PT_WAIT_UNTIL(pt, millis() - timestamp > U_Moter_time_3);
    timestamp = millis();
    digitalWrite(Moter_3, !digitalRead(Moter_3));
  }
  PT_END(pt);
}

void setup() {
  //i2c初期化
  Wire.begin();

  // シリアル通信を初期化します
  Serial.begin(115200);
  while (!Serial); // 立ち上がりまで待つ

  // デバイスの初期化
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // 接続の確認
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // 準備完了まで待機
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // DMPの構成初期化
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // デバイスとのコネクションが成立しているか(0 = success, !0 = error)
  if (devStatus == 0) {
    // DMPを有効に
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    //Arduino の割り込みの検出を有効にします。
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // DMPが有効かというフラグ
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    //  DMP パケット サイズを取得します。
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  //スレッドの初期化とピンのモード決め
  pinMode(Moter_1, OUTPUT);
  pinMode(Moter_2, OUTPUT);
  pinMode(Moter_3, OUTPUT);
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
}

void loop() {

  // 失敗したらさよなら
  if (!dmpReady) return ;

  // −１wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // 割り込みフラグをリセットし、INT_STATUS バイトを取得
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // 現在のFIFOの量を取得
  fifoCount = mpu.getFIFOCount();

  //  オーバーフローのチェック
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // オーバーフローリセット
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // それ以外の場合、DMP データ準備割り込み (頻繁に発生する必要あり) を確認します。
  } else if (mpuIntStatus & 0x02) {
    // 正確な利用可能なデータ長を待つ
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // FIFO からパケットを読む
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    //  1 パケットでもある場合ここで FIFO のカウントを追跡する。
    // (これにより、割り込みを待つことがなくすぐにできる)
    fifoCount -= packetSize;

    // オイラー角を度単位で表示
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
    //生データ取得
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    //y軸なのでX方向の角度系が取れてX軸なのでY方向の角度系が取れる
    thx = -(ypr[1] * 180 / M_PI); /* 傾向 X 軸プラス = 正  */
    thy = -(ypr[2] * 180 / M_PI); /* 傾向 Y 軸プラス = 正 */
    thvx = -(gy); /* angular vel. */
    thvy = -(gx); /* angular vel. */
    /*以下倒立振子制御*/
    if (lastcont == 0)
    {
      thofx = thx;  thofy = thy; /* 初期角度状態保存*/
      xofs = x; yofs = y;
      lastcont = 1;
    } else {
      
      thx -= thofx;  thy -= thofy;
      acx =   (((signed long)Ks[0] * thx ) >> 16) +
              (((signed long)Ks[1] * thvx) >> 16) +
              (((signed long)Ks[2] * x   ) >> 16) +
              (((signed long)Ks[3] * vx  ) >> 16);
      acy =
        (((signed long)Ks[0] * thy ) >> 16) +
        (((signed long)Ks[1] * thvy) >> 16) +
        (((signed long)Ks[2] * y   ) >> 16) +
        (((signed long)Ks[3] * vy  ) >> 16); 
    }

    /* filter */
          vx+=acx;  vy+=acy;
    /*x +=(vx>>4);   y +=(vy>>4);*/
       MotorSpeedX=(vx>>4);
       MotorSpeedY=(vy>>4);
       YawSpeed = 0;

      /* 指令速度を実際のモータの回転数に変換します。
         motor 1,2,3 , √3/2~=222/256->√3/2*256＝221.7
         n/256を>>8としてるので（割り算が負担なので）*/
      //TODO::変換したやつをMoter_time_n等を半分にして実際に変換する。
      Moter_time_1 = -(MotorSpeedX   ) +                0             + YawSpeed;
      Moter_time_2 =  (MotorSpeedX >> 1) - (((long)MotorSpeedY * 222) >> 8) + YawSpeed;
      Moter_time_3 =  (MotorSpeedX >> 1) + (((long)MotorSpeedY * 222) >> 8) + YawSpeed;

 Serial.print(Moter_time_1); Serial.print("\t");
  Serial.print(Moter_time_2); Serial.print("\t");
   Serial.print(Moter_time_3); Serial.println("\t");
      //モーターへ出力
      protothread1(&pt1);
      protothread2(&pt2);
      protothread3(&pt3);
  }
}
