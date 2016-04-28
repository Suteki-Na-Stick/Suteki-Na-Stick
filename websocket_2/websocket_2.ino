#include <Wire.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <WebSocketsServer.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Hash.h>


#define SENSOR_ADRS 0x60                // MPL115A2のI2Cアドレス
#define AVE_NUM     20                    // 圧力・温度のＡ／Ｄ変換値を平均化する回数
#define H_CORRECT   80                  // 自宅でのセンサと実際の高度差補正値(My自宅の標高は100m)
#define ESPSDA 14
#define ESPSCL 12
float a0 , b1 , b2 , c12 ;                // 係数のデータを保存する変数
unsigned long Press , Temp ;              // 圧力および温度の変換値を保存する変数


//天気予報
#define ALTITUDE 301.0 // Altitude of my home
#define MILS_IN_MIN  60000

#define CHILD_ID_TEMP        0
#define CHILD_ID_BARO        1

int minuteCount = 0;
double pressureSamples[9][6];
double pressureAvg[9];
double dP_dt;

const char *weather[] = {
  "stable", "sunny", "cloudy", "unstable", "thunderstorm", "unknown"
};

int forecast = 5;

unsigned long startTime;

//DHT sensor
#define DHTTYPE DHT22
#define DHTPIN 12
DHT dht(DHTPIN, DHTTYPE);
//websockets
WebSocketsServer webSocket = WebSocketsServer(81);
const char* ssid     = "takeru hayasaka";
const char* password = "i7n6mzy2w8ctj";
//const char* ssid     = "mems";
//const char* password = "4f44741cc1d80";
//const char* ssid     = "Xperia Z3_bddb";
//const char* password = "fjjrkdxl";
//const char* ssid     = "L02F_B3884E14_A";
//const char* password = "F2D3110F";
//char path[] = "/";
//char host[] = "websocket.org";
//const char* ssid     = "GlobalAreaNetwork";
//const char* password = "LocalAreaNetwork";


int calculateForecast(double pressure) {
  //From 0 to 5 min.
  if (minuteCount <= 5) {
    pressureSamples[0][minuteCount] = pressure;
  }
  //From 30 to 35 min.
  else if ((minuteCount >= 30) && (minuteCount <= 35)) {
    pressureSamples[1][minuteCount - 30] = pressure;
  }
  //From 60 to 65 min.
  else if ((minuteCount >= 60) && (minuteCount <= 65)) {
    pressureSamples[2][minuteCount - 60] = pressure;
  }
  //From 90 to 95 min.
  else if ((minuteCount >= 90) && (minuteCount <= 95)) {
    pressureSamples[3][minuteCount - 90] = pressure;
  }
  //From 120 to 125 min.
  else if ((minuteCount >= 120) && (minuteCount <= 125)) {
    pressureSamples[4][minuteCount - 120] = pressure;
  }
  //From 150 to 155 min.
  else if ((minuteCount >= 150) && (minuteCount <= 155)) {
    pressureSamples[5][minuteCount - 150] = pressure;
  }
  //From 180 to 185 min.
  else if ((minuteCount >= 180) && (minuteCount <= 185)) {
    pressureSamples[6][minuteCount - 180] = pressure;
  }
  //From 210 to 215 min.
  else if ((minuteCount >= 210) && (minuteCount <= 215)) {
    pressureSamples[7][minuteCount - 210] = pressure;
  }
  //From 240 to 245 min.
  else if ((minuteCount >= 240) && (minuteCount <= 245)) {
    pressureSamples[8][minuteCount - 240] = pressure;
  }


  if (minuteCount == 5) {
    // Avg pressure in first 5 min, value averaged from 0 to 5 min.
    pressureAvg[0] = ((pressureSamples[0][0] + pressureSamples[0][1]
                       + pressureSamples[0][2] + pressureSamples[0][3]
                       + pressureSamples[0][4] + pressureSamples[0][5]) / 6);
  }
  else if (minuteCount == 35) {
    // Avg pressure in 30 min, value averaged from 0 to 5 min.
    pressureAvg[1] = ((pressureSamples[1][0] + pressureSamples[1][1]
                       + pressureSamples[1][2] + pressureSamples[1][3]
                       + pressureSamples[1][4] + pressureSamples[1][5]) / 6);
    float change = (pressureAvg[1] - pressureAvg[0]);
    dP_dt = change / 5;
  }
  else if (minuteCount == 65) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[2] = ((pressureSamples[2][0] + pressureSamples[2][1]
                       + pressureSamples[2][2] + pressureSamples[2][3]
                       + pressureSamples[2][4] + pressureSamples[2][5]) / 6);
    float change = (pressureAvg[2] - pressureAvg[0]);
    dP_dt = change / 10;
  }
  else if (minuteCount == 95) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[3] = ((pressureSamples[3][0] + pressureSamples[3][1]
                       + pressureSamples[3][2] + pressureSamples[3][3]
                       + pressureSamples[3][4] + pressureSamples[3][5]) / 6);
    float change = (pressureAvg[3] - pressureAvg[0]);
    dP_dt = change / 15;
  }
  else if (minuteCount == 125) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[4] = ((pressureSamples[4][0] + pressureSamples[4][1]
                       + pressureSamples[4][2] + pressureSamples[4][3]
                       + pressureSamples[4][4] + pressureSamples[4][5]) / 6);
    float change = (pressureAvg[4] - pressureAvg[0]);
    dP_dt = change / 20;
  }
  else if (minuteCount == 155) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[5] = ((pressureSamples[5][0] + pressureSamples[5][1]
                       + pressureSamples[5][2] + pressureSamples[5][3]
                       + pressureSamples[5][4] + pressureSamples[5][5]) / 6);
    float change = (pressureAvg[5] - pressureAvg[0]);
    dP_dt = change / 25;
  }
  else if (minuteCount == 185) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[6] = ((pressureSamples[6][0] + pressureSamples[6][1]
                       + pressureSamples[6][2] + pressureSamples[6][3]
                       + pressureSamples[6][4] + pressureSamples[6][5]) / 6);
    float change = (pressureAvg[6] - pressureAvg[0]);
    dP_dt = change / 30;
  }
  else if (minuteCount == 215) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[7] = ((pressureSamples[7][0] + pressureSamples[7][1]
                       + pressureSamples[7][2] + pressureSamples[7][3]
                       + pressureSamples[7][4] + pressureSamples[7][5]) / 6);
    float change = (pressureAvg[7] - pressureAvg[0]);
    dP_dt = change / 35;
  }
  else if (minuteCount == 245) {
    // Avg pressure at end of the hour, value averaged from 0 to 5 min.
    pressureAvg[8] = ((pressureSamples[8][0] + pressureSamples[8][1]
                       + pressureSamples[8][2] + pressureSamples[8][3]
                       + pressureSamples[8][4] + pressureSamples[8][5]) / 6);
    float change = (pressureAvg[8] - pressureAvg[0]);
    dP_dt = change / 40; // note this is for t = 4 hour

    minuteCount -= 30;
    pressureAvg[0] = pressureAvg[1];
    pressureAvg[1] = pressureAvg[2];
    pressureAvg[2] = pressureAvg[3];
    pressureAvg[3] = pressureAvg[4];
    pressureAvg[4] = pressureAvg[5];
    pressureAvg[5] = pressureAvg[6];
    pressureAvg[6] = pressureAvg[7];
    pressureAvg[7] = pressureAvg[8];
  }

  minuteCount++;

  if (minuteCount < 36) //if time is less than 35 min
    return 5; // Unknown, more time needed
  else if (dP_dt < (-0.25))
    return 4; // Quickly falling LP, Thunderstorm, not stable
  else if (dP_dt > 0.25)
    return 3; // Quickly rising HP, not stable weather
  else if ((dP_dt > (-0.25)) && (dP_dt < (-0.05)))
    return 2; // Slowly falling Low Pressure System, stable rainy weather
  else if ((dP_dt > 0.05) && (dP_dt < 0.25))
    return 1; // Slowly rising HP stable good weather
  else if ((dP_dt > (-0.05)) && (dP_dt < 0.05))
    return 0; // Stable weather
  else
    return 5; // Unknown
}


//メモリーマップから係数を読み出す処理
int CoefficientRead()
{
  int ans ;
  unsigned int h , l ;

  Wire.beginTransmission(SENSOR_ADRS) ;        // 通信の開始
  Wire.write(0x04) ;                           // 係数の読出しコマンド発行
  ans = Wire.endTransmission() ;               // データの送信と通信の終了
  if (ans == 0) {
    ans = Wire.requestFrom(SENSOR_ADRS, 8) ; // 係数データの受信を行う
    if (ans == 8) {
      // ａ０の係数を得る
      h = Wire.read() ;
      l = Wire.read() ;
      a0 = (h << 5) + (l >> 3) + (l & 0x07) / 8.0 ;
      // ｂ１の係数を得る
      h = Wire.read() ;
      l = Wire.read() ;
      b1 = ( ( ( (h & 0x1F) * 0x100 ) + l ) / 8192.0 ) - 3 ;
      // ｂ２の係数を得る
      h = Wire.read() ;
      l = Wire.read() ;
      b2 = ( ( ( ( h - 0x80) << 8 ) + l ) / 16384.0 ) - 2 ;
      // Ｃ１２の係数を得る
      h = Wire.read() ;
      l = Wire.read() ;
      c12 = ( ( ( h * 0x100 ) + l ) / 16777216.0 )  ;
      ans = 0 ;
    } else ans = 5 ;
  }

  return ans ;
}
// メモリーマップから圧力および温度のＡ／Ｄ変換値を読み出す処理
int PressureRead()
{
  int ans ;
  unsigned int h , l ;

  // 圧力および温度の変換を開始させる処理
  Wire.beginTransmission(SENSOR_ADRS) ;        // 通信の開始
  Wire.write(0x12) ;                           // 圧力・温度の変換開始コマンド発行
  Wire.write(0x01) ;
  ans = Wire.endTransmission() ;               // データの送信と通信の終了
  if (ans != 0) return ans ;
  delay(3) ;                                   // 変換完了まで３ｍｓ待つ

  // Ａ／Ｄ変換値を得る処理
  Wire.beginTransmission(SENSOR_ADRS) ;        // 通信の開始
  Wire.write(0x00) ;                           // 圧力のHighバイトから読込むコマンド発行
  ans = Wire.endTransmission() ;               // データの送信と通信の終了
  if (ans == 0) {
    ans = Wire.requestFrom(SENSOR_ADRS, 4) ; // Ａ／Ｄ変換値データの受信を行う
    if (ans == 4) {
      // 圧力のＡ／Ｄ変換値を得る
      h = Wire.read() ;
      l = Wire.read() ;
      Press = ( ( h * 256 ) + l ) / 64 ;
      // 温度のＡ／Ｄ変換値を得る
      h = Wire.read() ;
      l = Wire.read() ;
      Temp = ( ( h * 256 ) + l ) / 64 ;
      ans = 0 ;
    } else ans = 5 ;
  }

  return ans ;
}
// 気圧値(hPa)を計算する処理
float PressureCalc()
{
  float ret , f ;

  f = a0 + ( b1 + c12 * Temp ) * Press + b2 * Temp ;
  ret = f * ( 650.0 / 1023.0 ) + 500.0 ;
  return ret ;
}
// 気圧値(hPa)から高度を計算する処理
float AltitudeCalc(float pressure, int Difference)
{
  float h ;

  h = 44330.8 * (1.0 - pow( (pressure / 1013.25) ,  0.190263 )) ;
  h = h + Difference ;
  return h ;
}
void forecasting() {
  //jsoncreate
  int i ;
  float ans ;
  unsigned long p , tp ;

  p = tp = 0 ;
  for (i = 0 ; i < AVE_NUM ; i++) {    // ２０回読み込んで平均化する
    PressureRead() ;                // メモリーマップから圧力および温度のＡ／Ｄ変換値を読み出す
    p = p + Press ;
    tp = tp + Temp ;
  }
  Press = p / AVE_NUM ;
  Temp  = tp / AVE_NUM ;

  ans = PressureCalc() ;               // 気圧値の計算を行う
  Serial.print(ans) ;                  // 気圧値の表示を行う
  Serial.print(" hPa    ") ;
  ans = AltitudeCalc(ans, H_CORRECT) ; // 高度の計算を行う
  Serial.print(ans) ;                  // 高度の表示を行う
  Serial.println(" m") ;

  static int lastSendTempInt;
  int temp_round = round(Temp);

  if (temp_round != lastSendTempInt)
  {
    lastSendTempInt = temp_round;
  }

  static int lastSendPresInt;
  int pres_round = round(Press);

  if (pres_round != lastSendPresInt)
  {
    lastSendPresInt = pres_round;
  }

  forecast = calculateForecast(Press);
  static int lastSendForeInt = -1;
  if (forecast != lastSendForeInt)
  {
    lastSendForeInt = forecast;
  }
  Serial.print("weather\t");
  Serial.println(weather[forecast]);
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {

  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("DISCONNECTED");
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.println(ip); Serial.println("CONNECTED");
      }
      break;
    case WStype_TEXT:
      {

        String text = String((char *) &payload[0]);
        if (text == "LED") {

          Serial.println("led just lit");

          webSocket.sendTXT(num, "led just lit", String("led just lit").length());
        }

        if (text == "RESET") {
          Serial.println("reset");
        }

        if (text == "Temp") {
          // Reading temperature or humidity takes about 250 milliseconds!
          // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
          float h = dht.readHumidity();
          // Read temperature as Celsius (the default)
          float t = dht.readTemperature();
          // Read temperature as Fahrenheit (isFahrenheit = true)
          float f = dht.readTemperature(true);

          // Check if any reads failed and exit early (to try again).
          if (isnan(h) || isnan(t) || isnan(f)) {
            Serial.println("Failed to read from DHT sensor!");
            webSocket.sendTXT(num, "Failed to read from DHT sensor!", String("lailed to read from DHT sensor!").length());
          } else {
            // Compute heat index in Fahrenheit (the default)
            float hif = dht.computeHeatIndex(f, h);
            // Compute heat index in Celsius (isFahreheit = false)
            float hic = dht.computeHeatIndex(t, h, false);

            String SEND = String(h);
            SEND += " ";
            SEND += String(t);
            webSocket.sendTXT(num, SEND);
            Serial.print("Temperature: ");//温度
            Serial.print(t);
            Serial.print(" *C ");
            Serial.print("Humidity: ");//湿度
            Serial.print(h);
            Serial.print(" %\t");

            Serial.print(f);
            Serial.print(" *F\t");

            Serial.print("Heat index: ");//体感温度&湿度
            Serial.print(hic);
            Serial.print(" *C ");
            Serial.print(hif);
            Serial.println(" *F");
          }
        }
        if (text == "Hum") {}
        if (text == "Temp_H") {}
        if (text == "Hum_H") {}
        if (text == "hPa") {

        }
        if (text == "ALLUPDATE") {
          // Reading temperature or humidity takes about 250 milliseconds!
          // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
          float h = dht.readHumidity();
          // Read temperature as Celsius (the default)
          float t = dht.readTemperature();
          // Read temperature as Fahrenheit (isFahrenheit = true)
          float f = dht.readTemperature(true);

          // Check if any reads failed and exit early (to try again).
          if (isnan(h) || isnan(t) || isnan(f)) {
            Serial.println("Failed to read from DHT sensor!");
            webSocket.sendTXT(num, "Failed to read from DHT sensor!", String("lailed to read from DHT sensor!").length());
          } else {
            float hif = dht.computeHeatIndex(f, h);// Compute heat index in Fahrenheit (the default)
            float hic = dht.computeHeatIndex(t, h, false);// Compute heat index in Celsius (isFahreheit = false)

            //jsoncreate
            int i ;
            float ans ;
            unsigned long p , tp ;

            p = tp = 0 ;
            for (i = 0 ; i < AVE_NUM ; i++) {    // ２０回読み込んで平均化する
              PressureRead() ;                // メモリーマップから圧力および温度のＡ／Ｄ変換値を読み出す
              p = p + Press ;
              tp = tp + Temp ;
            }
            Press = p / AVE_NUM ;
            Temp  = tp / AVE_NUM ;

            ans = PressureCalc() ;               // 気圧値の計算を行う
            Serial.print(ans) ;                  // 気圧値の表示を行う
            Serial.print(" hPa    ") ;
            ans = AltitudeCalc(ans, H_CORRECT) ; // 高度の計算を行う
            Serial.print(ans) ;                  // 高度の表示を行う
            Serial.println(" m") ;
            
            //String senddata = String("{\"Temp\": " + String(t) + ", \"Hum\": " + String(h) + " , \"Temp_H\": " + String(hic) + ", \"hPa\": " + String(ans)"}");

            String jsonString = "{\"Temp\":\"";
            jsonString += String(t);
            jsonString += "\",\"Hum\":\"";
            jsonString += String(h);
            jsonString += "\",\"Temp_H\":\"";
            jsonString += String(hic);
            jsonString += "\",\"hPa\":\"";
            jsonString += String(ans);
            jsonString += "\",\"weather\":\"";
            jsonString += String(weather[forecast]);
            jsonString += "\"}";
            //webSocket.sendTXT(num, SEND);
            //forecasting();
            webSocket.sendTXT(num, jsonString);//send
            Serial.print("Temperature: ");//温度
            Serial.print(t);
            Serial.print(" *C ");
            Serial.print("Humidity: ");//湿度
            Serial.print(h);
            Serial.print(" %\t");
            Serial.print(f);
            Serial.print(" *F\t");

            Serial.print("Heat index: ");//体感温度&湿度
            Serial.print(hic);
            Serial.print(" *C ");
            Serial.print(hif);
            Serial.println(" *F");

            Serial.print("weather\t");
            Serial.print(weather[forecast]);

          }
        }
      }
      //      webSocket.broadcastTXT(payload, lenght);

      break;

    case WStype_BIN:

      hexdump(payload, lenght);

      // echo data back to browser
      webSocket.sendBIN(num, payload, lenght);
      break;
  }

}
boolean IsTimeout()
{
  unsigned long now = millis();
  if (startTime <= now)
  {
    if ( (unsigned long)(now - startTime )  < MILS_IN_MIN ) 
      return false;
  }
  else
  {
    if ( (unsigned long)(startTime - now) < MILS_IN_MIN ) 
      return false;
  }

  return true;
}


void setup() {

  Serial.begin(115200);
  Wire.begin();
  Serial.println("DHT22 test!");
  dht.begin();
  CoefficientRead() ;            // MPL115A2のメモリーマップから係数を読出して置く
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  startTime =  -1;
  for (int count =0;count<=40;count++){
    forecasting();
    }
}

void loop() {

  webSocket.loop();
  if (IsTimeout()){
     forecasting();
    }
  startTime = millis(); 
}


