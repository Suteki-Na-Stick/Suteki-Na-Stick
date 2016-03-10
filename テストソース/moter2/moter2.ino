
#include <pt.h>

#define Moter_1 11
#define Moter_2 10
#define Moter_3 9

unsigned long Moter_time_1 = 3,Moter_time_2 = 3,Moter_time_3 = 3;
unsigned long U_Moter_time_1 = 2,U_Moter_time_2 = 2,U_Moter_time_3 = 2;
static struct pt pt1, pt2,pt3;

static int protothread1(struct pt *pt) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
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
  while(1) {
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
  while(1) {
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
  pinMode(Moter_1, OUTPUT);
  pinMode(Moter_2, OUTPUT);
  pinMode(Moter_3, OUTPUT);
  PT_INIT(&pt1);
  PT_INIT(&pt2);
  PT_INIT(&pt3);
}
 
void loop() {
 protothread1(&pt1);
 protothread2(&pt2);
 protothread3(&pt3);
}
